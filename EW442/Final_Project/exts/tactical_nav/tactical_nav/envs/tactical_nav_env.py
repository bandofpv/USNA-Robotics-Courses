from isaaclab.envs import ManagerBasedRLEnv
from .tactical_nav_env_cfg import TacticalNavEnvCfg
import torch
from .raycast_utils import check_line_of_sight
import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.markers.config import SPHERE_MARKER_CFG

class TacticalNavEnv(ManagerBasedRLEnv):
    """Custom environment for tactical navigation with line-of-sight exposure."""
    
    cfg: TacticalNavEnvCfg
    
    def __init__(self, cfg: TacticalNavEnvCfg, render_mode: str | None = None, **kwargs):
        # Initialize target/goal positions for the robots BEFORE super().__init__
        # as the observation manager needs these to initialize its buffer shapes
        self.goal_positions = torch.zeros((cfg.scene.num_envs, 3), device=cfg.sim.device)
        self.goal_positions[:, 0] = 3.0   # Keep goal within local 8m terrain patch
        self.goal_positions[:, 2] = 0.5   # Lift 0.5m above Z=0 so it isn't underground
        
        # Mask of exposed robots
        self.is_exposed_mask = torch.zeros((cfg.scene.num_envs, cfg.threat_positions.shape[0]), dtype=torch.bool, device=cfg.sim.device)
        self.is_blocked_mask = torch.zeros((cfg.scene.num_envs, cfg.threat_positions.shape[0]), dtype=torch.bool, device=cfg.sim.device)
        self.prev_high_level_cmd = torch.zeros((cfg.scene.num_envs, 3), device=cfg.sim.device)
        self._episode_los_exposed_sum = torch.zeros((cfg.scene.num_envs,), device=cfg.sim.device)
        self._episode_los_blocked_sum = torch.zeros((cfg.scene.num_envs,), device=cfg.sim.device)
        self._episode_los_step_count = torch.zeros((cfg.scene.num_envs,), device=cfg.sim.device)
        
        super().__init__(cfg, render_mode, **kwargs)
        
        # Set up visualizers AFTER super().__init__
        if self.render_mode is not None:
            # Visualizer for Goals (Green Sphere)
            goal_cfg = SPHERE_MARKER_CFG.copy()
            goal_cfg.prim_path = "/Visuals/Goals"
            goal_cfg.markers["sphere"].radius = 0.5
            goal_cfg.markers["sphere"].visual_material.diffuse_color = (0.0, 1.0, 0.0)
            self.goal_visualizer = VisualizationMarkers(goal_cfg)
            self.goal_visualizer.set_visibility(True)

            # Visualizer for Threats (Red Sphere)
            threat_cfg = SPHERE_MARKER_CFG.copy()
            threat_cfg.prim_path = "/Visuals/Threats"
            threat_cfg.markers["sphere"].radius = 0.7
            threat_cfg.markers["sphere"].visual_material.diffuse_color = (1.0, 0.0, 0.0)
            self.threat_visualizer = VisualizationMarkers(threat_cfg)
            self.threat_visualizer.set_visibility(True)

            los_cfg = VisualizationMarkersCfg(
                prim_path="/Visuals/LOS",
                markers={
                    "blocked": sim_utils.CylinderCfg(
                        radius=0.03,
                        height=1.0,
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
                    ),
                    "exposed": sim_utils.CylinderCfg(
                        radius=0.03,
                        height=1.0,
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                    ),
                },
            )
            self.los_visualizer = VisualizationMarkers(los_cfg)
            self.los_visualizer.set_visibility(True)
        
    def step(self, action: torch.Tensor):
        """Step the environment forward and update debug visualizations."""
        # Stabilize command velocities to keep the robot in safe planar motion.
        filtered_action = action.clone()
        filtered_action[:, 0] = torch.clamp(filtered_action[:, 0], -1.1, 1.5)   # vx
        filtered_action[:, 1] = torch.clamp(filtered_action[:, 1], -0.35, 0.35) # vy
        filtered_action[:, 2] = torch.clamp(filtered_action[:, 2], -1.0, 1.0)   # wz
        filtered_action = 0.5 * self.prev_high_level_cmd + 0.5 * filtered_action
        self.prev_high_level_cmd[:] = filtered_action

        step_out = super().step(filtered_action)
        if len(step_out) == 5:
            obs, rewards, terminated, truncated, extras = step_out
            dones = torch.logical_or(terminated, truncated)
        else:
            obs, rewards, dones, extras = step_out
            terminated, truncated = dones, torch.zeros_like(dones)

        # Update line-of-sight mask after physics.
        robot_positions = self.scene["robot"].data.root_pos_w[:, :3]
        threat_positions_w = self.scene.env_origins.unsqueeze(1) + self.cfg.threat_positions.to(self.device).float().unsqueeze(0)
        self.is_exposed_mask, self.is_blocked_mask = check_line_of_sight(
            self,
            robot_positions=robot_positions,
            threat_positions_w=threat_positions_w,
            max_distance=20.0,
        )

        # Accumulate per-environment LOS stats for episode-level logging.
        los_exposed_env = self.is_exposed_mask.float().mean(dim=1)
        los_blocked_env = self.is_blocked_mask.float().mean(dim=1)
        self._episode_los_exposed_sum += los_exposed_env
        self._episode_los_blocked_sum += los_blocked_env
        self._episode_los_step_count += 1.0

        if not isinstance(extras, dict):
            extras = {}

        log_dict = extras.get("log", {})
        if not isinstance(log_dict, dict):
            log_dict = {}

        # Step-level means are useful for quick sanity checks while running.
        log_dict["LOS/exposed_step_mean"] = los_exposed_env.mean()
        log_dict["LOS/blocked_step_mean"] = los_blocked_env.mean()

        if torch.any(dones):
            done_ids = torch.nonzero(dones, as_tuple=False).squeeze(-1)
            done_counts = self._episode_los_step_count[done_ids].clamp(min=1.0)
            exposed_episode = self._episode_los_exposed_sum[done_ids] / done_counts
            blocked_episode = self._episode_los_blocked_sum[done_ids] / done_counts

            # Episode-level LOS metrics for training logs.
            log_dict["Episode_LOS/exposed_ratio"] = exposed_episode.mean()
            log_dict["Episode_LOS/blocked_ratio"] = blocked_episode.mean()
            log_dict["Episode_LOS/done_count"] = torch.tensor(float(done_ids.numel()), device=self.device)

            # Reset accumulators for environments whose episodes just ended.
            self._episode_los_exposed_sum[done_ids] = 0.0
            self._episode_los_blocked_sum[done_ids] = 0.0
            self._episode_los_step_count[done_ids] = 0.0

        extras["log"] = log_dict

        if self.render_mode is not None:
            self._draw_debug_vis()

        if len(step_out) == 5:
            return obs, rewards, terminated, truncated, extras
        return obs, rewards, dones, extras

    def _draw_debug_vis(self):
        """Draws visual debugging lines for LoS in the simulator."""
        if hasattr(self, "goal_visualizer"):
            # Goals are relative to env_origin
            goal_pos_w = self.goal_positions + self.scene.env_origins
            self.goal_visualizer.visualize(translations=goal_pos_w)
            
        if hasattr(self, "threat_visualizer"):
            # Threats in the config were created generally, let's scatter them relative to each origin 
            # so each env has its own threat
            threat_positions = self.cfg.threat_positions.to(self.device).float()
            # Expand to match envs
            threat_pos_w = self.scene.env_origins.unsqueeze(1) + threat_positions.unsqueeze(0)
            threat_pos_w = threat_pos_w.view(-1, 3) 
            self.threat_visualizer.visualize(translations=threat_pos_w)

        if hasattr(self, "los_visualizer"):
            robot_pos_w = self.scene["robot"].data.root_pos_w[:, :3]
            threat_pos_w = self.scene.env_origins.unsqueeze(1) + self.cfg.threat_positions.to(self.device).float().unsqueeze(0)
            robot_pos_pairs = robot_pos_w.unsqueeze(1).expand(-1, threat_pos_w.shape[1], -1)

            start = threat_pos_w.reshape(-1, 3)
            end = robot_pos_pairs.reshape(-1, 3)
            seg = end - start
            seg_len = torch.norm(seg, dim=-1).clamp(min=1e-4)
            seg_dir = seg / seg_len.unsqueeze(-1)
            mid = 0.5 * (start + end)

            z_axis = torch.zeros_like(seg_dir)
            z_axis[:, 2] = 1.0
            axis = torch.cross(z_axis, seg_dir, dim=-1)
            axis_norm = torch.norm(axis, dim=-1, keepdim=True)
            axis = axis / (axis_norm + 1e-6)
            angle = torch.acos(torch.clamp(torch.sum(z_axis * seg_dir, dim=-1), -1.0, 1.0))
            quat = math_utils.quat_from_angle_axis(angle, axis)

            anti_parallel = (axis_norm.squeeze(-1) < 1e-6) & (seg_dir[:, 2] < 0.0)
            if torch.any(anti_parallel):
                quat[anti_parallel] = torch.tensor([0.0, 1.0, 0.0, 0.0], device=self.device)

            scales = torch.ones((seg_len.shape[0], 3), device=self.device)
            scales[:, 2] = seg_len

            marker_indices = self.is_exposed_mask.reshape(-1).to(torch.int32)
            self.los_visualizer.visualize(
                translations=mid,
                orientations=quat,
                scales=scales,
                marker_indices=marker_indices,
            )

    def _reset_idx(self, env_ids):
        """Reset specific environments."""
        super()._reset_idx(env_ids)
        # Randomize goal inside reachable area of the local terrain patch.
        num_ids = len(env_ids)
        self.goal_positions[env_ids, 0] = torch.rand(num_ids, device=self.device) * 1.5 + 2.0   # [2.0, 3.5]
        self.goal_positions[env_ids, 1] = torch.rand(num_ids, device=self.device) * 4.0 - 2.0   # [-2.0, 2.0]
        self.goal_positions[env_ids, 2] = 0.5
        self.prev_high_level_cmd[env_ids] = 0.0
        self._episode_los_exposed_sum[env_ids] = 0.0
        self._episode_los_blocked_sum[env_ids] = 0.0
        self._episode_los_step_count[env_ids] = 0.0

        if self.render_mode is not None:
            self._draw_debug_vis()
