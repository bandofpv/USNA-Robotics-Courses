import math
from dataclasses import MISSING
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg, ObservationGroupCfg, ObservationTermCfg, RewardTermCfg, TerminationTermCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.managers.action_manager import ActionTermCfg
import isaaclab.envs.mdp as mdp
from isaaclab.terrains import TerrainImporterCfg, TerrainGeneratorCfg
from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG
from isaaclab.terrains.height_field.hf_terrains_cfg import HfDiscreteObstaclesTerrainCfg
import tactical_nav.envs.rewards as custom_rewards
from isaaclab.utils import configclass
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

@configclass
class UrbanCoverTerrainCfg(TerrainGeneratorCfg):
    """Configuration for an urban cover terrain generator."""
    size = (8.0, 8.0)
    border_width = 20.0
    num_rows = 10
    num_cols = 20
    horizontal_scale = 0.1
    vertical_scale = 0.005
    slope_threshold = 0.75
    use_cache = False
    sub_terrains = {
        "discrete_obstacles": HfDiscreteObstaclesTerrainCfg(
            proportion=1.0,
            size=(8.0, 8.0),
            obstacle_height_mode="fixed",
            obstacle_height_range=(1.5, 1.5), # Tall enough to block line of sight
            obstacle_width_range=(0.5, 2.0),
            num_obstacles=8, # Sparse cover
            platform_width=3.0,
        ),
    }

from isaaclab.sensors import ContactSensorCfg
from isaaclab.sensors import RayCasterCfg, patterns
from isaaclab_tasks.manager_based.navigation.mdp.pre_trained_policy_action import PreTrainedPolicyActionCfg
from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.rough_env_cfg import UnitreeGo2RoughEnvCfg

LOW_LEVEL_ENV_CFG = UnitreeGo2RoughEnvCfg()

@configclass
class TacticalNavSceneCfg(InteractiveSceneCfg):
    """Configuration for the tactical navigation scene."""
    
    # Use custom urban terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=UrbanCoverTerrainCfg(),
        max_init_terrain_level=5,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )

    # Robot Asset
    robot: ArticulationCfg = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # Sensors
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        update_period=0.005,
        history_length=3,
        track_air_time=True,
    )
    
    # Height scanner for the low-level walking policy
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=True,
        mesh_prim_paths=["/World/ground"],
    )

    # Raycaster to detect the urban cover (Walls/Obstacles)
    lidar = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.3)), # Mount at 30cm
        ray_alignment="yaw",
        pattern_cfg=patterns.LidarPatternCfg(
            channels=1,
            vertical_fov_range=[0.0, 0.0],
            horizontal_fov_range=[-180.0, 180.0],
            horizontal_res=5.0, # 5 degree resolution
        ),
        max_distance=10.0,
        debug_vis=True,
        mesh_prim_paths=["/World/ground"],
    )

    # Lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


def vector_to_goal(env) -> torch.Tensor:
    """Observation: relative vector to goal."""
    robot_pos = env.scene["robot"].data.root_pos_w[:, :3]
    return env.goal_positions + env.scene.env_origins - robot_pos

def lidar_distances(env) -> torch.Tensor:
    """Observation: Lidar raycasting distances to detect walls."""
    sensor = env.scene.sensors["lidar"]
    hit_pos = sensor.data.ray_hits_w
    sensor_pos = sensor.data.pos_w.unsqueeze(1)
    
    # Calculate distance, max distance if it didn't hit anything
    distances = torch.norm(hit_pos - sensor_pos, dim=-1)
    
    # Handle infinite/missed rays by clamping to max distance
    distances = torch.nan_to_num(distances, posinf=10.0)
    return torch.clamp(distances, max=10.0)

@configclass
class ActionsCfg:
    """Action specifications for the MDP."""
    # High level policy outputs [vx, vy, angular_z]
    # Low level policy receives those and outputs 12 joint positions
    pre_trained_policy_action = PreTrainedPolicyActionCfg(
        asset_name="robot",
        policy_path="policies/policy.pt", # We will use the exported RSL-RL policy
        low_level_decimation=4, # Run the walking policy 4x faster than the nav policy
        low_level_actions=LOW_LEVEL_ENV_CFG.actions.joint_pos,
        low_level_observations=LOW_LEVEL_ENV_CFG.observations.policy,
        debug_vis=True, # Re-enabled velocity command arrows
    )

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group."""
        
        # We need something simple for now
        base_lin_vel = ObservationTermCfg(func=mdp.base_lin_vel, noise=None)
        base_ang_vel = ObservationTermCfg(func=mdp.base_ang_vel, noise=None)
        projected_gravity = ObservationTermCfg(func=mdp.projected_gravity, noise=None)
        
        # High level tactical observation inputs
        goal_vector = ObservationTermCfg(func=vector_to_goal, noise=None)
        lidar = ObservationTermCfg(func=lidar_distances, noise=None, scale=0.1) # scale by 1 / max_dist
        
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()

@configclass
class RewardsCfg:
    """Reward specifications for the MDP."""
    # Priority: goal reaching > obstacle avoidance > LOS avoidance.
    goal_navigation = RewardTermCfg(
        func=custom_rewards.reward_goal_navigation,
        weight=3.4,
        params={"goal_radius": 0.8},
    )
    obstacle_clearance = RewardTermCfg(
        func=custom_rewards.reward_obstacle_clearance,
        weight=1.5,
        params={"safe_distance": 0.6},
    )
    line_of_sight_penalty = RewardTermCfg(
        func=custom_rewards.penalty_line_of_sight,
        weight=-0.6,
    )
    motion_instability_penalty = RewardTermCfg(
        func=custom_rewards.penalty_motion_instability,
        weight=-0.25,
    )

@configclass
class EventsCfg:
    """Event specifications for the MDP."""
    # Reset base position and velocity
    reset_base = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.1, 0.1), "y": (-0.1, 0.1), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )

    # Reset joint position and velocity
    reset_robot_joints = EventTermCfg(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (1.0, 1.0), # Default default standing pose unmodified
            "velocity_range": (0.0, 0.0),
        },
    )

@configclass
class TerminationsCfg:
    """Termination specifications for the MDP."""
    goal_reached = TerminationTermCfg(
        func=custom_rewards.termination_goal_reached,
        params={"threshold": 0.8},
    )
    time_out = TerminationTermCfg(func=mdp.time_out)
    # Detect when base hits the ground
    base_contact = TerminationTermCfg(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 3.0},
    )

@configclass
class TacticalNavEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the tactical navigation environment."""

    scene: TacticalNavSceneCfg = TacticalNavSceneCfg(num_envs=128, env_spacing=4.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventsCfg = EventsCfg()
    commands: dict = {}
    curriculum: dict = {}

    def __post_init__(self):
        """Post initialization."""
        self.decimation = 4
        self.episode_length_s = 60.0
        self.sim.dt = 0.005 # Match usual settings
        self.viewer.eye = [5.0, 5.0, 5.0]
        self.viewer.lookat = [0.0, 0.0, 0.0]

    # Threat positions shared across the config
    threat_positions = torch.tensor([[5.0, 5.0, 1.0], [-5.0, 5.0, 1.0]])

