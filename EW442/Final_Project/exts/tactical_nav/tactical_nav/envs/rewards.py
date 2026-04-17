import torch
from .raycast_utils import check_line_of_sight

def reward_goal_navigation(env, goal_radius: float = 0.75) -> torch.Tensor:
    """Primary objective: move to goal quickly and get bonus when close to goal."""
    robot = env.scene["robot"]
    robot_pos = robot.data.root_pos_w[:, :3]
    robot_vel_xy = robot.data.root_lin_vel_w[:, :2]

    goal_pos_w = env.goal_positions + env.scene.env_origins
    vec_to_goal = goal_pos_w - robot_pos
    dist_to_goal = torch.norm(vec_to_goal, dim=-1)

    dir_to_goal_xy = vec_to_goal[:, :2] / (torch.norm(vec_to_goal[:, :2], dim=-1, keepdim=True) + 1e-6)
    progress = torch.sum(robot_vel_xy * dir_to_goal_xy, dim=-1)
    # Use steeper shaping so reward is concentrated near true completion.
    shaping = torch.exp(-1.0 * dist_to_goal)
    reach_bonus = (dist_to_goal < goal_radius).float()

    return 1.0 * progress + 0.4 * shaping + 6.0 * reach_bonus


def reward_obstacle_clearance(env, safe_distance: float = 0.55) -> torch.Tensor:
    """Secondary objective: avoid obstacles with smooth continuous shaping."""
    sensor = env.scene.sensors["lidar"]
    hit_pos = sensor.data.ray_hits_w
    sensor_pos = sensor.data.pos_w.unsqueeze(1)

    distances = torch.norm(hit_pos - sensor_pos, dim=-1)
    distances = torch.nan_to_num(distances, nan=10.0, posinf=10.0, neginf=10.0)
    distances = torch.clamp(distances, max=10.0)

    min_dist = torch.min(distances, dim=1).values
    clearance_bonus = torch.clamp(min_dist - safe_distance, min=0.0, max=1.5) / 1.5
    too_close_penalty = torch.relu(safe_distance - min_dist)
    return clearance_bonus - 2.5 * too_close_penalty


def penalty_line_of_sight(env) -> torch.Tensor:
    """Tertiary objective: reduce time spent exposed to threats."""
    robot_positions = env.scene["robot"].data.root_pos_w[:, :3]
    threat_positions_w = env.scene.env_origins.unsqueeze(1) + env.cfg.threat_positions.to(env.device).float().unsqueeze(0)
    exposure_mask, blocked_mask = check_line_of_sight(
        env,
        robot_positions=robot_positions,
        threat_positions_w=threat_positions_w,
        max_distance=20.0,
    )

    # Keep masks available for visual debugging in the same frame.
    env.is_exposed_mask = exposure_mask
    env.is_blocked_mask = blocked_mask

    num_threats = max(exposure_mask.shape[1], 1)
    return exposure_mask.sum(dim=-1).float() / float(num_threats)


def penalty_motion_instability(env) -> torch.Tensor:
    """Small regularizer to discourage aggressive motions that can cause flips."""
    robot = env.scene["robot"]
    ang_vel_roll_pitch = torch.norm(robot.data.root_ang_vel_w[:, :2], dim=-1)
    z_vel = torch.abs(robot.data.root_lin_vel_w[:, 2])
    return ang_vel_roll_pitch + 0.5 * z_vel


def termination_goal_reached(env, threshold: float = 0.75) -> torch.Tensor:
    """Terminate episode successfully when robot reaches the goal region."""
    robot_pos = env.scene["robot"].data.root_pos_w[:, :3]
    goal_pos_w = env.goal_positions + env.scene.env_origins
    dist_to_goal = torch.norm(goal_pos_w - robot_pos, dim=-1)
    return dist_to_goal < threshold
