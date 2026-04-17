import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

import torch

from isaaclab.app import AppLauncher


def _wrap_to_pi(angle: torch.Tensor) -> torch.Tensor:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _quat_wxyz_to_yaw(quat_wxyz: torch.Tensor) -> torch.Tensor:
    """Extract world yaw from quaternions in [w, x, y, z] format."""
    w = quat_wxyz[:, 0]
    x = quat_wxyz[:, 1]
    y = quat_wxyz[:, 2]
    z = quat_wxyz[:, 3]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return torch.atan2(siny_cosp, cosy_cosp)


def _extract_lidar_distances(env, max_distance: float = 10.0) -> torch.Tensor:
    sensor = env.scene.sensors["lidar"]
    hit_pos = sensor.data.ray_hits_w
    sensor_pos = sensor.data.pos_w.unsqueeze(1)
    distances = torch.norm(hit_pos - sensor_pos, dim=-1)
    distances = torch.nan_to_num(distances, nan=max_distance, posinf=max_distance, neginf=max_distance)
    return torch.clamp(distances, max=max_distance)


def _compute_baseline_actions(env, args_cli) -> torch.Tensor:
    """
    Non-RL controller:
    - Attractive component towards the goal in body frame.
    - Repulsive component from nearby static obstacles from lidar.
    - Yaw tracks the resultant desired heading.

    Enemy/LOS states are intentionally NOT used in this controller.
    """
    robot = env.scene["robot"]
    robot_pos = robot.data.root_pos_w[:, :3]
    robot_quat = robot.data.root_quat_w
    yaw = _quat_wxyz_to_yaw(robot_quat)

    goal_pos_w = env.goal_positions + env.scene.env_origins
    goal_vec_w = goal_pos_w[:, :2] - robot_pos[:, :2]
    goal_dist = torch.norm(goal_vec_w, dim=-1)

    cos_yaw = torch.cos(yaw)
    sin_yaw = torch.sin(yaw)

    # World -> body frame rotation using yaw.
    goal_vec_b_x = cos_yaw * goal_vec_w[:, 0] + sin_yaw * goal_vec_w[:, 1]
    goal_vec_b_y = -sin_yaw * goal_vec_w[:, 0] + cos_yaw * goal_vec_w[:, 1]
    goal_vec_b = torch.stack((goal_vec_b_x, goal_vec_b_y), dim=-1)
    goal_dir_b = goal_vec_b / (torch.norm(goal_vec_b, dim=-1, keepdim=True) + 1e-6)

    # Lidar-based obstacle repulsion in body frame.
    distances = _extract_lidar_distances(env, max_distance=10.0)
    num_rays = distances.shape[1]
    ray_angles = torch.linspace(-math.pi, math.pi, num_rays, device=env.device)
    ray_dx = torch.cos(ray_angles).unsqueeze(0)
    ray_dy = torch.sin(ray_angles).unsqueeze(0)

    near = torch.clamp(args_cli.safety_radius - distances, min=0.0)
    weights = (near / (args_cli.safety_radius + 1e-6)) ** 2

    repulse_x = -torch.sum(weights * ray_dx, dim=1)
    repulse_y = -torch.sum(weights * ray_dy, dim=1)
    repulse_vec_b = torch.stack((repulse_x, repulse_y), dim=-1)
    repulse_vec_b = repulse_vec_b / (torch.norm(repulse_vec_b, dim=-1, keepdim=True) + 1e-6)

    desired_vec_b = args_cli.goal_gain * goal_dir_b + args_cli.obstacle_gain * repulse_vec_b

    desired_heading_b = torch.atan2(desired_vec_b[:, 1], desired_vec_b[:, 0])
    heading_error = _wrap_to_pi(desired_heading_b)

    # Speed scheduling: slow down if turning hard or very close to goal.
    turn_slowdown = 1.0 - torch.clamp(torch.abs(heading_error) / math.pi, min=0.0, max=1.0)
    proximity_slowdown = torch.clamp(goal_dist / args_cli.slow_radius, min=0.15, max=1.0)
    speed_scale = turn_slowdown * proximity_slowdown

    vx = args_cli.max_vx * speed_scale * torch.clamp(desired_vec_b[:, 0], min=0.0, max=1.0)
    vy = args_cli.vy_gain * torch.clamp(desired_vec_b[:, 1], min=-1.0, max=1.0)
    wz = args_cli.yaw_kp * heading_error

    # Clamp to environment-safe command envelope.
    vx = torch.clamp(vx, 0.0, 1.4)
    vy = torch.clamp(vy, -0.35, 0.35)
    wz = torch.clamp(wz, -1.0, 1.0)

    # Stop when close enough to the goal region.
    close = goal_dist < args_cli.goal_stop_radius
    vx = torch.where(close, torch.zeros_like(vx), vx)
    vy = torch.where(close, torch.zeros_like(vy), vy)
    wz = torch.where(close, torch.zeros_like(wz), wz)

    return torch.stack((vx, vy, wz), dim=-1)


def _extract_los_from_extras_or_env(env, extras):
    los_exposed = float("nan")
    los_blocked = float("nan")

    if isinstance(extras, dict):
        log_dict = extras.get("log", {})
        if isinstance(log_dict, dict):
            exposed_val = log_dict.get("LOS/exposed_step_mean", None)
            blocked_val = log_dict.get("LOS/blocked_step_mean", None)
            if torch.is_tensor(exposed_val):
                los_exposed = float(exposed_val.item())
            elif isinstance(exposed_val, (int, float)):
                los_exposed = float(exposed_val)
            if torch.is_tensor(blocked_val):
                los_blocked = float(blocked_val.item())
            elif isinstance(blocked_val, (int, float)):
                los_blocked = float(blocked_val)

    if math.isnan(los_exposed) and hasattr(env, "is_exposed_mask"):
        los_exposed = float(env.is_exposed_mask.float().mean().item())
    if math.isnan(los_blocked) and hasattr(env, "is_blocked_mask"):
        los_blocked = float(env.is_blocked_mask.float().mean().item())

    return los_exposed, los_blocked


def _extract_episode_los_from_extras_or_env(env, extras):
    episode_exposed = float("nan")
    episode_blocked = float("nan")
    episode_count = 0.0

    if isinstance(extras, dict):
        log_dict = extras.get("log", {})
        if isinstance(log_dict, dict):
            exposed_val = log_dict.get("Episode_LOS/exposed_ratio", None)
            blocked_val = log_dict.get("Episode_LOS/blocked_ratio", None)
            count_val = log_dict.get("Episode_LOS/done_count", None)
            if torch.is_tensor(exposed_val):
                episode_exposed = float(exposed_val.item())
            elif isinstance(exposed_val, (int, float)):
                episode_exposed = float(exposed_val)
            if torch.is_tensor(blocked_val):
                episode_blocked = float(blocked_val.item())
            elif isinstance(blocked_val, (int, float)):
                episode_blocked = float(blocked_val)
            if torch.is_tensor(count_val):
                episode_count = float(count_val.item())
            elif isinstance(count_val, (int, float)):
                episode_count = float(count_val)

    return episode_exposed, episode_blocked, episode_count


def main():
    parser = argparse.ArgumentParser(description="Classical (non-RL) tactical navigation baseline")
    parser.add_argument("--num_envs", type=int, default=1, help="Number of parallel environments")
    parser.add_argument("--num_steps", type=int, default=6000, help="Number of simulation steps")
    parser.add_argument("--real_time", action="store_true", default=False, help="Try to run close to real-time")

    parser.add_argument("--goal_gain", type=float, default=1.0, help="Attractive gain toward goal")
    parser.add_argument("--obstacle_gain", type=float, default=0.9, help="Repulsive gain from lidar obstacles")
    parser.add_argument("--safety_radius", type=float, default=1.0, help="Obstacle influence distance (m)")

    parser.add_argument("--yaw_kp", type=float, default=1.2, help="Heading proportional gain")
    parser.add_argument("--max_vx", type=float, default=1.1, help="Max forward command")
    parser.add_argument("--vy_gain", type=float, default=0.25, help="Lateral correction gain")

    parser.add_argument("--slow_radius", type=float, default=2.0, help="Start slowing down inside this goal distance")
    parser.add_argument("--goal_stop_radius", type=float, default=0.75, help="Controller stop radius near goal")
    parser.add_argument("--goal_success_radius", type=float, default=0.8, help="Distance used for success accounting")

    parser.add_argument("--log_interval", type=int, default=200, help="Print stats every N steps")
    parser.add_argument(
        "--csv_log_path",
        type=str,
        default=None,
        help="Optional CSV path. Default: logs/baselines/non_rl_baseline_<timestamp>.csv",
    )

    AppLauncher.add_app_launcher_args(parser)
    args_cli = parser.parse_args()

    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    sys.path.append(os.path.join(os.path.dirname(__file__), "..", "exts", "tactical_nav"))
    from tactical_nav.envs.tactical_nav_env import TacticalNavEnv
    from tactical_nav.envs.tactical_nav_env_cfg import TacticalNavEnvCfg

    env_cfg = TacticalNavEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device

    env = TacticalNavEnv(cfg=env_cfg, render_mode="rgb_array")

    csv_path = args_cli.csv_log_path
    if csv_path is None:
        os.makedirs(os.path.join("logs", "baselines"), exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join("logs", "baselines", f"non_rl_baseline_{timestamp}.csv")
    else:
        parent = os.path.dirname(csv_path)
        if parent:
            os.makedirs(parent, exist_ok=True)

    csv_file = open(csv_path, "w", newline="")
    csv_writer = csv.DictWriter(
        csv_file,
        fieldnames=[
            "step",
            "sim_time_s",
            "num_envs",
            "mean_reward",
            "mean_goal_dist",
            "done_step",
            "episodes_done_total",
            "episodes_success_total",
            "success_rate",
            "los_exposed_step_mean",
            "los_blocked_step_mean",
            "episode_los_exposed_ratio",
            "episode_los_blocked_ratio",
            "episode_los_done_count",
            "los_exposed_ema",
            "los_blocked_ema",
        ],
    )
    csv_writer.writeheader()

    obs = env.reset()
    if isinstance(obs, tuple):
        obs = obs[0]

    dt = env.step_dt
    total_done = 0
    total_success = 0
    steps_run = 0
    total_episode_count = 0.0
    total_episode_exposed_weighted = 0.0
    total_episode_blocked_weighted = 0.0

    los_exposed_ema = 0.0
    los_blocked_ema = 0.0
    ema_alpha = 0.05

    print(
        f"[INFO] Running non-RL baseline for {args_cli.num_steps} steps "
        f"with {args_cli.num_envs} env(s)."
    )
    print(f"[INFO] CSV logging to: {csv_path}")

    for step in range(args_cli.num_steps):
        if not simulation_app.is_running():
            break
        steps_run += 1

        step_start = time.time()

        with torch.inference_mode():
            actions = _compute_baseline_actions(env, args_cli)
            step_out = env.step(actions)

        if len(step_out) == 5:
            obs, rewards, terminated, truncated, extras = step_out
            dones = torch.logical_or(terminated, truncated)
        else:
            obs, rewards, dones, extras = step_out

        robot_pos = env.scene["robot"].data.root_pos_w[:, :3]
        goal_pos_w = env.goal_positions + env.scene.env_origins
        goal_dist = torch.norm(goal_pos_w - robot_pos, dim=-1)

        done_count = int(dones.sum().item())
        success_count = int(torch.logical_and(dones, goal_dist < args_cli.goal_success_radius).sum().item())
        total_done += done_count
        total_success += success_count

        los_exposed, los_blocked = _extract_los_from_extras_or_env(env, extras)
        episode_exposed, episode_blocked, episode_count = _extract_episode_los_from_extras_or_env(env, extras)
        if not math.isnan(los_exposed):
            los_exposed_ema = (1.0 - ema_alpha) * los_exposed_ema + ema_alpha * los_exposed
        if not math.isnan(los_blocked):
            los_blocked_ema = (1.0 - ema_alpha) * los_blocked_ema + ema_alpha * los_blocked
        if episode_count > 0.0 and not math.isnan(episode_exposed):
            total_episode_count += episode_count
            total_episode_exposed_weighted += episode_exposed * episode_count
        if episode_count > 0.0 and not math.isnan(episode_blocked):
            total_episode_blocked_weighted += episode_blocked * episode_count

        if step % args_cli.log_interval == 0:
            mean_reward = float(rewards.float().mean().item())
            mean_goal_dist = float(goal_dist.mean().item())
            success_rate = (total_success / total_done) if total_done > 0 else 0.0
            episode_exposed_mean = (total_episode_exposed_weighted / total_episode_count) if total_episode_count > 0 else float("nan")
            episode_blocked_mean = (total_episode_blocked_weighted / total_episode_count) if total_episode_count > 0 else float("nan")
            sim_time_s = float(steps_run * dt)
            print(
                "[BASELINE] "
                f"step={step:05d} mean_reward={mean_reward:.4f} "
                f"goal_dist={mean_goal_dist:.3f} done_step={done_count} "
                f"success_rate={success_rate:.3f} "
                f"los_exposed={los_exposed:.3f} los_blocked={los_blocked:.3f} "
                f"episode_los_exposed={episode_exposed_mean:.3f} episode_los_blocked={episode_blocked_mean:.3f} "
                f"los_exposed_ema={los_exposed_ema:.3f} los_blocked_ema={los_blocked_ema:.3f}"
            )
            csv_writer.writerow(
                {
                    "step": step,
                    "sim_time_s": sim_time_s,
                    "num_envs": args_cli.num_envs,
                    "mean_reward": mean_reward,
                    "mean_goal_dist": mean_goal_dist,
                    "done_step": done_count,
                    "episodes_done_total": total_done,
                    "episodes_success_total": total_success,
                    "success_rate": success_rate,
                    "los_exposed_step_mean": los_exposed,
                    "los_blocked_step_mean": los_blocked,
                    "episode_los_exposed_ratio": episode_exposed,
                    "episode_los_blocked_ratio": episode_blocked,
                    "episode_los_done_count": episode_count,
                    "los_exposed_ema": los_exposed_ema,
                    "los_blocked_ema": los_blocked_ema,
                }
            )
            csv_file.flush()

        if args_cli.real_time:
            sleep_time = dt - (time.time() - step_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    final_success_rate = (total_success / total_done) if total_done > 0 else 0.0
    final_mean_goal_dist = float(goal_dist.mean().item()) if steps_run > 0 else float("nan")
    final_mean_reward = float(rewards.float().mean().item()) if steps_run > 0 else float("nan")
    final_episode_exposed_mean = (total_episode_exposed_weighted / total_episode_count) if total_episode_count > 0 else float("nan")
    final_episode_blocked_mean = (total_episode_blocked_weighted / total_episode_count) if total_episode_count > 0 else float("nan")
    print(
        "[SUMMARY] "
        f"steps={steps_run} episodes_done={total_done} episodes_success={total_success} "
        f"success_rate={final_success_rate:.3f} "
        f"episode_los_exposed_mean={final_episode_exposed_mean:.3f} "
        f"episode_los_blocked_mean={final_episode_blocked_mean:.3f} "
        f"los_exposed_ema={los_exposed_ema:.3f} los_blocked_ema={los_blocked_ema:.3f}"
    )

    csv_writer.writerow(
        {
            "step": "summary",
            "sim_time_s": float(steps_run * dt),
            "num_envs": args_cli.num_envs,
            "mean_reward": final_mean_reward,
            "mean_goal_dist": final_mean_goal_dist,
            "done_step": 0,
            "episodes_done_total": total_done,
            "episodes_success_total": total_success,
            "success_rate": final_success_rate,
            "los_exposed_step_mean": float("nan"),
            "los_blocked_step_mean": float("nan"),
            "episode_los_exposed_ratio": final_episode_exposed_mean,
            "episode_los_blocked_ratio": final_episode_blocked_mean,
            "episode_los_done_count": total_episode_count,
            "los_exposed_ema": los_exposed_ema,
            "los_blocked_ema": los_blocked_ema,
        }
    )
    csv_file.flush()
    csv_file.close()

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
