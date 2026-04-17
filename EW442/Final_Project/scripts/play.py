import argparse
import csv
import importlib.metadata
import math
import os
import re
import sys
import time
from datetime import datetime

import torch

from isaaclab.app import AppLauncher


def _resolve_installed_rsl_rl_version() -> str:
    """Resolve rsl-rl package version across common distribution names."""
    candidates = ["rsl-rl-lib", "rsl_rl_lib", "rsl-rl", "rsl_rl"]
    for name in candidates:
        try:
            return importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            continue
    # Safe fallback for compatibility conversion helper.
    return "4.0.0"


def _resolve_checkpoint(checkpoint: str | None, experiment_name: str) -> str:
    """Find checkpoint path. Uses explicit path if provided, else latest model_*.pt."""
    if checkpoint is not None:
        if not os.path.isfile(checkpoint):
            raise FileNotFoundError(f"Checkpoint not found: {checkpoint}")
        return checkpoint

    run_dir = os.path.join("logs", "rsl_rl", experiment_name)
    if not os.path.isdir(run_dir):
        raise FileNotFoundError(f"Run directory not found: {run_dir}")

    model_files = []
    for name in os.listdir(run_dir):
        match = re.fullmatch(r"model_(\d+)\.pt", name)
        if match is not None:
            model_files.append((int(match.group(1)), os.path.join(run_dir, name)))

    if not model_files:
        raise FileNotFoundError(f"No checkpoints found in: {run_dir}")

    model_files.sort(key=lambda x: x[0])
    return model_files[-1][1]


def _summarize_reset_reasons(extras) -> str:
    """Build a compact summary of termination/reset signals from extras."""
    if not isinstance(extras, dict):
        return "extras unavailable"

    candidates = ["goal_reached", "time_out", "base_contact", "terminated", "truncated"]
    found = []

    def _scan(prefix: str, value):
        if isinstance(value, dict):
            for key, item in value.items():
                _scan(f"{prefix}{key}.", item)
            return
        if torch.is_tensor(value):
            if value.numel() == 0:
                return
            name = prefix[:-1] if prefix.endswith(".") else prefix
            for candidate in candidates:
                if candidate in name:
                    found.append(f"{name}={value.float().mean().item():.3f}")
                    break

    _scan("", extras)
    if not found:
        return "no reset markers found"
    return ", ".join(found)


def _extract_episode_los_from_extras(extras) -> tuple[float, float, float]:
    episode_exposed = float("nan")
    episode_blocked = float("nan")
    episode_done_count = 0.0

    if not isinstance(extras, dict):
        return episode_exposed, episode_blocked, episode_done_count

    log_dict = extras.get("log", {})
    if not isinstance(log_dict, dict):
        return episode_exposed, episode_blocked, episode_done_count

    exposed_val = log_dict.get("Episode_LOS/exposed_ratio", None)
    blocked_val = log_dict.get("Episode_LOS/blocked_ratio", None)
    done_count_val = log_dict.get("Episode_LOS/done_count", None)

    if torch.is_tensor(exposed_val):
        episode_exposed = float(exposed_val.item())
    elif isinstance(exposed_val, (int, float)):
        episode_exposed = float(exposed_val)

    if torch.is_tensor(blocked_val):
        episode_blocked = float(blocked_val.item())
    elif isinstance(blocked_val, (int, float)):
        episode_blocked = float(blocked_val)

    if torch.is_tensor(done_count_val):
        episode_done_count = float(done_count_val.item())
    elif isinstance(done_count_val, (int, float)):
        episode_done_count = float(done_count_val)

    return episode_exposed, episode_blocked, episode_done_count


def main():
    parser = argparse.ArgumentParser(description="Play/Evaluate Tactical Navigation policy")
    parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to run")
    parser.add_argument("--num_steps", type=int, default=5000, help="Number of simulation steps to run")
    parser.add_argument("--checkpoint", type=str, default=None, help="Path to model checkpoint (*.pt)")
    parser.add_argument("--experiment_name", type=str, default="tactical_nav", help="RSL-RL experiment name")
    parser.add_argument("--real_time", action="store_true", default=False, help="Try to run at realtime speed")
    parser.add_argument("--goal_success_radius", type=float, default=0.8, help="Distance threshold used for success accounting")
    parser.add_argument("--log_interval", type=int, default=200, help="Print and CSV log interval in steps")
    parser.add_argument(
        "--csv_log_path",
        type=str,
        default=None,
        help="Optional CSV path. Default: logs/evals/rl_eval_<timestamp>.csv",
    )

    AppLauncher.add_app_launcher_args(parser)
    args_cli = parser.parse_args()

    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    sys.path.append(os.path.join(os.path.dirname(__file__), "..", "exts", "tactical_nav"))

    from isaaclab_rl.rsl_rl import (
        RslRlOnPolicyRunnerCfg,
        RslRlPpoActorCriticCfg,
        RslRlPpoAlgorithmCfg,
        RslRlVecEnvWrapper,
        handle_deprecated_rsl_rl_cfg,
    )
    from rsl_rl.runners import OnPolicyRunner
    from tactical_nav.envs.tactical_nav_env import TacticalNavEnv
    from tactical_nav.envs.tactical_nav_env_cfg import TacticalNavEnvCfg

    env_cfg = TacticalNavEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device

    # Keep this aligned with the train runner config.
    runner_cfg = RslRlOnPolicyRunnerCfg(
        num_steps_per_env=24,
        max_iterations=1500,
        save_interval=50,
        experiment_name=args_cli.experiment_name,
        seed=42,
        device=env_cfg.sim.device,
        obs_groups={"actor": ["policy"], "critic": ["policy"]},
        empirical_normalization=False,
        policy=RslRlPpoActorCriticCfg(
            init_noise_std=0.45,
            actor_obs_normalization=False,
            critic_obs_normalization=False,
            actor_hidden_dims=[256, 128, 64],
            critic_hidden_dims=[256, 128, 64],
            activation="elu",
        ),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=1.0,
            use_clipped_value_loss=True,
            clip_param=0.2,
            entropy_coef=0.003,
            num_learning_epochs=5,
            num_mini_batches=4,
            learning_rate=4e-4,
            schedule="adaptive",
            gamma=0.99,
            lam=0.95,
            desired_kl=0.010,
            max_grad_norm=1.0,
        ),
    )

    installed_version = _resolve_installed_rsl_rl_version()
    runner_cfg = handle_deprecated_rsl_rl_cfg(runner_cfg, installed_version)

    checkpoint_path = _resolve_checkpoint(args_cli.checkpoint, args_cli.experiment_name)
    print(f"[INFO] Loading checkpoint: {checkpoint_path}")

    csv_path = args_cli.csv_log_path
    if csv_path is None:
        os.makedirs(os.path.join("logs", "evals"), exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join("logs", "evals", f"rl_eval_{timestamp}.csv")
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
            "checkpoint",
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
    print(f"[INFO] CSV logging to: {csv_path}")

    env = TacticalNavEnv(cfg=env_cfg, render_mode="rgb_array")
    env = RslRlVecEnvWrapper(env, clip_actions=runner_cfg.clip_actions)

    runner = OnPolicyRunner(env, runner_cfg.to_dict(), log_dir=None, device=runner_cfg.device)
    runner.load(checkpoint_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    obs = env.get_observations()
    dt = env.unwrapped.step_dt
    los_exposure_ema = 0.0
    los_blocked_ema = 0.0
    ema_alpha = 0.05
    steps_run = 0
    total_done = 0
    total_success = 0
    total_episode_count = 0.0
    total_episode_exposed_weighted = 0.0
    total_episode_blocked_weighted = 0.0

    print(f"[INFO] Running evaluation for {args_cli.num_steps} steps with {args_cli.num_envs} env(s)")
    printed_extras_keys = False
    for step in range(args_cli.num_steps):
        if not simulation_app.is_running():
            break
        steps_run += 1

        start_time = time.time()
        with torch.inference_mode():
            actions = policy(obs)
            obs, rewards, dones, extras = env.step(actions)
            # Reset recurrent states for ended episodes.
            policy.reset(dones)

        # Direct LOS diagnostics from current frame masks.
        unwrapped_env = env.unwrapped
        if hasattr(unwrapped_env, "is_exposed_mask") and hasattr(unwrapped_env, "is_blocked_mask"):
            exposure_ratio = unwrapped_env.is_exposed_mask.float().mean().item()
            blocked_ratio = unwrapped_env.is_blocked_mask.float().mean().item()
            los_exposure_ema = (1.0 - ema_alpha) * los_exposure_ema + ema_alpha * exposure_ratio
            los_blocked_ema = (1.0 - ema_alpha) * los_blocked_ema + ema_alpha * blocked_ratio
        else:
            exposure_ratio = float("nan")
            blocked_ratio = float("nan")

        robot_pos = unwrapped_env.scene["robot"].data.root_pos_w[:, :3]
        goal_pos_w = unwrapped_env.goal_positions + unwrapped_env.scene.env_origins
        goal_dist = torch.norm(goal_pos_w - robot_pos, dim=-1)

        done_count = int(dones.sum().item())
        success_count = int(torch.logical_and(dones, goal_dist < args_cli.goal_success_radius).sum().item())
        total_done += done_count
        total_success += success_count

        episode_exposed, episode_blocked, episode_done_count = _extract_episode_los_from_extras(extras)
        if episode_done_count > 0.0 and not math.isnan(episode_exposed):
            total_episode_count += episode_done_count
            total_episode_exposed_weighted += episode_exposed * episode_done_count
        if episode_done_count > 0.0 and not math.isnan(episode_blocked):
            total_episode_blocked_weighted += episode_blocked * episode_done_count

        if step % args_cli.log_interval == 0:
            mean_reward = rewards.float().mean().item()
            done_rate = dones.float().mean().item()
            mean_goal_dist = goal_dist.mean().item()
            success_rate = (total_success / total_done) if total_done > 0 else 0.0
            sim_time_s = float(steps_run * dt)
            print(
                "[PLAY] "
                f"step={step:05d} mean_reward={mean_reward:.4f} done_rate={done_rate:.3f} "
                f"los_exposed={exposure_ratio:.3f} los_blocked={blocked_ratio:.3f} "
                f"success_rate={success_rate:.3f} "
                f"los_exposed_ema={los_exposure_ema:.3f} los_blocked_ema={los_blocked_ema:.3f}"
            )
            csv_writer.writerow(
                {
                    "step": step,
                    "sim_time_s": sim_time_s,
                    "num_envs": args_cli.num_envs,
                    "checkpoint": checkpoint_path,
                    "mean_reward": mean_reward,
                    "mean_goal_dist": mean_goal_dist,
                    "done_step": done_count,
                    "episodes_done_total": total_done,
                    "episodes_success_total": total_success,
                    "success_rate": success_rate,
                    "los_exposed_step_mean": exposure_ratio,
                    "los_blocked_step_mean": blocked_ratio,
                    "episode_los_exposed_ratio": episode_exposed,
                    "episode_los_blocked_ratio": episode_blocked,
                    "episode_los_done_count": episode_done_count,
                    "los_exposed_ema": los_exposure_ema,
                    "los_blocked_ema": los_blocked_ema,
                }
            )
            csv_file.flush()

        if not printed_extras_keys and isinstance(extras, dict):
            print(f"[DEBUG] extras keys: {sorted(list(extras.keys()))}")
            printed_extras_keys = True

        if torch.any(dones):
            reason_summary = _summarize_reset_reasons(extras)
            print(f"[RESET] step={step:05d} done_count={done_count} {reason_summary}")

        if args_cli.real_time:
            sleep_time = dt - (time.time() - start_time)
            if sleep_time > 0:
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
        f"episode_los_blocked_mean={final_episode_blocked_mean:.3f}"
    )

    csv_writer.writerow(
        {
            "step": "summary",
            "sim_time_s": float(steps_run * dt),
            "num_envs": args_cli.num_envs,
            "checkpoint": checkpoint_path,
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
            "los_exposed_ema": los_exposure_ema,
            "los_blocked_ema": los_blocked_ema,
        }
    )
    csv_file.flush()
    csv_file.close()

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
