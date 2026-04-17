import argparse
import os
import sys
import torch
from datetime import datetime

# Initialize Omniverse application before importing IsaacLab modules
from isaaclab.app import AppLauncher

def main():
    parser = argparse.ArgumentParser(description="Train Tactical Navigation Agent")
    parser.add_argument("--num_envs", type=int, default=128, help="Number of environments to run")
    parser.add_argument(
        "--baseline_no_los",
        action="store_true",
        help="Disable LOS penalty reward (goal+obstacle+motion-instability only baseline)",
    )
    
    # Add AppLauncher arguments
    AppLauncher.add_app_launcher_args(parser)
    args_cli = parser.parse_args()

    # Launch Omniverse application
    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    # Now we can import IsaacLab modules safely
    sys.path.append(os.path.join(os.path.dirname(__file__), "..", "exts", "tactical_nav"))
    from isaaclab.envs import ManagerBasedRLEnvCfg
    from tactical_nav.envs.tactical_nav_env import TacticalNavEnv
    from tactical_nav.envs.tactical_nav_env_cfg import TacticalNavEnvCfg

    import gymnasium as gym
    import importlib.metadata
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg
    from isaaclab_rl.rsl_rl import handle_deprecated_rsl_rl_cfg
    from rsl_rl.runners import OnPolicyRunner

    # Create environment configuration
    env_cfg = TacticalNavEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device

    if args_cli.baseline_no_los:
        env_cfg.rewards.line_of_sight_penalty.weight = 0.0
        print("[INFO] Baseline mode enabled: LOS penalty reward disabled.")

    experiment_name = "tactical_nav_baseline_no_los" if args_cli.baseline_no_los else "tactical_nav"

    # Configure runner
    runner_cfg = RslRlOnPolicyRunnerCfg(
        num_steps_per_env=24,
        max_iterations=1500,
        save_interval=50,
        experiment_name=experiment_name,
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
        )
    )

    # Logging setup
    log_dir = os.path.join("logs", "rsl_rl", runner_cfg.experiment_name)
    os.makedirs(log_dir, exist_ok=True)

    # Initialize environment (disable rendering when running headless)
    render_mode = None if getattr(args_cli, "headless", False) else "rgb_array"
    env = TacticalNavEnv(cfg=env_cfg, render_mode=render_mode)

    # Wrap for RSL-RL
    env = RslRlVecEnvWrapper(env, clip_actions=runner_cfg.clip_actions)

    # Handle deprecated rsl_rl configuration for newer rsl_rl versions
    try:
        installed_version = importlib.metadata.version("rsl-rl-lib")
    except importlib.metadata.PackageNotFoundError:
        try:
            installed_version = importlib.metadata.version("rsl_rl_lib")
        except importlib.metadata.PackageNotFoundError:
            installed_version = "4.0.0" # Fallback if all else fails
    
    runner_cfg = handle_deprecated_rsl_rl_cfg(runner_cfg, installed_version)

    print("Beginning RSL-RL training loop...")
    runner = OnPolicyRunner(env, runner_cfg.to_dict(), log_dir=log_dir, device=runner_cfg.device)
    runner.learn(num_learning_iterations=runner_cfg.max_iterations, init_at_random_ep_len=True)

    print("Training finished.")
    env.close()
    simulation_app.close()

if __name__ == '__main__':
    main()