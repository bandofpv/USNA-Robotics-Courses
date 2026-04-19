# Robot Dog Final Project

## Requirements

### Isaac Lab Workstation Setup

Set up Isaac Lab on your workstation by following the official installation guide:

- Isaac Lab Installation Documentation: https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html

This project uses **Isaac Sim 5.1**, so make sure you select and install the Isaac Lab/Isaac Sim versions that match that release.

### Conda Environment

Create the conda environment:

```bash
conda env create -f environment.yml
```

## Train Walking Policy (Isaac Lab)

In the Isaac Lab repository, run:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac--Velocity-Rough-Unitree-Go2-v0 --headless
```

Copy the trained policy file, `policy.pt`, into the `policies` directory in this project.

## Train the Custom RL Policy

From the `Final_Project` directory, run:

```bash
python scripts/train.py --num_envs 4096
```

If you do not wish to visualize the training process, add the `--headless` flag.
 
## Play the Custom RL Policy

From the `Final_Project` directory, run:

```bash
python scripts/play.py --experiment_name tactical_nav --num_envs 4096 --num_steps 147456000 --csv_log_path logs/evals/rl_eval.csv
```

## Play Artificial Field Function (Baseline)

From the `Final_Project` directory, run:

```bash
python scripts/baseline_traditional_nav.py --num_envs 1 --num_steps 147456000 --csv_log_path logs/evals/baseline_eval.csv
```

Compare baseline vs RL results:

```bash
python scripts/compare_baseline_vs_rl.py --baseline_csv logs/evals/baseline_eval.csv --rl_csv logs/evals/rl_eval.csv --metric los_exposed --metric los_blocked
```