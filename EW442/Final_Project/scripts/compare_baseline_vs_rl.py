import argparse
import csv
import math
import os
from datetime import datetime
from typing import Iterable

import matplotlib.pyplot as plt


def _to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _parse_candidates(csv_text: str) -> list[str]:
    return [x.strip() for x in csv_text.split(",") if x.strip()]


def _find_latest_event_file(run_dir: str) -> str:
    if os.path.isfile(run_dir):
        return run_dir

    if not os.path.isdir(run_dir):
        raise FileNotFoundError(f"RL log path does not exist: {run_dir}")

    candidates = []
    for root, _, files in os.walk(run_dir):
        for name in files:
            if name.startswith("events.out.tfevents"):
                full = os.path.join(root, name)
                candidates.append((os.path.getmtime(full), full))

    if not candidates:
        raise FileNotFoundError(f"No TensorBoard event files found under: {run_dir}")

    candidates.sort(key=lambda x: x[0])
    return candidates[-1][1]


def _resolve_rl_source(rl_log_path: str | None, rl_csv: str | None) -> str:
    if rl_csv:
        if not os.path.isfile(rl_csv):
            raise FileNotFoundError(f"RL CSV does not exist: {rl_csv}")
        return "csv"
    if rl_log_path:
        return "event"
    raise ValueError("Provide --rl_csv or --rl_log_path.")


def _read_baseline_series(csv_path: str, x_col: str, y_col: str) -> tuple[list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"CSV appears empty: {csv_path}")
        if x_col not in reader.fieldnames:
            raise ValueError(f"Column '{x_col}' not found in baseline CSV. Available: {reader.fieldnames}")
        if y_col not in reader.fieldnames:
            raise ValueError(f"Column '{y_col}' not found in baseline CSV. Available: {reader.fieldnames}")

        for row in reader:
            x = _to_float(row.get(x_col))
            y = _to_float(row.get(y_col))
            if x is None or y is None:
                continue
            xs.append(x)
            ys.append(y)

    return xs, ys


def _read_csv_series(csv_path: str, x_col: str, y_col: str) -> tuple[list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"CSV appears empty: {csv_path}")
        if x_col not in reader.fieldnames:
            raise ValueError(f"Column '{x_col}' not found in CSV. Available: {reader.fieldnames}")
        if y_col not in reader.fieldnames:
            raise ValueError(f"Column '{y_col}' not found in CSV. Available: {reader.fieldnames}")

        for row in reader:
            if str(row.get("step", "")).lower() == "summary":
                continue
            x = _to_float(row.get(x_col))
            y = _to_float(row.get(y_col))
            if x is None or y is None:
                continue
            xs.append(x)
            ys.append(y)

    return xs, ys


def _read_baseline_weighted_mean(csv_path: str, value_col: str, weight_col: str | None = None) -> tuple[float, int]:
    total = 0.0
    weight_total = 0.0

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"CSV appears empty: {csv_path}")
        if value_col not in reader.fieldnames:
            raise ValueError(f"Column '{value_col}' not found in baseline CSV. Available: {reader.fieldnames}")
        if weight_col is not None and weight_col not in reader.fieldnames:
            raise ValueError(f"Column '{weight_col}' not found in baseline CSV. Available: {reader.fieldnames}")

        for row in reader:
            if str(row.get("step", "")).lower() == "summary":
                continue
            value = _to_float(row.get(value_col))
            if value is None or math.isnan(value):
                continue
            if weight_col is None:
                total += value
                weight_total += 1.0
                continue

            weight = _to_float(row.get(weight_col))
            if weight is None or weight <= 0.0 or math.isnan(weight):
                continue
            total += value * weight
            weight_total += weight

    mean = total / weight_total if weight_total > 0.0 else float("nan")
    return mean, int(round(weight_total))


def _resolve_tag(available: Iterable[str], candidates: list[str]) -> str | None:
    available_list = list(available)
    available_set = set(available_list)

    for c in candidates:
        if c in available_set:
            return c

    lowered = {k.lower(): k for k in available_list}
    for c in candidates:
        if c.lower() in lowered:
            return lowered[c.lower()]

    for c in candidates:
        c_lower = c.lower()
        for k in available_list:
            if k.lower().endswith(c_lower):
                return k

    return None


def _smooth_ema(values: list[float], alpha: float) -> list[float]:
    if not values or alpha <= 0.0:
        return values

    out: list[float] = []
    ema = values[0]
    for v in values:
        ema = (1.0 - alpha) * ema + alpha * v
        out.append(ema)
    return out


def _load_rl_scalars(event_file: str, y_tag_candidates: list[str]) -> tuple[list[int], list[float], str | None, list[str]]:
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    except ImportError as exc:
        raise ImportError(
            "tensorboard is required to read RL event files. Install with: pip install tensorboard"
        ) from exc

    acc = EventAccumulator(event_file)
    acc.Reload()

    available_tags = sorted(acc.Tags().get("scalars", []))
    tag = _resolve_tag(available_tags, y_tag_candidates)
    if tag is None:
        return [], [], None, available_tags

    scalar_events = acc.Scalars(tag)
    xs = [int(e.step) for e in scalar_events]
    ys = [float(e.value) for e in scalar_events]
    return xs, ys, tag, available_tags


def _load_rl_scalar_map(event_file: str, tag_candidates: list[str]) -> tuple[dict[int, float], str | None, list[str]]:
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    except ImportError as exc:
        raise ImportError(
            "tensorboard is required to read RL event files. Install with: pip install tensorboard"
        ) from exc

    acc = EventAccumulator(event_file)
    acc.Reload()

    available_tags = sorted(acc.Tags().get("scalars", []))
    tag = _resolve_tag(available_tags, tag_candidates)
    if tag is None:
        return {}, None, available_tags

    scalar_events = acc.Scalars(tag)
    series = {int(e.step): float(e.value) for e in scalar_events}
    return series, tag, available_tags


def _save_combined_csv(
    out_csv: str,
    baseline_x: list[float],
    baseline_y: list[float],
    rl_x: list[int],
    rl_y: list[float],
    baseline_label: str,
    rl_label: str,
):
    max_len = max(len(baseline_x), len(rl_x))
    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "idx",
                "baseline_x",
                "baseline_y",
                "rl_x",
                "rl_y",
                "baseline_label",
                "rl_label",
            ],
        )
        writer.writeheader()

        for i in range(max_len):
            writer.writerow(
                {
                    "idx": i,
                    "baseline_x": baseline_x[i] if i < len(baseline_x) else "",
                    "baseline_y": baseline_y[i] if i < len(baseline_y) else "",
                    "rl_x": rl_x[i] if i < len(rl_x) else "",
                    "rl_y": rl_y[i] if i < len(rl_y) else "",
                    "baseline_label": baseline_label,
                    "rl_label": rl_label,
                }
            )


def _plot_one(
    out_png: str,
    baseline_x: list[float],
    baseline_y: list[float],
    rl_x: list[int],
    rl_y: list[float],
    title: str,
    x_label_baseline: str,
    y_label: str,
    baseline_name: str,
    rl_name: str,
):
    fig = plt.figure(figsize=(11, 6))
    ax = fig.add_subplot(1, 1, 1)

    if rl_x and rl_y:
        ax.plot(rl_x, rl_y, label=rl_name, linewidth=2.0)
    if baseline_x and baseline_y:
        ax.plot(baseline_x, baseline_y, label=baseline_name, linewidth=2.0)

    ax.set_title(title)
    ax.set_xlabel(f"Baseline X: {x_label_baseline} | RL X: global_step")
    ax.set_ylabel(y_label)
    ax.grid(True, alpha=0.3)
    ax.legend()

    fig.tight_layout()
    fig.savefig(out_png, dpi=160)
    plt.close(fig)


def _plot_summary_bars(out_png: str, labels: list[str], values: list[float], title: str, ylabel: str):
    fig = plt.figure(figsize=(8, 5))
    ax = fig.add_subplot(1, 1, 1)
    bars = ax.bar(labels, values, color=["#3b82f6", "#f97316"])
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True, axis="y", alpha=0.3)
    ax.set_ylim(bottom=0.0)
    for bar, value in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2.0, bar.get_height(), f"{value:.3f}", ha="center", va="bottom")
    fig.tight_layout()
    fig.savefig(out_png, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Compare non-RL baseline CSV against RL TensorBoard logs or RL CSV")

    parser.add_argument("--baseline_csv", type=str, required=True, help="Path to baseline CSV from baseline_traditional_nav.py")
    parser.add_argument(
        "--rl_log_path",
        type=str,
        default="logs/rsl_rl/tactical_nav",
        help="RL run directory (searched recursively for latest event file) or explicit event file",
    )
    parser.add_argument(
        "--rl_csv",
        type=str,
        default=None,
        help="Optional RL CSV path from play.py. If provided, RL data is loaded from CSV instead of TensorBoard.",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default="logs/comparisons",
        help="Directory for output plots and combined CSVs",
    )
    parser.add_argument(
        "--output_prefix",
        type=str,
        default=None,
        help="Optional output prefix. Default: compare_<timestamp>",
    )

    parser.add_argument("--baseline_x_col", type=str, default="sim_time_s", help="Baseline CSV x-axis column")
    parser.add_argument("--rl_x_col", type=str, default="sim_time_s", help="RL CSV x-axis column when using --rl_csv")

    parser.add_argument(
        "--metric",
        type=str,
        default="los_exposed",
        choices=["los_exposed", "los_blocked", "mean_reward", "success_rate"],
        help="Metric to compare",
    )
    parser.add_argument(
        "--rl_tag_candidates",
        type=str,
        default=None,
        help="Comma-separated RL scalar tag candidates. Overrides metric defaults.",
    )

    parser.add_argument(
        "--smooth_alpha",
        type=float,
        default=0.0,
        help="EMA smoothing alpha in [0, 1]. 0 disables smoothing.",
    )
    parser.add_argument(
        "--average_over_episodes",
        action="store_true",
        help="Compare the mean LOS exposure ratio across completed episodes.",
    )
    parser.add_argument(
        "--list_tags_only",
        action="store_true",
        help="Print available RL scalar tags and exit.",
    )

    args = parser.parse_args()

    metric_cfg = {
        "los_exposed": {
            "baseline_col": "los_exposed_ema",
            "rl_csv_col": "los_exposed_ema",
            "rl_candidates": ["Episode_LOS/exposed_ratio", "LOS/exposed_step_mean", "exposed_ratio"],
            "title": "LOS Exposure Comparison",
            "ylabel": "Exposure Ratio",
            "episode_baseline_col": "episode_los_exposed_ratio",
            "episode_baseline_weight_col": "episode_los_done_count",
            "episode_rl_csv_col": "episode_los_exposed_ratio",
            "episode_rl_csv_weight_col": "episode_los_done_count",
            "episode_rl_candidates": ["Episode_LOS/exposed_ratio"],
            "episode_rl_weight_candidates": ["Episode_LOS/done_count"],
        },
        "los_blocked": {
            "baseline_col": "los_blocked_ema",
            "rl_csv_col": "los_blocked_ema",
            "rl_candidates": ["Episode_LOS/blocked_ratio", "LOS/blocked_step_mean", "blocked_ratio"],
            "title": "LOS Blocked Comparison",
            "ylabel": "Blocked Ratio",
            "episode_baseline_col": "episode_los_blocked_ratio",
            "episode_baseline_weight_col": "episode_los_done_count",
            "episode_rl_csv_col": "episode_los_blocked_ratio",
            "episode_rl_csv_weight_col": "episode_los_done_count",
            "episode_rl_candidates": ["Episode_LOS/blocked_ratio"],
            "episode_rl_weight_candidates": ["Episode_LOS/done_count"],
        },
        "mean_reward": {
            "baseline_col": "mean_reward",
            "rl_csv_col": "mean_reward",
            "rl_candidates": ["Train/mean_reward", "Episode/rew_total", "mean_reward"],
            "title": "Mean Reward Comparison",
            "ylabel": "Reward",
        },
        "success_rate": {
            "baseline_col": "success_rate",
            "rl_csv_col": "success_rate",
            "rl_candidates": ["Episode/success_rate", "success_rate", "Goal/success_rate"],
            "title": "Success Rate Comparison",
            "ylabel": "Success Rate",
        },
    }

    cfg = metric_cfg[args.metric]
    baseline_col = cfg["baseline_col"]
    rl_csv_col = cfg.get("rl_csv_col", baseline_col)
    rl_candidates = _parse_candidates(args.rl_tag_candidates) if args.rl_tag_candidates else cfg["rl_candidates"]
    rl_source = _resolve_rl_source(args.rl_log_path, args.rl_csv)

    event_file = None
    rl_tag = None
    available_tags: list[str] = []
    rl_label_source = ""

    if rl_source == "event":
        event_file = _find_latest_event_file(args.rl_log_path)
        rl_x, rl_y, rl_tag, available_tags = _load_rl_scalars(event_file, rl_candidates)
        rl_label_source = rl_tag if rl_tag is not None else "not_found"
    else:
        rl_x, rl_y = _read_csv_series(args.rl_csv, args.rl_x_col, rl_csv_col)
        rl_tag = rl_csv_col
        rl_label_source = f"csv:{rl_csv_col}"

    if args.list_tags_only:
        if rl_source != "event":
            raise ValueError("--list_tags_only is only available when RL source is TensorBoard event logs.")
        print(f"[INFO] Event file: {event_file}")
        print("[INFO] Available scalar tags:")
        for tag in available_tags:
            print(f"  - {tag}")
        return

    if args.average_over_episodes and args.metric in {"los_exposed", "los_blocked"}:
        baseline_mean, baseline_count = _read_baseline_weighted_mean(
            args.baseline_csv,
            cfg["episode_baseline_col"],
            cfg["episode_baseline_weight_col"],
        )

        if rl_source == "csv":
            rl_mean, rl_count = _read_baseline_weighted_mean(
                args.rl_csv,
                cfg["episode_rl_csv_col"],
                cfg["episode_rl_csv_weight_col"],
            )
            rl_value_tag = cfg["episode_rl_csv_col"]
            rl_weight_tag = cfg["episode_rl_csv_weight_col"]
        else:
            rl_values, rl_value_tag, _ = _load_rl_scalar_map(event_file, cfg["episode_rl_candidates"])
            rl_weights, rl_weight_tag, _ = _load_rl_scalar_map(event_file, cfg["episode_rl_weight_candidates"])

            if not rl_values:
                raise ValueError("No matching RL episode metric tag found for average-over-episodes comparison.")

            common_steps = [step for step in rl_values.keys() if step in rl_weights] if rl_weight_tag is not None else list(rl_values.keys())
            if rl_weight_tag is not None and common_steps:
                rl_total = sum(rl_values[step] * rl_weights[step] for step in common_steps)
                rl_weight_total = sum(rl_weights[step] for step in common_steps)
                rl_mean = rl_total / rl_weight_total if rl_weight_total > 0.0 else float("nan")
                rl_count = int(round(rl_weight_total))
            else:
                rl_mean = sum(rl_values.values()) / len(rl_values)
                rl_count = len(rl_values)

        os.makedirs(args.output_dir, exist_ok=True)
        prefix = args.output_prefix or f"compare_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        out_png = os.path.join(args.output_dir, f"{prefix}_{args.metric}_episode_mean.png")
        out_csv = os.path.join(args.output_dir, f"{prefix}_{args.metric}_episode_mean.csv")

        _plot_summary_bars(
            out_png=out_png,
            labels=["baseline", "rl"],
            values=[baseline_mean, rl_mean],
            title=f"{cfg['title']} (episode mean)",
            ylabel=cfg["ylabel"],
        )

        with open(out_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["method", "mean", "samples", "value_tag", "weight_tag"])
            writer.writeheader()
            writer.writerow({"method": "baseline", "mean": baseline_mean, "samples": baseline_count, "value_tag": cfg["episode_baseline_col"], "weight_tag": cfg["episode_baseline_weight_col"]})
            writer.writerow({"method": "rl", "mean": rl_mean, "samples": rl_count, "value_tag": rl_value_tag, "weight_tag": rl_weight_tag or ""})

        if rl_source == "csv":
            print(f"[INFO] RL CSV file: {args.rl_csv}")
        else:
            print(f"[INFO] RL event file: {event_file}")
        print(f"[INFO] Baseline episode mean ({cfg['episode_baseline_col']}): {baseline_mean:.4f} over {baseline_count} episodes")
        print(f"[INFO] RL episode mean ({rl_value_tag}): {rl_mean:.4f} over {rl_count} episodes")
        print(f"[INFO] Plot saved: {out_png}")
        print(f"[INFO] Summary CSV saved: {out_csv}")
        return

    baseline_x, baseline_y = _read_csv_series(args.baseline_csv, args.baseline_x_col, baseline_col)

    if args.smooth_alpha < 0.0 or args.smooth_alpha > 1.0:
        raise ValueError("--smooth_alpha must be in [0, 1]")

    baseline_y_plot = _smooth_ema(baseline_y, args.smooth_alpha)
    rl_y_plot = _smooth_ema(rl_y, args.smooth_alpha)

    os.makedirs(args.output_dir, exist_ok=True)
    prefix = args.output_prefix or f"compare_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    out_png = os.path.join(args.output_dir, f"{prefix}_{args.metric}.png")
    out_csv = os.path.join(args.output_dir, f"{prefix}_{args.metric}.csv")

    baseline_name = f"baseline:{baseline_col}"
    rl_name = f"rl:{rl_label_source}"

    _plot_one(
        out_png=out_png,
        baseline_x=baseline_x,
        baseline_y=baseline_y_plot,
        rl_x=rl_x,
        rl_y=rl_y_plot,
        title=cfg["title"],
        x_label_baseline=args.baseline_x_col,
        y_label=cfg["ylabel"],
        baseline_name=baseline_name,
        rl_name=rl_name,
    )

    _save_combined_csv(
        out_csv=out_csv,
        baseline_x=baseline_x,
        baseline_y=baseline_y_plot,
        rl_x=rl_x,
        rl_y=rl_y_plot,
        baseline_label=baseline_name,
        rl_label=rl_name,
    )

    if rl_source == "csv":
        print(f"[INFO] RL CSV file: {args.rl_csv}")
        print(f"[INFO] RL column used: {rl_csv_col}")
    else:
        print(f"[INFO] RL event file: {event_file}")
        if rl_tag is None:
            print("[WARN] No matching RL tag found from candidates.")
            print("[WARN] Use --list_tags_only to inspect available tags and pass --rl_tag_candidates.")
        else:
            print(f"[INFO] RL tag used: {rl_tag}")

    print(f"[INFO] Baseline points: {len(baseline_x)}")
    print(f"[INFO] RL points: {len(rl_x)}")

    if baseline_y:
        print(f"[INFO] Baseline final ({baseline_col}): {baseline_y_plot[-1]:.4f}")
    if rl_y_plot:
        print(f"[INFO] RL final ({rl_tag}): {rl_y_plot[-1]:.4f}")

    print(f"[INFO] Plot saved: {out_png}")
    print(f"[INFO] Combined CSV saved: {out_csv}")


if __name__ == "__main__":
    main()
