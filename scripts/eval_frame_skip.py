#!/usr/bin/env python3
"""
Frame skip evaluation: compare 256x256 + CLAHE with and without frame skipping.
Tests frame_skip=1 (all frames) vs frame_skip=2 (every 2nd frame) for speedup vs accuracy trade-off.

Usage:
    python3 scripts/eval_frame_skip.py --json <output.json>
"""

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Tuple

# TUM-VI sequences to evaluate (room1 focus - long sequence)
SEQUENCES = ["room1", "slides1"]
NUM_RUNS = 2
DATA_BASE = "/Users/vincent/data/tum-vi"
PROJECT_ROOT = Path(__file__).parent.parent


def get_dataset_path(sequence: str) -> Path:
    """Get the dataset path for a TUM-VI sequence."""
    return Path(DATA_BASE) / f"dataset-{sequence}_512_16"


def get_gt_path(sequence: str) -> Path:
    """Get the ground truth path for a TUM-VI sequence."""
    return get_dataset_path(sequence) / "mav0" / "mocap0" / "data.csv"


def run_vio(sequence: str, config: str, run_num: int) -> Tuple[bool, str, float]:
    """
    Run RS-VIO on a sequence. Returns (success, estimated_trajectory_path, runtime_seconds).
    """
    dataset_path = get_dataset_path(sequence)
    config_path = PROJECT_ROOT / "config" / config
    binary_path = PROJECT_ROOT / "target" / "release" / "run_tum"

    if not dataset_path.exists():
        return False, f"Dataset not found: {dataset_path}", 0.0

    if not config_path.exists():
        return False, f"Config not found: {config_path}", 0.0

    if not binary_path.exists():
        return False, f"Binary not found: {binary_path}", 0.0

    # Remove old trajectory file
    est_traj = dataset_path / "estimated_trajectory.txt"
    if est_traj.exists():
        est_traj.unlink()

    try:
        print(f"    {config:<20} Run {run_num}/{NUM_RUNS}: ", end=" ", flush=True)
        env = os.environ.copy()
        env["RUST_LOG"] = "info,rs_vio=debug,rerun=warn,re_log=warn,re_sdk=warn"

        import time
        start_time = time.time()
        result = subprocess.run(
            [str(binary_path), str(config_path), str(dataset_path)],
            env=env,
            timeout=600,
            capture_output=True,
            text=True
        )
        elapsed = time.time() - start_time

        if result.returncode != 0:
            print(f"✗ VIO failed (exit code {result.returncode})")
            return False, str(result.stderr[-200:]), elapsed

        if not est_traj.exists():
            print("✗ No trajectory file generated")
            return False, "Missing estimated_trajectory.txt", elapsed

        print(f"✓ ({elapsed:.1f}s)")
        return True, str(est_traj), elapsed

    except subprocess.TimeoutExpired:
        print("✗ Timeout")
        return False, "VIO timed out after 600s", 0.0
    except Exception as e:
        print(f"✗ Exception: {e}")
        return False, str(e), 0.0


def evaluate_trajectory(est_path: str, gt_path: str, sequence: str) -> Dict:
    """
    Run eval_ate_rpe.py and parse the output.
    """
    try:
        result = subprocess.run(
            ["python3", str(PROJECT_ROOT / "scripts" / "eval_ate_rpe.py"),
             "--est", est_path,
             "--gt", gt_path],
            capture_output=True,
            text=True,
            timeout=30
        )

        if result.returncode != 0:
            return {"error": f"eval failed: {result.returncode}"}

        output = result.stdout
        metrics = {}

        # Extract Sim(3) scale (percentage)
        match = re.search(r"Sim\(3\) scale:\s+([\d.]+)\s*\(([-\d.]+)%\)", output)
        if match:
            metrics["scale"] = float(match.group(1))
            metrics["scale_pct"] = float(match.group(2))

        # Extract ATE RMSE
        match = re.search(r"RMSE\s*=\s*([\d.]+)", output)
        if match:
            metrics["ate_rmse"] = float(match.group(1))

        return metrics

    except Exception as e:
        return {"error": str(e)}


def main():
    parser = argparse.ArgumentParser(description="Evaluate frame skip performance trade-offs")
    parser.add_argument("--json", help="Output JSON file for results")
    args = parser.parse_args()

    print("=" * 80)
    print("Frame Skip Evaluation: 256×256 + CLAHE")
    print("=" * 80)
    print(f"Sequences: {', '.join(SEQUENCES)} (focus on speed vs accuracy)")
    print(f"Configs: frame_skip=1 (all frames) vs frame_skip=2 (every 2nd)")
    print(f"Runs per config: {NUM_RUNS}")
    print()

    configs = ["tum_vi_256.yaml", "tum_vi_256_2x_skip.yaml"]
    all_results = {}

    for sequence in SEQUENCES:
        print(f"\n{sequence.upper()}")
        print("-" * 80)

        dataset_path = get_dataset_path(sequence)
        gt_path = get_gt_path(sequence)

        if not dataset_path.exists():
            print(f"  Dataset not found: {dataset_path}")
            continue

        if not gt_path.exists():
            print(f"  GT not found: {gt_path}")
            continue

        seq_results = {}

        for config in configs:
            config_results = []
            runtimes = []

            for run_num in range(1, NUM_RUNS + 1):
                success, est_path, elapsed = run_vio(sequence, config, run_num)

                if not success:
                    print(f"      Error: {est_path}")
                    config_results.append({"error": est_path})
                    continue

                # Evaluate
                metrics = evaluate_trajectory(est_path, str(gt_path), sequence)
                config_results.append(metrics)
                runtimes.append(elapsed)

                if "error" not in metrics:
                    ate = metrics.get("ate_rmse")
                    scale_pct = metrics.get("scale_pct")
                    if ate is not None and scale_pct is not None:
                        print(f"      → ATE={ate:.3f}m, scale={scale_pct:+.2f}%")
                    elif ate is not None:
                        print(f"      → ATE={ate:.3f}m")

            # Average stats
            successful = [r for r in config_results if "error" not in r]
            if successful:
                avg_ate = sum(r.get("ate_rmse", 0) for r in successful) / len(successful)
                avg_runtime = sum(runtimes) / len(runtimes) if runtimes else 0
                print(f"  {config:<20} Average ATE: {avg_ate:.3f}m, Runtime: {avg_runtime:.1f}s/seq")

            seq_results[config] = config_results

        all_results[sequence] = seq_results

    # Summary comparison
    print("\n" + "=" * 80)
    print("SUMMARY - Speed vs Accuracy Trade-off")
    print("=" * 80)
    print(f"{'Sequence':<12} {'Config':<30} {'ATE (m)':<12} {'Speed gain':<12}")
    print("-" * 80)

    for sequence in SEQUENCES:
        if sequence not in all_results:
            continue

        results = all_results[sequence]

        # Get baseline (frame_skip=1)
        baseline_runs = results.get("tum_vi_256.yaml", [])
        baseline_ate = None
        if baseline_runs:
            successful = [r for r in baseline_runs if "error" not in r]
            if successful:
                baseline_ate = sum(r.get("ate_rmse", 0) for r in successful) / len(successful)

        # Compare skip=2
        skip_runs = results.get("tum_vi_256_2x_skip.yaml", [])
        skip_ate = None
        if skip_runs:
            successful = [r for r in skip_runs if "error" not in r]
            if successful:
                skip_ate = sum(r.get("ate_rmse", 0) for r in successful) / len(successful)

        if baseline_ate is not None:
            print(f"{sequence:<12} {'tum_vi_256.yaml':<30} {baseline_ate:.3f}m")
        if skip_ate is not None:
            if baseline_ate is not None:
                diff_pct = ((skip_ate - baseline_ate) / baseline_ate) * 100
                speedup = "~2.0×"  # Expected from frame_skip=2
                print(f"{'':<12} {'tum_vi_256_2x_skip.yaml':<30} {skip_ate:.3f}m {diff_pct:+.1f}% {speedup}")
            else:
                print(f"{'':<12} {'tum_vi_256_2x_skip.yaml':<30} {skip_ate:.3f}m")

    # Save JSON if requested
    if args.json:
        with open(args.json, "w") as f:
            json.dump(all_results, f, indent=2)
        print(f"\nResults saved to {args.json}")

    print()


if __name__ == "__main__":
    main()
