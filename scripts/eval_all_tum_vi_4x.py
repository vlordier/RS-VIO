#!/usr/bin/env python3
"""
Multi-sequence TUM-VI evaluation with 4 runs per sequence.
Tests selective priors strategy (σ=200m maps + σ=50m newest KF).

Usage:
    python3 scripts/eval_all_tum_vi_4x.py [--baseline <commit_hash>] [--json <output.json>]
"""

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Tuple

# TUM-VI sequences to evaluate
SEQUENCES = ["slides1", "corridor1", "room1", "corridor2"]
NUM_RUNS = 4
DATA_BASE = "/Users/vincent/data/tum-vi"
PROJECT_ROOT = Path(__file__).parent.parent


def get_dataset_path(sequence: str) -> Path:
    """Get the dataset path for a TUM-VI sequence."""
    return Path(DATA_BASE) / f"dataset-{sequence}_512_16"


def get_gt_path(sequence: str) -> Path:
    """Get the ground truth path for a TUM-VI sequence."""
    return get_dataset_path(sequence) / "mav0" / "mocap0" / "data.csv"


def run_vio(sequence: str, run_num: int) -> Tuple[bool, str]:
    """
    Run RS-VIO on a sequence. Returns (success, estimated_trajectory_path).
    """
    dataset_path = get_dataset_path(sequence)
    config_path = PROJECT_ROOT / "config" / "tum_vi.yaml"
    binary_path = PROJECT_ROOT / "target" / "release" / "run_tum"

    if not dataset_path.exists():
        return False, f"Dataset not found: {dataset_path}"

    if not config_path.exists():
        return False, f"Config not found: {config_path}"

    if not binary_path.exists():
        return False, f"Binary not found: {binary_path}"

    # Remove old trajectory file
    est_traj = dataset_path / "estimated_trajectory.txt"
    if est_traj.exists():
        est_traj.unlink()

    try:
        print(f"  Run {run_num}/{NUM_RUNS}: Running {sequence}...", end=" ", flush=True)
        env = os.environ.copy()
        env["RUST_LOG"] = "info,rs_vio=debug,rerun=warn,re_log=warn,re_sdk=warn"

        result = subprocess.run(
            [str(binary_path), str(config_path), str(dataset_path)],
            env=env,
            timeout=600,
            capture_output=True,
            text=True
        )

        if result.returncode != 0:
            print(f"✗ VIO failed (exit code {result.returncode})")
            return False, str(result.stderr[-200:])

        if not est_traj.exists():
            print("✗ No trajectory file generated")
            return False, "Missing estimated_trajectory.txt"

        print("✓")
        return True, str(est_traj)

    except subprocess.TimeoutExpired:
        print("✗ Timeout")
        return False, "VIO timed out after 600s"
    except Exception as e:
        print(f"✗ Exception: {e}")
        return False, str(e)


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

        # Extract ATE statistics (RMSE, mean, std, max)
        match = re.search(r"RMSE\s*=\s*([\d.]+)", output)
        if match:
            metrics["ate_rmse"] = float(match.group(1))

        match = re.search(r"mean\s*=\s*([\d.]+)", output)
        if match:
            metrics["ate_mean"] = float(match.group(1))

        match = re.search(r"std\s*=\s*([\d.]+)", output)
        if match:
            metrics["ate_std"] = float(match.group(1))

        match = re.search(r"max\s*=\s*([\d.]+)", output)
        if match:
            metrics["ate_max"] = float(match.group(1))

        # Extract per-segment ATEs (look for "Per-GT-segment breakdown" format)
        match = re.search(r"Per-GT-segment breakdown:(.*?)(?=\n\n|$)", output, re.DOTALL)
        if match:
            segments = re.findall(r"ATE=([\d.]+)m", match.group(1))
            if segments:
                metrics["seg_ates"] = [float(s) for s in segments]

        # Extract RPE at 0.1s interval (closest to 1s for instant feedback)
        match = re.search(r"=== RPE Δ=0\.10s.*?trans RMSE:\s*([\d.]+)\s*m/s", output, re.DOTALL)
        if match:
            metrics["rpe_01s"] = float(match.group(1))

        return metrics

    except Exception as e:
        return {"error": str(e)}


def aggregate_metrics(all_runs: List[Dict]) -> Dict:
    """
    Aggregate metrics across multiple runs.
    """
    failed = [r for r in all_runs if "error" in r]
    if failed:
        return {"error": f"Some runs failed: {len(failed)}/{len(all_runs)}"}

    agg = {}

    # Aggregate over runs: mean, std, min, max
    for key in ["scale", "ate_rmse", "ate_mean", "ate_std", "ate_max", "rpe_01s"]:
        values = [r.get(key) for r in all_runs if r.get(key) is not None]
        if values:
            agg[f"{key}_mean"] = sum(values) / len(values)
            agg[f"{key}_std"] = (sum((v - agg[f"{key}_mean"])**2 for v in values) / len(values)) ** 0.5
            agg[f"{key}_min"] = min(values)
            agg[f"{key}_max"] = max(values)

    return agg


def main():
    parser = argparse.ArgumentParser(description="Evaluate all TUM-VI sequences with 4 runs")
    parser.add_argument("--baseline", help="Baseline commit hash for comparison")
    parser.add_argument("--json", help="Output JSON file for results")
    args = parser.parse_args()

    print("=" * 80)
    print("RS-VIO Multi-Sequence Evaluation (Selective Priors)")
    print("=" * 80)
    print(f"Sequences: {', '.join(SEQUENCES)}")
    print(f"Runs per sequence: {NUM_RUNS}")
    print()

    all_results = {}

    for sequence in SEQUENCES:
        print(f"\n{sequence.upper()}")
        print("-" * 40)

        dataset_path = get_dataset_path(sequence)
        gt_path = get_gt_path(sequence)

        if not dataset_path.exists():
            print(f"  Dataset not found: {dataset_path}")
            continue

        if not gt_path.exists():
            print(f"  GT not found: {gt_path}")
            continue

        seq_runs = []

        for run_num in range(1, NUM_RUNS + 1):
            success, est_path = run_vio(sequence, run_num)

            if not success:
                print(f"    Error: {est_path}")
                seq_runs.append({"error": est_path})
                continue

            # Evaluate
            metrics = evaluate_trajectory(est_path, str(gt_path), sequence)
            seq_runs.append(metrics)

            if "error" not in metrics:
                ate = metrics.get("ate_rmse")
                scale_pct = metrics.get("scale_pct")
                if ate is not None and scale_pct is not None:
                    print(f"ATE={ate:.3f}m, scale={scale_pct:+.2f}%")
                elif ate is not None:
                    print(f"ATE={ate:.3f}m")
                else:
                    print("metrics incomplete")
            else:
                print(f"error: {metrics.get('error', 'unknown')}")

        # Aggregate
        agg = aggregate_metrics(seq_runs)
        all_results[sequence] = {
            "runs": seq_runs,
            "aggregate": agg
        }

        # Print aggregate
        if "error" not in agg and "ate_rmse_mean" in agg:
            mean_ate = agg["ate_rmse_mean"]
            std_ate = agg["ate_rmse_std"]
            min_ate = agg["ate_rmse_min"]
            max_ate = agg["ate_rmse_max"]
            print(f"\n  AGGREGATE ATE RMSE: {mean_ate:.3f}±{std_ate:.3f}m (min={min_ate:.3f}m, max={max_ate:.3f}m)")

            # Also print max error (per-frame worst case) if available
            if "ate_max_mean" in agg:
                max_err_mean = agg["ate_max_mean"]
                max_err_std = agg["ate_max_std"]
                print(f"  AGGREGATE ATE MAX:  {max_err_mean:.3f}±{max_err_std:.3f}m (per-frame worst case)")

            # And variance (per-frame std) if available
            if "ate_std_mean" in agg:
                var_mean = agg["ate_std_mean"]
                var_std = agg["ate_std_std"]
                print(f"  AGGREGATE ATE STD:  {var_mean:.3f}±{var_std:.3f}m (per-frame variance)")

            if "scale_mean" in agg:
                mean_scale = agg["scale_mean"]
                std_scale = agg["scale_std"]
                print(f"  AGGREGATE SCALE:    {mean_scale:.3f}±{std_scale:.4f}")

    # Summary table
    print("\n" + "=" * 100)
    print("SUMMARY - Baseline with Improved Relocalization")
    print("=" * 100)
    print(f"{'Sequence':<12} {'ATE RMSE':<15} {'ATE Max':<15} {'ATE Std':<15} {'Scale':<12} {'Runs':<6}")
    print("-" * 100)

    for sequence in SEQUENCES:
        if sequence not in all_results:
            print(f"{sequence:<12} {'N/A':<15} {'N/A':<15} {'N/A':<15} {'N/A':<12} {'':<6} Dataset not found")
            continue

        runs = all_results[sequence]["runs"]
        agg = all_results[sequence]["aggregate"]

        if "error" in agg:
            success = sum(1 for r in runs if "error" not in r)
            print(f"{sequence:<12} {'FAILED':<15} {'':<15} {'':<15} {'':<12} {success}/{NUM_RUNS:<5} {agg['error']}")
        elif "ate_rmse_mean" in agg:
            ate_rmse_str = f"{agg['ate_rmse_mean']:.3f}±{agg['ate_rmse_std']:.3f}"
            ate_max_str = f"{agg['ate_max_mean']:.3f}±{agg['ate_max_std']:.3f}" if "ate_max_mean" in agg else "N/A"
            ate_std_str = f"{agg['ate_std_mean']:.3f}±{agg['ate_std_std']:.3f}" if "ate_std_mean" in agg else "N/A"
            scale_str = f"{agg['scale_mean']:.3f}±{agg['scale_std']:.4f}"
            print(f"{sequence:<12} {ate_rmse_str:<15} {ate_max_str:<15} {ate_std_str:<15} {scale_str:<12} {NUM_RUNS:<6} ✓")
        else:
            print(f"{sequence:<12} {'ERROR':<15} {'':<15} {'':<15} {'':<12} {'':<6} Parsing failed")

    # Save JSON if requested
    if args.json:
        with open(args.json, "w") as f:
            json.dump(all_results, f, indent=2)
        print(f"\nResults saved to {args.json}")

    print()


if __name__ == "__main__":
    main()
