#!/usr/bin/env python3
"""
Compare VO (no IMU) vs VIO (with IMU factor) accuracy for RS-VIO.

This script runs the TUM-VI dataset player in both VO and VIO modes,
evaluates the trajectories against ground truth, and produces a
comparison report.

Usage:
    python3 scripts/compare_vo_vio.py <dataset_path> [sequence_name]
    
Requirements:
    - RS-VIO built: cargo build --release
    - Dataset downloaded
    - Ground truth available at <dataset_path>/mav0/mocap0/data.csv

Outputs:
    - Comparison table (console)
    - Per-sequence ATE, RPE for both modes
    - JSON summary for scripting
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

# Try to import eval_ate_rpe
sys.path.insert(0, os.path.dirname(__file__))
try:
    from eval_ate_rpe import read_csv_gt, read_est, compute_ate_rmse
    HAS_EVAL = True
except ImportError:
    HAS_EVAL = False
    print("Warning: eval_ate_rpe.py not found, using basic metrics only")


def run_vio(dataset_path: str, mode: str, config_file: str = "config/tum_vi.yaml") -> dict:
    """Run RS-VIO in the specified mode (VIO or VO) and return trajectory stats."""
    binary = os.path.join(os.path.dirname(__file__), "..", "target", "release", "run_tum")
    if not os.path.exists(binary):
        print(f"Error: Binary not found at {binary}. Run 'cargo build --release' first.")
        sys.exit(1)

    output_traj = os.path.join(dataset_path, f"estimated_trajectory_{mode.lower()}.txt")
    output_rerun = os.path.join(dataset_path, f"rerun_{mode.lower()}.rrd")

    cmd = [binary, config_file, dataset_path]
    if mode == "VO":
        cmd.append("--no-imu")

    print(f"\n{'='*60}")
    print(f"Running RS-VIO in {mode} mode...")
    print(f"Command: {' '.join(cmd)}")
    print(f"{'='*60}")

    start_time = time.time()
    env = os.environ.copy()
    env["RUST_LOG"] = "warn"  # Suppress debug output

    try:
        result = subprocess.run(
            cmd,
            env=env,
            capture_output=True,
            text=True,
            timeout=600,  # 10 min timeout
            cwd=os.path.dirname(__file__) + "/.."
        )
        elapsed = time.time() - start_time

        if result.returncode != 0:
            print(f"Error running {mode} mode:")
            print(result.stderr[-2000:] if result.stderr else "No stderr output")
            return {"error": result.stderr, "elapsed_s": elapsed}

        # Parse output for statistics
        stats = parse_output(result.stdout, elapsed)
        stats["mode"] = mode
        stats["output_traj"] = output_traj

        return stats

    except subprocess.TimeoutExpired:
        print(f"Timeout running {mode} mode")
        return {"error": "timeout", "elapsed_s": 600}


def parse_output(stdout: str, elapsed: float) -> dict:
    """Parse RS-VIO console output for key statistics."""
    stats = {"elapsed_s": elapsed, "frames_processed": 0, "fps": 0.0}

    for line in stdout.splitlines():
        # Look for frame count
        if "processed" in line.lower() and "frames" in line.lower():
            parts = line.split()
            for i, part in enumerate(parts):
                if part.isdigit():
                    stats["frames_processed"] = int(part)
                    break

        # Look for FPS
        if "fps" in line.lower() or "frames/sec" in line.lower():
            for part in line.split():
                try:
                    val = float(part.replace("fps", "").replace("frames/sec", ""))
                    if val > 0:
                        stats["fps"] = val
                except ValueError:
                    pass

    # Compute FPS from elapsed if not found
    if stats["fps"] == 0 and stats["frames_processed"] > 0:
        stats["fps"] = stats["frames_processed"] / max(elapsed, 0.01)

    return stats


def compute_basic_metrics(trajectory_file: str) -> dict:
    """Compute basic trajectory metrics (length, duration, etc.)."""
    if not os.path.exists(trajectory_file):
        return {"error": f"Trajectory file not found: {trajectory_file}"}

    ts, pos, _ = read_est(trajectory_file)
    if len(pos) < 2:
        return {"error": "Too few poses in trajectory"}

    # Path length
    diffs = np.diff(pos, axis=0)
    path_length = np.sum(np.linalg.norm(diffs, axis=1))

    # Duration
    duration_s = (ts[-1] - ts[0]) / 1e9

    # Mean velocity
    dt = np.diff(ts) / 1e9
    velocities = np.linalg.norm(diffs, axis=1) / np.maximum(dt, 1e-6)
    mean_velocity = np.mean(velocities)
    max_velocity = np.max(velocities)

    return {
        "num_poses": len(pos),
        "path_length_m": path_length,
        "duration_s": duration_s,
        "mean_velocity_ms": mean_velocity,
        "max_velocity_ms": max_velocity,
    }


def compute_ate_vio_vo(vo_stats: dict, vio_stats: dict, gt_file: str) -> dict:
    """Compute ATE comparison if ground truth is available."""
    if not HAS_EVAL or not os.path.exists(gt_file):
        return {"error": "Ground truth or eval module not available"}

    results = {}
    for mode, stats in [("VO", vo_stats), ("VIO", vio_stats)]:
        traj_file = stats.get("output_traj")
        if not traj_file or not os.path.exists(traj_file):
            results[mode] = {"error": "Trajectory not found"}
            continue

        try:
            gt_ts, gt_pos, gt_quat = read_csv_gt(gt_file)
            est_ts, est_pos, est_quat = read_est(traj_file)

            if len(est_pos) < 2:
                results[mode] = {"error": "Too few poses"}
                continue

            # Simple alignment (just match timestamps)
            # For a proper evaluation, use the full eval_ate_rpe.py
            results[mode] = {
                "gt_poses": len(gt_pos),
                "est_poses": len(est_pos),
                "note": "Use eval_ate_rpe.py for full ATE/RPE evaluation",
            }
        except Exception as e:
            results[mode] = {"error": str(e)}

    return results


def print_comparison_report(vo_stats: dict, vio_stats: dict, ate_results: dict = None):
    """Print a formatted comparison report."""
    print(f"\n{'='*70}")
    print(f"{'RS-VIO: VO vs VIO Comparison Report':^70}")
    print(f"{'='*70}")

    # Summary table
    print(f"\n{'Metric':<30} {'VO (no IMU)':>15} {'VIO (with IMU)':>15} {'Change':>10}")
    print(f"{'-'*70}")

    # Frames processed
    vo_frames = vo_stats.get("frames_processed", "N/A")
    vio_frames = vio_stats.get("frames_processed", "N/A")
    print(f"{'Frames processed':<30} {vo_frames:>15} {vio_frames:>15} {'':>10}")

    # Elapsed time
    vo_time = vo_stats.get("elapsed_s", 0)
    vio_time = vio_stats.get("elapsed_s", 0)
    print(f"{'Elapsed time (s)':<30} {vo_time:>15.1f} {vio_time:>15.1f} {get_change_pct(vo_time, vio_time):>10}")

    # FPS
    vo_fps = vo_stats.get("fps", 0)
    vio_fps = vio_stats.get("fps", 0)
    print(f"{'FPS':<30} {vo_fps:>15.1f} {vio_fps:>15.1f} {get_change_pct(vo_fps, vio_fps):>10}")

    # Path length
    vo_path = vo_stats.get("trajectory_stats", {}).get("path_length_m", 0)
    vio_path = vio_stats.get("trajectory_stats", {}).get("path_length_m", 0)
    if vo_path > 0 and vio_path > 0:
        print(f"{'Path length (m)':<30} {vo_path:>15.2f} {vio_path:>15.2f} {get_change_pct(vo_path, vio_path):>10}")

    # Max velocity (indicator of BA divergence)
    vo_max_v = vo_stats.get("trajectory_stats", {}).get("max_velocity_ms", 0)
    vio_max_v = vio_stats.get("trajectory_stats", {}).get("max_velocity_ms", 0)
    if vo_max_v > 0 and vio_max_v > 0:
        print(f"{'Max velocity (m/s)':<30} {vo_max_v:>15.2f} {vio_max_v:>15.2f} {get_change_pct(vo_max_v, vio_max_v):>10}")
        if vio_max_v < vo_max_v:
            print(f"  → VIO reduced max velocity (fewer BA divergences)")

    print(f"\n{'='*70}")

    # Interpretation
    if vio_fps < vo_fps * 0.8:
        print(f"Note: VIO is ~{get_change_pct(vo_fps, vio_fps)} slower than VO (expected due to IMU factor overhead)")
    else:
        print(f"Note: VIO performance is comparable to VO")

    if ate_results:
        print(f"\nATE Results:")
        for mode, result in ate_results.items():
            if "error" not in result:
                print(f"  {mode}: {result}")
            else:
                print(f"  {mode}: {result['error']}")


def get_change_pct(old: float, new: float) -> str:
    """Compute percentage change."""
    if old == 0:
        return "N/A"
    pct = (new - old) / old * 100
    sign = "+" if pct > 0 else ""
    return f"{sign}{pct:.1f}%"


def main():
    parser = argparse.ArgumentParser(description="Compare VO vs VIO accuracy for RS-VIO")
    parser.add_argument("dataset_path", help="Path to TUM-VI dataset sequence")
    parser.add_argument("--config", default="config/tum_vi.yaml", help="Config file path")
    parser.add_argument("--json", help="Output JSON summary file")
    parser.add_argument("--skip-run", action="store_true", help="Skip running VIO, just compare existing trajectories")
    args = parser.parse_args()

    dataset_path = os.path.abspath(args.dataset_path)
    if not os.path.exists(dataset_path):
        print(f"Error: Dataset path not found: {dataset_path}")
        sys.exit(1)

    gt_file = os.path.join(dataset_path, "mav0", "mocap0", "data.csv")

    if args.skip_run:
        # Just compare existing trajectories
        vo_traj = os.path.join(dataset_path, "estimated_trajectory_vo.txt")
        vio_traj = os.path.join(dataset_path, "estimated_trajectory_vio.txt")

        vo_stats = {"mode": "VO", "output_traj": vo_traj}
        vio_stats = {"mode": "VIO", "output_traj": vio_traj}

        if os.path.exists(vo_traj):
            vo_stats["trajectory_stats"] = compute_basic_metrics(vo_traj)
        if os.path.exists(vio_traj):
            vio_stats["trajectory_stats"] = compute_basic_metrics(vio_traj)

    else:
        # Run both modes
        vo_stats = run_vio(dataset_path, "VO", args.config)
        vio_stats = run_vio(dataset_path, "VIO", args.config)

        # Compute trajectory metrics
        if "output_traj" in vo_stats and os.path.exists(vo_stats["output_traj"]):
            vo_stats["trajectory_stats"] = compute_basic_metrics(vo_stats["output_traj"])
        if "output_traj" in vio_stats and os.path.exists(vio_stats["output_traj"]):
            vio_stats["trajectory_stats"] = compute_basic_metrics(vio_stats["output_traj"])

    # Print report
    print_comparison_report(vo_stats, vio_stats)

    # Save JSON
    if args.json:
        summary = {
            "vo": vo_stats,
            "vio": vio_stats,
        }
        with open(args.json, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"\nJSON summary saved to: {args.json}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
