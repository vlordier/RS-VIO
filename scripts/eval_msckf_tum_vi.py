#!/usr/bin/env python3
"""Run and evaluate the MSCKF TUM-VI pipeline.

This mirrors the existing TUM-VI evaluation helpers but targets the
`run_tum_msckf` binary and the `estimated_trajectory_msckf.txt` output.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DATA_BASE = Path("/Users/vincent/data/tum-vi")
DEFAULT_SEQUENCES = ["slides1", "corridor1", "room1", "corridor2"]
DEFAULT_TIMEOUT_S = 600

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_ate_rpe import read_csv_gt, read_est, compute_ate  # noqa: E402


def dataset_path(data_base: Path, sequence: str) -> Path:
    return data_base / f"dataset-{sequence}_512_16"


def gt_path(sequence_path: Path) -> Path:
    return sequence_path / "mav0" / "mocap0" / "data.csv"


def build_binary_if_needed(binary_path: Path, release: bool) -> None:
    if binary_path.exists():
        return
    cmd = ["cargo", "build"]
    if release:
        cmd.append("--release")
    cmd.extend(["--bin", "run_tum_msckf"])
    env = os.environ.copy()
    env.setdefault("CARGO_INCREMENTAL", "0")
    subprocess.run(
        cmd,
        cwd=PROJECT_ROOT,
        check=True,
        env=env,
    )


def resolve_binary(binary_override: str | None) -> tuple[Path, bool]:
    if binary_override:
        return Path(binary_override).resolve(), False

    release_path = PROJECT_ROOT / "target" / "release" / "run_tum_msckf"
    if release_path.exists():
        return release_path, True

    debug_path = PROJECT_ROOT / "target" / "debug" / "run_tum_msckf"
    if debug_path.exists():
        return debug_path, False

    return release_path, True


def run_msckf(sequence_path: Path, config_path: Path, timeout_s: int, no_imu: bool, binary_override: str | None) -> dict:
    binary_path, release = resolve_binary(binary_override)
    build_binary_if_needed(binary_path, release)

    output_traj = sequence_path / "estimated_trajectory_msckf.txt"
    if output_traj.exists():
        output_traj.unlink()

    cmd = [str(binary_path), str(config_path), str(sequence_path)]
    if no_imu:
        cmd.append("--no-imu")

    env = os.environ.copy()
    env.setdefault("RUST_LOG", "info")
    start = time.time()

    try:
        result = subprocess.run(
            cmd,
            cwd=PROJECT_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired:
        return {
            "ok": False,
            "error": f"timeout after {timeout_s}s",
            "elapsed_s": timeout_s,
        }

    elapsed_s = time.time() - start
    combined_output = "\n".join(part for part in (result.stdout, result.stderr) if part)

    if result.returncode != 0:
        return {
            "ok": False,
            "error": combined_output[-4000:] if combined_output else "binary failed",
            "elapsed_s": elapsed_s,
        }
    if not output_traj.exists():
        return {
            "ok": False,
            "error": f"missing trajectory output: {output_traj}",
            "elapsed_s": elapsed_s,
        }
    summary = parse_runner_output(combined_output)
    try:
        summary["poses_recorded"] = max(summary.get("poses_recorded", 0), count_trajectory_poses(output_traj))
    except OSError:
        pass
    summary.update(
        {
            "ok": True,
            "elapsed_s": elapsed_s,
            "trajectory_path": str(output_traj),
            "binary_path": str(binary_path),
            "stdout_tail": tail_lines(combined_output, 20),
        }
    )
    return summary


def tail_lines(text: str, line_count: int) -> str:
    lines = text.strip().splitlines()
    return "\n".join(lines[-line_count:])


def count_trajectory_poses(path: Path) -> int:
    with path.open() as handle:
        return sum(1 for line in handle if line.strip() and not line.startswith("#"))


def parse_runner_output(stdout: str) -> dict:
    metrics: dict[str, object] = {
        "frames_processed": 0,
    }
    frame_match = re.search(r"processed\s+(\d+)\s+frames", stdout)
    if frame_match:
        metrics["frames_processed"] = int(frame_match.group(1))
    pose_match = re.search(r"recorded\s+(\d+)\s+poses", stdout)
    if pose_match:
        metrics["poses_recorded"] = int(pose_match.group(1))
    init_match = re.findall(r"init=(true|false)", stdout)
    if init_match:
        metrics["final_initialized"] = init_match[-1] == "true"
    mode_match = re.findall(r"mode=([A-Za-z]+)", stdout)
    if mode_match:
        metrics["final_mode"] = mode_match[-1]
    return metrics


def evaluate_trajectory(est_path: Path, gt_csv: Path) -> dict:
    ts_gt, pos_gt, quat_gt = read_csv_gt(str(gt_csv))
    ts_est, pos_est, _quat_est = read_est(str(est_path))
    if len(pos_est) < 2:
        return {"ok": False, "error": "too few estimated poses"}

    pairs = []
    gt_index = 0
    for est_index, timestamp_ns in enumerate(ts_est):
        while gt_index + 1 < len(ts_gt) and abs(ts_gt[gt_index + 1] - timestamp_ns) <= abs(ts_gt[gt_index] - timestamp_ns):
            gt_index += 1
        if abs(ts_gt[gt_index] - timestamp_ns) <= 50_000_000:
            pairs.append((est_index, gt_index))

    if len(pairs) < 2:
        return {"ok": False, "error": "insufficient GT associations"}

    ate_errors, seg_rmse, scale, seg_info = compute_ate(pos_est, pos_gt, pairs, ts_gt=ts_gt)
    return {
        "ok": True,
        "matched_poses": len(pairs),
        "ate_rmse": float((ate_errors**2).mean() ** 0.5),
        "ate_mean": float(ate_errors.mean()),
        "ate_max": float(ate_errors.max()),
        "scale": float(scale),
        "segment_rmse": seg_rmse,
        "segment_info": seg_info,
    }


def evaluate_sequence(sequence: str, data_base: Path, config_path: Path, timeout_s: int, no_imu: bool, binary_override: str | None) -> dict:
    seq_path = dataset_path(data_base, sequence)
    gt_csv = gt_path(seq_path)
    if not seq_path.exists():
        return {"sequence": sequence, "ok": False, "error": f"dataset not found: {seq_path}"}
    if not gt_csv.exists():
        return {"sequence": sequence, "ok": False, "error": f"ground truth not found: {gt_csv}"}

    run_result = run_msckf(seq_path, config_path, timeout_s, no_imu, binary_override)
    if not run_result.get("ok"):
        run_result["sequence"] = sequence
        return run_result

    eval_result = evaluate_trajectory(Path(run_result["trajectory_path"]), gt_csv)
    return {
        "sequence": sequence,
        "run": run_result,
        "evaluation": eval_result,
        "ok": bool(eval_result.get("ok")),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate the MSCKF TUM-VI runner")
    parser.add_argument("--sequence", choices=DEFAULT_SEQUENCES, help="Single sequence to evaluate")
    parser.add_argument("--all", action="store_true", help="Evaluate all default sequences")
    parser.add_argument("--data-base", default=str(DEFAULT_DATA_BASE), help="Base path containing TUM-VI datasets")
    parser.add_argument("--config", default="config/tum_vi.yaml", help="Config file path")
    parser.add_argument("--timeout", type=int, default=DEFAULT_TIMEOUT_S, help="Runner timeout in seconds")
    parser.add_argument("--json", dest="json_path", help="Write JSON summary to this path")
    parser.add_argument("--no-imu", action="store_true", help="Run the MSCKF runner without IMU samples")
    parser.add_argument("--binary", dest="binary_path", help="Use an existing run_tum_msckf binary instead of building")
    args = parser.parse_args()

    sequences = DEFAULT_SEQUENCES if args.all or not args.sequence else [args.sequence]
    if args.sequence and not args.all:
        sequences = [args.sequence]

    config_path = (PROJECT_ROOT / args.config).resolve()
    data_base = Path(args.data_base).resolve()
    results = [
        evaluate_sequence(sequence, data_base, config_path, args.timeout, args.no_imu, args.binary_path)
        for sequence in sequences
    ]

    for result in results:
        sequence = result["sequence"]
        if not result.get("ok"):
            print(f"{sequence}: FAIL - {result.get('error', result.get('evaluation', {}).get('error', 'unknown error'))}")
            continue

        run = result["run"]
        evaluation = result["evaluation"]
        print(
            f"{sequence}: OK  elapsed={run['elapsed_s']:.1f}s  poses={run.get('poses_recorded', 0)}  "
            f"ATE_RMSE={evaluation['ate_rmse']:.3f}m  scale={evaluation['scale']:.4f}"
        )

    if args.json_path:
        output_path = Path(args.json_path)
        output_path.write_text(json.dumps(results, indent=2))
        print(f"wrote JSON summary to {output_path}")

    return 0 if all(result.get("ok") for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())