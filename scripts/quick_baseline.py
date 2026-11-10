#!/usr/bin/env python3
"""Quick baseline evaluation on all 4 TUM-VI sequences (single run each)."""

import subprocess
import re
from pathlib import Path

SEQUENCES = ["slides1", "corridor1", "room1", "corridor2"]
DATA_BASE = Path("/Users/vincent/data/tum-vi")
PROJECT_ROOT = Path("/Users/vincent/Work/RS-VIO")

def run_vio(sequence):
    """Run VIO and return path to trajectory file."""
    dataset = DATA_BASE / f"dataset-{sequence}_512_16"
    config = PROJECT_ROOT / "config" / "tum_vi.yaml"
    binary = PROJECT_ROOT / "target" / "release" / "run_tum"
    traj = dataset / "estimated_trajectory.txt"

    if traj.exists():
        traj.unlink()

    result = subprocess.run(
        [str(binary), str(config), str(dataset)],
        timeout=600,
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        return None
    return traj if traj.exists() else None


def evaluate(traj_path, gt_path):
    """Evaluate trajectory."""
    result = subprocess.run(
        ["python3", str(PROJECT_ROOT / "scripts" / "eval_ate_rpe.py"),
         "--est", str(traj_path),
         "--gt", str(gt_path)],
        timeout=30,
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        return None, None

    output = result.stdout

    # Extract ATE RMSE
    match = re.search(r"RMSE\s*=\s*([\d.]+)", output)
    ate = float(match.group(1)) if match else None

    # Extract scale percentage
    match = re.search(r"Sim\(3\) scale:.*\(([-\d.]+)%\)", output)
    scale = float(match.group(1)) if match else None

    return ate, scale


def main():
    print("=" * 80)
    print("RS-VIO Baseline (commit 70bafec)")
    print("=" * 80)
    print()

    for seq in SEQUENCES:
        dataset = DATA_BASE / f"dataset-{seq}_512_16"
        gt = dataset / "mav0" / "mocap0" / "data.csv"

        if not dataset.exists() or not gt.exists():
            print(f"{seq:12s} ✗ Dataset not found")
            continue

        print(f"{seq:12s} ", end="", flush=True)

        # Run
        traj = run_vio(seq)
        if not traj:
            print("✗ VIO failed")
            continue

        print("→ ", end="", flush=True)

        # Evaluate
        ate, scale = evaluate(traj, gt)
        if ate is None:
            print("✗ Eval failed")
            continue

        print(f"ATE={ate:.3f}m  scale={scale:+.2f}%")

    print()


if __name__ == "__main__":
    main()
