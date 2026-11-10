#!/usr/bin/env python3
"""
Diagnose pathologies in an estimated trajectory.

Detects:
  - Non-physical velocity jumps (BA divergence)
  - Large timestamp gaps (system stalls)
  - Position resets (system re-initialization, loss of track)
  - Unreasonable rotation rates
  - NaN/Inf values

Usage:
    python3 scripts/diagnose_trajectory.py <trajectory.txt>
"""

import sys
import numpy as np
from scipy.spatial.transform import Rotation

MAX_VELOCITY = 5.0         # m/s — anything above is suspicious for hand-held
MAX_ANGULAR = 500.0        # deg/s — > 8 rad/s is extreme
MAX_FRAME_JUMP = 1.0       # m — single-frame jump
MAX_GAP_S = 0.5            # s — timestamp gap indicating stall
RESET_DIST = 5.0           # m — position drop that looks like reset to origin


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <trajectory.txt>", file=sys.stderr)
        sys.exit(1)

    ts, pos, quat = [], [], []
    with open(sys.argv[1]) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line.split()
            if len(p) < 8:
                continue
            ts.append(float(p[0]))
            pos.append([float(x) for x in p[1:4]])
            quat.append([float(p[4]), float(p[5]), float(p[6]), float(p[7])])

    ts = np.array(ts)
    pos = np.array(pos)
    quat = np.array(quat)
    n = len(ts)
    print(f"Poses: {n}")

    # NaN/Inf
    nan_count = int(np.isnan(pos).any(axis=1).sum() + np.isnan(quat).any(axis=1).sum())
    inf_count = int(np.isinf(pos).any(axis=1).sum() + np.isinf(quat).any(axis=1).sum())
    print(f"NaN poses: {nan_count}, Inf poses: {inf_count}")

    if n < 2:
        return

    dts = np.diff(ts)
    diffs = pos[1:] - pos[:-1]
    dists = np.linalg.norm(diffs, axis=1)
    vels = dists / np.maximum(dts, 1e-6)

    # Rotation rate
    rots = Rotation.from_quat(quat).as_matrix()
    rel_R = np.einsum('nij,njk->nik', rots[:-1].transpose(0, 2, 1), rots[1:])
    cos_theta = np.clip((np.einsum('nii->n', rel_R) - 1) / 2, -1, 1)
    angles = np.degrees(np.arccos(cos_theta))
    ang_rates = angles / np.maximum(dts, 1e-6)

    print(f"\n=== Velocity ===")
    print(f"  median={np.median(vels):.2f}  p95={np.percentile(vels, 95):.2f}  p99={np.percentile(vels, 99):.2f}  max={vels.max():.2f} m/s")
    print(f"  > {MAX_VELOCITY} m/s:  {(vels > MAX_VELOCITY).sum()} frames")
    print(f"  > 50 m/s (impossible): {(vels > 50).sum()} frames")

    print(f"\n=== Angular rate ===")
    print(f"  median={np.median(ang_rates):.1f}  p95={np.percentile(ang_rates, 95):.1f}  max={ang_rates.max():.1f} deg/s")
    print(f"  > {MAX_ANGULAR} deg/s: {(ang_rates > MAX_ANGULAR).sum()} frames")

    print(f"\n=== Frame gaps ===")
    print(f"  median={np.median(dts)*1000:.1f}ms  max={dts.max()*1000:.1f}ms")
    print(f"  > {MAX_GAP_S*1000:.0f}ms: {(dts > MAX_GAP_S).sum()} gaps")

    # Detect position resets: near-zero position after large position
    print(f"\n=== Position resets ===")
    prev_dist_from_origin = np.linalg.norm(pos[:-1], axis=1)
    cur_dist_from_origin = np.linalg.norm(pos[1:], axis=1)
    resets = (prev_dist_from_origin > RESET_DIST) & (cur_dist_from_origin < 0.5)
    reset_idx = np.where(resets)[0]
    print(f"  Detected {len(reset_idx)} resets (prev > {RESET_DIST}m, new < 0.5m)")
    for i in reset_idx[:10]:
        print(f"    frame {i}: t={ts[i]:.2f}s, pos={pos[i]} -> {pos[i+1]} (dt={dts[i]*1000:.0f}ms)")

    # Top 10 worst velocity spikes
    print(f"\n=== Top velocity spikes ===")
    worst = np.argsort(vels)[::-1][:10]
    for i in worst:
        print(f"  frame {i}: v={vels[i]:.1f} m/s, dt={dts[i]*1000:.0f}ms, jump={dists[i]:.2f}m")

    # Summary diagnosis
    print(f"\n=== Diagnosis ===")
    issues = []
    if nan_count + inf_count > 0:
        issues.append(f"NUMERICAL: {nan_count + inf_count} NaN/Inf poses")
    if (vels > 50).sum() > 0:
        issues.append(f"BA DIVERGENCE: {(vels > 50).sum()} non-physical velocities")
    if len(reset_idx) > 0:
        issues.append(f"SYSTEM RESET: {len(reset_idx)} trajectory resets")
    if (dts > MAX_GAP_S).sum() > 0:
        issues.append(f"STALLS: {(dts > MAX_GAP_S).sum()} frame gaps > {MAX_GAP_S}s")
    if (vels > MAX_VELOCITY).sum() > 20:
        issues.append(f"HIGH-VELOCITY outliers: {(vels > MAX_VELOCITY).sum()} frames > {MAX_VELOCITY} m/s")

    if issues:
        for i in issues:
            print(f"  [!] {i}")
    else:
        print(f"  [OK] no major issues detected")


if __name__ == '__main__':
    main()
