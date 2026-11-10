#!/usr/bin/env python3
"""
Analyze trajectory jumps in slides1 to understand failure modes.
"""

import numpy as np
from pathlib import Path


def read_trajectory(filepath):
    """Read estimated trajectory (timestamp, x, y, z format)."""
    times, poses = [], []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            times.append(float(parts[0]) / 1e9)  # ns to seconds
            poses.append(np.array([float(parts[1]), float(parts[2]), float(parts[3])]))
    return np.array(times), np.array(poses)


def analyze_jumps(times, poses, threshold=0.5):
    """Identify frame-to-frame jumps above threshold."""
    jumps = []
    for i in range(1, len(poses)):
        disp = np.linalg.norm(poses[i] - poses[i-1])
        if disp > threshold:
            vel_implied = disp / (times[i] - times[i-1])
            jumps.append({
                'time': times[i],
                'disp': disp,
                'vel_implied': vel_implied,
                'frame': i
            })
    return jumps


def analyze_segments(times, poses):
    """Analyze trajectory by time segments."""
    # Extract GT segment boundaries from known sequences
    # slides1 has 2 GT segments: ~0-58s and ~170-224s
    # But total is 279s, so there's likely missing GT in middle

    print("Time-based trajectory analysis:")
    print(f"  Total duration: {times[-1]:.1f}s")
    print(f"  Total frames: {len(times)}")
    print(f"  Frame rate: {len(times) / times[-1]:.1f} Hz")

    # Velocity statistics
    velocities = []
    for i in range(1, len(poses)):
        v = np.linalg.norm(poses[i] - poses[i-1]) / (times[i] - times[i-1])
        velocities.append(v)

    velocities = np.array(velocities)
    print(f"\nVelocity statistics:")
    print(f"  Mean: {np.mean(velocities):.2f} m/s")
    print(f"  Std: {np.std(velocities):.2f} m/s")
    print(f"  Min: {np.min(velocities):.2f} m/s")
    print(f"  Max: {np.max(velocities):.2f} m/s")
    print(f"  p99: {np.percentile(velocities, 99):.2f} m/s")

    # Identify high-velocity periods
    high_vel_threshold = np.mean(velocities) + 3 * np.std(velocities)
    high_vel_indices = np.where(velocities > high_vel_threshold)[0]

    if len(high_vel_indices) > 0:
        print(f"\nHigh-velocity periods (>{high_vel_threshold:.2f} m/s):")
        # Cluster into contiguous regions
        clusters = []
        current = [high_vel_indices[0]]
        for idx in high_vel_indices[1:]:
            if idx == current[-1] + 1:
                current.append(idx)
            else:
                clusters.append(current)
                current = [idx]
        clusters.append(current)

        for cluster in clusters:
            start_time = times[cluster[0]]
            end_time = times[cluster[-1]]
            print(f"  t={start_time:.1f}-{end_time:.1f}s: {len(cluster)} high-velocity frames")


def main():
    est_path = Path("/Users/vincent/data/tum-vi/dataset-slides1_512_16/estimated_trajectory.txt")

    if not est_path.exists():
        print(f"Trajectory file not found: {est_path}")
        return

    print("Analyzing slides1 trajectory jumps...")
    print()

    times, poses = read_trajectory(str(est_path))
    print(f"Loaded {len(times)} poses")
    print()

    # Analyze jumps
    jumps = analyze_jumps(times, poses, threshold=0.5)
    print(f"Frame-to-frame jumps > 0.5m: {len(jumps)}")
    if jumps:
        print("\nTop 10 largest jumps:")
        sorted_jumps = sorted(jumps, key=lambda j: j['disp'], reverse=True)[:10]
        for i, j in enumerate(sorted_jumps, 1):
            print(f"  {i}. t={j['time']:.1f}s frame={j['frame']}: "
                  f"disp={j['disp']:.3f}m, implied_v={j['vel_implied']:.1f}m/s")

    print()
    analyze_segments(times, poses)


if __name__ == "__main__":
    main()
