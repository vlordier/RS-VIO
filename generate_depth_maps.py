#!/usr/bin/env python3
"""Generate synthetic depth maps for TUM-VI dataset."""

from pathlib import Path

import numpy as np


def generate_constant_depth(width=640, height=480, depth_value=5.0):
    """Constant depth plane (simplest)."""
    return np.full((height, width), depth_value, dtype=np.float32)

def generate_synthetic_depth(width=640, height=480, frame_id=0):
    """Pseudo-3D depth with radial variation."""
    y, x = np.meshgrid(np.arange(height), np.arange(width), indexing='ij')

    # Radial variation (closer in center, farther at edges)
    center_y, center_x = height // 2, width // 2
    r_squared = (x - center_x)**2 + (y - center_y)**2
    max_r_sq = center_x**2 + center_y**2

    # Depth: 2m center, 8m edges
    depth = 2.0 + 6.0 * np.sqrt(r_squared) / np.sqrt(max_r_sq)
    depth = np.clip(depth, 0.5, 50.0).astype(np.float32)

    return depth

if __name__ == '__main__':
    output_dir = Path('./exported_data/depth_maps')
    output_dir.mkdir(exist_ok=True, parents=True)

    num_frames = 2821
    depth_type = 'constant'  # Options: 'constant', 'synthetic'

    print(f"Generating {num_frames} depth maps ({depth_type})...")

    for frame_id in range(num_frames):
        if depth_type == 'constant':
            depth = generate_constant_depth()
        else:
            depth = generate_synthetic_depth(frame_id=frame_id)

        output_file = output_dir / f'frame_{frame_id:06d}.npy'
        np.save(output_file, depth)

        if (frame_id + 1) % 500 == 0:
            print(f"  Generated {frame_id + 1}/{num_frames}")

    print(f"✅ Complete! Generated {num_frames} depth maps")
    print(f"   Location: {output_dir}")

    # Verify
    total_size = sum(f.stat().st_size for f in output_dir.glob('*.npy')) / 1e6
    print(f"   Total size: {total_size:.1f} MB")
