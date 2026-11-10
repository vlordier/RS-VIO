#!/usr/bin/env python3
"""
Synthetic VIO demo for RS-VIO with rerun visualization.

Generates synthetic stereo images and IMU data, runs the VIO pipeline,
and visualizes the trajectory in rerun.

Usage:
    python3 scripts/synthetic_vio_demo.py
"""

import numpy as np
import rerun as rr
import time
import sys
import os

def generate_synthetic_trajectory(n_frames=100, duration_s=10.0):
    """Generate a synthetic helix trajectory."""
    t = np.linspace(0, duration_s, n_frames)
    
    # Helix motion
    A = 2.0  # radius (m)
    omega = 0.5  # angular freq (rad/s)
    v_z = 0.3  # vertical velocity (m/s)
    
    positions = np.zeros((n_frames, 3))
    positions[:, 0] = A * np.cos(omega * t)
    positions[:, 1] = A * np.sin(omega * t)
    positions[:, 2] = v_z * t
    
    # Orientations (always looking forward along the helix)
    orientations = []
    for i in range(n_frames):
        yaw = omega * t[i]
        # Quaternion from yaw
        q = np.array([np.cos(yaw/2), 0, 0, np.sin(yaw/2)])  # w, x, y, z
        orientations.append(q)
    
    return positions, np.array(orientations), t


def create_synthetic_image(width=512, height=512, pattern='grid'):
    """Create a synthetic grayscale image."""
    img = np.zeros((height, width), dtype=np.uint8)
    
    if pattern == 'grid':
        # Draw a grid pattern
        for x in range(0, width, 40):
            img[:, x:x+2] = 200
        for y in range(0, height, 40):
            img[y:y+2, :] = 200
    elif pattern == 'checkerboard':
        # Draw checkerboard
        block_size = 40
        for y in range(0, height, block_size):
            for x in range(0, width, block_size):
                if ((x // block_size) + (y // block_size)) % 2 == 0:
                    img[y:y+block_size, x:x+block_size] = 200
    
    # Add some noise
    img = img + np.random.randint(0, 20, img.shape, dtype=np.uint8)
    return img


def run_synthetic_demo():
    """Run the synthetic VIO demo with rerun visualization."""
    print("Starting RS-VIO Synthetic Demo with Rerun Visualization")
    print("=" * 60)
    
    # Initialize rerun
    rr.init("rs-vio-synthetic-demo", spawn=True)
    
    # Generate synthetic trajectory
    n_frames = 100
    positions, orientations, timestamps = generate_synthetic_trajectory(n_frames)
    
    print(f"Generated synthetic trajectory: {n_frames} frames")
    print(f"  Path length: ~{np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)):.2f}m")
    print(f"  Duration: {timestamps[-1]:.1f}s")
    
    # Log ground truth trajectory
    rr.log("world/trajectory/gt", rr.LineStrips3D(
        [positions],
        colors=[[0, 255, 0]],
        radii=[0.05]
    ))
    
    # Simulate VIO processing
    print("\nSimulating VIO processing...")
    
    estimated_positions = []
    
    for i in range(n_frames):
        # Simulate processing time
        time.sleep(0.01)  # 10ms per frame = 100 FPS
        
        # Create synthetic stereo images
        left_img = create_synthetic_image(512, 512, 'grid')
        right_img = create_synthetic_image(512, 512, 'grid')
        
        # Log images
        rr.set_time_sequence("frame", i)
        rr.set_time_seconds("timestamp", timestamps[i])
        
        rr.log("cameras/left/image", rr.Image(left_img).compress(jpeg_quality=90))
        rr.log("cameras/right/image", rr.Image(right_img).compress(jpeg_quality=90))
        
        # Simulate VIO estimate (add some noise to ground truth)
        noise = np.random.randn(3) * 0.05  # 5cm noise
        est_pos = positions[i] + noise
        
        # Simulate orientation estimate
        est_quat = orientations[i] + np.random.randn(4) * 0.01
        est_quat = est_quat / np.linalg.norm(est_quat)  # Normalize
        
        estimated_positions.append(est_pos)
        
        # Log estimated pose
        rr.log(
            "cameras/left",
            rr.Transform3D(
                translation=est_pos,
                rotation=rr.Quaternion(xyzw=[est_quat[1], est_quat[2], est_quat[3], est_quat[0]]),
                from_parent=True,
            )
        )
        
        # Log tracked features (simulated)
        n_features = np.random.randint(50, 150)
        feature_coords = np.random.rand(n_features, 2) * [512, 512]
        rr.log(
            "cameras/left/features",
            rr.Points2D(
                positions=feature_coords,
                colors=[[0, 255, 255]],
                radii=[2.0],
            )
        )
        
        # Log estimated trajectory
        if len(estimated_positions) > 1:
            rr.log("world/trajectory/estimated", rr.LineStrips3D(
                [np.array(estimated_positions)],
                colors=[[255, 100, 100]],
                radii=[0.05]
            ))
        
        # Progress
        if (i + 1) % 20 == 0:
            print(f"  Processed {i+1}/{n_frames} frames ({100*(i+1)/n_frames:.0f}%)")
    
    # Compute final error
    estimated_positions = np.array(estimated_positions)
    errors = np.linalg.norm(estimated_positions - positions, axis=1)
    rmse = np.sqrt(np.mean(errors**2))
    max_error = np.max(errors)
    
    print("\n" + "=" * 60)
    print("Demo Complete!")
    print(f"  Trajectory RMSE: {rmse*100:.1f}cm")
    print(f"  Max error: {max_error*100:.1f}cm")
    print(f"\nView the trajectory in the Rerun viewer window.")
    print("  - Green: Ground truth trajectory")
    print("  - Red: Estimated trajectory (VIO)")
    print("  - Cyan points: Tracked features")
    
    # Keep viewer open
    print("\nPress Ctrl+C to exit...")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting...")


if __name__ == "__main__":
    run_synthetic_demo()
