#!/usr/bin/env python3
"""
Visualization script for VIO optimization comparisons
Plots tracking, disparity, and rolling shutter comparisons
"""

import matplotlib.pyplot as plt
import pandas as pd

# Set style
plt.style.use('seaborn-v0_8-darkgrid')

def plot_tracking_comparison(csv_file):
    """Plot IMU-aided tracking vs baseline"""
    df = pd.read_csv(csv_file)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('IMU-Aided Tracking vs Baseline KLT', fontsize=16, fontweight='bold')

    # Feature count comparison
    axes[0, 0].plot(df['frame'], df['baseline_count'], label='Baseline (no IMU)', linewidth=2, alpha=0.7)
    axes[0, 0].plot(df['frame'], df['imu_aided_count'], label='IMU-Aided', linewidth=2, alpha=0.7)
    axes[0, 0].set_xlabel('Frame')
    axes[0, 0].set_ylabel('Feature Count')
    axes[0, 0].set_title('Tracked Features Over Time')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # Track length comparison
    axes[0, 1].plot(df['frame'], df['baseline_avg_length'], label='Baseline', linewidth=2, alpha=0.7)
    axes[0, 1].plot(df['frame'], df['imu_aided_avg_length'], label='IMU-Aided', linewidth=2, alpha=0.7)
    axes[0, 1].set_xlabel('Frame')
    axes[0, 1].set_ylabel('Average Track Length')
    axes[0, 1].set_title('Feature Track Persistence')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # Improvement ratio
    improvement = (df['imu_aided_count'] - df['baseline_count']) / df['baseline_count'] * 100
    axes[1, 0].plot(df['frame'], improvement, linewidth=2, color='green')
    axes[1, 0].axhline(y=0, color='r', linestyle='--', alpha=0.5)
    axes[1, 0].set_xlabel('Frame')
    axes[1, 0].set_ylabel('Improvement (%)')
    axes[1, 0].set_title('Tracking Improvement with IMU')
    axes[1, 0].grid(True, alpha=0.3)

    # Prediction error
    axes[1, 1].plot(df['frame'], df['prediction_error'], linewidth=2, color='orange')
    axes[1, 1].set_xlabel('Frame')
    axes[1, 1].set_ylabel('Prediction Error (px)')
    axes[1, 1].set_title('IMU Motion Prediction Accuracy')
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('./plot_output/tracking_comparison.png', dpi=300, bbox_inches='tight')
    print("Saved: ./plot_output/tracking_comparison.png")
    plt.show()

def plot_disparity_comparison(csv_file):
    """Plot sub-pixel disparity vs integer"""
    df = pd.read_csv(csv_file)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Sub-Pixel Disparity Refinement', fontsize=16, fontweight='bold')

    # Disparity comparison
    axes[0, 0].scatter(df['int_disp'], df['subpix_disp'], alpha=0.5, s=30)
    axes[0, 0].plot([df['int_disp'].min(), df['int_disp'].max()],
                    [df['int_disp'].min(), df['int_disp'].max()],
                    'r--', label='y=x', alpha=0.5)
    axes[0, 0].set_xlabel('Integer Disparity (px)')
    axes[0, 0].set_ylabel('Sub-pixel Disparity (px)')
    axes[0, 0].set_title('Disparity Refinement')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # Depth comparison
    axes[0, 1].scatter(df['int_depth'], df['subpix_depth'], alpha=0.5, s=30)
    axes[0, 1].plot([df['int_depth'].min(), df['int_depth'].max()],
                    [df['int_depth'].min(), df['int_depth'].max()],
                    'r--', alpha=0.5)
    axes[0, 1].set_xlabel('Integer Depth (m)')
    axes[0, 1].set_ylabel('Sub-pixel Depth (m)')
    axes[0, 1].set_title('Depth Accuracy Improvement')
    axes[0, 1].grid(True, alpha=0.3)

    # Error reduction
    axes[1, 0].hist([df['int_error'], df['subpix_error']],
                    bins=30, label=['Integer', 'Sub-pixel'], alpha=0.7)
    axes[1, 0].set_xlabel('Photometric Error (px)')
    axes[1, 0].set_ylabel('Count')
    axes[1, 0].set_title('Error Distribution')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # Sub-pixel offset histogram
    offset = df['subpix_disp'] - df['int_disp']
    axes[1, 1].hist(offset, bins=50, alpha=0.7, color='green')
    axes[1, 1].axvline(x=0, color='r', linestyle='--', alpha=0.5)
    axes[1, 1].set_xlabel('Sub-pixel Offset (px)')
    axes[1, 1].set_ylabel('Count')
    axes[1, 1].set_title('Sub-pixel Correction Distribution')
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('./plot_output/disparity_comparison.png', dpi=300, bbox_inches='tight')
    print("Saved: ./plot_output/disparity_comparison.png")
    plt.show()

def plot_rolling_shutter_comparison(csv_file):
    """Plot rolling shutter correction benefit"""
    df = pd.read_csv(csv_file)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Rolling Shutter Correction', fontsize=16, fontweight='bold')

    # Error vs angular velocity (GS)
    axes[0, 0].scatter(df['angular_velocity'], df['gs_error'],
                      alpha=0.5, s=30, label='Global Shutter', color='red')
    axes[0, 0].set_xlabel('Angular Velocity (rad/s)')
    axes[0, 0].set_ylabel('Reprojection Error (px)')
    axes[0, 0].set_title('Error vs Rotation (No RS Correction)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # Error vs angular velocity (RS)
    axes[0, 1].scatter(df['angular_velocity'], df['rs_error'],
                      alpha=0.5, s=30, label='RS Corrected', color='green')
    axes[0, 1].set_xlabel('Angular Velocity (rad/s)')
    axes[0, 1].set_ylabel('Reprojection Error (px)')
    axes[0, 1].set_title('Error vs Rotation (With RS Correction)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # Direct comparison
    axes[1, 0].plot(df['frame'], df['gs_error'], label='Global Shutter', linewidth=2, alpha=0.7)
    axes[1, 0].plot(df['frame'], df['rs_error'], label='RS Corrected', linewidth=2, alpha=0.7)
    axes[1, 0].set_xlabel('Frame')
    axes[1, 0].set_ylabel('Reprojection Error (px)')
    axes[1, 0].set_title('Error Reduction with RS Correction')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # Error reduction
    reduction = (df['gs_error'] - df['rs_error']) / df['gs_error'] * 100
    axes[1, 1].plot(df['frame'], reduction, linewidth=2, color='purple')
    axes[1, 1].axhline(y=0, color='r', linestyle='--', alpha=0.5)
    axes[1, 1].set_xlabel('Frame')
    axes[1, 1].set_ylabel('Error Reduction (%)')
    axes[1, 1].set_title('RS Correction Benefit')
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('./plot_output/rolling_shutter_comparison.png', dpi=300, bbox_inches='tight')
    print("Saved: ./plot_output/rolling_shutter_comparison.png")
    plt.show()

if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1:
        data_dir = sys.argv[1]
    else:
        data_dir = './plot_output'

    # Plot all comparisons
    try:
        plot_tracking_comparison(f'{data_dir}/tracking_comparison.csv')
    except Exception as e:
        print(f"Error plotting tracking: {e}")

    try:
        plot_disparity_comparison(f'{data_dir}/disparity_comparison.csv')
    except Exception as e:
        print(f"Error plotting disparity: {e}")

    try:
        plot_rolling_shutter_comparison(f'{data_dir}/rolling_shutter_comparison.csv')
    except Exception as e:
        print(f"Error plotting rolling shutter: {e}")
