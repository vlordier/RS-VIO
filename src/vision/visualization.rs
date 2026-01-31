/// Visualization and plotting utilities for comparing VIO optimizations
///
/// Provides tools to plot comparisons:
/// - IMU-aided tracking vs plain KLT
/// - Sub-pixel disparity vs integer disparity
/// - Rolling shutter correction vs global shutter
use std::fs::File;
use std::io::Write;

/// Tracking comparison data
#[derive(Clone, Debug)]
pub struct TrackingComparison {
    /// Frame indices
    pub frames: Vec<usize>,
    /// Number of tracked features (baseline - no IMU)
    pub baseline_count: Vec<usize>,
    /// Number of tracked features (with IMU)
    pub imu_aided_count: Vec<usize>,
    /// Average track length (baseline)
    pub baseline_avg_length: Vec<f64>,
    /// Average track length (with IMU)
    pub imu_aided_avg_length: Vec<f64>,
    /// Prediction error (pixels)
    pub prediction_errors: Vec<f64>,
}

impl TrackingComparison {
    pub fn new() -> Self {
        Self {
            frames: Vec::new(),
            baseline_count: Vec::new(),
            imu_aided_count: Vec::new(),
            baseline_avg_length: Vec::new(),
            imu_aided_avg_length: Vec::new(),
            prediction_errors: Vec::new(),
        }
    }

    /// Add frame data
    pub fn add_frame(
        &mut self,
        frame: usize,
        baseline: usize,
        imu_aided: usize,
        baseline_len: f64,
        imu_len: f64,
        pred_error: f64,
    ) {
        self.frames.push(frame);
        self.baseline_count.push(baseline);
        self.imu_aided_count.push(imu_aided);
        self.baseline_avg_length.push(baseline_len);
        self.imu_aided_avg_length.push(imu_len);
        self.prediction_errors.push(pred_error);
    }

    /// Export to CSV for plotting
    pub fn export_csv(&self, filename: &str) -> std::io::Result<()> {
        let mut file = File::create(filename)?;

        writeln!(file, "frame,baseline_count,imu_aided_count,baseline_avg_length,imu_aided_avg_length,prediction_error")?;

        for i in 0..self.frames.len() {
            writeln!(
                file,
                "{},{},{},{},{},{}",
                self.frames[i],
                self.baseline_count[i],
                self.imu_aided_count[i],
                self.baseline_avg_length[i],
                self.imu_aided_avg_length[i],
                self.prediction_errors[i]
            )?;
        }

        Ok(())
    }

    /// Compute improvement statistics
    pub fn compute_stats(&self) -> TrackingStats {
        let avg_baseline =
            self.baseline_count.iter().sum::<usize>() as f64 / self.baseline_count.len() as f64;
        let avg_imu =
            self.imu_aided_count.iter().sum::<usize>() as f64 / self.imu_aided_count.len() as f64;

        let improvement = (avg_imu - avg_baseline) / avg_baseline * 100.0;

        let avg_pred_error =
            self.prediction_errors.iter().sum::<f64>() / self.prediction_errors.len() as f64;

        TrackingStats {
            avg_baseline_features: avg_baseline,
            avg_imu_aided_features: avg_imu,
            improvement_percent: improvement,
            avg_prediction_error_px: avg_pred_error,
        }
    }
}

#[derive(Clone, Debug)]
pub struct TrackingStats {
    pub avg_baseline_features: f64,
    pub avg_imu_aided_features: f64,
    pub improvement_percent: f64,
    pub avg_prediction_error_px: f64,
}

/// Disparity comparison data
#[derive(Clone, Debug)]
pub struct DisparityComparison {
    /// Feature indices
    pub features: Vec<usize>,
    /// Integer disparity values
    pub integer_disparity: Vec<f64>,
    /// Sub-pixel disparity values
    pub subpixel_disparity: Vec<f64>,
    /// Depth from integer disparity (meters)
    pub integer_depth: Vec<f64>,
    /// Depth from sub-pixel disparity (meters)
    pub subpixel_depth: Vec<f64>,
    /// Photometric error (integer)
    pub integer_error: Vec<f64>,
    /// Photometric error (sub-pixel)
    pub subpixel_error: Vec<f64>,
}

impl DisparityComparison {
    pub fn new() -> Self {
        Self {
            features: Vec::new(),
            integer_disparity: Vec::new(),
            subpixel_disparity: Vec::new(),
            integer_depth: Vec::new(),
            subpixel_depth: Vec::new(),
            integer_error: Vec::new(),
            subpixel_error: Vec::new(),
        }
    }

    /// Add feature comparison
    pub fn add_feature(
        &mut self,
        id: usize,
        int_disp: f64,
        sub_disp: f64,
        int_depth: f64,
        sub_depth: f64,
        int_err: f64,
        sub_err: f64,
    ) {
        self.features.push(id);
        self.integer_disparity.push(int_disp);
        self.subpixel_disparity.push(sub_disp);
        self.integer_depth.push(int_depth);
        self.subpixel_depth.push(sub_depth);
        self.integer_error.push(int_err);
        self.subpixel_error.push(sub_err);
    }

    /// Export to CSV
    pub fn export_csv(&self, filename: &str) -> std::io::Result<()> {
        let mut file = File::create(filename)?;

        writeln!(
            file,
            "feature,int_disp,subpix_disp,int_depth,subpix_depth,int_error,subpix_error"
        )?;

        for i in 0..self.features.len() {
            writeln!(
                file,
                "{},{},{},{},{},{},{}",
                self.features[i],
                self.integer_disparity[i],
                self.subpixel_disparity[i],
                self.integer_depth[i],
                self.subpixel_depth[i],
                self.integer_error[i],
                self.subpixel_error[i]
            )?;
        }

        Ok(())
    }

    /// Compute depth improvement
    pub fn compute_stats(&self) -> DisparityStats {
        let depth_diffs: Vec<f64> = self
            .integer_depth
            .iter()
            .zip(self.subpixel_depth.iter())
            .map(|(i, s)| (i - s).abs())
            .collect();

        let avg_depth_diff = depth_diffs.iter().sum::<f64>() / depth_diffs.len() as f64;

        let avg_int_error =
            self.integer_error.iter().sum::<f64>() / self.integer_error.len() as f64;
        let avg_sub_error =
            self.subpixel_error.iter().sum::<f64>() / self.subpixel_error.len() as f64;

        let error_reduction = (avg_int_error - avg_sub_error) / avg_int_error * 100.0;

        DisparityStats {
            avg_depth_difference_m: avg_depth_diff,
            avg_integer_error_px: avg_int_error,
            avg_subpixel_error_px: avg_sub_error,
            error_reduction_percent: error_reduction,
        }
    }
}

#[derive(Clone, Debug)]
pub struct DisparityStats {
    pub avg_depth_difference_m: f64,
    pub avg_integer_error_px: f64,
    pub avg_subpixel_error_px: f64,
    pub error_reduction_percent: f64,
}

/// Rolling shutter comparison data
#[derive(Clone, Debug)]
pub struct RollingShutterComparison {
    /// Frame indices
    pub frames: Vec<usize>,
    /// Angular velocity (rad/s)
    pub angular_velocity: Vec<f64>,
    /// Reprojection error without RS correction (pixels)
    pub gs_error: Vec<f64>,
    /// Reprojection error with RS correction (pixels)
    pub rs_error: Vec<f64>,
    /// Number of features
    pub feature_count: Vec<usize>,
}

impl RollingShutterComparison {
    pub fn new() -> Self {
        Self {
            frames: Vec::new(),
            angular_velocity: Vec::new(),
            gs_error: Vec::new(),
            rs_error: Vec::new(),
            feature_count: Vec::new(),
        }
    }

    /// Add frame data
    pub fn add_frame(
        &mut self,
        frame: usize,
        ang_vel: f64,
        gs_err: f64,
        rs_err: f64,
        count: usize,
    ) {
        self.frames.push(frame);
        self.angular_velocity.push(ang_vel);
        self.gs_error.push(gs_err);
        self.rs_error.push(rs_err);
        self.feature_count.push(count);
    }

    /// Export to CSV
    pub fn export_csv(&self, filename: &str) -> std::io::Result<()> {
        let mut file = File::create(filename)?;

        writeln!(
            file,
            "frame,angular_velocity,gs_error,rs_error,feature_count"
        )?;

        for i in 0..self.frames.len() {
            writeln!(
                file,
                "{},{},{},{},{}",
                self.frames[i],
                self.angular_velocity[i],
                self.gs_error[i],
                self.rs_error[i],
                self.feature_count[i]
            )?;
        }

        Ok(())
    }

    /// Compute RS correction benefit
    pub fn compute_stats(&self) -> RollingShutterStats {
        let avg_gs = self.gs_error.iter().sum::<f64>() / self.gs_error.len() as f64;
        let avg_rs = self.rs_error.iter().sum::<f64>() / self.rs_error.len() as f64;

        let error_reduction = (avg_gs - avg_rs) / avg_gs * 100.0;

        // Find correlation with angular velocity
        let high_vel_frames: Vec<_> = self
            .angular_velocity
            .iter()
            .enumerate()
            .filter(|(_, v)| **v > 0.5)
            .map(|(i, _)| i)
            .collect();

        let high_vel_gs = if !high_vel_frames.is_empty() {
            high_vel_frames
                .iter()
                .map(|&i| self.gs_error[i])
                .sum::<f64>()
                / high_vel_frames.len() as f64
        } else {
            0.0
        };

        let high_vel_rs = if !high_vel_frames.is_empty() {
            high_vel_frames
                .iter()
                .map(|&i| self.rs_error[i])
                .sum::<f64>()
                / high_vel_frames.len() as f64
        } else {
            0.0
        };

        RollingShutterStats {
            avg_gs_error_px: avg_gs,
            avg_rs_error_px: avg_rs,
            error_reduction_percent: error_reduction,
            high_velocity_gs_error_px: high_vel_gs,
            high_velocity_rs_error_px: high_vel_rs,
        }
    }
}

#[derive(Clone, Debug)]
pub struct RollingShutterStats {
    pub avg_gs_error_px: f64,
    pub avg_rs_error_px: f64,
    pub error_reduction_percent: f64,
    pub high_velocity_gs_error_px: f64,
    pub high_velocity_rs_error_px: f64,
}

/// Generate Python plotting script
pub fn generate_plot_script(output_dir: &str) -> std::io::Result<()> {
    let script = format!(
        r#"#!/usr/bin/env python3
"""
Visualization script for VIO optimization comparisons
Plots tracking, disparity, and rolling shutter comparisons
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

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
    plt.savefig('{}/tracking_comparison.png', dpi=300, bbox_inches='tight')
    print(f"Saved: {}/tracking_comparison.png")
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
    plt.savefig('{}/disparity_comparison.png', dpi=300, bbox_inches='tight')
    print(f"Saved: {}/disparity_comparison.png")
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
    plt.savefig('{}/rolling_shutter_comparison.png', dpi=300, bbox_inches='tight')
    print(f"Saved: {}/rolling_shutter_comparison.png")
    plt.show()

if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1:
        data_dir = sys.argv[1]
    else:
        data_dir = '{}'

    # Plot all comparisons
    try:
        plot_tracking_comparison(f'{{data_dir}}/tracking_comparison.csv')
    except Exception as e:
        print(f"Error plotting tracking: {{e}}")

    try:
        plot_disparity_comparison(f'{{data_dir}}/disparity_comparison.csv')
    except Exception as e:
        print(f"Error plotting disparity: {{e}}")

    try:
        plot_rolling_shutter_comparison(f'{{data_dir}}/rolling_shutter_comparison.csv')
    except Exception as e:
        print(f"Error plotting rolling shutter: {{e}}")
"#,
        output_dir, output_dir, output_dir, output_dir, output_dir, output_dir, output_dir
    );

    let mut file = File::create(format!("{}/plot_comparisons.py", output_dir))?;
    file.write_all(script.as_bytes())?;

    // Make executable on Unix
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mut perms =
            std::fs::metadata(format!("{}/plot_comparisons.py", output_dir))?.permissions();
        perms.set_mode(0o755);
        std::fs::set_permissions(format!("{}/plot_comparisons.py", output_dir), perms)?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tracking_comparison() {
        let mut comp = TrackingComparison::new();
        comp.add_frame(0, 100, 120, 10.0, 12.0, 1.5);
        comp.add_frame(1, 95, 118, 9.5, 11.8, 1.2);

        let stats = comp.compute_stats();
        assert!(stats.avg_imu_aided_features > stats.avg_baseline_features);
        assert!(stats.improvement_percent > 0.0);
    }

    #[test]
    fn test_disparity_comparison() {
        let mut comp = DisparityComparison::new();
        comp.add_feature(0, 50.0, 50.3, 2.0, 2.01, 1.5, 0.8);
        comp.add_feature(1, 100.0, 100.7, 1.0, 1.007, 2.0, 1.0);

        let stats = comp.compute_stats();
        assert!(stats.avg_subpixel_error_px < stats.avg_integer_error_px);
        assert!(stats.error_reduction_percent > 0.0);
    }

    #[test]
    fn test_rolling_shutter_comparison() {
        let mut comp = RollingShutterComparison::new();
        comp.add_frame(0, 0.1, 1.0, 0.8, 100);
        comp.add_frame(1, 0.8, 3.5, 1.2, 95);

        let stats = comp.compute_stats();
        assert!(stats.avg_rs_error_px < stats.avg_gs_error_px);
    }
}
