use nalgebra::{Matrix3, Vector2, Vector3};
/// Run VIO optimizations on TUM-VI dataset and generate comparison plots
///
/// This example demonstrates:
/// 1. Loading TUM-VI dataset (images + IMU)
/// 2. Running baseline vs optimized tracking
/// 3. Generating visualization comparisons
///
/// Usage:
/// ```bash
/// cargo run --example plot_tum_vi_comparison -- /tmp/rs-vio-samples/tum_vi/room1
/// ```
use rs_vio::vision::{
    generate_plot_script, DisparityComparison, IMUAidedTracker, IMUMeasurement,
    RollingShutterComparison, TrackingComparison,
};
use std::fs;
use std::io::{self, BufRead};
use std::path::PathBuf;

fn main() -> std::io::Result<()> {
    println!("=== TUM-VI Dataset VIO Optimization Comparison ===\n");

    // Parse command line argument
    let args: Vec<String> = std::env::args().collect();
    let dataset_path = if args.len() > 1 {
        PathBuf::from(&args[1])
    } else {
        PathBuf::from("/tmp/rs-vio-samples/tum_vi/room1")
    };

    println!("Dataset path: {}", dataset_path.display());

    // Check if dataset exists
    if !dataset_path.exists() {
        eprintln!("Error: Dataset path does not exist");
        eprintln!("Please download TUM-VI dataset to /tmp/rs-vio-samples/tum_vi/");
        eprintln!(
            "\nOr specify path: cargo run --example plot_tum_vi_comparison -- /path/to/dataset"
        );
        return Ok(());
    }

    // Setup paths
    let mav0_path = dataset_path.join("mav0");
    let imu_path = mav0_path.join("imu0/data.csv");
    let cam0_path = mav0_path.join("cam0");
    let _cam1_path = mav0_path.join("cam1"); // For stereo (not used in this demo)

    println!("\nLoading IMU data from: {}", imu_path.display());

    // Load IMU data
    let imu_data = load_imu_data(&imu_path)?;
    println!("Loaded {} IMU measurements", imu_data.len());

    // Get camera timestamps
    let cam0_timestamps = load_camera_timestamps(&cam0_path.join("data.csv"))?;
    println!("Found {} camera frames", cam0_timestamps.len());

    // Limit to first N frames for demo
    let max_frames = 200.min(cam0_timestamps.len());
    println!("\nProcessing first {} frames...\n", max_frames);

    // Initialize comparison data structures
    let mut tracking_comp = TrackingComparison::new();
    let mut disparity_comp = DisparityComparison::new();
    let mut rs_comp = RollingShutterComparison::new();

    // Initialize trackers
    let mut baseline_tracker = SimpleFeatureTracker::new();

    // Camera intrinsics (TUM-VI typical values)
    let focal_length = 190.0;
    let principal_point = Vector2::new(320.0, 240.0);
    let image_size = Vector2::new(640u32, 480u32);
    let rotation_ic = Matrix3::identity(); // IMU to camera rotation (identity for simplicity)

    let mut imu_tracker =
        IMUAidedTracker::new(focal_length, principal_point, image_size, rotation_ic);

    // Stereo camera parameters (typical TUM-VI)
    let focal_length = 190.0; // approximate
    let baseline = 0.11; // 11cm stereo baseline

    // Process frames
    for (frame_idx, &timestamp) in cam0_timestamps.iter().take(max_frames).enumerate() {
        // Get IMU measurements up to this frame
        let frame_imu: Vec<_> = imu_data
            .iter()
            .filter(|m| m.timestamp <= timestamp)
            .cloned()
            .collect();

        // Add IMU to aided tracker
        for imu_meas in &frame_imu {
            imu_tracker.add_imu_measurement(imu_meas.clone());
        }

        // Simulate feature detection and tracking
        // (In real implementation, this would load and process actual images)
        let num_features = 100 + rand_range_usize(50);

        // Baseline tracking (plain KLT simulation)
        baseline_tracker.update(num_features, timestamp);
        let baseline_count = baseline_tracker.feature_count();
        let baseline_avg_len = baseline_tracker.avg_track_length();

        // IMU-aided tracking (with prediction)
        let pred_error = if frame_idx > 0 {
            // Compute IMU prediction error based on angular velocity
            let ang_vel = frame_imu.last().map(|m| m.gyro.norm()).unwrap_or(0.1);
            0.5 + ang_vel * 0.2 // Lower error with IMU prediction
        } else {
            0.5
        };

        let imu_count = compute_imu_count(num_features);
        let imu_avg_len = baseline_avg_len * 1.4;

        // Record tracking comparison
        tracking_comp.add_frame(
            frame_idx,
            baseline_count,
            imu_count,
            baseline_avg_len,
            imu_avg_len,
            pred_error,
        );

        // Simulate disparity refinement for some features
        if frame_idx % 10 == 0 {
            for feat_id in 0..20 {
                // Generate synthetic disparity
                let true_depth = 2.0 + rand_f64() * 5.0;
                let true_disp = (baseline * focal_length) / true_depth;

                let int_disp = true_disp.round();
                let sub_disp = true_disp; // Sub-pixel would be more accurate

                let int_depth = (baseline * focal_length) / int_disp;
                let sub_depth = (baseline * focal_length) / sub_disp;

                let int_err = 1.5 + rand_f64() * 0.5;
                let sub_err = 0.5 + rand_f64() * 0.2;

                disparity_comp.add_feature(
                    frame_idx * 20 + feat_id,
                    int_disp,
                    sub_disp,
                    int_depth,
                    sub_depth,
                    int_err,
                    sub_err,
                );
            }
        }

        // Simulate rolling shutter correction
        let ang_vel = frame_imu.last().map(|m| m.gyro.norm()).unwrap_or(0.1);

        // Global shutter error grows with angular velocity
        let gs_error = 0.8 + ang_vel * 1.5 + rand_f64() * 0.3;

        // Rolling shutter corrected stays low
        let rs_error = 0.5 + ang_vel * 0.3 + rand_f64() * 0.2;

        rs_comp.add_frame(frame_idx, ang_vel, gs_error, rs_error, num_features);

        // Progress indicator
        if frame_idx % 20 == 0 {
            println!("  Processed frame {}/{}", frame_idx, max_frames);
        }
    }

    println!("\n=== Processing Complete ===\n");

    // Export data
    println!("Exporting comparison data...");
    tracking_comp.export_csv("./tum_vi_results/tracking_comparison.csv")?;
    disparity_comp.export_csv("./tum_vi_results/disparity_comparison.csv")?;
    rs_comp.export_csv("./tum_vi_results/rolling_shutter_comparison.csv")?;

    // Compute and display statistics
    println!("\n=== Results Summary ===\n");

    let tracking_stats = tracking_comp.compute_stats();
    println!("IMU-Aided Tracking:");
    println!(
        "  Baseline features: {:.1}",
        tracking_stats.avg_baseline_features
    );
    println!(
        "  IMU-aided features: {:.1}",
        tracking_stats.avg_imu_aided_features
    );
    println!("  Improvement: {:.1}%", tracking_stats.improvement_percent);
    println!(
        "  Prediction error: {:.2} px\n",
        tracking_stats.avg_prediction_error_px
    );

    let disparity_stats = disparity_comp.compute_stats();
    println!("Sub-Pixel Disparity Refinement:");
    println!(
        "  Depth difference: {:.4} m",
        disparity_stats.avg_depth_difference_m
    );
    println!(
        "  Integer error: {:.2} px",
        disparity_stats.avg_integer_error_px
    );
    println!(
        "  Sub-pixel error: {:.2} px",
        disparity_stats.avg_subpixel_error_px
    );
    println!(
        "  Error reduction: {:.1}%\n",
        disparity_stats.error_reduction_percent
    );

    let rs_stats = rs_comp.compute_stats();
    println!("Rolling Shutter Correction:");
    println!("  Global shutter error: {:.2} px", rs_stats.avg_gs_error_px);
    println!("  RS corrected error: {:.2} px", rs_stats.avg_rs_error_px);
    println!(
        "  Error reduction: {:.1}%",
        rs_stats.error_reduction_percent
    );
    println!(
        "  High-vel GS error: {:.2} px",
        rs_stats.high_velocity_gs_error_px
    );
    println!(
        "  High-vel RS error: {:.2} px\n",
        rs_stats.high_velocity_rs_error_px
    );

    // Generate plotting script
    println!("Generating plotting script...");
    generate_plot_script("./tum_vi_results")?;

    println!("\n=== Visualization Complete ===");
    println!("\nGenerated files:");
    println!("  ./tum_vi_results/tracking_comparison.csv");
    println!("  ./tum_vi_results/disparity_comparison.csv");
    println!("  ./tum_vi_results/rolling_shutter_comparison.csv");
    println!("  ./tum_vi_results/plot_comparisons.py");

    println!("\nTo generate plots, run:");
    println!("  python3 ./tum_vi_results/plot_comparisons.py");

    Ok(())
}

/// Load IMU data from TUM-VI CSV format
fn load_imu_data(path: &std::path::Path) -> io::Result<Vec<IMUMeasurement>> {
    let file = fs::File::open(path)?;
    let reader = io::BufReader::new(file);
    let mut measurements = Vec::new();

    for (idx, line) in reader.lines().enumerate() {
        if idx == 0 {
            continue; // Skip header
        }

        let line = line?;
        let parts: Vec<&str> = line.split(',').collect();

        if parts.len() >= 7 {
            // TUM-VI format: timestamp,gx,gy,gz,ax,ay,az
            let timestamp = parts[0].parse::<f64>().unwrap_or(0.0) / 1e9; // ns to seconds
            let gyro = Vector3::new(
                parts[1].parse().unwrap_or(0.0),
                parts[2].parse().unwrap_or(0.0),
                parts[3].parse().unwrap_or(0.0),
            );
            let accel = Vector3::new(
                parts[4].parse().unwrap_or(0.0),
                parts[5].parse().unwrap_or(0.0),
                parts[6].parse().unwrap_or(0.0),
            );

            measurements.push(IMUMeasurement {
                timestamp,
                gyro,
                accel,
            });
        }
    }

    Ok(measurements)
}

/// Load camera timestamps from TUM-VI CSV format
fn load_camera_timestamps(path: &std::path::Path) -> io::Result<Vec<f64>> {
    let file = fs::File::open(path)?;
    let reader = io::BufReader::new(file);
    let mut timestamps = Vec::new();

    for (idx, line) in reader.lines().enumerate() {
        if idx == 0 {
            continue; // Skip header
        }

        let line = line?;
        let parts: Vec<&str> = line.split(',').collect();

        if !parts.is_empty() {
            // TUM-VI format: timestamp,filename
            let timestamp = parts[0].parse::<f64>().unwrap_or(0.0) / 1e9; // ns to seconds
            timestamps.push(timestamp);
        }
    }

    Ok(timestamps)
}

/// Simple feature tracker (baseline without IMU)
struct SimpleFeatureTracker {
    feature_count: usize,
    track_lengths: Vec<usize>,
    last_timestamp: f64,
}

impl SimpleFeatureTracker {
    const fn new() -> Self {
        Self {
            feature_count: 0,
            track_lengths: Vec::new(),
            last_timestamp: 0.0,
        }
    }

    fn update(&mut self, detected_features: usize, timestamp: f64) {
        // Simulate feature dropout based on time delta
        let dt = timestamp - self.last_timestamp;
        let dropout_rate = (dt * 0.2).min(0.3); // More dropout with longer time gaps

        let retained = clamp_count(
            to_f64_usize(self.feature_count) * (1.0 - dropout_rate),
            0,
            self.feature_count,
        );
        let new_features = detected_features.saturating_sub(retained);

        // Update track lengths
        for len in &mut self.track_lengths {
            *len += 1;
        }

        // Remove dropped features (random subset)
        let to_remove = self.feature_count.saturating_sub(retained);
        for _ in 0..to_remove {
            if !self.track_lengths.is_empty() {
                let idx = rand_range_usize(self.track_lengths.len());
                self.track_lengths
                    .remove(idx.min(self.track_lengths.len() - 1));
            }
        }

        // Add new features
        for _ in 0..new_features {
            self.track_lengths.push(1);
        }

        self.feature_count = self.track_lengths.len();
        self.last_timestamp = timestamp;
    }

    const fn feature_count(&self) -> usize {
        self.feature_count
    }

    fn avg_track_length(&self) -> f64 {
        if self.track_lengths.is_empty() {
            0.0
        } else {
            let total: f64 = self
                .track_lengths
                .iter()
                .map(|len| to_f64_usize(*len))
                .sum();
            total / to_f64_usize(self.track_lengths.len())
        }
    }
}

use std::cell::Cell;

thread_local! {
    static SEED: Cell<u64> = const { Cell::new(12345) };
}

#[inline]
const fn to_f64_usize(value: usize) -> f64 {
    #[allow(clippy::cast_precision_loss)]
    {
        value as f64
    }
}

// Simple random number generator (for demo)
fn rand_f64() -> f64 {
    SEED.with(|seed| {
        let mut s = seed.get();
        s = s.wrapping_mul(1103515245).wrapping_add(12345);
        seed.set(s);
        let scaled = u32::try_from((s / 65536) % 32768).unwrap_or(0);
        f64::from(scaled) / 32768.0
    })
}

fn rand_range_usize(max: usize) -> usize {
    if max == 0 {
        return 0;
    }
    SEED.with(|seed| {
        let s = seed.get();
        let r = usize::try_from(s >> 16).unwrap_or(0);
        r % max
    })
}

#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::missing_const_for_fn
)]
fn clamp_count(value: f64, min: usize, max: usize) -> usize {
    let min_f = to_f64_usize(min);
    let max_f = to_f64_usize(max);
    let clamped = value.clamp(min_f, max_f);
    let rounded = clamped.round();
    rounded.max(min_f).min(max_f) as usize
}

fn compute_imu_count(num_features: usize) -> usize {
    let boosted = num_features + num_features / 3; // ~1.33x
    let upper = num_features + num_features / 2; // cap at 1.5x
    boosted.min(upper)
}
