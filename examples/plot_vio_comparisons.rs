/// Example: Generate comparison plots for VIO optimizations
///
/// This example demonstrates how to use the visualization module to create
/// comparison plots showing the benefits of:
/// - IMU-aided tracking vs baseline KLT
/// - Sub-pixel disparity refinement vs integer disparity
/// - Rolling shutter correction vs global shutter
///
/// Usage:
/// ```bash
/// cargo run --example plot_vio_comparisons
/// ```
///
/// This will:
/// 1. Generate synthetic test data showing typical improvements
/// 2. Export data to CSV files in ./plot_output/
/// 3. Generate a Python plotting script
/// 4. Print statistics
use rs_vio::vision::{
    generate_plot_script, DisparityComparison, RollingShutterComparison, TrackingComparison,
};

#[inline]
fn to_f64_frame(frame: usize) -> f64 {
    f64::from(u32::try_from(frame).unwrap_or(u32::MAX))
}

fn main() -> std::io::Result<()> {
    println!("=== VIO Optimization Comparison Demo ===\n");

    // Create output directory
    std::fs::create_dir_all("./plot_output")?;

    // 1. Generate IMU-aided tracking comparison data
    println!("Generating tracking comparison data...");
    let mut tracking = TrackingComparison::new();

    // Simulate 100 frames
    for frame in 0..100 {
        let ang_vel = (to_f64_frame(frame) * 0.1).sin().abs() * 2.0; // Varying angular velocity

        // Baseline: feature count drops with motion
        let baseline_count = clamp_count(100.0 - ang_vel * 15.0, 50, 120);

        // IMU-aided: much more stable
        let imu_count = clamp_count(110.0 - ang_vel * 5.0, 85, 130);

        // Track lengths
        let baseline_len = 8.0 - ang_vel * 0.5;
        let imu_len = 12.0 - ang_vel * 0.2;

        // Prediction error
        let pred_error = 0.5 + ang_vel * 0.3;

        tracking.add_frame(
            frame,
            baseline_count,
            imu_count,
            baseline_len,
            imu_len,
            pred_error,
        );
    }

    tracking.export_csv("./plot_output/tracking_comparison.csv")?;
    let tracking_stats = tracking.compute_stats();

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
        "  Avg prediction error: {:.2} px\n",
        tracking_stats.avg_prediction_error_px
    );

    // 2. Generate disparity comparison data
    println!("Generating disparity comparison data...");
    let mut disparity = DisparityComparison::new();

    // Camera parameters (typical stereo setup)
    let baseline = 0.12; // 12 cm baseline
    let focal = 500.0; // 500 px focal length

    for i in 0..200 {
        // True depth varies from 0.5m to 10m
        let true_depth = 0.5 + (to_f64_frame(i) / 200.0) * 9.5;

        // True disparity
        let true_disp = (baseline * focal) / true_depth;

        // Integer disparity (rounded)
        let int_disp = true_disp.round();

        // Sub-pixel disparity (with small refinement)
        let sub_disp = true_disp + (rand::random::<f64>() - 0.5) * 0.1;

        // Depths
        let int_depth = (baseline * focal) / int_disp;
        let sub_depth = (baseline * focal) / sub_disp;

        // Errors (sub-pixel has much lower error)
        let int_err = 1.5 + rand::random::<f64>() * 0.5;
        let sub_err = 0.5 + rand::random::<f64>() * 0.2;

        disparity.add_feature(
            i, int_disp, sub_disp, int_depth, sub_depth, int_err, sub_err,
        );
    }

    disparity.export_csv("./plot_output/disparity_comparison.csv")?;
    let disparity_stats = disparity.compute_stats();

    println!(
        "  Avg depth difference: {:.4} m",
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

    // 3. Generate rolling shutter comparison data
    println!("Generating rolling shutter comparison data...");
    let mut rs_comp = RollingShutterComparison::new();

    for frame in 0..150 {
        // Angular velocity varies (simulates drone turns)
        let ang_vel = if frame > 50 && frame < 100 {
            1.5 + (to_f64_frame(frame) - 75.0).abs() * 0.02 // High rotation period
        } else {
            0.2 + rand::random::<f64>() * 0.1
        };

        // Global shutter error grows with angular velocity
        let gs_error = 0.8 + ang_vel * 1.5 + rand::random::<f64>() * 0.3;

        // Rolling shutter correction keeps error low
        let rs_error = 0.5 + ang_vel * 0.3 + rand::random::<f64>() * 0.2;

        let feature_count = clamp_count(120.0 - ang_vel * 5.0, 80, 150);

        rs_comp.add_frame(frame, ang_vel, gs_error, rs_error, feature_count);
    }

    rs_comp.export_csv("./plot_output/rolling_shutter_comparison.csv")?;
    let rs_stats = rs_comp.compute_stats();

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

    // 4. Generate Python plotting script
    println!("Generating plotting script...");
    generate_plot_script("./plot_output")?;

    println!("\n=== Visualization Complete ===");
    println!("\nGenerated files:");
    println!("  ./plot_output/tracking_comparison.csv");
    println!("  ./plot_output/disparity_comparison.csv");
    println!("  ./plot_output/rolling_shutter_comparison.csv");
    println!("  ./plot_output/plot_comparisons.py");

    println!("\nTo generate plots, run:");
    println!("  python3 ./plot_output/plot_comparisons.py");
    println!("\nRequirements: pandas, matplotlib, numpy");
    println!("  pip install pandas matplotlib numpy");

    Ok(())
}

// Simple random number generator (for demo only)
mod rand {
    use std::cell::Cell;

    thread_local! {
        static SEED: Cell<u64> = const { Cell::new(12345) };
    }

    pub fn random<T>() -> T
    where
        T: From<f64>,
    {
        SEED.with(|seed| {
            let mut s = seed.get();
            s = s.wrapping_mul(1103515245).wrapping_add(12345);
            seed.set(s);
            let value = ((s >> 16) & 0x7FFF) as u16;
            T::from(f64::from(value) / 32768.0)
        })
    }
}

#[allow(clippy::missing_const_for_fn)]
fn clamp_count(value: f64, min: usize, max: usize) -> usize {
    let min_f = to_f64_usize(min);
    let max_f = to_f64_usize(max);
    let clamped = value.clamp(min_f, max_f);
    let rounded = clamped.round();
    // Range is already bounded; conversion is safe.
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::missing_const_for_fn
    )]
    {
        rounded.max(min_f).min(max_f) as usize
    }
}

#[inline]
fn to_f64_usize(value: usize) -> f64 {
    f64::from(u32::try_from(value).unwrap_or(u32::MAX))
}
