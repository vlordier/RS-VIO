//! Example output from stereo calibration logging system
//!
//! This shows the type of logging output you can expect when using
//! the enhanced calibration system with logging enabled.
use nalgebra as na;
use rs_vio::calibration::stereo_calibrator::StereoPair;
use rs_vio::calibration::CalibrationConfig;
use rs_vio::calibration::StereoCalibrator;
fn main() {
    println!("🎯 Stereo Calibration Logging Demo");
    println!("==================================\n");

    println!("📋 Sample Logging Output:");
    println!("-------------------------\n");

    // Simulate the logging output during calibration
    println!("[CALIBRATION 0.00s] 🚀 Starting calibration with 8 stereo pairs");
    println!("[CALIBRATION 0.00s] 📊 Target: <70.0% accuracy at <1.0px error");
    println!("[CALIBRATION 0.05s] 📷 Added stereo pair 1/10 (18 matches) - Progress: 5.0%");
    println!("[CALIBRATION 0.08s] 📷 Added stereo pair 2/10 (22 matches) - Progress: 10.0%");
    println!("[CALIBRATION 0.12s] 📷 Added stereo pair 3/10 (19 matches) - Progress: 15.0%");
    println!("[CALIBRATION 0.15s] 🎯 Ready for calibration! Call calibrate() when data collection complete.");
    println!("[CALIBRATION 0.20s] 📷 Added stereo pair 4/10 (21 matches) - Progress: 20.0%");
    println!("[CALIBRATION 0.25s] 📷 Added stereo pair 5/10 (20 matches) - Progress: 25.0%");
    println!("[CALIBRATION 0.30s] 📷 Added stereo pair 6/10 (18 matches) - Progress: 30.0%");
    println!("[CALIBRATION 0.35s] 📷 Added stereo pair 7/10 (23 matches) - Progress: 35.0%");
    println!("[CALIBRATION 0.40s] 📷 Added stereo pair 8/10 (19 matches) - Progress: 40.0%");
    println!();

    println!("🔍 Calibration Status Report (CollectingData)");
    println!("⏱️  Elapsed: 0.4s");
    println!("📊 Progress: 40.0%");
    println!("📷 Stereo pairs: 8/10");
    println!("🎯 Feature matches: 160");
    println!("📷 Collecting data... 2 pairs remaining");
    println!("✅ Ready for calibration!");
    println!();

    // Simulate optimization logging
    println!("[CALIBRATION 0.45s] ✅ Optimization completed in 42 iterations");
    println!("[CALIBRATION 0.45s] 📈 Final cost: 0.000234");
    println!("[CALIBRATION 0.45s] 🎯 Final reprojection error: 0.73 pixels");
    println!("[CALIBRATION 0.45s] Quality: 0.73px mean error, 78.3% <1px accuracy, 2 outliers");
    println!("[CALIBRATION 0.45s] Assessment: ✅ ACCEPTABLE");
    println!("[CALIBRATION 0.45s] 📊 Used 8 stereo pairs, 160 total feature matches");
    println!("[CALIBRATION 0.45s] 🔄 Calibration stability: 0.92");
    println!();

    println!("🔍 Calibration Status Report (Success)");
    println!("⏱️  Elapsed: 0.5s");
    println!("📊 Progress: 100.0%");
    println!("📷 Stereo pairs: 8/10");
    println!("🎯 Feature matches: 160");
    println!("✅ Calibration completed successfully!");
    println!("🔄 Stability: 0.92");
    println!("🎯 Accuracy: 78.3%");
    println!();

    println!("💡 Key Logging Features:");
    println!("----------------------");
    println!("✅ Real-time progress with timestamps");
    println!("✅ Quality metrics: mean error, accuracy percentages");
    println!("✅ Automatic pass/fail assessment");
    println!("✅ Outlier detection and counting");
    println!("✅ Stereo pair validation and progress tracking");
    println!("✅ Optimization convergence monitoring");
    println!("✅ Final comprehensive quality report");
    println!("✅ Stability analysis over time");
    println!();

    println!("� Rolling Shutter Auto-Detection Demo:");
    println!("--------------------------------------\n");

    // Create a calibrator with auto-detection enabled (None = auto-detect)
    let config = CalibrationConfig {
        rolling_shutter_enabled: None, // Auto-detect!
        log_calibration_metrics: true,
        ..Default::default()
    };

    let mut calibrator = StereoCalibrator::new(config);

    println!("📋 Initial Status (Auto-Detection Mode):");
    println!("{}", calibrator.status_report());

    // Add some stereo pairs that simulate rolling shutter effects
    println!("\n🎥 Adding Stereo Pairs with Simulated Rolling Shutter:");

    for i in 1..=5 {
        // Create features with rolling shutter-like distortions
        // Higher rows have more distortion (typical rolling shutter pattern)
        let left_features: Vec<_> = (0..15)
            .map(|j| {
                let row = j as f64 / 15.0; // 0 to 1
                let distortion = row * 5.0 * (i as f64 * 0.1).sin(); // Rolling shutter effect
                na::Vector2::new(
                    320.0 + (j as f64 - 7.0) * 20.0 + distortion,
                    240.0 + (j as f64 - 7.0) * 15.0,
                )
            })
            .collect();

        let right_features: Vec<_> = (0..15)
            .map(|j| {
                let row = j as f64 / 15.0;
                let distortion = row * 3.0 * (i as f64 * 0.1).cos(); // Different distortion pattern
                na::Vector2::new(
                    320.0 + (j as f64 - 7.0) * 20.0 - 40.0 + distortion,
                    240.0 + (j as f64 - 7.0) * 15.0,
                )
            })
            .collect();

        let correspondences = (0..12).map(|j| (j, j)).collect();

        let stereo_pair = StereoPair {
            left_features,
            right_features,
            correspondences,
            timestamp: i as f64 * 0.1,
            angular_velocity: Some(na::Vector3::new(0.2, 0.0, 0.0)), // Simulate motion
            linear_velocity: Some(na::Vector3::new(0.0, 0.0, 0.0)),
            feature_qualities: (0..12).map(|_| 0.8).collect(),
        };

        calibrator.add_stereo_pair(stereo_pair);

        // Show detection progress
        if i >= 3 {
            println!("\n🔍 Detection Analysis (after {} pairs):", i);
            if let Some(info) = calibrator.rolling_shutter_detection_info() {
                println!(
                    "  Position distortion: {:.2}",
                    info.position_distortion_score
                );
                println!(
                    "  Temporal consistency: {:.2}",
                    info.temporal_consistency_score
                );
                println!(
                    "  Geometric distortion: {:.2}",
                    info.geometric_distortion_score
                );
                println!("  Combined score: {:.2}", info.combined_score);
                println!(
                    "  Rolling shutter detected: {}",
                    info.rolling_shutter_detected
                );
                println!("  Confidence: {}%", info.confidence);
            }
        }
    }

    println!("\n📊 Final Status Report:");
    println!("{}", calibrator.status_report());

    println!("\n✅ Auto-Detection Results:");
    println!("  - Analyzes feature positions for distortion patterns");
    println!("  - Checks temporal consistency across frames");
    println!("  - Validates geometric constraints");
    println!("  - Combines multiple detection methods");
    println!("  - Provides confidence scores");

    println!("\n🚀 Ready for Raspberry Pi deployment!");
    println!("   The system now automatically detects rolling shutter!");
    println!("   Use this logging to monitor calibration quality in real-time.");
}
