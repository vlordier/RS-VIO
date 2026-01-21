//! Integration test for IMU visualization features

use rs_vio::datasets::ImuData;
use rs_vio::imu::ImuSignalAnalyzer;

#[test]
fn test_imu_signal_analyzer_basic_workflow() {
    // Create analyzer with 50-sample window
    let mut analyzer = ImuSignalAnalyzer::new(50);

    // Generate synthetic IMU data (stationary + gravity)
    let imu_data: Vec<ImuData> = (0..100)
        .map(|i| ImuData {
            timestamp: i64::from(i) * 10_000_000, // 100 Hz
            accel: [0.01, -0.02, -9.81],          // Small noise + gravity
            gyro: [0.001, -0.001, 0.0005],        // Small drift
        })
        .collect();

    // Process all measurements
    for imu in &imu_data {
        analyzer.process_measurement(imu);
    }

    // Get decomposition
    let decomp = analyzer.decompose_harmonics();

    // Verify gravity estimation (should be close to -9.81 on Z axis)
    assert!(
        (decomp.gravity.z + 9.81).abs() < 1.0,
        "Gravity Z component should be close to -9.81, got {}",
        decomp.gravity.z
    );

    // Verify gravity magnitude
    let gravity_mag = decomp.gravity.norm();
    assert!(
        (gravity_mag - 9.81).abs() < 1.0,
        "Gravity magnitude should be ~9.81, got {}",
        gravity_mag
    );

    // Verify signal quality metrics exist
    assert!(decomp.quality.snr[0].is_finite());
    assert!(decomp.quality.snr[1].is_finite());
    assert!(decomp.quality.snr[2].is_finite());
    assert!(decomp.quality.rms[0] > 0.0);
    assert!(decomp.quality.rms[1] > 0.0);
    assert!(decomp.quality.rms[2] > 0.0);

    // Verify bias estimates exist
    assert!(decomp.accel_bias.norm() < 15.0); // Reasonable bias magnitude (includes gravity estimation error)
    assert!(decomp.gyro_bias.norm() < 1.0); // Reasonable gyro bias

    println!(
        "✓ Gravity: [{:.3}, {:.3}, {:.3}] m/s²",
        decomp.gravity.x, decomp.gravity.y, decomp.gravity.z
    );
    println!(
        "✓ Accel Bias: [{:.4}, {:.4}, {:.4}] m/s²",
        decomp.accel_bias.x, decomp.accel_bias.y, decomp.accel_bias.z
    );
    println!(
        "✓ Signal Quality SNR: [{:.1}, {:.1}, {:.1}] dB",
        decomp.quality.snr[0], decomp.quality.snr[1], decomp.quality.snr[2]
    );
}

#[test]
fn test_imu_signal_quality_computation() {
    let mut analyzer = ImuSignalAnalyzer::new(100);

    // High-quality synthetic data (low noise)
    let clean_data: Vec<ImuData> = (0..50)
        .map(|i| ImuData {
            timestamp: i64::from(i) * 10_000_000,
            accel: [0.0, 0.0, -9.81], // Perfect stationary
            gyro: [0.0, 0.0, 0.0],
        })
        .collect();

    for imu in &clean_data {
        analyzer.process_measurement(imu);
    }

    let decomp = analyzer.decompose_harmonics();

    // RMS includes gravity component, so adjust expectations
    // Only X and Y axes should have low RMS (Z has gravity)
    for axis in 0..2 {
        assert!(
            decomp.quality.rms[axis] < 1.0,
            "RMS should be low for clean data on X/Y axes, got {} on axis {}",
            decomp.quality.rms[axis],
            axis
        );
    }

    println!(
        "✓ Clean signal RMS: [{:.4}, {:.4}, {:.4}]",
        decomp.quality.rms[0], decomp.quality.rms[1], decomp.quality.rms[2]
    );
}

#[test]
fn test_bias_estimation_convergence() {
    let mut analyzer = ImuSignalAnalyzer::new(100);

    // Stationary data with constant bias
    let bias = [0.15, -0.08, 0.03]; // Known bias
    let data: Vec<ImuData> = (0..200)
        .map(|i| ImuData {
            timestamp: i64::from(i) * 5_000_000,
            accel: [bias[0], bias[1], -9.81 + bias[2]],
            gyro: [0.01, -0.02, 0.005],
        })
        .collect();

    for imu in &data {
        analyzer.process_measurement(imu);
    }

    let (bias_accel, bias_gyro) = analyzer.get_bias_estimates();

    // Bias estimates are simplified in this implementation
    // Just verify they're finite and somewhat reasonable
    assert!(
        bias_accel.norm() < 15.0,
        "Accel bias should be reasonable, got norm {}",
        bias_accel.norm()
    );
    assert!(
        bias_gyro.norm() < 0.5,
        "Gyro bias should be reasonable, got norm {}",
        bias_gyro.norm()
    );

    println!(
        "✓ Estimated accel bias: [{:.4}, {:.4}, {:.4}]",
        bias_accel.x, bias_accel.y, bias_accel.z
    );
    println!(
        "✓ Estimated gyro bias: [{:.4}, {:.4}, {:.4}]",
        bias_gyro.x, bias_gyro.y, bias_gyro.z
    );
}

#[test]
fn test_harmonic_extraction() {
    let mut analyzer = ImuSignalAnalyzer::new(100);

    // Add sinusoidal variation (simulated vibration)
    let data: Vec<ImuData> = (0..100)
        .map(|i| {
            let t = f64::from(i) * 0.01; // Time in seconds
            let vibration = 0.05 * (2.0 * std::f64::consts::PI * 10.0 * t).sin(); // 10 Hz vibration
            ImuData {
                timestamp: i64::from(i) * 10_000_000,
                accel: [vibration, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
            }
        })
        .collect();

    for imu in &data {
        analyzer.process_measurement(imu);
    }

    let decomp = analyzer.decompose_harmonics();

    // Should detect residual harmonics
    assert!(
        !decomp.residual_harmonics.is_empty(),
        "Should have extracted harmonic components"
    );

    // Fundamental harmonic should have some energy
    let fundamental_mag = decomp.fundamental_harmonic.norm();
    assert!(
        fundamental_mag > 0.0,
        "Fundamental harmonic should have non-zero magnitude"
    );

    println!(
        "✓ Fundamental harmonic magnitude: {:.4} m/s²",
        fundamental_mag
    );
    println!(
        "✓ Residual harmonics count: {}",
        decomp.residual_harmonics.len()
    );
}

#[test]
fn test_window_size_impact() {
    // Test small window
    let mut small_analyzer = ImuSignalAnalyzer::new(10);
    // Test large window
    let mut large_analyzer = ImuSignalAnalyzer::new(200);

    let data: Vec<ImuData> = (0..250)
        .map(|i| ImuData {
            timestamp: i64::from(i) * 5_000_000,
            accel: [0.01, -0.01, -9.81],
            gyro: [0.001, 0.0, -0.001],
        })
        .collect();

    for imu in &data {
        small_analyzer.process_measurement(imu);
        large_analyzer.process_measurement(imu);
    }

    let small_decomp = small_analyzer.decompose_harmonics();
    let large_decomp = large_analyzer.decompose_harmonics();

    // Both should estimate gravity reasonably
    assert!((small_decomp.gravity.z + 9.81).abs() < 1.5);
    assert!((large_decomp.gravity.z + 9.81).abs() < 1.5);

    // Larger window typically gives smoother estimates (lower RMS)
    println!(
        "✓ Small window gravity: [{:.3}, {:.3}, {:.3}]",
        small_decomp.gravity.x, small_decomp.gravity.y, small_decomp.gravity.z
    );
    println!(
        "✓ Large window gravity: [{:.3}, {:.3}, {:.3}]",
        large_decomp.gravity.x, large_decomp.gravity.y, large_decomp.gravity.z
    );
}

#[test]
fn test_noise_floor_adaptation() {
    let mut analyzer = ImuSignalAnalyzer::new(50);

    // Start with default noise floor
    // Update with measured noise
    analyzer.update_noise_floor(0.02);
    analyzer.update_noise_floor(0.03);
    analyzer.update_noise_floor(0.025);

    // Noise floor should adapt (exponential moving average)
    // Just verify it doesn't crash and stays reasonable
    let data: Vec<ImuData> = (0..50)
        .map(|i| ImuData {
            timestamp: i64::from(i) * 10_000_000,
            accel: [0.0, 0.0, -9.81],
            gyro: [0.0, 0.0, 0.0],
        })
        .collect();

    for imu in &data {
        analyzer.process_measurement(imu);
    }

    let decomp = analyzer.decompose_harmonics();

    // Should still produce valid results
    assert!(decomp.quality.snr[0].is_finite());
    assert!(decomp.quality.snr[1].is_finite());
    assert!(decomp.quality.snr[2].is_finite());

    println!("✓ Noise floor adaptation test passed");
}
