//! Comprehensive tests for rolling shutter compensation
//! Tests happy paths, edge cases, error conditions, and performance

use rs_vio::camera::rolling_shutter::RollingShutterCompensator;
use rs_vio::datasets::ImuData;

/// Test basic rolling shutter compensator creation
#[test]
fn test_rolling_shutter_compensator_creation() {
    let compensator = RollingShutterCompensator::new(0.02, 480); // 20ms readout, 480 rows

    assert!((compensator.config().readout_time - 0.02).abs() < 1e-12);
    assert_eq!(compensator.config().image_height, 480);
    assert!(compensator.config().top_to_bottom);
}

/// Test compensator creation with different parameters
#[test]
fn test_compensator_creation_variations() {
    // Fast readout (near global shutter)
    let fast_compensator = RollingShutterCompensator::new(0.001, 480);

    // Slow readout
    let slow_compensator = RollingShutterCompensator::new(0.1, 1080); // 4K height

    // Both should work
    assert!((fast_compensator.config().readout_time - 0.001).abs() < 1e-12);
    assert_eq!(slow_compensator.config().image_height, 1080);
}

/// Test compensation with normal feature data (happy path)
#[test]
fn test_rolling_shutter_normal_compensation() {
    let compensator = RollingShutterCompensator::new(0.02, 480);

    // Create some test features
    let features = vec![
        (100.0, 50.0),  // Top of image
        (200.0, 240.0), // Middle of image
        (300.0, 430.0), // Bottom of image
    ];

    // Create IMU data with some motion
    let imu_data = generate_test_imu_data(10);

    // Test compensation
    let compensated = compensator.compensate_features(
        &features,
        &imu_data,
        1000000000,
        &nalgebra::Matrix4::identity(),
    );

    // Should return same number of features
    assert_eq!(compensated.len(), features.len());

    // All outputs should be finite
    for (u, v) in compensated {
        assert!(u.is_finite());
        assert!(v.is_finite());
    }
}

/// Test compensation with empty IMU data
#[test]
fn test_rolling_shutter_empty_imu() {
    let compensator = RollingShutterCompensator::new(0.02, 480);
    let features = vec![(100.0, 100.0), (200.0, 200.0)];

    // Empty IMU data
    let imu_data = vec![];

    let compensated = compensator.compensate_features(
        &features,
        &imu_data,
        1000000000,
        &nalgebra::Matrix4::identity(),
    );

    // Should return original features unchanged
    assert_eq!(compensated.len(), features.len());
    for ((orig_u, orig_v), (comp_u, comp_v)) in features.iter().zip(compensated.iter()) {
        assert!((orig_u - comp_u).abs() < 1e-6);
        assert!((orig_v - comp_v).abs() < 1e-6);
    }
}

/// Test compensation with empty features
#[test]
fn test_rolling_shutter_empty_features() {
    let compensator = RollingShutterCompensator::new(0.02, 480);
    let features = vec![];
    let imu_data = generate_test_imu_data(5);

    let compensated = compensator.compensate_features(
        &features,
        &imu_data,
        1000000000,
        &nalgebra::Matrix4::identity(),
    );

    assert!(compensated.is_empty());
}

/// Test compensation with extreme IMU values
#[test]
fn test_rolling_shutter_extreme_imu() {
    let compensator = RollingShutterCompensator::new(0.02, 480);
    let features = vec![(320.0, 240.0)]; // Center of image

    // Create IMU data with extreme motion
    let extreme_imu = vec![ImuData {
        timestamp: 1000000000,
        gyro: [10.0, 5.0, -8.0],     // Very fast rotation
        accel: [50.0, -30.0, 100.0], // Extreme acceleration
    }];

    let compensated = compensator.compensate_features(
        &features,
        &extreme_imu,
        1000000000,
        &nalgebra::Matrix4::identity(),
    );

    // Should still produce finite outputs
    assert_eq!(compensated.len(), 1);
    let (u, v) = compensated[0];
    assert!(u.is_finite());
    assert!(v.is_finite());
}

/// Test compensation with different image sizes
#[test]
fn test_rolling_shutter_different_sizes() {
    let sizes: [u32; 4] = [240, 480, 720, 1080];

    for &height in &sizes {
        let compensator = RollingShutterCompensator::new(0.02, height as usize);
        let features = vec![(100.0, f64::from(height) / 2.0)]; // Middle row
        let imu_data = generate_test_imu_data(5);

        let compensated = compensator.compensate_features(
            &features,
            &imu_data,
            1000000000,
            &nalgebra::Matrix4::identity(),
        );

        assert_eq!(compensated.len(), 1);
        let (_, v) = compensated[0];
        assert!(v.is_finite());
        assert!(v >= 0.0);
        assert!(v < f64::from(height));
    }
}

/// Test compensation with very fast readout times
#[test]
fn test_rolling_shutter_fast_readout() {
    let compensator = RollingShutterCompensator::new(0.001, 480); // 1ms readout (global shutter like)
    let features = vec![(100.0, 100.0), (200.0, 380.0)];
    let imu_data = generate_test_imu_data(10);

    let compensated = compensator.compensate_features(
        &features,
        &imu_data,
        1000000000,
        &nalgebra::Matrix4::identity(),
    );

    // With fast readout, compensation should be minimal
    assert_eq!(compensated.len(), features.len());
    for ((orig_u, orig_v), (comp_u, comp_v)) in features.iter().zip(compensated.iter()) {
        // Should be very close to original (minimal distortion)
        assert!((orig_u - comp_u).abs() < 10.0);
        assert!((orig_v - comp_v).abs() < 10.0);
    }
}

/// Test compensation with slow readout times
#[test]
fn test_rolling_shutter_slow_readout() {
    let compensator = RollingShutterCompensator::new(0.1, 480); // 100ms readout (very slow)
    let features = vec![(100.0, 100.0), (200.0, 380.0)];
    let imu_data = generate_test_imu_data(50);

    let compensated = compensator.compensate_features(
        &features,
        &imu_data,
        1000000000,
        &nalgebra::Matrix4::identity(),
    );

    // Should still produce valid outputs
    assert_eq!(compensated.len(), features.len());
    for (u, v) in compensated {
        assert!(u.is_finite());
        assert!(v.is_finite());
    }
}

/// Test compensation with different IMU data patterns
#[test]
fn test_compensation_different_imu_patterns() {
    let compensator = RollingShutterCompensator::new(0.02, 480);
    let features = vec![(200.0, 120.0), (400.0, 360.0)];

    // Test with steady IMU (no motion)
    let steady_imu = vec![ImuData {
        timestamp: 1000000000,
        gyro: [0.0, 0.0, 0.0],
        accel: [0.0, 0.0, 9.81],
    }];

    let compensated_steady = compensator.compensate_features(
        &features,
        &steady_imu,
        1000000000,
        &nalgebra::Matrix4::identity(),
    );
    assert_eq!(compensated_steady.len(), features.len());
    assert!(compensated_steady
        .iter()
        .all(|(u, v)| u.is_finite() && v.is_finite()));

    // Test with rotating IMU
    let rotating_imu = vec![ImuData {
        timestamp: 1000000000,
        gyro: [1.0, 0.5, -0.3], // Significant rotation
        accel: [0.0, 0.0, 9.81],
    }];

    let compensated_rotating = compensator.compensate_features(
        &features,
        &rotating_imu,
        1000000000,
        &nalgebra::Matrix4::identity(),
    );
    assert_eq!(compensated_rotating.len(), features.len());
    assert!(compensated_rotating
        .iter()
        .all(|(u, v)| u.is_finite() && v.is_finite()));
}

/// Test compensation with identity transform
#[test]
fn test_compensation_identity_transform() {
    let compensator = RollingShutterCompensator::new(0.02, 480);
    let identity = nalgebra::Matrix4::identity();
    let test_features = vec![(320.0, 240.0)];
    let empty_imu = vec![];

    let compensated =
        compensator.compensate_features(&test_features, &empty_imu, 1000000000, &identity);

    // Should return original features when no IMU data
    assert_eq!(compensated.len(), 1);
    let (u, v) = compensated[0];
    assert!(u.is_finite());
    assert!(v.is_finite());
    // Should be very close to original
    assert!((u - 320.0).abs() < 1.0);
    assert!((v - 240.0).abs() < 1.0);
}

/// Test performance under load
#[test]
fn test_rolling_shutter_performance() {
    let compensator = RollingShutterCompensator::new(0.02, 480);
    let features = (0..100)
        .map(|i| (f64::from(i) * 6.4, f64::from(i) * 4.8))
        .collect::<Vec<_>>();
    let imu_data = generate_test_imu_data(100);

    let start = std::time::Instant::now();
    let compensated = compensator.compensate_features(
        &features,
        &imu_data,
        1000000000,
        &nalgebra::Matrix4::identity(),
    );
    let duration = start.elapsed();

    assert_eq!(compensated.len(), features.len());
    // Should be reasonably fast (less than 10ms for 100 features)
    assert!(duration.as_millis() < 10);
}

// Helper function for test IMU data
fn generate_test_imu_data(num_samples: u32) -> Vec<ImuData> {
    (0..num_samples)
        .map(|i| ImuData {
            timestamp: 1_000_000_000 + i64::from(i) * 5_000_000, // 5ms intervals
            gyro: [0.1 * (f64::from(i) * 0.1).sin(), 0.05, 0.02], // Some rotation
            accel: [0.0, 0.0, 9.81],                      // Gravity only
        })
        .collect()
}
