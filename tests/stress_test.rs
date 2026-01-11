#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::cast_lossless
)]

use rs_vio::datasets::config::Config;
use rs_vio::datasets::ImuData;
use rs_vio::estimator::Estimator;
use std::time::Duration;

fn small_test_config() -> Config {
    let yaml = r#"
    camera:
      image_width: 64
      image_height: 48
      left_intrinsics: [400.0, 400.0, 32.0, 24.0]
      left_distortion: [0.0, 0.0, 0.0, 0.0]
      right_intrinsics: [400.0, 400.0, 32.0, 24.0]
      right_distortion: [0.0, 0.0, 0.0, 0.0]
      left_model: pinhole-radtan
      right_model: pinhole-radtan
      T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
      T_B_Cr: [1.0, 0.0, 0.0, 0.05, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    keyframe_management:
      keyframe_window_size: 5
      translation_threshold: 0.05
      rotation_threshold: 0.05
    feature_detection:
      grid_size: 8
      max_features_per_grid: 32
      optical_flow_max_iterations: 20
      optical_flow_convergence_threshold: 0.01
    optimization:
      bundle_adjustment_max_iterations: 5
      pnp_max_iterations: 3
    "#;
    serde_yaml::from_str(yaml).expect("valid test config")
}

fn generate_synthetic_imu_data(timestamp_ns: i64, num_samples: usize) -> Vec<ImuData> {
    let dt_ns = 5_000_000; // 5ms between IMU samples (200Hz)
    let mut imu_data = Vec::with_capacity(num_samples);

    for i in 0..num_samples {
        let sample_ts = timestamp_ns + (i as i64) * dt_ns;

        // Simulate gyroscope: slow rotation around z-axis (yaw)
        let wx = 0.01; // rad/s
        let wy = 0.005;
        let wz = 0.02;

        // Simulate accelerometer: gravity + slight motion
        let ax = 0.1; // m/s^2
        let ay = 0.05;
        let az = -9.81 + 0.2;

        imu_data.push(ImuData {
            timestamp: sample_ts,
            gyro: [wx, wy, wz],
            accel: [ax, ay, az],
        });
    }

    imu_data
}

#[test]
fn stress_process_frames_stays_within_bounds() {
    let config = small_test_config();
    let mut estimator = Estimator::new(config.clone(), None);
    estimator.set_max_map_points(100);
    estimator.set_max_frame_processing_time(Duration::from_millis(500));

    let img_w = 64;
    let img_h = 48;
    let mut left = vec![0u8; img_w * img_h];
    let mut right = vec![0u8; img_w * img_h];

    // Run a moderate-length soak to mimic a longer mission without making CI slow.
    for frame_id in 0..200 {
        // Simple checker pattern with slight horizontal offset to ensure features move.
        for y in 0..img_h {
            for x in 0..img_w {
                let idx = y * img_w + x;
                left[idx] = if (x / 8 + y / 8) % 2 == 0 { 255 } else { 0 };
                right[idx] = if ((x + 2) / 8 + y / 8) % 2 == 0 {
                    255
                } else {
                    0
                };
            }
        }

        let ts = (frame_id as i64) * 10_000_000; // 10ms increments

        // Generate synthetic IMU data for this frame (200Hz IMU, 2 samples per 10ms frame)
        let imu_data = generate_synthetic_imu_data(ts, 2);

        estimator
            .process_frame(&left, &right, ts, Some(&imu_data))
            .unwrap_or_else(|e| panic!("Frame {} failed: {}", frame_id, e));
    }

    // Ensure bounded map size and progress.
    assert!(estimator.map_points_len() <= 100);
    assert!(estimator.frame_count() >= 200);

    // Verify IMU measurements were processed
    assert!(
        estimator.imu_measurement_count() > 0,
        "IMU measurements should be processed"
    );

    // Verify IMU motion prior can be retrieved
    let prior = estimator.get_imu_motion_prior();
    assert!(
        prior.is_some(),
        "IMU motion prior should be available after processing"
    );
}

#[test]
fn stress_imu_features_only() {
    use rs_vio::datasets::config::Config;
    use rs_vio::datasets::ImuData;
    use rs_vio::estimator::Estimator;

    let yaml = r#"
    camera:
      image_width: 128
      image_height: 96
      left_intrinsics: [400.0, 400.0, 64.0, 48.0]
      left_distortion: [0.0, 0.0, 0.0, 0.0]
      right_intrinsics: [400.0, 400.0, 64.0, 48.0]
      right_distortion: [0.0, 0.0, 0.0, 0.0]
      left_model: pinhole-radtan
      right_model: pinhole-radtan
      T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
      T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    keyframe_management:
      keyframe_window_size: 5
      translation_threshold: 0.1
      rotation_threshold: 0.1
    feature_detection:
      grid_size: 8
      max_features_per_grid: 32
      optical_flow_max_iterations: 20
      optical_flow_convergence_threshold: 0.01
    optimization:
      bundle_adjustment_max_iterations: 5
      pnp_max_iterations: 3
    "#;

    let config: Config = serde_yaml::from_str(yaml).expect("valid config");
    let mut estimator = Estimator::new(config, None);
    estimator.set_max_map_points(50);
    estimator.set_max_frame_processing_time(Duration::from_millis(100));

    let img_w = 128;
    let img_h = 96;
    let mut left = vec![128u8; img_w * img_h];
    let mut right = vec![128u8; img_w * img_h];

    // Add corner features for tracking
    for y in 0..img_h {
        for x in 0..img_w {
            let idx = y * img_w + x;
            // Create corner-like pattern
            let cx = x as i32 - 32;
            let cy = y as i32 - 24;
            if (cx * cx + cy * cy) < 100 {
                left[idx] = 255;
                right[idx] = 255;
            } else {
                left[idx] = 64;
                right[idx] = 64;
            }
        }
    }

    let mut imu_measurement_count = 0;

    // Process frames with simulated motion and IMU
    for frame_id in 0..100 {
        let ts = (frame_id as i64) * 30_000_000; // 30ms per frame (~33Hz)

        // Simulate increasing motion
        let motion_factor = (frame_id as f64) / 100.0;
        let omega_z = 0.1 * motion_factor; // Increasing rotation
        let acc_x = 0.5 * motion_factor;

        let num_imu_samples = 6; // 200Hz IMU, 6 samples per 30ms frame
        let dt_ns = 5_000_000;

        let imu_data: Vec<ImuData> = (0..num_imu_samples)
            .map(|i| {
                let sample_ts = ts + (i as i64) * dt_ns;
                ImuData {
                    timestamp: sample_ts,
                    gyro: [0.02 * motion_factor, 0.01 * motion_factor, omega_z],
                    accel: [acc_x, 0.1 * motion_factor, -9.81],
                }
            })
            .collect();

        imu_measurement_count += imu_data.len();

        estimator
            .process_frame(&left, &right, ts, Some(&imu_data))
            .unwrap_or_else(|e| panic!("Frame {} failed: {}", frame_id, e));

        // Periodically check IMU rate
        if frame_id % 20 == 0 && frame_id > 0 {
            let rate = estimator.get_imu_rate();
            // With 6 IMU samples per 30ms frame, rate = 6/0.03 = 200 samples/frame / frames = ~200 Hz
            // Actually rate = imu_measurements / frames, so 6/1 = 6, then 12/2 = 6, etc.
            // For increasing motion scenario, we check that it's reasonable
            assert!(
                rate > 5.0,
                "IMU rate should be ~6 samples/frame, got {:.1}",
                rate
            );
        }
    }

    // Verify all IMU measurements were processed
    assert_eq!(
        estimator.imu_measurement_count(),
        imu_measurement_count,
        "All IMU measurements should be counted"
    );

    // Verify velocity estimation is working (now that it's initialized)
    let velocity = estimator.get_velocity();
    assert!(
        velocity.is_some(),
        "Velocity should be estimable after initialization"
    );

    // Verify IMU motion prior is available
    let prior = estimator.get_imu_motion_prior();
    assert!(prior.is_some(), "IMU motion prior should be available");
}

#[test]
fn stress_high_frequency_imu() {
    use rs_vio::datasets::config::Config;
    use rs_vio::datasets::ImuData;
    use rs_vio::estimator::Estimator;

    let yaml = r#"
    camera:
      image_width: 64
      image_height: 48
      left_intrinsics: [400.0, 400.0, 32.0, 24.0]
      left_distortion: [0.0, 0.0, 0.0, 0.0]
      right_intrinsics: [400.0, 400.0, 32.0, 24.0]
      right_distortion: [0.0, 0.0, 0.0, 0.0]
      left_model: pinhole-radtan
      right_model: pinhole-radtan
      T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
      T_B_Cr: [1.0, 0.0, 0.0, 0.05, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    keyframe_management:
      keyframe_window_size: 3
      translation_threshold: 0.1
      rotation_threshold: 0.1
    feature_detection:
      grid_size: 8
      max_features_per_grid: 16
      optical_flow_max_iterations: 10
      optical_flow_convergence_threshold: 0.01
    optimization:
      bundle_adjustment_max_iterations: 3
      pnp_max_iterations: 3
    "#;

    let config: Config = serde_yaml::from_str(yaml).expect("valid config");
    let mut estimator = Estimator::new(config, None);
    estimator.set_max_map_points(30);
    estimator.set_max_frame_processing_time(Duration::from_millis(50));

    let img_w = 64;
    let img_h = 48;
    let left = vec![128u8; img_w * img_h];
    let right = vec![128u8; img_w * img_h];

    // Very high frequency IMU (1000Hz) with 10Hz frame rate
    let num_imu_per_frame = 100;
    let dt_imu_ns = 1_000_000; // 1ms between IMU samples

    for frame_id in 0..50 {
        let ts = (frame_id as i64) * 100_000_000; // 100ms per frame (10Hz)

        let imu_data: Vec<ImuData> = (0..num_imu_per_frame)
            .map(|i| {
                let sample_ts = ts + (i as i64) * dt_imu_ns;
                let t = (sample_ts - ts) as f64 / 1e9; // Time since frame start
                ImuData {
                    timestamp: sample_ts,
                    gyro: [0.05, 0.05, 0.05 + t * 0.1],
                    accel: [(t * 10.0).sin() * 2.0, (t * 10.0).cos() * 2.0, -9.81],
                }
            })
            .collect();

        estimator
            .process_frame(&left, &right, ts, Some(&imu_data))
            .unwrap_or_else(|e| panic!("Frame {} failed: {}", frame_id, e));
    }

    // Verify high-frequency IMU was handled
    let expected_imu_count = 50 * num_imu_per_frame;
    assert_eq!(
        estimator.imu_measurement_count(),
        expected_imu_count,
        "All high-frequency IMU samples should be processed"
    );

    // IMU rate should be very high (~100 samples per frame, since 1000Hz / 10Hz = 100)
    let rate = estimator.get_imu_rate();
    assert!(
        rate > 50.0 && rate < 150.0,
        "IMU rate should be ~100 samples/frame for high-frequency test, got {:.1}",
        rate
    );
}
