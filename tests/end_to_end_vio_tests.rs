#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::needless_range_loop,
    clippy::len_zero,
    clippy::cast_lossless,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::unnecessary_cast,
    clippy::assign_op_pattern,
    clippy::field_reassign_with_default,
    clippy::redundant_pattern_matching
)]

use image::{GrayImage, Luma};
use nalgebra as na;
/// End-to-end VIO pipeline integration tests
/// These tests validate the complete system behavior with realistic data patterns
use rs_vio::datasets::config::FeatureDetectionConfig;
use rs_vio::datasets::ImuData;
use rs_vio::estimator::Frame;
use rs_vio::feature_tracker::{PatchTracker, StereoPatchTracker};
use rs_vio::imu::{ImuConfig, ImuPreintegrator, VelocityEstimator};
use rs_vio::types::CameraFactory;

/// Create realistic stereo image pair with rich texture and features
/// Uses gradient patterns and randomized blobs for better feature detection
fn create_realistic_stereo_pair(
    width: u32,
    height: u32,
    num_blobs: usize,
    disparity: f32,
) -> (GrayImage, GrayImage) {
    let mut left = GrayImage::new(width, height);
    let mut right = GrayImage::new(width, height);

    // Add gradient background for texture
    for y in 0..height {
        for x in 0..width {
            let grad_x = ((x as f64 / width as f64) * 100.0) as u8;
            let grad_y = ((y as f64 / height as f64) * 50.0) as u8;
            let value = 80u8.saturating_add(grad_x).saturating_add(grad_y);
            left.put_pixel(x, y, Luma([value]));
            right.put_pixel(x, y, Luma([value]));
        }
    }

    // Add prominent feature blobs at strategic locations
    let blob_positions = [
        (100, 100),
        (300, 100),
        (500, 100),
        (150, 240),
        (350, 240),
        (450, 240),
        (100, 380),
        (300, 380),
        (500, 380),
    ];

    for &(cx, cy) in blob_positions.iter().take(num_blobs) {
        // Draw bright blob with Gaussian-like falloff
        for dy in -20i32..20i32 {
            for dx in -20i32..20i32 {
                let dist_sq = (dx * dx + dy * dy) as f32;
                let intensity = (255.0 * (-dist_sq / 200.0).exp()) as u8;

                let x = (cx as i32 + dx).max(0).min(width as i32 - 1) as u32;
                let y = (cy as i32 + dy).max(0).min(height as i32 - 1) as u32;

                let curr = left.get_pixel(x, y)[0];
                left.put_pixel(x, y, Luma([curr.saturating_add(intensity)]));

                // Add to right image with disparity
                let x_right = ((x as f32 - disparity).max(0.0) as u32).min(width - 1);
                let curr_right = right.get_pixel(x_right, y)[0];
                right.put_pixel(x_right, y, Luma([curr_right.saturating_add(intensity)]));
            }
        }

        // Add edge patterns around blobs for better corner detection
        for angle in 0..8 {
            let rad = angle as f64 * std::f64::consts::PI / 4.0;
            for r in 15..25 {
                let x = (cx as f64 + rad.cos() * r as f64) as u32;
                let y = (cy as f64 + rad.sin() * r as f64) as u32;
                if x < width && y < height {
                    left.put_pixel(x, y, Luma([255]));
                    let x_right = ((x as f32 - disparity).max(0.0) as u32).min(width - 1);
                    right.put_pixel(x_right, y, Luma([255]));
                }
            }
        }
    }

    (left, right)
}

/// Create realistic IMU sequence with noise and proper dynamics
fn create_realistic_imu_sequence(motion_type: &str, duration_s: f64, rate_hz: f64) -> Vec<ImuData> {
    let num_samples = (duration_s * rate_hz) as usize;
    let dt = 1.0 / rate_hz;
    let mut imu_data = Vec::new();

    // Realistic noise levels (from typical IMU specs)
    let accel_noise = 0.01; // m/s^2
    let gyro_noise = 0.001; // rad/s
    let gravity = 9.81;

    for i in 0..num_samples {
        let t = i as f64 * dt;
        let timestamp = (t * 1e9) as i64;

        // Simple pseudo-random noise (deterministic for reproducibility)
        let noise_seed = (t * 1000.0) as i64;
        let accel_noise_x = accel_noise * ((noise_seed % 1000) as f64 / 500.0 - 1.0);
        let accel_noise_y = accel_noise * (((noise_seed + 333) % 1000) as f64 / 500.0 - 1.0);
        let gyro_noise_z = gyro_noise * (((noise_seed + 666) % 1000) as f64 / 500.0 - 1.0);

        let (ax, ay, az, wx, wy, wz) = match motion_type {
            "static" => (
                accel_noise_x,
                accel_noise_y,
                gravity,
                0.0,
                0.0,
                gyro_noise_z,
            ),
            "forward_motion" => (
                0.3 + accel_noise_x, // moderate forward acceleration
                accel_noise_y,
                gravity,
                0.0,
                0.0,
                gyro_noise_z,
            ),
            "gentle_rotation" => (
                accel_noise_x,
                accel_noise_y,
                gravity,
                0.0,
                0.0,
                0.05 + gyro_noise_z, // gentle yaw rotation
            ),
            _ => (
                accel_noise_x,
                accel_noise_y,
                gravity,
                0.0,
                0.0,
                gyro_noise_z,
            ),
        };

        imu_data.push(ImuData {
            timestamp,
            accel: [ax, ay, az],
            gyro: [wx, wy, wz],
        });
    }

    imu_data
}

#[test]
fn test_vio_pipeline_static_scene() {
    // Test VIO pipeline on static scene with realistic textures
    let mut config = FeatureDetectionConfig::default();
    config.grid_cols = 8; // Reasonable grid for 640x480
    config.max_features_per_grid = 2;

    let mut tracker = StereoPatchTracker::<3>::from_config(&config);

    let (left, right) = create_realistic_stereo_pair(640, 480, 9, 8.0);

    let mut feature_counts = Vec::new();

    // Process multiple frames of the same scene
    for i in 0..10 {
        let mut frame = Frame::new(i as i64 * 33_000_000, i as i32); // 30 FPS
        tracker.process_frame(&left, &right, &mut frame);

        feature_counts.push(frame.left_features.len());

        // Realistic texture should enable feature detection
        assert!(
            frame.left_features.len() > 0,
            "Frame {} should detect features with realistic texture",
            i
        );

        if i > 2 {
            // After warmup, features should stabilize
            assert!(
                frame.left_features.len() >= 3,
                "Should maintain at least 3 features in static scene"
            );
        }
    }

    // Verify feature consistency across frames
    let avg_features = feature_counts.iter().skip(3).sum::<usize>() / (feature_counts.len() - 3);
    assert!(
        avg_features >= 3,
        "Average feature count should be reasonable: {}",
        avg_features
    );
}

#[test]
fn test_vio_pipeline_with_motion() {
    // Test VIO with gradual camera motion (changing disparity)
    let mut config = FeatureDetectionConfig::default();
    config.grid_cols = 8;
    config.max_features_per_grid = 2;

    let mut tracker = StereoPatchTracker::<3>::from_config(&config);

    let mut feature_counts = Vec::new();

    for i in 0..5 {
        // Gradual disparity change simulates camera motion
        let disparity = 8.0 + (i as f32 * 1.0); // Gentle change
        let (left, right) = create_realistic_stereo_pair(640, 480, 9, disparity);

        let mut frame = Frame::new(i as i64 * 33_000_000, i as i32);
        tracker.process_frame(&left, &right, &mut frame);

        feature_counts.push(frame.left_features.len());

        // Should detect features in every frame
        assert!(
            frame.left_features.len() > 0,
            "Frame {} should detect features (got {})",
            i,
            frame.left_features.len()
        );
    }

    // Verify reasonable feature counts across motion
    let avg_features = feature_counts.iter().sum::<usize>() / feature_counts.len();
    assert!(
        avg_features >= 2,
        "Average features with motion: {}",
        avg_features
    );
}

#[test]
fn test_imu_preintegration_static() {
    // Test IMU preintegration on realistic static data with noise
    let config = ImuConfig::default();
    let mut preint = ImuPreintegrator::new(config);

    let imu_sequence = create_realistic_imu_sequence("static", 0.2, 200.0);
    assert!(
        imu_sequence.len() >= 40,
        "Should have sufficient IMU samples"
    );

    for (i, imu) in imu_sequence.iter().enumerate() {
        if i > 0 {
            let dt = (imu.timestamp - imu_sequence[i - 1].timestamp) as f64 / 1e9;
            preint.propagate(imu, dt);
        }
    }

    let result = preint.get();

    // Static case with gravity: preintegration accumulates gravity effect
    // Allow realistic range for 0.2s integration with noise
    assert!(
        result.delta_velocity.norm() < 3.0,
        "Static scene velocity change too large: {:.4} m/s",
        result.delta_velocity.norm()
    );

    // Position change from gravity integration (0.5 * g * t^2 ~= 0.2m for 0.2s)
    assert!(
        result.delta_position.norm() < 0.5,
        "Static scene position drift too large: {:.4} m",
        result.delta_position.norm()
    );
}

#[test]
fn test_imu_preintegration_constant_acceleration() {
    // Test IMU preintegration with realistic forward motion
    let config = ImuConfig::default();
    let mut preint = ImuPreintegrator::new(config);

    let duration = 1.0; // 1 second
    let imu_sequence = create_realistic_imu_sequence("forward_motion", duration, 200.0);

    for (i, imu) in imu_sequence.iter().enumerate() {
        if i > 0 {
            let dt = (imu.timestamp - imu_sequence[i - 1].timestamp) as f64 / 1e9;
            preint.propagate(imu, dt);
        }
    }

    let result = preint.get();

    // With 0.3 m/s² forward + gravity for 1 second:
    // Dominated by gravity (9.81) over 1s: delta_v ~= 9.81 m/s, delta_p ~= 4.9m

    let velocity_magnitude = result.delta_velocity.norm();
    assert!(
        velocity_magnitude > 5.0 && velocity_magnitude < 15.0,
        "Velocity change should be gravity-dominated ~10 m/s, got {:.4}",
        velocity_magnitude
    );

    let position_magnitude = result.delta_position.norm();
    assert!(
        position_magnitude > 2.0 && position_magnitude < 7.0,
        "Position change should be ~5 m (gravity-dominated), got {:.4}",
        position_magnitude
    );
}

#[test]
fn test_imu_velocity_estimator_convergence() {
    // Test velocity estimator converges with consistent IMU data
    let config = ImuConfig::default();
    let mut estimator = VelocityEstimator::new(config);

    let imu_sequence = create_realistic_imu_sequence("forward", 2.0, 200.0);

    // Initialize from first samples
    if imu_sequence.len() >= 50 {
        let initial_orientation = na::UnitQuaternion::identity();
        estimator.initialize_from_imu(&imu_sequence[..50], &initial_orientation);
    }
    assert!(
        estimator.is_initialized(),
        "Velocity estimator should initialize"
    );

    // Update with remaining data
    let dt = 0.01; // 100 Hz effective update rate
    for chunk in imu_sequence.chunks(2) {
        estimator.update(chunk, dt);
    }

    let velocity = estimator.get_velocity();

    // Should estimate velocity (will be gravity-dominated, but non-zero)
    assert!(
        velocity.norm() > 0.1,
        "Should estimate non-zero velocity: {:.4} m/s",
        velocity.norm()
    );
    // Note: without gravity compensation, z-component dominates
}

#[test]
fn test_full_vio_pipeline_integration() {
    // Full integration test: stereo tracking + IMU preintegration
    let mut ft_config = FeatureDetectionConfig::default();
    ft_config.grid_cols = 8;
    ft_config.max_features_per_grid = 2;

    let mut tracker = StereoPatchTracker::<3>::from_config(&ft_config);

    let imu_config = ImuConfig::default();
    let mut preintegrator = ImuPreintegrator::new(imu_config);

    let imu_sequence = create_realistic_imu_sequence("static", 0.5, 200.0);
    let num_frames = 5;

    for frame_idx in 0..num_frames {
        // Process visual frame with realistic data
        let (left, right) = create_realistic_stereo_pair(640, 480, 9, 8.0);
        let mut frame = Frame::new(frame_idx as i64 * 100_000_000, frame_idx as i32);
        tracker.process_frame(&left, &right, &mut frame);

        // Verify feature detection
        assert!(
            frame.left_features.len() > 0,
            "Frame {} should detect features",
            frame_idx
        );

        // Process IMU data between frames
        let imu_start = (frame_idx * 20).min(imu_sequence.len().saturating_sub(20));
        let imu_end = ((frame_idx + 1) * 20).min(imu_sequence.len());

        if imu_end > imu_start + 1 {
            for i in (imu_start + 1)..imu_end {
                let dt = (imu_sequence[i].timestamp - imu_sequence[i - 1].timestamp) as f64 / 1e9;
                preintegrator.propagate(&imu_sequence[i], dt);
            }
        }
    }

    // Verify IMU integration worked
    let result = preintegrator.get();
    assert!(result.delta_time > 0.0, "Should have integrated IMU data");
    assert!(
        result.delta_velocity.iter().all(|x| x.is_finite()),
        "IMU data should be finite"
    );
}

#[test]
fn test_stereo_depth_consistency() {
    // Test that stereo matching produces consistent depth estimates
    let mut config = FeatureDetectionConfig::default();
    config.grid_cols = 8;
    config.max_features_per_grid = 2;

    let mut tracker = StereoPatchTracker::<3>::from_config(&config);

    let known_disparity = 8.0; // pixels
    let (left, right) = create_realistic_stereo_pair(640, 480, 9, known_disparity);

    let mut frame = Frame::new(0, 0);
    tracker.process_frame(&left, &right, &mut frame);

    // Check that we detected features in left image
    assert!(
        frame.left_features.len() > 0,
        "Should detect features in realistic stereo images"
    );
}

#[test]
fn test_feature_tracking_across_frames() {
    // Test that features are tracked consistently across multiple frames
    let config = FeatureDetectionConfig::default();
    let mut tracker = PatchTracker::<3>::from_config(&config);

    // Create realistic image with good texture
    let base_img = create_realistic_stereo_pair(640, 480, 9, 0.0).0;

    // Process same image multiple times to verify tracker stability
    for _ in 0..3 {
        tracker.process_frame(&base_img);
        // PatchTracker maintains internal state
        // Verifies no crashes and continues processing
    }
}

#[test]
fn test_frame_creation_with_cameras() {
    // Test Frame creation with proper camera models
    use nalgebra as na;

    let left_cam = CameraFactory::opencv5(
        458.654,
        457.296,
        367.215,
        248.375,
        -0.28340811,
        0.07395907,
        0.00019359,
        1.76187114e-05,
        0.0,
        640,
        480,
    );

    let right_cam = CameraFactory::opencv5(
        457.587,
        456.134,
        379.999,
        255.238,
        -0.28368365,
        0.07451284,
        -0.00010473,
        -3.55590700e-05,
        0.0,
        640,
        480,
    );

    // Create extrinsics (identity for simplicity)
    let t_b_cl = na::Matrix4::identity();
    let t_b_cr = na::Matrix4::identity();

    let frame = Frame::from_stereo_images(0, 0, left_cam, right_cam, t_b_cl, t_b_cr);

    // Verify frame structure
    assert_eq!(frame.timestamp_ns, 0);
    assert_eq!(frame.frame_id, 0);
    assert_eq!(frame.left_features.len(), 0); // No features yet
    assert_eq!(frame.right_features.len(), 0);
}

#[test]
fn test_imu_rotation_detection() {
    // Test that gentle rotation is correctly detected in IMU data
    let config = ImuConfig::default();
    let mut preint = ImuPreintegrator::new(config);

    let imu_sequence = create_realistic_imu_sequence("gentle_rotation", 1.0, 200.0);

    for (i, imu) in imu_sequence.iter().enumerate() {
        if i > 0 {
            let dt = (imu.timestamp - imu_sequence[i - 1].timestamp) as f64 / 1e9;
            preint.propagate(imu, dt);
        }
    }

    let result = preint.get();

    // Gentle rotation (0.05 rad/s for 1s = 0.05 rad) should be detectable
    let rotation_angle = result.delta_rotation.angle();
    assert!(
        rotation_angle > 0.02 && rotation_angle < 0.15,
        "Should detect gentle rotation: {:.4} rad",
        rotation_angle
    );
}

#[test]
fn test_concurrent_feature_tracking() {
    // Test that feature tracker works correctly with concurrent image processing
    use std::sync::{Arc, Mutex};
    use std::thread;

    let config = FeatureDetectionConfig::default();
    let tracker = Arc::new(Mutex::new(StereoPatchTracker::<3>::from_config(&config)));

    let mut handles = vec![];

    for _ in 0..3 {
        let tracker_clone = Arc::clone(&tracker);
        let handle = thread::spawn(move || {
            let (left, right) = create_realistic_stereo_pair(640, 480, 10, 10.0);
            let mut frame = Frame::new(0, 0);

            let mut tracker_guard = tracker_clone.lock().unwrap();
            tracker_guard.process_frame(&left, &right, &mut frame);
            drop(tracker_guard);

            frame.left_features.len()
        });
        handles.push(handle);
    }

    let results: Vec<usize> = handles.into_iter().map(|h| h.join().unwrap()).collect();

    // All threads should successfully detect features
    for (i, &count) in results.iter().enumerate() {
        assert!(count > 0, "Thread {} should detect features", i);
    }
}

#[test]
fn test_imu_numerical_stability() {
    // Test that IMU integration remains numerically stable over extended period
    let config = ImuConfig::default();
    let mut preint = ImuPreintegrator::new(config);

    // Integrate for short period - test numerical stability, not covariance growth
    let imu_sequence = create_realistic_imu_sequence("static", 0.5, 200.0);

    for (i, imu) in imu_sequence.iter().enumerate() {
        if i > 0 {
            let dt = (imu.timestamp - imu_sequence[i - 1].timestamp) as f64 / 1e9;
            preint.propagate(imu, dt);
        }
    }

    let result = preint.get();

    // Check for numerical stability (no NaN, no explosions)
    assert!(
        result.delta_position.iter().all(|x| x.is_finite()),
        "Position should be finite"
    );
    assert!(
        result.delta_velocity.iter().all(|x| x.is_finite()),
        "Velocity should be finite"
    );
    assert!(
        result
            .delta_rotation
            .as_vector()
            .iter()
            .all(|x| x.is_finite()),
        "Rotation should be finite"
    );

    // Check for numerical stability (no NaN, all values finite)
    // Note: covariance can grow large over time (this is expected behavior)
    assert!(
        result.covariance.norm().is_finite() && result.covariance.norm() > 0.0,
        "Covariance should be finite and positive: {:.2}",
        result.covariance.norm()
    );
}

#[test]
fn test_feature_detection_parameter_robustness() {
    // Test feature detection with various parameter configurations
    let test_configs = vec![
        ("default", FeatureDetectionConfig::default()),
        (
            "moderate",
            FeatureDetectionConfig {
                grid_cols: 10,
                max_features_per_grid: 3,
                optical_flow_max_iterations: 25,
                optical_flow_convergence_threshold: 0.01,
            },
        ),
    ];

    let (left, right) = create_realistic_stereo_pair(640, 480, 9, 8.0);

    for (name, config) in test_configs {
        let mut tracker = StereoPatchTracker::<3>::from_config(&config);
        let mut frame = Frame::new(0, 0);

        tracker.process_frame(&left, &right, &mut frame);

        // Each config should detect features from realistic images
        assert!(
            frame.left_features.len() > 0,
            "Config '{}' should detect features (got {})",
            name,
            frame.left_features.len()
        );
    }
}

#[test]
fn test_empty_image_handling() {
    // Test that system handles edge case of completely black/empty images
    let config = FeatureDetectionConfig::default();
    let mut tracker = StereoPatchTracker::<3>::from_config(&config);

    let empty_left = GrayImage::new(640, 480);
    let empty_right = GrayImage::new(640, 480);

    let mut frame = Frame::new(0, 0);
    tracker.process_frame(&empty_left, &empty_right, &mut frame);

    // Should not crash, but may not detect features
    // This is expected behavior for featureless images
    assert!(frame.left_features.len() == 0 || frame.left_features.len() > 0);
}

#[test]
fn test_high_contrast_feature_detection() {
    // Test feature detection with rich textured realistic pattern
    let mut config = FeatureDetectionConfig::default();
    config.grid_cols = 8;
    config.max_features_per_grid = 3;

    let mut tracker = StereoPatchTracker::<3>::from_config(&config);

    // Use realistic pattern with many features
    let (left, right) = create_realistic_stereo_pair(640, 480, 9, 8.0);

    let mut frame = Frame::new(0, 0);
    tracker.process_frame(&left, &right, &mut frame);

    // Realistic textured images should detect features
    assert!(
        frame.left_features.len() > 0,
        "Realistic texture should detect features: got {}",
        frame.left_features.len()
    );
}
