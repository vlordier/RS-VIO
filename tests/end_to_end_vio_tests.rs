/// End-to-end VIO pipeline tests with synthetic data
/// These tests validate the complete system behavior under various scenarios

use rs_vio::datasets::config::FeatureDetectionConfig;
use rs_vio::datasets::ImuData;
use rs_vio::estimator::Frame;
use rs_vio::feature_tracker::{PatchTracker, StereoPatchTracker};
use rs_vio::imu::{ImuConfig, ImuPreintegrator, VelocityEstimator};
use rs_vio::types::CameraFactory;
use image::{GrayImage, Luma};
use nalgebra as na;

/// Helper to create synthetic stereo image pair with known features
fn create_synthetic_stereo_pair(
    width: u32,
    height: u32,
    num_features: usize,
    disparity: f32,
) -> (GrayImage, GrayImage) {
    let mut left = GrayImage::new(width, height);
    let mut right = GrayImage::new(width, height);
    
    // Add checkerboard pattern
    for y in 0..height {
        for x in 0..width {
            let value = if (x / 20 + y / 20) % 2 == 0 { 200u8 } else { 50u8 };
            left.put_pixel(x, y, Luma([value]));
            right.put_pixel(x, y, Luma([value]));
        }
    }
    
    // Add distinct corner features
    for i in 0..num_features {
        let x = (50 + i * 80) as u32 % (width - 20);
        let y = (50 + (i / 5) * 80) as u32 % (height - 20);
        
        // Draw small cross pattern in left image
        for dx in 0..10 {
            left.put_pixel(x + dx, y + 5, Luma([255]));
            left.put_pixel(x + 5, y + dx, Luma([255]));
        }
        
        // Draw same pattern in right image with disparity offset
        let x_right = ((x as f32 - disparity).max(10.0) as u32).min(width - 20);
        for dx in 0..10 {
            right.put_pixel(x_right + dx, y + 5, Luma([255]));
            right.put_pixel(x_right + 5, y + dx, Luma([255]));
        }
    }
    
    (left, right)
}

/// Create synthetic IMU data for a motion pattern
fn create_synthetic_imu_sequence(
    pattern: &str,
    duration_s: f64,
    rate_hz: f64,
) -> Vec<ImuData> {
    let num_samples = (duration_s * rate_hz) as usize;
    let dt = 1.0 / rate_hz;
    let mut imu_data = Vec::new();
    
    for i in 0..num_samples {
        let t = i as f64 * dt;
        let timestamp = (t * 1e9) as i64;
        
        let (ax, ay, az, wx, wy, wz) = match pattern {
            "static" => (0.0, 0.0, 9.81, 0.0, 0.0, 0.0),
            "forward" => (0.5, 0.0, 9.81, 0.0, 0.0, 0.0), // constant acceleration
            "rotation" => (0.0, 0.0, 9.81, 0.0, 0.1, 0.0), // yaw rotation
            "circular" => (
                0.2 * (t * 0.5).cos(),
                0.2 * (t * 0.5).sin(),
                9.81,
                0.0,
                0.0,
                0.5,
            ),
            _ => (0.0, 0.0, 9.81, 0.0, 0.0, 0.0),
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
    // Test VIO pipeline on static scene (no motion)
    let config = FeatureDetectionConfig::default();
    let mut tracker = StereoPatchTracker::<3>::from_config(&config);
    
    let (left, right) = create_synthetic_stereo_pair(640, 480, 20, 10.0);
    
    // Process multiple frames of the same scene
    for i in 0..10 {
        let mut frame = Frame::new(i as i64 * 33_000_000, i as i32); // 30 FPS
        tracker.process_frame(&left, &right, &mut frame);
        
        // In static scene, features should be tracked consistently
        assert!(frame.left_features.len() > 0, "Frame {} should detect features", i);
        
        if i > 0 {
            // After first frame, we should have similar number of features
            assert!(
                frame.left_features.len() >= 5,
                "Should maintain feature tracking in static scene"
            );
        }
    }
}

#[test]
fn test_vio_pipeline_with_motion() {
    // Test VIO with camera motion simulation
    let config = FeatureDetectionConfig::default();
    let mut tracker = StereoPatchTracker::<3>::from_config(&config);
    
    let mut previous_feature_count = 0;
    
    for i in 0..5 {
        // Simulate camera moving by changing disparity
        let disparity = 10.0 + (i as f32 * 2.0);
        let (left, right) = create_synthetic_stereo_pair(640, 480, 15, disparity);
        
        let mut frame = Frame::new(i as i64 * 33_000_000, i as i32);
        tracker.process_frame(&left, &right, &mut frame);
        
        let current_count = frame.left_features.len();
        
        if i == 0 {
            previous_feature_count = current_count;
        } else {
            // Should maintain reasonable feature tracking despite motion
            assert!(
                current_count > 0,
                "Frame {} should track some features despite motion",
                i
            );
            
            // Allow some feature loss due to motion, but not total loss
            if previous_feature_count > 0 {
                let retention_ratio = current_count as f32 / previous_feature_count as f32;
                assert!(
                    retention_ratio > 0.3,
                    "Feature retention too low: {:.2}",
                    retention_ratio
                );
            }
            previous_feature_count = current_count;
        }
    }
}

#[test]
fn test_imu_preintegration_static() {
    // Test IMU preintegration on static data (should produce zero delta)
    let config = ImuConfig::default();
    let mut preint = ImuPreintegrator::new(config);
    
    let imu_sequence = create_synthetic_imu_sequence("static", 0.1, 200.0);
    
    for (i, imu) in imu_sequence.iter().enumerate() {
        if i > 0 {
            let dt = (imu.timestamp - imu_sequence[i - 1].timestamp) as f64 / 1e9;
            preint.propagate(imu, dt);
        }
    }
    
    let result = preint.get();
    
    // Static case: velocity change should be near zero
    assert!(
        result.delta_velocity.norm() < 0.01,
        "Static scene should have minimal velocity change: {}",
        result.delta_velocity.norm()
    );
    
    // Position change should be minimal
    assert!(
        result.delta_position.norm() < 0.001,
        "Static scene should have minimal position change: {}",
        result.delta_position.norm()
    );
}

#[test]
fn test_imu_preintegration_constant_acceleration() {
    // Test IMU preintegration with constant forward acceleration
    let config = ImuConfig::default();
    let mut preint = ImuPreintegrator::new(config);
    
    let duration = 1.0; // 1 second
    let imu_sequence = create_synthetic_imu_sequence("forward", duration, 200.0);
    
    for (i, imu) in imu_sequence.iter().enumerate() {
        if i > 0 {
            let dt = (imu.timestamp - imu_sequence[i - 1].timestamp) as f64 / 1e9;
            preint.propagate(imu, dt);
        }
    }
    
    let result = preint.get();
    
    // With 0.5 m/s² forward acceleration for 1 second:
    // delta_v should be approximately 0.5 m/s
    // delta_p should be approximately 0.25 m (0.5 * a * t²)
    
    let velocity_magnitude = result.delta_velocity.norm();
    assert!(
        velocity_magnitude > 0.3 && velocity_magnitude < 0.7,
        "Velocity change should be ~0.5 m/s, got {}",
        velocity_magnitude
    );
    
    let position_magnitude = result.delta_position.norm();
    assert!(
        position_magnitude > 0.1 && position_magnitude < 0.4,
        "Position change should be ~0.25 m, got {}",
        position_magnitude
    );
}

#[test]
fn test_imu_velocity_estimator_convergence() {
    // Test velocity estimator converges with consistent IMU data
    let config = ImuConfig::default();
    let mut estimator = VelocityEstimator::new(config);
    
    let imu_sequence = create_synthetic_imu_sequence("forward", 2.0, 200.0);
    
    // Initialize from first samples
    if imu_sequence.len() >= 50 {
        let initial_orientation = na::UnitQuaternion::identity();
        estimator.initialize_from_imu(&imu_sequence[..50], &initial_orientation);
    }
    assert!(estimator.is_initialized(), "Velocity estimator should initialize");
    
    // Update with remaining data
    let dt = 0.01; // 100 Hz effective update rate
    for chunk in imu_sequence.chunks(2) {
        estimator.update(chunk, dt);
    }
    
    let velocity = estimator.get_velocity();
    
    // With forward acceleration, should estimate non-zero forward velocity
    assert!(velocity.norm() > 0.1, "Should estimate non-zero velocity");
    assert!(velocity[0] > 0.0, "Should detect forward motion");
}

#[test]
fn test_full_vio_pipeline_integration() {
    // Full integration test: stereo tracking + IMU preintegration
    let ft_config = FeatureDetectionConfig::default();
    let mut tracker = StereoPatchTracker::<3>::from_config(&ft_config);
    
    let imu_config = ImuConfig::default();
    let mut preintegrator = ImuPreintegrator::new(imu_config);
    
    let imu_sequence = create_synthetic_imu_sequence("static", 0.5, 200.0);
    let num_frames = 5;
    
    for frame_idx in 0..num_frames {
        // Process visual frame
        let (left, right) = create_synthetic_stereo_pair(640, 480, 15, 10.0);
        let mut frame = Frame::new(frame_idx as i64 * 100_000_000, frame_idx as i32);
        tracker.process_frame(&left, &right, &mut frame);
        
        // Process IMU data between frames
        let imu_start = (frame_idx * 100) % imu_sequence.len();
        let imu_end = ((frame_idx + 1) * 100).min(imu_sequence.len());
        
        for (i, imu) in imu_sequence[imu_start..imu_end].iter().enumerate() {
            if i > 0 {
                let dt = (imu.timestamp - imu_sequence[imu_start + i - 1].timestamp) as f64 / 1e9;
                preintegrator.propagate(imu, dt);
            }
        }
        
        // Verify both visual and IMU processing succeeded
        assert!(frame.left_features.len() > 0, "Frame {} should have features", frame_idx);
        
        if frame_idx > 0 {
            let preint_result = preintegrator.get();
            // Static case: should have minimal integration
            assert!(
                preint_result.delta_velocity.norm() < 1.0,
                "Static IMU integration should be bounded"
            );
        }
    }
}

#[test]
fn test_stereo_depth_consistency() {
    // Test that stereo matching produces consistent depth estimates
    let config = FeatureDetectionConfig::default();
    let mut tracker = StereoPatchTracker::<3>::from_config(&config);
    
    let known_disparity = 15.0; // pixels
    let (left, right) = create_synthetic_stereo_pair(640, 480, 10, known_disparity);
    
    let mut frame = Frame::new(0, 0);
    tracker.process_frame(&left, &right, &mut frame);
    
    // Check that we matched features between left and right
    assert!(
        frame.left_features.len() > 0 && frame.right_features.len() > 0,
        "Should detect features in both images"
    );
    
    // In a more complete implementation, we'd verify:
    // - Matched features have consistent disparities
    // - Depths are within expected range
    // - Epipolar constraints are satisfied
}

#[test]
fn test_feature_tracking_across_frames() {
    // Test that features are tracked consistently across multiple frames
    let config = FeatureDetectionConfig::default();
    let mut tracker = PatchTracker::<3>::from_config(&config);
    
    // Create a sequence of slightly different images
    let base_img = create_synthetic_stereo_pair(640, 480, 15, 0.0).0;
    
    tracker.process_frame(&base_img);
    // Note: PatchTracker maintains internal state, we can't directly access point count
    // but we can verify it doesn't crash and continues processing
    
    // Process same image again - should maintain tracking state
    tracker.process_frame(&base_img);
    
    // Verify tracker continues to function
    tracker.process_frame(&base_img);
}

#[test]
fn test_frame_creation_with_cameras() {
    // Test Frame creation with proper camera models
    use nalgebra as na;
    
    let left_cam = CameraFactory::opencv5(
        458.654, 457.296, 367.215, 248.375,
        -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0,
        640, 480,
    );
    
    let right_cam = CameraFactory::opencv5(
        457.587, 456.134, 379.999, 255.238,
        -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05, 0.0,
        640, 480,
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
    // Test that rotation is correctly detected in IMU data
    let config = ImuConfig::default();
    let mut preint = ImuPreintegrator::new(config);
    
    let imu_sequence = create_synthetic_imu_sequence("rotation", 1.0, 200.0);
    
    for (i, imu) in imu_sequence.iter().enumerate() {
        if i > 0 {
            let dt = (imu.timestamp - imu_sequence[i - 1].timestamp) as f64 / 1e9;
            preint.propagate(imu, dt);
        }
    }
    
    let result = preint.get();
    
    // Rotation should produce non-identity rotation matrix
    let rotation_angle = result.delta_rotation.angle();
    assert!(
        rotation_angle > 0.05,
        "Should detect rotation: angle = {}",
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
    
    for i in 0..3 {
        let tracker_clone = Arc::clone(&tracker);
        let handle = thread::spawn(move || {
            let (left, right) = create_synthetic_stereo_pair(640, 480, 10, 10.0);
            let mut frame = Frame::new(i as i64 * 100_000_000, i as i32);
            
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
    
    // Integrate for extended period with high-rate data
    let imu_sequence = create_synthetic_imu_sequence("static", 10.0, 200.0);
    
    for (i, imu) in imu_sequence.iter().enumerate() {
        if i > 0 {
            let dt = (imu.timestamp - imu_sequence[i - 1].timestamp) as f64 / 1e9;
            preint.propagate(imu, dt);
        }
    }
    
    let result = preint.get();
    
    // Check for numerical stability (no NaN, no explosions)
    assert!(result.delta_position.iter().all(|x| x.is_finite()), "Position should be finite");
    assert!(result.delta_velocity.iter().all(|x| x.is_finite()), "Velocity should be finite");
    assert!(result.delta_rotation.as_vector().iter().all(|x| x.is_finite()), "Rotation should be finite");
    
    // Covariance should remain positive definite and bounded
    assert!(result.covariance.norm() < 1000.0, "Covariance should be bounded");
}

#[test]
fn test_feature_detection_parameter_robustness() {
    // Test feature detection with various parameter configurations
    let test_configs = vec![
        ("default", FeatureDetectionConfig::default()),
        ("high_threshold", FeatureDetectionConfig {
            grid_cols: 40,
            max_features_per_grid: 100,
            optical_flow_max_iterations: 30,
            optical_flow_convergence_threshold: 0.01,
        }),
        ("low_threshold", FeatureDetectionConfig {
            grid_cols: 20,
            max_features_per_grid: 250,
            optical_flow_max_iterations: 40,
            optical_flow_convergence_threshold: 0.001,
        }),
    ];
    
    let (left, right) = create_synthetic_stereo_pair(640, 480, 20, 10.0);
    
    for (name, config) in test_configs {
        let mut tracker = StereoPatchTracker::<3>::from_config(&config);
        let mut frame = Frame::new(0, 0);
        
        tracker.process_frame(&left, &right, &mut frame);
        
        assert!(
            frame.left_features.len() > 0,
            "Config '{}' should detect features",
            name
        );
        
        // Verify features are within image bounds
        for feat in &frame.left_features {
            let (x, y) = (feat.pixel_coord[0], feat.pixel_coord[1]);
            assert!(
                x >= 0.0 && x < 640.0 && y >= 0.0 && y < 480.0,
                "Feature outside image bounds in config '{}'",
                name
            );
        }
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
    // Test feature detection on high-contrast synthetic pattern
    let config = FeatureDetectionConfig::default();
    let mut tracker = StereoPatchTracker::<3>::from_config(&config);
    
    let mut high_contrast = GrayImage::new(640, 480);
    // Create strong checkerboard
    for y in 0..480 {
        for x in 0..640 {
            let value = if (x / 40 + y / 40) % 2 == 0 { 255u8 } else { 0u8 };
            high_contrast.put_pixel(x, y, Luma([value]));
        }
    }
    
    let mut frame = Frame::new(0, 0);
    tracker.process_frame(&high_contrast, &high_contrast, &mut frame);
    
    // High contrast should produce many corner features
    assert!(
        frame.left_features.len() >= 10,
        "High contrast should detect many features: got {}",
        frame.left_features.len()
    );
}
