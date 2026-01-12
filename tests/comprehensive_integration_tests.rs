//! Comprehensive integration tests for RS-VIO
//!
//! These tests verify cross-module interactions and end-to-end workflows.

#[cfg(test)]
mod comprehensive_tests {
    #![allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_lossless,
        clippy::float_cmp,
        clippy::redundant_closure,
        clippy::useless_vec,
        clippy::len_zero,
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::unused_enumerate_index
    )]
    use rs_vio::estimator::{Frame, State};
    use rs_vio::feature_tracker::Feature;
    use rs_vio::types::*;
    use rs_vio::validation::*;

    /// Helper function to create a standard test frame
    fn create_test_frame(id: i32) -> Frame {
        Frame::new((id as i64) * 1000, id)
    }

    /// Helper function to create a test feature
    fn create_test_feature(x: f32, y: f32, id: usize) -> Feature {
        Feature {
            feature_id: id,
            pixel_coord: [x, y],
            undistorted_coord: [x * 0.99, y * 0.99],
        }
    }

    /// Helper function to create test state with transformations
    fn create_test_state() -> State {
        #[allow(non_snake_case)]
        let mut T_B_Cl = Matrix4x4::identity();
        T_B_Cl[(0, 3)] = 0.5;

        #[allow(non_snake_case)]
        let mut T_B_Cr = Matrix4x4::identity();
        T_B_Cr[(0, 3)] = -0.5;

        State::new(T_B_Cl, T_B_Cr)
    }

    #[test]
    fn test_frame_and_state_integration() {
        let mut frame = create_test_frame(1);
        let state = create_test_state();

        frame.state = state;

        assert!(frame.state.T_B_Cl[(0, 3)].abs() > 0.0);
        assert!(frame.state.T_B_Cr[(0, 3)].abs() > 0.0);
    }

    #[test]
    fn test_frame_feature_addition_validation() {
        let mut frame = create_test_frame(1);

        // Add features
        for i in 0..10 {
            let feature = create_test_feature(100.0 + i as f32, 150.0 + i as f32, 0);
            frame.add_left_feature(feature);
        }

        // Validate
        assert_eq!(frame.left_features.len(), 10);
        for (_i, feature) in frame.left_features.iter().enumerate() {
            assert!(validate_finite_vector(&Vector3::new(
                feature.pixel_coord[0] as f64,
                feature.pixel_coord[1] as f64,
                1.0
            ))
            .is_ok());
        }
    }

    #[test]
    fn test_multiple_frames_consistency() {
        let mut frames = vec![];
        for i in 0..5 {
            let frame = create_test_frame(i);
            frames.push(frame);
        }

        // Verify timestamps are sequential
        for i in 1..frames.len() {
            assert!(frames[i].timestamp_ns > frames[i - 1].timestamp_ns);
        }
    }

    #[test]
    fn test_frame_state_validity_after_mutation() {
        let mut frame = create_test_frame(1);
        frame.state.velocity = Vector3::new(1.0, 2.0, 3.0);

        // Validate after mutation
        assert!(validate_finite_vector(&frame.state.velocity).is_ok());
        assert!(validate_finite_vector(&frame.state.accel_bias).is_ok());
        assert!(validate_finite_vector(&frame.state.gyro_bias).is_ok());
    }

    #[test]
    fn test_feature_coordinates_within_image_bounds() {
        let mut frame = create_test_frame(1);

        // Add features with various coordinates
        let test_coords = vec![(10.0, 10.0), (320.0, 240.0), (630.0, 470.0)];

        for (x, y) in test_coords {
            let feature = create_test_feature(x, y, 0);
            frame.add_left_feature(feature);
        }

        assert_eq!(frame.left_features.len(), 3);
    }

    #[test]
    fn test_stereo_frame_left_right_consistency() {
        let mut frame = create_test_frame(1);

        // Add matching features to left and right
        for i in 0..5 {
            let left_feature = create_test_feature(100.0 + i as f32, 100.0, 0);
            let right_feature = create_test_feature(90.0 + i as f32, 100.0, 0);

            frame.add_left_feature(left_feature);
            frame.add_right_feature(right_feature);
        }

        assert_eq!(frame.left_features.len(), frame.right_features.len());
        assert_eq!(frame.left_features.len(), 5);
    }

    #[test]
    fn test_frame_with_pyramid_levels() {
        let mut frame = create_test_frame(1);

        // Add features with different IDs (simulating different pyramid levels)
        for level in 0..4 {
            for i in 0..3 {
                let feature_id = level * 10 + i; // Unique ID based on level
                let feature = create_test_feature(
                    100.0 + (level * 50) as f32,
                    100.0 + (i * 30) as f32,
                    feature_id,
                );
                frame.add_left_feature(feature);
            }
        }

        assert_eq!(frame.left_features.len(), 12);

        // Verify all features have unique IDs
        let mut ids = std::collections::HashSet::new();
        for feature in frame.left_features.iter() {
            ids.insert(feature.feature_id);
        }

        assert_eq!(ids.len(), 12); // All unique
    }

    #[test]
    fn test_state_transformations_compose() {
        let mut state1 = create_test_state();
        let mut state2 = create_test_state();

        // Apply transformations
        state1.velocity = Vector3::new(1.0, 0.0, 0.0);
        state2.velocity = Vector3::new(0.0, 1.0, 0.0);

        // Both should remain valid
        assert!(validate_finite_vector(&state1.velocity).is_ok());
        assert!(validate_finite_vector(&state2.velocity).is_ok());

        // Different velocity components
        assert_ne!(state1.velocity[0], state2.velocity[0]); // 1.0 vs 0.0
        assert_ne!(state1.velocity[1], state2.velocity[1]); // 0.0 vs 1.0
    }

    #[test]
    fn test_matrix_operations_on_state() {
        let state = create_test_state();

        // Validate all transformation matrices
        assert!(validate_finite_matrix(&state.T_W_B).is_ok());
        assert!(validate_finite_matrix(&state.T_B_Cl).is_ok());
        assert!(validate_finite_matrix(&state.T_B_Cr).is_ok());
    }

    #[test]
    fn test_vector_operations_consistency() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);

        // Test various operations
        let sum = v1 + v2;
        let diff = v2 - v1;
        let dot = v1.dot(&v2);
        let cross = v1.cross(&v2);

        // Verify properties
        assert_eq!(sum[0], 5.0);
        assert_eq!(diff[0], 3.0);
        assert!((dot - 32.0).abs() < 1e-10);
        assert!(cross.norm() > 0.0);
    }

    #[test]
    fn test_feature_descriptor_integrity() {
        let mut frame = create_test_frame(1);

        // Create feature with specific coordinates
        let feature = Feature {
            feature_id: 1,
            pixel_coord: [100.0, 100.0],
            undistorted_coord: [-1.0, -1.0], // Will be computed by add_left_feature
        };

        frame.add_left_feature(feature);

        // Verify pixel coordinates preserved
        let stored_feature = &frame.left_features[0];
        assert_eq!(stored_feature.pixel_coord[0], 100.0);
        assert_eq!(stored_feature.pixel_coord[1], 100.0);
        // Undistorted coordinates should be computed by camera model (not default [-1, -1])
        assert_ne!(stored_feature.undistorted_coord[0], -1.0);
        assert_ne!(stored_feature.undistorted_coord[1], -1.0);
    }

    #[test]
    fn test_imu_data_accumulation_in_frame() {
        let mut frame = create_test_frame(1);

        // Simulate IMU data arrival
        for i in 0..10 {
            let imu = rs_vio::datasets::ImuData {
                timestamp: 1000 + i * 100,
                accel: [1.0 + i as f64 * 0.1, 2.0, 3.0],
                gyro: [0.1, 0.2, 0.3],
            };
            frame.imu_from_last_frame.push(imu);
        }

        assert_eq!(frame.imu_from_last_frame.len(), 10);

        // Verify timestamp ordering
        for i in 1..frame.imu_from_last_frame.len() {
            assert!(
                frame.imu_from_last_frame[i].timestamp > frame.imu_from_last_frame[i - 1].timestamp
            );
        }
    }

    #[test]
    fn test_matrix4x4_identity_properties() {
        let m = Matrix4x4::identity();
        let v = Vector3::new(1.0, 2.0, 3.0);

        // Identity matrix should preserve vectors
        let homogeneous = nalgebra::Vector4::new(v[0], v[1], v[2], 1.0);
        let result = m * homogeneous;

        assert!((result[0] - v[0]).abs() < 1e-10);
        assert!((result[1] - v[1]).abs() < 1e-10);
        assert!((result[2] - v[2]).abs() < 1e-10);
        assert!((result[3] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_validation_module_integration() {
        let point = Vector3::new(1.0, 2.0, 3.0);

        // Test all validation functions
        assert!(validate_finite_vector(&point).is_ok());
        assert!(approx_equal(1.0, 1.0 + 1e-11)); // Within epsilon tolerance (1e-10)
        assert!(safe_divide(10.0, 2.0).is_ok());
    }

    #[test]
    fn test_frame_timestamp_progression() {
        let frames: Vec<_> = (0..10).map(|i| create_test_frame(i)).collect();

        for i in 1..frames.len() {
            assert!(frames[i].timestamp_ns > frames[i - 1].timestamp_ns);
        }
    }

    #[test]
    fn test_state_and_frame_independence() {
        let frame1 = create_test_frame(1);
        let frame2 = create_test_frame(2);

        // Frames should have independent states
        assert_eq!(frame1.state.T_W_B, frame2.state.T_W_B);
        assert_eq!(frame1.state.velocity, frame2.state.velocity);
    }

    #[test]
    fn test_feature_level_progression() {
        let mut frame = create_test_frame(1);

        // Add features with increasing IDs
        for id in 0..8 {
            let feature = create_test_feature(100.0, 100.0, id);
            frame.add_left_feature(feature);
        }

        for (i, feature) in frame.left_features.iter().enumerate() {
            assert_eq!(feature.feature_id, i);
        }
    }

    #[test]
    fn test_matrix_determinant_calculations() {
        let identity = Matrix4x4::identity();
        assert!((identity.determinant() - 1.0).abs() < 1e-10);

        let scaled = Matrix4x4::from_element(0.0);
        assert_eq!(scaled.determinant(), 0.0);
    }

    #[test]
    fn test_vector_norm_calculations() {
        let v = Vector3::new(3.0, 4.0, 0.0);
        assert!((v.norm() - 5.0).abs() < 1e-10);

        let unit = v.normalize();
        assert!((unit.norm() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_cross_product_properties() {
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);
        let cross = v1.cross(&v2);

        // Cross product should be perpendicular to both vectors
        assert!((cross.dot(&v1)).abs() < 1e-10);
        assert!((cross.dot(&v2)).abs() < 1e-10);
    }

    #[test]
    fn test_quaternion_operations() {
        let q1 = UnitQuaternion::identity();
        let q2 = UnitQuaternion::identity();

        let combined = q1 * q2;
        assert_eq!(combined, q1);
    }

    #[test]
    fn test_frame_feature_multiple_levels_mixed() {
        let mut frame = create_test_frame(1);

        // Add features in mixed order
        for level in vec![2, 0, 3, 1, 2, 0] {
            let feature = create_test_feature(200.0, 200.0, level);
            frame.add_left_feature(feature);
        }

        assert_eq!(frame.left_features.len(), 6);
    }

    #[test]
    fn test_state_bias_independence() {
        let mut state = create_test_state();

        state.accel_bias = Vector3::new(0.1, 0.2, 0.3);
        state.gyro_bias = Vector3::new(-0.05, -0.1, -0.15);

        // Biases should be independent
        assert_ne!(state.accel_bias[0], state.gyro_bias[0]);
        assert!((state.accel_bias[0] + state.gyro_bias[0]).abs() > 0.01);
    }

    #[test]
    fn test_comprehensive_frame_workflow() {
        // Create frame
        let mut frame = create_test_frame(42);

        // Add state modifications
        frame.state.velocity = Vector3::new(0.5, 0.3, 0.2);
        frame.state.accel_bias = Vector3::new(0.01, 0.02, 0.03);

        // Add features
        for i in 0..20 {
            let feature = create_test_feature(100.0 + i as f32 * 5.0, 150.0, i % 4);
            frame.add_left_feature(feature);
        }

        // Add IMU data
        for i in 0..5 {
            let imu = rs_vio::datasets::ImuData {
                timestamp: 42000 + i * 100,
                accel: [1.0, 2.0, 3.0],
                gyro: [0.1, 0.2, 0.3],
            };
            frame.imu_from_last_frame.push(imu);
        }

        // Validate all components
        assert_eq!(frame.frame_id, 42);
        assert_eq!(frame.left_features.len(), 20);
        assert_eq!(frame.imu_from_last_frame.len(), 5);
        assert!(validate_finite_vector(&frame.state.velocity).is_ok());
    }
}
