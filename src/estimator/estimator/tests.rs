#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::super::*;
    use crate::datasets::config::Config;
    use serde_yaml;

    fn create_test_config() -> Config {
        // Create a minimal config for testing
        let yaml = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
    grid_size: 10
    max_features_per_grid: 50
    optical_flow_max_iterations: 30
    optical_flow_convergence_threshold: 0.01
optimization:
    bundle_adjustment_max_iterations: 10
    pnp_max_iterations: 5
"#;
        serde_yaml::from_str(yaml).unwrap()
    }

    #[test]
    fn test_estimator_creation() {
        let config = create_test_config();
        let estimator = Estimator::new(config, None);
        assert_eq!(estimator.frame_count(), 0);
    }

    #[test]
    fn test_camera_model_creation() {
        let config = create_test_config();
        let (_left_cam, _right_cam) = crate::datasets::create_camera_models_from_config(&config);
        // Cameras are created successfully if no panic
    }

    #[test]
    fn test_process_frame_basic() {
        let config = create_test_config();
        let mut estimator = Estimator::new(config, None);
        // Increase timeout to allow for slower test environments
        estimator.set_max_frame_processing_time(std::time::Duration::from_secs(10));

        // Create dummy image data
        let left_image = vec![128u8; 640 * 480];
        let right_image = vec![128u8; 640 * 480];
        let timestamp_ns = 1000000000; // 1 second

        // This should not error
        estimator
            .process_frame(&left_image, &right_image, timestamp_ns, None)
            .expect("process_frame should succeed with synthetic input");
    }

    #[test]
    fn test_frame_creation() {
        let config = create_test_config();
        let (left_cam, right_cam) = crate::datasets::create_camera_models_from_config(&config);
        let t_b_cl = nalgebra::Matrix4::identity();
        let t_b_cr = nalgebra::Matrix4::from_row_slice(&[
            1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ]);

        let frame = crate::estimator::Frame::from_stereo_images(1000000000, 0, left_cam, right_cam, t_b_cl, t_b_cr);

        assert_eq!(frame.frame_id, 0);
        assert_eq!(frame.timestamp_ns, 1000000000);
    }
}
