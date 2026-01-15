#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]

use rs_vio::*;

#[test]
fn test_vio_pipeline_integration() {
    // Create a minimal config for testing
    let yaml_config = r#"
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
  pnp_max_iterations: 8
"#;

    let config: datasets::config::Config = serde_yaml::from_str(yaml_config).unwrap();

    // Create estimator
    let mut estimator = estimator::Estimator::new(config, None);
    // Increase timeout to allow for slower test environments
    estimator.set_max_frame_processing_time(std::time::Duration::from_secs(10));
    estimator.set_max_map_points(200);

    // Simulate processing multiple frames
    for frame_id in 0..5 {
        let timestamp_ns = frame_id * 100000000; // 100ms intervals

        // Create dummy stereo images (gray checkerboard pattern)
        let mut left_image = vec![0u8; 640 * 480];
        let mut right_image = vec![0u8; 640 * 480];

        for y in 0..480 {
            for x in 0..640 {
                let idx = y * 640 + x;
                left_image[idx] = if (x / 32 + y / 32) % 2 == 0 { 255 } else { 0 };
                right_image[idx] = if ((x + 10) / 32 + y / 32) % 2 == 0 {
                    255
                } else {
                    0
                }; // slight offset
            }
        }

        // Process frame
        estimator
            .process_frame(&left_image, &right_image, timestamp_ns, None)
            .unwrap_or_else(|e| panic!("Frame {} failed: {}", frame_id, e));
    }

    // Verify that some frames were processed
    assert!(estimator.frame_count() > 0);
}

#[test]
fn test_config_parsing_integration() {
    let config_str = r#"
camera:
  image_width: 1280
  image_height: 720
  left_intrinsics: [600.0, 600.0, 640.0, 360.0]
  left_distortion: [-0.1, 0.05, 0.0, 0.0]
  right_intrinsics: [600.0, 600.0, 640.0, 360.0]
  right_distortion: [-0.1, 0.05, 0.0, 0.0]
  left_model: pinhole-radtan
  right_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.05, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 10
  translation_threshold: 0.2
  rotation_threshold: 0.2
feature_detection:
  grid_size: 15
  max_features_per_grid: 60
  optical_flow_max_iterations: 50
  optical_flow_convergence_threshold: 0.001
optimization:
  bundle_adjustment_max_iterations: 20
  pnp_max_iterations: 15
"#;

    let config: datasets::config::Config = serde_yaml::from_str(config_str).unwrap();

    // Verify parsed values
    assert_eq!(config.camera.image_width, 1280);
    assert_eq!(config.camera.image_height, 720);
    assert_eq!(config.keyframe_management.keyframe_window_size, 10);
    assert_eq!(config.feature_detection.grid_cols, 15);
    assert_eq!(config.optimization.bundle_adjustment_max_iterations, 20);
    assert_eq!(config.optimization.pnp_max_iterations, 15);
}

#[test]
fn test_types_conversion() {
    use rs_vio::types::*;

    let array4x4 = [
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 2.0],
        [0.0, 0.0, 1.0, 3.0],
        [0.0, 0.0, 0.0, 1.0],
    ];

    let matrix = array4x4.to_matrix();
    assert_eq!(matrix[(0, 3)], 1.0);
    assert_eq!(matrix[(1, 3)], 2.0);

    let back_to_array = matrix.to_array();
    assert_eq!(back_to_array, array4x4);

    let vector = [1.0, 2.0, 3.0].to_vector();
    assert_eq!(vector[0], 1.0);
    assert_eq!(vector[2], 3.0);
}
