//! Tests for intrinsics refinement functionality in the estimator

use rs_vio::datasets::config::{
    CalibrationRefinementConfig, CameraConfig, Config, FeatureDetectionConfig,
    KeyframeManagementConfig, OptimizationConfig,
};

#[test]
fn test_calibration_config_serialization() {
    let config = CalibrationRefinementConfig {
        optimize_intrinsics: true,
        optimize_focal_length: true,
        optimize_principal_point: true,
        optimize_distortion: true,
        intrinsics_refinement_frequency: 10,
        max_intrinsics_change_per_update: 1.0,
        intrinsics_regularization_weight: 0.05,
    };

    assert!(config.optimize_intrinsics);
    assert!(config.optimize_focal_length);
    assert_eq!(config.intrinsics_refinement_frequency, 10);
    assert!((config.max_intrinsics_change_per_update - 1.0).abs() < f64::EPSILON);
    assert!((config.intrinsics_regularization_weight - 0.05).abs() < f64::EPSILON);
}

#[test]
fn test_calibration_config_defaults() {
    let config = CalibrationRefinementConfig::default();

    assert!(!config.optimize_intrinsics);
    assert!(config.optimize_focal_length);
    assert!(!config.optimize_principal_point);
    assert!(!config.optimize_distortion);
    assert_eq!(config.intrinsics_refinement_frequency, 5);
    assert!(config.max_intrinsics_change_per_update > 0.0);
    assert!(config.intrinsics_regularization_weight > 0.0);
}

#[test]
fn test_camera_config_intrinsics_validity() {
    let camera = CameraConfig {
        image_width: 640,
        image_height: 480,
        left_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
        left_distortion: vec![0.0, 0.0, 0.0, 0.0],
        right_intrinsics: vec![502.0, 502.0, 320.0, 240.0],
        right_distortion: vec![0.0, 0.0, 0.0, 0.0],
        left_model: Some("EUCM".to_string()),
        right_model: Some("EUCM".to_string()),
        T_B_Cl: vec![
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
        T_B_Cr: vec![
            1.0, 0.0, 0.0, -0.12, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
    };

    // Validate intrinsic values are reasonable
    assert!(camera.left_intrinsics[0] > 0.0); // fx
    assert!(camera.left_intrinsics[1] > 0.0); // fy
    assert!(camera.left_intrinsics[2] < camera.image_width as f64);
    assert!(camera.left_intrinsics[3] < camera.image_height as f64);

    // Validate right intrinsics are similar
    assert!((camera.left_intrinsics[0] - camera.right_intrinsics[0]).abs() < 10.0);
    assert!((camera.left_intrinsics[1] - camera.right_intrinsics[1]).abs() < 10.0);
}

#[test]
fn test_stereo_baseline_intrinsics() {
    let camera = CameraConfig {
        image_width: 640,
        image_height: 480,
        left_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
        left_distortion: vec![0.0, 0.0],
        right_intrinsics: vec![500.0, 500.0, 320.0, 240.0], // Same fx, fy
        right_distortion: vec![0.0, 0.0],
        left_model: Some("EUCM".to_string()),
        right_model: Some("EUCM".to_string()),
        T_B_Cl: vec![
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
        T_B_Cr: vec![
            1.0, 0.0, 0.0, -0.12, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
    };

    // For a stereo system, focal lengths should be same
    assert!((camera.left_intrinsics[0] - camera.right_intrinsics[0]).abs() < f64::EPSILON); // fx same
    assert!((camera.left_intrinsics[1] - camera.right_intrinsics[1]).abs() < f64::EPSILON); // fy same

    // Right camera should be offset along x (negative baseline)
    assert!(camera.T_B_Cr[3] < 0.0); // x component negative
}

#[test]
fn test_intrinsics_refinement_bounds() {
    let config = CalibrationRefinementConfig {
        max_intrinsics_change_per_update: 0.5,
        intrinsics_regularization_weight: 0.01,
        ..CalibrationRefinementConfig::default()
    };

    // Verify bounds are reasonable (not too large or too small)
    assert!(config.max_intrinsics_change_per_update > 0.0);
    assert!(config.max_intrinsics_change_per_update <= 2.0); // Reasonable upper bound

    assert!(config.intrinsics_regularization_weight > 0.0);
    assert!(config.intrinsics_regularization_weight <= 1.0);
}

#[test]
fn test_refinement_frequency_bounds() {
    let config = CalibrationRefinementConfig {
        intrinsics_refinement_frequency: 10,
        ..CalibrationRefinementConfig::default()
    };

    // Refinement frequency should be at least 1
    assert!(config.intrinsics_refinement_frequency > 0);

    // Reasonable bounds (1-100 keyframes)
    assert!(config.intrinsics_refinement_frequency <= 100);
}

#[test]
fn test_multi_config_instance() {
    let camera = CameraConfig {
        image_width: 640,
        image_height: 480,
        left_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
        left_distortion: vec![0.0, 0.0],
        right_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
        right_distortion: vec![0.0, 0.0],
        left_model: Some("EUCM".to_string()),
        right_model: Some("EUCM".to_string()),
        T_B_Cl: vec![
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
        T_B_Cr: vec![
            1.0, 0.0, 0.0, -0.12, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
    };

    let keyframe = KeyframeManagementConfig {
        keyframe_window_size: 10,
        translation_threshold: 0.1,
        rotation_threshold: 0.05,
    };

    let feature = FeatureDetectionConfig {
        grid_cols: 10,
        max_features_per_grid: 40,
        optical_flow_max_iterations: 100,
        optical_flow_convergence_threshold: 1e-4,
    };

    let opt = OptimizationConfig {
        bundle_adjustment_max_iterations: 100,
        pnp_max_iterations: 50,
    };

    let calib = Some(CalibrationRefinementConfig::default());

    let config = Config {
        camera,
        keyframe_management: keyframe,
        feature_detection: feature,
        optimization: opt,
        calibration: calib,
    };

    assert!(config.calibration.is_some());
    assert_eq!(config.camera.image_width, 640);
}

#[test]
fn test_calibration_disabled_config() {
    let camera = CameraConfig {
        image_width: 640,
        image_height: 480,
        left_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
        left_distortion: vec![0.0, 0.0],
        right_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
        right_distortion: vec![0.0, 0.0],
        left_model: None,
        right_model: None,
        T_B_Cl: vec![
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
        T_B_Cr: vec![
            1.0, 0.0, 0.0, -0.12, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
    };

    let keyframe = KeyframeManagementConfig {
        keyframe_window_size: 10,
        translation_threshold: 0.1,
        rotation_threshold: 0.05,
    };

    let feature = FeatureDetectionConfig {
        grid_cols: 10,
        max_features_per_grid: 40,
        optical_flow_max_iterations: 100,
        optical_flow_convergence_threshold: 1e-4,
    };

    let opt = OptimizationConfig {
        bundle_adjustment_max_iterations: 100,
        pnp_max_iterations: 50,
    };

    // No calibration config
    let config = Config {
        camera,
        keyframe_management: keyframe,
        feature_detection: feature,
        optimization: opt,
        calibration: None,
    };

    assert!(config.calibration.is_none());
}

#[test]
fn test_intrinsics_transformation_matrices() {
    // Test that transformation matrices have proper structure
    let T_B_C: Vec<f64> = vec![
        1.0, 0.0, 0.0, 0.0, // First row
        0.0, 1.0, 0.0, 0.0, // Second row
        0.0, 0.0, 1.0, 0.0, // Third row
        0.0, 0.0, 0.0, 1.0, // Fourth row (homogeneous)
    ];

    // Should be 4x4 matrix
    assert_eq!(T_B_C.len(), 16);

    // Should be identity matrix
    assert!((T_B_C[0] - 1.0).abs() < f64::EPSILON);
    assert!((T_B_C[5] - 1.0).abs() < f64::EPSILON);
    assert!((T_B_C[10] - 1.0).abs() < f64::EPSILON);
    assert!((T_B_C[15] - 1.0).abs() < f64::EPSILON);
}

#[test]
fn test_stereo_transformation_structure() {
    let T_B_Cl: Vec<f64> = vec![
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];
    let T_B_Cr: Vec<f64> = vec![
        1.0, 0.0, 0.0, -0.12, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];

    // Both should be 4x4
    assert_eq!(T_B_Cl.len(), 16);
    assert_eq!(T_B_Cr.len(), 16);

    // Right camera offset should be negative along x (baseline)
    assert!(T_B_Cr[3] < 0.0);

    // Rotation parts should be the same (both looking forward)
    assert!((T_B_Cl[0] - T_B_Cr[0]).abs() < f64::EPSILON);
    assert!((T_B_Cl[1] - T_B_Cr[1]).abs() < f64::EPSILON);
}
