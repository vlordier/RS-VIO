#![cfg(feature = "rerun")]

//! Integration tests for Rerun viewer functionality
//! Tests the core visualization methods that are actually used in production

use rs_vio::viewers::{Viewer, RerunViewer};
use nalgebra::Matrix4;

#[test]
fn test_rerun_viewer_initialization() {
    let mut viewer = RerunViewer::new();
    let result = viewer.initialize();
    assert!(
        result.is_ok(),
        "Viewer should initialize successfully in test environment"
    );
}

#[test]
fn test_log_pose() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    // Create a simple identity pose
    let identity_pose = Matrix4::identity();

    // Should not panic when logging a pose
    viewer.log_pose(identity_pose, "test/pose");
}

#[test]
fn test_log_image_raw() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    // Create a small 2x2 grayscale image
    let image = vec![0_u8, 255, 128, 64];
    viewer.log_image_raw(&image, 2, 2, "test/image_raw");
}

#[test]
fn test_log_image_equalized() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    // Create a small 2x2 grayscale image
    let image = vec![0_u8, 255, 128, 64];
    viewer.log_image_equalized(&image, 2, 2, "test/image_equalized");
}

#[test]
fn test_log_image_with_features() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    // Create a small 2x2 grayscale image
    let image = vec![0_u8, 255, 128, 64];
    let features = vec![[0.5_f32, 0.5_f32], [1.5_f32, 1.5_f32]];

    viewer.log_image_with_features(&image, 2, 2, &features, "test/image_with_features");
}

#[test]
fn test_log_image_with_features_colored() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    // Create a small 2x2 grayscale image
    let image = vec![0_u8, 255, 128, 64];
    let features = vec![(1_usize, [0.5_f32, 0.5_f32]), (2_usize, [1.5_f32, 1.5_f32])];

    viewer.log_image_with_features_colored(
        &image,
        2,
        2,
        &features,
        "test/image_with_features_colored",
    );
}

#[test]
fn test_log_points() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    let points = vec![
        [0.0_f32, 0.0_f32, 0.0_f32],
        [1.0_f32, 0.0_f32, 0.0_f32],
        [0.0_f32, 1.0_f32, 0.0_f32],
    ];

    viewer.log_points(&points, "test/points");
}

#[test]
fn test_log_points_colored() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    let points = vec![
        (1_usize, [0.0_f32, 0.0_f32, 0.0_f32]),
        (2_usize, [1.0_f32, 0.0_f32, 0.0_f32]),
        (3_usize, [0.0_f32, 1.0_f32, 0.0_f32]),
    ];

    viewer.log_points_colored(&points, "test/points_colored");
}

#[test]
fn test_set_frame() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    // Should not panic
    viewer.set_frame(0);
    viewer.set_frame(42);
    viewer.set_frame(1000);
}

#[test]
fn test_log_camera_frustum() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    let focal_length = 500.0_f32;
    let width = 640_u32;
    let height = 480_u32;
    let size = 0.1_f32;

    viewer.log_camera_frustum(focal_length, width, height, "test/frustum", size);
}

#[test]
fn test_log_trajectory() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    // Create trajectory poses
    let mut poses = Vec::new();

    // Pose 1: identity
    poses.push(Matrix4::identity());

    // Pose 2: translated by (1, 0, 0)
    let mut pose2 = Matrix4::identity();
    pose2[(0, 3)] = 1.0;
    poses.push(pose2);

    // Pose 3: translated by (2, 0, 0)
    let mut pose3 = Matrix4::identity();
    pose3[(0, 3)] = 2.0;
    poses.push(pose3);

    viewer.log_trajectory(&poses, "test/trajectory");
}

#[test]
fn test_log_imu_raw() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    let accel = vec![
        [9.81_f32, 0.0, 0.0],
        [9.82_f32, 0.01, 0.0],
        [9.80_f32, -0.01, 0.0],
    ];

    let gyro = vec![
        [0.0_f32, 0.0, 0.0],
        [0.01_f32, 0.0, 0.0],
        [0.02_f32, 0.0, 0.0],
    ];

    viewer.log_imu_raw(1000000, &accel, &gyro, "test/imu_raw");
}

#[test]
fn test_log_imu_processed() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    let accel = vec![
        [9.81_f32, 0.0, 0.0],
        [9.81_f32, 0.0, 0.0],
        [9.81_f32, 0.0, 0.0],
    ];

    let gyro = vec![
        [0.0_f32, 0.0, 0.0],
        [0.0_f32, 0.0, 0.0],
        [0.0_f32, 0.0, 0.0],
    ];

    viewer.log_imu_processed(1000000, &accel, &gyro, "test/imu_processed");
}

#[test]
fn test_log_imu_harmonics() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    let gravity = [0.0_f32, 0.0_f32, 9.81_f32];
    let bias_accel = [0.01_f32, 0.02_f32, 0.03_f32];
    let bias_gyro = [0.001_f32, 0.002_f32, 0.003_f32];
    let harmonic_accel = vec![
        [0.1_f32, 0.2_f32, 0.3_f32],
        [0.15_f32, 0.25_f32, 0.35_f32],
    ];

    viewer.log_imu_harmonics(
        1000000,
        gravity,
        bias_accel,
        bias_gyro,
        &harmonic_accel,
        "test/imu_harmonics",
    );
}

#[test]
fn test_log_imu_signal_quality() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    let snr = [20.0_f32, 25.0_f32, 22.0_f32];
    let rms = [0.5_f32, 0.6_f32, 0.55_f32];
    let peak = [1.0_f32, 1.2_f32, 1.1_f32];

    viewer.log_imu_signal_quality(
        1000000,
        snr,
        rms,
        peak,
        "running",
        50.0_f32,
        "test/imu_quality",
    );
}

#[test]
fn test_log_loop_closure() {
    use rs_vio::optimization::loop_closure::LoopClosureConstraint;
    use nalgebra::{Isometry3, Matrix6, Translation3, UnitQuaternion};

    let mut viewer = RerunViewer::new();
    viewer.initialize().expect("initialize failed");

    // Create mock loop closure constraints using Isometry3
    let identity_iso = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, 0.0),
        UnitQuaternion::identity(),
    );

    let constraint1 = LoopClosureConstraint {
        keyframe_id_1: 0,
        keyframe_id_2: 10,
        relative_pose: identity_iso,
        information_matrix: Matrix6::identity(),
    };

    let constraint2 = LoopClosureConstraint {
        keyframe_id_1: 5,
        keyframe_id_2: 15,
        relative_pose: identity_iso,
        information_matrix: Matrix6::identity(),
    };

    let constraints = vec![constraint1, constraint2];

    // Should not panic and correctly handle loop closure visualization
    viewer.log_loop_closure(&constraints, "loop_closure/test");
}

#[test]
fn test_multiple_operations_sequence() {
    let mut viewer = RerunViewer::new();
    let _ = viewer.initialize();

    // Simulate a sequence of operations like the estimator would do
    viewer.set_frame(0);

    // Log pose
    let pose = Matrix4::identity();
    viewer.log_pose(pose, "pose");

    // Log camera frustum
    viewer.log_camera_frustum(500.0, 640, 480, "camera", 0.1);

    // Log points
    let points = vec![
        [0.0_f32, 0.0_f32, 1.0_f32],
        [1.0_f32, 0.0_f32, 1.0_f32],
    ];
    viewer.log_points(&points, "map/points");

    // Log IMU data
    let accel = vec![[9.81_f32, 0.0, 0.0]];
    let gyro = vec![[0.0_f32, 0.0, 0.0]];
    viewer.log_imu_raw(1000000, &accel, &gyro, "imu/raw");
    viewer.log_imu_processed(1000000, &accel, &gyro, "imu/processed");

    // Should complete without panicking
}
