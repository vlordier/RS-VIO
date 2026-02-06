#![allow(clippy::unwrap_used, clippy::expect_used)]

use rs_vio::datasets::config::{
    CameraConfig, Config, FeatureDetectionConfig, KeyframeManagementConfig, OptimizationConfig,
};
use rs_vio::datasets::ImuData;
use rs_vio::estimator::async_wrapper::AsyncEstimator;
use std::sync::Arc;

/// Create a minimal config for async estimator testing
fn create_test_config() -> Config {
    Config {
        camera: CameraConfig {
            image_width: 640,
            image_height: 480,
            left_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
            left_distortion: vec![0.0, 0.0, 0.0, 0.0, 0.0],
            right_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
            right_distortion: vec![0.0, 0.0, 0.0, 0.0, 0.0],
            left_model: None,
            right_model: None,
            T_B_Cl: vec![
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
            T_B_Cr: vec![
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
        },
        keyframe_management: KeyframeManagementConfig {
            keyframe_window_size: 8,
            translation_threshold: 0.2,
            rotation_threshold: 0.1,
        },
        feature_detection: FeatureDetectionConfig {
            grid_cols: 16,
            max_features_per_grid: 80,
            optical_flow_max_iterations: 30,
            optical_flow_convergence_threshold: 0.01,
        },
        optimization: OptimizationConfig {
            bundle_adjustment_max_iterations: 5,
            pnp_max_iterations: 5,
        },
        calibration: None,
    }
}

/// Generate synthetic checkerboard pattern for feature detection testing
fn create_checkerboard_image(width: usize, height: usize, square_size: usize) -> Vec<u8> {
    let mut image = vec![0u8; width * height];
    for y in 0..height {
        for x in 0..width {
            let square_x = x / square_size;
            let square_y = y / square_size;
            if (square_x + square_y) % 2 == 0 {
                image[y * width + x] = 200; // Light squares
            } else {
                image[y * width + x] = 50; // Dark squares
            }
        }
    }
    image
}

/// Generate gradient image for edge detection and continuous feature patterns
fn create_gradient_image(width: usize, height: usize) -> Vec<u8> {
    let mut image = vec![0u8; width * height];
    for y in 0..height {
        for x in 0..width {
            let val = ((x as f32 / width as f32) * 200.0 + (y as f32 / height as f32) * 55.0) as u8;
            image[y * width + x] = val;
        }
    }
    image
}

#[tokio::test]
async fn test_async_estimator_with_synthetic_features() {
    // INTEGRATION: Validate async estimator initialization with dataset player config
    // and processing real synthetic feature data (not uniform dummy data)
    let config = create_test_config();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    // Use synthetic checkerboard pattern instead of uniform gray
    let left = create_checkerboard_image(640, 480, 20);
    let right = create_checkerboard_image(640, 480, 20);

    let result = estimator
        .process_frame_async(0, left, right, 1_000_000_000, None)
        .await;

    assert!(
        result.is_ok(),
        "AsyncEstimator with synthetic features should process successfully: {:?}",
        result.err()
    );

    estimator.shutdown().await;
}

#[tokio::test]
async fn test_async_estimator_with_imu_synchronized() {
    // INTEGRATION: Validate IMU data integration with visual frames
    // Tests timestamp synchronization between visual and inertial
    let config = create_test_config();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    let left = create_gradient_image(640, 480);
    let right = create_gradient_image(640, 480);

    // Create realistic IMU sequence with proper timestamps
    let imu_sequence = vec![
        ImuData {
            timestamp: 990_000_000,
            gyro: [0.001, 0.002, 0.003],
            accel: [0.0, 0.0, 9.81],
        },
        ImuData {
            timestamp: 995_000_000,
            gyro: [0.001, 0.002, 0.003],
            accel: [0.0, 0.0, 9.81],
        },
        ImuData {
            timestamp: 1_000_000_000,
            gyro: [0.001, 0.002, 0.003],
            accel: [0.0, 0.0, 9.81],
        },
    ];

    let result = estimator
        .process_frame_async(0, left, right, 1_000_000_000, Some(imu_sequence))
        .await;

    assert!(
        result.is_ok(),
        "Frame with IMU data should process successfully: {:?}",
        result.err()
    );

    estimator.shutdown().await;
}

#[tokio::test]
async fn test_async_estimator_prefetch_with_features() {
    // INTEGRATION: Test the pattern where prefetch thread feeds frames to async estimator
    // This simulates TUM-VI/EuRoC player's prefetch + async_estimator flow with real images
    use std::sync::mpsc::sync_channel;

    let config = create_test_config();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    // Create a bounded channel (similar to prefetch channel capacity)
    let (tx, rx) = sync_channel::<(Vec<u8>, Vec<u8>)>(4);

    // Simulate prefetch thread generating frames with features
    let prefetch_handle = std::thread::spawn(move || {
        for i in 0..5 {
            let (left, right) = if i % 2 == 0 {
                (
                    create_checkerboard_image(640, 480, 20),
                    create_checkerboard_image(640, 480, 20),
                )
            } else {
                (
                    create_gradient_image(640, 480),
                    create_gradient_image(640, 480),
                )
            };

            tx.send((left, right))
                .expect("Prefetch send failed - channel closed");
        }
    });

    // Consume prefetch queue and process through async estimator
    for i in 0..5 {
        let (left, right) = rx.recv().expect("Prefetch recv failed");
        let timestamp = 1_000_000_000 + (i as i64 * 33_333_333); // ~30 fps
        let result = estimator
            .process_frame_async(i as i64, left, right, timestamp, None)
            .await;

        assert!(
            result.is_ok(),
            "Frame {} should process in prefetch pattern: {:?}",
            i,
            result.err()
        );
    }

    prefetch_handle
        .join()
        .expect("Prefetch thread panicked unexpectedly");

    estimator.shutdown().await;
}

#[tokio::test]
async fn test_async_estimator_high_throughput() {
    // INTEGRATION: Stress test with rapid successive frame submissions
    // Validates channel doesn't drop frames and handles high throughput
    let config = create_test_config();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    const NUM_FRAMES: usize = 30;
    let left_pattern = create_gradient_image(640, 480);
    let right_pattern = create_gradient_image(640, 480);

    for i in 0..NUM_FRAMES {
        let timestamp = 1_000_000_000i64 + (i as i64 * 33_333_333);

        let result = estimator
            .process_frame_async(
                i as i64,
                left_pattern.clone(),
                right_pattern.clone(),
                timestamp,
                None,
            )
            .await;

        assert!(
            result.is_ok(),
            "Frame {} should process under high throughput: {:?}",
            i,
            result.err()
        );
    }

    estimator.shutdown().await;
}

#[tokio::test]
async fn test_async_estimator_graceful_shutdown_with_frames() {
    // INTEGRATION: Validate graceful shutdown when frames may still be processing
    // This is critical for real-world dataset scenarios where player may exit mid-stream
    let config = create_test_config();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    // Submit multiple frames
    for i in 0..5 {
        let left = create_checkerboard_image(640, 480, 20);
        let right = create_checkerboard_image(640, 480, 20);
        let timestamp = 1_000_000_000 + (i as i64 * 33_000_000);

        let _ = estimator
            .process_frame_async(i as i64, left, right, timestamp, None)
            .await;
    }

    // Shutdown should gracefully stop worker thread without panic
    estimator.shutdown().await;

    // Object destruction and cleanup should be clean
}

#[tokio::test]
async fn test_async_estimator_config_variations() {
    // INTEGRATION: Test that estimator handles configuration variations
    // Simulate different TUM-VI vs EuRoC vs 4Seasons camera settings
    let mut config = create_test_config();
    config.camera.image_width = 320;
    config.camera.image_height = 240;
    config.feature_detection.grid_cols = 8;

    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);
    let left = create_checkerboard_image(320, 240, 15);
    let right = create_checkerboard_image(320, 240, 15);

    let result = estimator
        .process_frame_async(0, left, right, 1_000_000_000, None)
        .await;

    assert!(
        result.is_ok(),
        "Different config should process correctly: {:?}",
        result.err()
    );

    estimator.shutdown().await;
}

#[tokio::test]
async fn test_concurrent_estimator_isolation() {
    // INTEGRATION: Multiple async estimators (representing different dataset players)
    // should not interfere with each other
    let config1 = create_test_config();
    let config2 = create_test_config();

    let estimator1 = Arc::new(AsyncEstimator::new_with_cameras(config1, None, None, None));
    let estimator2 = Arc::new(AsyncEstimator::new_with_cameras(config2, None, None, None));

    // Process frames concurrently on both estimators
    let est1_clone = Arc::clone(&estimator1);
    let handle1 = tokio::spawn(async move {
        for i in 0..3 {
            let left = create_checkerboard_image(640, 480, 20);
            let right = create_checkerboard_image(640, 480, 20);
            let timestamp = 1_000_000_000 + (i as i64 * 33_000_000);

            let _ = est1_clone
                .process_frame_async(i as i64, left, right, timestamp, None)
                .await;
        }
    });

    let est2_clone = Arc::clone(&estimator2);
    let handle2 = tokio::spawn(async move {
        for i in 0..3 {
            let left = create_gradient_image(640, 480);
            let right = create_gradient_image(640, 480);
            let timestamp = 1_000_000_000 + (i as i64 * 33_000_000);

            let _ = est2_clone
                .process_frame_async(i as i64, left, right, timestamp, None)
                .await;
        }
    });

    let r1 = handle1.await;
    let r2 = handle2.await;

    assert!(
        r1.is_ok(),
        "Estimator 1 concurrent processing should complete"
    );
    assert!(
        r2.is_ok(),
        "Estimator 2 concurrent processing should complete"
    );

    // Cleanup happens when Arc is dropped at test end
}

#[tokio::test]
async fn test_async_estimator_frame_sequence_with_imu() {
    // INTEGRATION: Test realistic sequence: multiple frames with interleaved IMU data
    // Simulates actual dataset playback pattern
    let config = create_test_config();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    // Frame 0
    let result0 = estimator
        .process_frame_async(
            0,
            create_checkerboard_image(640, 480, 20),
            create_checkerboard_image(640, 480, 20),
            1_000_000_000,
            Some(vec![
                ImuData {
                    timestamp: 990_000_000,
                    gyro: [0.001, 0.0, 0.0],
                    accel: [0.0, 0.0, 9.81],
                },
                ImuData {
                    timestamp: 1_000_000_000,
                    gyro: [0.001, 0.0, 0.0],
                    accel: [0.0, 0.0, 9.81],
                },
            ]),
        )
        .await;

    assert!(result0.is_ok(), "Frame 0 with IMU should process");

    // Frame 1
    let result1 = estimator
        .process_frame_async(
            1,
            create_gradient_image(640, 480),
            create_gradient_image(640, 480),
            1_033_333_333,
            Some(vec![
                ImuData {
                    timestamp: 1_010_000_000,
                    gyro: [0.001, 0.0, 0.0],
                    accel: [0.0, 0.0, 9.81],
                },
                ImuData {
                    timestamp: 1_030_000_000,
                    gyro: [0.001, 0.0, 0.0],
                    accel: [0.0, 0.0, 9.81],
                },
            ]),
        )
        .await;

    assert!(result1.is_ok(), "Frame 1 with IMU should process");

    estimator.shutdown().await;
}
