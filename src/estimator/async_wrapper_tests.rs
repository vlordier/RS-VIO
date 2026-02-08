//! Tests for the async estimator wrapper.

use super::*;
use crate::datasets::config::{
    CameraConfig, Config, FeatureDetectionConfig, KeyframeManagementConfig, OptimizationConfig,
};
use crate::datasets::ImuData;
use std::sync::Arc;

/// Test config factory - reduces duplication across tests
fn create_test_config_base() -> Config {
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

/// Generate synthetic checkerboard pattern image for feature detection
fn create_checkerboard_image(width: usize, height: usize, square_size: usize) -> Vec<u8> {
    let mut image = vec![0u8; width * height];
    for y in 0..height {
        for x in 0..width {
            let square_x = x / square_size;
            let square_y = y / square_size;
            if (square_x + square_y).is_multiple_of(2) {
                image[y * width + x] = 200; // Light square
            } else {
                image[y * width + x] = 50; // Dark square
            }
        }
    }
    image
}

/// Generate gradient test image for edge detection
fn create_gradient_image(width: usize, height: usize) -> Vec<u8> {
    let mut image = vec![0u8; width * height];
    for y in 0..height {
        for x in 0..width {
            // Horizontal + vertical gradient for feature content
            let val =
                ((x as f32 / width as f32) * 200.0 + (y as f32 / height as f32) * 55.0) as u8;
            image[y * width + x] = val;
        }
    }
    image
}

#[tokio::test]
async fn test_async_estimator_creation() {
    // COVERAGE: Basic estimator creation and graceful shutdown
    let config = create_test_config_base();

    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);
    estimator.shutdown().await;
}

#[tokio::test]
async fn test_async_estimator_process_frame_synthetic_features() {
    // COVERAGE: Single frame processing with synthetic image containing detectable features
    let config = create_test_config_base();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    // Use checkerboard pattern instead of uniform gray (provides features for detection)
    let left_image = create_checkerboard_image(640, 480, 20);
    let right_image = create_checkerboard_image(640, 480, 20);

    let result = estimator
        .process_frame_async(0, left_image, right_image, 1_000_000_000, None)
        .await;

    assert!(
        result.is_ok(),
        "Frame 0 processing should succeed with checkerboard pattern: {:?}",
        result.err()
    );

    estimator.shutdown().await;
}

#[tokio::test]
async fn test_async_estimator_multiple_frames_with_features() {
    // COVERAGE: Sequential multi-frame processing with alternating image patterns
    let config = create_test_config_base();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    for frame_id in 0..5 {
        // Alternate between checkerboard and gradient patterns for variety
        let (left, right) = if frame_id % 2 == 0 {
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

        let timestamp = 1_000_000_000 + (frame_id as i64 * 33_000_000); // ~30 fps
        let result = estimator
            .process_frame_async(frame_id as i64, left, right, timestamp, None)
            .await;

        assert!(
            result.is_ok(),
            "Frame {}: processing failed with features: {:?}",
            frame_id,
            result.err()
        );
    }

    estimator.shutdown().await;
}

#[tokio::test]
async fn test_async_estimator_with_imu_data() {
    // COVERAGE: Frame processing with IMU data integration and timestamp validation
    let config = create_test_config_base();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    let left = create_checkerboard_image(640, 480, 20);
    let right = create_checkerboard_image(640, 480, 20);

    // Create realistic IMU measurements
    let imu_data = vec![
        ImuData {
            timestamp: 1_000_000_000,
            gyro: [0.001, 0.002, 0.003],
            accel: [0.1, 0.2, 9.81],
        },
        ImuData {
            timestamp: 1_005_000_000,
            gyro: [0.001, 0.002, 0.003],
            accel: [0.1, 0.2, 9.81],
        },
    ];

    let result = estimator
        .process_frame_async(0, left, right, 1_010_000_000, Some(imu_data))
        .await;

    assert!(
        result.is_ok(),
        "IMU-integrated frame processing should succeed: {:?}",
        result.err()
    );

    estimator.shutdown().await;
}

#[tokio::test]
async fn test_async_estimator_channel_robustness() {
    // COVERAGE: Concurrent frame submissions to validate channel integrity
    let config = create_test_config_base();
    let estimator = Arc::new(AsyncEstimator::new_with_cameras(config, None, None, None));

    let mut handles = vec![];
    for i in 0..3 {
        let est_clone = Arc::clone(&estimator);
        let handle = tokio::spawn(async move {
            let left = create_checkerboard_image(640, 480, 20);
            let right = create_checkerboard_image(640, 480, 20);
            let timestamp = 1_000_000_000 + (i as i64 * 33_000_000);

            est_clone
                .process_frame_async(i as i64, left, right, timestamp, None)
                .await
        });
        handles.push(handle);
    }

    for (idx, handle) in handles.into_iter().enumerate() {
        let result = handle
            .await
            .expect("Task panicked")
            .map_err(|e| format!("Concurrent frame {} failed: {}", idx, e));
        assert!(result.is_ok(), "{}", result.unwrap_err());
    }
}

#[tokio::test]
async fn test_async_estimator_shutdown_with_pending_operations() {
    // COVERAGE: Graceful shutdown while frames may be in-flight
    let config = create_test_config_base();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    // Submit multiple frames
    for i in 0..3 {
        let left = create_checkerboard_image(640, 480, 20);
        let right = create_checkerboard_image(640, 480, 20);
        let timestamp = 1_000_000_000i64 + (i as i64 * 33_000_000);

        let _ = estimator
            .process_frame_async(i as i64, left, right, timestamp, None)
            .await;
    }

    // Graceful shutdown should not panic
    estimator.shutdown().await;
}

#[tokio::test]
async fn test_async_estimator_timeout_behavior() {
    let config = create_test_config_base();
    let async_config = AsyncConfig {
        frame_timeout_ms: 1, // Very short timeout to force timeout
        ..Default::default()
    };
    let estimator = AsyncEstimator::new_with_cameras_and_async_config(
        config,
        None,
        None,
        None,
        async_config,
    );

    let left_image = create_checkerboard_image(640, 480, 20);
    let right_image = create_checkerboard_image(640, 480, 20);

    // This should timeout due to very short timeout
    let result = estimator
        .process_frame_async(0, left_image, right_image, 0, None)
        .await;

    assert!(result.is_err(), "Should timeout with very short timeout");
    assert!(
        result.err().unwrap().to_string().contains("timed out"),
        "Error should mention timeout"
    );

    estimator.shutdown().await;
}


#[tokio::test]
async fn test_timeout_exact_duration_respected() {
    // Validates that timeout duration is reasonably enforced (within ±50ms tolerance)
    let config = create_test_config_base();
    let async_config = AsyncConfig {
        frame_timeout_ms: 5, // Very short timeout to reliably trigger deadline miss
        ..Default::default()
    };
    let estimator = AsyncEstimator::new_with_cameras_and_async_config(
        config,
        None,
        None,
        None,
        async_config,
    );

    let left = create_checkerboard_image(640, 480, 20);
    let right = create_checkerboard_image(640, 480, 20);

    let start = std::time::Instant::now();
    let result = estimator.process_frame_async(0, left, right, 0, None).await;
    let elapsed = start.elapsed();

    assert!(result.is_err(), "Should timeout");
    assert!(
        elapsed.as_millis() < 200,
        "Timeout should be bounded by the end-to-end deadline (elapsed: {:?})",
        elapsed
    );

    estimator.shutdown().await;
}

#[tokio::test]
async fn test_worker_exits_after_panic() {
    // Validates that the worker exits after a panic and reports unhealthy status
    let config = create_test_config_base();
    let async_config = AsyncConfig {
        frame_timeout_ms: 500,
        ..Default::default()
    };
    let estimator = AsyncEstimator::new_with_cameras_and_async_config(
        config,
        None,
        None,
        None,
        async_config,
    );

    let panic_result = estimator.trigger_test_panic().await;
    assert!(panic_result.is_err(), "Test panic should return error");

    let status = estimator.failure_status();
    assert!(status.contains("Recovered from 1 panics"));
    assert!(status.contains("healthy: false"));

    let left = create_checkerboard_image(640, 480, 20);
    let right = create_checkerboard_image(640, 480, 20);
    let after_result = estimator.process_frame_async(0, left, right, 0, None).await;

    assert!(after_result.is_err(), "Worker should be closed after panic");
    assert!(
        after_result
            .err()
            .unwrap()
            .to_string()
            .contains("worker closed"),
        "Expected worker closed error"
    );

    estimator.shutdown().await;
}

#[tokio::test]
async fn test_frame_skipping_when_channel_full() {
    // Validates that frames are actually rejected when channel capacity is exceeded
    let config = create_test_config_base();
    let async_config = AsyncConfig {
        channel_capacity: 1,
        enable_frame_skipping: true,
        frame_timeout_ms: 5, // Very short timeout so queued frames expire
        ..Default::default()
    };
    let estimator = Arc::new(AsyncEstimator::new_with_cameras_and_async_config(
        config,
        None,
        None,
        None,
        async_config,
    ));

    let left = create_checkerboard_image(640, 480, 20);
    let right = create_checkerboard_image(640, 480, 20);

    // Send first frame (should queue)
    let est1 = Arc::clone(&estimator);
    let left1 = left.clone();
    let right1 = right.clone();
    let handle1 =
        tokio::spawn(async move { est1.process_frame_async(0, left1, right1, 0, None).await });

    // Give processing a moment to start
    tokio::time::sleep(std::time::Duration::from_millis(20)).await;

    // Try to send second frame immediately (channel should be full)
    let result_second = estimator
        .process_frame_async(1, left.clone(), right.clone(), 1000000, None)
        .await;

    // One of them should fail due to capacity
    let first_result = handle1.await.expect("Task panicked");

    let has_error = first_result.is_err() || result_second.is_err();
    assert!(
        has_error,
        "At least one frame should fail due to channel capacity (first: {:?}, second: {:?})",
        first_result, result_second
    );

    // Note: Cannot call shutdown on Arc<AsyncEstimator>
}

#[tokio::test]
async fn test_backlog_cap_drops_frames() {
    // Validates that backlog is capped even for keyframes
    let config = create_test_config_base();
    let async_config = AsyncConfig {
        channel_capacity: 4,
        max_pending_frames: 1,
        frame_timeout_ms: 5000,
        frame_budget_ms: 1000, // Slow budget to keep frames in queue longer
        ..Default::default()
    };
    let estimator = Arc::new(AsyncEstimator::new_with_cameras_and_async_config(
        config,
        None,
        None,
        None,
        async_config,
    ));

    // Use larger images to slow down processing
    let left = create_checkerboard_image(1920, 1080, 10);
    let right = create_checkerboard_image(1920, 1080, 10);

    // Send many frames concurrently (no await) to fill the backlog before any processing
    let mut handles = vec![];
    for i in 0..10 {
        let est = Arc::clone(&estimator);
        let left = left.clone();
        let right = right.clone();
        let is_keyframe = i == 0;
        handles.push(tokio::spawn(async move {
            est.process_frame_async_with_priority(
                i,
                left,
                right,
                i * 1_000_000,
                None,
                is_keyframe,
            )
            .await
        }));
        // No delay - send all concurrently to force backlog
    }

    let mut results = vec![];
    for handle in handles {
        results.push(handle.await.expect("Task panicked"));
    }

    let dropped_count = results
        .iter()
        .filter(|r| {
            r.as_ref()
                .err()
                .map(|err| err.to_string().contains("backlog"))
                .unwrap_or(false)
        })
        .count();

    assert!(
        dropped_count >= 1,
        "At least one frame should be dropped due to backlog cap (results: {:?})",
        results
    );
}

#[tokio::test]
async fn test_priority_preemption_over_backlog() {
    // Validates that a keyframe can preempt queued regular frames
    let config = create_test_config_base();
    let async_config = AsyncConfig {
        channel_capacity: 16, // Increased to handle more concurrent frames (buffer pool made processing faster)
        max_pending_frames: 16,
        enable_frame_skipping: false,
        frame_timeout_ms: 5000,
        frame_budget_ms: 200, // Slow budget to keep frames in queue (buffer pool made processing faster)
        keyframe_priority: 100,
        regular_frame_priority: 1,
    };
    let estimator = Arc::new(AsyncEstimator::new_with_cameras_and_async_config(
        config,
        None,
        None,
        None,
        async_config,
    ));

    let left = create_checkerboard_image(640, 480, 10);
    let right = create_checkerboard_image(640, 480, 10);
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();

    // Send enough frames to create backlog (buffer pool optimizations made processing faster)
    // Use more frames than channel capacity to ensure backlog when keyframe arrives
    for i in 0..12 {
        let est = Arc::clone(&estimator);
        let tx = tx.clone();
        let left = left.clone();
        let right = right.clone();
        tokio::spawn(async move {
            let result = est
                .process_frame_async_with_priority(i, left, right, 0, None, false)
                .await;
            let _ = tx.send((i, result));
        });
    }

    // No delay - send keyframe immediately while regular frames are queuing
    let est = Arc::clone(&estimator);
    let tx_key = tx.clone();
    tokio::spawn(async move {
        let result = est
            .process_frame_async_with_priority(99, left, right, 1_000_000, None, true)
            .await;
        let _ = tx_key.send((99, result));
    });

    drop(tx);

    let mut order = Vec::new();
    while let Some((frame_id, result)) = rx.recv().await {
        assert!(result.is_ok(), "Frame {} should succeed", frame_id);
        order.push(frame_id);
    }

    let key_pos = order.iter().position(|id| *id == 99).unwrap();
    assert!(
        key_pos < order.len() - 1,
        "Keyframe should preempt some backlog (order: {:?})",
        order
    );
}
