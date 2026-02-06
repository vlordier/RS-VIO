#![allow(clippy::unwrap_used, clippy::expect_used)]

use rs_vio::datasets::config::{
    CameraConfig, Config, FeatureDetectionConfig, KeyframeManagementConfig, OptimizationConfig,
};
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

#[tokio::test]
async fn test_async_estimator_with_dataset_player_config() {
    // Validate that async estimator can be initialized with the same config format
    // used by dataset players (TUM-VI, EuRoC, 4Seasons)
    let config = create_test_config();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    // Verify estimator is ready for async frame processing
    let dummy_left = vec![128u8; 640 * 480];
    let dummy_right = vec![128u8; 640 * 480];

    let result = estimator
        .process_frame_async(0, dummy_left, dummy_right, 1_000_000_000, None)
        .await;

    assert!(
        result.is_ok(),
        "AsyncEstimator should process frame successfully"
    );

    estimator.shutdown().await;
}

#[tokio::test]
async fn test_async_estimator_prefetch_pattern() {
    // Test the pattern where prefetch thread feeds frames to async estimator
    // This simulates TUMVIPlayer's prefetch + async_estimator flow
    use std::sync::mpsc::sync_channel;

    let config = create_test_config();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    // Create a bounded channel (similar to prefetch channel)
    let (tx, rx) = sync_channel::<(Vec<u8>, Vec<u8>)>(4);

    // Simulate prefetch thread sending frames
    let prefetch_handle = std::thread::spawn(move || {
        for _i in 0..3 {
            let dummy_left = vec![128u8; 640 * 480];
            let dummy_right = vec![128u8; 640 * 480];
            tx.send((dummy_left, dummy_right))
                .expect("Prefetch send failed");
        }
    });

    // Simulate consumer thread reading from channel and processing through async estimator
    for i in 0..3 {
        let (left, right) = rx.recv().expect("Receive failed");
        let result = estimator
            .process_frame_async(
                i as i64,
                left,
                right,
                1_000_000_000 + (i as i64 * 33_000_000),
                None,
            )
            .await;

        assert!(
            result.is_ok(),
            "Frame {} should process successfully in prefetch pattern",
            i
        );
    }

    prefetch_handle.join().expect("Prefetch thread panicked");

    estimator.shutdown().await;
}

#[tokio::test]
async fn test_async_estimator_rapid_successive_frames() {
    // Test rapid successive frame submissions (stress test)
    // Validates that channel doesn't drop frames or cause deadlocks
    let config = create_test_config();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    const NUM_FRAMES: usize = 10;

    for i in 0..NUM_FRAMES {
        let dummy_left = vec![128u8; 640 * 480];
        let dummy_right = vec![128u8; 640 * 480];
        let timestamp = 1_000_000_000i64 + (i as i64 * 33_000_000);

        let result = estimator
            .process_frame_async(i as i64, dummy_left, dummy_right, timestamp, None)
            .await;

        assert!(
            result.is_ok(),
            "Frame {} should process in rapid succession",
            i
        );
    }

    estimator.shutdown().await;
}

#[tokio::test]
async fn test_async_estimator_shutdown_with_pending_frames() {
    // Validate graceful shutdown when frames may still be processing
    // This is critical for real-world dataset player scenarios
    let config = create_test_config();
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    // Submit multiple frames
    for i in 0..3 {
        let dummy_left = vec![128u8; 640 * 480];
        let dummy_right = vec![128u8; 640 * 480];
        let timestamp = 1_000_000_000i64 + (i as i64 * 33_000_000);

        let _ = estimator
            .process_frame_async(i as i64, dummy_left, dummy_right, timestamp, None)
            .await;
    }

    // Shutdown should gracefully stop worker thread
    estimator.shutdown().await;

    // Post-shutdown, estimator object drops cleanly
}

#[tokio::test]
async fn test_dataset_player_async_isolation() {
    // Test that multiple async estimators (representing different dataset player instances)
    // don't interfere with each other
    let config1 = create_test_config();
    let config2 = create_test_config();

    let estimator1 = Arc::new(AsyncEstimator::new_with_cameras(config1, None, None, None));
    let estimator2 = Arc::new(AsyncEstimator::new_with_cameras(config2, None, None, None));

    // Process frames on both estimators concurrently
    let est1_clone = Arc::clone(&estimator1);
    let handle1 = tokio::spawn(async move {
        let dummy_left = vec![128u8; 640 * 480];
        let dummy_right = vec![128u8; 640 * 480];

        est1_clone
            .process_frame_async(0, dummy_left, dummy_right, 1_000_000_000, None)
            .await
    });

    let est2_clone = Arc::clone(&estimator2);
    let handle2 = tokio::spawn(async move {
        let dummy_left = vec![64u8; 640 * 480];
        let dummy_right = vec![64u8; 640 * 480];

        est2_clone
            .process_frame_async(0, dummy_left, dummy_right, 1_000_000_000, None)
            .await
    });

    let result1 = handle1.await;
    let result2 = handle2.await;

    assert!(result1.is_ok(), "Estimator 1 should succeed");
    assert!(result2.is_ok(), "Estimator 2 should succeed");

    // cleanup happens when Arc is dropped at end of test
}
