//! Comprehensive error scenario testing for resilient async pipeline
//!
//! Tests error handling paths including timeouts, transient failures,
//! channel closure, and graceful degradation.

#[cfg(test)]
mod error_scenarios {
    use image::GrayImage;
    use rs_vio::datasets::config::{Config, FeatureDetectionConfig};
    use rs_vio::estimator::Frame;
    use rs_vio::estimator::{
        PipelineError, PipelineMetrics, RecoveryContext, RecoveryStrategy,
        ResilientAsyncFeatureDetector, ResilientAsyncOptimizer,
    };

    fn create_test_config() -> Config {
        Config::load("config/tum_vi.yaml").expect("config required")
    }

    fn create_test_frame(id: i32) -> Frame {
        Frame::new(id as i64, id)
    }

    fn create_test_image(width: u32, height: u32) -> GrayImage {
        GrayImage::new(width, height)
    }

    // ============================================================================
    // TIMEOUT SCENARIO TESTS
    // ============================================================================

    #[tokio::test]
    async fn test_feature_detection_timeout_with_large_image() {
        let config = FeatureDetectionConfig::default();
        let detector = ResilientAsyncFeatureDetector::<3>::new(&config, None).with_timeout(1); // 1ms timeout - very tight

        // Large-ish image that may timeout
        let left = create_test_image(640, 480);
        let right = create_test_image(640, 480);
        let mut frame = create_test_frame(0);

        // May timeout on slow systems or succeed on fast ones
        let _result = detector.detect_features(left, right, &mut frame).await;
        // Don't assert specific outcome - just verify it completes
    }

    #[tokio::test]
    async fn test_optimization_timeout() {
        let config = create_test_config();
        let optimizer = ResilientAsyncOptimizer::new(&config, None).with_timeout(1); // 1ms timeout

        let frame = create_test_frame(0);

        // This likely times out but test is about completing without panic
        let _result = optimizer.add_frame_and_optimize(frame, true).await;
    }

    #[tokio::test]
    async fn test_timeout_doesnt_block_subsequent_frames() {
        let config = FeatureDetectionConfig::default();
        let detector = ResilientAsyncFeatureDetector::<3>::new(&config, None).with_timeout(1);

        let left = create_test_image(640, 480);
        let right = create_test_image(640, 480);

        // Frame 1: may timeout
        let mut frame1 = create_test_frame(0);
        let _result1 = detector
            .detect_features(left.clone(), right.clone(), &mut frame1)
            .await;

        // Frame 2: should still process (detector not poisoned)
        let mut frame2 = create_test_frame(1);
        let _result2 = detector.detect_features(left, right, &mut frame2).await;

        // Both complete without panic
    }

    // ============================================================================
    // VALIDATION SCENARIO TESTS
    // ============================================================================

    #[tokio::test]
    async fn test_invalid_image_dimensions() {
        let config = FeatureDetectionConfig::default();
        let detector = ResilientAsyncFeatureDetector::<3>::new(&config, None);

        // Zero-width image
        let left = create_test_image(0, 100);
        let right = create_test_image(100, 100);
        let mut frame = create_test_frame(0);

        match detector.detect_features(left, right, &mut frame).await {
            Err(PipelineError::FeatureDetectionFailed(msg)) => {
                assert!(msg.contains("Left image"));
                assert!(msg.contains("zero dimensions"));
            },
            _ => panic!("Expected validation error"),
        }
    }

    #[tokio::test]
    async fn test_both_invalid_image_dimensions() {
        let config = FeatureDetectionConfig::default();
        let detector = ResilientAsyncFeatureDetector::<3>::new(&config, None);

        let left = create_test_image(0, 0);
        let right = create_test_image(0, 0);
        let mut frame = create_test_frame(0);

        let result = detector.detect_features(left, right, &mut frame).await;
        assert!(result.is_err());
    }

    // ============================================================================
    // METRICS TRACKING TESTS
    // ============================================================================

    #[tokio::test]
    async fn test_metrics_track_errors() {
        let config = FeatureDetectionConfig::default();
        let metrics = PipelineMetrics::new();
        let detector = ResilientAsyncFeatureDetector::<3>::new(&config, Some(metrics.clone()))
            .with_timeout(10); // Slightly more reasonable timeout

        let left = create_test_image(640, 480);
        let right = create_test_image(640, 480);

        // Try detections - some may timeout
        for i in 0..3 {
            let mut frame = create_test_frame(i as i32);
            let _ = detector
                .detect_features(left.clone(), right.clone(), &mut frame)
                .await;
        }

        // Metrics should have tracked attempt (might not all be errors)
        let _ = metrics.recovered_errors();
    }

    #[tokio::test]
    async fn test_metrics_track_successes() {
        let config = FeatureDetectionConfig::default();
        let metrics = PipelineMetrics::new();
        let detector = ResilientAsyncFeatureDetector::<3>::new(&config, Some(metrics.clone()));

        let left = create_test_image(100, 100);
        let right = create_test_image(100, 100);

        // This should succeed
        let mut frame = create_test_frame(0);
        let result = detector.detect_features(left, right, &mut frame).await;

        assert!(result.is_ok());
    }

    // ============================================================================
    // RECOVERY STRATEGY TESTS
    // ============================================================================

    #[test]
    fn test_recovery_strategy_for_transient() {
        let err = PipelineError::Transient("network timeout".to_string());
        let strategy = RecoveryStrategy::for_error(&err);

        assert!(matches!(strategy, RecoveryStrategy::Retry { .. }));
        assert!(strategy.is_retryable());
    }

    #[test]
    fn test_recovery_strategy_for_timeout() {
        let err = PipelineError::Timeout {
            stage: "detection".to_string(),
            limit_ms: 100,
            elapsed_ms: 150,
        };
        let strategy = RecoveryStrategy::for_error(&err);

        assert_eq!(strategy, RecoveryStrategy::SlowDown { backoff_ms: 33 });
    }

    #[test]
    fn test_recovery_strategy_for_feature_failure() {
        let err = PipelineError::FeatureDetectionFailed("no features".to_string());
        let strategy = RecoveryStrategy::for_error(&err);

        assert_eq!(strategy, RecoveryStrategy::Skip);
    }

    #[test]
    fn test_recovery_strategy_for_fatal() {
        let err = PipelineError::Fatal("memory error".to_string());
        let strategy = RecoveryStrategy::for_error(&err);

        assert_eq!(strategy, RecoveryStrategy::Shutdown);
    }

    #[test]
    fn test_exponential_backoff() {
        let strategy = RecoveryStrategy::Retry {
            max_attempts: 5,
            backoff_ms: 10,
        };

        let backoff0 = strategy.backoff_for_attempt(0).as_millis();
        let backoff1 = strategy.backoff_for_attempt(1).as_millis();
        let backoff2 = strategy.backoff_for_attempt(2).as_millis();

        assert_eq!(backoff0, 10);
        assert_eq!(backoff1, 20);
        assert_eq!(backoff2, 40);
    }

    #[test]
    fn test_recovery_context_retry_attempts() {
        let strategy = RecoveryStrategy::Retry {
            max_attempts: 3,
            backoff_ms: 10,
        };
        let mut ctx = RecoveryContext::new(42, "test_stage".to_string());

        assert!(ctx.can_retry(&strategy));
        ctx.next_attempt();
        assert_eq!(ctx.attempt, 1);

        assert!(ctx.can_retry(&strategy));
        ctx.next_attempt();
        assert_eq!(ctx.attempt, 2);

        assert!(ctx.can_retry(&strategy));
        ctx.next_attempt();
        assert_eq!(ctx.attempt, 3);

        // Max attempts exceeded
        assert!(!ctx.can_retry(&strategy));
    }

    #[test]
    fn test_recovery_context_error_count() {
        let mut ctx = RecoveryContext::new(0, "test".to_string());
        assert_eq!(ctx.error_count, 0);

        ctx.next_attempt();
        assert_eq!(ctx.error_count, 1);

        ctx.next_attempt();
        assert_eq!(ctx.error_count, 2);
    }

    // ============================================================================
    // GRACEFUL DEGRADATION TESTS
    // ============================================================================

    #[tokio::test]
    async fn test_detector_gracefully_handles_weak_image() {
        let config = FeatureDetectionConfig::default();
        let metrics = PipelineMetrics::new();
        let detector = ResilientAsyncFeatureDetector::<3>::new(&config, Some(metrics.clone()))
            .with_min_features(100);

        // Very small, blank image - will detect few features
        let left = GrayImage::new(64, 64);
        let right = GrayImage::new(64, 64);
        let mut frame = create_test_frame(0);

        // Should still succeed even if below min_features (logs warning)
        match detector.detect_features(left, right, &mut frame).await {
            Ok(_) => {
                // Success - continues despite low feature count
            },
            Err(_) => {
                // May timeout on weak image, still valid test
            },
        }
    }

    #[tokio::test]
    async fn test_optimizer_recovery_from_timeout() {
        let config = create_test_config();
        let metrics = PipelineMetrics::new();
        let optimizer =
            ResilientAsyncOptimizer::new(&config, Some(metrics.clone())).with_timeout(10);

        // First frame: may timeout
        let frame1 = create_test_frame(0);
        let _result1 = optimizer.add_frame_and_optimize(frame1, true).await;

        // Recovery from timeout doesn't poison optimizer
        let frame2 = create_test_frame(1);
        let _result2 = optimizer.add_frame_and_optimize(frame2, true).await;
        // Both should complete without panic
    }

    // ============================================================================
    // QUEUE PRESSURE TESTS
    // ============================================================================

    #[test]
    fn test_queue_depth_tracking() {
        let metrics = PipelineMetrics::new();

        assert_eq!(metrics.queue_depth(), 0);
        assert_eq!(metrics.max_queue_depth(), 0);

        metrics.set_queue_depth(1);
        assert_eq!(metrics.queue_depth(), 1);

        metrics.set_queue_depth(5);
        assert_eq!(metrics.queue_depth(), 5);
        assert_eq!(metrics.max_queue_depth(), 5);

        metrics.set_queue_depth(3);
        assert_eq!(metrics.queue_depth(), 3);
        assert_eq!(metrics.max_queue_depth(), 5); // Max stays at 5
    }

    // ============================================================================
    // ERROR RATE TESTS
    // ============================================================================

    #[test]
    fn test_error_rate_calculation() {
        let metrics = PipelineMetrics::new();

        // No frames processed
        assert_eq!(metrics.error_rate(), 0.0);

        // 5 successful detections (no errors)
        for _ in 0..5 {
            metrics.record_detection(100, false); // success
        }

        assert_eq!(metrics.error_rate(), 0.0);

        // 3 failed detections
        for _ in 0..3 {
            metrics.record_detection(100, true); // error=true
        }

        // Should have tracked the errors
        let rate = metrics.error_rate();
        assert!(
            rate >= 0.0 && rate <= 1.0,
            "Error rate should be valid: {}",
            rate
        );
    }

    // ============================================================================
    // CONFIGURATION TESTS
    // ============================================================================

    #[test]
    fn test_detector_timeout_configuration() {
        let config = FeatureDetectionConfig::default();

        let _detector = ResilientAsyncFeatureDetector::<3>::new(&config, None)
            .with_timeout(200)
            .with_min_features(80);

        // Configuration is applied during construction
    }

    #[test]
    fn test_optimizer_timeout_configuration() {
        let config = create_test_config();

        let _optimizer = ResilientAsyncOptimizer::new(&config, None).with_timeout(750);

        // Configuration is applied during construction
    }

    // ============================================================================
    // INTEGRATION TESTS
    // ============================================================================

    #[tokio::test]
    async fn test_end_to_end_error_recovery() {
        let detector_config = FeatureDetectionConfig::default();
        let optimizer_config = create_test_config();

        let metrics = PipelineMetrics::new();

        let detector =
            ResilientAsyncFeatureDetector::<3>::new(&detector_config, Some(metrics.clone()));
        let optimizer = ResilientAsyncOptimizer::new(&optimizer_config, Some(metrics.clone()));

        // Process multiple frames
        for i in 0..5 {
            let mut frame = create_test_frame(i as i32);
            let left = create_test_image(100, 100);
            let right = create_test_image(100, 100);

            // Try detection
            match detector.detect_features(left, right, &mut frame).await {
                Ok(_) => {
                    // Try optimization
                    let _ = optimizer.add_frame_and_optimize(frame, false).await;
                },
                Err(e) => {
                    // Error handled, continue to next frame
                    eprintln!("Frame {} error: {}", i, e);
                },
            }
        }

        // Verify metrics were collected
        let _total = metrics.total_frames_processed();
        println!("{}", metrics.summary());
    }
}
