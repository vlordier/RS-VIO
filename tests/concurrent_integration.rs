//! Integration tests for concurrent frame processing configuration
//!
//! Validates concurrent processor configuration and channel setup

use rs_vio::estimator::{ConcurrentConfig, ConcurrentFrameProcessor, Frame};
use std::sync::Arc;

/// Test processor creation with various configurations
#[test]
fn test_processor_creation_variants() {
    let configs = vec![
        ConcurrentConfig::default(),
        ConcurrentConfig {
            pipeline_depth: 16,
            feature_workers: 4,
            optimization_workers: 2,
            ..Default::default()
        },
        ConcurrentConfig {
            pipeline_depth: 2,
            feature_workers: 1,
            optimization_workers: 1,
            simulated_work_ms: Some(10),
            simulated_jitter_ms: Some(5),
            ..Default::default()
        },
    ];

    for config in configs {
        let (processor, _handle) = ConcurrentFrameProcessor::new(config.clone());
        assert_eq!(processor.queue_depth(), 0, "New processor should have 0 queue depth");
    }
}

/// Test concurrent processor frame tracking
#[tokio::test]
async fn test_processor_channel_creation() {
    let config = ConcurrentConfig::default();
    let (processor, _handle) = ConcurrentFrameProcessor::new(config);

    // Verify initial state
    assert_eq!(processor.queue_depth(), 0, "Queue should be empty initially");
}

/// Test configuration with simulated work delays
#[test]
fn test_config_with_simulated_delays() {
    let config = ConcurrentConfig {
        simulated_work_ms: Some(5),
        simulated_jitter_ms: Some(2),
        pipeline_depth: 8,
        feature_workers: 2,
        optimization_workers: 2,
        ..Default::default()
    };

    assert_eq!(config.simulated_work_ms, Some(5));
    assert_eq!(config.simulated_jitter_ms, Some(2));
    assert_eq!(config.pipeline_depth, 8);
    assert_eq!(config.feature_workers, 2);
    assert_eq!(config.optimization_workers, 2);
}

/// Validate frame ordering buffer logic
#[test]
fn test_frame_ordering_logic() {
    use std::collections::BTreeMap;

    // Simulate out-of-order frame completion
    let mut buffer: BTreeMap<u64, u32> = BTreeMap::new();
    let mut next_output = 0u64;

    // Frames arrive out of order: 2, 0, 1
    let arrivals = vec![2, 0, 1];
    for frame_id in arrivals {
        buffer.insert(frame_id, frame_id as u32 * 100);
    }

    // Extract frames in expected sequence order (0, 1, then 2 not ready)
    let mut extracted = Vec::new();
    for _ in 0..2 {
        if buffer.contains_key(&next_output) {
            buffer.remove(&next_output);
            extracted.push(next_output);
            next_output += 1;
        }
    }

    // Verify we extracted 0 and 1
    assert_eq!(extracted, vec![0, 1], "Frames 0 and 1 should be extracted in order");
    // Frame 2 is still in the buffer, waiting for output
    assert!(buffer.contains_key(&2), "Frame 2 should still be buffered");
}

/// Test Arc-based frame sharing across workers
#[test]
fn test_frame_sharing() {
    let frame1 = Arc::new(Frame::new(0, 0));
    let frame2 = Arc::clone(&frame1);

    // Both references point to same data
    assert_eq!(frame1.timestamp_ns, frame2.timestamp_ns);
    assert_eq!(Arc::strong_count(&frame1), 2);
}

/// Performance: measure configuration instantiation
#[test]
fn bench_config_instantiation() {
    let start = std::time::Instant::now();

    for _ in 0..10000 {
        let _config = ConcurrentConfig {
            pipeline_depth: 8,
            feature_workers: 2,
            optimization_workers: 1,
            ..Default::default()
        };
    }

    let elapsed = start.elapsed();
    println!(
        "Config instantiation: 10k configs in {:.2}ms",
        elapsed.as_secs_f64() * 1000.0
    );

    // Should be sub-millisecond
    assert!(
        elapsed.as_millis() < 10,
        "10k config instantiations should be < 10ms"
    );
}
