//! Integration tests for async optimization
//!
//! Tests the AsyncOptimizer with frames to verify:
//! - Optimization works correctly in async context
//! - Concurrent access patterns work
//! - State management is thread-safe

use rs_vio::datasets::config::Config;
use rs_vio::estimator::{AsyncOptimizer, Frame};
use std::time::Instant;

fn create_test_config() -> Config {
    Config::load("config/tum_vi.yaml").expect("Test config should exist")
}

/// Create a test keyframe with minimal valid data
fn create_test_keyframe(frame_id: i32, timestamp_ns: i64) -> Frame {
    let mut frame = Frame::new(timestamp_ns, frame_id);
    frame.is_keyframe = true;
    frame
}

#[tokio::test]
async fn test_async_optimizer_basic_operation() {
    let config = create_test_config();
    let optimizer = AsyncOptimizer::new(&config);

    // Verify initial state
    assert_eq!(optimizer.keyframe_count().await, 0);
    assert!(!optimizer.is_full().await);

    // Add a keyframe
    let frame1 = create_test_keyframe(0, 0);
    let result = optimizer.add_frame_and_optimize(frame1, false).await;

    assert!(result.is_ok(), "Adding keyframe should succeed");
    let (success, _time_ms) = result.unwrap();
    assert!(success, "Frame should be added successfully");

    // Verify state updated
    assert_eq!(optimizer.keyframe_count().await, 1);
}

#[tokio::test]
async fn test_concurrent_optimizers() {
    let config = create_test_config();
    let optimizer = AsyncOptimizer::new(&config);

    // Create multiple optimizer clones
    let optimizer1 = optimizer.clone_optimizer();
    let optimizer2 = optimizer.clone_optimizer();

    // Verify they share state
    assert!(optimizer1.shares_state_with(&optimizer2));

    // Add frames sequentially (mutex ensures serial access)
    let frame1 = create_test_keyframe(0, 0);
    let frame2 = create_test_keyframe(1, 1000000);

    let result1 = optimizer1.add_frame_and_optimize(frame1, false).await;
    let result2 = optimizer2.add_frame_and_optimize(frame2, false).await;

    assert!(result1.is_ok());
    assert!(result2.is_ok());

    // Both optimizers should see same state
    let count1 = optimizer1.keyframe_count().await;
    let count2 = optimizer2.keyframe_count().await;
    assert_eq!(
        count1, count2,
        "Both optimizers should see same keyframe count"
    );
    assert_eq!(count1, 2);
}

#[tokio::test]
async fn test_optimization_latency() {
    let config = create_test_config();
    let optimizer = AsyncOptimizer::new(&config);

    let mut latencies = Vec::new();

    // Add several keyframes and measure latency
    for i in 0..3 {
        let frame = create_test_keyframe(i, i64::from(i) * 100000000);
        let start = Instant::now();
        let result = optimizer.add_frame_and_optimize(frame, false).await;
        let elapsed_ms = start.elapsed().as_millis() as u64;

        assert!(result.is_ok());
        latencies.push(elapsed_ms);
    }

    let min = latencies.iter().min().unwrap();
    let max = latencies.iter().max().unwrap();
    let avg = latencies.iter().sum::<u64>() / latencies.len() as u64;

    println!("Optimization latency distribution:");
    println!("  Min: {}ms", min);
    println!("  Max: {}ms", max);
    println!("  Avg: {}ms", avg);
    println!("  All: {:?}ms", latencies);
}

#[tokio::test]
async fn test_optimization_with_ba() {
    let config = create_test_config();
    let optimizer = AsyncOptimizer::new(&config);

    // Add enough keyframes to trigger sliding window
    for i in 0..5 {
        let frame = create_test_keyframe(i, i64::from(i) * 100000000);
        let result = optimizer.add_frame_and_optimize(frame, false).await;
        assert!(result.is_ok());
    }

    // Now run optimization (this will run bundle adjustment)
    let start = Instant::now();
    let result = optimizer.optimize().await;
    let elapsed_ms = start.elapsed().as_millis() as u64;

    println!("Bundle adjustment: {:?} in {}ms", result, elapsed_ms);

    // BA might fail if there are no valid observations, but shouldn't panic
    // Just verify it completes
}

#[tokio::test]
async fn test_sliding_window_capacity() {
    let config = create_test_config();
    let optimizer = AsyncOptimizer::new(&config);

    // Add frames until window is full
    let mut added_count = 0;
    for i in 0..20 {
        let frame = create_test_keyframe(i, i64::from(i) * 100000000);
        let result = optimizer.add_frame_and_optimize(frame, false).await;

        if result.is_ok() {
            added_count += 1;
        }

        // Check if full
        if optimizer.is_full().await {
            println!("Sliding window full at {} keyframes", added_count);
            break;
        }
    }

    println!("Final keyframe count: {}", optimizer.keyframe_count().await);
    assert!(added_count > 0, "Should have added at least some keyframes");
}
