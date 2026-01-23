//! End-to-end pipeline integration tests
//!
//! Tests the complete async VIO components: AsyncOptimizer in realistic scenarios.
//! Note: AsyncFeatureDetector requires image inputs, so those tests are in separate integration tests.

use rs_vio::datasets::config::Config;
use rs_vio::estimator::{AsyncOptimizer, Frame};
use std::time::Instant;

fn create_test_config() -> Config {
    Config::load("config/tum_vi.yaml").expect("Test config should exist")
}

/// Create a test frame with minimal valid data
fn create_test_keyframe(id: i32, timestamp_ns: i64) -> Frame {
    let mut frame = Frame::new(timestamp_ns, id);
    frame.is_keyframe = true;
    frame
}

/// Test: Sequential keyframe optimization workflow
#[tokio::test]
async fn test_sequential_keyframe_optimization() {
    let config = create_test_config();
    let optimizer = AsyncOptimizer::new(&config);
    
    // Process a sequence of keyframes
    let mut keyframe_count = 0;
    for i in 0..10 {
        let frame = create_test_keyframe(i, i as i64 * 33_000_000); // ~30Hz
        
        let opt_result = optimizer.add_frame_and_optimize(frame, false).await;
        assert!(opt_result.is_ok(), "Optimization should succeed");
        keyframe_count += 1;
    }
    
    // Verify keyframes were added
    assert_eq!(keyframe_count, 10, "Should have processed 10 keyframes");
    assert_eq!(optimizer.keyframe_count().await, 10);
    assert!(optimizer.is_full().await, "Window should be full");
}

/// Test: Multiple optimizers with shared state (LocalSet for !Send)
#[tokio::test]
async fn test_optimizers_with_local_set() {
    let config = create_test_config();
    let optimizer = AsyncOptimizer::new(&config);
    
    // Use LocalSet for !Send types
    let local = tokio::task::LocalSet::new();
    
    local.run_until(async {
        // Spawn local tasks that share the optimizer
        let mut tasks = Vec::new();
        
        for i in 0..3 {
            let opt = optimizer.clone_optimizer();
            let task = tokio::task::spawn_local(async move {
                let frame = create_test_keyframe(i, i as i64 * 100_000_000);
                opt.add_frame_and_optimize(frame, false).await
            });
            tasks.push(task);
        }
        
        // Wait for all tasks
        for task in tasks {
            let result = task.await;
            assert!(result.is_ok(), "Task should complete");
            assert!(result.unwrap().is_ok(), "Optimization should succeed");
        }
    }).await;
    
    assert_eq!(optimizer.keyframe_count().await, 3);
}

/// Test: Optimization with bundle adjustment
#[tokio::test]
async fn test_optimization_with_bundle_adjustment() {
    let config = create_test_config();
    let optimizer = AsyncOptimizer::new(&config);
    
    // Add keyframes sequentially
    let mut latencies = Vec::new();
    for i in 0..5 {
        let frame = create_test_keyframe(i, i as i64 * 100_000_000);
        
        let start = Instant::now();
        let result = optimizer.add_frame_and_optimize(frame, false).await;
        let latency = start.elapsed().as_micros() as u64;
        
        assert!(result.is_ok());
        latencies.push(latency);
    }
    
    // Run full bundle adjustment
    let ba_start = Instant::now();
    let ba_result = optimizer.optimize().await;
    let ba_latency = ba_start.elapsed().as_micros() as u64;
    
    // Calculate statistics
    let min = latencies.iter().min().unwrap();
    let max = latencies.iter().max().unwrap();
    let avg = latencies.iter().sum::<u64>() / latencies.len() as u64;
    
    println!("Add frame latency (no BA):");
    println!("  Min: {}μs", min);
    println!("  Max: {}μs", max);
    println!("  Avg: {}μs", avg);
    println!("Bundle adjustment latency: {}μs (result: {:?})", ba_latency, ba_result);
    
    assert_eq!(optimizer.keyframe_count().await, 5);
}

/// Test: Pipeline simulation (keyframes → optimization)
#[tokio::test]
async fn test_pipeline_simulation() {
    let config = create_test_config();
    let optimizer = AsyncOptimizer::new(&config);
    
    // Process multiple keyframes
    let frame_count = 20;
    let mut processed_frames = Vec::new();
    let mut optimization_times = Vec::new();
    
    for i in 0..frame_count {
        let start = Instant::now();
        let frame = create_test_keyframe(i, i as i64 * 33_000_000);
        
        // Optimization
        let opt_start = Instant::now();
        let opt_result = optimizer.add_frame_and_optimize(frame.clone(), false).await;
        let opt_time = opt_start.elapsed().as_micros() as u64;
        assert!(opt_result.is_ok());
        optimization_times.push(opt_time);
        
        let total_time = start.elapsed().as_micros() as u64;
        processed_frames.push((i, total_time, opt_time));
    }
    
    // Verify results
    let keyframe_count = processed_frames.len();
    println!("Processed {} keyframes", keyframe_count);
    
    // Print statistics
    if !optimization_times.is_empty() {
        let min_opt = optimization_times.iter().min().unwrap();
        let max_opt = optimization_times.iter().max().unwrap();
        let avg_opt = optimization_times.iter().sum::<u64>() / optimization_times.len() as u64;
        
        println!("Optimization latency:");
        println!("  Min: {}μs", min_opt);
        println!("  Max: {}μs", max_opt);
        println!("  Avg: {}μs", avg_opt);
    }
    
    let final_count = optimizer.keyframe_count().await;
    println!("Final keyframes in window: {}", final_count);
    assert!(final_count <= 10, "Window should be capped at max size");
}

/// Test: Latency distribution measurement
#[tokio::test]
async fn test_optimization_latency_distribution() {
    let config = create_test_config();
    let optimizer = AsyncOptimizer::new(&config);
    
    let mut all_latencies = Vec::new();
    
    // Process 100 keyframes
    for i in 0..100 {
        let frame = create_test_keyframe(i, i as i64 * 33_000_000);
        
        let start = Instant::now();
        let _ = optimizer.add_frame_and_optimize(frame, false).await;
        let latency_us = start.elapsed().as_micros() as u64;
        all_latencies.push(latency_us);
    }
    
    // Sort for percentile calculation
    all_latencies.sort_unstable();
    
    let p50_idx = all_latencies.len() / 2;
    let p90_idx = (all_latencies.len() * 90) / 100;
    let p99_idx = (all_latencies.len() * 99) / 100;
    
    let p50 = all_latencies[p50_idx];
    let p90 = all_latencies[p90_idx];
    let p99 = all_latencies[p99_idx];
    let max = all_latencies.iter().max().unwrap();
    
    println!("Optimization latency distribution (100 keyframes):");
    println!("  P50: {}μs", p50);
    println!("  P90: {}μs", p90);
    println!("  P99: {}μs", p99);
    println!("  Max: {}μs", max);
    
    // Verify we processed keyframes
    let keyframes_added = optimizer.keyframe_count().await;
    println!("  Final keyframes in window: {}", keyframes_added);
    assert!(keyframes_added > 0, "Should have added some keyframes");
}

/// Test: Throughput measurement
#[tokio::test]
async fn test_optimization_throughput() {
    let config = create_test_config();
    let optimizer = AsyncOptimizer::new(&config);
    
    let frame_count = 50;
    let overall_start = Instant::now();
    
    for i in 0..frame_count {
        let frame = create_test_keyframe(i, i as i64 * 33_000_000);
        let _ = optimizer.add_frame_and_optimize(frame, false).await;
    }
    
    let elapsed_ms = overall_start.elapsed().as_millis() as f64;
    let fps = (frame_count as f64 / elapsed_ms) * 1000.0;
    
    println!("Throughput: {:.2} fps ({} keyframes in {:.2}ms)", fps, frame_count, elapsed_ms);
    println!("Final window size: {}", optimizer.keyframe_count().await);
    
    // Should be able to process >10 fps even with synthetic frames
    assert!(fps > 10.0, "Throughput should be >10 fps, got {:.2}", fps);
}

/// Test: Sliding window capacity
#[tokio::test]
async fn test_sliding_window_capacity() {
    let config = create_test_config();
    let optimizer = AsyncOptimizer::new(&config);
    
    // Add more keyframes than window capacity
    for i in 0..15 {
        let frame = create_test_keyframe(i, i as i64 * 100_000_000);
        let _ = optimizer.add_frame_and_optimize(frame, false).await;
    }
    
    let final_count = optimizer.keyframe_count().await;
    println!("Added 15 keyframes, window contains: {}", final_count);
    
    // Window should be at or below max capacity (typically 10)
    assert!(final_count <= 10, "Sliding window should maintain capacity limit");
    assert!(optimizer.is_full().await, "Window should be full");
}
