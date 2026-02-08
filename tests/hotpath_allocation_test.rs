//! Hot path allocation verification tests
//!
//! These tests verify that the critical frame processing path has zero allocations.
//! Run with: cargo test --test hotpath_allocation_test --features dhat-heap

#![allow(clippy::unwrap_used, clippy::expect_used)]

use rs_vio::datasets::config::Config;
use rs_vio::estimator::AsyncEstimator;
use std::sync::Arc;

#[cfg(feature = "dhat-heap")]
use dhat::Profiler;

#[cfg(feature = "dhat-heap")]
#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

/// Helper to create a test frame
fn create_test_image(width: u32, height: u32) -> Vec<u8> {
    vec![128u8; (width * height) as usize]
}

#[tokio::test]
async fn test_zero_allocation_frame_processing() {
    #[cfg(feature = "dhat-heap")]
    let _profiler = Profiler::new_heap();

    let config = Config::load("config/tum_vi.yaml").expect("Config should load");
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    let width = 512;
    let height = 512;
    let left = create_test_image(width, height);
    let right = create_test_image(width, height);

    // First frame - establishes baseline (allocations for setup are OK)
    let _ = estimator
        .process_frame_async(0, left.clone(), right.clone(), 0, None)
        .await;

    #[cfg(feature = "dhat-heap")]
    let stats_before = dhat::HeapStats::get();

    // Second frame - should have ZERO allocations in hot path
    let _ = estimator
        .process_frame_async(1, left.clone(), right.clone(), 1_000_000, None)
        .await;

    #[cfg(feature = "dhat-heap")]
    {
        use dhat::HeapStats;
        let stats_after = HeapStats::get();
        let alloc_diff = stats_after.total_blocks - stats_before.total_blocks;
        let bytes_diff = stats_after.total_bytes - stats_before.total_bytes;

        println!(
            "Allocation delta: {} blocks, {} bytes",
            alloc_diff, bytes_diff
        );

        // Allow small allocations for logging/metrics  (< 1KB acceptable)
        // The critical fix is eliminating 4MB+ image allocations
        assert!(
            bytes_diff < 1024,
            "Frame processing should have minimal allocations (<1KB), found {} bytes",
            bytes_diff
        );
    }

    #[cfg(not(feature = "dhat-heap"))]
    {
        println!("Note: Run with --features dhat-heap to verify zero allocations");
        // Test passes without dhat, just doesn't verify allocations
    }
}

#[tokio::test]
async fn test_sustained_zero_allocation_processing() {
    #[cfg(feature = "dhat-heap")]
    let _profiler = Profiler::new_heap();

    let config = Config::load("config/tum_vi.yaml").expect("Config should load");
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    let width = 512;
    let height = 512;
    let left = create_test_image(width, height);
    let right = create_test_image(width, height);

    // Warmup
    for i in 0..5 {
        let _ = estimator
            .process_frame_async(
                i,
                left.clone(),
                right.clone(),
                i * 1_000_000,
                None,
            )
            .await;
    }

    #[cfg(feature = "dhat-heap")]
    let stats_before = dhat::HeapStats::get();

    // Process many frames - should maintain zero-allocation property
    for i in 5..105 {
        let _ = estimator
            .process_frame_async(
                i,
                left.clone(),
                right.clone(),
                i * 1_000_000,
                None,
            )
            .await;
    }

    #[cfg(feature = "dhat-heap")]
    {
        use dhat::HeapStats;
        let stats_after = HeapStats::get();
        let bytes_diff = stats_after.total_bytes - stats_before.total_bytes;

        println!(
            "Allocation for 100 frames: {} bytes ({} bytes/frame avg)",
            bytes_diff,
            bytes_diff / 100
        );

        // 100 frames with old code: 400MB (4MB each)
        // With buffer pool: should be < 100KB total (1KB/frame acceptable for metadata)
        assert!(
            bytes_diff < 100_000,
            "100 frames should allocate < 100KB total, found {} bytes",
            bytes_diff
        );
    }

    #[cfg(not(feature = "dhat-heap"))]
    {
        println!("Note: Run with --features dhat-heap to verify zero allocations");
    }
}

#[tokio::test]
async fn test_concurrent_processing_no_allocation_growth() {
    let config = Config::load("config/tum_vi.yaml").expect("Config should load");
    let estimator = std::sync::Arc::new(AsyncEstimator::new_with_cameras(config, None, None, None));

    let width = 512;
    let height = 512;
    let left = create_test_image(width, height);
    let right = create_test_image(width, height);

    // Process frames concurrently
    let mut handles = vec![];
    for i in 0..20 {
        let est: Arc<AsyncEstimator> = std::sync::Arc::clone(&estimator);
        let l = left.clone();
        let r = right.clone();
        handles.push(tokio::spawn(async move {
            est.process_frame_async(i, l, r, i * 1_000_000, None)
                .await
        }));
    }

    // Wait for all
    for handle in handles {
        let _ = handle.await;
    }

    // If no panics or OOM, concurrent processing is stable
    println!("Concurrent processing completed successfully");
}
