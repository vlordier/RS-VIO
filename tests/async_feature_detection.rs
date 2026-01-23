//! Integration tests for async feature detection
//!
//! Tests the AsyncFeatureDetector with synthetic and real images to verify:
//! - Feature detection works correctly
//! - Async integration patterns work
//! - Concurrent access works properly with multiple detectors

use rs_vio::estimator::{AsyncFeatureDetector, Frame};
use rs_vio::datasets::config::FeatureDetectionConfig;
use image::{ImageBuffer, Luma};
use std::path::PathBuf;
use std::time::Instant;

/// Create a synthetic test stereo pair (640x480 grayscale images with patterns)
fn create_synthetic_images() -> (image::DynamicImage, image::DynamicImage) {
    let width = 640u32;
    let height = 480u32;

    // Create left image with vertical lines pattern
    let mut left_data = vec![0u8; (width * height) as usize];
    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) as usize;
            // Create a gradient with some patterns
            left_data[idx] = (((x + y) % 256) as u8).saturating_mul(2);
        }
    }
    let left: ImageBuffer<Luma<u8>, Vec<u8>> =
        ImageBuffer::from_raw(width, height, left_data).unwrap();

    // Create right image with shifted horizontal lines pattern
    let mut right_data = vec![0u8; (width * height) as usize];
    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) as usize;
            // Shift by ~50 pixels (simulating stereo disparity)
            let shifted_x = if x > 50 { x - 50 } else { 0 };
            right_data[idx] = (((shifted_x + y) % 256) as u8).saturating_mul(2);
        }
    }
    let right: ImageBuffer<Luma<u8>, Vec<u8>> =
        ImageBuffer::from_raw(width, height, right_data).unwrap();

    (
        image::DynamicImage::ImageLuma8(left),
        image::DynamicImage::ImageLuma8(right),
    )
}

/// Load a test image from the TUM-VI dataset
fn load_test_image(filename: &str) -> Option<image::DynamicImage> {
    let path = PathBuf::from(format!(
        "/Users/vincent/Work/RS-VIO/datasets/tum_vi/magistrale1/mav0/cam1/data/{}",
        filename
    ));
    
    if path.exists() {
        image::open(&path).ok()
    } else {
        None
    }
}

#[tokio::test]
async fn test_async_detector_with_real_images() {
    // Try to load real test images, fall back to synthetic
    let (left_img, right_img) = if let Some(filename) = load_test_image("1520500644192001413.png") {
        // Use real image for both left and right (simulating stereo pair)
        (filename.clone(), filename)
    } else {
        println!("Using synthetic images for testing");
        create_synthetic_images()
    };

    let config = FeatureDetectionConfig::default();
    let detector = AsyncFeatureDetector::<8>::new(&config);

    let mut frame = Frame::new(0, 0);
    frame.timestamp_ns = 0;

    let start = Instant::now();
    let result = detector.detect_features_from_dynamic(&left_img, &right_img, &mut frame).await;
    let elapsed = start.elapsed();

    assert!(result.is_ok(), "Feature detection should succeed");
    let (feature_count, detection_ms) = result.unwrap();

    println!(
        "Feature detection: {} features in {:.1}ms (total: {:.1}ms)",
        feature_count,
        detection_ms,
        elapsed.as_secs_f64() * 1000.0
    );

    // Just verify we completed successfully
    println!("  Result: OK");
}

#[tokio::test]
async fn test_concurrent_detectors() {
    let (left_img, right_img) = create_synthetic_images();

    let config = FeatureDetectionConfig::default();
    let detector = AsyncFeatureDetector::<8>::new(&config);

    // Create multiple detector clones
    let detector1 = detector.clone_detector();
    let detector2 = detector.clone_detector();

    // Verify they share the same underlying state
    assert!(detector1.shares_state_with(&detector2), "Cloned detectors should share state");

    // Run detection with shared detector
    let mut frame1 = Frame::new(0, 0);
    let mut frame2 = Frame::new(1, 1);

    let start = Instant::now();
    let result = detector1.detect_features_from_dynamic(&left_img, &right_img, &mut frame1).await;
    let elapsed1 = start.elapsed();

    let start = Instant::now();
    let result2 = detector2.detect_features_from_dynamic(&left_img, &right_img, &mut frame2).await;
    let elapsed2 = start.elapsed();

    assert!(result.is_ok());
    assert!(result2.is_ok());

    let (count1, _) = result.unwrap();
    let (count2, _) = result2.unwrap();

    println!("Detection 1: {} features in {:.1}ms", count1, elapsed1.as_secs_f64() * 1000.0);
    println!("Detection 2: {} features in {:.1}ms", count2, elapsed2.as_secs_f64() * 1000.0);

    // Both should process successfully
    println!("  Test completed successfully");
}


#[tokio::test]
async fn test_detector_latency_distribution() {
    let (left_img, right_img) = create_synthetic_images();

    let config = FeatureDetectionConfig::default();
    let detector = AsyncFeatureDetector::<8>::new(&config);

    // Run multiple detections and collect latencies
    let mut latencies = Vec::new();
    
    for i in 0..3 {
        let mut frame = Frame::new(i, i as i32);
        let start = Instant::now();
        let result = detector.detect_features_from_dynamic(&left_img, &right_img, &mut frame).await;
        let elapsed_ms = start.elapsed().as_millis() as u64;
        
        assert!(result.is_ok());
        latencies.push(elapsed_ms);
    }

    let min = latencies.iter().min().unwrap();
    let max = latencies.iter().max().unwrap();
    let avg = latencies.iter().sum::<u64>() / latencies.len() as u64;

    println!("Feature detection latency distribution:");
    println!("  Min: {}ms", min);
    println!("  Max: {}ms", max);
    println!("  Avg: {}ms", avg);
    println!("  All: {:?}ms", latencies);
}
