//! # Feature Tracker Benchmarks
//!
//! Comprehensive performance benchmarks for the feature tracking subsystem.
//!
//! ## Benchmarks
//!
//! - **Feature Detection** - Time to detect features in a single image
//! - **Feature Tracking** - Optical flow tracking between consecutive frames
//! - **Noisy Tracking** - Tracking performance under sensor noise conditions
//! - **Grid Allocation** - Overhead of grid initialization
//!
//! ## Test Scenarios
//!
//! Tests parameterize over grid sizes (5×5, 10×10, 15×15, 20×20) to evaluate:
//! - Feature density impact on detection speed
//! - Memory allocation patterns
//! - Cache efficiency at different granularities
//!
//! ## Expected Results (baseline: 640×480)
//!
//! | Benchmark | Grid | Time |
//! |-----------|------|------|
//! | detection | 5×5  | ~5ms |
//! | detection | 20×20 | ~20ms |
//! | tracking | 10×10 | ~15ms |
//!
//! See [BENCHMARKING.md](../BENCHMARKING.md#feature-tracker-benchmarks) for detailed analysis.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::missing_const_for_fn
)]

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use rs_vio::feature_tracker::FeatureTracker;

/// Create a synthetic test image with a checkerboard pattern
///
/// Used for feature detection benchmarks. The checkerboard provides clear,
/// regularly-spaced features suitable for reproducible benchmarking.
///
/// # Arguments
/// * `width` - Image width in pixels
/// * `height` - Image height in pixels
///
/// # Returns
/// A vector of grayscale pixel values (0-255) forming a checkerboard pattern.
fn create_test_image(width: usize, height: usize) -> Vec<u8> {
    let mut image = vec![0u8; width * height];
    let square_size = 32;
    for y in 0..height {
        for x in 0..width {
            let square_x = x / square_size;
            let square_y = y / square_size;
            image[y * width + x] = if (square_x + square_y) % 2 == 0 {
                255
            } else {
                100
            };
        }
    }
    image
}

/// Create a synthetic test image with Gaussian noise
///
/// Simulates realistic camera sensor noise for robustness evaluation.
/// Adds random noise to the checkerboard pattern to test feature detection
/// and tracking stability under adverse conditions.
///
/// # Arguments
/// * `width` - Image width in pixels
/// * `height` - Image height in pixels
///
/// # Returns
/// A noisy checkerboard image for robustness testing
fn create_noisy_test_image(width: usize, height: usize) -> Vec<u8> {
    let mut image = vec![0u8; width * height];
    let square_size = 32;
    for y in 0..height {
        for x in 0..width {
            let square_x = x / square_size;
            let square_y = y / square_size;
            let base = if (square_x + square_y) % 2 == 0 {
                255
            } else {
                100
            };
            let noise = ((y * x) % 50) as u8;
            image[y * width + x] = base.saturating_sub(noise / 2).saturating_add(noise / 3);
        }
    }
    image
}

fn bench_feature_detection(c: &mut Criterion) {
    let width = 640;
    let height = 480;
    let mut group = c.benchmark_group("feature_detection");
    group.sample_size(10);

    // Test with different grid sizes
    for grid_size in &[5, 10, 15, 20] {
        group.bench_with_input(
            BenchmarkId::from_parameter(grid_size),
            grid_size,
            |b, &grid_size| {
                let image = black_box(create_test_image(width, height));
                let mut tracker = FeatureTracker::new(width, height, grid_size, 50, 0.01);

                b.iter(|| {
                    tracker.detect_features(black_box(&image));
                });
            },
        );
    }
    group.finish();
}

fn bench_feature_tracking(c: &mut Criterion) {
    let width = 640;
    let height = 480;
    let mut group = c.benchmark_group("feature_tracking");
    group.sample_size(10);

    // Initialize tracker and detect features
    let image1 = create_test_image(width, height);
    let mut tracker = FeatureTracker::new(width, height, 10, 50, 0.01);
    tracker.detect_features(&image1);

    // Create second image with slight translation
    let mut image2 = vec![0u8; width * height];
    for y in 0..height {
        for x in 0..width {
            let src_x = if x >= 10 { x - 10 } else { x };
            image2[y * width + x] = image1[y * width + src_x];
        }
    }

    let mut group_size = c.benchmark_group("tracking_grid_sizes");
    group_size.sample_size(10);

    // Test with different grid sizes
    for grid_size in &[5, 10, 15, 20] {
        group_size.bench_with_input(
            BenchmarkId::from_parameter(grid_size),
            grid_size,
            |b, &grid_size| {
                let mut tracker = FeatureTracker::new(width, height, grid_size, 50, 0.01);
                tracker.detect_features(&image1);

                b.iter(|| {
                    let _tracked = tracker.track_features(black_box(&image2));
                });
            },
        );
    }
    group_size.finish();
}

fn bench_feature_tracking_noisy(c: &mut Criterion) {
    let width = 640;
    let height = 480;
    let mut group = c.benchmark_group("feature_tracking_noisy");
    group.sample_size(10);

    let image1 = create_noisy_test_image(width, height);
    let mut image2 = create_noisy_test_image(width, height);

    // Add slight translation
    for y in 0..height {
        for x in 0..width {
            if x >= 10 {
                image2[y * width + x] = image1[y * width + (x - 10)];
            }
        }
    }

    let mut tracker = FeatureTracker::new(width, height, 10, 50, 0.01);
    tracker.detect_features(&image1);

    group.bench_function("track_features_noisy_input", |b| {
        b.iter(|| {
            let _tracked = tracker.track_features(black_box(&image2));
        });
    });

    group.finish();
}

fn bench_grid_allocation(c: &mut Criterion) {
    let width = 640;
    let height = 480;
    let mut group = c.benchmark_group("grid_allocation");
    group.sample_size(50);

    for grid_size in &[5, 10, 15, 20] {
        group.bench_with_input(
            BenchmarkId::from_parameter(grid_size),
            grid_size,
            |b, &grid_size| {
                b.iter(|| {
                    let _tracker = FeatureTracker::new(
                        black_box(width),
                        black_box(height),
                        grid_size,
                        50,
                        0.01,
                    );
                });
            },
        );
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_feature_detection,
    bench_feature_tracking,
    bench_feature_tracking_noisy,
    bench_grid_allocation
);
criterion_main!(benches);
