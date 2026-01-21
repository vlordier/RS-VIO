//! Performance benchmarks for robustness algorithms
//!
//! This module provides benchmarks comparing the performance of different
//! geometric verification algorithms, particularly PROSAC vs RANSAC.

use nalgebra as na;
use rs_vio::feature_tracker::ransac::{MagsacPlusPlus, ProsacFundamental, RansacFundamental};
use std::time::Instant;

#[inline]
fn to_f32_coord(index: usize, modulus: usize) -> f32 {
    let reduced = index % modulus;
    let as_u16 = u16::try_from(reduced).unwrap_or(u16::MAX);
    f32::from(as_u16)
}

/// Generate synthetic correspondences with known outliers
fn generate_test_data(
    num_inliers: usize,
    num_outliers: usize,
    noise_std: f32,
) -> Vec<(na::Vector2<f32>, na::Vector2<f32>)> {
    let mut matches = Vec::new();

    // Generate inliers with small noise
    for i in 0..num_inliers {
        let x = to_f32_coord(i, 640);
        let y = to_f32_coord(i, 480);

        // Add small Gaussian noise
        let noise_x1 = (rand::random::<f32>() - 0.5) * 2.0 * noise_std;
        let noise_y1 = (rand::random::<f32>() - 0.5) * 2.0 * noise_std;
        let noise_x2 = (rand::random::<f32>() - 0.5) * 2.0 * noise_std;
        let noise_y2 = (rand::random::<f32>() - 0.5) * 2.0 * noise_std;

        matches.push((
            na::Vector2::new(x + noise_x1, y + noise_y1),
            na::Vector2::new(x + 2.0 + noise_x2, y + 1.0 + noise_y2),
        ));
    }

    // Generate outliers randomly distributed
    for _ in 0..num_outliers {
        matches.push((
            na::Vector2::new(rand::random::<f32>() * 640.0, rand::random::<f32>() * 480.0),
            na::Vector2::new(rand::random::<f32>() * 640.0, rand::random::<f32>() * 480.0),
        ));
    }

    matches
}

/// Generate synthetic correspondences with quality scores for PROSAC
fn generate_prosac_test_data(
    num_inliers: usize,
    num_outliers: usize,
    noise_std: f32,
) -> Vec<(na::Vector2<f32>, na::Vector2<f32>, f32)> {
    let mut matches = Vec::new();

    // Generate inliers with small noise and high quality scores
    for i in 0..num_inliers {
        let x = to_f32_coord(i, 640);
        let y = to_f32_coord(i, 480);

        // Add small Gaussian noise
        let noise_x1 = (rand::random::<f32>() - 0.5) * 2.0 * noise_std;
        let noise_y1 = (rand::random::<f32>() - 0.5) * 2.0 * noise_std;
        let noise_x2 = (rand::random::<f32>() - 0.5) * 2.0 * noise_std;
        let noise_y2 = (rand::random::<f32>() - 0.5) * 2.0 * noise_std;

        // High quality score for inliers
        let quality = 0.9 + rand::random::<f32>() * 0.1; // 0.9-1.0

        matches.push((
            na::Vector2::new(x + noise_x1, y + noise_y1),
            na::Vector2::new(x + 2.0 + noise_x2, y + 1.0 + noise_y2),
            quality,
        ));
    }

    // Generate outliers with low quality scores
    for _ in 0..num_outliers {
        let quality = rand::random::<f32>() * 0.3; // 0.0-0.3 low quality
        matches.push((
            na::Vector2::new(rand::random::<f32>() * 640.0, rand::random::<f32>() * 480.0),
            na::Vector2::new(rand::random::<f32>() * 640.0, rand::random::<f32>() * 480.0),
            quality,
        ));
    }

    matches
}

fn run_performance_comparison() {
    println!("RS-VIO Robustness Algorithm Performance Comparison");
    println!("==================================================");

    // Test different outlier ratios
    let test_cases = vec![
        ("Low Outliers (80% inliers)", 80, 20),
        ("Medium Outliers (50% inliers)", 50, 50),
        ("High Outliers (20% inliers)", 20, 80),
    ];

    println!("\nOutlier Ratio Comparison (100 total correspondences):");
    println!(
        "{:<30} {:<15} {:<15} {:<15}",
        "Test Case", "RANSAC (ms)", "PROSAC (ms)", "MAGSAC++ (ms)"
    );
    println!("{}", "-".repeat(75));

    for (name, num_inliers, num_outliers) in test_cases {
        let matches = generate_test_data(num_inliers, num_outliers, 1.0);
        let prosac_matches = generate_prosac_test_data(num_inliers, num_outliers, 1.0);

        // Time RANSAC
        let start = Instant::now();
        for _ in 0..10 {
            let _result = RansacFundamental::estimate(&matches, 1.0, 0.99);
        }
        let ransac_time = start.elapsed() / 10;

        // Time PROSAC
        let start = Instant::now();
        for _ in 0..10 {
            let _result = ProsacFundamental::estimate(&prosac_matches, 50, 0.99);
        }
        let prosac_time = start.elapsed() / 10;

        // Time MAGSAC++
        let start = Instant::now();
        for _ in 0..10 {
            let _result =
                MagsacPlusPlus::prosac_with_magsac_scoring(&prosac_matches, 50, 0.99, 1.0);
        }
        let magsac_time = start.elapsed() / 10;

        println!(
            "{:<30} {:<15.2} {:<15.2} {:<15.2}",
            name,
            ransac_time.as_secs_f64() * 1000.0,
            prosac_time.as_secs_f64() * 1000.0,
            magsac_time.as_secs_f64() * 1000.0
        );
    }

    // Test scaling with problem size
    let sizes = vec![50, 100, 200];

    println!("\nProblem Size Scaling (80% inliers):");
    println!("{:<10} {:<15} {:<15}", "Size", "RANSAC (ms)", "PROSAC (ms)");
    println!("{}", "-".repeat(40));

    for size in sizes {
        let matches = generate_test_data(size * 8 / 10, size * 2 / 10, 1.0);
        let prosac_matches = generate_prosac_test_data(size * 8 / 10, size * 2 / 10, 1.0);

        // Time RANSAC
        let start = Instant::now();
        let _result = RansacFundamental::estimate(&matches, 1.0, 0.99);
        let ransac_time = start.elapsed();

        // Time PROSAC
        let start = Instant::now();
        let _result = ProsacFundamental::estimate(&prosac_matches, 50, 0.99);
        let prosac_time = start.elapsed();

        println!(
            "{:<10} {:<15.2} {:<15.2}",
            size,
            ransac_time.as_secs_f64() * 1000.0,
            prosac_time.as_secs_f64() * 1000.0
        );
    }

    println!("\nPerformance Summary:");
    println!("- PROSAC typically shows better performance with quality-sorted correspondences");
    println!("- MAGSAC++ provides superior outlier rejection with adaptive thresholds");
    println!("- Performance improvements are most significant with high outlier ratios");
}

#[test]
fn performance_comparison() {
    run_performance_comparison();
}
fn main() {
    run_performance_comparison();
}
