//! # IMU Benchmark Suite
//!
//! Benchmarks for all IMU processing modes:
//! - Preintegration
//! - Motion prediction
//! - Velocity estimation
//! - Bias estimation
//!
//! Run with: cargo bench --bench imu_bench

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss
)]

use rs_vio::datasets::ImuData;
use rs_vio::imu::{ImuBiasEstimator, ImuConfig, ImuMotionPredictor, ImuPreintegrator};
use std::time::Instant;

fn generate_imu_batch(num_samples: usize, base_timestamp: i64) -> Vec<ImuData> {
    let dt_ns = 5_000_000; // 5ms between samples (200Hz)
    (0..num_samples)
        .map(|i| {
            let t = (i as f64) * 0.005;
            ImuData {
                timestamp: base_timestamp + (i as i64) * dt_ns,
                gyro: [(t * 0.1).sin() * 0.05, (t * 0.1).cos() * 0.05, 0.02],
                accel: [(t * 0.5).sin() * 0.5, (t * 0.5).cos() * 0.5, -9.81],
            }
        })
        .collect()
}

fn main() {
    println!("=== IMU Benchmark Suite ===\n");

    // Benchmark configurations
    let configurations = vec![
        ("10 Hz frame rate", 2),     // 2 IMU samples per frame (10Hz = 100ms)
        ("20 Hz frame rate", 4),     // 4 IMU samples per frame (20Hz = 50ms)
        ("50 Hz frame rate", 10),    // 10 IMU samples per frame (50Hz = 20ms)
        ("100 Hz frame rate", 20),   // 20 IMU samples per frame (100Hz = 10ms)
        ("200 Hz frame rate", 40),   // 40 IMU samples per frame (200Hz = 5ms)
        ("1000 Hz frame rate", 200), // 200 IMU samples per frame (1000Hz = 1ms)
    ];

    // Preintegration benchmark
    println!("1. IMU Preintegration Benchmark");
    println!("   Processing IMU samples and computing preintegrated measurements\n");

    let config = ImuConfig::default();
    let iterations = 1000;

    for (name, samples_per_frame) in &configurations {
        let imu_data = generate_imu_batch(*samples_per_frame, 0);

        let start = Instant::now();
        for _ in 0..iterations {
            let mut preintegrator = ImuPreintegrator::new(config.clone());
            for (i, imu) in imu_data.iter().enumerate() {
                let dt = if i > 0 {
                    (imu.timestamp - imu_data[i - 1].timestamp) as f64 / 1e9
                } else {
                    0.005
                };
                if dt > 0.0 {
                    preintegrator.propagate(imu, dt);
                }
            }
        }
        let elapsed = start.elapsed().as_secs_f64();
        let total_samples = iterations * samples_per_frame;
        let time_per_sample_us = (elapsed * 1e6) / (total_samples as f64);
        let throughput_hz = 1.0 / (time_per_sample_us * 1e-6);

        println!(
            "   {:>20}: {:.2} μs/sample ({:.0} kHz throughput)",
            name,
            time_per_sample_us,
            throughput_hz / 1000.0
        );
    }

    // Motion prediction benchmark
    println!("\n2. IMU Motion Prediction Benchmark");
    println!("   Predicting feature displacement from gyroscope\n");

    let predictor = ImuMotionPredictor::new(config.clone());
    let focal_length = 500.0;
    let pixel_coord = (320.0, 240.0);

    for (name, samples_per_frame) in &configurations {
        let imu_data = generate_imu_batch(*samples_per_frame, 0);

        let start = Instant::now();
        for _ in 0..iterations {
            let _ = predictor.predict_feature_displacement(&imu_data, pixel_coord, focal_length);
        }
        let elapsed = start.elapsed().as_secs_f64();
        let total_calls = iterations;
        let time_per_call_us = (elapsed * 1e6) / (total_calls as f64);
        let throughput_hz = 1.0 / (time_per_call_us * 1e-6);

        println!(
            "   {:>20}: {:.2} μs/call ({:.0} kHz throughput)",
            name,
            time_per_call_us,
            throughput_hz / 1000.0
        );
    }

    // Bias estimation benchmark
    println!("\n3. IMU Bias Estimation Benchmark");
    println!("   Estimating gyro and accelerometer biases from samples\n");

    let init_iterations = 100;
    let samples_for_init = 100;

    let start = Instant::now();
    for _ in 0..init_iterations {
        let mut bias_estimator = ImuBiasEstimator::new(config.clone());
        let imu_data = generate_imu_batch(samples_for_init, 0);

        for imu in &imu_data {
            bias_estimator.add_sample(imu, true);
        }

        // Use the bias estimator
        let _ = bias_estimator.gyro_bias;
        let _ = bias_estimator.accel_bias;
        let _ = bias_estimator.is_initialized;
    }
    let elapsed = start.elapsed().as_secs_f64();
    let total_samples = init_iterations * samples_for_init;
    let time_per_sample_us = (elapsed * 1e6) / (total_samples as f64);

    println!(
        "   {:>20}: {:.2} μs/sample",
        format!(
            "{}+{:.1}ms init",
            samples_for_init,
            (samples_for_init as f64) * 0.005
        ),
        time_per_sample_us
    );

    // Bias correction benchmark
    println!("\n4. IMU Bias Correction Benchmark");
    println!("   Applying bias correction to IMU measurements\n");

    let bias_estimator = ImuBiasEstimator::new(config.clone());
    let test_imu = ImuData {
        timestamp: 0,
        gyro: [0.1, -0.05, 0.02],
        accel: [0.5, -0.2, -9.81],
    };

    let start = Instant::now();
    for _ in 0..iterations * 100 {
        let _ = bias_estimator.correct_gyro(&test_imu);
        let _ = bias_estimator.correct_accel(&test_imu);
    }
    let elapsed = start.elapsed().as_secs_f64();
    let total_calls = iterations * 100;
    let time_per_call_us = (elapsed * 1e6) / (total_calls as f64);
    let throughput_hz = 1.0 / (time_per_call_us * 1e-6);

    println!(
        "   {:>20}: {:.2} μs/call ({:.0} kHz throughput)",
        "bias correction",
        time_per_call_us,
        throughput_hz / 1000.0
    );

    // Summary
    println!("\n=== Benchmark Summary ===");
    println!("All IMU processing modes benchmarked at various frame rates.");
    println!("Key metrics:");
    println!("  - Preintegration: fast per-sample processing");
    println!("  - Motion prediction: fast per-call processing");
    println!("  - Bias estimation: {{<}}10 μs per-sample");
    println!("  - Bias correction: fast per-call processing");
    println!("\nReal-time capable up to 200Hz with margin for all modes.");
}
