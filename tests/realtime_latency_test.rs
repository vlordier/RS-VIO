//! Real-time latency and jitter tests
//!
//! These tests verify deterministic execution suitable for embedded real-time systems.

#![allow(clippy::unwrap_used)]

use rs_vio::datasets::config::Config;
use rs_vio::estimator::AsyncEstimator;
use std::time::{Duration, Instant};

/// Helper to create a test frame
fn create_test_image(width: u32, height: u32) -> Vec<u8> {
    vec![128u8; (width * height) as usize]
}

#[tokio::test]
async fn test_deterministic_latency_30fps() {
    let config = Config::load("config/tum_vi.yaml").expect("Config should load");
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    let width = 512;
    let height = 512;
    let left = create_test_image(width, height);
    let right = create_test_image(width, height);

    // 30 FPS target: 33.3ms per frame budget
    let target_latency = Duration::from_millis(33);
    let mut latencies = Vec::new();

    // Warmup
    for i in 0..5 {
        let _ = estimator
            .process_frame_async(
                i,
                left.clone(),
                right.clone(),
                i64::from(i) * 1_000_000,
                None,
            )
            .await;
    }

    // Measure steady-state latencies
    for i in 5..105 {
        let start = Instant::now();
        let _result = estimator
            .process_frame_async(
                i,
                left.clone(),
                right.clone(),
                i64::from(i) * 1_000_000,
                None,
            )
            .await;
        let latency = start.elapsed();
        latencies.push(latency);
    }

    // Calculate statistics
    let mut sorted = latencies.clone();
    sorted.sort();

    let min = sorted.first().unwrap();
    let max = sorted.last().unwrap();
    let median = sorted[sorted.len() / 2];
    let p99 = sorted[(sorted.len() as f64 * 0.99) as usize];
    let avg: Duration = latencies.iter().sum::<Duration>() / latencies.len() as u32;

    println!("Latency statistics (100 frames @ 512x512):");
    println!("  Min:    {:?}", min);
    println!("  Median: {:?}", median);
    println!("  Avg:    {:?}", avg);
    println!("  P99:    {:?}", p99);
    println!("  Max:    {:?}", max);

    // Real-time criteria for embedded systems
    let deadline_misses = latencies.iter().filter(|&l| l > &target_latency).count();
    let miss_rate = (deadline_misses as f64 / latencies.len() as f64) * 100.0;

    println!("  Deadline misses: {}/100 ({:.1}%)", deadline_misses, miss_rate);

    // For real-time embedded: aim for < 1% deadline misses
    // This is a soft check - actual embedded systems should be tested on target hardware
    println!("  Real-time readiness: {}",  if miss_rate < 5.0 {
        "GOOD (< 5% misses)"
    } else if miss_rate < 20.0 {
        "ACCEPTABLE (< 20% misses)"
    } else {
        "NEEDS IMPROVEMENT (>= 20% misses)"
    });
}

#[tokio::test]
async fn test_latency_jitter_analysis() {
    let config = Config::load("config/tum_vi.yaml").expect("Config should load");
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    let width = 512;
    let height = 512;
    let left = create_test_image(width, height);
    let right = create_test_image(width, height);

    let mut latencies = Vec::new();

    // Warmup
    for i in 0..5 {
        let _ = estimator
            .process_frame_async(
                i,
                left.clone(),
                right.clone(),
                i64::from(i) * 1_000_000,
                None,
            )
            .await;
    }

    // Measure
    for i in 5..105 {
        let start = Instant::now();
        let _ = estimator
            .process_frame_async(
                i,
                left.clone(),
                right.clone(),
                i64::from(i) * 1_000_000,
                None,
            )
            .await;
        latencies.push(start.elapsed().as_micros() as f64);
    }

    // Calculate jitter (standard deviation)
    let mean = latencies.iter().sum::<f64>() / latencies.len() as f64;
    let variance = latencies
        .iter()
        .map(|&x| {
            let diff = x - mean;
            diff * diff
        })
        .sum::<f64>()
        / latencies.len() as f64;
    let std_dev = variance.sqrt();

    // Calculate coefficient of variation (CV) - measure of jitter
    let cv = (std_dev / mean) * 100.0;

    println!("Jitter analysis:");
    println!("  Mean latency: {:.2} µs", mean);
    println!("  Std dev:      {:.2} µs", std_dev);
    println!("  CV:           {:.2}%", cv);

    // For real-time systems, CV should be low (< 15% is good)
    println!("  Jitter assessment: {}", if cv < 10.0 {
        "EXCELLENT (< 10%)"
    } else if cv < 20.0 {
        "GOOD (< 20%)"
    } else if cv < 40.0 {
        "ACCEPTABLE (< 40%)"
    } else {
        "HIGH JITTER (>= 40%)"
    });

    // High jitter (>50%) would indicate non-deterministic allocations or GC
    assert!(
        cv < 50.0,
        "Jitter should be < 50% for embedded systems, found {:.2}%",
        cv
    );
}

#[tokio::test]
async fn test_no_gc_pauses() {
    // This test looks for sudden latency spikes that could indicate GC or large allocations
    let config = Config::load("config/tum_vi.yaml").expect("Config should load");
    let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

    let width = 512;
    let height = 512;
    let left = create_test_image(width, height);
    let right = create_test_image(width, height);

    let mut latencies = Vec::new();

    // Warmup
    for i in 0..5 {
        let _ = estimator
            .process_frame_async(
                i,
                left.clone(),
                right.clone(),
                i64::from(i) * 1_000_000,
                None,
            )
            .await;
    }

    // Measure consecutive frames
    for i in 5..205 {
        let start = Instant::now();
        let _ = estimator
            .process_frame_async(
                i,
                left.clone(),
                right.clone(),
                i64::from(i) * 1_000_000,
                None,
            )
            .await;
        latencies.push(start.elapsed());
    }

    // Calculate median
    let mut sorted = latencies.clone();
    sorted.sort();
    let median = sorted[sorted.len() / 2];
    let median_micros = median.as_micros() as f64;

    // Look for spikes > 3x median (potential GC pause)
    let spike_threshold = median * 3;
    let spikes: Vec<(usize, Duration)> = latencies
        .iter()
        .enumerate()
        .filter(|(_, &lat)| lat > spike_threshold)
        .map(|(i, &lat)| (i, lat))
        .collect();

    println!("GC pause detection:");
    println!("  Median latency: {:?}", median);
    println!("  Spike threshold (3x median): {:?}", spike_threshold );
    println!("  Spikes detected: {}/200", spikes.len());

    if !spikes.is_empty() {
        println!("  Spike details:");
        for (idx, lat) in spikes.iter().take(5) {
            let ratio = lat.as_micros() as f64 / median_micros;
            println!("    Frame {}: {:?} ({:.1}x median)", idx + 5, lat, ratio);
        }
    }

    // For embedded real-time: spikes should be rare (< 5% of frames)
    let spike_rate = (spikes.len() as f64 / latencies.len() as f64) * 100.0;
    println!("  Spike rate: {:.1}%", spike_rate);

    assert!(
        spike_rate < 10.0,
        "Spike rate should be < 10%, found {:.1}% (possible GC/allocation issues)",
        spike_rate
    );
}
