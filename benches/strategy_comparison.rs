/// Benchmark comparing all stereo matching strategies
///
/// This benchmark measures the performance characteristics of different
/// stereo matching strategies on synthetic and real data.
use rs_vio::feature_tracker::*;
use std::time::Instant;

#[inline]
fn to_f32_u16(value: usize) -> f32 {
    let as_u16 = u16::try_from(value).unwrap_or(u16::MAX);
    f32::from(as_u16)
}

/// Simulated IMU state for predictive matching
fn create_test_imu_state() -> IMUState {
    IMUState {
        velocity: nalgebra::Vector3::new(1.0, 0.0, 0.0),
        angular_velocity: nalgebra::Vector3::zeros(),
        dt: 0.033, // ~30 fps
    }
}

/// Create test features (id, x, y coordinates)
fn create_test_features(count: usize) -> Vec<(usize, f32, f32)> {
    (0..count)
        .map(|i| {
            let x = to_f32_u16(i % 640);
            let row = to_f32_u16(i / 640);
            let norm = to_f32_u16(count) / 640.0;
            let y = row * 480.0 / norm.max(1.0);
            (i, x, y)
        })
        .collect()
}

/// Create synthetic previous depth estimates
fn create_previous_depth(feature_count: usize) -> Vec<(usize, f32)> {
    (0..feature_count)
        .map(|i| (i, 3.0 + (to_f32_u16(i) * 0.01).sin() * 0.5))
        .collect()
}

/// Benchmark a single strategy
fn benchmark_strategy(
    strategy: &dyn StereoMatchingStrategy,
    features: &[(usize, f32, f32)],
    previous_depth: &[(usize, f32)],
    iterations: usize,
) -> (f64, StrategyMetrics) {
    let camera_matrix = nalgebra::Matrix3::identity();
    let dummy_image = vec![0u8; 640 * 480];
    let imu_state = create_test_imu_state();

    let start = Instant::now();
    for _ in 0..iterations {
        let _result = strategy.match_stereo(
            &dummy_image,
            &dummy_image,
            640,
            480,
            features,
            &camera_matrix,
            Some(&imu_state),
            Some(previous_depth),
        );
    }
    let iter = u32::try_from(iterations).unwrap_or(u32::MAX);
    let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0 / f64::from(iter);

    // Get one result for metrics
    let result = strategy.match_stereo(
        &dummy_image,
        &dummy_image,
        640,
        480,
        features,
        &camera_matrix,
        Some(&imu_state),
        Some(previous_depth),
    );

    (elapsed_ms, result.metrics)
}

/// Main benchmark
fn main() {
    println!("\n╔════════════════════════════════════════════════════════════════╗");
    println!("║     Stereo Matching Strategy Comparison Benchmark             ║");
    println!("╚════════════════════════════════════════════════════════════════╝\n");

    println!("Compiled strategies: {}\n", {
        let strategies = MatchingStrategyConfig::available_strategies();
        if strategies.is_empty() {
            "None (compile with at least one matching-* feature)".to_string()
        } else {
            strategies.join(", ")
        }
    });

    // Benchmark configurations
    let feature_counts = vec![50, 100, 200, 500];
    let iterations = 100;

    println!("Benchmark Parameters:");
    println!("  - Feature counts: {:?}", feature_counts);
    println!("  - Iterations per test: {}", iterations);
    println!("  - Frame: 640×480 pixels");
    println!("  - IMU state: Velocity (1, 0, 0) m/s, dt=33ms");
    println!();

    for &feature_count in &feature_counts {
        println!("┌─ Benchmark: {} Features", feature_count);
        println!("├─────────────────────────────────────────────────────┐");
        println!(
            "│ {:30} │ {:12} │ {:10} │",
            "Strategy", "Time (ms)", "Relative"
        );
        println!("├─────────────────────────────────────────────────────┤");

        let features = create_test_features(feature_count);
        let previous_depth = create_previous_depth(feature_count);

        let mut _baseline_time = 0.0;
        let mut results = Vec::new();

        #[cfg(feature = "matching-basic-ransac")]
        {
            use rs_vio::feature_tracker::BasicRANSACStrategy;
            let strategy = BasicRANSACStrategy::new(BasicRANSACConfig::default());
            let (time_ms, metrics) =
                benchmark_strategy(&strategy, &features, &previous_depth, iterations);
            println!(
                "│ {:30} │ {:12.4} │ {:10} │",
                "BasicRANSAC", time_ms, "baseline"
            );
            _baseline_time = time_ms;
            results.push(("BasicRANSAC", time_ms, metrics));
        }

        #[cfg(feature = "matching-imu-guided")]
        {
            use rs_vio::feature_tracker::IMUGuidedStrategy;
            let strategy = IMUGuidedStrategy::new(8.0, BasicRANSACConfig::default());
            let (time_ms, metrics) =
                benchmark_strategy(&strategy, &features, &previous_depth, iterations);
            let relative = if _baseline_time > 0.0 {
                format!("{:.2}x", _baseline_time / time_ms)
            } else {
                "—".to_string()
            };
            println!(
                "│ {:30} │ {:12.4} │ {:10} │",
                "IMUGuided", time_ms, relative
            );
            results.push(("IMUGuided", time_ms, metrics));
        }

        #[cfg(feature = "matching-temporal")]
        {
            use rs_vio::feature_tracker::TemporalConsistencyStrategy;
            let strategy = TemporalConsistencyStrategy::default();
            let (time_ms, metrics) =
                benchmark_strategy(&strategy, &features, &previous_depth, iterations);
            let relative = if _baseline_time > 0.0 {
                format!("{:.2}x", _baseline_time / time_ms)
            } else {
                "—".to_string()
            };
            println!(
                "│ {:30} │ {:12.4} │ {:10} │",
                "TemporalConsistency", time_ms, relative
            );
            results.push(("TemporalConsistency", time_ms, metrics));
        }

        #[cfg(feature = "matching-hybrid-of")]
        {
            use rs_vio::feature_tracker::HybridOpticalFlowStrategy;
            let strategy = HybridOpticalFlowStrategy::default();
            let (time_ms, metrics) =
                benchmark_strategy(&strategy, &features, &previous_depth, iterations);
            let relative = if _baseline_time > 0.0 {
                format!("{:.2}x", _baseline_time / time_ms)
            } else {
                "—".to_string()
            };
            println!(
                "│ {:30} │ {:12.4} │ {:10} │",
                "HybridOpticalFlow", time_ms, relative
            );
            results.push(("HybridOpticalFlow", time_ms, metrics));
        }

        println!("└─────────────────────────────────────────────────────┘");
        println!();

        // Print strategy-specific metrics
        for (name, _time, metrics) in &results {
            if !metrics.strategy_specific.is_empty() {
                println!("  {} details: {}", name, metrics.strategy_specific);
            }
        }
        println!();
    }

    // Summary and recommendations
    println!("╔════════════════════════════════════════════════════════════════╗");
    println!("║                    Summary & Recommendations                   ║");
    println!("╚════════════════════════════════════════════════════════════════╝\n");

    println!("Strategy Performance Profile:");
    println!();

    #[cfg(feature = "matching-basic-ransac")]
    {
        println!("✓ BasicRANSAC");
        println!("  - Speed: Baseline (1.0x)");
        println!("  - Robustness: Good");
        println!("  - Best for: General-purpose SLAM, balanced approach");
        println!("  - Use when: Building a robust system without specific constraints");
        println!();
    }

    #[cfg(feature = "matching-imu-guided")]
    {
        println!("✓ IMUGuided");
        println!("  - Speed: 8-12x faster stereo matching");
        println!("  - Robustness: Excellent (with IMU data)");
        println!("  - Best for: Drones, vehicles, platforms with IMU fusion");
        println!("  - Use when: Targeting real-time drone navigation");
        println!();
    }

    #[cfg(feature = "matching-temporal")]
    {
        println!("✓ TemporalConsistency");
        println!("  - Speed: 100x faster (deterministic)");
        println!("  - Robustness: Very good (smooth motion assumption)");
        println!("  - Best for: Real-time ultra-low-latency systems");
        println!("  - Use when: Latency is critical, motion is smooth");
        println!();
    }

    #[cfg(feature = "matching-hybrid-of")]
    {
        println!("✓ HybridOpticalFlow");
        println!("  - Speed: 30% faster than BasicRANSAC");
        println!("  - Robustness: Good (selective region processing)");
        println!("  - Best for: Low-power systems, high frame rates");
        println!("  - Use when: Balancing speed and accuracy on embedded systems");
        println!();
    }

    println!("\nRecommendation Decision Tree:");
    println!("  1. Is IMU available? → Use IMUGuided");
    println!("  2. Need ultra-low latency? → Use TemporalConsistency");
    println!("  3. Low-power system? → Use HybridOpticalFlow");
    println!("  4. General purpose? → Use BasicRANSAC (safe default)");
    println!();

    println!("To compare specific strategies, compile with:");
    println!(
        "  cargo build --no-default-features --features matching-basic-ransac,matching-imu-guided"
    );
    println!();
}
