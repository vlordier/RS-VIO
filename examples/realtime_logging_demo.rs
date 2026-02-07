//! Real-time logging example demonstrating structured logging and performance metrics

use rs_vio::logging::StructuredLogger;
use rs_vio::PerformanceMetrics;
use std::thread;
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    println!("═══════════════════════════════════════════════════════");
    println!("  RS-VIO Real-Time Logging Example");
    println!("═══════════════════════════════════════════════════════\n");

    // Initialize structured logger
    let mut logger = StructuredLogger::new(None)?;

    // Create performance metrics collector
    let metrics = PerformanceMetrics::new(Duration::from_secs(10), 1000);

    // Example 1: Basic structured logging
    println!("▶ Example 1: Structured Logging with Context");
    println!("─────────────────────────────────────────────\n");

    logger.info("Application started");

    let ctx = rs_vio::logging::LogContext::new("estimator", "process_frame", "vio");
    logger.set_context(ctx);
    logger.add_metadata("frame_id", "1");
    logger.add_metadata("features_detected", "256");
    logger.info("Frame processing started");
    logger.clear_metadata();

    println!();

    // Example 2: Performance metrics recording
    println!("▶ Example 2: Real-Time Performance Metrics");
    println!("───────────────────────────────────────────\n");

    // Simulate 30 frame measurements
    for frame_id in 1..=30 {
        let fps = 30.0 + (frame_id as f64 * 0.5).sin() * 5.0; // Sinusoidal variation
        let latency_ms = 33.33 + (frame_id as f64 * 0.3).cos() * 5.0;
        let throughput = fps;

        metrics.record_fps(fps);
        metrics.record_latency(latency_ms);
        metrics.record_throughput(throughput);

        if frame_id % 10 == 0 {
            println!(
                "  [Frame {}] FPS: {:.2}, Latency: {:.2}ms",
                frame_id, fps, latency_ms
            );
        }

        // Small delay to simulate frame processing
        thread::sleep(Duration::from_millis(10));
    }

    println!();

    // Example 3: Performance statistics
    println!("▶ Example 3: Performance Statistics");
    println!("────────────────────────────────────\n");

    if let Some(stats) = metrics.fps_stats() {
        println!("  FPS Statistics:");
        println!("    Min: {:.2} fps", stats.min);
        println!("    Max: {:.2} fps", stats.max);
        println!("    Avg: {:.2} fps", stats.mean);
        println!("    Std Dev: {:.2}", stats.std_dev);
        println!("    Count: {}", stats.count);
    }

    if let Some(stats) = metrics.latency_stats() {
        println!("\n  Latency Statistics:");
        println!("    Min: {:.2} ms", stats.min);
        println!("    Max: {:.2} ms", stats.max);
        println!("    Avg: {:.2} ms", stats.mean);
        println!("    Std Dev: {:.2} ms", stats.std_dev);
        println!("    Count: {}", stats.count);
    }

    println!();

    // Example 4: Full performance report
    println!("▶ Example 4: Comprehensive Performance Report");
    println!("──────────────────────────────────────────────\n");

    let report = metrics.report();
    println!("{}", report);

    // Example 5: Logging with different levels
    println!("▶ Example 5: Logging Levels");
    println!("────────────────────────────\n");

    logger.debug("Debug: Detailed trace information");
    logger.info("Info: Important operational events");
    logger.warn("Warning: Unusual but recoverable condition");
    logger.error("Error: Failed to load configuration");

    println!("\n═══════════════════════════════════════════════════════");
    println!("  Example completed successfully!");
    println!("═══════════════════════════════════════════════════════");

    Ok(())
}
