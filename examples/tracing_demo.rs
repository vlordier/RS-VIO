//! Tracing + non-blocking logging with atomic counters example
//!
//! Demonstrates:
//! - Non-blocking file I/O (doesn't stall real-time loops)
//! - Atomic counters for hot-loop telemetry (zero logging in tight loops)
//! - Tracing spans for structured logging
//! - Runtime log level control via RUST_LOG

use rs_vio::{init_tracing_logging, TelemetryCounters};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use tracing::info;

fn main() -> anyhow::Result<()> {
    // Initialize tracing with non-blocking rolling file appender
    // Creates logs/ directory with daily rolling files
    let _guard = init_tracing_logging("./logs", "vio_demo")?;

    info!("═══════════════════════════════════════════════════════");
    info!("  RS-VIO Real-Time Logging with Tracing");
    info!("═══════════════════════════════════════════════════════");

    // Example 1: Basic structured logging with spans
    println!("\n▶ Example 1: Structured Logging with Tracing Spans");
    println!("─────────────────────────────────────────────────────");

    info!(module = "estimator", operation = "vio_init", "Initializing VIO estimator");

    // Example 2: No logging in hot loop - use atomic counters
    println!("\n▶ Example 2: Atomic Counters (Zero Logging in Hot Loop)");
    println!("──────────────────────────────────────────────────────");

    let counters = Arc::new(TelemetryCounters::new());
    let counters_clone = Arc::clone(&counters);

    // Simulate tight VIO processing loop (runs fast, minimal latency)
    let vio_thread = thread::spawn(move || {
        info!("VIO loop: starting hot loop (no logs inside)");

        for frame_id in 0..150 {
            // IMPORTANT: NO logging here! Just atomic counter updates
            counters_clone.record_frame();

            if frame_id % 2 == 0 {
                // Simulate feature detection
                counters_clone.record_features_detected(256);
            }

            if frame_id % 3 == 0 {
                // Simulate tracking
                counters_clone.record_features_tracked(240);
            }

            if frame_id % 5 == 0 {
                // Simulate IMU processing (4 measurements per frame)
                for _ in 0..4 {
                    counters_clone.record_imu_measurement();
                }
            }

            if frame_id % 100 == 0 && frame_id > 0 {
                // Occasional loop closure
                counters_clone.record_loop_closure();
            }

            // Simulate frame processing at ~30 fps
            thread::sleep(Duration::from_millis(33));
        }

        info!("VIO loop: completed 150 frames");
    });

    // Example 3: Telemetry logger thread (1 Hz)
    println!("  [VIO running, telemetry thread will log every 1 second]\n");

    let counters_telemetry = Arc::clone(&counters);

    let telemetry_thread = thread::spawn(move || {
        // Wait for VIO to start
        thread::sleep(Duration::from_millis(100));

        let start = Instant::now();

        loop {
            thread::sleep(Duration::from_secs(1)); // 1 Hz telemetry reporting

            let report = counters_telemetry.swap_and_report();

            // Log the telemetry (this happens at low frequency, doesn't block hot loop)
            info!(
                frames = report.frames,
                dropped = report.frames_dropped,
                features_detected = report.features_detected,
                features_tracked = report.features_tracked,
                imu_measurements = report.imu_measurements,
                loop_closures = report.loop_closures,
                "telemetry"
            );

            println!(
                "  [Telemetry] {} frames, {} features tracked, {} imu samples",
                report.frames, report.features_tracked, report.imu_measurements
            );

            // Stop after enough time
            if start.elapsed() > Duration::from_secs(8) {
                break;
            }
        }
    });

    // Wait for VIO to complete
    vio_thread.join().expect("VIO thread failed");

    // Wait for telemetry to complete
    telemetry_thread.join().expect("Telemetry thread failed");

    // Example 4: Final report
    println!("\n▶ Example 4: Final Summary");
    println!("──────────────────────────");

    let final_report = counters.swap_and_report();
    println!("{}", final_report);

    info!(
        total_frames = final_report.frames + final_report.frames_dropped,
        success_rate = (final_report.frames as f64 / (final_report.frames + final_report.frames_dropped).max(1) as f64 * 100.0),
        "session_complete"
    );

    println!("\n═══════════════════════════════════════════════════════");
    println!("  Logging output written to: ./logs/vio_demo.log.*");
    println!("═══════════════════════════════════════════════════════");
    println!("\n  Try controlling log level with:");
    println!("  RUST_LOG=trace cargo run --example tracing_demo");
    println!("  RUST_LOG=rs_vio=debug cargo run --example tracing_demo");

    Ok(())
}
