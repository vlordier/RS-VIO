/// Simple tracing example with non-blocking I/O and hot-loop telemetry
use rs_vio::logging::{init_tracing_logging, TelemetryCounters};
use std::time::Duration;
use std::thread;

fn main() -> anyhow::Result<()> {
    // Initialize tracing with non-blocking file writer (doesn't block hot loop)
    let _guard = init_tracing_logging("./logs", "vio")?;
    
    tracing::info!("VIO started");

    // Lock-free counters for hot loop (no I/O in tight loop)
    let telem = TelemetryCounters::new();

    // Simulate hot loop: just atomic increments (~1 nanosecond per call)
    for frame_id in 0..100 {
        telem.record_frame();
        
        if frame_id % 20 == 0 {
            telem.record_features_detected(256);
        }
        
        thread::sleep(Duration::from_millis(5));
    }

    // Periodic telemetry logging from timer thread (1 Hz, not in hot loop)
    let report = telem.swap_and_report();
    tracing::info!(
        frames = report.frames,
        features_detected = report.features_detected,
        "telemetry"
    );

    tracing::info!("VIO finished");
    Ok(())
}
