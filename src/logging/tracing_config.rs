//! Enhanced logging with tracing + non-blocking writers
//!
//! Provides:
//! - Structured logging with spans and fields
//! - Non-blocking file I/O (doesn't stall hot loops)
//! - Rolling file appenders for log rotation
//! - Atomic counters for telemetry without logging in hot paths
//! - Runtime-configurable log levels via RUST_LOG

use anyhow::Result;
use std::sync::atomic::{AtomicU64, Ordering};
use tracing_appender::non_blocking::WorkerGuard;
use tracing_subscriber::EnvFilter;

/// Guard that keeps logging workers alive
pub struct LoggingGuard {
    _guard: WorkerGuard,
}

/// Initialize tracing with non-blocking rolling file appender
///
/// # Arguments
/// * `log_dir` - Directory for log files (will be created if missing)
/// * `app_name` - Name for log files (e.g., "vio" -> "vio.log.*")
///
/// # Environment Variables
/// Control log levels at runtime with RUST_LOG:
/// - `RUST_LOG=info` - Info and above
/// - `RUST_LOG=rs_vio=debug` - Debug for crate only
/// - `RUST_LOG=trace` - Everything
///
/// # Example
/// ```ignore
/// let _guard = init_tracing_logging("./logs", "vio")?;
/// tracing::info!("VIO initialized");
/// ```
pub fn init_tracing_logging(log_dir: &str, app_name: &str) -> Result<LoggingGuard> {
    std::fs::create_dir_all(log_dir)?;

    // Rolling daily files: vio.log.2026-02-05, vio.log.2026-02-06, etc.
    let file_appender = tracing_appender::rolling::daily(log_dir, format!("{}.log", app_name));
    let (non_blocking, guard) = tracing_appender::non_blocking(file_appender);

    let filter =
        EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info,rs_vio=debug"));

    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_writer(non_blocking)
        .with_ansi(false) // Files don't need ANSI colors
        .with_target(true)
        .with_thread_ids(true)
        .with_thread_names(true)
        .with_line_number(true)
        .compact()
        .init();

    Ok(LoggingGuard { _guard: guard })
}

/// Atomic counters for real-time telemetry without blocking
///
/// Use in hot loops for ultra-low-latency telemetry:
/// - Count events with atomic operations
/// - Flush to logs at 1-5 Hz from a timer or less-hot path
pub struct TelemetryCounters {
    /// Total frames processed
    pub frames: AtomicU64,
    /// Dropped frames (bad tracking, etc.)
    pub frames_dropped: AtomicU64,
    /// Features detected
    pub features_detected: AtomicU64,
    /// Features tracked
    pub features_tracked: AtomicU64,
    /// IMU measurements processed
    pub imu_measurements: AtomicU64,
    /// Loop closure detections
    pub loop_closures: AtomicU64,
}

impl TelemetryCounters {
    /// Create new telemetry counters
    pub fn new() -> Self {
        Self {
            frames: AtomicU64::new(0),
            frames_dropped: AtomicU64::new(0),
            features_detected: AtomicU64::new(0),
            features_tracked: AtomicU64::new(0),
            imu_measurements: AtomicU64::new(0),
            loop_closures: AtomicU64::new(0),
        }
    }

    /// Record frame completion (use in hot loop)
    pub fn record_frame(&self) {
        self.frames.fetch_add(1, Ordering::Relaxed);
    }

    /// Record dropped frame
    pub fn record_dropped_frame(&self) {
        self.frames_dropped.fetch_add(1, Ordering::Relaxed);
    }

    /// Record feature detection (use in hot loop)
    pub fn record_features_detected(&self, count: u64) {
        self.features_detected.fetch_add(count, Ordering::Relaxed);
    }

    /// Record feature tracking
    pub fn record_features_tracked(&self, count: u64) {
        self.features_tracked.fetch_add(count, Ordering::Relaxed);
    }

    /// Record IMU measurement
    pub fn record_imu_measurement(&self) {
        self.imu_measurements.fetch_add(1, Ordering::Relaxed);
    }

    /// Record loop closure
    pub fn record_loop_closure(&self) {
        self.loop_closures.fetch_add(1, Ordering::Relaxed);
    }

    /// Get and reset all counters (call periodically from 1-5 Hz timer)
    pub fn swap_and_report(&self) -> TelemetryReport {
        TelemetryReport {
            frames: self.frames.swap(0, Ordering::Relaxed),
            frames_dropped: self.frames_dropped.swap(0, Ordering::Relaxed),
            features_detected: self.features_detected.swap(0, Ordering::Relaxed),
            features_tracked: self.features_tracked.swap(0, Ordering::Relaxed),
            imu_measurements: self.imu_measurements.swap(0, Ordering::Relaxed),
            loop_closures: self.loop_closures.swap(0, Ordering::Relaxed),
        }
    }
}

impl Default for TelemetryCounters {
    fn default() -> Self {
        Self::new()
    }
}

/// Telemetry report from counter reset
#[derive(Clone, Debug)]
pub struct TelemetryReport {
    pub frames: u64,
    pub frames_dropped: u64,
    pub features_detected: u64,
    pub features_tracked: u64,
    pub imu_measurements: u64,
    pub loop_closures: u64,
}

impl std::fmt::Display for TelemetryReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let fps = self.frames;
        let drop_rate = if self.frames > 0 {
            (self.frames_dropped as f64 / (self.frames + self.frames_dropped) as f64) * 100.0
        } else {
            0.0
        };

        writeln!(f, "Telemetry Report:")?;
        writeln!(
            f,
            "  Frames: {} (dropped: {}, {:.1}% loss)",
            self.frames, self.frames_dropped, drop_rate
        )?;
        writeln!(
            f,
            "  Features: {} detected, {} tracked",
            self.features_detected, self.features_tracked
        )?;
        writeln!(f, "  IMU measurements: {}", self.imu_measurements)?;
        writeln!(f, "  Loop closures: {}", self.loop_closures)?;
        writeln!(f, "  FPS: {} (if 1-second window)", fps)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_telemetry_counters() {
        let counters = TelemetryCounters::new();

        counters.record_frame();
        counters.record_frame();
        counters.record_features_detected(10);
        counters.record_imu_measurement();

        let report = counters.swap_and_report();
        assert_eq!(report.frames, 2);
        assert_eq!(report.features_detected, 10);
        assert_eq!(report.imu_measurements, 1);

        // Verify counters reset
        let report2 = counters.swap_and_report();
        assert_eq!(report2.frames, 0);
    }

    #[test]
    fn test_telemetry_drop_rate() {
        let counters = TelemetryCounters::new();

        counters.record_frame();
        counters.record_frame();
        counters.record_frame();
        counters.record_dropped_frame();

        let report = counters.swap_and_report();
        assert_eq!(report.frames, 3);
        assert_eq!(report.frames_dropped, 1);
    }
}
