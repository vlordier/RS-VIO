//! Real-time logging system with structured output and performance metrics
//!
//! Provides:
//! - Structured logging with context and metadata (custom module)
//! - Real-time metrics aggregation (FPS, latency, throughput)
//! - Non-blocking I/O for real-time systems
//! - Shared colored logger initialization for all binaries

pub mod metrics;
pub mod structured;

pub use metrics::PerformanceMetrics;
pub use structured::{LogContext, StructuredLogger};

use anyhow::Result;
use std::sync::{Arc, Mutex};

/// Initialize the shared colored console logger used by all RS-VIO binaries.
///
/// Must be called once at process start, before any `log::*` macros.
/// Defaults to `debug` level; override with the `RUST_LOG` environment variable.
/// Rerun SDK noise is suppressed to `warn` level.
pub fn init_logger() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug"))
        .filter_module("rerun", log::LevelFilter::Warn)
        .format(|buf, record| {
            use std::io::Write;
            let level = match record.level() {
                log::Level::Error => "\x1b[31mERROR\x1b[0m",
                log::Level::Warn => "\x1b[33mWARN\x1b[0m",
                log::Level::Info => "\x1b[32mINFO\x1b[0m",
                log::Level::Debug => "\x1b[34mDEBUG\x1b[0m",
                log::Level::Trace => "\x1b[36mTRACE\x1b[0m",
            };
            writeln!(
                buf,
                "[{}] [{}] {}",
                buf.timestamp_millis(),
                level,
                record.args()
            )
        })
        .init();
}

/// Initialize real-time logging system
pub fn init_realtime_logging(log_file: Option<&str>) -> Result<Arc<Mutex<StructuredLogger>>> {
    let logger = StructuredLogger::new(log_file)?;
    Ok(Arc::new(Mutex::new(logger)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_realtime_logging_init() {
        let result = init_realtime_logging(None);
        assert!(result.is_ok());
    }
}
