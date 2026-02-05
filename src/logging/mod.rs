//! Real-time logging system with structured output and performance metrics
//!
//! Provides:
//! - Structured logging with context and metadata (custom module)
//! - Real-time metrics aggregation (FPS, latency, throughput)
//! - Tracing-based structured logging with spans (production-grade)
//! - Non-blocking I/O for real-time systems
//! - Atomic counters for hot-loop telemetry
//! - Log rotation and archival

pub mod metrics;
pub mod structured;
pub mod tracing_config;

pub use metrics::PerformanceMetrics;
pub use structured::{LogContext, StructuredLogger};
pub use tracing_config::{init_tracing_logging, LoggingGuard, TelemetryCounters, TelemetryReport};

use anyhow::Result;
use std::sync::{Arc, Mutex};

/// Initialize real-time logging system
pub fn init_realtime_logging(
    enable_metrics: bool,
    log_file: Option<&str>,
) -> Result<Arc<Mutex<StructuredLogger>>> {
    let logger = StructuredLogger::new(log_file)?;
    
    if enable_metrics {
        logger.enable_metrics();
    }
    
    Ok(Arc::new(Mutex::new(logger)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_realtime_logging_init() {
        let result = init_realtime_logging(true, None);
        assert!(result.is_ok());
    }
}
