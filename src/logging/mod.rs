//! Real-time logging system with structured output and performance metrics
//!
//! Provides:
//! - Structured logging with context and metadata (custom module)
//! - Real-time metrics aggregation (FPS, latency, throughput)
//! - Non-blocking I/O for real-time systems

pub mod metrics;
pub mod structured;

pub use metrics::PerformanceMetrics;
pub use structured::{LogContext, StructuredLogger};

use anyhow::Result;
use std::sync::{Arc, Mutex};

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
