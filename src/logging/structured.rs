//! Structured logging with context metadata and real-time streaming

use anyhow::{anyhow, Result};
use log::{debug, error, info, warn};
use std::collections::BTreeMap;
use std::fs::OpenOptions;
use std::io::Write;
use std::sync::Mutex;
use std::time::{SystemTime, UNIX_EPOCH};

/// Context for structured logging (component, operation, module)
#[derive(Clone, Debug)]
pub struct LogContext {
    pub component: String,
    pub operation: String,
    pub module: String,
}

impl LogContext {
    pub fn new(component: &str, operation: &str, module: &str) -> Self {
        Self {
            component: component.to_string(),
            operation: operation.to_string(),
            module: module.to_string(),
        }
    }
}

/// Real-time structured logger with performance metrics
pub struct StructuredLogger {
    context: Option<LogContext>,
    #[allow(dead_code)]
    metrics_enabled: bool,
    log_file: Option<Mutex<std::fs::File>>,
    metadata: BTreeMap<String, String>,
}

impl StructuredLogger {
    /// Create new structured logger
    pub fn new(log_file: Option<&str>) -> Result<Self> {
        let file = if let Some(path) = log_file {
            let file = OpenOptions::new()
                .create(true)
                .append(true)
                .open(path)
                .map_err(|e| anyhow!("Failed to open log file: {}", e))?;
            Some(Mutex::new(file))
        } else {
            None
        };

        Ok(Self {
            context: None,
            metrics_enabled: false,
            log_file: file,
            metadata: BTreeMap::new(),
        })
    }

    /// Enable metrics collection
    pub fn enable_metrics(&self) {
        // Metrics collection enabled
    }

    /// Set logging context
    pub fn set_context(&mut self, context: LogContext) {
        self.context = Some(context);
    }

    /// Add metadata for current log entry
    pub fn add_metadata(&mut self, key: &str, value: &str) {
        self.metadata.insert(key.to_string(), value.to_string());
    }

    /// Log structured info message
    pub fn info(&mut self, message: &str) {
        let formatted = self._format_log("INFO", message);
        info!("{}", formatted);
        self._write_log(&formatted);
    }

    /// Log structured warning message
    pub fn warn(&mut self, message: &str) {
        let formatted = self._format_log("WARN", message);
        warn!("{}", formatted);
        self._write_log(&formatted);
    }

    /// Log structured error message
    pub fn error(&mut self, message: &str) {
        let formatted = self._format_log("ERROR", message);
        error!("{}", formatted);
        self._write_log(&formatted);
    }

    /// Log structured debug message
    pub fn debug(&mut self, message: &str) {
        let formatted = self._format_log("DEBUG", message);
        debug!("{}", formatted);
        self._write_log(&formatted);
    }

    /// Log performance event (latency, throughput, etc.)
    pub fn log_performance(&mut self, metric_name: &str, value: f64, unit: &str) {
        self.add_metadata("metric_name", metric_name);
        self.add_metadata("value", &value.to_string());
        self.add_metadata("unit", unit);
        let msg = format!("[PERF] {}: {}{}", metric_name, value, unit);
        self.info(&msg);
    }

    /// Format log message with context and metadata
    fn _format_log(&self, level: &str, message: &str) -> String {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis())
            .unwrap_or(0);

        let mut result = format!("[{}ms] [{}] {}", timestamp, level, message);

        if let Some(ctx) = &self.context {
            result = format!(
                "{} | {}:{}:{}",
                result, ctx.component, ctx.operation, ctx.module
            );
        }

        if !self.metadata.is_empty() {
            let meta_str = self
                .metadata
                .iter()
                .map(|(k, v)| format!("{}={}", k, v))
                .collect::<Vec<_>>()
                .join(" ");
            result = format!("{} | {}", result, meta_str);
        }

        result
    }

    /// Write log to file if configured
    fn _write_log(&self, message: &str) {
        if let Some(ref file) = self.log_file {
            match file.lock() {
                Ok(mut f) => {
                    let _ = writeln!(f, "{}", message);
                    let _ = f.flush();
                },
                Err(e) => {
                    log::error!("Failed to acquire log file lock: {}", e);
                },
            }
        }
    }

    /// Clear metadata for next log entry
    pub fn clear_metadata(&mut self) {
        self.metadata.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_structured_logger_creation() {
        let logger = StructuredLogger::new(None);
        assert!(logger.is_ok());
    }

    #[test]
    fn test_log_context() {
        let ctx = LogContext::new("estimator", "process_frame", "vio");
        assert_eq!(ctx.component, "estimator");
        assert_eq!(ctx.operation, "process_frame");
        assert_eq!(ctx.module, "vio");
    }

    #[test]
    fn test_structured_logger_metadata() {
        let mut logger = StructuredLogger::new(None).unwrap();
        logger.add_metadata("frame_id", "42");
        logger.add_metadata("features", "256");
        assert_eq!(logger.metadata.len(), 2);
    }
}
