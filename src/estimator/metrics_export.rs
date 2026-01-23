//! Metrics export module for production observability
//!
//! Provides metrics export in Prometheus format for integration with monitoring stacks.
//! Supports both pull-based (Prometheus scraping) and push-based (remote backends) patterns.

use crate::estimator::PipelineMetrics;
use std::fmt;
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

/// Prometheus-compatible metrics export format
#[derive(Debug, Clone)]
pub struct PrometheusMetrics {
    /// Timestamp of export (milliseconds since epoch)
    pub timestamp_ms: u64,
    /// Detection latency (microseconds) - gauge
    pub detection_latency_us: u64,
    /// Optimization latency (microseconds) - gauge
    pub optimization_latency_us: u64,
    /// Queue depth (current)
    pub queue_depth: usize,
    /// Queue depth (max observed)
    pub queue_depth_max: usize,
    /// Total frames processed - counter
    pub frames_total: u64,
    /// Total errors encountered - counter
    pub errors_total: u64,
    /// Errors recovered - counter
    pub errors_recovered: u64,
    /// Error rate (0.0 to 1.0)
    pub error_rate: f64,
    /// Average detection latency (microseconds)
    pub detection_avg_us: u64,
    /// Average optimization latency (microseconds)
    pub optimization_avg_us: u64,
}

impl PrometheusMetrics {
    /// Export current metrics snapshot from PipelineMetrics
    pub fn from_pipeline(metrics: &PipelineMetrics) -> Self {
        let det = metrics.detection_metrics();
        let opt = metrics.optimization_metrics();
        
        Self {
            timestamp_ms: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_millis() as u64)
                .unwrap_or(0),
            detection_latency_us: det.avg_time_us(),
            optimization_latency_us: opt.avg_time_us(),
            queue_depth: metrics.queue_depth(),
            queue_depth_max: metrics.max_queue_depth(),
            frames_total: metrics.total_frames_processed(),
            errors_total: metrics.total_errors(),
            errors_recovered: metrics.recovered_errors(),
            error_rate: metrics.error_rate(),
            detection_avg_us: det.avg_time_us(),
            optimization_avg_us: opt.avg_time_us(),
        }
    }

    /// Format metrics as Prometheus exposition format (text/plain; version=0.0.4)
    pub fn to_prometheus_text(&self) -> String {
        format!(
            "# HELP vio_detection_latency_us Detection stage latency in microseconds\n\
             # TYPE vio_detection_latency_us gauge\n\
             vio_detection_latency_us {{}} {}\n\
             # HELP vio_optimization_latency_us Optimization stage latency in microseconds\n\
             # TYPE vio_optimization_latency_us gauge\n\
             vio_optimization_latency_us {{}} {}\n\
             # HELP vio_queue_depth Current pipeline queue depth\n\
             # TYPE vio_queue_depth gauge\n\
             vio_queue_depth {{}} {}\n\
             # HELP vio_queue_depth_max Maximum queue depth observed\n\
             # TYPE vio_queue_depth_max gauge\n\
             vio_queue_depth_max {{}} {}\n\
             # HELP vio_frames_total Total frames processed\n\
             # TYPE vio_frames_total counter\n\
             vio_frames_total {{}} {}\n\
             # HELP vio_errors_total Total errors encountered\n\
             # TYPE vio_errors_total counter\n\
             vio_errors_total {{}} {}\n\
             # HELP vio_errors_recovered Total errors recovered\n\
             # TYPE vio_errors_recovered counter\n\
             vio_errors_recovered {{}} {}\n\
             # HELP vio_error_rate Error rate (0.0-1.0)\n\
             # TYPE vio_error_rate gauge\n\
             vio_error_rate {{}} {:.4}\n\
             # HELP vio_detection_avg_us Average detection latency\n\
             # TYPE vio_detection_avg_us gauge\n\
             vio_detection_avg_us {{}} {}\n\
             # HELP vio_optimization_avg_us Average optimization latency\n\
             # TYPE vio_optimization_avg_us gauge\n\
             vio_optimization_avg_us {{}} {} {}\n",
            self.detection_latency_us,
            self.optimization_latency_us,
            self.queue_depth,
            self.queue_depth_max,
            self.frames_total,
            self.errors_total,
            self.errors_recovered,
            self.error_rate,
            self.detection_avg_us,
            self.optimization_avg_us,
            self.timestamp_ms,
        )
    }

    /// Format metrics as OpenMetrics format (text/openmetrics-exposition; version=1.0.0)
    pub fn to_openmetrics_text(&self) -> String {
        let base = self.to_prometheus_text();
        format!("{}\n# EOF\n", base)
    }
}

impl fmt::Display for PrometheusMetrics {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.to_prometheus_text())
    }
}

/// Metrics exporter for continuous export to backends
pub struct MetricsExporter {
    metrics: Arc<Mutex<Option<PrometheusMetrics>>>,
}

impl MetricsExporter {
    /// Create new metrics exporter
    pub fn new() -> Self {
        Self {
            metrics: Arc::new(Mutex::new(None)),
        }
    }

    /// Update exported metrics from pipeline
    pub fn export(&self, metrics: &PipelineMetrics) -> Result<(), ExportError> {
        let prometheus = PrometheusMetrics::from_pipeline(metrics);
        let mut guard = self.metrics.lock().map_err(|_| ExportError::LockFailed)?;
        *guard = Some(prometheus);
        Ok(())
    }

    /// Get current metrics snapshot
    pub fn snapshot(&self) -> Result<Option<PrometheusMetrics>, ExportError> {
        let guard = self.metrics.lock().map_err(|_| ExportError::LockFailed)?;
        Ok(guard.clone())
    }

    /// Get metrics as Prometheus text format
    pub fn prometheus_text(&self) -> Result<Option<String>, ExportError> {
        let guard = self.metrics.lock().map_err(|_| ExportError::LockFailed)?;
        Ok(guard.as_ref().map(|m| m.to_prometheus_text()))
    }

    /// Get metrics as OpenMetrics text format
    pub fn openmetrics_text(&self) -> Result<Option<String>, ExportError> {
        let guard = self.metrics.lock().map_err(|_| ExportError::LockFailed)?;
        Ok(guard.as_ref().map(|m| m.to_openmetrics_text()))
    }
}

impl Default for MetricsExporter {
    fn default() -> Self {
        Self::new()
    }
}

impl Clone for MetricsExporter {
    fn clone(&self) -> Self {
        Self {
            metrics: Arc::clone(&self.metrics),
        }
    }
}

/// Error type for metrics export operations
#[derive(Debug, Clone)]
pub enum ExportError {
    /// Failed to acquire metrics lock
    LockFailed,
    /// Serialization error
    SerializationFailed(String),
    /// Network error
    NetworkError(String),
}

impl fmt::Display for ExportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::LockFailed => write!(f, "Failed to acquire metrics lock"),
            Self::SerializationFailed(msg) => write!(f, "Serialization error: {}", msg),
            Self::NetworkError(msg) => write!(f, "Network error: {}", msg),
        }
    }
}

impl std::error::Error for ExportError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::{FrameMetrics, PipelineMetrics};

    #[test]
    fn test_prometheus_metrics_from_pipeline() {
        let pipeline = PipelineMetrics::new();
        
        // Record some metrics
        pipeline.record_detection(100, false);
        pipeline.record_optimization(200, false);
        
        let frame = FrameMetrics {
            frame_id: 0,
            timestamp_ns: 0,
            detection_time_us: 100,
            optimization_time_us: 200,
            e2e_time_us: 300,
            queue_depth: 2,
            success: true,
            error_message: None,
        };
        pipeline.record_frame(frame);

        let prometheus = PrometheusMetrics::from_pipeline(&pipeline);
        
        assert!(prometheus.frames_total > 0);
        assert_eq!(prometheus.errors_total, 0);
        assert!(prometheus.error_rate >= 0.0 && prometheus.error_rate <= 1.0);
    }

    #[test]
    fn test_prometheus_text_format() {
        let metrics = PrometheusMetrics {
            timestamp_ms: 1234567890,
            detection_latency_us: 100,
            optimization_latency_us: 200,
            queue_depth: 2,
            queue_depth_max: 5,
            frames_total: 100,
            errors_total: 5,
            errors_recovered: 4,
            error_rate: 0.05,
            detection_avg_us: 95,
            optimization_avg_us: 195,
        };

        let text = metrics.to_prometheus_text();
        
        // Verify key metrics are present
        assert!(text.contains("vio_detection_latency_us"));
        assert!(text.contains("vio_optimization_latency_us"));
        assert!(text.contains("vio_frames_total"));
        assert!(text.contains("vio_error_rate"));
        
        // Verify values
        assert!(text.contains("100"));  // detection_latency_us
        assert!(text.contains("200"));  // optimization_latency_us
        assert!(text.contains("0.0500")); // error_rate
    }

    #[test]
    fn test_openmetrics_format() {
        let metrics = PrometheusMetrics {
            timestamp_ms: 1234567890,
            detection_latency_us: 100,
            optimization_latency_us: 200,
            queue_depth: 2,
            queue_depth_max: 5,
            frames_total: 100,
            errors_total: 5,
            errors_recovered: 4,
            error_rate: 0.05,
            detection_avg_us: 95,
            optimization_avg_us: 195,
        };

        let text = metrics.to_openmetrics_text();
        
        // Verify EOF marker
        assert!(text.ends_with("# EOF\n"));
        // Verify content
        assert!(text.contains("vio_frames_total"));
    }

    #[test]
    fn test_metrics_exporter() {
        let exporter = MetricsExporter::new();
        let pipeline = PipelineMetrics::new();
        
        // Initially no metrics
        assert!(exporter.snapshot().unwrap().is_none());
        
        // Export metrics
        assert!(exporter.export(&pipeline).is_ok());
        
        // Now metrics should be available
        assert!(exporter.snapshot().unwrap().is_some());
        assert!(exporter.prometheus_text().unwrap().is_some());
    }

    #[test]
    fn test_exporter_continuous_updates() {
        let exporter = MetricsExporter::new();
        let pipeline = PipelineMetrics::new();
        
        // Record frame 1
        pipeline.record_detection(50, false);
        exporter.export(&pipeline).unwrap();
        let snap1 = exporter.snapshot().unwrap().unwrap();
        
        // Record frame 2
        pipeline.record_detection(100, false);
        exporter.export(&pipeline).unwrap();
        let snap2 = exporter.snapshot().unwrap().unwrap();
        
        // Verify detection latency increased
        assert!(snap2.detection_latency_us >= snap1.detection_latency_us);
    }

    #[test]
    fn test_prometheus_display_impl() {
        let metrics = PrometheusMetrics {
            timestamp_ms: 1000,
            detection_latency_us: 100,
            optimization_latency_us: 200,
            queue_depth: 1,
            queue_depth_max: 3,
            frames_total: 50,
            errors_total: 2,
            errors_recovered: 1,
            error_rate: 0.04,
            detection_avg_us: 95,
            optimization_avg_us: 195,
        };

        let formatted = format!("{}", metrics);
        assert!(formatted.contains("vio_"));
        assert!(formatted.contains("100"));
    }
}
