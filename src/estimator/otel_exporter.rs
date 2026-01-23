//! OpenTelemetry metrics export for distributed tracing and monitoring
//!
//! Implements OTLP (OpenTelemetry Line Protocol) compatible metric export
//! for integration with OpenTelemetry collectors and backends (Jaeger, Datadog, etc.)

use crate::estimator::PipelineMetrics;
use std::collections::HashMap;
use std::fmt;
use std::sync::Arc;

/// OpenTelemetry metric attribute (key-value pair)
#[derive(Debug, Clone, PartialEq)]
pub struct MetricAttribute {
    pub key: String,
    pub value: String,
}

impl MetricAttribute {
    /// Create new metric attribute
    pub fn new(key: impl Into<String>, value: impl Into<String>) -> Self {
        Self {
            key: key.into(),
            value: value.into(),
        }
    }
}

/// OpenTelemetry metric data point
#[derive(Debug, Clone)]
pub struct MetricDataPoint {
    /// Metric name
    pub name: String,
    /// Metric value
    pub value: f64,
    /// Attributes (labels/tags)
    pub attributes: Vec<MetricAttribute>,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
    /// Unit of measurement
    pub unit: String,
}

impl MetricDataPoint {
    /// Create new metric data point
    pub fn new(name: impl Into<String>, value: f64) -> Self {
        Self {
            name: name.into(),
            value,
            attributes: Vec::new(),
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(0),
            unit: String::new(),
        }
    }

    /// Add attribute to data point
    pub fn with_attribute(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.attributes.push(MetricAttribute::new(key, value));
        self
    }

    /// Set unit of measurement
    pub fn with_unit(mut self, unit: impl Into<String>) -> Self {
        self.unit = unit.into();
        self
    }

    /// Format as OTLP JSON
    pub fn to_otlp_json(&self) -> String {
        let attrs = self.attributes.iter()
            .map(|a| format!(r#""{}":"{}""#, a.key, a.value))
            .collect::<Vec<_>>()
            .join(",");
        
        format!(
            r#"{{"name":"{}","value":{},"unit":"{}","attributes"{{{}}},"timestamp_ns":{}}}"#,
            self.name,
            self.value,
            self.unit,
            attrs,
            self.timestamp_ns
        )
    }
}

/// OpenTelemetry metrics batch export
#[derive(Debug, Clone)]
pub struct OtelMetricsBatch {
    /// Collected data points
    pub data_points: Vec<MetricDataPoint>,
    /// Resource attributes (service name, version, etc.)
    pub resource_attributes: HashMap<String, String>,
    /// Scope name (instrumentation library)
    pub scope_name: String,
}

impl OtelMetricsBatch {
    /// Create new metrics batch
    pub fn new() -> Self {
        Self {
            data_points: Vec::new(),
            resource_attributes: HashMap::new(),
            scope_name: "rs-vio-pipeline".to_string(),
        }
    }

    /// Add data point to batch
    pub fn add_point(&mut self, point: MetricDataPoint) {
        self.data_points.push(point);
    }

    /// Set resource attribute (e.g., service name)
    pub fn set_resource_attribute(&mut self, key: impl Into<String>, value: impl Into<String>) {
        self.resource_attributes.insert(key.into(), value.into());
    }

    /// Convert to OTLP JSON array
    pub fn to_otlp_json(&self) -> String {
        let points = self.data_points.iter()
            .map(|p| p.to_otlp_json())
            .collect::<Vec<_>>()
            .join(",");
        
        let resources = self.resource_attributes.iter()
            .map(|(k, v)| format!(r#""{}":"{}""#, k, v))
            .collect::<Vec<_>>()
            .join(",");

        format!(
            r#"{{"resource"{{{}}},"scope_name":"{}","metrics":[{}]}}"#,
            resources,
            self.scope_name,
            points
        )
    }

    /// Convert pipeline metrics to OTLP batch
    pub fn from_pipeline(metrics: &PipelineMetrics) -> Self {
        let mut batch = OtelMetricsBatch::new();
        
        // Set resource attributes
        batch.set_resource_attribute("service.name", "rs-vio-pipeline");
        batch.set_resource_attribute("service.version", env!("CARGO_PKG_VERSION"));
        
        let det = metrics.detection_metrics();
        let opt = metrics.optimization_metrics();
        
        // Add detection metrics
        batch.add_point(
            MetricDataPoint::new("vio.detection.latency", det.avg_time_us() as f64)
                .with_attribute("stage", "detection")
                .with_unit("us")
        );

        // Add optimization metrics
        batch.add_point(
            MetricDataPoint::new("vio.optimization.latency", opt.avg_time_us() as f64)
                .with_attribute("stage", "optimization")
                .with_unit("us")
        );

        // Add queue metrics
        batch.add_point(
            MetricDataPoint::new("vio.queue.depth", metrics.queue_depth() as f64)
                .with_attribute("type", "current")
        );

        batch.add_point(
            MetricDataPoint::new("vio.queue.depth_max", metrics.max_queue_depth() as f64)
                .with_attribute("type", "maximum")
        );

        // Add frame metrics
        batch.add_point(
            MetricDataPoint::new("vio.frames.total", metrics.total_frames_processed() as f64)
                .with_unit("frames")
        );

        // Add error metrics
        batch.add_point(
            MetricDataPoint::new("vio.errors.total", metrics.total_errors() as f64)
                .with_unit("count")
        );

        batch.add_point(
            MetricDataPoint::new("vio.errors.recovered", metrics.recovered_errors() as f64)
                .with_unit("count")
        );

        batch.add_point(
            MetricDataPoint::new("vio.error.rate", metrics.error_rate())
                .with_unit("ratio")
        );

        batch
    }
}

impl Default for OtelMetricsBatch {
    fn default() -> Self {
        Self::new()
    }
}

/// OpenTelemetry metrics exporter
pub struct OtelMetricsExporter {
    batch: Arc<std::sync::Mutex<OtelMetricsBatch>>,
}

impl OtelMetricsExporter {
    /// Create new OpenTelemetry metrics exporter
    pub fn new() -> Self {
        Self {
            batch: Arc::new(std::sync::Mutex::new(OtelMetricsBatch::new())),
        }
    }

    /// Export metrics from pipeline
    pub fn export(&self, metrics: &PipelineMetrics) -> Result<(), OtelError> {
        let batch = OtelMetricsBatch::from_pipeline(metrics);
        let mut guard = self.batch.lock().map_err(|_| OtelError::ExportFailed("Lock failed".to_string()))?;
        *guard = batch;
        Ok(())
    }

    /// Get current batch as OTLP JSON
    pub fn get_otlp_json(&self) -> Result<String, OtelError> {
        let batch = self.batch.lock().map_err(|_| OtelError::ExportFailed("Lock failed".to_string()))?;
        Ok(batch.to_otlp_json())
    }

    /// Get metrics as formatted string for stdout
    pub fn to_string(&self) -> Result<String, OtelError> {
        let batch = self.batch.lock().map_err(|_| OtelError::ExportFailed("Lock failed".to_string()))?;
        Ok(format!("{:#?}", batch))
    }
}

impl Default for OtelMetricsExporter {
    fn default() -> Self {
        Self::new()
    }
}

impl Clone for OtelMetricsExporter {
    fn clone(&self) -> Self {
        Self {
            batch: Arc::clone(&self.batch),
        }
    }
}

/// Error type for OpenTelemetry export operations
#[derive(Debug, Clone)]
pub enum OtelError {
    /// Serialization error
    SerializationFailed(String),
    /// Network error
    NetworkError(String),
    /// Export failed
    ExportFailed(String),
}

impl fmt::Display for OtelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SerializationFailed(msg) => write!(f, "Serialization error: {}", msg),
            Self::NetworkError(msg) => write!(f, "Network error: {}", msg),
            Self::ExportFailed(msg) => write!(f, "Export failed: {}", msg),
        }
    }
}

impl std::error::Error for OtelError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::{FrameMetrics, PipelineMetrics};

    #[test]
    fn test_metric_attribute() {
        let attr = MetricAttribute::new("stage", "detection");
        assert_eq!(attr.key, "stage");
        assert_eq!(attr.value, "detection");
    }

    #[test]
    fn test_metric_data_point() {
        let point = MetricDataPoint::new("vio.latency", 100.5)
            .with_attribute("stage", "detection")
            .with_unit("us");

        assert_eq!(point.name, "vio.latency");
        assert_eq!(point.value, 100.5);
        assert_eq!(point.unit, "us");
        assert_eq!(point.attributes.len(), 1);
    }

    #[test]
    fn test_metric_data_point_otlp_json() {
        let point = MetricDataPoint::new("vio.latency", 100.5)
            .with_unit("us");

        let json = point.to_otlp_json();
        assert!(json.contains("vio.latency"));
        assert!(json.contains("100.5"));
        assert!(json.contains("us"));
    }

    #[test]
    fn test_otel_batch_creation() {
        let mut batch = OtelMetricsBatch::new();
        
        batch.set_resource_attribute("service.name", "test-service");
        batch.add_point(MetricDataPoint::new("test.metric", 42.0));

        assert_eq!(batch.resource_attributes.len(), 1);
        assert_eq!(batch.data_points.len(), 1);
    }

    #[test]
    fn test_otel_batch_json() {
        let mut batch = OtelMetricsBatch::new();
        
        batch.set_resource_attribute("service.name", "test-service");
        batch.add_point(MetricDataPoint::new("test.metric", 42.0));

        let json = batch.to_otlp_json();
        assert!(json.contains("test.metric"));
        assert!(json.contains("42"));
        assert!(json.contains("service.name"));
    }

    #[test]
    fn test_otel_batch_from_pipeline() {
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
            queue_depth: 1,
            success: true,
            error_message: None,
        };
        pipeline.record_frame(frame);

        let batch = OtelMetricsBatch::from_pipeline(&pipeline);
        
        assert!(batch.resource_attributes.contains_key("service.name"));
        assert!(!batch.data_points.is_empty());
    }

    #[test]
    fn test_otel_exporter() {
        let exporter = OtelMetricsExporter::new();
        let pipeline = PipelineMetrics::new();
        
        assert!(exporter.export(&pipeline).is_ok());
        let json = exporter.get_otlp_json().unwrap();
        assert!(json.contains("vio."));
    }

    #[test]
    fn test_otel_error_display() {
        let err = OtelError::SerializationFailed("test".to_string());
        assert!(format!("{}", err).contains("Serialization"));
    }
}
