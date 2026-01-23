//! Embedded HTTP server for metrics export
//!
//! Provides `/metrics` endpoint for Prometheus scraping and OpenTelemetry collection
//! Supports both text and JSON formats based on content-type negotiation

use crate::estimator::{
    metrics_export::MetricsExporter, otel_exporter::OtelMetricsExporter, PipelineMetrics,
};
use std::net::SocketAddr;
use std::sync::Arc;

/// HTTP metrics server configuration
#[derive(Debug, Clone)]
pub struct MetricsServerConfig {
    /// Port to listen on (default: 9090)
    pub port: u16,
    /// Host to bind to (default: 127.0.0.1)
    pub host: String,
    /// Enable Prometheus text format (default: true)
    pub enable_prometheus: bool,
    /// Enable OpenTelemetry JSON format (default: true)
    pub enable_otel: bool,
    /// Export interval in milliseconds
    pub export_interval_ms: u64,
}

impl Default for MetricsServerConfig {
    fn default() -> Self {
        Self {
            port: 9090,
            host: "127.0.0.1".to_string(),
            enable_prometheus: true,
            enable_otel: true,
            export_interval_ms: 1000,
        }
    }
}

impl MetricsServerConfig {
    /// Create configuration with custom port
    pub fn with_port(mut self, port: u16) -> Self {
        self.port = port;
        self
    }

    /// Create configuration with custom host
    pub fn with_host(mut self, host: impl Into<String>) -> Self {
        self.host = host.into();
        self
    }

    /// Get socket address
    pub fn socket_addr(&self) -> Result<SocketAddr, Box<dyn std::error::Error>> {
        format!("{}:{}", self.host, self.port)
            .parse()
            .map_err(|e| Box::new(e) as _)
    }
}

/// HTTP metrics server
pub struct MetricsServer {
    config: MetricsServerConfig,
    prometheus_exporter: MetricsExporter,
    otel_exporter: OtelMetricsExporter,
    pipeline_metrics: Arc<PipelineMetrics>,
}

impl MetricsServer {
    /// Create new metrics server
    pub fn new(config: MetricsServerConfig, pipeline_metrics: Arc<PipelineMetrics>) -> Self {
        Self {
            config,
            prometheus_exporter: MetricsExporter::new(),
            otel_exporter: OtelMetricsExporter::new(),
            pipeline_metrics,
        }
    }

    /// Create metrics server with default configuration
    pub fn with_default_config(pipeline_metrics: Arc<PipelineMetrics>) -> Self {
        Self::new(MetricsServerConfig::default(), pipeline_metrics)
    }

    /// Get Prometheus-format metrics
    pub fn prometheus_metrics(&self) -> Result<String, String> {
        self.prometheus_exporter
            .prometheus_text()
            .map_err(|e| e.to_string())?
            .ok_or_else(|| "No metrics available".to_string())
    }

    /// Get OpenTelemetry-format metrics
    pub fn otel_metrics(&self) -> Result<String, String> {
        self.otel_exporter
            .get_otlp_json()
            .map_err(|e| e.to_string())
    }

    /// Update metrics from pipeline
    pub fn refresh_metrics(&self) -> Result<(), String> {
        self.prometheus_exporter
            .export(&self.pipeline_metrics)
            .map_err(|e| e.to_string())?;

        self.otel_exporter
            .export(&self.pipeline_metrics)
            .map_err(|e| e.to_string())?;

        Ok(())
    }

    /// Get server configuration
    pub fn config(&self) -> &MetricsServerConfig {
        &self.config
    }

    /// Get metrics endpoint URL
    pub fn metrics_url(&self) -> String {
        format!("http://{}:{}/metrics", self.config.host, self.config.port)
    }
}

/// Metrics handler response format
#[derive(Debug, Clone)]
pub enum MetricsResponse {
    /// Prometheus text format (text/plain)
    PrometheusText(String),
    /// OpenMetrics text format (text/openmetrics-exposition)
    OpenMetricsText(String),
    /// OTLP JSON format (application/json)
    OtelJson(String),
}

impl MetricsResponse {
    /// Get content-type header
    pub fn content_type(&self) -> &'static str {
        match self {
            Self::PrometheusText(_) => "text/plain; charset=utf-8",
            Self::OpenMetricsText(_) => "text/openmetrics-exposition; version=1.0.0; charset=utf-8",
            Self::OtelJson(_) => "application/json; charset=utf-8",
        }
    }

    /// Get response body
    pub fn body(&self) -> &str {
        match self {
            Self::PrometheusText(s) | Self::OpenMetricsText(s) | Self::OtelJson(s) => s,
        }
    }
}

/// Determine response format based on accept header
pub fn negotiate_metrics_format(
    accept_header: Option<&str>,
    metrics_server: &MetricsServer,
) -> Result<MetricsResponse, String> {
    if let Some(accept) = accept_header {
        if accept.contains("application/json") {
            let json = metrics_server.otel_metrics()?;
            return Ok(MetricsResponse::OtelJson(json));
        } else if accept.contains("openmetrics") {
            let text = metrics_server.prometheus_metrics()?;
            return Ok(MetricsResponse::OpenMetricsText(text));
        }
    }

    // Default to Prometheus format
    let text = metrics_server.prometheus_metrics()?;
    Ok(MetricsResponse::PrometheusText(text))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_metrics_server_config() {
        let config = MetricsServerConfig::default();
        assert_eq!(config.port, 9090);
        assert_eq!(config.host, "127.0.0.1");
        assert!(config.enable_prometheus);
        assert!(config.enable_otel);
    }

    #[test]
    fn test_config_with_port() {
        let config = MetricsServerConfig::default().with_port(8080);
        assert_eq!(config.port, 8080);
    }

    #[test]
    fn test_config_with_host() {
        let config = MetricsServerConfig::default().with_host("0.0.0.0");
        assert_eq!(config.host, "0.0.0.0");
    }

    #[test]
    fn test_socket_addr() {
        let config = MetricsServerConfig::default();
        let addr = config.socket_addr().unwrap();
        assert_eq!(addr.port(), 9090);
    }

    #[test]
    fn test_metrics_server_creation() {
        let metrics = Arc::new(PipelineMetrics::new());
        let server = MetricsServer::with_default_config(metrics);

        assert_eq!(server.config.port, 9090);
        assert_eq!(server.metrics_url(), "http://127.0.0.1:9090/metrics");
    }

    #[test]
    fn test_prometheus_metrics() {
        let metrics = Arc::new(PipelineMetrics::new());
        let server = MetricsServer::with_default_config(metrics);

        server.refresh_metrics().unwrap();
        let prom = server.prometheus_metrics().unwrap();

        assert!(prom.contains("vio_"));
    }

    #[test]
    fn test_otel_metrics() {
        let metrics = Arc::new(PipelineMetrics::new());
        let server = MetricsServer::with_default_config(metrics);

        server.refresh_metrics().unwrap();
        let otel = server.otel_metrics().unwrap();

        assert!(otel.contains("vio."));
    }

    #[test]
    fn test_metrics_response_content_type() {
        let resp = MetricsResponse::PrometheusText("test".to_string());
        assert!(resp.content_type().contains("text/plain"));

        let resp = MetricsResponse::OtelJson("{}".to_string());
        assert!(resp.content_type().contains("application/json"));
    }

    #[test]
    fn test_format_negotiation_default() {
        let metrics = Arc::new(PipelineMetrics::new());
        let server = MetricsServer::with_default_config(metrics);
        server.refresh_metrics().unwrap();

        let resp = negotiate_metrics_format(None, &server).unwrap();
        assert!(resp.content_type().contains("text/plain"));
    }

    #[test]
    fn test_format_negotiation_json() {
        let metrics = Arc::new(PipelineMetrics::new());
        let server = MetricsServer::with_default_config(metrics);
        server.refresh_metrics().unwrap();

        let resp = negotiate_metrics_format(Some("application/json"), &server).unwrap();
        assert!(resp.content_type().contains("application/json"));
    }

    #[test]
    fn test_format_negotiation_openmetrics() {
        let metrics = Arc::new(PipelineMetrics::new());
        let server = MetricsServer::with_default_config(metrics);
        server.refresh_metrics().unwrap();

        let resp = negotiate_metrics_format(Some("openmetrics"), &server).unwrap();
        assert!(resp.content_type().contains("openmetrics"));
    }
}
