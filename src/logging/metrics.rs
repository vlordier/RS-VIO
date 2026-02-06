//! Real-time performance metrics collection and reporting

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Performance metric sample (value + timestamp)
#[derive(Clone, Debug)]
pub struct MetricSample {
    pub timestamp: Instant,
    pub value: f64,
}

/// Real-time metrics aggregator with sliding window statistics
pub struct PerformanceMetrics {
    fps_samples: Arc<Mutex<VecDeque<MetricSample>>>,
    latency_samples: Arc<Mutex<VecDeque<MetricSample>>>,
    throughput_samples: Arc<Mutex<VecDeque<MetricSample>>>,
    window_size: Duration,
    max_samples: usize,
}

impl PerformanceMetrics {
    /// Create new metrics collector
    pub fn new(window_size: Duration, max_samples: usize) -> Self {
        Self {
            fps_samples: Arc::new(Mutex::new(VecDeque::new())),
            latency_samples: Arc::new(Mutex::new(VecDeque::new())),
            throughput_samples: Arc::new(Mutex::new(VecDeque::new())),
            window_size,
            max_samples,
        }
    }

    /// Record FPS measurement
    pub fn record_fps(&self, fps: f64) {
        self._add_sample(&self.fps_samples, fps);
    }

    /// Record latency measurement (milliseconds)
    pub fn record_latency(&self, latency_ms: f64) {
        self._add_sample(&self.latency_samples, latency_ms);
    }

    /// Record throughput measurement (frames per second)
    pub fn record_throughput(&self, throughput: f64) {
        self._add_sample(&self.throughput_samples, throughput);
    }

    /// Get average FPS over window
    pub fn avg_fps(&self) -> Option<f64> {
        self._average(&self.fps_samples)
    }

    /// Get average latency over window (milliseconds)
    pub fn avg_latency(&self) -> Option<f64> {
        self._average(&self.latency_samples)
    }

    /// Get average throughput over window
    pub fn avg_throughput(&self) -> Option<f64> {
        self._average(&self.throughput_samples)
    }

    /// Get min/max/std for FPS
    pub fn fps_stats(&self) -> Option<MetricStats> {
        self._stats(&self.fps_samples)
    }

    /// Get min/max/std for latency
    pub fn latency_stats(&self) -> Option<MetricStats> {
        self._stats(&self.latency_samples)
    }

    /// Get min/max/std for throughput
    pub fn throughput_stats(&self) -> Option<MetricStats> {
        self._stats(&self.throughput_samples)
    }

    /// Get comprehensive performance report
    pub fn report(&self) -> PerformanceReport {
        PerformanceReport {
            fps_avg: self.avg_fps(),
            fps_stats: self.fps_stats(),
            latency_avg: self.avg_latency(),
            latency_stats: self.latency_stats(),
            throughput_avg: self.avg_throughput(),
            throughput_stats: self.throughput_stats(),
        }
    }

    /// Add sample and clean old entries
    fn _add_sample(&self, samples: &Arc<Mutex<VecDeque<MetricSample>>>, value: f64) {
        let now = Instant::now();
        match samples.lock() {
            Ok(mut samples) => {
                // Remove old samples outside window
                while let Some(front) = samples.front() {
                    if now.duration_since(front.timestamp) > self.window_size {
                        samples.pop_front();
                    } else {
                        break;
                    }
                }

                // Add new sample
                samples.push_back(MetricSample {
                    timestamp: now,
                    value,
                });

                // Limit total samples
                while samples.len() > self.max_samples {
                    samples.pop_front();
                }
            }
            Err(e) => {
                log::error!("Failed to acquire metrics samples lock: {}", e);
            }
        }
    }

    /// Calculate average of samples in window
    fn _average(&self, samples: &Arc<Mutex<VecDeque<MetricSample>>>) -> Option<f64> {
        match samples.lock() {
            Ok(samples) => {
                if samples.is_empty() {
                    return None;
                }
                let sum: f64 = samples.iter().map(|s| s.value).sum();
                Some(sum / samples.len() as f64)
            }
            Err(e) => {
                log::error!("Failed to acquire metrics samples lock for average: {}", e);
                None
            }
        }
    }

    /// Calculate min/max/std of samples in window
    fn _stats(&self, samples: &Arc<Mutex<VecDeque<MetricSample>>>) -> Option<MetricStats> {
        match samples.lock() {
            Ok(samples) => {
                if samples.is_empty() {
                    return None;
                }

                let values: Vec<f64> = samples.iter().map(|s| s.value).collect();
                let count = values.len() as f64;

                let min = values.iter().cloned().fold(f64::INFINITY, f64::min);
                let max = values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                let mean = values.iter().sum::<f64>() / count;

                let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / count;
                let std_dev = variance.sqrt();

                Some(MetricStats {
                    min,
                    max,
                    mean,
                    std_dev,
                    count: values.len(),
                })
            }
            Err(e) => {
                log::error!("Failed to acquire metrics samples lock for stats: {}", e);
                None
            }
        }
    }
}

/// Statistics for a single metric
#[derive(Clone, Debug)]
pub struct MetricStats {
    pub min: f64,
    pub max: f64,
    pub mean: f64,
    pub std_dev: f64,
    pub count: usize,
}

/// Complete performance report
#[derive(Clone, Debug)]
pub struct PerformanceReport {
    pub fps_avg: Option<f64>,
    pub fps_stats: Option<MetricStats>,
    pub latency_avg: Option<f64>,
    pub latency_stats: Option<MetricStats>,
    pub throughput_avg: Option<f64>,
    pub throughput_stats: Option<MetricStats>,
}

impl std::fmt::Display for PerformanceReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "╔════════════════════════════════════════════╗")?;
        writeln!(f, "║ Real-time Performance Report               ║")?;
        writeln!(f, "╠════════════════════════════════════════════╣")?;

        if let Some(fps) = self.fps_avg {
            writeln!(f, "║ FPS (average): {:.2} fps                   ║", fps)?;
        }
        if let Some(stats) = &self.fps_stats {
            writeln!(
                f,
                "║   Range: {:.2} - {:.2} fps (σ={:.2})          ║",
                stats.min, stats.max, stats.std_dev
            )?;
        }

        if let Some(latency) = self.latency_avg {
            writeln!(f, "║ Latency (average): {:.2} ms               ║", latency)?;
        }
        if let Some(stats) = &self.latency_stats {
            writeln!(
                f,
                "║   Range: {:.2} - {:.2} ms (σ={:.2})        ║",
                stats.min, stats.max, stats.std_dev
            )?;
        }

        if let Some(throughput) = self.throughput_avg {
            writeln!(
                f,
                "║ Throughput (average): {:.2} frames/s        ║",
                throughput
            )?;
        }

        writeln!(f, "╚════════════════════════════════════════════╝")?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_performance_metrics_creation() {
        let metrics = PerformanceMetrics::new(Duration::from_secs(10), 1000);
        assert!(metrics.avg_fps().is_none());
    }

    #[test]
    fn test_record_fps() {
        let metrics = PerformanceMetrics::new(Duration::from_secs(10), 1000);
        metrics.record_fps(30.0);
        metrics.record_fps(31.0);
        metrics.record_fps(29.5);

        let avg = metrics.avg_fps();
        assert!(avg.is_some());
        assert!((avg.unwrap() - 30.17).abs() < 0.01);
    }

    #[test]
    fn test_metric_stats() {
        let metrics = PerformanceMetrics::new(Duration::from_secs(10), 1000);
        metrics.record_latency(10.0);
        metrics.record_latency(15.0);
        metrics.record_latency(20.0);

        let stats = metrics.latency_stats();
        assert!(stats.is_some());
        let s = stats.unwrap();
        assert_eq!(s.min, 10.0);
        assert_eq!(s.max, 20.0);
        assert!((s.mean - 15.0).abs() < 0.01);
    }
}
