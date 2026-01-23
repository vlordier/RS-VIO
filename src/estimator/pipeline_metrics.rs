//! Pipeline metrics and observability
//!
//! Tracks performance and health metrics for the async VIO pipeline including:
//! - Per-stage latencies (feature detection, optimization)
//! - End-to-end frame processing latency
//! - Throughput (frames per second)
//! - Queue depth and backpressure
//! - Error counts and recovery statistics

use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use std::collections::VecDeque;

/// Metrics for a single pipeline stage
#[derive(Debug, Clone)]
pub struct StageMetrics {
    /// Number of frames processed in this stage
    pub frame_count: usize,
    /// Total time spent in this stage (µs)
    pub total_time_us: u64,
    /// Minimum latency (µs)
    pub min_time_us: u64,
    /// Maximum latency (µs)
    pub max_time_us: u64,
    /// Number of errors in this stage
    pub error_count: usize,
}

impl StageMetrics {
    pub fn new() -> Self {
        Self {
            frame_count: 0,
            total_time_us: 0,
            min_time_us: u64::MAX,
            max_time_us: 0,
            error_count: 0,
        }
    }

    pub fn avg_time_us(&self) -> u64 {
        if self.frame_count > 0 {
            self.total_time_us / self.frame_count as u64
        } else {
            0
        }
    }

    pub fn update(&mut self, elapsed_us: u64, error: bool) {
        self.frame_count += 1;
        self.total_time_us += elapsed_us;
        self.min_time_us = self.min_time_us.min(elapsed_us);
        self.max_time_us = self.max_time_us.max(elapsed_us);
        if error {
            self.error_count += 1;
        }
    }
}

impl Default for StageMetrics {
    fn default() -> Self {
        Self::new()
    }
}

/// Frame-level metrics snapshot
#[derive(Debug, Clone)]
pub struct FrameMetrics {
    pub frame_id: u64,
    pub timestamp_ns: i64,
    pub detection_time_us: u64,
    pub optimization_time_us: u64,
    pub e2e_time_us: u64,
    pub queue_depth: usize,
    pub success: bool,
    pub error_message: Option<String>,
}

/// Pipeline-wide metrics collector (thread-safe, lock-free for reads)
#[derive(Clone)]
pub struct PipelineMetrics {
    inner: Arc<PipelineMetricsInner>,
}

struct PipelineMetricsInner {
    // Per-stage metrics
    detection_stage: std::sync::Mutex<StageMetrics>,
    optimization_stage: std::sync::Mutex<StageMetrics>,
    
    // Frame history (circular buffer)
    frame_history: std::sync::Mutex<VecDeque<FrameMetrics>>,
    max_history_size: usize,
    
    // Atomic counters (for fast, lock-free access)
    total_frames: AtomicU64,
    total_errors: AtomicU64,
    current_queue_depth: AtomicUsize,
    max_queue_depth: AtomicUsize,
    recovered_errors: AtomicU64,
}

impl PipelineMetrics {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(PipelineMetricsInner {
                detection_stage: std::sync::Mutex::new(StageMetrics::new()),
                optimization_stage: std::sync::Mutex::new(StageMetrics::new()),
                frame_history: std::sync::Mutex::new(VecDeque::with_capacity(100)),
                max_history_size: 100,
                total_frames: AtomicU64::new(0),
                total_errors: AtomicU64::new(0),
                current_queue_depth: AtomicUsize::new(0),
                max_queue_depth: AtomicUsize::new(0),
                recovered_errors: AtomicU64::new(0),
            }),
        }
    }

    // Detection stage metrics
    pub fn record_detection(&self, elapsed_us: u64, error: bool) {
        let mut stage = self.inner.detection_stage.lock().unwrap();
        stage.update(elapsed_us, error);
    }

    pub fn detection_metrics(&self) -> StageMetrics {
        self.inner.detection_stage.lock().unwrap().clone()
    }

    // Optimization stage metrics
    pub fn record_optimization(&self, elapsed_us: u64, error: bool) {
        let mut stage = self.inner.optimization_stage.lock().unwrap();
        stage.update(elapsed_us, error);
    }

    pub fn optimization_metrics(&self) -> StageMetrics {
        self.inner.optimization_stage.lock().unwrap().clone()
    }

    // Frame tracking
    pub fn record_frame(&self, frame_metrics: FrameMetrics) {
        let mut history = self.inner.frame_history.lock().unwrap();
        if history.len() >= self.inner.max_history_size {
            history.pop_front();
        }
        history.push_back(frame_metrics.clone());

        self.inner.total_frames.fetch_add(1, Ordering::Relaxed);
        if !frame_metrics.success {
            self.inner.total_errors.fetch_add(1, Ordering::Relaxed);
        }
    }

    pub fn frame_history(&self) -> Vec<FrameMetrics> {
        self.inner
            .frame_history
            .lock()
            .unwrap()
            .iter()
            .cloned()
            .collect()
    }

    // Queue depth tracking
    pub fn set_queue_depth(&self, depth: usize) {
        self.inner.current_queue_depth.store(depth, Ordering::Relaxed);
        
        // Update max if needed
        let current_max = self.inner.max_queue_depth.load(Ordering::Relaxed);
        if depth > current_max {
            self.inner.max_queue_depth.store(depth, Ordering::Relaxed);
        }
    }

    pub fn queue_depth(&self) -> usize {
        self.inner.current_queue_depth.load(Ordering::Relaxed)
    }

    pub fn max_queue_depth(&self) -> usize {
        self.inner.max_queue_depth.load(Ordering::Relaxed)
    }

    // Error recovery tracking
    pub fn record_recovered_error(&self) {
        self.inner.recovered_errors.fetch_add(1, Ordering::Relaxed);
    }

    pub fn recovered_errors(&self) -> u64 {
        self.inner.recovered_errors.load(Ordering::Relaxed)
    }

    // Summary statistics
    pub fn total_frames_processed(&self) -> u64 {
        self.inner.total_frames.load(Ordering::Relaxed)
    }

    pub fn total_errors(&self) -> u64 {
        self.inner.total_errors.load(Ordering::Relaxed)
    }

    pub fn error_rate(&self) -> f64 {
        let total = self.total_frames_processed();
        if total == 0 {
            return 0.0;
        }
        (self.total_errors() as f64) / (total as f64)
    }

    pub fn throughput_fps(&self, elapsed: Duration) -> f64 {
        let frames = self.total_frames_processed() as f64;
        let secs = elapsed.as_secs_f64();
        if secs > 0.0 {
            frames / secs
        } else {
            0.0
        }
    }

    /// Generate summary report
    pub fn summary(&self) -> String {
        let detection = self.detection_metrics();
        let optimization = self.optimization_metrics();
        let total = self.total_frames_processed();
        let errors = self.total_errors();
        let recovered = self.recovered_errors();

        format!(
            "Pipeline Metrics Summary:\n\
             ├─ Total Frames: {}\n\
             ├─ Total Errors: {} ({:.2}%)\n\
             ├─ Recovered Errors: {}\n\
             ├─ Queue Depth: {}/{}\n\
             ├─ Detection Stage:\n\
             │  ├─ Avg: {:.1}µs, Min: {}µs, Max: {}µs\n\
             │  └─ Errors: {}\n\
             └─ Optimization Stage:\n\
                ├─ Avg: {:.1}µs, Min: {}µs, Max: {}µs\n\
                └─ Errors: {}",
            total,
            errors,
            if total > 0 { (errors as f64 / total as f64) * 100.0 } else { 0.0 },
            recovered,
            self.queue_depth(),
            self.max_queue_depth(),
            detection.avg_time_us() as f64,
            detection.min_time_us,
            detection.max_time_us,
            detection.error_count,
            optimization.avg_time_us() as f64,
            optimization.min_time_us,
            optimization.max_time_us,
            optimization.error_count,
        )
    }

    pub fn reset(&self) {
        let mut history = self.inner.frame_history.lock().unwrap();
        history.clear();
        
        let mut detection = self.inner.detection_stage.lock().unwrap();
        *detection = StageMetrics::new();
        
        let mut optimization = self.inner.optimization_stage.lock().unwrap();
        *optimization = StageMetrics::new();
        
        self.inner.total_frames.store(0, Ordering::Relaxed);
        self.inner.total_errors.store(0, Ordering::Relaxed);
        self.inner.recovered_errors.store(0, Ordering::Relaxed);
        self.inner.current_queue_depth.store(0, Ordering::Relaxed);
        self.inner.max_queue_depth.store(0, Ordering::Relaxed);
    }
}

impl Default for PipelineMetrics {
    fn default() -> Self {
        Self::new()
    }
}

/// Scoped timer for automatic latency tracking
pub struct MetricsTimer {
    start: Instant,
    metrics: PipelineMetrics,
    stage_fn: Box<dyn Fn(&PipelineMetrics, u64, bool) + Send + Sync>,
}

impl MetricsTimer {
    pub fn new_detection(metrics: &PipelineMetrics) -> Self {
        Self {
            start: Instant::now(),
            metrics: metrics.clone(),
            stage_fn: Box::new(|m, elapsed_us, error| m.record_detection(elapsed_us, error)),
        }
    }

    pub fn new_optimization(metrics: &PipelineMetrics) -> Self {
        Self {
            start: Instant::now(),
            metrics: metrics.clone(),
            stage_fn: Box::new(|m, elapsed_us, error| m.record_optimization(elapsed_us, error)),
        }
    }

    pub fn stop(self, error: bool) {
        let elapsed_us = self.start.elapsed().as_micros() as u64;
        (self.stage_fn)(&self.metrics, elapsed_us, error);
    }

    pub fn elapsed_us(&self) -> u64 {
        self.start.elapsed().as_micros() as u64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stage_metrics_averaging() {
        let mut stage = StageMetrics::new();
        stage.update(100, false);
        stage.update(200, false);
        stage.update(300, false);

        assert_eq!(stage.frame_count, 3);
        assert_eq!(stage.avg_time_us(), 200);
        assert_eq!(stage.min_time_us, 100);
        assert_eq!(stage.max_time_us, 300);
    }

    #[test]
    fn test_pipeline_metrics() {
        let metrics = PipelineMetrics::new();
        
        metrics.record_detection(150, false);
        metrics.record_detection(200, false);
        metrics.record_optimization(500, false);
        metrics.set_queue_depth(2);
        metrics.record_frame(FrameMetrics {
            frame_id: 0,
            timestamp_ns: 1000,
            detection_time_us: 175,
            optimization_time_us: 500,
            e2e_time_us: 675,
            queue_depth: 2,
            success: true,
            error_message: None,
        });

        assert_eq!(metrics.total_frames_processed(), 1);
        assert_eq!(metrics.total_errors(), 0);
        assert_eq!(metrics.queue_depth(), 2);
        assert_eq!(metrics.max_queue_depth(), 2);
    }

    #[test]
    fn test_error_rate() {
        let metrics = PipelineMetrics::new();
        
        // Record 6 frames with errors and 4 frames with success (60% error rate)
        for i in 0..10 {
            let is_error = i < 6;  // First 6 are errors, last 4 are successes
            let frame_metrics = FrameMetrics {
                frame_id: i,
                timestamp_ns: (i as i64) * 1000,
                detection_time_us: 100,
                optimization_time_us: 200,
                e2e_time_us: 300,
                queue_depth: 1,
                success: !is_error,
                error_message: if is_error { Some("test error".to_string()) } else { None },
            };
            metrics.record_frame(frame_metrics);
        }
        
        // 6 errors out of 10 = 0.6 error rate
        let rate = metrics.error_rate();
        assert!((rate - 0.6).abs() < 0.001, "Expected 0.6, got {}", rate);
    }

    #[test]
    fn test_metrics_timer() {
        let metrics = PipelineMetrics::new();
        {
            let timer = MetricsTimer::new_detection(&metrics);
            std::thread::sleep(Duration::from_millis(1));
            timer.stop(false);
        }
        
        let stage = metrics.detection_metrics();
        assert_eq!(stage.frame_count, 1);
        assert!(stage.total_time_us >= 1000); // At least 1ms
    }
}
