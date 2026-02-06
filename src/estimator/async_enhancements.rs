//! Enhanced AsyncEstimator features for production deployment
//!
//! This module provides advanced features built on top of AsyncEstimator:
//! - Priority queue-based frame scheduling
//! - Processing metrics and performance monitoring  
//! - Latency histograms and deadline tracking
//! - Worker thread panic recovery
//! - Advanced streaming pattern support
//! - Budget violation detection

use std::cmp::Ordering;

/// Processing metrics for real-time performance monitoring
#[derive(Clone, Debug, Default)]
pub struct ProcessingMetrics {
    /// Total frames successfully processed
    pub frames_processed: u64,
    /// Frames skipped due to overload
    pub frames_skipped: u64,
    /// Frames that exceeded deadline
    pub deadline_misses: u64,
    /// Frames that exceeded budget
    pub budget_violations: u64,
    /// Total processing time accumulated (nanoseconds)
    pub total_processing_time_ns: u64,
    /// Minimum single-frame processing time (nanoseconds)
    pub min_latency_ns: u64,
    /// Maximum single-frame processing time (nanoseconds)
    pub max_latency_ns: u64,
    /// Average frame processing time (nanoseconds)
    pub avg_latency_ns: u64,
}

impl ProcessingMetrics {
    /// Calculate average frames per second based on processing time
    pub fn avg_fps(&self) -> f64 {
        if self.total_processing_time_ns == 0 {
            return 0.0;
        }
        self.frames_processed as f64 / (self.total_processing_time_ns as f64 / 1_000_000_000.0)
    }

    /// Calculate frame skip rate as percentage
    pub fn skip_rate(&self) -> f64 {
        let total = self.frames_processed + self.frames_skipped;
        if total == 0 {
            return 0.0;
        }
        (self.frames_skipped as f64 / total as f64) * 100.0
    }

    /// Calculate deadline miss rate as percentage
    pub fn deadline_miss_rate(&self) -> f64 {
        if self.frames_processed == 0 {
            return 0.0;
        }
        (self.deadline_misses as f64 / self.frames_processed as f64) * 100.0
    }

    /// Calculate budget violation rate as percentage
    pub fn budget_violation_rate(&self) -> f64 {
        if self.frames_processed == 0 {
            return 0.0;
        }
        (self.budget_violations as f64 / self.frames_processed as f64) * 100.0
    }

    /// Get summary string for logging
    pub fn summary(&self) -> String {
        format!(
            "Metrics: {} frames, {} skipped ({:.1}%), {} deadline misses ({:.1}%), {} budget violations ({:.1}%), avg {:.2}ms latency",
            self.frames_processed,
            self.frames_skipped,
            self.skip_rate(),
            self.deadline_misses,
            self.deadline_miss_rate(),
            self.budget_violations,
            self.budget_violation_rate(),
            self.avg_latency_ns as f64 / 1_000_000.0
        )
    }
}

/// Latency histogram for performance analysis
#[derive(Clone, Debug, Default)]
pub struct LatencyHistogram {
    /// Bucket ranges in milliseconds: [0-1), [1-2), [2-5), [5-10), [10-33), [33-100), [100+)
    pub buckets: [u64; 7],
}

impl LatencyHistogram {
    /// Record a single latency measurement
    pub fn record(&mut self, latency_ns: u64) {
        let latency_ms = latency_ns as f64 / 1_000_000.0;
        let bucket_idx = match latency_ms {
            x if x < 1.0 => 0,
            x if x < 2.0 => 1,
            x if x < 5.0 => 2,
            x if x < 10.0 => 3,
            x if x < 33.0 => 4,
            x if x < 100.0 => 5,
            _ => 6,
        };
        self.buckets[bucket_idx] += 1;
    }

    /// Get percentile
    pub fn percentile(&self, p: f64) -> usize {
        let total: u64 = self.buckets.iter().sum();
        if total == 0 {
            return 0;
        }
        let target = (total as f64 * p / 100.0) as u64;
        let mut cumulative = 0;
        for (idx, &count) in self.buckets.iter().enumerate() {
            cumulative += count;
            if cumulative >= target {
                return idx;
            }
        }
        self.buckets.len() - 1
    }

    /// Get summary statistics
    pub fn summary(&self) -> String {
        let total: u64 = self.buckets.iter().sum();
        if total == 0 {
            return "No latency data".to_string();
        }

        let bounds = ["<1ms", "1-2ms", "2-5ms", "5-10ms", "10-33ms", "33-100ms", ">100ms"];
        let mut summary = String::new();
        for (idx, &count) in self.buckets.iter().enumerate() {
            let pct = (count as f64 / total as f64) * 100.0;
            summary.push_str(&format!("{}: {:.1}% ", bounds[idx], pct));
        }
        summary
    }
}

/// Priority queue entry for frame scheduling
#[derive(Clone, Debug)]
pub struct PriorityFrameEntry {
    pub priority: u8,
    pub frame_id: i64,
    pub timestamp_ns: i64,
}

impl PartialEq for PriorityFrameEntry {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority && self.frame_id == other.frame_id
    }
}

impl Eq for PriorityFrameEntry {}

impl PartialOrd for PriorityFrameEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PriorityFrameEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Higher priority first (BinaryHeap is max-heap, so higher values come first)
        self.priority.cmp(&other.priority)
            .then_with(|| self.timestamp_ns.cmp(&other.timestamp_ns))
    }
}

/// Deadline tracker for real-time frame processing
#[derive(Clone, Debug)]
pub struct DeadlineTracker {
    /// Frame deadline in nanoseconds from system start
    pub deadline_ns: i64,
    /// Frame ID
    pub frame_id: i64,
    /// Whether deadline was met
    pub met: bool,
}

impl DeadlineTracker {
    /// Create new deadline tracker
    pub fn new(deadline_ns: i64, frame_id: i64) -> Self {
        Self {
            deadline_ns,
            frame_id,
            met: false,
        }
    }

    /// Check if deadline is currently missed
    pub fn is_missed(&self, current_time_ns: i64) -> bool {
        current_time_ns > self.deadline_ns
    }

    /// Mark deadline as met
    pub fn mark_met(&mut self) {
        self.met = true;
    }
}

/// Streaming pattern analyzer for buffer management
#[derive(Clone, Debug, Default)]
pub struct StreamingPatternAnalyzer {
    /// Frame arrival times (last 10 frames)
    pub recent_arrivals_ns: Vec<i64>,
    /// Detected burst activity
    pub is_burst: bool,
    /// Average inter-frame interval (nanoseconds)
    pub avg_interval_ns: u64,
    /// Jitter in inter-frame interval (nanoseconds)
    pub jitter_ns: u64,
}

impl StreamingPatternAnalyzer {
    /// Update with new frame arrival
    pub fn update(&mut self, timestamp_ns: i64) {
        self.recent_arrivals_ns.push(timestamp_ns);
        if self.recent_arrivals_ns.len() > 10 {
            self.recent_arrivals_ns.remove(0);
        }

        if self.recent_arrivals_ns.len() >= 3 {
            let intervals: Vec<i64> = self.recent_arrivals_ns.windows(2)
                .map(|w| w[1] - w[0])
                .collect();

            let avg: i64 = intervals.iter().sum::<i64>() as i64 / intervals.len() as i64;
            self.avg_interval_ns = avg as u64;

            // Calculate jitter
            let variance: i64 = intervals.iter()
                .map(|&i| (i - avg).pow(2))
                .sum::<i64>() / intervals.len() as i64;
            self.jitter_ns = (variance as f64).sqrt() as u64;

            // Detect burst: high jitter or rapid arrivals
            self.is_burst = self.jitter_ns > avg as u64 / 2 || self.avg_interval_ns < 5_000_000; // 5ms
        }
    }

    /// Get description of current streaming pattern
    pub fn description(&self) -> String {
        if self.recent_arrivals_ns.len() < 2 {
            return "Insufficient data".to_string();
        }
        format!(
            "avg_interval: {:.2}ms, jitter: {:.2}ms, pattern: {}",
            self.avg_interval_ns as f64 / 1_000_000.0,
            self.jitter_ns as f64 / 1_000_000.0,
            if self.is_burst { "BURST" } else { "STEADY" }
        )
    }
}

/// Failure recovery tracker
#[derive(Clone, Debug)]
pub struct FailureRecoveryTracker {
    /// Number of worker thread panics recovered from
    pub panic_recoveries: u64,
    /// Last panic timestamp (nanoseconds from system start)
    pub last_panic_ns: Option<i64>,
    /// Worker thread status
    pub worker_healthy: bool,
}

impl Default for FailureRecoveryTracker {
    fn default() -> Self {
        Self {
            panic_recoveries: 0,
            last_panic_ns: None,
            worker_healthy: true, // Worker starts healthy
        }
    }
}

impl FailureRecoveryTracker {
    /// Record a panic recovery event
    pub fn record_panic_recovery(&mut self, timestamp_ns: i64) {
        self.panic_recoveries += 1;
        self.last_panic_ns = Some(timestamp_ns);
        self.worker_healthy = false;
    }

    /// Mark worker as healthy
    pub fn mark_worker_healthy(&mut self) {
        self.worker_healthy = true;
    }

    /// Get recovery status
    pub fn status(&self) -> String {
        match self.last_panic_ns {
            Some(ts) => format!(
                "Recovered from {} panics, last at {}ns, healthy: {}",
                self.panic_recoveries, ts, self.worker_healthy
            ),
            None => "No panics recorded".to_string(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BinaryHeap;

    #[test]
    fn test_processing_metrics_calculations() {
        let metrics = ProcessingMetrics {
            frames_processed: 100,
            frames_skipped: 10,
            deadline_misses: 5,
            budget_violations: 8,
            total_processing_time_ns: 3_300_000_000, // 3.3 seconds
            min_latency_ns: 20_000_000,               // 20ms
            max_latency_ns: 50_000_000,               // 50ms
            avg_latency_ns: 33_000_000,               // 33ms
        };

        assert!((metrics.avg_fps() - 30.3).abs() < 0.1, "FPS calculation");
        assert!((metrics.skip_rate() - 9.09).abs() < 0.1, "Skip rate");
        assert!((metrics.deadline_miss_rate() - 5.0).abs() < 0.1, "Deadline miss rate");
        assert!((metrics.budget_violation_rate() - 8.0).abs() < 0.1, "Budget violation rate");
    }

    #[test]
    fn test_latency_histogram_bucketing() {
        let mut histogram = LatencyHistogram::default();

        // Add samples to different buckets
        histogram.record(500_000);       // 0.5ms -> bucket 0
        histogram.record(1_500_000);     // 1.5ms -> bucket 1
        histogram.record(3_000_000);     // 3ms -> bucket 2
        histogram.record(7_000_000);     // 7ms -> bucket 3
        histogram.record(20_000_000);    // 20ms -> bucket 4
        histogram.record(50_000_000);    // 50ms -> bucket 5
        histogram.record(150_000_000);   // 150ms -> bucket 6

        assert_eq!(histogram.buckets[0], 1);
        assert_eq!(histogram.buckets[1], 1);
        assert_eq!(histogram.buckets[2], 1);
        assert_eq!(histogram.buckets[3], 1);
        assert_eq!(histogram.buckets[4], 1);
        assert_eq!(histogram.buckets[5], 1);
        assert_eq!(histogram.buckets[6], 1);
    }

    #[test]
    fn test_priority_frame_ordering() {
        let mut heap = BinaryHeap::new();

        // Add frames with different priorities (higher number = higher priority)
        heap.push(PriorityFrameEntry {
            priority: 5,
            frame_id: 1,
            timestamp_ns: 0,
        });
        heap.push(PriorityFrameEntry {
            priority: 10,
            frame_id: 2,
            timestamp_ns: 0,
        });
        heap.push(PriorityFrameEntry {
            priority: 3,
            frame_id: 3,
            timestamp_ns: 0,
        });

        // Higher priority should come out first
        assert_eq!(heap.pop().unwrap().priority, 10);
        assert_eq!(heap.pop().unwrap().priority, 5);
        assert_eq!(heap.pop().unwrap().priority, 3);
    }

    #[test]
    fn test_streaming_pattern_analyzer() {
        let mut analyzer = StreamingPatternAnalyzer::default();

        // Simulate steady streaming
        let base_time = 1_000_000_000i64;
        let interval = 33_333_333i64; // ~30fps

        analyzer.update(base_time);
        analyzer.update(base_time + interval);
        analyzer.update(base_time + 2 * interval);
        analyzer.update(base_time + 3 * interval);

        assert!(!analyzer.is_burst);
        assert!(analyzer.avg_interval_ns > 0);
    }

    #[test]
    fn test_failure_recovery_tracker() {
        let mut tracker = FailureRecoveryTracker::default();

        assert_eq!(tracker.panic_recoveries, 0);
        assert!(tracker.worker_healthy);

        tracker.record_panic_recovery(1000);
        assert_eq!(tracker.panic_recoveries, 1);
        assert!(!tracker.worker_healthy);

        tracker.mark_worker_healthy();
        assert!(tracker.worker_healthy);
    }
}
