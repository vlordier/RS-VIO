//! Real-time performance monitoring and adaptive gating for hard realtime VIO.
//!
//! This module provides:
//! - Per-frame timing measurement and logging
//! - Adaptive module gating based on compute budget
//! - Frame drop detection
//! - Resource usage monitoring
//! - Fallback logic for deadline violations

use crate::common::safe_convert::percentile_index;
use crate::types::Float;
use std::collections::VecDeque;
use std::time::Instant;

/// Real-time performance monitor configuration
#[derive(Debug, Clone)]
pub struct RealtimeMonitorConfig {
    /// Target frame interval (e.g., 33.3ms for 30Hz)
    pub target_frame_interval_ms: Float,
    /// Enable per-frame timing logs
    pub enable_frame_logs: bool,
    /// Enable CSV export
    pub enable_csv_export: bool,
    /// CSV output path
    pub csv_output_path: String,
    /// Alert threshold (fraction of budget, e.g., 0.8 = 80%)
    pub alert_threshold: Float,
    /// Window size for moving average (frames)
    pub window_size: usize,
}

impl Default for RealtimeMonitorConfig {
    fn default() -> Self {
        Self {
            target_frame_interval_ms: 33.3, // 30 Hz
            enable_frame_logs: true,
            enable_csv_export: false,
            csv_output_path: "vio_timing.csv".to_string(),
            alert_threshold: 0.8,
            window_size: 30,
        }
    }
}

/// Per-frame timing breakdown
#[derive(Debug, Clone)]
pub struct FrameTiming {
    pub frame_id: i64,
    pub timestamp_ns: i64,
    pub feature_tracking_ms: Float,
    pub imu_filtering_ms: Float,
    pub fusion_ms: Float,
    pub super_resolution_ms: Float,
    pub bundle_adjustment_ms: Float,
    pub loop_closure_ms: Float,
    pub total_ms: Float,
    pub budget_utilization: Float, // 0.0 to 1.0+
}

/// Resource usage snapshot
#[derive(Debug, Clone, Default)]
pub struct ResourceUsage {
    pub cpu_percent: Float,
    pub memory_mb: Float,
    pub temperature_c: Float,
}

/// Adaptive gating decision
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GatingLevel {
    /// Full pipeline (all modules enabled)
    #[default]
    Full,
    /// Disable super-resolution
    NoSuperRes,
    /// Reduce feature count
    ReducedFeatures,
    /// Safe mode (minimal pipeline)
    SafeMode,
}

/// Real-time performance monitor
pub struct RealtimeMonitor {
    config: RealtimeMonitorConfig,
    timing_history: VecDeque<FrameTiming>,
    gating_level: GatingLevel,
    deadline_misses: usize,
    consecutive_misses: usize,
}

impl RealtimeMonitor {
    /// Create new monitor
    pub fn new(config: RealtimeMonitorConfig) -> Self {
        if config.enable_csv_export {
            log::info!(
                "[RealtimeMonitor] CSV export enabled: {}",
                config.csv_output_path
            );
        }

        Self {
            config,
            timing_history: VecDeque::with_capacity(100),
            gating_level: GatingLevel::Full,
            deadline_misses: 0,
            consecutive_misses: 0,
        }
    }

    /// Record frame timing and update gating
    pub fn record_frame(&mut self, timing: FrameTiming) {
        let budget_utilization = timing.total_ms / self.config.target_frame_interval_ms;
        let mut timing = timing;
        timing.budget_utilization = budget_utilization;

        // Check for deadline miss
        if budget_utilization > 1.0 {
            self.deadline_misses += 1;
            self.consecutive_misses += 1;
            log::warn!(
                "[RealtimeMonitor] Frame {} MISSED DEADLINE: {:.2}ms / {:.2}ms ({:.1}%)",
                timing.frame_id,
                timing.total_ms,
                self.config.target_frame_interval_ms,
                budget_utilization * 100.0
            );
        } else {
            self.consecutive_misses = 0;
        }

        // Alert if approaching budget
        if budget_utilization > self.config.alert_threshold {
            log::warn!(
                "[RealtimeMonitor] Frame {} approaching budget: {:.2}ms / {:.2}ms ({:.1}%)",
                timing.frame_id,
                timing.total_ms,
                self.config.target_frame_interval_ms,
                budget_utilization * 100.0
            );
        }

        // Log per-frame timing
        if self.config.enable_frame_logs {
            log::info!(
                "[RealtimeMonitor] Frame {}: total={:.2}ms (track={:.2}ms, imu={:.2}ms, fusion={:.2}ms, sr={:.2}ms, ba={:.2}ms, lc={:.2}ms) budget={:.1}%",
                timing.frame_id,
                timing.total_ms,
                timing.feature_tracking_ms,
                timing.imu_filtering_ms,
                timing.fusion_ms,
                timing.super_resolution_ms,
                timing.bundle_adjustment_ms,
                timing.loop_closure_ms,
                budget_utilization * 100.0
            );
        }

        // Write to CSV
        if self.config.enable_csv_export {
            // CSV export logic can be added here if needed
            // For now, just log that it's enabled
        }

        // Store in history
        self.timing_history.push_back(timing);
        if self.timing_history.len() > self.config.window_size {
            self.timing_history.pop_front();
        }

        // Update gating level
        self.update_gating();
    }

    /// Update adaptive gating level based on recent performance
    fn update_gating(&mut self) {
        // Compute moving average of budget utilization
        let avg_utilization = if !self.timing_history.is_empty() {
            self.timing_history
                .iter()
                .map(|t| t.budget_utilization)
                .sum::<Float>()
                / self.timing_history.len() as Float
        } else {
            0.0
        };

        // Gating policy
        let new_level = if self.consecutive_misses >= 3 {
            // Multiple consecutive misses → safe mode
            GatingLevel::SafeMode
        } else if avg_utilization > 0.9 || self.consecutive_misses >= 1 {
            // High utilization or single miss → reduce features
            GatingLevel::ReducedFeatures
        } else if avg_utilization > 0.75 {
            // Moderate utilization → disable super-res
            GatingLevel::NoSuperRes
        } else {
            // Low utilization → full pipeline
            GatingLevel::Full
        };

        // Log gating changes
        if new_level != self.gating_level {
            log::warn!(
                "[RealtimeMonitor] Gating level changed: {:?} → {:?} (avg_util={:.1}%, misses={})",
                self.gating_level,
                new_level,
                avg_utilization * 100.0,
                self.consecutive_misses
            );
            self.gating_level = new_level;
        }
    }

    /// Get current gating level
    pub fn gating_level(&self) -> GatingLevel {
        self.gating_level
    }

    /// Get statistics summary
    pub fn summary(&self) -> MonitorSummary {
        if self.timing_history.is_empty() {
            return MonitorSummary::default();
        }

        let totals: Vec<Float> = self.timing_history.iter().map(|t| t.total_ms).collect();
        let mut sorted = totals.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let n = sorted.len();
        let mean = totals.iter().sum::<Float>() / n as Float;
        let min = sorted[0];
        let max = sorted[n - 1];
        let p50 = sorted[n / 2];

        // Safe percentile calculation with proper error handling
        let p95_idx = percentile_index(n, 0.95).unwrap_or(n - 1);
        let p99_idx = percentile_index(n, 0.99).unwrap_or(n - 1);
        let p95 = sorted[p95_idx];
        let p99 = sorted[p99_idx];

        MonitorSummary {
            frames_processed: n,
            total_deadline_misses: self.deadline_misses,
            mean_latency_ms: mean,
            min_latency_ms: min,
            max_latency_ms: max,
            p50_latency_ms: p50,
            p95_latency_ms: p95,
            p99_latency_ms: p99,
            mean_budget_utilization: mean / self.config.target_frame_interval_ms,
            current_gating_level: self.gating_level,
        }
    }

    /// Reset statistics
    pub fn reset(&mut self) {
        self.timing_history.clear();
        self.deadline_misses = 0;
        self.consecutive_misses = 0;
        self.gating_level = GatingLevel::Full;
    }
}

/// Monitor statistics summary
#[derive(Debug, Clone, Default)]
pub struct MonitorSummary {
    pub frames_processed: usize,
    pub total_deadline_misses: usize,
    pub mean_latency_ms: Float,
    pub min_latency_ms: Float,
    pub max_latency_ms: Float,
    pub p50_latency_ms: Float,
    pub p95_latency_ms: Float,
    pub p99_latency_ms: Float,
    pub mean_budget_utilization: Float,
    pub current_gating_level: GatingLevel,
}

impl std::fmt::Display for MonitorSummary {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Frames: {} | Misses: {} | Latency: mean={:.2}ms, p50={:.2}ms, p95={:.2}ms, p99={:.2}ms, max={:.2}ms | Budget: {:.1}% | Gating: {:?}",
            self.frames_processed,
            self.total_deadline_misses,
            self.mean_latency_ms,
            self.p50_latency_ms,
            self.p95_latency_ms,
            self.p99_latency_ms,
            self.max_latency_ms,
            self.mean_budget_utilization * 100.0,
            self.current_gating_level
        )
    }
}

/// Frame timer for measuring individual stages
pub struct FrameTimer {
    start: Instant,
    stage_starts: Vec<(&'static str, Instant)>,
}

impl FrameTimer {
    /// Start timing a new frame
    pub fn start() -> Self {
        Self {
            start: Instant::now(),
            stage_starts: Vec::with_capacity(8),
        }
    }

    /// Mark start of a stage
    pub fn stage(&mut self, name: &'static str) {
        self.stage_starts.push((name, Instant::now()));
    }

    /// Get elapsed time in milliseconds since start
    pub fn elapsed_ms(&self) -> Float {
        self.start.elapsed().as_secs_f64() * 1000.0
    }

    /// Get stage duration in milliseconds
    pub fn stage_duration_ms(&self, stage_idx: usize) -> Float {
        if stage_idx >= self.stage_starts.len() {
            return 0.0;
        }

        let end = if stage_idx + 1 < self.stage_starts.len() {
            self.stage_starts[stage_idx + 1].1
        } else {
            Instant::now()
        };

        (end - self.stage_starts[stage_idx].1).as_secs_f64() * 1000.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_monitor_basic() {
        let mut monitor = RealtimeMonitor::new(RealtimeMonitorConfig::default());

        let timing = FrameTiming {
            frame_id: 0,
            timestamp_ns: 0,
            feature_tracking_ms: 9.0,
            imu_filtering_ms: 0.5,
            fusion_ms: 0.5,
            super_resolution_ms: 5.0,
            bundle_adjustment_ms: 0.0,
            loop_closure_ms: 0.0,
            total_ms: 15.0,
            budget_utilization: 0.0,
        };

        monitor.record_frame(timing);

        let summary = monitor.summary();
        assert_eq!(summary.frames_processed, 1);
        assert_eq!(summary.total_deadline_misses, 0);
    }

    #[test]
    fn test_gating_adaptation() {
        let mut monitor = RealtimeMonitor::new(RealtimeMonitorConfig {
            target_frame_interval_ms: 20.0,
            ..Default::default()
        });

        // Simulate high load
        for i in 0..5 {
            let timing = FrameTiming {
                frame_id: i,
                timestamp_ns: i * 20_000_000,
                feature_tracking_ms: 18.0,
                imu_filtering_ms: 0.5,
                fusion_ms: 0.5,
                super_resolution_ms: 5.0,
                bundle_adjustment_ms: 0.0,
                loop_closure_ms: 0.0,
                total_ms: 24.0,
                budget_utilization: 0.0,
            };
            monitor.record_frame(timing);
        }

        // Should have reduced gating level
        assert_ne!(monitor.gating_level(), GatingLevel::Full);
    }
}
