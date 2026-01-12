//! Adaptive frame skipping for real-time VIO
//!
//! This module implements intelligent frame skipping that maintains real-time performance
//! while maximizing visual information utilization.
use std::time::{Duration, Instant};

/// Frame skipping strategy for maintaining real-time constraints
pub struct AdaptiveFrameSkipper {
    /// Target frame processing budget in milliseconds
    target_frame_time_ms: f64,
    
    /// Recent processing times (ring buffer)
    recent_times: Vec<Duration>,
    recent_idx: usize,
    
    /// Current skip counter
    skip_count: u32,
    
    /// Maximum frames to skip before forcing a process
    max_skip: u32,
    
    /// Minimum motion threshold (pixels) to force processing despite time budget
    motion_threshold: f32,
    
    /// Last processed timestamp
    last_processed_time: Option<Instant>,
}

impl AdaptiveFrameSkipper {
    /// Create a new adaptive frame skipper
    ///
    /// # Arguments
    /// * `target_fps` - Target frames per second (e.g., 30.0 for 30 FPS)
    /// * `max_skip` - Maximum consecutive frames to skip
    /// * `motion_threshold` - Minimum motion (pixels) to force processing
    pub fn new(target_fps: f64, max_skip: u32, motion_threshold: f32) -> Self {
        Self {
            target_frame_time_ms: 1000.0 / target_fps,
            recent_times: vec![Duration::from_millis(0); 10],
            recent_idx: 0,
            skip_count: 0,
            max_skip,
            motion_threshold,
            last_processed_time: None,
        }
    }

    /// Check if current frame should be processed
    ///
    /// # Arguments
    /// * `estimated_motion` - Estimated motion in pixels since last frame
    ///
    /// # Returns
    /// `true` if frame should be processed, `false` to skip
    pub fn should_process(&mut self, estimated_motion: Option<f32>) -> bool {
        // Force processing if max skip reached
        if self.skip_count >= self.max_skip {
            self.skip_count = 0;
            return true;
        }

        // Force processing if significant motion detected
        if let Some(motion) = estimated_motion {
            if motion > self.motion_threshold {
                self.skip_count = 0;
                return true;
            }
        }

        // Check if we have processing time budget
        let avg_time = self.average_processing_time();
        let has_budget = avg_time.as_secs_f64() * 1000.0 < self.target_frame_time_ms * 0.8;

        if has_budget {
            self.skip_count = 0;
            true
        } else {
            self.skip_count += 1;
            false
        }
    }

    /// Record processing time for a frame
    pub fn record_processing_time(&mut self, duration: Duration) {
        self.recent_times[self.recent_idx] = duration;
        self.recent_idx = (self.recent_idx + 1) % self.recent_times.len();
        self.last_processed_time = Some(Instant::now());
    }
    
    /// Record frame processing time (alias for record_processing_time)
    pub fn record_frame_time(&mut self, duration: Duration) {
        self.record_processing_time(duration);
    }

    /// Get average processing time from recent frames
    fn average_processing_time(&self) -> Duration {
        let sum: Duration = self.recent_times.iter().sum();
        sum / self.recent_times.len() as u32
    }

    /// Get current skip ratio (0.0 = no skipping, 0.5 = skip half)
    pub fn skip_ratio(&self) -> f32 {
        if self.max_skip == 0 {
            0.0
        } else {
            self.skip_count as f32 / self.max_skip as f32
        }
    }

    /// Get statistics
    pub fn stats(&self) -> FrameSkipStats {
        let avg = self.average_processing_time();
        FrameSkipStats {
            avg_processing_ms: avg.as_secs_f64() * 1000.0,
            target_frame_time_ms: self.target_frame_time_ms,
            current_skip_count: self.skip_count,
            max_skip: self.max_skip,
        }
    }
}

/// Statistics about frame skipping performance
#[derive(Debug, Clone)]
pub struct FrameSkipStats {
    pub avg_processing_ms: f64,
    pub target_frame_time_ms: f64,
    pub current_skip_count: u32,
    pub max_skip: u32,
}

impl FrameSkipStats {
    /// Check if system is running in real-time
    pub fn is_realtime(&self) -> bool {
        self.avg_processing_ms < self.target_frame_time_ms
    }

    /// Get CPU headroom percentage (100% = not using full budget)
    pub fn headroom_percent(&self) -> f64 {
        100.0 * (1.0 - self.avg_processing_ms / self.target_frame_time_ms)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_skip_on_overload() {
        let mut skipper = AdaptiveFrameSkipper::new(30.0, 2, 10.0);
        
        // Simulate sustained slow processing (fill buffer with slow times)
        for _ in 0..10 {
            skipper.record_processing_time(Duration::from_millis(50)); // Over budget
        }
        
        // Should skip next frame
        assert!(!skipper.should_process(None));
        assert_eq!(skipper.skip_count, 1);
    }

    #[test]
    fn test_force_on_motion() {
        let mut skipper = AdaptiveFrameSkipper::new(30.0, 2, 10.0);
        
        // Fill buffer with slow processing times
        for _ in 0..10 {
            skipper.record_processing_time(Duration::from_millis(50));
        }
        
        // High motion should force processing despite overload
        assert!(skipper.should_process(Some(15.0)));
        assert_eq!(skipper.skip_count, 0);
    }

    #[test]
    fn test_max_skip_limit() {
        let mut skipper = AdaptiveFrameSkipper::new(30.0, 2, 10.0);
        
        // Simulate sustained overload (fill buffer)
        for _ in 0..10 {
            skipper.record_processing_time(Duration::from_millis(50));
        }
        
        // Skip first
        assert!(!skipper.should_process(None));
        // Skip second
        assert!(!skipper.should_process(None));
        // Force process on third (max_skip=2)
        assert!(skipper.should_process(None));
    }
}
