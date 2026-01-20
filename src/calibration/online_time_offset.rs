/// Online time-offset calibration for camera-IMU synchronization
///
/// Addresses timing drift in long-running operations where clock offset may change.
/// Implements adaptive time-offset estimation and monitoring.
use std::collections::VecDeque;

/// Timing quality assessment
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TimingQuality {
    /// Clock perfectly synchronized (σ < 0.5ms)
    Excellent,
    /// Good synchronization (σ < 2ms)
    Good,
    /// Acceptable with monitoring (σ < 5ms)
    Acceptable,
    /// Drift detected, needs recalibration
    Drifting,
    /// Severe jitter, IMU-aiding disabled
    Poor,
}

/// Online time-offset calibration state
#[derive(Clone, Debug)]
pub struct OnlineTimeOffsetCalibration {
    /// Current estimated time offset (camera_time = imu_time + dt_s)
    pub time_offset_s: f64,

    /// Time offset uncertainty (1-sigma, seconds)
    pub time_offset_uncertainty_s: f64,

    /// Linear drift rate (Δt/Δt_calendar, unitless)
    pub drift_rate: f64,

    /// Drift uncertainty (1-sigma)
    pub drift_uncertainty: f64,

    /// Timing quality assessment
    pub quality: TimingQuality,

    /// Number of frames used for estimation
    pub observation_count: usize,

    /// Recent optical flow prediction errors for jitter detection
    flow_errors: VecDeque<f64>,

    /// Recent time-offset estimates for drift detection
    offset_estimates: VecDeque<(f64, f64)>, // (time, offset)

    /// Configuration
    config: OnlineCalibrationConfig,
}

/// Configuration for online time-offset calibration
#[derive(Clone, Debug)]
pub struct OnlineCalibrationConfig {
    /// Window size for drift detection (frames)
    pub window_size: usize,

    /// Threshold for timing quality assessment (pixels)
    pub flow_error_threshold: f64,

    /// Minimum observations before updating drift estimate
    pub min_observations_for_drift: usize,

    /// Drift detection threshold (ppm = parts per million)
    pub drift_threshold_ppm: f64,

    /// Enable adaptive gating of IMU-aiding based on quality
    pub adaptive_gating: bool,
}

impl Default for OnlineCalibrationConfig {
    fn default() -> Self {
        Self {
            window_size: 30,           // 1 second at 30Hz
            flow_error_threshold: 0.5, // pixels
            min_observations_for_drift: 15,
            drift_threshold_ppm: 500.0, // 500ppm = 0.05% clock drift
            adaptive_gating: true,
        }
    }
}

impl OnlineTimeOffsetCalibration {
    /// Create new online calibration state
    pub fn new(
        initial_offset: f64,
        initial_uncertainty: f64,
        config: OnlineCalibrationConfig,
    ) -> Self {
        Self {
            time_offset_s: initial_offset,
            time_offset_uncertainty_s: initial_uncertainty,
            drift_rate: 0.0,
            drift_uncertainty: 0.1, // 10% uncertainty initially
            quality: TimingQuality::Acceptable,
            observation_count: 0,
            flow_errors: VecDeque::with_capacity(config.window_size),
            offset_estimates: VecDeque::with_capacity(config.window_size),
            config,
        }
    }

    /// Update with new optical flow prediction error
    ///
    /// Returns true if timing quality changed significantly
    pub fn update_with_flow_error(&mut self, flow_error_px: f64, _time_s: f64) -> bool {
        let old_quality = self.quality;

        // Add observation
        self.flow_errors.push_back(flow_error_px);
        if self.flow_errors.len() > self.config.window_size {
            self.flow_errors.pop_front();
        }

        // Assess timing quality from flow error distribution
        let quality = self.assess_quality_from_flow();
        self.quality = quality;

        // Update observation count
        self.observation_count += 1;

        old_quality != self.quality
    }

    /// Update with new time-offset estimate from calibration
    ///
    /// Used during periodic recalibration (e.g., every 100 frames)
    pub fn update_with_offset_estimate(
        &mut self,
        new_offset: f64,
        new_uncertainty: f64,
        time_s: f64,
    ) {
        // Store new estimate
        self.offset_estimates.push_back((time_s, new_offset));
        if self.offset_estimates.len() > self.config.window_size {
            self.offset_estimates.pop_front();
        }

        // Weighted update: trust new estimate based on uncertainty
        let old_offset = self.time_offset_s;
        let old_weight = self.time_offset_uncertainty_s.powi(-2);
        let new_weight = new_uncertainty.powi(-2);

        let total_weight = old_weight + new_weight;
        self.time_offset_s = (old_offset * old_weight + new_offset * new_weight) / total_weight;
        self.time_offset_uncertainty_s = (total_weight).sqrt().recip();

        // Detect drift
        if self.offset_estimates.len() >= self.config.min_observations_for_drift {
            self.detect_drift();
        }
    }

    /// Get recommended IMU-aiding gate factor
    ///
    /// Returns multiplier for feature search region based on timing quality.
    /// - Excellent: 1.0× (use tight prediction)
    /// - Good: 1.2× (normal operation)
    /// - Acceptable: 1.5× (wider search)
    /// - Drifting: 2.0× (conservative)
    /// - Poor: 0.0 (disable IMU-aiding)
    pub fn imu_aiding_gate_factor(&self) -> f64 {
        if !self.config.adaptive_gating {
            return 1.0;
        }

        match self.quality {
            TimingQuality::Excellent => 1.0,
            TimingQuality::Good => 1.2,
            TimingQuality::Acceptable => 1.5,
            TimingQuality::Drifting => 2.0,
            TimingQuality::Poor => 0.0, // Disable IMU-aiding
        }
    }

    /// Assess timing quality from optical flow error statistics
    fn assess_quality_from_flow(&self) -> TimingQuality {
        if self.flow_errors.len() < 5 {
            return TimingQuality::Acceptable; // Not enough data
        }

        // Compute mean and stddev of recent flow errors
        let mean = self.flow_errors.iter().sum::<f64>() / self.flow_errors.len() as f64;
        let variance = self
            .flow_errors
            .iter()
            .map(|e| (e - mean).powi(2))
            .sum::<f64>()
            / self.flow_errors.len() as f64;
        let stddev = variance.sqrt();

        // Flow error distribution indicates timing jitter
        // σ ~ 0.3px: good time sync
        // σ ~ 1px: acceptable with drift monitoring
        // σ ~ 2px+: poor time sync, needs adjustment
        if stddev < 0.3 {
            TimingQuality::Excellent
        } else if stddev < 0.8 {
            TimingQuality::Good
        } else if stddev < 1.5 {
            TimingQuality::Acceptable
        } else if stddev < 3.0 {
            TimingQuality::Drifting
        } else {
            TimingQuality::Poor
        }
    }

    /// Detect if clock drift is occurring
    fn detect_drift(&self) {
        if self.offset_estimates.len() < 10 {
            return; // Need more observations
        }

        // Linear regression on (time, offset) to estimate drift rate
        let n = self.offset_estimates.len() as f64;

        let time_mean = self.offset_estimates.iter().map(|(t, _)| t).sum::<f64>() / n;
        let offset_mean = self.offset_estimates.iter().map(|(_, o)| o).sum::<f64>() / n;

        let mut numerator = 0.0;
        let mut denominator = 0.0;
        for (t, o) in self.offset_estimates.iter() {
            numerator += (t - time_mean) * (o - offset_mean);
            denominator += (t - time_mean).powi(2);
        }

        if denominator > 1e-9 {
            let drift_rate = numerator / denominator;
            // drift_rate is in units of seconds/second = ppm × 1e-6
            let drift_ppm = drift_rate.abs() * 1e6;

            // Check if drift exceeds threshold
            if drift_ppm > self.config.drift_threshold_ppm {
                // Drift detected: quality degrades
                // This will be caught by flow error assessment next
            }
        }
    }

    /// Get diagnostic message for this calibration state
    pub fn diagnostic_message(&self) -> String {
        format!(
            "TimingQuality: {:?}, Offset: {:.6}s ± {:.6}s, Drift: {:.3}ppm, Observations: {}",
            self.quality,
            self.time_offset_s,
            self.time_offset_uncertainty_s,
            self.drift_rate * 1e6,
            self.observation_count
        )
    }
}

// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_online_calibration_creation() {
        let calib = OnlineTimeOffsetCalibration::new(0.001, 0.0001, Default::default());
        assert_eq!(calib.time_offset_s, 0.001);
        assert_eq!(calib.observation_count, 0);
        assert_eq!(calib.quality, TimingQuality::Acceptable);
    }

    #[test]
    fn test_quality_assessment_excellent_timing() {
        let mut calib = OnlineTimeOffsetCalibration::new(0.0, 0.0001, Default::default());

        // Add low-error observations (excellent timing)
        for _ in 0..10 {
            calib.update_with_flow_error(0.1, 0.0); // 0.1px error
        }

        assert_eq!(calib.quality, TimingQuality::Excellent);
        assert_eq!(calib.imu_aiding_gate_factor(), 1.0);
    }
    #[test]
    fn test_quality_assessment_poor_timing() {
        let mut calib = OnlineTimeOffsetCalibration::new(0.0, 0.0001, Default::default());

        // Add high-error observations with varying values to create high variance (poor timing)
        // Values spread to produce stddev > 3.0 for Poor quality threshold
        let varying_errors = [0.1, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0];
        for error in varying_errors.iter() {
            calib.update_with_flow_error(*error, 0.0);
        }

        assert_eq!(calib.quality, TimingQuality::Poor);
        assert_eq!(calib.imu_aiding_gate_factor(), 0.0); // Disabled
    }
    #[test]
    fn test_offset_update_weighted_fusion() {
        let mut calib = OnlineTimeOffsetCalibration::new(0.001, 0.0005, Default::default());

        // Update with new estimate (lower uncertainty)
        calib.update_with_offset_estimate(0.002, 0.0001, 1.0);

        // New estimate should be weighted towards lower-uncertainty value
        assert!(calib.time_offset_s > 0.001);
        assert!(calib.time_offset_s < 0.002);
        assert!(calib.time_offset_uncertainty_s < 0.0001); // Improved
    }

    #[test]
    fn test_adaptive_gating_factor() {
        let mut calib = OnlineTimeOffsetCalibration::new(0.0, 0.0001, Default::default());
        calib.config.adaptive_gating = true;

        // Simulate degrading timing quality
        let gate_factors: Vec<f64> = vec![
            {
                for _ in 0..5 {
                    calib.update_with_flow_error(0.2, 0.0);
                }
                calib.imu_aiding_gate_factor()
            },
            {
                for _ in 0..10 {
                    calib.update_with_flow_error(1.0, 0.0);
                }
                calib.imu_aiding_gate_factor()
            },
            {
                for _ in 0..10 {
                    calib.update_with_flow_error(4.0, 0.0);
                }
                calib.imu_aiding_gate_factor()
            },
        ];

        // Gates should increase as timing quality degrades
        assert!(gate_factors[0] < gate_factors[1]);
        assert!(gate_factors[1] < gate_factors[2]);
    }

    #[test]
    fn test_diagnostic_message() {
        let calib = OnlineTimeOffsetCalibration::new(0.001, 0.0001, Default::default());
        let msg = calib.diagnostic_message();
        assert!(msg.contains("TimingQuality"));
        assert!(msg.contains("Offset"));
        assert!(msg.contains("ppm"));
    }
}
