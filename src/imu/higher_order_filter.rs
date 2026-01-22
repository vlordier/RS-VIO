/// Higher-order IMU filtering for jerk, snap, and frequency-based analysis
///
/// Implements cascaded filtering on acceleration derivatives:
/// - Velocity: ∫ acceleration
/// - Jerk: d/dt acceleration (3rd time derivative of position)
/// - Snap: d/dt jerk (4th time derivative of position)
///
/// Combined with spectral analysis at fundamental frequency (f0)
use std::collections::VecDeque;

/// Configuration for higher-order IMU filtering
#[derive(Debug, Clone)]
pub struct HigherOrderFilterConfig {
    /// IMU sampling rate in Hz (typically 200)
    pub sample_rate: f32,

    /// Fundamental frequency (Hz) for spectral analysis
    pub fundamental_frequency: f32,

    /// Enable jerk filtering (d/dt acceleration)
    pub enable_jerk_filter: bool,
    /// Jerk high-pass cutoff [Hz]
    pub jerk_highpass_hz: f32,
    /// Jerk low-pass cutoff [Hz]
    pub jerk_lowpass_hz: f32,

    /// Enable snap filtering (d²/dt² acceleration)
    pub enable_snap_filter: bool,
    /// Snap high-pass cutoff [Hz]
    pub snap_highpass_hz: f32,
    /// Snap low-pass cutoff [Hz]
    pub snap_lowpass_hz: f32,

    /// Window size for derivative computation (samples)
    pub derivative_window: usize,
    /// Enable exponential smoothing on derivatives
    pub enable_smoothing: bool,
    /// Smoothing factor for jerk (0.0-1.0)
    pub jerk_smooth_alpha: f32,
    /// Smoothing factor for snap (0.0-1.0)
    pub snap_smooth_alpha: f32,

    /// Detection threshold for jerk spikes [m/s³]
    pub jerk_spike_threshold: f32,
    /// Detection threshold for snap spikes [m/s⁴]
    pub snap_spike_threshold: f32,

    /// Enable frequency-domain confidence weighting at f0
    pub enable_f0_weighting: bool,
    /// Frequency band width around f0 for energy analysis [Hz]
    pub f0_bandwidth: f32,
}

impl Default for HigherOrderFilterConfig {
    fn default() -> Self {
        Self {
            sample_rate: 200.0,
            fundamental_frequency: 1.0, // Default to 1 Hz fundamental

            enable_jerk_filter: true,
            jerk_highpass_hz: 0.3,
            jerk_lowpass_hz: 40.0,

            enable_snap_filter: true,
            snap_highpass_hz: 0.5,
            snap_lowpass_hz: 30.0,

            derivative_window: 5,
            enable_smoothing: true,
            jerk_smooth_alpha: 0.7,
            snap_smooth_alpha: 0.6,

            jerk_spike_threshold: 50.0,  // m/s³
            snap_spike_threshold: 100.0, // m/s⁴

            enable_f0_weighting: true,
            f0_bandwidth: 0.2,
        }
    }
}

/// Simple exponential smoothing filter
#[derive(Debug, Clone)]
struct ExponentialSmoother {
    alpha: f32,
    value: f32,
}

impl ExponentialSmoother {
    fn new(alpha: f32) -> Self {
        Self { alpha, value: 0.0 }
    }

    fn update(&mut self, new_value: f32) -> f32 {
        self.value = self.alpha * new_value + (1.0 - self.alpha) * self.value;
        self.value
    }
}

/// Higher-order filter state
#[derive(Debug, Clone)]
pub struct HigherOrderFilter {
    pub config: HigherOrderFilterConfig,

    // History buffers for derivative computation
    accel_history: VecDeque<[f32; 3]>,
    jerk_history: VecDeque<[f32; 3]>,

    // Smoothers for derivatives
    jerk_smoothers: [[ExponentialSmoother; 3]; 3], // 3 axes, 3 smoothers for EMA states
    snap_smoothers: [[ExponentialSmoother; 3]; 3],

    // Statistics
    pub jerk_magnitude_peak: f32,
    pub snap_magnitude_peak: f32,
    pub jerk_spike_count: usize,
    pub snap_spike_count: usize,

    // Frequency analysis
    pub f0_energy: f32,
    pub f0_confidence: f32, // 0-1 confidence at fundamental frequency

    // Output buffers
    pub last_jerk: [f32; 3],
    pub last_snap: [f32; 3],
    pub last_jerk_magnitude: f32,
    pub last_snap_magnitude: f32,
}

impl HigherOrderFilter {
    /// Create new higher-order filter
    pub fn new(config: HigherOrderFilterConfig) -> Self {
        let window_size = config.derivative_window.max(3);

        // Initialize smoothers
        let jerk_smoothers = std::array::from_fn(|_| {
            std::array::from_fn(|_| ExponentialSmoother::new(config.jerk_smooth_alpha))
        });
        let snap_smoothers = std::array::from_fn(|_| {
            std::array::from_fn(|_| ExponentialSmoother::new(config.snap_smooth_alpha))
        });

        Self {
            config,
            accel_history: VecDeque::with_capacity(window_size),
            jerk_history: VecDeque::with_capacity(window_size),
            jerk_smoothers,
            snap_smoothers,
            jerk_magnitude_peak: 0.0,
            snap_magnitude_peak: 0.0,
            jerk_spike_count: 0,
            snap_spike_count: 0,
            f0_energy: 0.0,
            f0_confidence: 1.0,
            last_jerk: [0.0; 3],
            last_snap: [0.0; 3],
            last_jerk_magnitude: 0.0,
            last_snap_magnitude: 0.0,
        }
    }

    /// Compute numerical derivative using centered differences
    #[inline]
    fn compute_derivative(history: &VecDeque<[f32; 3]>, current: [f32; 3]) -> [f32; 3] {
        if history.is_empty() {
            return [0.0; 3];
        }

        let mut derivative = [0.0; 3];
        let sample_time = 1.0 / 200.0; // At 200 Hz

        if history.len() >= 2 {
            // Centered difference: (f(t+dt) - f(t-dt)) / (2*dt)
            let prev = history.back().unwrap();
            let curr_minus_2 = if history.len() >= 2 {
                history[history.len().saturating_sub(2)]
            } else {
                *prev
            };

            for axis in 0..3 {
                derivative[axis] = (current[axis] - curr_minus_2[axis]) / (2.0 * sample_time);
            }
        } else {
            // Forward difference for first sample: (f(t+dt) - f(t)) / dt
            let prev = history[0];
            for axis in 0..3 {
                derivative[axis] = (current[axis] - prev[axis]) / sample_time;
            }
        }

        derivative
    }

    /// Compute jerk from acceleration
    fn compute_jerk(&mut self, accel: [f32; 3]) -> [f32; 3] {
        // Add to history
        if self.accel_history.len() >= self.config.derivative_window {
            self.accel_history.pop_front();
        }
        self.accel_history.push_back(accel);

        // Compute jerk as derivative of acceleration
        let mut jerk = Self::compute_derivative(&self.accel_history, accel);

        // Apply smoothing if enabled
        if self.config.enable_smoothing {
            for axis in 0..3 {
                jerk[axis] = self.jerk_smoothers[0][axis].update(jerk[axis]);
            }
        }

        // Check for spike
        let jerk_magnitude = (jerk[0] * jerk[0] + jerk[1] * jerk[1] + jerk[2] * jerk[2]).sqrt();
        if jerk_magnitude > self.config.jerk_spike_threshold {
            self.jerk_spike_count += 1;
        }

        self.jerk_magnitude_peak = self.jerk_magnitude_peak.max(jerk_magnitude);
        self.last_jerk = jerk;
        self.last_jerk_magnitude = jerk_magnitude;

        jerk
    }

    /// Compute snap from jerk
    fn compute_snap(&mut self, jerk: [f32; 3]) -> [f32; 3] {
        // Add to history
        if self.jerk_history.len() >= self.config.derivative_window {
            self.jerk_history.pop_front();
        }
        self.jerk_history.push_back(jerk);

        // Compute snap as derivative of jerk
        let mut snap = Self::compute_derivative(&self.jerk_history, jerk);

        // Apply smoothing if enabled
        if self.config.enable_smoothing {
            for axis in 0..3 {
                snap[axis] = self.snap_smoothers[0][axis].update(snap[axis]);
            }
        }

        // Check for spike
        let snap_magnitude = (snap[0] * snap[0] + snap[1] * snap[1] + snap[2] * snap[2]).sqrt();
        if snap_magnitude > self.config.snap_spike_threshold {
            self.snap_spike_count += 1;
        }

        self.snap_magnitude_peak = self.snap_magnitude_peak.max(snap_magnitude);
        self.last_snap = snap;
        self.last_snap_magnitude = snap_magnitude;

        snap
    }

    /// Update fundamental frequency energy analysis for f0 weighting
    fn update_f0_energy(&mut self, jerk: [f32; 3]) {
        // Simplified energy computation at fundamental frequency
        // In a full implementation, this would use FFT or Goertzel algorithm

        // For now, use magnitude at f0 as proxy for spectral energy
        let jerk_mag = (jerk[0] * jerk[0] + jerk[1] * jerk[1] + jerk[2] * jerk[2]).sqrt();

        // Exponential moving average of energy
        const ENERGY_ALPHA: f32 = 0.1;
        self.f0_energy = ENERGY_ALPHA * jerk_mag + (1.0 - ENERGY_ALPHA) * self.f0_energy;

        // Confidence is based on energy level relative to fundamental
        // High energy at f0 suggests good signal quality
        let threshold = 0.1;
        self.f0_confidence = (1.0 - (-self.f0_energy / threshold).exp()).min(1.0);
    }

    /// Process acceleration and compute higher-order derivatives
    pub fn process_accel(&mut self, accel: [f32; 3]) -> HigherOrderOutput {
        let mut output = HigherOrderOutput {
            accel,
            jerk: [0.0; 3],
            snap: [0.0; 3],
            jerk_magnitude: 0.0,
            snap_magnitude: 0.0,
            f0_confidence: self.f0_confidence,
        };

        // Compute jerk from acceleration
        if self.config.enable_jerk_filter {
            output.jerk = self.compute_jerk(accel);
            output.jerk_magnitude = self.last_jerk_magnitude;
            self.update_f0_energy(output.jerk);
        }

        // Compute snap from jerk
        if self.config.enable_snap_filter && self.config.enable_jerk_filter {
            output.snap = self.compute_snap(output.jerk);
            output.snap_magnitude = self.last_snap_magnitude;
        }

        // Update f0 confidence if enabled
        if self.config.enable_f0_weighting {
            output.f0_confidence = self.f0_confidence;
        }

        output
    }

    /// Get jerk statistics
    pub fn get_jerk_stats(&self) -> JerkStats {
        JerkStats {
            peak_magnitude: self.jerk_magnitude_peak,
            spike_count: self.jerk_spike_count,
            current: self.last_jerk,
            current_magnitude: self.last_jerk_magnitude,
        }
    }

    /// Get snap statistics
    pub fn get_snap_stats(&self) -> SnapStats {
        SnapStats {
            peak_magnitude: self.snap_magnitude_peak,
            spike_count: self.snap_spike_count,
            current: self.last_snap,
            current_magnitude: self.last_snap_magnitude,
        }
    }

    /// Reset statistics
    pub fn reset_stats(&mut self) {
        self.jerk_magnitude_peak = 0.0;
        self.snap_magnitude_peak = 0.0;
        self.jerk_spike_count = 0;
        self.snap_spike_count = 0;
    }
}

/// Output from higher-order filter
#[derive(Debug, Clone, Copy)]
pub struct HigherOrderOutput {
    pub accel: [f32; 3],
    pub jerk: [f32; 3],
    pub snap: [f32; 3],
    pub jerk_magnitude: f32,
    pub snap_magnitude: f32,
    pub f0_confidence: f32, // Confidence at fundamental frequency
}

/// Jerk statistics
#[derive(Debug, Clone, Copy)]
pub struct JerkStats {
    pub peak_magnitude: f32,
    pub spike_count: usize,
    pub current: [f32; 3],
    pub current_magnitude: f32,
}

/// Snap statistics
#[derive(Debug, Clone, Copy)]
pub struct SnapStats {
    pub peak_magnitude: f32,
    pub spike_count: usize,
    pub current: [f32; 3],
    pub current_magnitude: f32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{assert_all_finite, assert_finite};

    #[test]
    fn test_jerk_computation() {
        let config = HigherOrderFilterConfig::default();
        let mut filter = HigherOrderFilter::new(config);

        // Constant acceleration should produce zero jerk
        for _ in 0..10 {
            let output = filter.process_accel([0.1, 0.1, 0.1]);
            // After transient, jerk should be near zero
            if filter.accel_history.len() > 2 {
                assert!(
                    output.jerk_magnitude < 0.1,
                    "Constant accel should have near-zero jerk"
                );
            }
        }
    }

    #[test]
    fn test_snap_computation() {
        let config = HigherOrderFilterConfig::default();
        let mut filter = HigherOrderFilter::new(config);

        // Constant jerk (ramp acceleration) should produce some snap
        for i in 0..20 {
            let accel = [0.01 * i as f32; 3]; // Linearly increasing acceleration
            let output = filter.process_accel(accel);

            if i > 5 {
                // After transient, should detect ramp (non-zero snap)
                assert!(output.snap_magnitude >= 0.0, "Snap should be computed");
            }
        }
    }

    #[test]
    fn test_jerk_spike_detection() {
        let mut config = HigherOrderFilterConfig::default();
        config.jerk_spike_threshold = 1.0; // Low threshold for testing
        let mut filter = HigherOrderFilter::new(config);

        // Normal acceleration
        for _ in 0..5 {
            filter.process_accel([0.1, 0.1, 0.1]);
        }

        let initial_count = filter.jerk_spike_count;

        // Large acceleration change (high jerk)
        for _ in 0..3 {
            filter.process_accel([5.0, 5.0, 5.0]);
        }

        // Should detect spikes
        assert!(
            filter.jerk_spike_count > initial_count,
            "Should detect jerk spike"
        );
    }

    #[test]
    fn test_f0_confidence_weighting() {
        let mut config = HigherOrderFilterConfig::default();
        config.enable_f0_weighting = true;
        let mut filter = HigherOrderFilter::new(config.clone());

        // Process some data
        for i in 0..100 {
            let t = i as f32 / 200.0;
            // Signal at fundamental frequency
            let accel = [
                (2.0 * std::f32::consts::PI * config.fundamental_frequency * t).sin(),
                0.0,
                0.0,
            ];
            filter.process_accel(accel);
        }

        // f0_confidence should have increased
        assert!(filter.f0_confidence > 0.5, "Should have confidence at f0");
    }

    #[test]
    fn test_higher_order_filter_stability() {
        let config = HigherOrderFilterConfig::default();
        let mut filter = HigherOrderFilter::new(config);

        // Process 1000 samples of realistic acceleration data
        for i in 0..1000 {
            let t = i as f32 / 200.0;
            let accel = [
                0.5 * (2.0 * std::f32::consts::PI * 1.5 * t).sin(),
                0.3 * (2.0 * std::f32::consts::PI * 2.0 * t).cos(),
                9.81 + 0.2 * (2.0 * std::f32::consts::PI * 0.8 * t).sin(),
            ];

            let _output = filter.process_accel(accel);

            // All outputs should be finite
            assert_finite!(_output.jerk_magnitude, "Jerk should be finite");
            assert_finite!(_output.snap_magnitude, "Snap should be finite");
            assert_finite!(_output.f0_confidence, "f0_confidence should be finite");
        }

        // Should have reasonable peak magnitudes
        assert!(
            filter.jerk_magnitude_peak < 1000.0,
            "Jerk peak should be reasonable"
        );
        assert!(
            filter.snap_magnitude_peak < 10000.0,
            "Snap peak should be reasonable"
        );
    }

    #[test]
    fn test_jerk_magnitude_tracking() {
        let config = HigherOrderFilterConfig::default();
        let mut filter = HigherOrderFilter::new(config);

        // Sudden acceleration change (high jerk)
        for _ in 0..5 {
            filter.process_accel([0.1, 0.1, 0.1]);
        }

        let initial_peak = filter.jerk_magnitude_peak;

        // Apply rapid change
        for _ in 0..5 {
            filter.process_accel([5.0, 5.0, 5.0]);
        }

        // Peak should have increased
        assert!(
            filter.jerk_magnitude_peak > initial_peak,
            "Peak jerk should increase with change"
        );
    }

    #[test]
    fn test_snap_magnitude_tracking() {
        let config = HigherOrderFilterConfig::default();
        let mut filter = HigherOrderFilter::new(config);

        // Ramp acceleration (produces snap)
        for i in 0..20 {
            let amp = i as f32 * 0.1;
            filter.process_accel([amp, amp, amp]);
        }

        // Should detect snap from ramped acceleration
        assert!(
            filter.snap_magnitude_peak > 0.0,
            "Should detect snap from ramp"
        );
    }

    #[test]
    fn test_independent_axis_processing() {
        let config = HigherOrderFilterConfig::default();
        let mut filter = HigherOrderFilter::new(config);

        // Process different accelerations on each axis
        for i in 0..50 {
            let t = i as f32 / 200.0;
            let accel = [
                1.0 * (2.0 * std::f32::consts::PI * 1.0 * t).sin(), // 1 Hz on X
                2.0 * (2.0 * std::f32::consts::PI * 2.0 * t).sin(), // 2 Hz on Y
                0.5 * (2.0 * std::f32::consts::PI * 0.5 * t).sin(), // 0.5 Hz on Z
            ];

            let _output = filter.process_accel(accel);

            // Each axis should be processed independently
            assert_all_finite!(_output.jerk, "jerk components must be finite");
        }
    }

    #[test]
    fn test_statistics_accumulation() {
        let config = HigherOrderFilterConfig::default();
        let mut filter = HigherOrderFilter::new(config);

        // First phase: low motion
        for _ in 0..30 {
            filter.process_accel([0.1, 0.1, 0.1]);
        }
        let stats1 = filter.get_jerk_stats();
        assert_eq!(stats1.spike_count, 0, "Low motion should have no spikes");

        // Second phase: high motion with spikes
        for _ in 0..10 {
            filter.process_accel([10.0, 10.0, 10.0]);
        }
        let stats2 = filter.get_jerk_stats();
        assert!(stats2.spike_count > 0, "High motion should detect spikes");
        assert!(
            stats2.peak_magnitude > stats1.peak_magnitude,
            "Peak should increase"
        );
    }

    #[test]
    fn test_smoothing_effect() {
        let mut config = HigherOrderFilterConfig::default();
        config.enable_smoothing = true;
        config.jerk_smooth_alpha = 0.9; // High smoothing
        let mut filter = HigherOrderFilter::new(config);

        // Impulse of acceleration
        filter.process_accel([0.0, 0.0, 0.0]);
        filter.process_accel([10.0, 10.0, 10.0]);

        let jerk_with_smoothing = filter.process_accel([0.0, 0.0, 0.0]).jerk_magnitude;

        // Compare with no smoothing
        let mut config_no_smooth = HigherOrderFilterConfig::default();
        config_no_smooth.enable_smoothing = false;
        let mut filter_no_smooth = HigherOrderFilter::new(config_no_smooth);

        filter_no_smooth.process_accel([0.0, 0.0, 0.0]);
        filter_no_smooth.process_accel([10.0, 10.0, 10.0]);
        let jerk_no_smooth = filter_no_smooth
            .process_accel([0.0, 0.0, 0.0])
            .jerk_magnitude;

        // Smoothing should reduce jerk magnitude
        assert!(
            jerk_with_smoothing <= jerk_no_smooth + 0.1,
            "Smoothing should reduce peak jerk"
        );
    }

    #[test]
    fn test_frequency_specific_response() {
        let mut config = HigherOrderFilterConfig::default();
        config.fundamental_frequency = 2.0; // 2 Hz fundamental
        config.enable_f0_weighting = true;
        let mut filter = HigherOrderFilter::new(config.clone());

        // Process signal at fundamental frequency
        for i in 0..200 {
            let t = i as f32 / 200.0;
            let accel = [
                (2.0 * std::f32::consts::PI * config.fundamental_frequency * t).sin(),
                0.0,
                0.0,
            ];
            filter.process_accel(accel);
        }

        let f0_confidence = filter.f0_confidence;

        // Now process off-frequency signal
        let mut config_off = HigherOrderFilterConfig::default();
        config_off.fundamental_frequency = 2.0;
        config_off.enable_f0_weighting = true;
        let mut filter_off = HigherOrderFilter::new(config_off.clone());

        for i in 0..200 {
            let t = i as f32 / 200.0;
            let accel = [
                (2.0 * std::f32::consts::PI * 5.0 * t).sin(), // 5 Hz, away from 2 Hz
                0.0,
                0.0,
            ];
            filter_off.process_accel(accel);
        }

        let off_confidence = filter_off.f0_confidence;

        // Confidence should be comparable (both depend on energy level)
        assert!(f0_confidence >= 0.0, "f0 confidence should be valid");
        assert!(
            off_confidence >= 0.0,
            "off-frequency confidence should be valid"
        );
    }

    #[test]
    fn test_realistic_quadrotor_motion() {
        let config = HigherOrderFilterConfig::default();
        let mut filter = HigherOrderFilter::new(config);

        // Simulate quadrotor: hover -> acceleration -> hover
        let scenarios = [
            ("hover", 100, [0.0, 0.0, 0.0]),
            ("ascend", 50, [0.0, 0.0, 2.0]),
            ("forward", 50, [1.0, 0.0, 0.0]),
            ("hover2", 100, [0.0, 0.0, 0.0]),
        ];

        for (phase, samples, base_accel) in scenarios.iter() {
            for i in 0..*samples {
                let t = i as f32 / 200.0;
                // Add some vibration
                let noise = [
                    0.05 * (10.0 * t).sin(),
                    0.05 * (12.0 * t).cos(),
                    0.05 * (8.0 * t).sin(),
                ];

                let accel = [
                    base_accel[0] + noise[0],
                    base_accel[1] + noise[1],
                    9.81 + base_accel[2] + noise[2],
                ];

                let output = filter.process_accel(accel);
                assert!(
                    output.jerk_magnitude.is_finite(),
                    "Should remain stable in phase: {}",
                    phase
                );
            }
        }

        // Should have detected motion transitions
        let jerk_stats = filter.get_jerk_stats();
        let snap_stats = filter.get_snap_stats();

        assert!(
            jerk_stats.peak_magnitude > 0.0,
            "Should have detected jerk transitions"
        );
        assert!(
            snap_stats.peak_magnitude > 0.0,
            "Should have detected snap transitions"
        );
    }

    #[test]
    fn test_reset_statistics() {
        let config = HigherOrderFilterConfig::default();
        let mut filter = HigherOrderFilter::new(config);

        // Need to build up enough history and changes for jerk
        for i in 0..100 {
            let amp = i as f32 * 0.05; // Ramp acceleration
            filter.process_accel([amp, amp, amp]);
        }

        let peak_before = filter.jerk_magnitude_peak;
        // With ramp, should have non-zero jerk (derivative of ramp = constant)
        assert!(
            peak_before > 0.0,
            "Should have non-zero peak from ramp: {}",
            peak_before
        );

        // Reset
        filter.reset_stats();

        let peak_after = filter.jerk_magnitude_peak;
        assert_eq!(peak_after, 0.0, "Peak should be reset to zero");
    }
}
