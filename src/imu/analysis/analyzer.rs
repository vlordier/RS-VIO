//! Main IMU signal analyzer
//!
//! Combines all analysis components into a single analyzer struct that processes
//! IMU measurements and produces harmonic decompositions.

use super::bias;
use super::config::{
    HarmonicDecomposition, MotorState, SignalQuality, DEFAULT_MOTOR_THRESHOLD, DEFAULT_NOISE_FLOOR,
    MIN_SAMPLES_FOR_DETECTION,
};
use super::harmonic;
use super::motor_detection;
use super::quality;
use super::spectral;

use crate::datasets::ImuData;
use crate::types::Vector3;
use std::collections::VecDeque;

/// IMU Signal Analyzer for decomposing measurements into components
pub struct ImuSignalAnalyzer {
    /// Rolling window of recent accel measurements
    accel_history: VecDeque<Vector3>,
    /// Rolling window of recent gyro measurements
    gyro_history: VecDeque<Vector3>,
    /// Rolling history of timestamps for frequency estimation
    timestamp_history: VecDeque<i64>,
    /// Running estimate of gravity direction
    gravity_estimate: Vector3,
    /// Running estimate of accel bias
    accel_bias_estimate: Vector3,
    /// Running estimate of gyro bias
    gyro_bias_estimate: Vector3,
    /// Window size for moving statistics
    window_size: usize,
    /// Adaptive noise floor estimate
    noise_floor: f32,
    /// Current detected motor state
    motor_state: MotorState,
    /// Estimated fundamental rotor frequency [Hz]
    fundamental_freq_hz: f32,
    /// Vibration magnitude threshold for motor detection [m/s²]
    motor_on_threshold: f32,
}

impl ImuSignalAnalyzer {
    /// Create new IMU signal analyzer with default motor threshold
    pub fn new(window_size: usize) -> Self {
        Self {
            accel_history: VecDeque::with_capacity(window_size),
            gyro_history: VecDeque::with_capacity(window_size),
            timestamp_history: VecDeque::with_capacity(window_size),
            gravity_estimate: Vector3::new(0.0, 0.0, -9.81),
            accel_bias_estimate: Vector3::zeros(),
            gyro_bias_estimate: Vector3::zeros(),
            window_size,
            noise_floor: DEFAULT_NOISE_FLOOR,
            motor_state: MotorState::Off,
            fundamental_freq_hz: 0.0,
            motor_on_threshold: DEFAULT_MOTOR_THRESHOLD,
        }
    }

    /// Create analyzer with custom motor detection threshold
    pub fn new_with_threshold(window_size: usize, motor_threshold: f32) -> Self {
        let mut analyzer = Self::new(window_size);
        analyzer.motor_on_threshold = motor_threshold;
        analyzer
    }

    /// Process single IMU measurement
    pub fn process_measurement(&mut self, imu_data: &ImuData) {
        let accel = Vector3::new(imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]);
        let gyro = Vector3::new(imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);

        // Maintain rolling history
        self._add_to_history(accel, gyro, imu_data.timestamp);

        // Detect motor state and update bias estimates
        if self.accel_history.len() >= MIN_SAMPLES_FOR_DETECTION {
            self._detect_motor_state();
            self._update_bias_estimates();
        }
    }

    /// Add measurement to rolling history windows
    fn _add_to_history(&mut self, accel: Vector3, gyro: Vector3, timestamp: i64) {
        self.accel_history.push_back(accel);
        self.gyro_history.push_back(gyro);
        self.timestamp_history.push_back(timestamp);

        if self.accel_history.len() > self.window_size {
            self.accel_history.pop_front();
            self.gyro_history.pop_front();
            self.timestamp_history.pop_front();
        }
    }

    /// Get current bias estimates
    pub fn get_bias_estimates(&self) -> (Vector3, Vector3) {
        (self.accel_bias_estimate, self.gyro_bias_estimate)
    }

    /// Decompose measurements into harmonic components
    pub fn decompose_harmonics(&mut self) -> HarmonicDecomposition {
        let quality = self._compute_signal_quality();
        let residuals = self._compute_residuals();
        let (fundamental, residual_harmonics) =
            harmonic::extract_harmonics(&residuals, self.motor_state);

        HarmonicDecomposition {
            gravity: self.gravity_estimate,
            accel_bias: self.accel_bias_estimate,
            gyro_bias: self.gyro_bias_estimate,
            fundamental_harmonic: fundamental,
            residual_harmonics,
            quality,
            motor_state: self.motor_state,
        }
    }

    /// Compute residuals (accel - gravity - bias)
    fn _compute_residuals(&self) -> Vec<Vector3> {
        self.accel_history
            .iter()
            .map(|accel| accel - self.gravity_estimate - self.accel_bias_estimate)
            .collect()
    }

    /// Detect motor state based on vibration magnitude
    fn _detect_motor_state(&mut self) {
        let vibration_rms = motor_detection::compute_vibration_rms(&self.accel_history);
        self.motor_state = motor_detection::detect_motor_state(
            self.motor_state,
            vibration_rms,
            self.motor_on_threshold,
        );

        if matches!(self.motor_state, MotorState::Running) {
            self.fundamental_freq_hz = spectral::estimate_fundamental_frequency(
                &self.accel_history,
                &self.timestamp_history,
            );
        } else if self.motor_state == MotorState::Off {
            self.fundamental_freq_hz = 0.0;
        }
    }

    /// Update running bias estimates
    fn _update_bias_estimates(&mut self) {
        let samples: Vec<Vector3> = self.accel_history.iter().copied().collect();
        let accel_mean = spectral::compute_mean(&samples);

        bias::update_gravity_estimate(&mut self.gravity_estimate, &accel_mean);
        bias::update_accel_bias_estimate(
            &mut self.accel_bias_estimate,
            &accel_mean,
            &self.gravity_estimate,
            self.motor_state,
        );
    }

    /// Compute signal quality metrics
    fn _compute_signal_quality(&self) -> SignalQuality {
        let noise_floor = quality::get_noise_floor(self.noise_floor, self.motor_state);
        quality::compute_signal_quality(
            &self.accel_history,
            self.motor_state,
            self.fundamental_freq_hz,
            noise_floor,
        )
    }

    /// Adaptive noise floor tracking
    pub fn update_noise_floor(&mut self, new_noise_floor: f32) {
        if new_noise_floor > 0.0 {
            // Exponential moving average
            self.noise_floor = self.noise_floor * 0.9 + new_noise_floor * 0.1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_analyzer_creation() {
        let analyzer = ImuSignalAnalyzer::new(100);
        assert_eq!(analyzer.window_size, 100);
        assert_eq!(analyzer.motor_state, MotorState::Off);
    }

    #[test]
    fn test_gravity_estimation() {
        let mut analyzer = ImuSignalAnalyzer::new(100);

        // Feed stationary measurements (gravity only)
        for _ in 0..100 {
            let imu = ImuData {
                timestamp: 0,
                accel: [0.01, -0.02, -9.81],
                gyro: [0.0, 0.0, 0.0],
            };
            analyzer.process_measurement(&imu);
        }

        let decomp = analyzer.decompose_harmonics();
        // Gravity should be close to -9.81 on Z axis
        assert!((decomp.gravity.z + 9.81).abs() < 0.5);
        // Motors should be detected as off (stationary data)
        assert_eq!(decomp.motor_state, MotorState::Off);
        assert_eq!(decomp.quality.fundamental_freq_hz, 0.0);
    }

    #[test]
    fn test_motor_state_detection() {
        let mut analyzer = ImuSignalAnalyzer::new(100);

        // Phase 1: Motors off (stationary)
        for _ in 0..50 {
            let imu = ImuData {
                timestamp: 0,
                accel: [0.01, -0.01, -9.81],
                gyro: [0.0, 0.0, 0.0],
            };
            analyzer.process_measurement(&imu);
        }

        let decomp = analyzer.decompose_harmonics();
        assert_eq!(decomp.motor_state, MotorState::Off);

        // Phase 2: Motors running (high vibration - random component)
        for i in 0..100 {
            // Add random high-frequency vibration
            let vibration_x = 1.5 * ((i % 7) as f64 / 7.0 - 0.5);
            let vibration_y = 1.2 * ((i % 5) as f64 / 5.0 - 0.5);
            let vibration_z = 0.8 * ((i % 3) as f64 / 3.0 - 0.5);

            let imu = ImuData {
                timestamp: (i * 5_000_000) as i64,
                accel: [vibration_x, vibration_y, -9.81 + vibration_z],
                gyro: [0.0, 0.0, 0.0],
            };
            analyzer.process_measurement(&imu);
        }

        let decomp = analyzer.decompose_harmonics();
        assert!(
            matches!(
                decomp.motor_state,
                MotorState::Running | MotorState::Transitioning
            ),
            "Expected motors running/transitioning, got {:?}",
            decomp.motor_state
        );
    }

    #[test]
    fn test_empty_history_handling() {
        let analyzer = ImuSignalAnalyzer::new(50);

        // Should handle empty history gracefully
        let (bias_a, bias_g) = analyzer.get_bias_estimates();
        assert_eq!(bias_a.norm(), 0.0);
        assert_eq!(bias_g.norm(), 0.0);
    }

    #[test]
    fn test_single_measurement() {
        let mut analyzer = ImuSignalAnalyzer::new(50);

        analyzer.process_measurement(&ImuData {
            timestamp: 0,
            accel: [0.0, 0.0, -9.81],
            gyro: [0.0, 0.0, 0.0],
        });

        // Should not crash with single measurement
        let decomp = analyzer.decompose_harmonics();
        assert_eq!(decomp.motor_state, MotorState::Off);
    }

    #[test]
    fn test_window_size_limits() {
        // Small window
        let mut analyzer_small = ImuSignalAnalyzer::new(10);
        for i in 0..20 {
            analyzer_small.process_measurement(&ImuData {
                timestamp: i,
                accel: [0.0, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
            });
        }

        let decomp_small = analyzer_small.decompose_harmonics();
        assert!(decomp_small.quality.snr[0].is_finite());

        // Large window
        let mut analyzer_large = ImuSignalAnalyzer::new(200);
        for i in 0..250 {
            analyzer_large.process_measurement(&ImuData {
                timestamp: i,
                accel: [0.0, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
            });
        }

        let decomp_large = analyzer_large.decompose_harmonics();
        assert!(decomp_large.quality.snr[0].is_finite());
    }

    #[test]
    fn test_noise_floor_update() {
        let mut analyzer = ImuSignalAnalyzer::new(50);

        // Update noise floor
        analyzer.update_noise_floor(0.05);

        for _ in 0..50 {
            analyzer.process_measurement(&ImuData {
                timestamp: 0,
                accel: [0.0, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
            });
        }

        let quality = analyzer.decompose_harmonics().quality;

        // SNR should reflect updated noise floor
        assert!(quality.snr[2].is_finite() && quality.snr[2] > 0.0);
    }
}
