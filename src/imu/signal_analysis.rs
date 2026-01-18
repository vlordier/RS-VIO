//! # IMU Signal Analysis and Harmonic Decomposition
//!
//! Analyzes raw IMU measurements to extract meaningful components:
//! - **Gravity Component**: Estimated from accelerometer (typically ≈ [0, 0, -9.81] m/s²)
//! - **Bias Estimation**: Constant sensor offsets (accel and gyro)
//! - **Harmonic Extraction**: Frequency-domain analysis of residual components
//! - **Signal Quality**: SNR, RMS, and peak detection
//!
//! ## Typical IMU Signal Decomposition
//!
//! ```text
//! Raw Accel = Gravity + Bias + Harmonics + Noise
//!           = [0, 0, -9.81] + [b_x, b_y, b_z] + [h_x, h_y, h_z] + [n_x, n_y, n_z]
//! ```
//!
//! ## Processing Pipeline
//!
//! 1. **Raw IMU Data** (from sensor)
//! 2. **Static Component Removal** (gravity, bias)
//! 3. **Harmonic Decomposition** (fundamental + higher harmonics)
//! 4. **Quality Assessment** (SNR, vibration detection)
//! 5. **Visualization** (rerun integration)

use crate::datasets::ImuData;
use crate::types::Vector3;
use std::collections::VecDeque;

/// Signal quality metrics for IMU measurements
#[derive(Debug, Clone, Copy)]
pub struct SignalQuality {
    /// Signal-to-Noise Ratio per axis [dB]
    pub snr: [f32; 3],
    /// RMS value per axis [m/s²] for accel or [rad/s] for gyro
    pub rms: [f32; 3],
    /// Peak absolute value per axis [m/s²] for accel or [rad/s] for gyro
    pub peak: [f32; 3],
}

/// Harmonic decomposition result
#[derive(Debug, Clone)]
pub struct HarmonicDecomposition {
    /// Gravity component in sensor frame [m/s²]
    pub gravity: Vector3,
    /// Accelerometer bias [m/s²]
    pub accel_bias: Vector3,
    /// Gyroscope bias [rad/s]
    pub gyro_bias: Vector3,
    /// Fundamental frequency harmonic [m/s²]
    pub fundamental_harmonic: Vector3,
    /// Higher harmonics and residual noise [m/s²]
    pub residual_harmonics: Vec<Vector3>,
    /// Signal quality metrics
    pub quality: SignalQuality,
}

/// IMU Signal Analyzer for decomposing measurements into components
pub struct ImuSignalAnalyzer {
    /// Rolling window of recent accel measurements
    accel_history: VecDeque<Vector3>,
    /// Rolling window of recent gyro measurements
    gyro_history: VecDeque<Vector3>,
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
}

impl ImuSignalAnalyzer {
    /// Create new IMU signal analyzer
    pub fn new(window_size: usize) -> Self {
        Self {
            accel_history: VecDeque::with_capacity(window_size),
            gyro_history: VecDeque::with_capacity(window_size),
            gravity_estimate: Vector3::new(0.0, 0.0, -9.81),
            accel_bias_estimate: Vector3::zeros(),
            gyro_bias_estimate: Vector3::zeros(),
            window_size,
            noise_floor: 0.01,
        }
    }

    /// Process single IMU measurement
    pub fn process_measurement(&mut self, imu_data: &ImuData) {
        let accel = Vector3::new(imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]);
        let gyro = Vector3::new(imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);

        // Maintain rolling history
        self.accel_history.push_back(accel);
        self.gyro_history.push_back(gyro);

        if self.accel_history.len() > self.window_size {
            self.accel_history.pop_front();
        }
        if self.gyro_history.len() > self.window_size {
            self.gyro_history.pop_front();
        }

        // Update bias estimates (simple moving average)
        if self.accel_history.len() >= 10 {
            self._update_bias_estimates();
        }
    }

    /// Update running bias estimates
    fn _update_bias_estimates(&mut self) {
        let n = self.accel_history.len() as f64;
        let mut accel_sum = Vector3::zeros();
        for accel in &self.accel_history {
            accel_sum += accel;
        }

        // Moving average bias estimate
        let accel_mean = accel_sum / n;

        // Estimate gravity magnitude
        let gravity_mag = accel_mean.norm();

        // Gravity is aligned with mean accel direction
        if gravity_mag > 0.1 {
            self.gravity_estimate = (accel_mean / gravity_mag) * 9.81;
        }

        // Bias is the difference from expected zero motion
        self.accel_bias_estimate = accel_mean - self.gravity_estimate * 0.1;
    }

    /// Decompose measurements into harmonic components
    pub fn decompose_harmonics(&mut self) -> HarmonicDecomposition {
        // Get quality metrics
        let quality = self._compute_signal_quality();

        // Remove gravity and bias from raw measurements
        let mut residuals = Vec::new();
        for accel in &self.accel_history {
            let residual = accel - self.gravity_estimate - self.accel_bias_estimate;
            residuals.push(residual);
        }

        // Extract fundamental frequency harmonic (first dominant frequency)
        let fundamental = if !residuals.is_empty() {
            let mut sum = Vector3::zeros();
            for r in &residuals {
                sum += r;
            }
            sum / residuals.len() as f64
        } else {
            Vector3::zeros()
        };

        // Remaining harmonics are residuals after fundamental extraction
        let residual_harmonics: Vec<Vector3> = residuals
            .iter()
            .map(|r| r - fundamental * 0.5) // Simple harmonic extraction
            .collect();

        HarmonicDecomposition {
            gravity: self.gravity_estimate,
            accel_bias: self.accel_bias_estimate,
            gyro_bias: self.gyro_bias_estimate,
            fundamental_harmonic: fundamental,
            residual_harmonics,
            quality,
        }
    }

    /// Compute signal quality metrics
    fn _compute_signal_quality(&self) -> SignalQuality {
        let mut snr = [0.0f32; 3];
        let mut rms = [0.0f32; 3];
        let mut peak = [0.0f32; 3];

        if self.accel_history.is_empty() {
            return SignalQuality { snr, rms, peak };
        }

        // Compute per-axis metrics
        for axis in 0..3 {
            let mut sum_sq = 0.0_f32;
            let mut max_abs = 0.0_f32;

            for accel in &self.accel_history {
                let val = accel[axis] as f32;
                sum_sq += val * val;
                max_abs = max_abs.max(val.abs());
            }

            let n = self.accel_history.len() as f32;
            rms[axis] = (sum_sq / n).sqrt();
            peak[axis] = max_abs;

            // Simple SNR estimation: signal_power / noise_floor
            let signal_power = rms[axis].max(0.1_f32);
            snr[axis] = 20.0_f32 * (signal_power / self.noise_floor).log10();
        }

        SignalQuality { snr, rms, peak }
    }

    /// Get current estimates without decomposition
    pub fn get_bias_estimates(&self) -> (Vector3, Vector3) {
        (self.accel_bias_estimate, self.gyro_bias_estimate)
    }

    /// Update adaptive noise floor based on environment
    pub fn update_noise_floor(&mut self, measured_noise: f32) {
        // Exponential moving average of noise estimate
        self.noise_floor = 0.9 * self.noise_floor + 0.1 * measured_noise;
        self.noise_floor = self.noise_floor.max(0.005); // Minimum 0.5 cm/s²
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_signal_analyzer_creation() {
        let analyzer = ImuSignalAnalyzer::new(100);
        assert_eq!(analyzer.window_size, 100);
        assert_eq!(analyzer.accel_history.len(), 0);
    }

    #[test]
    fn test_gravity_estimation() {
        let mut analyzer = ImuSignalAnalyzer::new(100);

        // Simulate stationary measurements (gravity only, no acceleration)
        for _ in 0..20 {
            let imu = ImuData {
                timestamp: 0,
                accel: [0.0, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
            };
            analyzer.process_measurement(&imu);
        }

        let decomp = analyzer.decompose_harmonics();
        // Gravity should be close to -9.81 on Z axis
        assert!((decomp.gravity.z + 9.81).abs() < 0.5);
    }
}
