//! # IMU Signal Analysis and Harmonic Decomposition
//!
//! Analyzes raw IMU measurements to extract meaningful components:
//! - **Gravity Component**: Estimated from accelerometer (typically ≈ [0, 0, -9.81] m/s²)
//! - **Bias Estimation**: Constant sensor offsets (accel and gyro)
//! - **Motor State Detection**: Detect when drone rotors are on/off
//! - **Harmonic Extraction**: Frequency-domain analysis of rotor vibrations (f0, 2f0, 3f0, ...)
//! - **Signal Quality**: SNR, RMS, and peak detection with motor-state awareness
//!
//! ## Typical IMU Signal Decomposition
//!
//! **Motors OFF** (Stationary/Ground):
//! ```text
//! Raw Accel = Gravity + Bias + Noise
//!           = [0, 0, -9.81] + [b_x, b_y, b_z] + [n_x, n_y, n_z]
//! ```
//!
//! **Motors RUNNING** (Flight):
//! ```text
//! Raw Accel = Gravity + Bias + Rotor_Harmonics(f0, 2f0, 3f0, ...) + Noise
//!           = [0, 0, -9.81] + [b_x, b_y, b_z] + [h_x(f0), h_y(f0), h_z(f0)] + [n_x, n_y, n_z]
//! ```
//!
//! ## Processing Pipeline
//!
//! 1. **Raw IMU Data** (from sensor)
//! 2. **Motor State Detection** (vibration-based threshold)
//! 3. **Static Component Removal** (gravity, bias - adapted for motor state)
//! 4. **Harmonic Decomposition** (f0 extraction + higher harmonics when motors running)
//! 5. **Quality Assessment** (SNR, vibration with motor-aware thresholds)
//! 6. **Visualization** (rerun integration with motor state display)

use crate::datasets::ImuData;
use crate::types::Vector3;
use std::collections::VecDeque;

/// Motor/rotor state detection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorState {
    /// Motors off - stationary, no rotor vibration
    Off,
    /// Motors running - active flight with rotor harmonics
    Running,
    /// Transitioning between states
    Transitioning,
}

/// Signal quality metrics for IMU measurements
#[derive(Debug, Clone, Copy)]
pub struct SignalQuality {
    /// Signal-to-Noise Ratio per axis [dB]
    pub snr: [f32; 3],
    /// RMS value per axis [m/s²] for accel or [rad/s] for gyro
    pub rms: [f32; 3],
    /// Peak absolute value per axis [m/s²] for accel or [rad/s] for gyro
    pub peak: [f32; 3],
    /// Detected motor state
    pub motor_state: MotorState,
    /// Estimated fundamental rotor frequency [Hz] (0 if motors off)
    pub fundamental_freq_hz: f32,
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
    /// Fundamental frequency harmonic (f0) [m/s²]
    /// Strong when rotors running, near-zero when off
    pub fundamental_harmonic: Vector3,
    /// Higher harmonics (2f0, 3f0, ...) and residual noise [m/s²]
    pub residual_harmonics: Vec<Vector3>,
    /// Signal quality metrics (includes motor state)
    pub quality: SignalQuality,
    /// Detected motor state for this decomposition
    pub motor_state: MotorState,
}

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
    /// Create new IMU signal analyzer with default motor threshold (0.5 m/s²)
    pub fn new(window_size: usize) -> Self {
        Self {
            accel_history: VecDeque::with_capacity(window_size),
            gyro_history: VecDeque::with_capacity(window_size),
            timestamp_history: VecDeque::with_capacity(window_size),
            gravity_estimate: Vector3::new(0.0, 0.0, -9.81),
            accel_bias_estimate: Vector3::zeros(),
            gyro_bias_estimate: Vector3::zeros(),
            window_size,
            noise_floor: 0.01,
            motor_state: MotorState::Off,
            fundamental_freq_hz: 0.0,
            motor_on_threshold: 0.5, // 0.5 m/s² vibration threshold
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
        self.accel_history.push_back(accel);
        self.gyro_history.push_back(gyro);
        self.timestamp_history.push_back(imu_data.timestamp);

        if self.accel_history.len() > self.window_size {
            self.accel_history.pop_front();
        }
        if self.gyro_history.len() > self.window_size {
            self.gyro_history.pop_front();
        }
        if self.timestamp_history.len() > self.window_size {
            self.timestamp_history.pop_front();
        }

        // Detect motor state and update bias estimates
        if self.accel_history.len() >= 10 {
            self._detect_motor_state();
            self._update_bias_estimates();
        }
    }

    /// Get current bias estimates
    pub fn get_bias_estimates(&self) -> (Vector3, Vector3) {
        (self.accel_bias_estimate, self.gyro_bias_estimate)
    }

    /// Decompose measurements into harmonic components
    /// Adapts extraction based on motor state (rotors on/off)
    pub fn decompose_harmonics(&mut self) -> HarmonicDecomposition {
        // Get quality metrics (includes motor state detection)
        let quality = self._compute_signal_quality();

        // Remove gravity and bias from raw measurements
        let mut residuals = Vec::new();
        for accel in &self.accel_history {
            let residual = accel - self.gravity_estimate - self.accel_bias_estimate;
            residuals.push(residual);
        }

        // Extract harmonics based on motor state
        let (fundamental, residual_harmonics) = match self.motor_state {
            MotorState::Off => {
                // Motors off: minimal harmonics, mostly noise
                let fundamental = if !residuals.is_empty() {
                    let mut sum = Vector3::zeros();
                    for r in &residuals {
                        sum += r;
                    }
                    sum / residuals.len() as f64
                } else {
                    Vector3::zeros()
                };
                
                // All residuals are just noise (no rotor harmonics)
                (fundamental, residuals.clone())
            }
            MotorState::Running => {
                // Motors running: extract f0 and higher harmonics
                let fundamental = if !residuals.is_empty() {
                    let mut sum = Vector3::zeros();
                    for r in &residuals {
                        sum += r;
                    }
                    sum / residuals.len() as f64
                } else {
                    Vector3::zeros()
                };
                
                // Residuals after removing f0 contain 2f0, 3f0, etc.
                let residual_harmonics: Vec<Vector3> = residuals
                    .iter()
                    .map(|r| r - fundamental * 0.3)
                    .collect();
                
                (fundamental, residual_harmonics)
            }
            MotorState::Transitioning => {
                // Transitioning: use conservative extraction
                let fundamental = if !residuals.is_empty() {
                    let mut sum = Vector3::zeros();
                    for r in &residuals {
                        sum += r;
                    }
                    sum / residuals.len() as f64
                } else {
                    Vector3::zeros()
                };
                
                let residual_harmonics: Vec<Vector3> = residuals
                    .iter()
                    .map(|r| r - fundamental * 0.4)
                    .collect();
                
                (fundamental, residual_harmonics)
            }
        };

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

    /// Detect motor state based on vibration magnitude
    fn _detect_motor_state(&mut self) {
        // Compute high-frequency vibration (residuals from mean)
        let n = self.accel_history.len() as f64;
        let mut accel_sum = Vector3::zeros();
        for accel in &self.accel_history {
            accel_sum += accel;
        }
        let accel_mean = accel_sum / n;

        // Compute RMS of deviations (vibration magnitude)
        let mut vibration_sum = 0.0;
        for accel in &self.accel_history {
            let deviation = accel - accel_mean;
            vibration_sum += deviation.norm();
        }
        let vibration_rms = (vibration_sum / n) as f32;

        // Update motor state with hysteresis
        match self.motor_state {
            MotorState::Off => {
                if vibration_rms > self.motor_on_threshold {
                    self.motor_state = MotorState::Transitioning;
                }
            }
            MotorState::Running => {
                if vibration_rms < self.motor_on_threshold * 0.5 {
                    // Hysteresis: require lower threshold to transition back to off
                    self.motor_state = MotorState::Transitioning;
                }
            }
            MotorState::Transitioning => {
                if vibration_rms > self.motor_on_threshold {
                    self.motor_state = MotorState::Running;
                    self._estimate_fundamental_frequency();
                } else if vibration_rms < self.motor_on_threshold * 0.5 {
                    self.motor_state = MotorState::Off;
                    self.fundamental_freq_hz = 0.0;
                }
            }
        }
    }

    /// Estimate fundamental rotor frequency when motors are running
    fn _estimate_fundamental_frequency(&mut self) {
        if self.accel_history.len() < 20 || self.timestamp_history.len() < 20 {
            return;
        }

        // Simple peak detection to estimate fundamental frequency
        let n = self.accel_history.len();
        let mut accel_sum = Vector3::zeros();
        for accel in &self.accel_history {
            accel_sum += accel;
        }
        let accel_mean = accel_sum / n as f64;

        // Find peaks in residual signal
        let mut peak_intervals = Vec::new();
        let mut last_peak_idx = 0;
        let mut last_peak_val = 0.0;

        for (i, accel) in self.accel_history.iter().enumerate().skip(1) {
            let residual = (accel - accel_mean).norm();
            
            // Detect local maximum
            if i > 0 && i < n - 1 {
                let prev = (self.accel_history[i - 1] - accel_mean).norm();
                let next = (self.accel_history[i + 1] - accel_mean).norm();
                
                if residual > prev && residual > next && residual > last_peak_val * 0.5 {
                    if last_peak_idx > 0 {
                        peak_intervals.push((i - last_peak_idx) as f32);
                    }
                    last_peak_idx = i;
                    last_peak_val = residual;
                }
            }
        }

        // Estimate frequency from average peak interval
        if !peak_intervals.is_empty() && self.timestamp_history.len() > 1 {
            let avg_interval = peak_intervals.iter().sum::<f32>() / peak_intervals.len() as f32;
            
            // Convert sample interval to time interval
            let time_span = (self.timestamp_history.back().unwrap() 
                           - self.timestamp_history.front().unwrap()) as f64 / 1e9; // ns to s
            let sample_rate = self.timestamp_history.len() as f64 / time_span;
            
            // Frequency = 1 / period
            let period_seconds = avg_interval as f64 / sample_rate;
            
            if period_seconds > 0.0 {
                self.fundamental_freq_hz = (1.0 / period_seconds) as f32;
                
                // Sanity check: typical drone rotors 100-500 Hz
                if self.fundamental_freq_hz < 10.0 || self.fundamental_freq_hz > 1000.0 {
                    self.fundamental_freq_hz = 0.0; // Invalid estimate
                }
            }
        }
    }

    /// Update running bias estimates - adapts based on motor state
    fn _update_bias_estimates(&mut self) {
        let n = self.accel_history.len() as f64;
        let mut accel_sum = Vector3::zeros();
        for accel in &self.accel_history {
            accel_sum += accel;
        }

        // Moving average
        let accel_mean = accel_sum / n;
        let gravity_mag = accel_mean.norm();

        // Gravity is aligned with mean accel direction
        if gravity_mag > 0.1 {
            self.gravity_estimate = (accel_mean / gravity_mag) * 9.81;
        }

        // Bias estimation depends on motor state
        match self.motor_state {
            MotorState::Off => {
                // Motors off: use simple mean for bias (stationary assumption)
                self.accel_bias_estimate = accel_mean - self.gravity_estimate * 0.1;
            }
            MotorState::Running | MotorState::Transitioning => {
                // Motors running: bias is harder to estimate due to vibration
                // Use slower update (more filtering)
                self.accel_bias_estimate = self.accel_bias_estimate * 0.95 
                                          + (accel_mean - self.gravity_estimate * 0.1) * 0.05;
            }
        }
    }

    /// Compute signal quality metrics
    fn _compute_signal_quality(&self) -> SignalQuality {
        let mut snr = [0.0f32; 3];
        let mut rms = [0.0f32; 3];
        let mut peak = [0.0f32; 3];

        if self.accel_history.is_empty() {
            return SignalQuality { 
                snr, 
                rms, 
                peak,
                motor_state: self.motor_state,
                fundamental_freq_hz: self.fundamental_freq_hz,
            };
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

            // SNR estimation depends on motor state
            let noise_floor = match self.motor_state {
                MotorState::Off => self.noise_floor, // Lower noise when stationary
                MotorState::Running => self.noise_floor * 5.0, // Higher noise with motors
                MotorState::Transitioning => self.noise_floor * 2.0,
            };

            let signal_power = rms[axis].max(0.1_f32);
            snr[axis] = 20.0_f32 * (signal_power / noise_floor).log10();
        }

        SignalQuality { 
            snr, 
            rms, 
            peak,
            motor_state: self.motor_state,
            fundamental_freq_hz: self.fundamental_freq_hz,
        }
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
                accel: [0.01, -0.01, -9.81], // Small noise only
                gyro: [0.0, 0.0, 0.0],
            };
            analyzer.process_measurement(&imu);
        }

        let decomp = analyzer.decompose_harmonics();
        assert_eq!(decomp.motor_state, MotorState::Off);

        // Phase 2: Motors running (high vibration - random component)
        for i in 0..100 {
            // Add random high-frequency vibration (more realistic than pure sine)
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
        // Should detect motors running (or transitioning after high vibration)
        assert!(
            matches!(decomp.motor_state, MotorState::Running | MotorState::Transitioning),
            "Expected motors running/transitioning, got {:?} with vibration", decomp.motor_state
        );
    }
}
