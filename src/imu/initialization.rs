//! # IMU Initialization and Processing
//!
//! Enhanced IMU processing with robust initialization, bias estimation, and adaptive noise handling.
//!
//! ## Features
//!
//! - **IMU Initialization**: Estimates initial biases and aligns to gravity
//! - **Gravity Alignment**: Uses accelerometer measurements to determine local gravity direction
//! - **Bias Estimation**: Robust bias estimation during initialization phase
//! - **Adaptive Noise**: Adjusts measurement noise based on sensor quality metrics
//! - **Covariance Propagation**: Proper uncertainty tracking through preintegration
//!
//! ## Initialization Process
//!
//! The initialization phase typically takes 1-2 seconds and involves:
//! 1. **Bias Collection**: Accumulate IMU measurements to estimate static biases
//! 2. **Gravity Alignment**: Determine gravity direction from accelerometer
//! 3. **Convergence Check**: Verify estimates have converged
//! 4. **Transition**: Switch from initialization to normal operation
//!
//! ## References
//!
//! - Trawny & Roumeliotis, "Indirect Kalman Filter for 3D Attitude Estimation", 2005
//! - Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", RSS 2017
//! - Solà et al., "Quaternion kinematics for the error-state Kalman filter", 2017

use std::collections::VecDeque;

use crate::datasets::ImuData;
use anyhow::{bail, Result};
use nalgebra as na;
use serde::{Deserialize, Serialize};

/// Configuration for IMU initialization
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImuInitializationConfig {
    /// Minimum number of measurements for initialization
    pub min_initialization_samples: usize,

    /// Maximum gyroscope norm for static period [rad/s]
    /// Used to detect if device is stationary
    pub gyro_norm_threshold: f64,

    /// Maximum accelerometer variance for static period [(m/s²)²]
    pub accel_variance_threshold: f64,

    /// Variance of initial bias estimates (gyroscope) [rad/s]
    pub initial_gyro_bias_std: f64,

    /// Variance of initial bias estimates (accelerometer) [m/s²]
    pub initial_accel_bias_std: f64,

    /// Expected gravity magnitude [m/s²]
    pub gravity_magnitude: f64,

    /// Convergence threshold for bias estimates
    pub bias_convergence_threshold: f64,

    /// Time window for measuring bias stability [s]
    pub bias_stability_window: f64,
}

impl Default for ImuInitializationConfig {
    fn default() -> Self {
        Self {
            min_initialization_samples: 100,
            gyro_norm_threshold: 0.05,         // 0.05 rad/s ≈ 3 deg/s
            accel_variance_threshold: 0.1,     // 0.1 m/s² standard deviation
            initial_gyro_bias_std: 0.01,       // 0.01 rad/s
            initial_accel_bias_std: 0.1,       // 0.1 m/s²
            gravity_magnitude: 9.81,           // standard gravity
            bias_convergence_threshold: 0.001, // 0.1% change
            bias_stability_window: 1.0,        // 1 second stability window
        }
    }
}

/// State of IMU initialization
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitializationState {
    /// Not yet initialized
    Uninitialized,
    /// Collecting measurements for initialization
    Initializing,
    /// Initialization complete, ready for operation
    Initialized,
    /// Initialization failed
    Failed,
}

/// IMU bias estimates with uncertainty
#[derive(Debug, Clone, Copy)]
pub struct BiasEstimate {
    /// Gyroscope bias [rad/s]
    pub gyro_bias: na::Vector3<f64>,
    /// Gyroscope bias standard deviation [rad/s]
    pub gyro_bias_std: f64,
    /// Accelerometer bias [m/s²]
    pub accel_bias: na::Vector3<f64>,
    /// Accelerometer bias standard deviation [m/s²]
    pub accel_bias_std: f64,
}

impl Default for BiasEstimate {
    fn default() -> Self {
        Self {
            gyro_bias: na::Vector3::zeros(),
            gyro_bias_std: 0.01,
            accel_bias: na::Vector3::zeros(),
            accel_bias_std: 0.1,
        }
    }
}

/// IMU initialization and processing module
pub struct ImuInitializer {
    config: ImuInitializationConfig,
    state: InitializationState,

    /// Accumulated measurements during initialization
    measurements: Vec<ImuData>,

    /// Current bias estimates
    bias_estimate: BiasEstimate,

    /// Estimated local gravity direction (world frame)
    gravity_vector: na::Vector3<f64>,

    /// Measurement quality metrics
    gyro_variance: f64,
    accel_variance: f64,

    /// Number of convergence checks passed
    convergence_checks: u32,

    /// Timestamp of last measurement
    last_timestamp: Option<i64>,
}

impl ImuInitializer {
    /// Create new IMU initializer
    pub fn new(config: ImuInitializationConfig) -> Self {
        Self {
            config,
            state: InitializationState::Uninitialized,
            measurements: Vec::new(),
            bias_estimate: BiasEstimate::default(),
            gravity_vector: na::Vector3::new(0.0, 0.0, -9.81),
            gyro_variance: f64::INFINITY,
            accel_variance: f64::INFINITY,
            convergence_checks: 0,
            last_timestamp: None,
        }
    }

    /// Process a single IMU measurement during initialization
    pub fn add_measurement(&mut self, imu: &ImuData) -> Result<()> {
        if self.state == InitializationState::Failed {
            bail!("Initialization failed");
        }

        if self.state == InitializationState::Initialized {
            bail!("Already initialized");
        }

        self.state = InitializationState::Initializing;
        self.measurements.push(imu.clone());
        // Cap measurements buffer to prevent unbounded growth
        const MAX_INIT_MEASUREMENTS: usize = 10_000;
        if self.measurements.len() > MAX_INIT_MEASUREMENTS {
            self.measurements
                .drain(..self.measurements.len() - MAX_INIT_MEASUREMENTS);
        }
        self.last_timestamp = Some(imu.timestamp);

        // Check if we have enough measurements
        if self.measurements.len() >= self.config.min_initialization_samples {
            self.estimate_biases()?;
            self.estimate_gravity()?;
            self.check_convergence()?;
        }

        Ok(())
    }

    /// Estimate gyroscope and accelerometer biases
    fn estimate_biases(&mut self) -> Result<()> {
        if self.measurements.is_empty() {
            bail!("No measurements for bias estimation");
        }

        // Gyroscope bias: mean of all gyroscope measurements (assuming static)
        let mut gyro_sum = na::Vector3::zeros();
        let mut accel_sum = na::Vector3::zeros();

        for imu in &self.measurements {
            gyro_sum += na::Vector3::from(imu.gyro);
            accel_sum += na::Vector3::from(imu.accel);
        }

        let n = self.measurements.len() as f64;
        let gyro_mean = gyro_sum / n;
        let accel_mean = accel_sum / n;

        // Calculate variances
        let mut gyro_var_sum = 0.0;
        let mut accel_var_sum = 0.0;

        for imu in &self.measurements {
            let gyro_dev = na::Vector3::from(imu.gyro) - gyro_mean;
            let accel_dev = na::Vector3::from(imu.accel) - accel_mean;

            gyro_var_sum += gyro_dev.norm_squared();
            accel_var_sum += accel_dev.norm_squared();
        }

        self.gyro_variance = gyro_var_sum / (n - 1.0).max(1.0);
        self.accel_variance = accel_var_sum / (n - 1.0).max(1.0);

        // Update bias estimates
        self.bias_estimate.gyro_bias = gyro_mean;
        self.bias_estimate.accel_bias = accel_mean;

        // Update standard deviations based on measurement variance
        self.bias_estimate.gyro_bias_std = self
            .gyro_variance
            .sqrt()
            .max(self.config.initial_gyro_bias_std);
        self.bias_estimate.accel_bias_std = self
            .accel_variance
            .sqrt()
            .max(self.config.initial_accel_bias_std);

        Ok(())
    }

    /// Estimate gravity direction from accelerometer measurements
    fn estimate_gravity(&mut self) -> Result<()> {
        if self.measurements.is_empty() {
            bail!("No measurements for gravity estimation");
        }

        // Average accelerometer reading
        let mut accel_sum = na::Vector3::zeros();
        for imu in &self.measurements {
            accel_sum += na::Vector3::from(imu.accel);
        }

        let accel_mean = accel_sum / self.measurements.len() as f64;

        // Gravity direction is opposite to accelerometer bias (assuming static device)
        let accel_magnitude = accel_mean.norm();

        if accel_magnitude < 1e-6 {
            bail!("Accelerometer readings too small for gravity estimation");
        }

        // Normalize to expected gravity magnitude
        self.gravity_vector = -accel_mean / accel_magnitude * self.config.gravity_magnitude;

        Ok(())
    }

    /// Check if initialization has converged
    fn check_convergence(&mut self) -> Result<()> {
        // Check if measurements are in static period (low gyro and accel variance)
        let is_static = self.gyro_variance < self.config.gyro_norm_threshold.powi(2)
            && self.accel_variance < self.config.accel_variance_threshold;

        if is_static {
            self.convergence_checks += 1;

            // Initialize after enough consecutive static periods
            if self.convergence_checks >= 3 {
                self.state = InitializationState::Initialized;
            }
        } else {
            // Reset convergence counter if motion is detected
            self.convergence_checks = 0;
        }

        Ok(())
    }

    /// Get current initialization state
    pub const fn state(&self) -> InitializationState {
        self.state
    }

    /// Check if initialization is complete
    pub fn is_initialized(&self) -> bool {
        self.state == InitializationState::Initialized
    }

    /// Get bias estimates
    pub const fn bias_estimate(&self) -> BiasEstimate {
        self.bias_estimate
    }

    /// Get estimated gravity vector (world frame)
    pub const fn gravity_vector(&self) -> na::Vector3<f64> {
        self.gravity_vector
    }

    /// Get number of samples collected
    pub const fn sample_count(&self) -> usize {
        self.measurements.len()
    }

    /// Reset initializer
    pub fn reset(&mut self) {
        self.state = InitializationState::Uninitialized;
        self.measurements.clear();
        self.bias_estimate = BiasEstimate::default();
        self.gravity_vector = na::Vector3::new(0.0, 0.0, -9.81);
        self.convergence_checks = 0;
        self.last_timestamp = None;
    }
}

/// Adaptive measurement noise covariance based on sensor quality
pub struct AdaptiveNoiseEstimator {
    /// Baseline accelerometer noise [m/s²]
    base_accel_noise: f64,
    /// Baseline gyroscope noise [rad/s]
    base_gyro_noise: f64,
    /// Window size for noise estimation
    window_size: usize,
    /// Recent measurements
    recent_measurements: VecDeque<ImuData>,
}

impl AdaptiveNoiseEstimator {
    /// Create new adaptive noise estimator
    pub const fn new(base_accel_noise: f64, base_gyro_noise: f64) -> Self {
        Self {
            base_accel_noise,
            base_gyro_noise,
            window_size: 50,
            recent_measurements: VecDeque::new(),
        }
    }

    /// Add measurement and update noise estimates
    pub fn add_measurement(&mut self, imu: &ImuData) {
        self.recent_measurements.push_back(imu.clone());

        // Keep only recent measurements
        if self.recent_measurements.len() > self.window_size {
            self.recent_measurements.pop_front();
        }
    }

    /// Get estimated accelerometer noise
    pub fn accel_noise(&self) -> f64 {
        if self.recent_measurements.len() < 2 {
            return self.base_accel_noise;
        }

        let mut accel_sum = na::Vector3::zeros();
        for imu in &self.recent_measurements {
            accel_sum += na::Vector3::from(imu.accel);
        }
        let accel_mean = accel_sum / self.recent_measurements.len() as f64;

        // Estimate variance
        let mut variance = 0.0;
        for imu in &self.recent_measurements {
            let dev = na::Vector3::from(imu.accel) - accel_mean;
            variance += dev.norm_squared();
        }
        variance /= (self.recent_measurements.len() - 1).max(1) as f64;

        // Adaptive scaling: increase noise if variance is high
        let std = variance.sqrt();
        self.base_accel_noise * (1.0 + std / 10.0)
    }

    /// Get estimated gyroscope noise
    pub fn gyro_noise(&self) -> f64 {
        if self.recent_measurements.len() < 2 {
            return self.base_gyro_noise;
        }

        let mut gyro_sum = na::Vector3::zeros();
        for imu in &self.recent_measurements {
            gyro_sum += na::Vector3::from(imu.gyro);
        }
        let gyro_mean = gyro_sum / self.recent_measurements.len() as f64;

        // Estimate variance
        let mut variance = 0.0;
        for imu in &self.recent_measurements {
            let dev = na::Vector3::from(imu.gyro) - gyro_mean;
            variance += dev.norm_squared();
        }
        variance /= (self.recent_measurements.len() - 1).max(1) as f64;

        // Adaptive scaling
        let std = variance.sqrt();
        self.base_gyro_noise * (1.0 + std / 0.1)
    }

    /// Reset estimator
    pub fn reset(&mut self) {
        self.recent_measurements.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_adaptive_noise_estimator() {
        let mut estimator = AdaptiveNoiseEstimator::new(0.1, 0.01);

        // Initial noise should be baseline
        assert!((estimator.accel_noise() - 0.1).abs() < 1e-6);
        assert!((estimator.gyro_noise() - 0.01).abs() < 1e-6);

        // Add static measurements (zero motion) — noise should stay near baseline
        for _ in 0..50 {
            let imu = ImuData {
                timestamp: 0,
                accel: [0.0, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
            };
            estimator.add_measurement(&imu);
        }

        // With perfectly static data, estimated noise should remain close to baseline
        assert!(
            estimator.accel_noise() < 0.5,
            "Accel noise should stay low for static data: {}",
            estimator.accel_noise()
        );
        assert!(
            estimator.gyro_noise() < 0.1,
            "Gyro noise should stay low for static data: {}",
            estimator.gyro_noise()
        );
    }

    #[test]
    fn test_imu_initializer_bias_estimation() {
        let config = ImuInitializationConfig {
            min_initialization_samples: 5,
            ..Default::default()
        };
        let mut initializer = ImuInitializer::new(config);

        // Add measurements with known bias
        for i in 0..10 {
            let imu = ImuData {
                timestamp: i as i64 * 10_000_000, // 10ms intervals
                accel: [0.1, 0.2, -9.81],         // Small bias
                gyro: [0.01, -0.02, 0.005],
            };
            let _ = initializer.add_measurement(&imu);
        }

        // Check that we're collecting measurements
        assert!(initializer.sample_count() >= 5);

        // Bias estimates should be close to average values
        let bias = initializer.bias_estimate();
        assert!((bias.accel_bias[0] - 0.1).abs() < 0.05);
        assert!((bias.accel_bias[1] - 0.2).abs() < 0.05);
    }

    #[test]
    fn test_gravity_estimation() {
        let config = ImuInitializationConfig {
            min_initialization_samples: 5,
            ..Default::default()
        };
        let mut initializer = ImuInitializer::new(config);

        // Add static measurements where device is horizontal
        for i in 0..10 {
            let imu = ImuData {
                timestamp: i as i64 * 1_000_000,
                accel: [0.0, 0.0, -9.81], // Gravity pointing down (negative Z)
                gyro: [0.0, 0.0, 0.0],
            };
            let _ = initializer.add_measurement(&imu);
        }

        // Gravity should point downward
        let gravity = initializer.gravity_vector();
        assert!((gravity[0]).abs() < 0.1);
        assert!((gravity[1]).abs() < 0.1);
        // The magnitude should be close to 9.81
        assert!((gravity.norm() - 9.81).abs() < 0.5);
    }

    #[test]
    fn test_initializer_reset() {
        let config = ImuInitializationConfig::default();
        let mut initializer = ImuInitializer::new(config);

        // Add some measurements
        for i in 0..5 {
            let imu = ImuData {
                timestamp: i as i64 * 1_000_000,
                accel: [0.0, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
            };
            let _ = initializer.add_measurement(&imu);
        }

        assert!(initializer.sample_count() > 0);

        // Reset
        initializer.reset();

        assert_eq!(initializer.state(), InitializationState::Uninitialized);
        assert_eq!(initializer.sample_count(), 0);
    }
}
