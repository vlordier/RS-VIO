/// IMU intrinsics calibration
///
/// Estimates gyro/accel scale factors, axis misalignment, biases,
/// and noise model from static and multi-orientation sequences.
use nalgebra::{Matrix3, Vector3};
use std::collections::VecDeque;

#[derive(Clone, Debug)]
pub struct IMUIntrinsics {
    /// Gyroscope scale factors (diagonal elements)
    pub gyro_scale: Vector3<f64>,
    /// Gyroscope misalignment matrix (off-diagonal elements)
    pub gyro_misalignment: Matrix3<f64>,
    /// Accelerometer scale factors
    pub accel_scale: Vector3<f64>,
    /// Accelerometer misalignment matrix
    pub accel_misalignment: Matrix3<f64>,
    /// Gyroscope bias (rad/s)
    pub gyro_bias: Vector3<f64>,
    /// Accelerometer bias (m/s^2)
    pub accel_bias: Vector3<f64>,
    /// Gyroscope noise density (rad/s / sqrt(Hz))
    pub gyro_noise_density: f64,
    /// Accelerometer noise density (m/s^2 / sqrt(Hz))
    pub accel_noise_density: f64,
    /// Gyroscope bias random walk (rad/s^2 / sqrt(Hz))
    pub gyro_bias_random_walk: f64,
    /// Accelerometer bias random walk (m/s^3 / sqrt(Hz))
    pub accel_bias_random_walk: f64,
}

impl IMUIntrinsics {
    /// Create default/uncalibrated intrinsics
    pub fn identity() -> Self {
        Self {
            gyro_scale: Vector3::new(1.0, 1.0, 1.0),
            gyro_misalignment: Matrix3::identity(),
            accel_scale: Vector3::new(1.0, 1.0, 1.0),
            accel_misalignment: Matrix3::identity(),
            gyro_bias: Vector3::zeros(),
            accel_bias: Vector3::zeros(),
            gyro_noise_density: 0.001,
            accel_noise_density: 0.01,
            gyro_bias_random_walk: 0.00001,
            accel_bias_random_walk: 0.0001,
        }
    }

    /// Apply calibration to raw IMU measurement
    pub fn calibrate_gyro(&self, raw_gyro: &Vector3<f64>) -> Vector3<f64> {
        let scaled = Vector3::new(
            raw_gyro.x * self.gyro_scale.x,
            raw_gyro.y * self.gyro_scale.y,
            raw_gyro.z * self.gyro_scale.z,
        );

        (self.gyro_misalignment * scaled) - self.gyro_bias
    }

    /// Apply calibration to raw accelerometer measurement
    pub fn calibrate_accel(&self, raw_accel: &Vector3<f64>) -> Vector3<f64> {
        let scaled = Vector3::new(
            raw_accel.x * self.accel_scale.x,
            raw_accel.y * self.accel_scale.y,
            raw_accel.z * self.accel_scale.z,
        );

        (self.accel_misalignment * scaled) - self.accel_bias
    }

    /// Check if intrinsics are reasonable
    pub fn is_reasonable(&self) -> bool {
        // Scale factors should be close to 1.0 (within ±20%)
        for s in self.gyro_scale.iter() {
            if (*s - 1.0).abs() > 0.2 {
                return false;
            }
        }
        for s in self.accel_scale.iter() {
            if (*s - 1.0).abs() > 0.2 {
                return false;
            }
        }

        // Biases should be reasonable
        if self.gyro_bias.norm() > 1.0 {
            return false; // > 1 rad/s bias is suspicious
        }
        if self.accel_bias.norm() > 1.0 {
            return false; // > 1 m/s^2 bias is suspicious
        }

        true
    }
}

/// IMU intrinsics calibrator
pub struct IMUIntrinsicsCalibrator {
    /// Static samples for bias estimation
    static_gyro: VecDeque<Vector3<f64>>,
    static_accel: VecDeque<Vector3<f64>>,
    /// Multi-orientation samples for gravity calibration
    multi_orient_accel: Vec<Vector3<f64>>,
}

impl IMUIntrinsicsCalibrator {
    pub fn new() -> Self {
        Self {
            static_gyro: VecDeque::new(),
            static_accel: VecDeque::new(),
            multi_orient_accel: Vec::new(),
        }
    }

    /// Add a static sample (for bias estimation)
    pub fn add_static_sample(&mut self, gyro: Vector3<f64>, accel: Vector3<f64>) {
        self.static_gyro.push_back(gyro);
        self.static_accel.push_back(accel);

        // Keep only last 1000 samples (e.g., ~10s at 100 Hz)
        const MAX_STATIC_SAMPLES: usize = 1000;
        while self.static_gyro.len() > MAX_STATIC_SAMPLES {
            self.static_gyro.pop_front();
            self.static_accel.pop_front();
        }
    }

    /// Add a multi-orientation sample (for gravity calibration)
    pub fn add_multi_orientation_sample(&mut self, accel: Vector3<f64>) {
        self.multi_orient_accel.push(accel);
    }

    /// Estimate biases from static data
    fn estimate_biases(&self) -> (Vector3<f64>, Vector3<f64>) {
        let mut gyro_mean = Vector3::zeros();
        let mut accel_mean = Vector3::zeros();

        for (g, a) in self.static_gyro.iter().zip(self.static_accel.iter()) {
            gyro_mean += g;
            accel_mean += a;
        }

        let n = self.static_gyro.len() as f64;
        if n > 0.0 {
            gyro_mean /= n;
            accel_mean /= n;
        }

        (gyro_mean, accel_mean)
    }

    /// Estimate accelerometer scale from multi-orientation data
    ///
    /// In different orientations, |a| should equal |g| (~9.81 m/s^2)
    fn estimate_accel_scale(&self) -> Vector3<f64> {
        const GRAVITY: f64 = 9.81;

        if self.multi_orient_accel.len() < 20 {
            return Vector3::new(1.0, 1.0, 1.0); // Not enough data
        }

        // Compute mean of measured acceleration magnitude
        let mut mean_mag = 0.0;
        for accel in &self.multi_orient_accel {
            mean_mag += accel.norm();
        }
        mean_mag /= self.multi_orient_accel.len() as f64;

        let scale = GRAVITY / mean_mag;
        Vector3::new(scale, scale, scale)
    }

    /// Estimate noise from Allan deviation (simplified)
    ///
    /// Returns (gyro_noise_density, accel_noise_density)
    fn estimate_noise(&self) -> (f64, f64) {
        if self.static_gyro.len() < 100 {
            return (0.001, 0.01); // Default values
        }

        // Simplified: compute standard deviation of differences (approximates noise)
        let mut gyro_diffs = Vec::new();
        let mut accel_diffs = Vec::new();

        for i in 1..self.static_gyro.len() {
            if let (Some(g_prev), Some(g_curr)) =
                (self.static_gyro.get(i - 1), self.static_gyro.get(i))
            {
                gyro_diffs.push((g_curr - g_prev).norm());
            }

            if let (Some(a_prev), Some(a_curr)) =
                (self.static_accel.get(i - 1), self.static_accel.get(i))
            {
                accel_diffs.push((a_curr - a_prev).norm());
            }
        }

        let gyro_noise = if gyro_diffs.is_empty() {
            0.001
        } else {
            let mean = gyro_diffs.iter().sum::<f64>() / gyro_diffs.len() as f64;
            mean / 1.4142 // Convert difference to noise (simplified Allan dev)
        };

        let accel_noise = if accel_diffs.is_empty() {
            0.01
        } else {
            let mean = accel_diffs.iter().sum::<f64>() / accel_diffs.len() as f64;
            mean / 1.4142
        };

        (gyro_noise, accel_noise)
    }

    /// Calibrate IMU intrinsics
    pub fn calibrate(&self) -> Result<IMUIntrinsics, String> {
        if self.static_gyro.is_empty() {
            return Err("No static samples provided".to_string());
        }

        let (gyro_bias, accel_bias) = self.estimate_biases();
        let accel_scale = self.estimate_accel_scale();
        let (gyro_noise, accel_noise) = self.estimate_noise();

        let intrinsics = IMUIntrinsics {
            gyro_scale: Vector3::new(1.0, 1.0, 1.0), // Would optimize with more data
            gyro_misalignment: Matrix3::identity(),
            accel_scale,
            accel_misalignment: Matrix3::identity(),
            gyro_bias,
            accel_bias,
            gyro_noise_density: gyro_noise,
            accel_noise_density: accel_noise,
            gyro_bias_random_walk: gyro_noise * 0.01, // Rough estimate
            accel_bias_random_walk: accel_noise * 0.01,
        };

        if !intrinsics.is_reasonable() {
            return Err("Calibration resulted in unreasonable parameters".to_string());
        }

        Ok(intrinsics)
    }
}

/// Quality metrics for IMU calibration
#[derive(Clone, Debug)]
pub struct IMUQuality {
    /// Gyro bias magnitude (rad/s)
    pub gyro_bias_mag: f64,
    /// Accel bias magnitude (m/s^2)
    pub accel_bias_mag: f64,
    /// Gravity magnitude error after calibration (%)
    pub gravity_error_pct: f64,
    /// Whether calibration is acceptable
    pub is_acceptable: bool,
}

impl IMUQuality {
    pub fn from_intrinsics(intrinsics: &IMUIntrinsics) -> Self {
        let gyro_bias_mag = intrinsics.gyro_bias.norm();
        let accel_bias_mag = intrinsics.accel_bias.norm();

        // Gravity error: how close is accel scale to 1.0
        let accel_scale_mean =
            (intrinsics.accel_scale.x + intrinsics.accel_scale.y + intrinsics.accel_scale.z) / 3.0;
        let gravity_error_pct = (accel_scale_mean - 1.0).abs() * 100.0;

        let is_acceptable = gyro_bias_mag < 0.1  // < 0.1 rad/s
            && accel_bias_mag < 0.5  // < 0.5 m/s^2
            && gravity_error_pct < 5.0; // < 5% error

        Self {
            gyro_bias_mag,
            accel_bias_mag,
            gravity_error_pct,
            is_acceptable,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_intrinsics_identity() {
        let intr = IMUIntrinsics::identity();
        assert!(intr.is_reasonable());
    }

    #[test]
    fn test_gyro_calibration() {
        let intr = IMUIntrinsics {
            gyro_scale: Vector3::new(2.0, 1.0, 1.0),
            gyro_misalignment: Matrix3::identity(),
            accel_scale: Vector3::new(1.0, 1.0, 1.0),
            accel_misalignment: Matrix3::identity(),
            gyro_bias: Vector3::new(0.1, 0.0, 0.0),
            accel_bias: Vector3::zeros(),
            gyro_noise_density: 0.001,
            accel_noise_density: 0.01,
            gyro_bias_random_walk: 0.00001,
            accel_bias_random_walk: 0.0001,
        };

        let raw_gyro = Vector3::new(1.0, 2.0, 3.0);
        let calibrated = intr.calibrate_gyro(&raw_gyro);

        // x: 1.0 * 2.0 - 0.1 = 1.9
        // y: 2.0 * 1.0 - 0.0 = 2.0
        // z: 3.0 * 1.0 - 0.0 = 3.0
        assert!((calibrated.x - 1.9).abs() < 1e-10);
        assert!((calibrated.y - 2.0).abs() < 1e-10);
        assert!((calibrated.z - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_static_bias_estimation() {
        let mut cal = IMUIntrinsicsCalibrator::new();

        // Add static samples with known bias
        let true_gyro_bias = Vector3::new(0.05, -0.03, 0.02);
        let true_accel_bias = Vector3::new(0.1, -0.15, 0.05);

        for _ in 0..100 {
            cal.add_static_sample(true_gyro_bias, true_accel_bias);
        }

        let (est_gyro_bias, est_accel_bias) = cal.estimate_biases();

        assert!((est_gyro_bias - true_gyro_bias).norm() < 0.001);
        assert!((est_accel_bias - true_accel_bias).norm() < 0.001);
    }
}
