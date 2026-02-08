//! # Error-State Kalman Filter for IMU
//!
//! Implements an ESKF for tracking:
//! - 3D velocity
//! - Gyroscope bias
//! - Accelerometer bias
//!
//! Based on:
//! - "Quaternion kinematics for the error-state Kalman filter" (Solà 2017)
//! - "Indirect Kalman Filter for 3D Attitude Estimation" (Trawny & Roumeliotis 2005)
//!
//! ## State Vector (9D)
//!
//! ```text
//! x = [v, b_g, b_a]^T
//! where:
//!   v   ∈ R³: velocity in world frame [m/s]
//!   b_g ∈ R³: gyroscope bias [rad/s]
//!   b_a ∈ R³: accelerometer bias [m/s²]
//! ```
//!
//! ## Error State (9D)
//!
//! ```text
//! δx = [δv, δb_g, δb_a]^T
//! ```
//!
//! ## Process Model
//!
//! ```text
//! v̇ = R * (a - b_a) + g
//! ḃ_g = η_bg ~ N(0, σ²_bg)
//! ḃ_a = η_ba ~ N(0, σ²_ba)
//! ```

use crate::datasets::ImuData;
use crate::imu::preintegration::{exp_map_so3, skew_symmetric, ImuNoise};
use nalgebra as na;

/// State of the ESKF
#[derive(Debug, Clone)]
pub struct EskfState {
    /// Velocity in world frame [m/s]
    pub velocity: na::Vector3<f64>,

    /// Gyroscope bias [rad/s]
    pub gyro_bias: na::Vector3<f64>,

    /// Accelerometer bias [m/s²]
    pub accel_bias: na::Vector3<f64>,

    /// 9x9 state covariance matrix
    pub covariance: na::SMatrix<f64, 9, 9>,

    /// Last update timestamp
    pub timestamp: Option<i64>,
}

impl EskfState {
    /// Create new ESKF state with initial uncertainty
    pub fn new() -> Self {
        let mut covariance = na::SMatrix::<f64, 9, 9>::zeros();

        // Initial velocity uncertainty: 1 m/s
        covariance.fixed_view_mut::<3, 3>(0, 0).fill_diagonal(1.0);

        // Initial gyro bias uncertainty: 0.01 rad/s
        covariance
            .fixed_view_mut::<3, 3>(3, 3)
            .fill_diagonal(0.01f64.powi(2));

        // Initial accel bias uncertainty: 0.1 m/s²
        covariance
            .fixed_view_mut::<3, 3>(6, 6)
            .fill_diagonal(0.1f64.powi(2));

        Self {
            velocity: na::Vector3::zeros(),
            gyro_bias: na::Vector3::zeros(),
            accel_bias: na::Vector3::zeros(),
            covariance,
            timestamp: None,
        }
    }

    /// Initialize from static IMU measurements
    pub fn initialize_from_static(
        &mut self,
        gyro_mean: na::Vector3<f64>,
        accel_mean: na::Vector3<f64>,
        gyro_std: f64,
        accel_std: f64,
    ) {
        self.velocity = na::Vector3::zeros(); // Static = zero velocity
        self.gyro_bias = gyro_mean;
        self.accel_bias = accel_mean;

        // Set covariance based on measurement statistics
        self.covariance
            .fixed_view_mut::<3, 3>(0, 0)
            .fill_diagonal(0.01); // Small velocity uncertainty
        self.covariance
            .fixed_view_mut::<3, 3>(3, 3)
            .fill_diagonal(gyro_std.powi(2));
        self.covariance
            .fixed_view_mut::<3, 3>(6, 6)
            .fill_diagonal(accel_std.powi(2));
    }

    /// Force the covariance matrix to be symmetric (numerical drift correction).
    fn ensure_symmetric(&mut self) {
        self.covariance = 0.5 * (self.covariance + self.covariance.transpose());
    }
}

impl Default for EskfState {
    fn default() -> Self {
        Self::new()
    }
}

/// Extract 3 diagonal standard deviations from a covariance matrix starting at `offset`.
///
/// Clamps negative variances (numerical artifacts) to zero before taking the square root.
#[inline]
fn diagonal_std(cov: &na::SMatrix<f64, 9, 9>, offset: usize) -> na::Vector3<f64> {
    na::Vector3::new(
        cov[(offset, offset)].max(0.0).sqrt(),
        cov[(offset + 1, offset + 1)].max(0.0).sqrt(),
        cov[(offset + 2, offset + 2)].max(0.0).sqrt(),
    )
}

/// Error-State Kalman Filter for velocity and bias estimation
pub struct Eskf {
    /// Current state estimate
    pub state: EskfState,

    /// Noise parameters
    noise: ImuNoise,

    /// Gravity vector in world frame [m/s²]
    gravity: na::Vector3<f64>,

    /// Current orientation (needed for accel transformation)
    orientation: na::UnitQuaternion<f64>,
}

impl Eskf {
    /// Create new ESKF with noise parameters
    pub fn new(noise: ImuNoise, gravity: na::Vector3<f64>) -> Self {
        Self {
            state: EskfState::new(),
            noise,
            gravity,
            orientation: na::UnitQuaternion::identity(),
        }
    }

    /// Update orientation from external source (e.g., visual odometry)
    pub const fn update_orientation(&mut self, R: na::UnitQuaternion<f64>) {
        self.orientation = R;
    }

    /// Predict step: propagate state and covariance using IMU
    ///
    /// Process model:
    /// ```text
    /// v̇ = R * (a - b_a) + g
    /// ω̇ = ω - b_g  (gyro rotation correction)
    /// ḃ_g = η_bg    (gyro bias random walk)
    /// ḃ_a = η_ba    (accel bias random walk)
    /// ```
    ///
    /// For tight visual-inertial coupling:
    /// - Orientation R is updated from visual odometry measurements
    /// - This provides the rotation needed to transform acceleration to world frame
    /// - Gyro provides inter-frame orientation estimate when visual updates are sparse
    pub fn predict(&mut self, imu: &ImuData, dt: f64) {
        if dt <= 0.0 || dt > 1.0 {
            log::warn!("Invalid dt = {:.6}s for IMU prediction", dt);
            return;
        }

        let gyro = imu.gyro_vec3();
        let accel = imu.accel_vec3();

        // Gyro integration: Update orientation for continuous tracking
        // This is essential even with visual odometry for high-rate IMU fusion
        let gyro_corrected = gyro - self.state.gyro_bias;
        let delta_R = exp_map_so3(gyro_corrected * dt);
        self.orientation *= delta_R;

        // Accelerometer prediction: Update velocity with bias correction
        let accel_corrected = accel - self.state.accel_bias;

        // Transform acceleration from sensor frame to world frame
        // v_{k+1} = v_k + (R * a_corrected + g) * dt
        let accel_world = self.orientation * accel_corrected;
        self.state.velocity += (accel_world + self.gravity) * dt;

        // Biases evolve as random walk (no dynamics, just noise accumulation)
        // b_{k+1} = b_k + noise

        // Covariance prediction (uncertainty growth)
        self.predict_covariance(dt, accel_corrected);

        // Update timestamp for next iteration
        self.state.timestamp = Some(imu.timestamp);
    }

    /// Predict covariance using continuous-time linearization
    fn predict_covariance(&mut self, dt: f64, accel_corrected: na::Vector3<f64>) {
        let R_rot = self.orientation.to_rotation_matrix();
        let R = R_rot.matrix();

        // State transition matrix F (9x9) for state [v, b_g, b_a]
        // dv/db_a = -R (accel bias error rotated to world frame)
        // dv/db_g = -R * [a_corrected]_x (gyro bias error causes rotation error
        //           which misrotates the accelerometer measurement)
        let mut F = na::SMatrix::<f64, 9, 9>::zeros();
        F.fixed_view_mut::<3, 3>(0, 6).copy_from(&(-R)); // dv/db_a
        let accel_skew = skew_symmetric(accel_corrected);
        F.fixed_view_mut::<3, 3>(0, 3).copy_from(&(-R * accel_skew)); // dv/db_g

        // Process noise covariance Q (9x9)
        let mut Q = na::SMatrix::<f64, 9, 9>::zeros();

        // Continuous-time process noise spectral density Q_c.
        // The outer discretization `P += P_dot * dt` provides the single dt factor,
        // so Q_c must NOT contain dt (otherwise noise is double-scaled).
        let accel_noise_var = self.noise.accel_noise_density.powi(2);
        Q.fixed_view_mut::<3, 3>(0, 0)
            .fill_diagonal(accel_noise_var);

        // Gyro bias random walk
        Q.fixed_view_mut::<3, 3>(3, 3)
            .fill_diagonal(self.noise.gyro_bias_random_walk.powi(2));

        // Accel bias random walk
        Q.fixed_view_mut::<3, 3>(6, 6)
            .fill_diagonal(self.noise.accel_bias_random_walk.powi(2));

        // Discretized covariance update (first-order)
        // P_{k+1} = P_k + (F * P_k + P_k * F^T + Q) * dt
        let P_dot = F * self.state.covariance + self.state.covariance * F.transpose() + Q;
        self.state.covariance += P_dot * dt;

        // Ensure symmetry and positive definiteness
        self.state.ensure_symmetric();
    }

    /// Update step: correct state with visual velocity measurement
    ///
    /// Used for tight visual-inertial coupling where visual system provides
    /// velocity estimates from optical flow or feature tracking.
    ///
    /// Measurement model: z = v + n, where n ~ N(0, R)
    /// Standard Kalman update:
    /// - Innovation: y = z - v
    /// - Kalman gain: K = P*H^T / (H*P*H^T + R)
    /// - State update: x := x + K*y
    /// - Covariance: P := (I - K*H)*P  (Joseph form for numerical stability)
    ///
    /// # Arguments
    /// * `measured_velocity` - Velocity estimate from visual system [m/s]
    /// * `measurement_cov` - Measurement covariance 3x3 [m²/s²]
    pub fn update_velocity(
        &mut self,
        measured_velocity: na::Vector3<f64>,
        measurement_cov: na::Matrix3<f64>,
    ) {
        // Innovation
        let y = measured_velocity - self.state.velocity;

        // Measurement matrix H (3x9): measures only velocity
        let mut H = na::SMatrix::<f64, 3, 9>::zeros();
        H.fixed_view_mut::<3, 3>(0, 0).fill_diagonal(1.0);

        // Innovation covariance
        let S = H * self.state.covariance * H.transpose() + measurement_cov;

        // Kalman gain
        let S_inv = match S.try_inverse() {
            Some(inv) => inv,
            None => return, // Skip update if S is singular
        };
        let K = self.state.covariance * H.transpose() * S_inv;

        // State update
        let correction = K * y;
        self.state.velocity += correction.fixed_rows::<3>(0).into_owned();
        self.state.gyro_bias += correction.fixed_rows::<3>(3).into_owned();
        self.state.accel_bias += correction.fixed_rows::<3>(6).into_owned();

        // Covariance update (Joseph form for numerical stability)
        let I_KH = na::SMatrix::<f64, 9, 9>::identity() - K * H;
        self.state.covariance =
            I_KH * self.state.covariance * I_KH.transpose() + K * measurement_cov * K.transpose();

        // Ensure symmetry
        self.state.ensure_symmetric();
    }

    /// Update step: correct state with zero-velocity constraint
    ///
    /// Used when system is stationary (e.g., at start/end of motion, or detected via motion detection).
    /// This strongly corrects velocity and bias estimates when we know v should be zero.
    ///
    /// # Arguments
    /// * `measurement_uncertainty` - Estimated measurement uncertainty [m/s]
    pub fn update_zero_velocity(&mut self, measurement_uncertainty: f64) {
        let measurement_cov =
            na::SMatrix::<f64, 3, 3>::from_diagonal_element(measurement_uncertainty.powi(2));
        self.update_velocity(na::Vector3::zeros(), measurement_cov);
    }

    /// Apply bias correction from optimization/refinement
    ///
    /// When biases are updated through optimization or other external process,
    /// use this to update the ESKF state and reset uncertainty appropriately.
    ///
    /// # Arguments
    /// * `new_gyro_bias` - Updated gyro bias [rad/s]
    /// * `new_accel_bias` - Updated accel bias [m/s²]
    /// * `bias_uncertainty` - New bias uncertainty (standard deviation)
    pub fn apply_bias_correction(
        &mut self,
        new_gyro_bias: na::Vector3<f64>,
        new_accel_bias: na::Vector3<f64>,
        bias_uncertainty: f64,
    ) {
        self.state.gyro_bias = new_gyro_bias;
        self.state.accel_bias = new_accel_bias;

        // Update bias covariance to reflect refinement
        self.state
            .covariance
            .fixed_view_mut::<3, 3>(3, 3)
            .fill_diagonal(bias_uncertainty.powi(2));
        self.state
            .covariance
            .fixed_view_mut::<3, 3>(6, 6)
            .fill_diagonal(bias_uncertainty.powi(2));
    }

    /// Get current velocity estimate
    pub const fn get_velocity(&self) -> na::Vector3<f64> {
        self.state.velocity
    }

    /// Get current bias estimates
    pub const fn get_biases(&self) -> (na::Vector3<f64>, na::Vector3<f64>) {
        (self.state.gyro_bias, self.state.accel_bias)
    }

    /// Get velocity uncertainty (standard deviation)
    pub fn get_velocity_std(&self) -> na::Vector3<f64> {
        diagonal_std(&self.state.covariance, 0)
    }

    /// Get bias uncertainties
    pub fn get_bias_std(&self) -> (na::Vector3<f64>, na::Vector3<f64>) {
        (
            diagonal_std(&self.state.covariance, 3),
            diagonal_std(&self.state.covariance, 6),
        )
    }

    /// Reset filter (e.g., after reinitialization)
    pub fn reset(&mut self) {
        self.state = EskfState::new();
        self.orientation = na::UnitQuaternion::identity();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eskf_initialization() {
        let noise = ImuNoise::default();
        let gravity = na::Vector3::new(0.0, 0.0, -9.81);
        let eskf = Eskf::new(noise, gravity);

        assert!(eskf.state.velocity.norm() < 1e-10);
        assert!(eskf.state.gyro_bias.norm() < 1e-10);
        assert!(eskf.state.accel_bias.norm() < 1e-10);
    }

    #[test]
    fn test_eskf_static_predict() {
        let noise = ImuNoise::default();
        let gravity = na::Vector3::new(0.0, 0.0, -9.81);
        let mut eskf = Eskf::new(noise, gravity);

        // Static IMU: accel reads [0, 0, 9.81] (gravity), gyro reads zero.
        // Net acceleration = R * (accel - bias) + gravity = [0,0,9.81] + [0,0,-9.81] = 0
        // Velocity must remain exactly zero (to floating-point precision).
        let imu = ImuData {
            timestamp: 0,
            gyro: [0.0, 0.0, 0.0],
            accel: [0.0, 0.0, 9.81],
        };

        for _ in 0..100 {
            eskf.predict(&imu, 0.01);
        }

        assert!(
            eskf.state.velocity.norm() < 1e-6,
            "Static velocity should be ~0, got {}",
            eskf.state.velocity.norm()
        );

        // Covariance must remain positive semi-definite (all eigenvalues >= 0)
        let eigenvalues = eskf.state.covariance.symmetric_eigenvalues();
        for i in 0..eigenvalues.len() {
            assert!(
                eigenvalues[i] >= -1e-12,
                "Covariance eigenvalue {} is negative: {}",
                i,
                eigenvalues[i]
            );
        }
    }

    #[test]
    fn test_eskf_constant_acceleration() {
        let noise = ImuNoise::default();
        let gravity = na::Vector3::new(0.0, 0.0, -9.81);
        let mut eskf = Eskf::new(noise, gravity);

        // Apply 1 m/s² along x. Sensor reads [1, 0, 9.81] (includes gravity).
        let imu = ImuData {
            timestamp: 0,
            gyro: [0.0, 0.0, 0.0],
            accel: [1.0, 0.0, 9.81],
        };

        let dt = 0.01;
        let steps = 100;
        for _ in 0..steps {
            eskf.predict(&imu, dt);
        }

        let expected = na::Vector3::new(1.0, 0.0, 0.0); // v = a*t = 1*1 = 1 m/s
        let error = (eskf.state.velocity - expected).norm();
        assert!(
            error < 1e-3,
            "Expected velocity ~[1,0,0], got {:?} (error {})",
            eskf.state.velocity,
            error
        );
    }

    #[test]
    fn test_eskf_covariance_growth_during_prediction() {
        let noise = ImuNoise::default();
        let gravity = na::Vector3::new(0.0, 0.0, -9.81);
        let mut eskf = Eskf::new(noise, gravity);

        let imu = ImuData {
            timestamp: 0,
            gyro: [0.0, 0.0, 0.0],
            accel: [0.0, 0.0, 9.81],
        };

        // Velocity covariance block is rows/cols 3..6 of the 15×15 covariance.
        let vel_cov_trace = |eskf: &Eskf| {
            eskf.state.covariance[(3, 3)] + eskf.state.covariance[(4, 4)] + eskf.state.covariance[(5, 5)]
        };

        let mut prev_trace = vel_cov_trace(&eskf);
        for step in 1..=50 {
            eskf.predict(&imu, 0.01);
            let cur_trace = vel_cov_trace(&eskf);
            assert!(
                cur_trace > prev_trace,
                "Velocity covariance trace must grow monotonically (step {}: {} <= {})",
                step,
                cur_trace,
                prev_trace
            );
            prev_trace = cur_trace;
        }
    }

    #[test]
    fn test_eskf_zero_velocity_update_corrects_drift() {
        let noise = ImuNoise::default();
        let gravity = na::Vector3::new(0.0, 0.0, -9.81);
        let mut eskf = Eskf::new(noise, gravity);

        // Use a slightly biased accelerometer reading to induce drift.
        let imu = ImuData {
            timestamp: 0,
            gyro: [0.0, 0.0, 0.0],
            accel: [0.05, -0.03, 9.81],
        };

        for _ in 0..50 {
            eskf.predict(&imu, 0.01);
        }

        let vel_before = eskf.state.velocity.norm();
        assert!(
            vel_before > 1e-6,
            "Drift should have caused nonzero velocity, got {}",
            vel_before
        );

        // Apply zero-velocity update with reasonable uncertainty
        eskf.update_zero_velocity(0.01);

        let vel_after = eskf.state.velocity.norm();
        assert!(
            vel_after < vel_before,
            "Zero-velocity update should reduce velocity ({} -> {})",
            vel_before,
            vel_after
        );
    }

    #[test]
    fn test_eskf_velocity_update_reduces_uncertainty() {
        let noise = ImuNoise::default();
        let gravity = na::Vector3::new(0.0, 0.0, -9.81);
        let mut eskf = Eskf::new(noise, gravity);

        let imu = ImuData {
            timestamp: 0,
            gyro: [0.0, 0.0, 0.0],
            accel: [0.0, 0.0, 9.81],
        };

        // Grow covariance via prediction
        for _ in 0..50 {
            eskf.predict(&imu, 0.01);
        }

        let vel_cov_trace = |eskf: &Eskf| {
            eskf.state.covariance[(3, 3)] + eskf.state.covariance[(4, 4)] + eskf.state.covariance[(5, 5)]
        };
        let trace_before = vel_cov_trace(&eskf);

        // Apply velocity measurement with tight uncertainty
        let measured_velocity = na::Vector3::zeros();
        let measurement_cov = na::Matrix3::identity() * 0.001;
        eskf.update_velocity(measured_velocity, measurement_cov);

        let trace_after = vel_cov_trace(&eskf);
        assert!(
            trace_after < trace_before,
            "Velocity update should reduce uncertainty ({} -> {})",
            trace_before,
            trace_after
        );
    }

    #[test]
    fn test_eskf_bias_correction_sets_values() {
        let noise = ImuNoise::default();
        let gravity = na::Vector3::new(0.0, 0.0, -9.81);
        let mut eskf = Eskf::new(noise, gravity);

        let gyro_bias = na::Vector3::new(0.01, -0.02, 0.005);
        let accel_bias = na::Vector3::new(0.1, -0.05, 0.03);
        let bias_uncertainty = 0.001;

        eskf.apply_bias_correction(gyro_bias, accel_bias, bias_uncertainty);

        let gyro_err = (eskf.state.gyro_bias - gyro_bias).norm();
        let accel_err = (eskf.state.accel_bias - accel_bias).norm();
        assert!(
            gyro_err < 1e-12,
            "Gyro bias should be exactly set, error {}",
            gyro_err
        );
        assert!(
            accel_err < 1e-12,
            "Accel bias should be exactly set, error {}",
            accel_err
        );

        // Bias uncertainty should be reflected in covariance diagonal
        let (gyro_std, accel_std) = eskf.get_bias_std();
        for i in 0..3 {
            assert!(
                (gyro_std[i] - bias_uncertainty).abs() < 1e-6,
                "Gyro bias std[{}] = {}, expected {}",
                i,
                gyro_std[i],
                bias_uncertainty
            );
            assert!(
                (accel_std[i] - bias_uncertainty).abs() < 1e-6,
                "Accel bias std[{}] = {}, expected {}",
                i,
                accel_std[i],
                bias_uncertainty
            );
        }
    }
}
