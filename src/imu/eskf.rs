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
}

impl Default for EskfState {
    fn default() -> Self {
        Self::new()
    }
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
    pub fn update_orientation(&mut self, R: na::UnitQuaternion<f64>) {
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
            eprintln!("Warning: Invalid dt = {:.6}s for IMU prediction", dt);
            return;
        }

        let gyro = na::Vector3::new(imu.gyro[0], imu.gyro[1], imu.gyro[2]);
        let accel = na::Vector3::new(imu.accel[0], imu.accel[1], imu.accel[2]);

        // Gyro integration: Update orientation for continuous tracking
        // This is essential even with visual odometry for high-rate IMU fusion
        let gyro_corrected = gyro - self.state.gyro_bias;
        let delta_R = exp_map_so3(gyro_corrected * dt);
        self.orientation = self.orientation * delta_R;

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
        self.state.covariance = 0.5 * (self.state.covariance + self.state.covariance.transpose());
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
        self.state.covariance = 0.5 * (self.state.covariance + self.state.covariance.transpose());
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
            na::SMatrix::<f64, 3, 3>::from_element(measurement_uncertainty.powi(2));
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
    pub fn get_velocity(&self) -> na::Vector3<f64> {
        self.state.velocity
    }

    /// Get current bias estimates
    pub fn get_biases(&self) -> (na::Vector3<f64>, na::Vector3<f64>) {
        (self.state.gyro_bias, self.state.accel_bias)
    }

    /// Get velocity uncertainty (standard deviation)
    pub fn get_velocity_std(&self) -> na::Vector3<f64> {
        let var = self.state.covariance.fixed_view::<3, 3>(0, 0).diagonal();
        na::Vector3::new(var[0].sqrt(), var[1].sqrt(), var[2].sqrt())
    }

    /// Get bias uncertainties
    pub fn get_bias_std(&self) -> (na::Vector3<f64>, na::Vector3<f64>) {
        let gyro_var = self.state.covariance.fixed_view::<3, 3>(3, 3).diagonal();
        let accel_var = self.state.covariance.fixed_view::<3, 3>(6, 6).diagonal();

        let gyro_std = na::Vector3::new(gyro_var[0].sqrt(), gyro_var[1].sqrt(), gyro_var[2].sqrt());
        let accel_std = na::Vector3::new(
            accel_var[0].sqrt(),
            accel_var[1].sqrt(),
            accel_var[2].sqrt(),
        );

        (gyro_std, accel_std)
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

        // For static condition: accel = gravity in sensor frame
        // After removing bias and rotating to world frame, net acceleration should be zero
        let imu = ImuData {
            timestamp: 0,
            gyro: [0.0, 0.0, 0.0],
            accel: [0.0, 0.0, 9.81], // Gravity measurement
        };

        for _ in 0..100 {
            eskf.predict(&imu, 0.01);
        }

        // Velocity should remain near zero for static condition
        // The acceleration is gravity, bias is initially 0, so after removing bias
        // we get gravity in sensor frame, which rotates to gravity in world frame
        // Adding the gravity vector results in net zero acceleration
        assert!(
            eskf.state.velocity.norm() < 0.1,
            "Velocity: {}",
            eskf.state.velocity.norm()
        );
    }
}
