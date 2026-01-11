//! # IMU Preintegration
//!
//! Implements IMU preintegration for visual-inertial odometry.
//!
//! ## Key Features
//!
//! - **Preintegrated IMU Factor**: Computes relative motion between two poses
//!   using accumulated IMU measurements
//! - **Motion Prediction**: Uses gyroscope to predict feature displacement
//! - **Velocity Estimation**: Derives velocity from accelerometer integration
//! - **Robust Initialization**: Bias estimation and gravity alignment
//! - **Adaptive Noise**: Quality-based measurement noise estimation
//!
//! ## Algorithm
//!
//! ### Preintegration
//! Given IMU measurements ω(t), a(t) between times t_i and t_j:
//!
//! ```text
//! ΔR_ij = ∏ exp(ω_k * Δt)
//! Δv_ij = Σ R_ik * a_k * Δt
//! Δp_ij = Σ (R_ik * a_k * Δt²/2 + v_k * Δt)
//! ```
//!
//! ### Motion Prediction
//! Feature displacement from camera i to camera j:
//! ```text
//! p_j ≈ p_i + v_i * Δt + 0.5 * a * Δt² - R_i * T_BC * ω × p_i_c
//! ```
//!
//! ## References
//!
//! - Forster et al., "IMU Preintegration on Manifold for Efficient
//!   Visual-Inertial SLAM", RSS 2017
//! - Lupton & Sukkarieh, "Visual-Inertial-Aided Navigation for
//!   High-Dynamic Motion in GPS-Denied Environments", 2011

pub mod initialization;

pub use initialization::{
    AdaptiveNoiseEstimator, BiasEstimate, ImuInitializationConfig, ImuInitializer, InitializationState,
};

use crate::datasets::ImuData;
use nalgebra as na;

/// Configuration for IMU processing
#[derive(Debug, Clone)]
pub struct ImuConfig {
    /// Gyroscope noise density [rad/s/√Hz]
    pub gyro_noise_density: f64,
    /// Accelerometer noise density [m/s²/√Hz]
    pub accel_noise_density: f64,
    /// Gyroscope bias random walk [rad/s²/√Hz]
    pub gyro_bias_random_walk: f64,
    /// Accelerometer bias random walk [m/s³/√Hz]
    pub accel_bias_random_walk: f64,
    /// Gravity vector in world frame [m/s²]
    pub gravity: [f64; 3],
}

impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            gyro_noise_density: 1e-4,
            accel_noise_density: 1e-2,
            gyro_bias_random_walk: 1e-5,
            accel_bias_random_walk: 1e-4,
            gravity: [0.0, 0.0, -9.81],
        }
    }
}

/// Preintegrated IMU measurements between two keyframes
#[derive(Debug, Clone)]
pub struct PreintegratedImu {
    /// Delta rotation from i to j [R_ij]
    pub delta_rotation: na::UnitQuaternion<f64>,
    /// Delta velocity from i to j [m/s]
    pub delta_velocity: na::Vector3<f64>,
    /// Delta position from i to j [m]
    pub delta_position: na::Vector3<f64>,
    /// Time interval [s]
    pub delta_time: f64,
    /// Covariance matrix (9x9 for rotation, velocity, position)
    pub covariance: na::DMatrix<f64>,
    /// Jacobian of preintegration w.r.t. rotation at i
    pub jacobian_wrt_rotation: na::Matrix3<f64>,
    /// Jacobian of preintegration w.r.t. velocity at i
    pub jacobian_wrt_velocity: na::Matrix3<f64>,
    /// Jacobian of preintegration w.r.t. acceleration bias at i
    pub jacobian_wrt_accel_bias: na::Matrix3<f64>,
    /// Jacobian of preintegration w.r.t. gyro bias at i
    pub jacobian_wrt_gyro_bias: na::Matrix3<f64>,
}

impl PreintegratedImu {
    /// Create new preintegration with identity
    pub fn new() -> Self {
        Self {
            delta_rotation: na::UnitQuaternion::identity(),
            delta_velocity: na::Vector3::zeros(),
            delta_position: na::Vector3::zeros(),
            delta_time: 0.0,
            covariance: na::DMatrix::identity(9, 9) * 1e8,
            jacobian_wrt_rotation: na::Matrix3::identity(),
            jacobian_wrt_velocity: na::Matrix3::zeros(),
            jacobian_wrt_accel_bias: na::Matrix3::zeros(),
            jacobian_wrt_gyro_bias: na::Matrix3::zeros(),
        }
    }

    /// Reset preintegration to identity
    pub fn reset(&mut self) {
        self.delta_rotation = na::UnitQuaternion::identity();
        self.delta_velocity = na::Vector3::zeros();
        self.delta_position = na::Vector3::zeros();
        self.delta_time = 0.0;
        self.covariance = na::DMatrix::identity(9, 9) * 1e8;
        self.jacobian_wrt_rotation = na::Matrix3::identity();
        self.jacobian_wrt_velocity = na::Matrix3::zeros();
        self.jacobian_wrt_accel_bias = na::Matrix3::zeros();
        self.jacobian_wrt_gyro_bias = na::Matrix3::zeros();
    }
}

impl Default for PreintegratedImu {
    fn default() -> Self {
        Self::new()
    }
}

/// IMU preintegrator for visual-inertial odometry
#[allow(dead_code)]
pub struct ImuPreintegrator {
    config: ImuConfig,
    current: PreintegratedImu,
    last_gyro: na::Vector3<f64>,
    last_accel: na::Vector3<f64>,
    has_initial_measurement: bool,
}

impl ImuPreintegrator {
    /// Create new IMU preintegrator
    pub fn new(config: ImuConfig) -> Self {
        Self {
            config,
            current: PreintegratedImu::new(),
            last_gyro: na::Vector3::zeros(),
            last_accel: na::Vector3::zeros(),
            has_initial_measurement: false,
        }
    }

    /// Process a single IMU measurement and update preintegration
    pub fn propagate(&mut self, imu: &ImuData, dt: f64) {
        let gyro = na::Vector3::new(imu.gyro[0] as f64, imu.gyro[1] as f64, imu.gyro[2] as f64);
        let accel = na::Vector3::new(
            imu.accel[0] as f64,
            imu.accel[1] as f64,
            imu.accel[2] as f64,
        );

        if !self.has_initial_measurement {
            self.last_gyro = gyro;
            self.last_accel = accel;
            self.has_initial_measurement = true;
            return;
        }

        // Average of first and last measurements for midpoint integration
        let gyro_avg = (self.last_gyro + gyro) * 0.5;
        let accel_avg = (self.last_accel + accel) * 0.5;

        // Update rotation using Rodrigues' formula
        let delta_angle = gyro_avg * dt;
        let delta_rot = na::UnitQuaternion::new(delta_angle);
        self.current.delta_rotation = delta_rot * self.current.delta_rotation;

        // Update velocity (in local frame)
        self.current.delta_velocity += self.current.delta_rotation * accel_avg * dt;

        // Update position
        self.current.delta_position += self.current.delta_velocity * dt;

        // Update time
        self.current.delta_time += dt;

        // Store for next iteration
        self.last_gyro = gyro;
        self.last_accel = accel;
    }

    /// Get current preintegrated measurements
    pub fn get(&self) -> &PreintegratedImu {
        &self.current
    }

    /// Reset preintegrator
    pub fn reset(&mut self) {
        self.current.reset();
        self.has_initial_measurement = false;
    }
}

/// Motion prediction using gyroscope
#[allow(dead_code)]
pub struct ImuMotionPredictor {
    config: ImuConfig,
    last_rotation: Option<na::UnitQuaternion<f64>>,
    last_timestamp: Option<i64>,
}

impl ImuMotionPredictor {
    /// Create new motion predictor
    pub fn new(config: ImuConfig) -> Self {
        Self {
            config,
            last_rotation: None,
            last_timestamp: None,
        }
    }

    /// Predict feature displacement from gyroscope measurements
    ///
    /// Given a feature observed at pixel (u, v) in the previous frame,
    /// predict its position in the current frame based on angular velocity.
    ///
    /// # Arguments
    /// * `imu_measurements` - List of IMU measurements between frames
    /// * `prev_pixel` - Feature pixel in previous frame (u, v)
    /// * `focal_length` - Camera focal length [pixels]
    /// * `baseline` - Stereo baseline [m]
    ///
    /// # Returns
    /// Predicted pixel displacement (du, dv)
    #[allow(dead_code)]
    pub fn predict_feature_displacement(
        &self,
        imu_measurements: &[ImuData],
        _prev_pixel: (f64, f64),
        focal_length: f64,
    ) -> (f64, f64) {
        if imu_measurements.is_empty() {
            return (0.0, 0.0);
        }

        // Integrate gyroscope to get total rotation
        let mut total_rotation = na::Vector3::zeros();
        let mut last_ts = imu_measurements[0].timestamp;

        for imu in imu_measurements {
            let dt = (imu.timestamp - last_ts) as f64 / 1e9;
            if dt > 0.0 {
                total_rotation += na::Vector3::new(
                    imu.gyro[0] as f64 * dt,
                    imu.gyro[1] as f64 * dt,
                    imu.gyro[2] as f64 * dt,
                );
            }
            last_ts = imu.timestamp;
        }

        // Convert to rotation angle and axis
        let angle = total_rotation.norm();
        if angle < 1e-6 {
            return (0.0, 0.0);
        }
        let _axis = total_rotation / angle;

        // Rotation angle in image plane (simplified)
        // For small rotations, du ≈ -ω_y * f, dv ≈ ω_x * f
        let predicted_du = -total_rotation[1] * focal_length;
        let predicted_dv = total_rotation[0] * focal_length;

        (predicted_du, predicted_dv)
    }

    /// Update predictor with new rotation
    pub fn update(&mut self, timestamp: i64, rotation: na::UnitQuaternion<f64>) {
        if let Some((last_ts, _last_rot)) = self.last_timestamp.zip(self.last_rotation.as_ref()) {
            let _dt = (timestamp - last_ts) as f64 / 1e9;
            if _dt > 0.0 {
                // Could use this for velocity estimation
            }
        }
        self.last_timestamp = Some(timestamp);
        self.last_rotation = Some(rotation);
    }
}

/// Velocity estimator using accelerometer
pub struct VelocityEstimator {
    config: ImuConfig,
    velocity: na::Vector3<f64>,
    initialized: bool,
}

impl VelocityEstimator {
    /// Create new velocity estimator
    pub fn new(config: ImuConfig) -> Self {
        Self {
            config,
            velocity: na::Vector3::zeros(),
            initialized: false,
        }
    }

    /// Initialize velocity from IMU integration
    ///
    /// # Arguments
    /// * `imu_measurements` - IMU measurements during initialization
    /// * `initial_orientation` - Initial body orientation
    pub fn initialize_from_imu(
        &mut self,
        imu_measurements: &[ImuData],
        initial_orientation: &na::UnitQuaternion<f64>,
    ) {
        if imu_measurements.len() < 2 {
            return;
        }

        let gravity = na::Vector3::new(
            self.config.gravity[0],
            self.config.gravity[1],
            self.config.gravity[2],
        );

        // Integrate accelerometer to get velocity change
        let mut delta_v = na::Vector3::zeros();
        let mut last_ts = imu_measurements[0].timestamp;

        for imu in imu_measurements.iter().skip(1) {
            let dt = (imu.timestamp - last_ts) as f64 / 1e9;
            if dt > 0.0 {
                let accel = na::Vector3::new(
                    imu.accel[0] as f64,
                    imu.accel[1] as f64,
                    imu.accel[2] as f64,
                );
                // Rotate to world frame and remove gravity
                let accel_world = initial_orientation * accel - gravity;
                delta_v += accel_world * dt;
            }
            last_ts = imu.timestamp;
        }

        self.velocity = delta_v;
        self.initialized = true;
    }

    /// Get current velocity estimate
    pub fn get_velocity(&self) -> na::Vector3<f64> {
        self.velocity
    }

    /// Update velocity estimate with new IMU measurements
    pub fn update(&mut self, imu_measurements: &[ImuData], dt: f64) {
        if !self.initialized || imu_measurements.is_empty() {
            return;
        }

        let gravity = na::Vector3::new(
            self.config.gravity[0],
            self.config.gravity[1],
            self.config.gravity[2],
        );

        // Integrate accelerometer
        for imu in imu_measurements {
            let accel = na::Vector3::new(
                imu.accel[0] as f64,
                imu.accel[1] as f64,
                imu.accel[2] as f64,
            );
            // Assume current orientation is approximately identity
            let accel_world = accel - gravity;
            self.velocity += accel_world * dt;
        }
    }

    /// Check if velocity is initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }
}

/// Online IMU-camera extrinsic calibration
pub struct ExtrinsicCalibrator {
    /// Current estimate of T_BC (body to camera transform)
    pub T_BC: na::Matrix4<f64>,
    /// Optimization state
    accumulated_rotations: Vec<na::UnitQuaternion<f64>>,
    accumulated_cam_poses: Vec<na::Matrix4<f64>>,
    iterations: usize,
}

impl ExtrinsicCalibrator {
    /// Create new calibrator with initial guess
    pub fn new(initial_T_BC: na::Matrix4<f64>) -> Self {
        Self {
            T_BC: initial_T_BC,
            accumulated_rotations: Vec::new(),
            accumulated_cam_poses: Vec::new(),
            iterations: 0,
        }
    }

    /// Accumulate pose measurements for calibration
    pub fn add_measurement(
        &mut self,
        camera_pose: &na::Matrix4<f64>,
        rotation_quaternion: na::UnitQuaternion<f64>,
    ) {
        self.accumulated_cam_poses.push(*camera_pose);
        self.accumulated_rotations.push(rotation_quaternion);
    }

    /// Run one iteration of extrinsic calibration
    ///
    /// Minimizes: Σ || q(BC_i) ⊗ q(CB) - q(WC_i) ⊗ q(CW_{i-1}) ||²
    pub fn calibrate_iteration(&mut self) -> f64 {
        if self.accumulated_rotations.len() < 10 {
            return 0.0;
        }

        // Simple iterative refinement of rotation
        let mut total_correction = na::Vector3::zeros();

        for i in 1..self.accumulated_rotations.len() {
            let dq = self.accumulated_rotations[i - 1].inverse() * self.accumulated_rotations[i];

            // Extract rotation axis-angle
            let angle = 2.0 * dq.i.atan2(dq.w);
            let axis = na::Vector3::new(dq.i, dq.j, dq.k) / (dq.w + 1e-10).sqrt();
            if angle.is_finite() {
                total_correction += axis * angle;
            }
        }

        let avg_correction = total_correction / (self.accumulated_rotations.len() as f64 - 1.0);

        // Apply small correction
        let correction_rot = na::UnitQuaternion::new(avg_correction * 0.1);
        let rotmat =
            na::Rotation3::from_matrix_unchecked(self.T_BC.fixed_view::<3, 3>(0, 0).into_owned());
        let current_rot = na::UnitQuaternion::from_rotation_matrix(&rotmat);
        let new_rot = correction_rot * current_rot;

        // Update rotation part of T_BC
        let new_rotmat = new_rot.to_rotation_matrix().into_inner();
        self.T_BC
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&new_rotmat);

        self.iterations += 1;

        avg_correction.norm()
    }

    /// Get current extrinsic calibration
    pub fn get_extrinsics(&self) -> na::Matrix4<f64> {
        self.T_BC
    }

    /// Number of accumulated measurements
    pub fn measurement_count(&self) -> usize {
        self.accumulated_rotations.len()
    }
}

/// IMU motion prior for optimization
///
/// Provides motion constraints from preintegrated IMU measurements
/// to guide bundle adjustment optimization.
#[derive(Debug, Clone)]
pub struct ImuMotionPrior {
    /// Preintegrated rotation from i to j
    pub delta_rotation: na::UnitQuaternion<f64>,
    /// Preintegrated velocity change from i to j
    pub delta_velocity: na::Vector3<f64>,
    /// Preintegrated position change from i to j
    pub delta_position: na::Vector3<f64>,
    /// Time interval
    pub delta_time: f64,
    /// Initial pose at time i
    pub initial_pose: na::Matrix4<f64>,
    /// Initial velocity at time i
    pub initial_velocity: na::Vector3<f64>,
    /// Gravity vector in world frame
    pub gravity: na::Vector3<f64>,
}

impl ImuMotionPrior {
    /// Create from preintegrated measurements
    pub fn from_preintegration(
        preint: &PreintegratedImu,
        initial_pose: na::Matrix4<f64>,
        initial_velocity: na::Vector3<f64>,
        gravity: na::Vector3<f64>,
    ) -> Self {
        Self {
            delta_rotation: preint.delta_rotation,
            delta_velocity: preint.delta_velocity,
            delta_position: preint.delta_position,
            delta_time: preint.delta_time,
            initial_pose,
            initial_velocity,
            gravity,
        }
    }

    /// Compute predicted state at time j
    ///
    /// Returns (predicted_pose, predicted_velocity)
    pub fn predict_state(&self) -> (na::Matrix4<f64>, na::Vector3<f64>) {
        // Rotation prediction
        let R_W_Bi = na::Rotation3::from_matrix_unchecked(
            self.initial_pose.fixed_view::<3, 3>(0, 0).into_owned(),
        );
        let R_W_Bj = R_W_Bi * self.delta_rotation.to_rotation_matrix();

        // Velocity prediction
        let v_W_Bj = self.initial_velocity + self.delta_velocity + self.gravity * self.delta_time;

        // Position prediction
        let p_W_Bi = self.initial_pose.fixed_view::<3, 1>(0, 3).into_owned();
        let p_W_Bj = p_W_Bi
            + self.initial_velocity * self.delta_time
            + 0.5 * self.gravity * self.delta_time * self.delta_time
            + self.delta_position;

        // Compose pose matrix
        let mut T_W_Bj = na::Matrix4::identity();
        T_W_Bj
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&R_W_Bj.into_inner());
        T_W_Bj.fixed_view_mut::<3, 1>(0, 3).copy_from(&p_W_Bj);

        (T_W_Bj, v_W_Bj)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_preintegrated_imu_creation() {
        let pimu = PreintegratedImu::new();
        assert!((pimu.delta_time - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_imu_preintegrator() {
        let config = ImuConfig::default();
        let mut integrator = ImuPreintegrator::new(config);

        // Simulate two IMU measurements
        let imu_data1 = ImuData {
            timestamp: 1000000000,
            gyro: [0.01, 0.0, 0.0],
            accel: [0.0, 0.0, 9.81],
        };
        let imu_data2 = ImuData {
            timestamp: 1000000010,
            gyro: [0.01, 0.0, 0.0],
            accel: [0.0, 0.0, 9.81],
        };

        integrator.propagate(&imu_data1, 0.01);
        integrator.propagate(&imu_data2, 0.01);

        assert!(integrator.get().delta_time > 0.0);
    }

    #[test]
    fn test_motion_predictor() {
        let config = ImuConfig::default();
        let predictor = ImuMotionPredictor::new(config);

        // No IMU measurements should give zero displacement
        let disp = predictor.predict_feature_displacement(&[], (320.0, 240.0), 500.0);
        assert!((disp.0).abs() < 1e-6);
        assert!((disp.1).abs() < 1e-6);
    }

    #[test]
    fn test_velocity_estimator() {
        let config = ImuConfig::default();
        let mut estimator = VelocityEstimator::new(config);

        assert!(!estimator.is_initialized());

        let imu_measurements = vec![
            ImuData {
                timestamp: 0,
                gyro: [0.0; 3],
                accel: [0.0, 0.0, 9.81],
            },
            ImuData {
                timestamp: 10000000,
                gyro: [0.0; 3],
                accel: [0.0, 0.0, 9.81],
            },
        ];

        let orientation = na::UnitQuaternion::identity();
        estimator.initialize_from_imu(&imu_measurements, &orientation);

        assert!(estimator.is_initialized());
    }

    #[test]
    fn test_extrinsic_calibrator() {
        let T_BC = na::Matrix4::identity();
        let mut calibrator = ExtrinsicCalibrator::new(T_BC);

        assert_eq!(calibrator.measurement_count(), 0);

        let pose = na::Matrix4::identity();
        let rot = na::UnitQuaternion::identity();
        calibrator.add_measurement(&pose, rot);

        assert_eq!(calibrator.measurement_count(), 1);
    }
}
