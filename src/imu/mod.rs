//! # IMU Processing Module - Tight Visual-Inertial Coupling
//!
//! ## Architecture Overview
//!
//! This module implements a **tightly-coupled visual-inertial odometry** system based on
//! bundle adjustment with IMU preintegration factors. The system combines:
//!
//! - **High-rate IMU preintegration** (Forster et al. 2017)
//!   - Accumulates measurements between visual keyframes
//!   - Provides motion priors and constraints
//!   - Enables bias optimization through Jacobians
//!
//! - **Error-State Kalman Filter (ESKF)**
//!   - Tracks velocity and biases between optimization iterations
//!   - Integrates gyro for continuous orientation estimate
//!   - Receives corrections from visual measurements
//!
//! - **Robust bias estimation**
//!   - Estimates gyro/accel biases from static initialization phase
//!   - Feeds into ESKF and optimization
//!   - Updated online through optimization refinement
//!
//! ## Key Architectural Decisions
//!
//! ### 1. **Preintegration is Primary**
//! Unlike loosely-coupled systems, preintegrated IMU measurements directly constrain
//! the optimization objective. Each preintegration block between keyframes appears as
//! factors in the bundle adjustment:
//!
//! ```text
//! Objective = image_reprojection_error
//!           + λ₁ * IMU_preintegration_error
//!           + λ₂ * smoothness_regularizers
//! ```
//!
//! ### 2. **Tight Visual-IMU Feedback**
//! The optimization loop refines both visual and inertial parameters:
//! - Visual: pose and feature positions
//! - Inertial: velocity at keyframes, gyro/accel biases
//!
//! Refined biases feed back into ESKF for high-rate velocity estimation.
//!
//! ### 3. **Gyro Integration for Robustness**
//! Even with visual odometry, gyro measurements provide:
//! - Continuous orientation tracking between visual updates
//! - Fallback when visual tracking fails (motion blur, low-texture)
//! - High-rate motion information for prediction
//!
//! ### 4. **Orientation from Visual System**
//! The ESKF receives orientation updates from visual odometry, ensuring:
//! - Proper transformation of accelerations to world frame
//! - Consistency between visual and inertial estimates
//! - Complementary sensor fusion
//!
//! ## Data Flow for Tight Coupling
//!
//! ```text
//! ┌─ INITIALIZATION (First 1-2 seconds)
//! │
//! ├─ ImuInitializer: Detect static, estimate biases
//! ├─ VisualInitializer: Compute initial orientation from features
//! └─ Initialize ESKF with bias estimates and orientation
//!
//! ┌─ TRACKING (Continuous)
//! │
//! ├─ PreintegratedImu: Accumulate IMU between keyframes
//! │  └─ Compute bias Jacobians for optimization
//! │
//! ├─ ESKF.predict(): High-rate motion estimate
//! │  ├─ Integrate gyro for continuous orientation
//! │  ├─ Integrate accel for velocity
//! │  └─ Track covariance growth
//! │
//! ├─ Visual subsystem: Feature tracking, optical flow
//! │  └─ Provide: orientation, position measurements, feature tracks
//! │
//! ├─ ESKF.update(): Correct from visual measurements
//! │  └─ Reduce uncertainty, correct drift
//! │
//! └─ Bundle Adjustment Optimization:
//!    ├─ Objective: Reproject errors + IMU factors + regularizers
//!    ├─ Variables: Poses, velocities, biases, features, extrinsics
//!    ├─ Uses: Preintegration Jacobians for bias correction
//!    └─ Outputs: Refined biases → back to ESKF
//!
//! ```
//!
//! ## References
//!
//! - **Forster et al.** "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
//!   IEEE Trans. Robotics 2017
//!
//! - **Solà et al.** "Quaternion kinematics for the error-state Kalman filter"
//!   arXiv 2017 - Error-state formulation and gyro integration
//!
//! - **Trawny & Roumeliotis** "Indirect Kalman Filter for 3D Attitude Estimation"
//!   MRISL 2005 - Indirect filtering foundations
//!
//! ## Critical Implementation Notes
//!
//! ### ✓ FIXED Issues
//! - **Noise covariance sign**: Multiply by dt (was dividing)
//! - **Variable timestamp handling**: Use actual measurement deltas
//! - **Orientation feedback**: Gyro integration + visual updates
//! - **Bias initialization**: Connected from ImuInitializer
//!
//! ### ⚠ Design Constraints
//! - IMU measurements must have precise timestamps (nanosecond resolution)
//! - Visual orientation updates must be synchronized with IMU
//! - Optimization must include IMU preintegration factors
//! - Biases must be fed back from optimization to ESKF
//!
//! Proper on-manifold integration:
//! - Forster et al. 2017
//! - Error-state Kalman Filter for velocity/bias estimation
//! - IMU measurement buffering and interpolation
//! - Robust initialization with bias estimation
//! - Online extrinsic and time offset calibration
//!
//! ## Architecture
//!
//! - **Preintegration**: Accumulates IMU between keyframes with covariance
//! - **ESKF**: Tracks velocity and biases with proper uncertainty
//! - **Buffer**: Handles out-of-order measurements and interpolation
//! - **Initialization**: Static period bias estimation

#[cfg(test)]
mod bias_feedback_tests;
pub mod buffer;
pub mod eskf;
pub mod initialization;
pub mod preintegration;

pub use buffer::ImuBuffer;
pub use eskf::{Eskf, EskfState};
pub use initialization::{
    AdaptiveNoiseEstimator, BiasEstimate, ImuInitializationConfig, ImuInitializer,
    InitializationState,
};
pub use preintegration::{ImuNoise, PreintegratedImu};

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

/// IMU motion prior for optimization
///
/// Provides motion constraints from preintegrated IMU measurements
/// to guide bundle adjustment optimization.
///
/// # Usage
///
/// ```rust,ignore
/// let prior = ImuMotionPrior::from_preintegration(preintegrated_imu, T_W_B_i);
/// // Add to BA problem as prior on T_W_B_j
/// ```
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
    pub const fn from_preintegration(
        preint: &PreintegratedImu,
        initial_pose: na::Matrix4<f64>,
        initial_velocity: na::Vector3<f64>,
        gravity: na::Vector3<f64>,
    ) -> Self {
        Self {
            delta_rotation: preint.delta_R,
            delta_velocity: preint.delta_v,
            delta_position: preint.delta_p,
            delta_time: preint.delta_t,
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
        let R_W_Bi =
            na::Rotation3::from_matrix(&self.initial_pose.fixed_view::<3, 3>(0, 0).into_owned());
        let R_W_Bj = R_W_Bi * self.delta_rotation.to_rotation_matrix();

        // Velocity prediction: delta_v is in body frame, rotate to world
        let R_W_Bi_mat = R_W_Bi.matrix();
        let v_W_Bj = self.initial_velocity
            + R_W_Bi_mat * self.delta_velocity
            + self.gravity * self.delta_time;

        // Position prediction: delta_p is in body frame, rotate to world
        let p_W_Bi = self.initial_pose.fixed_view::<3, 1>(0, 3).into_owned();
        let p_W_Bj = p_W_Bi
            + self.initial_velocity * self.delta_time
            + 0.5 * self.gravity * self.delta_time * self.delta_time
            + R_W_Bi_mat * self.delta_position;

        // Compose pose matrix
        let mut T_W_Bj = na::Matrix4::identity();
        T_W_Bj
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&R_W_Bj.into_inner());
        T_W_Bj.fixed_view_mut::<3, 1>(0, 3).copy_from(&p_W_Bj);

        (T_W_Bj, v_W_Bj)
    }

    /// Compute innovation (prediction error) given observed pose
    ///
    /// Returns (position_error, rotation_error, velocity_error)
    pub fn compute_innovation(
        &self,
        observed_pose: &na::Matrix4<f64>,
        observed_velocity: &na::Vector3<f64>,
    ) -> (na::Vector3<f64>, f64, na::Vector3<f64>) {
        let (predicted_pose, predicted_velocity) = self.predict_state();

        // Position innovation
        let pos_error = observed_pose.fixed_view::<3, 1>(0, 3).into_owned()
            - predicted_pose.fixed_view::<3, 1>(0, 3).into_owned();

        // Rotation innovation (angle-axis)
        let R_obs =
            na::Rotation3::from_matrix(&observed_pose.fixed_view::<3, 3>(0, 0).into_owned());
        let R_pred =
            na::Rotation3::from_matrix(&predicted_pose.fixed_view::<3, 3>(0, 0).into_owned());
        let dq = R_pred.inverse() * R_obs;
        let rot_error = dq.angle();

        // Velocity innovation
        let vel_error = observed_velocity - predicted_velocity;

        (pos_error, rot_error, vel_error)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_motion_prior() {
        let noise = ImuNoise::default();
        let preint = PreintegratedImu::new(noise);
        let pose = na::Matrix4::identity();
        let velocity = na::Vector3::zeros();
        let gravity = na::Vector3::new(0.0, 0.0, -9.81);

        let prior = ImuMotionPrior::from_preintegration(&preint, pose, velocity, gravity);
        let (pred_pose, pred_vel) = prior.predict_state();

        // Should be close to initial for zero IMU motion
        assert!((pred_pose - pose).abs().sum() < 1e-10);
        assert!((pred_vel - velocity).norm() < 1e-10);
    }

    #[test]
    fn test_imu_motion_prior_innovation() {
        let noise = ImuNoise::default();
        let mut preint = PreintegratedImu::new(noise);
        preint.delta_R = na::UnitQuaternion::identity();
        preint.delta_v = na::Vector3::new(0.1, 0.0, 0.0);
        preint.delta_p = na::Vector3::zeros();
        preint.delta_t = 1.0;

        let pose = na::Matrix4::identity();
        let velocity = na::Vector3::zeros();
        let gravity = na::Vector3::zeros();

        let prior = ImuMotionPrior::from_preintegration(&preint, pose, velocity, gravity);
        let obs_pose = na::Matrix4::identity();
        let obs_vel = na::Vector3::zeros();

        let (pos_err, rot_err, vel_err) = prior.compute_innovation(&obs_pose, &obs_vel);

        // Position error should be small (no delta position)
        assert!(pos_err.norm() < 1e-10);
        // Rotation error should be small
        assert!(rot_err.abs() < 1e-10);
        // Velocity error should be ~0.1 (predicted has velocity, observed doesn't)
        assert!((vel_err - na::Vector3::new(-0.1, 0.0, 0.0)).norm() < 1e-10);
    }
}
