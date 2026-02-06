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

pub mod batch_processing;
pub mod bias_feedback_tests;
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

use crate::datasets::ImuData;
use crate::fl;
use crate::types::Float;
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

/// IMU-aided keyframe selection based on visual-inertial motion
///
/// Estimates gyroscope and accelerometer biases from IMU measurements during
/// initialization (when the system is assumed to be stationary).
///
/// The estimator uses a simple least-squares approach:
/// - Gyro bias = mean(angular_velocity)  (assuming zero rotation)
/// - Accel bias = mean(accelerometer) - gravity  (assuming zero acceleration)
#[derive(Debug, Clone)]
pub struct ImuBiasEstimator {
    /// Configuration
    config: ImuConfig,
    /// Accumulated gyroscope measurements for bias estimation
    gyro_samples: Vec<na::Vector3<f64>>,
    /// Accumulated accelerometer measurements for bias estimation
    accel_samples: Vec<na::Vector3<f64>>,
    /// Timestamps for each sample
    timestamps: Vec<i64>,
    /// Estimated gyroscope bias [rad/s]
    pub gyro_bias: na::Vector3<f64>,
    /// Estimated accelerometer bias [m/s²]
    pub accel_bias: na::Vector3<f64>,
    /// Whether the estimator has collected enough samples
    pub is_initialized: bool,
    /// Number of samples required for initialization
    min_samples: usize,
}

impl ImuBiasEstimator {
    /// Create new bias estimator
    pub fn new(config: ImuConfig) -> Self {
        Self {
            config,
            gyro_samples: Vec::with_capacity(1000),
            accel_samples: Vec::with_capacity(1000),
            timestamps: Vec::with_capacity(1000),
            gyro_bias: na::Vector3::zeros(),
            accel_bias: na::Vector3::zeros(),
            is_initialized: false,
            min_samples: 100, // At 200Hz, this is 0.5 seconds
        }
    }

    /// Add IMU measurement for bias estimation
    ///
    /// # Arguments
    /// * `imu` - IMU measurement
    /// * `_assume_stationary` - If true, assume system is stationary (use for initialization)
    #[allow(dead_code)]
    pub fn add_sample(&mut self, imu: &ImuData, _assume_stationary: bool) {
        let gyro = na::Vector3::new(imu.gyro[0], imu.gyro[1], imu.gyro[2]);
        let accel = na::Vector3::new(imu.accel[0], imu.accel[1], imu.accel[2]);

        self.gyro_samples.push(gyro);
        self.accel_samples.push(accel);
        self.timestamps.push(imu.timestamp);

        // Check if we have enough samples and time window
        if !self.is_initialized {
            self.update_bias_estimate();
        }
    }

    /// Update bias estimates from accumulated samples
    fn update_bias_estimate(&mut self) {
        if self.timestamps.is_empty() {
            return;
        }

        // Need enough samples
        if self.gyro_samples.len() < self.min_samples {
            return;
        }

        // Estimate gyro bias: mean of measurements (assuming stationary = zero rotation)
        let mut gyro_sum = na::Vector3::zeros();
        for g in &self.gyro_samples {
            gyro_sum += g;
        }
        self.gyro_bias = gyro_sum / self.gyro_samples.len() as f64;

        // Estimate accel bias: mean - gravity (assuming stationary = gravity only)
        let gravity = na::Vector3::new(
            self.config.gravity[0],
            self.config.gravity[1],
            self.config.gravity[2],
        );
        let mut accel_sum = na::Vector3::zeros();
        for a in &self.accel_samples {
            accel_sum += a;
        }
        let mean_accel = accel_sum / self.accel_samples.len() as f64;
        self.accel_bias = mean_accel - gravity;

        self.is_initialized = true;
    }

    /// Get bias-corrected gyroscope measurement
    pub fn correct_gyro(&self, imu: &ImuData) -> na::Vector3<f64> {
        na::Vector3::new(
            imu.gyro[0] - self.gyro_bias[0],
            imu.gyro[1] - self.gyro_bias[1],
            imu.gyro[2] - self.gyro_bias[2],
        )
    }

    /// Get bias-corrected accelerometer measurement
    pub fn correct_accel(&self, imu: &ImuData) -> na::Vector3<f64> {
        na::Vector3::new(
            imu.accel[0] - self.accel_bias[0],
            imu.accel[1] - self.accel_bias[1],
            imu.accel[2] - self.accel_bias[2],
        )
    }

    /// Reset the bias estimator (e.g., after motion restart)
    pub fn reset(&mut self) {
        self.gyro_samples.clear();
        self.accel_samples.clear();
        self.timestamps.clear();
        self.gyro_bias = na::Vector3::zeros();
        self.accel_bias = na::Vector3::zeros();
        self.is_initialized = false;
    }

    /// Get number of samples collected
    pub fn sample_count(&self) -> usize {
        self.gyro_samples.len()
    }
}

/// Online IMU-camera extrinsic calibration
pub struct ExtrinsicCalibrator {
    /// Current estimate of T_BC (body to camera transform)
    pub T_BC: na::Matrix4<Float>,
    /// Optimization state
    accumulated_rotations: Vec<na::UnitQuaternion<Float>>,
    accumulated_cam_poses: Vec<na::Matrix4<Float>>,
    iterations: usize,
}

impl ExtrinsicCalibrator {
    /// Create new calibrator with initial guess
    pub fn new(initial_T_BC: na::Matrix4<Float>) -> Self {
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
        camera_pose: &na::Matrix4<Float>,
        rotation_quaternion: na::UnitQuaternion<Float>,
    ) {
        self.accumulated_cam_poses.push(*camera_pose);
        self.accumulated_rotations.push(rotation_quaternion);
    }

    /// Run one iteration of extrinsic calibration
    ///
    /// Minimizes: Σ || q(BC_i) ⊗ q(CB) - q(WC_i) ⊗ q(CW_{i-1}) ||²
    pub fn calibrate_iteration(&mut self) -> Float {
        if self.accumulated_rotations.len() < 10 {
            return fl!(0.0);
        }

        // Simple iterative refinement of rotation
        let mut total_correction = na::Vector3::zeros();

        for i in 1..self.accumulated_rotations.len() {
            let dq = self.accumulated_rotations[i - 1].inverse() * self.accumulated_rotations[i];

            // Extract rotation axis-angle
            let angle = fl!(2.0) * dq.i.atan2(dq.w);
            let axis = na::Vector3::new(dq.i, dq.j, dq.k) / (dq.w + fl!(1e-10)).sqrt();
            if angle.is_finite() {
                total_correction += axis * angle;
            }
        }

        let avg_correction = total_correction / fl!(self.accumulated_rotations.len() as f64 - 1.0);

        // Apply small correction
        let correction_rot = na::UnitQuaternion::new(avg_correction * fl!(0.1));
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
    pub fn get_extrinsics(&self) -> na::Matrix4<Float> {
        self.T_BC
    }

    /// Number of accumulated measurements
    pub fn measurement_count(&self) -> usize {
        self.accumulated_rotations.len()
    }
}

/// IMU-aided keyframe selection based on visual-inertial motion
///
/// Uses IMU measurements to determine when sufficient motion has occurred
/// for a new keyframe, combining both visual and inertial cues.
///
/// # Algorithm
///
/// Keyframe triggered when:
/// - Translation since last keyframe > threshold OR
/// - Rotation since last keyframe > threshold OR
/// - IMU delta rotation significantly differs from visual rotation (indicates tracking issues)
///
/// This is more robust than pure visual keyframe selection as IMU provides
/// motion information even in low-texture scenes.
#[derive(Debug, Clone)]
pub struct ImuAidedKeyframeSelector {
    /// Translation threshold for keyframe [m]
    translation_threshold: Float,
    /// Rotation threshold for keyframe [rad]
    rotation_threshold: Float,
    /// Last keyframe timestamp
    last_keyframe_timestamp: Option<i64>,
    /// Last keyframe pose (T_W_B)
    last_keyframe_pose: Option<na::Matrix4<Float>>,
    /// IMU delta rotation from last keyframe
    imu_delta_rotation: na::UnitQuaternion<Float>,
    /// IMU delta translation from last keyframe
    imu_delta_translation: na::Vector3<Float>,
    /// IMU delta time from last keyframe
    imu_delta_time: Float,
}

impl ImuAidedKeyframeSelector {
    /// Create new selector with thresholds
    pub fn new(translation_threshold: Float, rotation_threshold: Float) -> Self {
        Self {
            translation_threshold,
            rotation_threshold,
            last_keyframe_timestamp: None,
            last_keyframe_pose: None,
            imu_delta_rotation: na::UnitQuaternion::identity(),
            imu_delta_translation: na::Vector3::zeros(),
            imu_delta_time: fl!(0.0),
        }
    }

    /// Reset selector (e.g., after loop closure)
    pub fn reset(&mut self) {
        self.last_keyframe_timestamp = None;
        self.last_keyframe_pose = None;
        self.imu_delta_rotation = na::UnitQuaternion::identity();
        self.imu_delta_translation = na::Vector3::zeros();
        self.imu_delta_time = fl!(0.0);
    }

    /// Update with single IMU measurement (with timestamp-based dt)
    ///
    /// **Deprecated**: Use `accumulate_imu` instead for proper timestamp handling
    #[deprecated(note = "Use accumulate_imu for proper timestamp-based integration")]
    pub fn update_imu(&mut self, imu: &ImuData) {
        // For backward compatibility, estimate dt from timestamps
        let dt = if let Some(last_ts) = self.last_keyframe_timestamp {
            fl!((imu.timestamp - last_ts) as f64 / 1e9)
        } else {
            fl!(0.01) // Fallback: assume 100Hz
        };

        let gyro = na::Vector3::new(fl!(imu.gyro[0]), fl!(imu.gyro[1]), fl!(imu.gyro[2]));
        let accel = na::Vector3::new(fl!(imu.accel[0]), fl!(imu.accel[1]), fl!(imu.accel[2]));

        // Integrate rotation using proper dt
        let delta_rot = na::UnitQuaternion::new(gyro * dt);
        self.imu_delta_rotation = delta_rot * self.imu_delta_rotation;

        // Integrate translation
        self.imu_delta_translation += accel * dt * dt * fl!(0.5);
        self.imu_delta_time += dt;
    }

    /// Accumulate IMU measurements between frames
    pub fn accumulate_imu(&mut self, imu_measurements: &[ImuData]) {
        if imu_measurements.is_empty() {
            return;
        }

        let mut last_ts = imu_measurements[0].timestamp;
        let mut integrated_rot = na::UnitQuaternion::identity();
        let mut integrated_trans = na::Vector3::zeros();
        let mut integrated_vel = na::Vector3::zeros(); // Track velocity for accurate displacement

        for imu in imu_measurements {
            let dt = fl!((imu.timestamp - last_ts) as f64 / 1e9);
            if dt > fl!(0.0) {
                let gyro = na::Vector3::new(fl!(imu.gyro[0]), fl!(imu.gyro[1]), fl!(imu.gyro[2]));
                let accel =
                    na::Vector3::new(fl!(imu.accel[0]), fl!(imu.accel[1]), fl!(imu.accel[2]));

                // Rotation integration using exponential map (small angle approximation)
                let delta_rot = na::UnitQuaternion::new(gyro * dt);
                integrated_rot = delta_rot * integrated_rot;

                // Velocity integration: v_new = v_old + a * dt
                // Tracks cumulative velocity throughout the measurement window
                integrated_vel += accel * dt;

                // Displacement integration using trapezoid rule: p += (v_prev + v_curr) / 2 * dt
                // Equivalent to: p += v_curr * dt + 0.5 * a * dt^2
                // This properly accounts for continuous acceleration between frames, not just rest
                integrated_trans += integrated_vel * dt - accel * dt * dt * fl!(0.5);

                self.imu_delta_time += dt;
            }
            last_ts = imu.timestamp;
        }

        self.imu_delta_rotation = integrated_rot;
        self.imu_delta_translation = integrated_trans;
    }

    /// Determine if new keyframe should be created
    ///
    /// # Arguments
    /// * `current_pose` - Current camera pose (T_W_B)
    /// * `current_timestamp` - Current frame timestamp
    /// * `imu_rotation_deviation` - Deviation between IMU and visual rotation [rad]
    ///
    /// # Returns
    /// (should_be_keyframe, motion_info)
    pub fn should_be_keyframe(
        &mut self,
        current_pose: &na::Matrix4<Float>,
        current_timestamp: i64,
        imu_rotation_deviation: Float,
    ) -> (bool, String) {
        let mut is_keyframe = false;
        let mut reason = String::new();

        // Compute visual motion since last keyframe
        if let Some(last_pose) = self.last_keyframe_pose {
            let T_rel = last_pose.try_inverse().unwrap() * current_pose;
            let t_rel = T_rel.fixed_view::<3, 1>(0, 3).into_owned();
            let r_rel = T_rel.fixed_view::<3, 3>(0, 0);
            let rotmat = na::Rotation3::from_matrix_unchecked(r_rel.into_owned());
            let euler: (Float, Float, Float) = rotmat.euler_angles();

            let translation_norm = t_rel.norm();
            let rotation_norm = (euler.0.abs() + euler.1.abs() + euler.2.abs()).abs();

            // Check thresholds
            let visual_trigger = translation_norm > self.translation_threshold
                || rotation_norm > self.rotation_threshold;

            // IMU-aided triggers
            let imu_translation_norm = self.imu_delta_translation.norm();
            let imu_rotation_angle = self.imu_delta_rotation.angle();

            // If visual motion is small but IMU shows significant motion, trigger keyframe
            let imu_motion_trigger = imu_translation_norm > self.translation_threshold * fl!(0.5)
                || imu_rotation_angle.abs() > self.rotation_threshold * fl!(0.5);

            // If IMU and visual disagree significantly,可能有跟踪问题
            let imu_visual_disagree = imu_rotation_deviation > self.rotation_threshold * fl!(2.0);

            if visual_trigger || imu_motion_trigger || imu_visual_disagree {
                is_keyframe = true;
                if visual_trigger {
                    reason = format!(
                        "Visual motion: trans={:.3}m, rot={:.3}rad",
                        translation_norm, rotation_norm
                    );
                } else if imu_motion_trigger {
                    reason = format!(
                        "IMU motion: trans={:.3}m, rot={:.3}rad",
                        imu_translation_norm,
                        imu_rotation_angle.abs()
                    );
                } else {
                    reason = format!("IMU-visual disagreement: {:.3}rad", imu_rotation_deviation);
                }
            }
        } else {
            // First frame after initialization is always a keyframe
            is_keyframe = true;
            reason = "Initial frame".to_string();
        }

        // Update state if keyframe
        if is_keyframe {
            self.last_keyframe_timestamp = Some(current_timestamp);
            self.last_keyframe_pose = Some(*current_pose);
            self.imu_delta_rotation = na::UnitQuaternion::identity();
            self.imu_delta_translation = na::Vector3::zeros();
            self.imu_delta_time = fl!(0.0);
        }

        (is_keyframe, reason)
    }
}

/// Visual measurement update for tight visual-inertial coupling
///
/// Provides orientation (and optionally velocity) from the visual subsystem
/// to correct the ESKF state.
#[derive(Debug, Clone)]
pub struct VisualMeasurement {
    /// Visual orientation (world-from-body)
    pub orientation: na::UnitQuaternion<f64>,
    /// Visual velocity estimate in world frame (optional)
    pub velocity: Option<na::Vector3<f64>>,
    /// Velocity measurement covariance (optional)
    pub velocity_covariance: Option<na::Matrix3<f64>>,
    /// Measurement timestamp (optional)
    pub timestamp: Option<i64>,
}

impl VisualMeasurement {
    pub fn orientation_only(orientation: na::UnitQuaternion<f64>) -> Self {
        Self {
            orientation,
            velocity: None,
            velocity_covariance: None,
            timestamp: None,
        }
    }

    pub fn with_velocity(
        orientation: na::UnitQuaternion<f64>,
        velocity: na::Vector3<f64>,
        velocity_covariance: na::Matrix3<f64>,
    ) -> Self {
        Self {
            orientation,
            velocity: Some(velocity),
            velocity_covariance: Some(velocity_covariance),
            timestamp: None,
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
    pub fn from_preintegration(
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

    /// Create from legacy preintegrated measurements (compatibility)
    pub fn from_legacy_preintegration(
        preint: &LegacyPreintegratedImu,
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
        let R_obs = na::Rotation3::from_matrix_unchecked(
            observed_pose.fixed_view::<3, 3>(0, 0).into_owned(),
        );
        let R_pred = na::Rotation3::from_matrix_unchecked(
            predicted_pose.fixed_view::<3, 3>(0, 0).into_owned(),
        );
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
    fn test_imu_aided_keyframe_selector() {
        let mut selector = ImuAidedKeyframeSelector::new(0.1, 0.1);

        // First frame should be keyframe
        let pose = na::Matrix4::identity();
        let (is_kf, reason) = selector.should_be_keyframe(&pose, 1000000000, 0.0);
        assert!(is_kf);
        assert_eq!(reason, "Initial frame");

        // Small motion shouldn't trigger keyframe
        let small_motion = na::Matrix4::new_translation(&na::Vector3::new(0.01, 0.0, 0.0));
        let (is_kf, _) = selector.should_be_keyframe(&small_motion, 1000001000, 0.01);
        assert!(!is_kf);

        // Large motion should trigger keyframe
        let large_motion = na::Matrix4::new_translation(&na::Vector3::new(0.2, 0.0, 0.0));
        let (is_kf, reason) = selector.should_be_keyframe(&large_motion, 1000002000, 0.01);
        assert!(is_kf);
        assert!(reason.contains("Visual motion"));
    }

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
        estimator.initialize_from_bias_and_orientation(&imu_measurements, &orientation, None);

        assert!(estimator.is_initialized());
    }

    #[test]
    fn test_visual_update_corrects_velocity() {
        let config = ImuConfig::default();
        let mut estimator = VelocityEstimator::new(config);

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
        estimator.initialize_from_bias_and_orientation(&imu_measurements, &orientation, None);

        let before = estimator.get_velocity();
        let measured = na::Vector3::new(1.0, 0.0, 0.0);
        let covariance = na::Matrix3::identity() * 1e-4;

        estimator.update_velocity_from_visual(measured, covariance);
        let after = estimator.get_velocity();

        assert!((after - measured).norm() < (before - measured).norm());
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

    #[test]
    fn test_imu_bias_estimator() {
        let config = ImuConfig::default();
        let mut bias_estimator = ImuBiasEstimator::new(config);

        // Initially not initialized
        assert!(!bias_estimator.is_initialized);
        assert_eq!(bias_estimator.sample_count(), 0);

        // Add some IMU samples with known biases
        // Simulate: gyro bias = [0.01, -0.02, 0.005], accel bias = [0.05, -0.03, 0.1]
        let gyro_bias = [0.01, -0.02, 0.005];
        let accel_bias = [0.05, -0.03, 0.1];
        let gravity = -9.81;

        for i in 0..200 {
            let ts = (i * 5000) as i64; // 200Hz
            let imu = ImuData {
                timestamp: ts,
                gyro: [
                    0.001 + gyro_bias[0], // Small motion + bias
                    -0.002 + gyro_bias[1],
                    0.001 + gyro_bias[2],
                ],
                accel: [
                    0.02 + accel_bias[0], // Small acceleration + bias
                    -0.01 + accel_bias[1],
                    gravity + accel_bias[2],
                ],
            };
            bias_estimator.add_sample(&imu, true);
        }

        // Should now be initialized
        assert!(bias_estimator.is_initialized);
        assert_eq!(bias_estimator.sample_count(), 200);

        // Check bias estimates (should be close to true biases, accounting for small motion)
        assert!((bias_estimator.gyro_bias[0] - gyro_bias[0] - 0.001).abs() < 0.005);
        assert!((bias_estimator.gyro_bias[1] - gyro_bias[1] + 0.002).abs() < 0.005);
        assert!((bias_estimator.gyro_bias[2] - gyro_bias[2] - 0.001).abs() < 0.005);

        assert!((bias_estimator.accel_bias[0] - accel_bias[0] - 0.02).abs() < 0.05);
        assert!((bias_estimator.accel_bias[1] - accel_bias[1] + 0.01).abs() < 0.05);
        assert!((bias_estimator.accel_bias[2] - accel_bias[2]).abs() < 0.05);

        // Test bias correction - should remove the known bias
        let test_imu = ImuData {
            timestamp: 0,
            gyro: [gyro_bias[0] + 0.1, gyro_bias[1] + 0.1, gyro_bias[2] + 0.1],
            accel: [
                accel_bias[0] + 1.0,
                accel_bias[1] + 1.0,
                accel_bias[2] + 1.0, // No gravity - bias estimator handles that
            ],
        };

        let corrected_gyro = bias_estimator.correct_gyro(&test_imu);
        // Corrected value should be approximately 0.1 (the added motion, minus small bias estimation error)
        assert!(
            (corrected_gyro[0] - 0.1).abs() < 0.02,
            "gyro x: {} vs expected 0.1",
            corrected_gyro[0]
        );
        assert!(
            (corrected_gyro[1] - 0.1).abs() < 0.02,
            "gyro y: {} vs expected 0.1",
            corrected_gyro[1]
        );
        assert!(
            (corrected_gyro[2] - 0.1).abs() < 0.02,
            "gyro z: {} vs expected 0.1",
            corrected_gyro[2]
        );

        let corrected_accel = bias_estimator.correct_accel(&test_imu);
        // Corrected value should be approximately 1.0 (the added acceleration, minus small bias estimation error)
        assert!(
            (corrected_accel[0] - 1.0).abs() < 0.1,
            "accel x: {} vs expected 1.0",
            corrected_accel[0]
        );
        assert!(
            (corrected_accel[1] - 1.0).abs() < 0.1,
            "accel y: {} vs expected 1.0",
            corrected_accel[1]
        );
        assert!(
            (corrected_accel[2] - 1.0).abs() < 0.1,
            "accel z: {} vs expected 1.0",
            corrected_accel[2]
        );

        // Test reset
        bias_estimator.reset();
        assert!(!bias_estimator.is_initialized);
        assert_eq!(bias_estimator.sample_count(), 0);
    }
}

/// Time offset calibration between IMU and camera
///
/// Estimates the time offset τ such that: t_cam = t_imu + τ
#[derive(Debug, Clone)]
pub struct TimeOffsetCalibrator {
    /// Current time offset estimate [nanoseconds]
    pub time_offset: i64,

    /// Uncertainty in time offset [nanoseconds]
    pub time_offset_std: f64,

    /// Measurements for calibration
    observations: Vec<(i64, i64, f64)>, // (t_cam, t_imu, correlation)

    /// Maximum observations to keep
    max_observations: usize,
}

impl TimeOffsetCalibrator {
    /// Create new time offset calibrator
    pub fn new() -> Self {
        Self {
            time_offset: 0,
            time_offset_std: 1e6, // 1ms initial uncertainty
            observations: Vec::new(),
            max_observations: 100,
        }
    }

    /// Add observation from visual-inertial correlation
    ///
    /// # Arguments
    /// * `t_cam` - Camera frame timestamp
    /// * `t_imu` - IMU measurement timestamp
    /// * `correlation` - Correlation score (higher = better match)
    pub fn add_observation(&mut self, t_cam: i64, t_imu: i64, correlation: f64) {
        self.observations.push((t_cam, t_imu, correlation));

        if self.observations.len() > self.max_observations {
            // Remove lowest correlation observation
            let min_idx = self
                .observations
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.2.partial_cmp(&b.2).unwrap())
                .map(|(idx, _)| idx)
                .unwrap();
            self.observations.remove(min_idx);
        }

        // Update estimate
        self.estimate_offset();
    }

    /// Estimate time offset from observations
    fn estimate_offset(&mut self) {
        if self.observations.is_empty() {
            return;
        }

        // Weighted average by correlation
        let total_weight: f64 = self.observations.iter().map(|(_, _, c)| c).sum();

        if total_weight < 1e-6 {
            return;
        }

        let weighted_offset: i64 = self
            .observations
            .iter()
            .map(|(t_cam, t_imu, corr)| {
                let offset = t_cam - t_imu;
                (offset as f64 * corr) as i64
            })
            .sum();

        self.time_offset = (weighted_offset as f64 / total_weight) as i64;

        // Estimate uncertainty from variance
        let variance: f64 = self
            .observations
            .iter()
            .map(|(t_cam, t_imu, corr)| {
                let offset = (t_cam - t_imu) as f64;
                let deviation = offset - self.time_offset as f64;
                deviation * deviation * corr
            })
            .sum::<f64>()
            / total_weight;

        self.time_offset_std = variance.sqrt();
    }

    /// Apply time offset correction to IMU timestamp
    pub fn correct_imu_time(&self, t_imu: i64) -> i64 {
        t_imu + self.time_offset
    }

    /// Get current offset estimate [seconds]
    pub fn get_offset_seconds(&self) -> f64 {
        self.time_offset as f64 / 1e9
    }

    /// Check if calibration has converged
    pub fn is_converged(&self) -> bool {
        self.observations.len() >= 20 && self.time_offset_std < 1e6 // < 1ms
    }
}

impl Default for TimeOffsetCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

// ==============================================================================
// Legacy Compatibility Wrappers
// ==============================================================================

/// Legacy wrapper for ImuPreintegrator (maintains old API)
///
/// **Note**: This is a compatibility shim. New code should use `PreintegratedImu` directly.
pub struct ImuPreintegrator {
    preint: PreintegratedImu,
    #[allow(dead_code)]
    config: ImuConfig,
    last_gyro: na::Vector3<f64>,
    last_accel: na::Vector3<f64>,
    has_measurement: bool,
}

impl ImuPreintegrator {
    pub fn new(config: ImuConfig) -> Self {
        let noise = ImuNoise {
            gyro_noise_density: config.gyro_noise_density,
            accel_noise_density: config.accel_noise_density,
            gyro_bias_random_walk: config.gyro_bias_random_walk,
            accel_bias_random_walk: config.accel_bias_random_walk,
        };

        Self {
            preint: PreintegratedImu::new(noise),
            config,
            last_gyro: na::Vector3::zeros(),
            last_accel: na::Vector3::zeros(),
            has_measurement: false,
        }
    }

    pub fn propagate(&mut self, imu: &ImuData, dt: f64) {
        let gyro = na::Vector3::new(imu.gyro[0] as f64, imu.gyro[1] as f64, imu.gyro[2] as f64);
        let accel = na::Vector3::new(
            imu.accel[0] as f64,
            imu.accel[1] as f64,
            imu.accel[2] as f64,
        );

        if self.has_measurement {
            // Use midpoint
            let gyro_mid = (self.last_gyro + gyro) * 0.5;
            let accel_mid = (self.last_accel + accel) * 0.5;
            self.preint.integrate(gyro_mid, accel_mid, dt);
        }

        self.last_gyro = gyro;
        self.last_accel = accel;
        self.has_measurement = true;
    }

    pub fn propagate_corrected(
        &mut self,
        gyro_corrected: na::Vector3<f64>,
        accel_corrected: na::Vector3<f64>,
        dt: f64,
    ) {
        if self.has_measurement {
            let gyro_mid = (self.last_gyro + gyro_corrected) * 0.5;
            let accel_mid = (self.last_accel + accel_corrected) * 0.5;
            self.preint.integrate(gyro_mid, accel_mid, dt);
        }

        self.last_gyro = gyro_corrected;
        self.last_accel = accel_corrected;
        self.has_measurement = true;
    }

    pub fn get(&self) -> LegacyPreintegratedImu {
        // Convert SMatrix<9,9> to DMatrix
        let cov_dmat = na::DMatrix::from_fn(9, 9, |i, j| self.preint.covariance[(i, j)]);

        LegacyPreintegratedImu {
            delta_rotation: na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                self.preint.delta_R.w,
                self.preint.delta_R.i,
                self.preint.delta_R.j,
                self.preint.delta_R.k,
            )),
            delta_velocity: self.preint.delta_v,
            delta_position: self.preint.delta_p,
            delta_time: self.preint.delta_t,
            covariance: cov_dmat,
        }
    }

    pub fn reset(&mut self) {
        self.preint
            .reset(na::Vector3::zeros(), na::Vector3::zeros());
        self.has_measurement = false;
    }

    /// Take the current preintegration and reset for the next keyframe segment
    ///
    /// Resets the internal preintegration with the provided bias linearization point.
    pub fn take_preintegration(
        &mut self,
        gyro_bias: na::Vector3<f64>,
        accel_bias: na::Vector3<f64>,
    ) -> PreintegratedImu {
        let preint = self.preint.clone();
        self.preint.reset(gyro_bias, accel_bias);
        self.has_measurement = false;
        preint
    }
}

/// Legacy PreintegratedImu structure
#[derive(Debug, Clone)]
pub struct LegacyPreintegratedImu {
    pub delta_rotation: na::UnitQuaternion<f64>,
    pub delta_velocity: na::Vector3<f64>,
    pub delta_position: na::Vector3<f64>,
    pub delta_time: f64,
    pub covariance: na::DMatrix<f64>,
}

/// Legacy velocity estimator wrapper
pub struct VelocityEstimator {
    eskf: Eskf,
}

impl VelocityEstimator {
    pub fn new(config: ImuConfig) -> Self {
        let noise = ImuNoise {
            gyro_noise_density: config.gyro_noise_density,
            accel_noise_density: config.accel_noise_density,
            gyro_bias_random_walk: config.gyro_bias_random_walk,
            accel_bias_random_walk: config.accel_bias_random_walk,
        };

        let gravity = na::Vector3::new(config.gravity[0], config.gravity[1], config.gravity[2]);

        Self {
            eskf: Eskf::new(noise, gravity),
        }
    }

    /// Update velocity estimator with IMU measurements
    ///
    /// Uses actual timestamp deltas from measurements rather than constant dt parameter.
    /// Processes measurements in order, computing dt from timestamp differences.
    pub fn update(&mut self, imu_measurements: &[ImuData]) {
        if imu_measurements.is_empty() {
            return;
        }

        // Process measurements with actual timestamp spacing
        let mut prev_timestamp = imu_measurements[0].timestamp;

        for imu in &imu_measurements[1..] {
            // Compute dt from actual timestamp delta (nanoseconds to seconds)
            let dt_ns = (imu.timestamp - prev_timestamp) as f64;
            let dt_s = dt_ns * 1e-9;

            // Sanity check on dt (should be between 1ms and 100ms for typical IMU)
            if dt_s > 0.0 && dt_s < 0.1 {
                self.eskf.predict(imu, dt_s);
            } else if dt_s <= 0.0 {
                eprintln!("Warning: Out-of-order or duplicate IMU timestamps detected");
            } else {
                eprintln!(
                    "Warning: Very large IMU timestamp gap: {:.3}s (measurement skipped)",
                    dt_s
                );
            }

            prev_timestamp = imu.timestamp;
        }
    }

    /// Legacy update method (deprecated, use new update signature)
    #[deprecated(
        since = "0.2.0",
        note = "Use update(&imu_measurements) without dt parameter"
    )]
    pub fn update_legacy(&mut self, imu_measurements: &[ImuData], _dt: f64) {
        self.update(imu_measurements);
    }

    pub fn is_initialized(&self) -> bool {
        self.eskf.state.timestamp.is_some()
    }

    /// Apply refined bias estimates from optimization
    ///
    /// This implements the feedback loop from bundle adjustment optimization
    /// back to the ESKF. Refined biases improve future IMU predictions.
    ///
    /// # Arguments
    ///
    /// * `gyro_bias` - Optimized gyro bias [rad/s]
    /// * `accel_bias` - Optimized accel bias [m/s²]
    /// * `uncertainty` - Bias uncertainty (standard deviation) from optimization
    ///
    /// # Example
    ///
    /// ```ignore
    /// // After optimization completes
    /// let (optimized_bg, optimized_ba) = optimizer.get_refined_biases();
    /// velocity_estimator.apply_optimized_biases(optimized_bg, optimized_ba, 0.001);
    /// ```
    pub fn apply_optimized_biases(
        &mut self,
        gyro_bias: na::Vector3<f64>,
        accel_bias: na::Vector3<f64>,
        uncertainty: f64,
    ) {
        self.eskf
            .apply_bias_correction(gyro_bias, accel_bias, uncertainty);
    }

    /// Get current bias estimates
    ///
    /// Returns (gyro_bias, accel_bias) in rad/s and m/s² respectively
    pub fn get_biases(&self) -> (na::Vector3<f64>, na::Vector3<f64>) {
        self.eskf.get_biases()
    }

    /// Get current velocity estimate
    pub fn get_velocity(&self) -> na::Vector3<f64> {
        self.eskf.get_velocity()
    }

    /// Get velocity uncertainty (standard deviation)
    pub fn get_velocity_uncertainty(&self) -> na::Vector3<f64> {
        self.eskf.get_velocity_std()
    }

    /// Update orientation from visual measurement
    pub fn update_orientation_from_visual(&mut self, orientation: na::UnitQuaternion<f64>) {
        self.eskf.update_orientation(orientation);
    }

    /// Update velocity from visual measurement
    pub fn update_velocity_from_visual(
        &mut self,
        velocity: na::Vector3<f64>,
        covariance: na::Matrix3<f64>,
    ) {
        self.eskf.update_velocity(velocity, covariance);
    }

    /// Apply a full visual measurement update (orientation + optional velocity)
    pub fn update_from_visual(&mut self, measurement: &VisualMeasurement) {
        self.eskf.update_orientation(measurement.orientation);
        if let Some(velocity) = measurement.velocity {
            let covariance = measurement
                .velocity_covariance
                .unwrap_or_else(|| na::Matrix3::identity() * 0.25);
            self.eskf.update_velocity(velocity, covariance);
        }
    }

    /// Initialize velocity estimator with bias estimates and orientation
    ///
    /// For tight visual-inertial coupling, this sets up the ESKF with:
    /// - Initial orientation from visual system
    /// - Gyro and accel biases estimated during static phase
    /// - Initial covariance based on bias estimation uncertainty
    pub fn initialize_from_bias_and_orientation(
        &mut self,
        imu_measurements: &[ImuData],
        initial_orientation: &na::UnitQuaternion<f64>,
        bias_estimate: Option<&BiasEstimate>,
    ) {
        // Set initial orientation from visual system
        self.eskf.update_orientation(*initial_orientation);

        // Apply bias estimates from initialization phase if available
        if let Some(bias) = bias_estimate {
            self.eskf.state.gyro_bias = bias.gyro_bias;
            self.eskf.state.accel_bias = bias.accel_bias;

            // Set covariance based on bias estimation quality
            self.eskf
                .state
                .covariance
                .fixed_view_mut::<3, 3>(3, 3)
                .fill_diagonal(bias.gyro_bias_std.powi(2));
            self.eskf
                .state
                .covariance
                .fixed_view_mut::<3, 3>(6, 6)
                .fill_diagonal(bias.accel_bias_std.powi(2));
        }

        // Initialize timestamp from first measurement
        if let Some(first) = imu_measurements.first() {
            self.eskf.state.timestamp = Some(first.timestamp);
        }
    }

    /// Legacy initialization method (deprecated, use initialize_from_bias_and_orientation)
    #[deprecated(
        since = "0.2.0",
        note = "Use initialize_from_bias_and_orientation instead"
    )]
    pub fn initialize_from_imu(
        &mut self,
        imu_measurements: &[ImuData],
        initial_orientation: &na::UnitQuaternion<f64>,
    ) {
        self.initialize_from_bias_and_orientation(imu_measurements, initial_orientation, None);
    }
}

/// Legacy motion predictor (minimal implementation)
pub struct ImuMotionPredictor {
    _config: ImuConfig,
}

impl ImuMotionPredictor {
    pub fn new(config: ImuConfig) -> Self {
        Self { _config: config }
    }

    pub fn predict_feature_displacement(
        &self,
        imu_measurements: &[ImuData],
        _prev_pixel: (f64, f64),
        focal_length: f64,
    ) -> (f64, f64) {
        if imu_measurements.is_empty() {
            return (0.0, 0.0);
        }

        // Compute IMU-based camera motion estimate using proper velocity integration
        let mut velocity = na::Vector3::zeros();
        let mut last_ts = imu_measurements[0].timestamp;

        for imu in imu_measurements {
            let dt = (imu.timestamp - last_ts) as f64 / 1e9;
            if dt > 0.0 {
                let accel = na::Vector3::new(
                    imu.accel[0] as f64,
                    imu.accel[1] as f64,
                    imu.accel[2] as f64,
                );
                // Accumulate velocity: v = sum(a * dt)
                velocity += accel * dt;
            }
            last_ts = imu.timestamp;
        }

        // Estimate motion magnitude and project to image plane
        // Using simplified perspective projection: pixel_motion = (v_x / v_z) * focal_length
        // For small rotations, v_z ~= 1 (forward-looking assumption)
        let speed_norm = velocity.norm();
        if speed_norm < 1e-6 {
            return (0.0, 0.0);
        }

        // Normalize velocity to unit magnitude and scale by focal length
        let v_normalized = velocity / speed_norm;
        let pixel_motion_scale = focal_length * speed_norm.min(0.1); // Clamp to avoid outliers

        let du = v_normalized[0] * pixel_motion_scale;
        let dv = v_normalized[1] * pixel_motion_scale;

        // Sanity check: motion should be reasonable (< 50 pixels typical for feature tracking)
        if du.abs() > 50.0 || dv.abs() > 50.0 {
            return (0.0, 0.0);
        }

        (du, dv)
    }

    pub fn update(&mut self, _timestamp: i64, _rotation: na::UnitQuaternion<f64>) {
        // No-op in new implementation
    }
}
