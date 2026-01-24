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

pub mod analysis;
pub mod denoise;
pub mod higher_order_filter;
pub mod imu_types; // New module for IMU-specific types
pub mod initialization;
pub mod learned_vibration;
pub mod vibration_filter;

use crate::debug_assert_approx;
pub use crate::imu_fl;
pub use analysis::{HarmonicDecomposition, ImuSignalAnalyzer, MotorState, SignalQuality};
pub use denoise::{DenoiseConfig, ImuDenoiseFilter};
pub use higher_order_filter::{
    HigherOrderFilter, HigherOrderFilterConfig, HigherOrderOutput, JerkStats, SnapStats,
};
pub use imu_types::{ImuFloat, ImuUnitQuaternion, ImuVector3}; // Import new IMU types

pub use initialization::{
    AdaptiveNoiseEstimator, BiasEstimate, ImuInitializationConfig, ImuInitializer,
    InitializationState,
};
pub use learned_vibration::{
    LearnedVibrationScheduler, RuleBasedVibrationScheduler, VibrationInputs, VibrationOutputs,
    VibrationTrainingSample,
};
pub use vibration_filter::{
    NotchFilter, VibrationFilterConfig, VibrationNotchFilter, VibrationPeak,
};

use crate::datasets::ImuData;
use crate::fl;
use crate::types::Float;
use nalgebra as na;

/// Configuration for IMU processing
#[derive(Debug, Clone)]
pub struct ImuConfig {
    /// Gyroscope noise density [rad/s/√Hz]
    pub gyro_noise_density: ImuFloat,
    /// Accelerometer noise density [m/s²/√Hz]
    pub accel_noise_density: ImuFloat,
    /// Gyroscope bias random walk [rad/s²/√Hz]
    pub gyro_bias_random_walk: ImuFloat,
    /// Accelerometer bias random walk [m/s³/√Hz]
    pub accel_bias_random_walk: ImuFloat,
    /// Gravity vector in world frame [m/s²]
    pub gravity: [ImuFloat; 3],
}

impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            gyro_noise_density: imu_fl!(1e-4),
            accel_noise_density: imu_fl!(1e-2),
            gyro_bias_random_walk: imu_fl!(1e-5),
            accel_bias_random_walk: imu_fl!(1e-4),
            gravity: [imu_fl!(0.0), imu_fl!(0.0), imu_fl!(-9.81)],
        }
    }
}

/// Preintegrated IMU measurements between two keyframes
#[derive(Debug, Clone)]
pub struct PreintegratedImu {
    /// Delta rotation from i to j [R_ij]
    pub delta_rotation: ImuUnitQuaternion,
    /// Delta velocity from i to j [m/s]
    pub delta_velocity: ImuVector3,
    /// Delta position from i to j [m]
    pub delta_position: ImuVector3,
    /// Time interval [s]
    pub delta_time: ImuFloat,
    /// Covariance matrix (9x9 for rotation, velocity, position)
    pub covariance: na::DMatrix<ImuFloat>,
    /// Jacobian of preintegration w.r.t. rotation at i
    pub jacobian_wrt_rotation: na::Matrix3<ImuFloat>,
    /// Jacobian of preintegration w.r.t. velocity at i
    pub jacobian_wrt_velocity: na::Matrix3<ImuFloat>,
    /// Jacobian of preintegration w.r.t. acceleration bias at i
    pub jacobian_wrt_accel_bias: na::Matrix3<ImuFloat>,
    /// Jacobian of preintegration w.r.t. gyro bias at i
    pub jacobian_wrt_gyro_bias: na::Matrix3<ImuFloat>,
}

impl PreintegratedImu {
    /// Create new preintegration with identity
    pub fn new() -> Self {
        Self {
            delta_rotation: ImuUnitQuaternion::identity(),
            delta_velocity: ImuVector3::zeros(),
            delta_position: ImuVector3::zeros(),
            delta_time: imu_fl!(0.0),
            covariance: na::DMatrix::identity(9, 9) * imu_fl!(1e8),
            jacobian_wrt_rotation: na::Matrix3::identity(),
            jacobian_wrt_velocity: na::Matrix3::zeros(),
            jacobian_wrt_accel_bias: na::Matrix3::zeros(),
            jacobian_wrt_gyro_bias: na::Matrix3::zeros(),
        }
    }

    /// Reset preintegration to identity
    pub fn reset(&mut self) {
        self.delta_rotation = ImuUnitQuaternion::identity();
        self.delta_velocity = ImuVector3::zeros();
        self.delta_position = ImuVector3::zeros();
        self.delta_time = imu_fl!(0.0);
        self.covariance = na::DMatrix::identity(9, 9) * imu_fl!(1e8);
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
pub struct ImuPreintegrator {
    config: ImuConfig,
    current: PreintegratedImu,
    last_gyro: ImuVector3,
    last_accel: ImuVector3,
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
    /// Hotpath: inlined for zero function call overhead
    #[inline(always)]
    pub fn propagate(&mut self, imu: &ImuData, dt: Float) {
        let gyro = ImuVector3::new(
            imu_fl!(imu.gyro[0]),
            imu_fl!(imu.gyro[1]),
            imu_fl!(imu.gyro[2]),
        );
        let accel = ImuVector3::new(
            imu_fl!(imu.accel[0]),
            imu_fl!(imu.accel[1]),
            imu_fl!(imu.accel[2]),
        );

        self.propagate_raw(gyro, accel, dt as ImuFloat);
    }

    /// Process bias-corrected IMU measurements
    #[inline(always)]
    pub fn propagate_corrected(
        &mut self,
        gyro_corrected: ImuVector3,
        accel_corrected: ImuVector3,
        dt: Float,
    ) {
        self.propagate_raw(gyro_corrected, accel_corrected, dt as ImuFloat);
    }

    /// Internal method for propagation with raw (possibly corrected) measurements
    /// Hotpath: aggressively inlined, optimized for numerical stability
    #[inline(always)]
    fn propagate_raw(&mut self, gyro: ImuVector3, accel: ImuVector3, dt: ImuFloat) {
        if !self.has_initial_measurement {
            self.last_gyro = gyro;
            self.last_accel = accel;
            self.has_initial_measurement = true;
            return;
        }

        // Midpoint integration for 2nd order accuracy
        // Fused multiply-add for better performance
        let gyro_avg = (self.last_gyro + gyro) * imu_fl!(0.5);
        let accel_avg = (self.last_accel + accel) * imu_fl!(0.5);

        // Update rotation using Rodrigues' formula (exponential map)
        let delta_angle = gyro_avg * dt;
        let delta_rot = ImuUnitQuaternion::new(delta_angle);
        debug_assert_approx!(delta_rot.norm_squared(), imu_fl!(1.0), imu_fl!(1e-6));

        // Quaternion multiplication (right-to-left composition)
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

    /// Check if we have valid preintegrated measurements
    pub fn is_valid(&self) -> bool {
        self.current.delta_time > 0.0 && self.has_initial_measurement
    }

    /// Create an IMU motion prior from current preintegration state
    ///
    /// This should be called after processing IMU measurements between two keyframes
    /// to create a prior for bundle adjustment.
    ///
    /// # Arguments
    /// * `initial_pose` - Pose at the start of the interval (T_W_B at time i)
    /// * `initial_velocity` - Velocity at the start of the interval
    ///
    /// # Returns
    /// Some(ImuMotionPrior) if valid preintegration exists, None otherwise
    pub fn create_motion_prior(
        &self,
        initial_pose: na::Matrix4<Float>,
        initial_velocity: na::Vector3<Float>,
    ) -> Option<ImuMotionPrior> {
        if !self.is_valid() {
            return None;
        }

        let gravity = na::Vector3::new(
            self.config.gravity[0] as Float,
            self.config.gravity[1] as Float,
            self.config.gravity[2] as Float,
        );

        Some(ImuMotionPrior::from_preintegration(
            &self.current,
            initial_pose,
            initial_velocity,
            gravity,
        ))
    }

    /// Create preintegration data for tight-coupled VIO
    ///
    /// This creates the data structure needed by InterKeyframeImuFactor
    /// for true tight coupling between consecutive keyframes.
    ///
    /// # Returns
    /// Some(ImuPreintegration) if valid data exists, None otherwise
    pub fn create_tight_coupling_preintegration(
        &self,
    ) -> Option<crate::optimization::tight_coupling::ImuPreintegration> {
        if !self.is_valid() {
            return None;
        }

        Some(crate::optimization::tight_coupling::ImuPreintegration {
            dt: self.current.delta_time as Float,
            delta_R: self
                .current
                .delta_rotation
                .to_rotation_matrix()
                .into_inner()
                .cast::<Float>(),
            delta_v: self.current.delta_velocity.cast::<Float>(),
            delta_p: self.current.delta_position.cast::<Float>(),
            cov_R: self
                .current
                .covariance
                .fixed_view::<3, 3>(0, 0)
                .into_owned()
                .cast::<Float>(),
            cov_v: self
                .current
                .covariance
                .fixed_view::<3, 3>(3, 3)
                .into_owned()
                .cast::<Float>(),
            cov_p: self
                .current
                .covariance
                .fixed_view::<3, 3>(6, 6)
                .into_owned()
                .cast::<Float>(),
            cov_R_bw: self.current.jacobian_wrt_gyro_bias.cast::<Float>(),
            cov_v_ba: self.current.jacobian_wrt_accel_bias.cast::<Float>(),
            cov_p_ba: na::Matrix3::zeros(),
        })
    }

    /// Get the time interval of preintegrated measurements
    pub fn delta_time(&self) -> Float {
        self.current.delta_time as Float
    }

    /// Reset preintegrator
    pub fn reset(&mut self) {
        self.current.reset();
        self.has_initial_measurement = false;
    }
}

/// Motion prediction using gyroscope
pub struct ImuMotionPredictor {
    /// Configuration (reserved for future prediction tuning)
    _config: ImuConfig,
    last_rotation: Option<ImuUnitQuaternion>,
    last_timestamp: Option<i64>,
}

impl ImuMotionPredictor {
    /// Create new motion predictor
    pub fn new(config: ImuConfig) -> Self {
        Self {
            _config: config,
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
    pub fn predict_feature_displacement(
        &self,
        imu_measurements: &[ImuData],
        _prev_pixel: (Float, Float),
        focal_length: Float,
    ) -> (Float, Float) {
        if imu_measurements.is_empty() {
            return (fl!(0.0), fl!(0.0));
        }

        // Integrate gyroscope to get total rotation
        let mut total_rotation = ImuVector3::zeros();
        let mut last_ts = imu_measurements[0].timestamp;

        for imu in imu_measurements {
            let dt = imu_fl!((imu.timestamp - last_ts) as f64 / 1e9); // Cast to f32
            if dt > imu_fl!(0.0) {
                total_rotation += ImuVector3::new(
                    imu_fl!(imu.gyro[0]) * dt,
                    imu_fl!(imu.gyro[1]) * dt,
                    imu_fl!(imu.gyro[2]) * dt,
                );
            }
            last_ts = imu.timestamp;
        }

        // Convert to rotation angle and axis
        let angle = total_rotation.norm();
        if angle < imu_fl!(1e-6) {
            return (fl!(0.0), fl!(0.0));
        }
        let _axis = total_rotation / angle;

        // Rotation angle in image plane (simplified)
        // For small rotations, du ≈ -ω_y * f, dv ≈ ω_x * f
        let predicted_du = -total_rotation[1] as Float * focal_length;
        let predicted_dv = total_rotation[0] as Float * focal_length;

        (predicted_du, predicted_dv)
    }

    /// Update predictor with new rotation
    pub fn update(&mut self, timestamp: i64, rotation: ImuUnitQuaternion) {
        if let Some((last_ts, _last_rot)) = self.last_timestamp.zip(self.last_rotation.as_ref()) {
            let _dt = imu_fl!((timestamp - last_ts) as f64 / 1e9);
            if _dt > imu_fl!(0.0) {
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
    velocity: na::Vector3<Float>, // Velocity is global Float
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
    /// * `initial_orientation` - Initial body orientation (global Float)
    pub fn initialize_from_imu(
        &mut self,
        imu_measurements: &[ImuData],
        initial_orientation: &na::UnitQuaternion<Float>,
    ) {
        if imu_measurements.len() < 2 {
            return;
        }

        let gravity = na::Vector3::new(
            self.config.gravity[0] as Float,
            self.config.gravity[1] as Float,
            self.config.gravity[2] as Float,
        );

        // Integrate accelerometer to get velocity change
        let mut delta_v = na::Vector3::zeros();
        let mut last_ts = imu_measurements[0].timestamp;

        for imu in imu_measurements.iter().skip(1) {
            let dt = (imu.timestamp - last_ts) as Float / fl!(1e9);
            if dt > fl!(0.0) {
                let accel = na::Vector3::new(
                    imu.accel[0] as Float,
                    imu.accel[1] as Float,
                    imu.accel[2] as Float,
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
    pub fn get_velocity(&self) -> na::Vector3<Float> {
        self.velocity
    }

    /// Update velocity estimate with new IMU measurements
    pub fn update(&mut self, imu_measurements: &[ImuData], dt: Float) {
        if !self.initialized || imu_measurements.is_empty() {
            return;
        }

        let dt_float = dt;
        let gravity = na::Vector3::new(
            self.config.gravity[0] as Float,
            self.config.gravity[1] as Float,
            self.config.gravity[2] as Float,
        );

        // Integrate accelerometer
        for imu in imu_measurements {
            let accel = na::Vector3::new(
                imu.accel[0] as Float,
                imu.accel[1] as Float,
                imu.accel[2] as Float,
            );
            // Assume current orientation is approximately identity
            let accel_world = accel - gravity;
            self.velocity += accel_world * dt_float;
        }
    }

    /// Check if velocity is initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }
}

/// IMU bias estimator for static initialization and continuous bias tracking
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
    gyro_samples: Vec<ImuVector3>,
    /// Accumulated accelerometer measurements for bias estimation
    accel_samples: Vec<ImuVector3>,
    /// Timestamps for each sample
    timestamps: Vec<i64>,
    /// Estimated gyroscope bias [rad/s]
    pub gyro_bias: ImuVector3,
    /// Estimated accelerometer bias [m/s²]
    pub accel_bias: ImuVector3,
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
            gyro_bias: ImuVector3::zeros(),
            accel_bias: ImuVector3::zeros(),
            is_initialized: false,
            min_samples: 100, // At 200Hz, this is 0.5 seconds
        }
    }

    /// Add IMU measurement for bias estimation
    ///
    /// # Arguments
    /// * `imu` - IMU measurement
    /// * `_assume_stationary` - If true, assume system is stationary (use for initialization)
    pub fn add_sample(&mut self, imu: &ImuData, _assume_stationary: bool) {
        let gyro = ImuVector3::new(
            imu_fl!(imu.gyro[0]),
            imu_fl!(imu.gyro[1]),
            imu_fl!(imu.gyro[2]),
        );
        let accel = ImuVector3::new(
            imu_fl!(imu.accel[0]),
            imu_fl!(imu.accel[1]),
            imu_fl!(imu.accel[2]),
        );

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
        let mut gyro_sum = ImuVector3::zeros();
        for g in &self.gyro_samples {
            gyro_sum += g;
        }
        self.gyro_bias = gyro_sum / imu_fl!(self.gyro_samples.len() as ImuFloat);

        // Estimate accel bias: mean - gravity (assuming stationary = gravity only)
        let gravity = ImuVector3::new(
            self.config.gravity[0],
            self.config.gravity[1],
            self.config.gravity[2],
        );
        let mut accel_sum = ImuVector3::zeros();
        for a in &self.accel_samples {
            accel_sum += a;
        }
        let mean_accel = accel_sum / imu_fl!(self.accel_samples.len() as ImuFloat);
        self.accel_bias = mean_accel - gravity;

        self.is_initialized = true;
    }

    /// Get bias-corrected gyroscope measurement
    pub fn correct_gyro(&self, imu: &ImuData) -> ImuVector3 {
        ImuVector3::new(
            imu_fl!(imu.gyro[0]) - self.gyro_bias[0],
            imu_fl!(imu.gyro[1]) - self.gyro_bias[1],
            imu_fl!(imu.gyro[2]) - self.gyro_bias[2],
        )
    }

    /// Get bias-corrected accelerometer measurement
    pub fn correct_accel(&self, imu: &ImuData) -> ImuVector3 {
        ImuVector3::new(
            imu_fl!(imu.accel[0]) - self.accel_bias[0],
            imu_fl!(imu.accel[1]) - self.accel_bias[1],
            imu_fl!(imu.accel[2]) - self.accel_bias[2],
        )
    }

    /// Reset the bias estimator (e.g., after motion restart)
    pub fn reset(&mut self) {
        self.gyro_samples.clear();
        self.accel_samples.clear();
        self.timestamps.clear();
        self.gyro_bias = ImuVector3::zeros();
        self.accel_bias = ImuVector3::zeros();
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

    /// Number of calibration iterations run
    pub fn iterations(&self) -> usize {
        self.iterations
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
    imu_delta_rotation: ImuUnitQuaternion,
    /// IMU delta translation from last keyframe
    imu_delta_translation: ImuVector3,
    /// IMU delta time from last keyframe
    imu_delta_time: ImuFloat,
}

impl ImuAidedKeyframeSelector {
    /// Create new selector with thresholds
    pub fn new(translation_threshold: Float, rotation_threshold: Float) -> Self {
        Self {
            translation_threshold,
            rotation_threshold,
            last_keyframe_timestamp: None,
            last_keyframe_pose: None,
            imu_delta_rotation: ImuUnitQuaternion::identity(),
            imu_delta_translation: ImuVector3::zeros(),
            imu_delta_time: imu_fl!(0.0),
        }
    }

    /// Reset selector (e.g., after loop closure)
    pub fn reset(&mut self) {
        self.last_keyframe_timestamp = None;
        self.last_keyframe_pose = None;
        self.imu_delta_rotation = ImuUnitQuaternion::identity();
        self.imu_delta_translation = ImuVector3::zeros();
        self.imu_delta_time = imu_fl!(0.0);
    }

    /// Update with new IMU measurement
    pub fn update_imu(&mut self, imu: &ImuData) {
        let gyro = ImuVector3::new(
            imu_fl!(imu.gyro[0]),
            imu_fl!(imu.gyro[1]),
            imu_fl!(imu.gyro[2]),
        );
        let accel = ImuVector3::new(
            imu_fl!(imu.accel[0]),
            imu_fl!(imu.accel[1]),
            imu_fl!(imu.accel[2]),
        );

        // Integrate rotation
        let delta_rot = ImuUnitQuaternion::new(gyro * imu_fl!(0.01)); // Approximate dt
        self.imu_delta_rotation = delta_rot * self.imu_delta_rotation;

        // Integrate translation (simplified - assumes small motion)
        self.imu_delta_translation += accel * imu_fl!(0.01) * imu_fl!(0.01) * imu_fl!(0.5);
        self.imu_delta_time += imu_fl!(0.01);
    }

    /// Accumulate IMU measurements between frames
    pub fn accumulate_imu(&mut self, imu_measurements: &[ImuData]) {
        if imu_measurements.is_empty() {
            return;
        }

        let mut last_ts = imu_measurements[0].timestamp;
        let mut integrated_rot = ImuUnitQuaternion::identity();
        let mut integrated_trans = ImuVector3::zeros();

        for imu in imu_measurements {
            let dt = imu_fl!((imu.timestamp - last_ts) as f64 / 1e9);
            if dt > imu_fl!(0.0) {
                let gyro = ImuVector3::new(
                    imu_fl!(imu.gyro[0]),
                    imu_fl!(imu.gyro[1]),
                    imu_fl!(imu.gyro[2]),
                );
                let accel = ImuVector3::new(
                    imu_fl!(imu.accel[0]),
                    imu_fl!(imu.accel[1]),
                    imu_fl!(imu.accel[2]),
                );

                // Rotation integration
                let delta_rot = ImuUnitQuaternion::new(gyro * dt);
                integrated_rot = delta_rot * integrated_rot;

                // Translation integration (assuming constant velocity model)
                integrated_trans += accel * dt * dt * imu_fl!(0.5);

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
            let T_rel = if let Some(inv) = last_pose.try_inverse() {
                inv * current_pose
            } else {
                na::Matrix4::identity()
            };
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
            let imu_translation_norm = self.imu_delta_translation.norm() as Float;
            let imu_rotation_angle = self.imu_delta_rotation.angle() as Float;

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

            log::info!(
                "[IMU KF] decision: is_kf={}, visual_trig={}, imu_trig={}, disagree={}, t={:.3}m, r={:.3}rad, imu_t={:.3}m, imu_r={:.3}rad, dev={:.3}rad",
                is_keyframe,
                visual_trigger,
                imu_motion_trigger,
                imu_visual_disagree,
                translation_norm,
                rotation_norm,
                imu_translation_norm,
                imu_rotation_angle.abs(),
                imu_rotation_deviation
            );
        } else {
            // First frame after initialization is always a keyframe
            is_keyframe = true;
            reason = "Initial frame".to_string();
        }

        // Update state if keyframe
        if is_keyframe {
            self.last_keyframe_timestamp = Some(current_timestamp);
            self.last_keyframe_pose = Some(*current_pose);
            self.imu_delta_rotation = ImuUnitQuaternion::identity();
            self.imu_delta_translation = ImuVector3::zeros();
            self.imu_delta_time = imu_fl!(0.0);
        }

        (is_keyframe, reason)
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
    pub delta_rotation: na::UnitQuaternion<Float>,
    /// Preintegrated velocity change from i to j
    pub delta_velocity: na::Vector3<Float>,
    /// Preintegrated position change from i to j
    pub delta_position: na::Vector3<Float>,
    /// Time interval
    pub delta_time: Float,
    /// Initial pose at time i
    pub initial_pose: na::Matrix4<Float>,
    /// Initial velocity at time i
    pub initial_velocity: na::Vector3<Float>,
    /// Gravity vector in world frame
    pub gravity: na::Vector3<Float>,
}

impl ImuMotionPrior {
    /// Create from preintegrated measurements
    pub fn from_preintegration(
        preint: &PreintegratedImu,
        initial_pose: na::Matrix4<Float>,
        initial_velocity: na::Vector3<Float>,
        gravity: na::Vector3<Float>,
    ) -> Self {
        Self {
            delta_rotation: preint.delta_rotation.cast::<Float>(),
            delta_velocity: preint.delta_velocity.cast::<Float>(),
            delta_position: preint.delta_position.cast::<Float>(),
            delta_time: preint.delta_time as Float,
            initial_pose,
            initial_velocity,
            gravity,
        }
    }

    /// Compute predicted state at time j
    ///
    /// Returns (predicted_pose, predicted_velocity)
    pub fn predict_state(&self) -> (na::Matrix4<Float>, na::Vector3<Float>) {
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
            + fl!(0.5) * self.gravity * self.delta_time * self.delta_time
            + self.delta_position;

        // Compose pose matrix
        let mut T_W_Bj = na::Matrix4::<Float>::identity();
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
        observed_pose: &na::Matrix4<Float>,
        observed_velocity: &na::Vector3<Float>,
    ) -> (na::Vector3<Float>, Float, na::Vector3<Float>) {
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
        let mut selector = ImuAidedKeyframeSelector::new(fl!(0.1), fl!(0.1));

        // First frame should be keyframe
        let pose = na::Matrix4::identity();
        let (is_kf, reason) = selector.should_be_keyframe(&pose, 1000000000, fl!(0.0));
        assert!(is_kf);
        assert_eq!(reason, "Initial frame");

        // Small motion shouldn't trigger keyframe
        let small_motion =
            na::Matrix4::new_translation(&na::Vector3::new(fl!(0.01), fl!(0.0), fl!(0.0)));
        let (is_kf, _) = selector.should_be_keyframe(&small_motion, 1000001000, fl!(0.01));
        assert!(!is_kf);

        // Large motion should trigger keyframe
        let large_motion =
            na::Matrix4::new_translation(&na::Vector3::new(fl!(0.2), fl!(0.0), fl!(0.0)));
        let (is_kf, reason) = selector.should_be_keyframe(&large_motion, 1000002000, fl!(0.01));
        assert!(is_kf);
        assert!(reason.contains("Visual motion"));
    }

    #[test]
    fn test_imu_motion_prior() {
        let preint = PreintegratedImu::new();
        let pose = na::Matrix4::<Float>::identity();
        let velocity = na::Vector3::<Float>::zeros();
        let gravity = na::Vector3::new(fl!(0.0), fl!(0.0), fl!(-9.81));

        let prior = ImuMotionPrior::from_preintegration(&preint, pose, velocity, gravity);
        let (pred_pose, pred_vel) = prior.predict_state();

        // Should be close to initial for zero IMU motion
        assert!((pred_pose - pose).abs().sum() < fl!(1e-10));
        assert!((pred_vel - velocity).norm() < fl!(1e-10));
    }

    #[test]
    fn test_imu_motion_prior_innovation() {
        let mut preint = PreintegratedImu::new();
        preint.delta_rotation = ImuUnitQuaternion::new(ImuVector3::zeros());
        preint.delta_velocity = ImuVector3::new(imu_fl!(0.1), imu_fl!(0.0), imu_fl!(0.0));
        preint.delta_position = ImuVector3::zeros();
        preint.delta_time = imu_fl!(1.0);

        let pose = na::Matrix4::identity();
        let velocity = na::Vector3::zeros();
        let gravity = na::Vector3::zeros();

        let prior = ImuMotionPrior::from_preintegration(&preint, pose, velocity, gravity);
        let obs_pose = na::Matrix4::identity();
        let obs_vel = na::Vector3::zeros();

        let (pos_err, rot_err, vel_err) = prior.compute_innovation(&obs_pose, &obs_vel);

        // Position error should be small (no delta position)
        assert!(pos_err.norm() < fl!(1e-10));
        // Rotation error should be small
        assert!(rot_err.abs() < fl!(1e-10));
        // Velocity error should be ~0.1 (predicted has velocity, observed doesn't)
        assert!((vel_err - na::Vector3::new(fl!(-0.1), fl!(0.0), fl!(0.0))).norm() < fl!(1e-5));
    }

    #[test]
    fn test_motion_predictor() {
        let config = ImuConfig::default();
        let predictor = ImuMotionPredictor::new(config);

        // No IMU measurements should give zero displacement
        let disp =
            predictor.predict_feature_displacement(&[], (fl!(320.0), fl!(240.0)), fl!(500.0));
        assert!((disp.0).abs() < fl!(1e-6));
        assert!((disp.1).abs() < fl!(1e-6));
    }

    #[test]
    fn test_velocity_estimator() {
        let config = ImuConfig::default();
        let mut estimator = VelocityEstimator::new(config);

        assert!(!estimator.is_initialized());

        let imu_measurements = vec![
            ImuData {
                timestamp: 0,
                gyro: [fl!(0.0); 3],
                accel: [fl!(0.0), fl!(0.0), fl!(9.81)],
            },
            ImuData {
                timestamp: 10000000,
                gyro: [fl!(0.0); 3],
                accel: [fl!(0.0), fl!(0.0), fl!(9.81)],
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

    #[test]
    fn test_imu_bias_estimator() {
        let config = ImuConfig::default();
        let mut bias_estimator = ImuBiasEstimator::new(config);

        // Initially not initialized
        assert!(!bias_estimator.is_initialized);
        assert_eq!(bias_estimator.sample_count(), 0);

        // Add some IMU samples with known biases
        // Simulate: gyro bias = [0.01, -0.02, 0.005], accel bias = [0.05, -0.03, 0.1]
        let gyro_bias = [imu_fl!(0.01), imu_fl!(-0.02), imu_fl!(0.005)];
        let accel_bias = [imu_fl!(0.05), imu_fl!(-0.03), imu_fl!(0.1)];
        let gravity = imu_fl!(-9.81);

        for i in 0..200 {
            let ts = (i * 5000) as i64; // 200Hz
            let imu = ImuData {
                timestamp: ts,
                gyro: [
                    (imu_fl!(0.001) + gyro_bias[0]) as f64, // Small motion + bias
                    (imu_fl!(-0.002) + gyro_bias[1]) as f64,
                    (imu_fl!(0.001) + gyro_bias[2]) as f64,
                ],
                accel: [
                    (imu_fl!(0.02) + accel_bias[0]) as f64, // Small acceleration + bias
                    (imu_fl!(-0.01) + accel_bias[1]) as f64,
                    (gravity + accel_bias[2]) as f64,
                ],
            };
            bias_estimator.add_sample(&imu, true);
        }

        // Should now be initialized
        assert!(bias_estimator.is_initialized);
        assert_eq!(bias_estimator.sample_count(), 200);

        // Check bias estimates (should be close to true biases, accounting for small motion)
        assert!(
            (bias_estimator.gyro_bias[0] - gyro_bias[0] - imu_fl!(0.001)).abs() < imu_fl!(0.005)
        );
        assert!(
            (bias_estimator.gyro_bias[1] - gyro_bias[1] + imu_fl!(0.002)).abs() < imu_fl!(0.005)
        );
        assert!(
            (bias_estimator.gyro_bias[2] - gyro_bias[2] - imu_fl!(0.001)).abs() < imu_fl!(0.005)
        );

        assert!(
            (bias_estimator.accel_bias[0] - accel_bias[0] - imu_fl!(0.02)).abs() < imu_fl!(0.05)
        );
        assert!(
            (bias_estimator.accel_bias[1] - accel_bias[1] + imu_fl!(0.01)).abs() < imu_fl!(0.05)
        );
        assert!((bias_estimator.accel_bias[2] - accel_bias[2]).abs() < imu_fl!(0.05));

        // Test bias correction - should remove the known bias
        let test_imu = ImuData {
            timestamp: 0,
            gyro: [
                (gyro_bias[0] + imu_fl!(0.1)) as f64,
                (gyro_bias[1] + imu_fl!(0.1)) as f64,
                (gyro_bias[2] + imu_fl!(0.1)) as f64,
            ],
            accel: [
                (accel_bias[0] + imu_fl!(1.0)) as f64,
                (accel_bias[1] + imu_fl!(1.0)) as f64,
                (accel_bias[2] + imu_fl!(1.0)) as f64, // No gravity - bias estimator handles that
            ],
        };

        let corrected_gyro = bias_estimator.correct_gyro(&test_imu);
        // Corrected value should be approximately 0.1 (the added motion, minus small bias estimation error)
        assert!(
            (corrected_gyro[0] - imu_fl!(0.1)).abs() < imu_fl!(0.02),
            "gyro x: {} vs expected 0.1",
            corrected_gyro[0]
        );
        assert!(
            (corrected_gyro[1] - imu_fl!(0.1)).abs() < imu_fl!(0.02),
            "gyro y: {} vs expected 0.1",
            corrected_gyro[1]
        );
        assert!(
            (corrected_gyro[2] - imu_fl!(0.1)).abs() < imu_fl!(0.02),
            "gyro z: {} vs expected 0.1",
            corrected_gyro[2]
        );

        let corrected_accel = bias_estimator.correct_accel(&test_imu);
        // Corrected value should be approximately 1.0 (the added acceleration, minus small bias estimation error)
        assert!(
            (corrected_accel[0] - imu_fl!(1.0)).abs() < imu_fl!(0.1),
            "accel x: {} vs expected 1.0",
            corrected_accel[0]
        );
        assert!(
            (corrected_accel[1] - imu_fl!(1.0)).abs() < imu_fl!(0.1),
            "accel y: {} vs expected 1.0",
            corrected_accel[1]
        );
        assert!(
            (corrected_accel[2] - imu_fl!(1.0)).abs() < imu_fl!(0.1),
            "accel z: {} vs expected 1.0",
            corrected_accel[2]
        );

        // Test reset
        bias_estimator.reset();
        assert!(!bias_estimator.is_initialized);
        assert_eq!(bias_estimator.sample_count(), 0);
    }

    #[test]
    fn test_imu_preintegrator_with_bias_correction() {
        let config = ImuConfig::default();
        let mut preintegrator = ImuPreintegrator::new(config);

        // Simulate IMU with known biases
        let gyro_bias = [imu_fl!(0.01), imu_fl!(-0.02), imu_fl!(0.005)];
        let accel_bias = [imu_fl!(0.05), imu_fl!(-0.03), imu_fl!(0.1)];

        // Process 10 samples with bias correction
        for i in 0..10 {
            let dt = fl!(0.005); // 5ms
            let _ts = (i * 5000) as i64;

            // Measurements with bias
            let gyro_raw = [
                imu_fl!(0.1) + gyro_bias[0],
                imu_fl!(0.05) + gyro_bias[1],
                imu_fl!(0.02) + gyro_bias[2],
            ];
            let accel_raw = [
                imu_fl!(0.5) + accel_bias[0],
                imu_fl!(-0.2) + accel_bias[1],
                imu_fl!(-9.81) + accel_bias[2],
            ];

            // Corrected measurements
            let gyro_corrected = ImuVector3::new(
                gyro_raw[0] - gyro_bias[0],
                gyro_raw[1] - gyro_bias[1],
                gyro_raw[2] - gyro_bias[2],
            );
            let accel_corrected = ImuVector3::new(
                accel_raw[0] - accel_bias[0],
                accel_raw[1] - accel_bias[1],
                accel_raw[2] - accel_bias[2],
            );

            preintegrator.propagate_corrected(gyro_corrected, accel_corrected, dt);
        }

        let preint = preintegrator.get();
        assert!(
            preint.delta_time > imu_fl!(0.0),
            "Should have non-zero delta time"
        );
        assert!(
            preint.delta_rotation.angle() > imu_fl!(0.0),
            "Should have rotation from angular velocity"
        );

        // Preintegrator should accumulate position change
        // With constant velocity, position change should be significant
        assert!(
            preint.delta_position.norm() > imu_fl!(0.001),
            "Should have position change: {}",
            preint.delta_position.norm()
        );
    }

    #[test]
    fn test_imu_motion_prior_innovation_comprehensive() {
        // Test IMU motion prior with various motion scenarios
        let test_cases = vec![
            // (delta_rotation_deg, delta_velocity, delta_position, delta_time, description)
            (
                fl!(0.0),
                na::Vector3::zeros(),
                na::Vector3::zeros(),
                fl!(1.0),
                "stationary",
            ),
            (
                fl!(10.0),
                na::Vector3::new(fl!(0.1), fl!(0.0), fl!(0.0)),
                na::Vector3::zeros(),
                fl!(1.0),
                "pure_translation",
            ),
            (
                fl!(0.0),
                na::Vector3::zeros(),
                na::Vector3::new(fl!(0.05), fl!(0.0), fl!(0.0)),
                fl!(1.0),
                "position_offset",
            ),
            (
                fl!(5.0),
                na::Vector3::new(fl!(0.05), fl!(0.02), fl!(0.0)),
                na::Vector3::new(fl!(0.02), fl!(0.01), fl!(0.0)),
                fl!(0.5),
                "combined_motion",
            ),
        ];

        for (rot_deg, vel, pos, dt, desc) in test_cases {
            let mut preint = PreintegratedImu::new();
            let rot_rad = rot_deg as ImuFloat * std::f64::consts::PI as ImuFloat / imu_fl!(180.0);
            if rot_rad > imu_fl!(0.0) {
                preint.delta_rotation = ImuUnitQuaternion::new(ImuVector3::z() * rot_rad);
            }
            preint.delta_velocity = vel.cast::<ImuFloat>();
            preint.delta_position = pos.cast::<ImuFloat>();
            preint.delta_time = dt as ImuFloat;

            let initial_pose = na::Matrix4::identity();
            let initial_velocity = na::Vector3::zeros();
            let gravity = na::Vector3::new(fl!(0.0), fl!(0.0), fl!(-9.81));

            let prior = ImuMotionPrior::from_preintegration(
                &preint,
                initial_pose,
                initial_velocity,
                gravity,
            );
            let (pred_pose, _pred_vel) = prior.predict_state();

            // Verify prediction properties
            if desc == "stationary" {
                // For stationary case, rotation should be identity
                assert!(
                    pred_pose.fixed_view::<3, 3>(0, 0).abs().sum() > fl!(2.9),
                    "Rotation matrix should be approximately identity for stationary"
                );
            }

            // Innovation with matching observation should be small
            let (_, rot_err, _) = prior.compute_innovation(&initial_pose, &initial_velocity);

            // For stationary case with zero preintegration, rotation error should be zero
            if desc == "stationary" {
                assert!(
                    rot_err.abs() < fl!(1e-10),
                    "Rotation error should be zero for stationary"
                );
            }
        }
    }

    #[test]
    fn test_imu_bias_estimator_reset() {
        let config = ImuConfig::default();
        let mut bias_estimator = ImuBiasEstimator::new(config);

        // Add samples to initialize
        for i in 0..150 {
            let _ts = (i * 5000) as i64;
            let imu = ImuData {
                timestamp: _ts,
                gyro: [
                    imu_fl!(0.01) as f64,
                    imu_fl!(-0.02) as f64,
                    imu_fl!(0.005) as f64,
                ],
                accel: [
                    imu_fl!(0.05) as f64,
                    imu_fl!(-0.03) as f64,
                    (imu_fl!(-9.81) + imu_fl!(0.1)) as f64,
                ],
            };
            bias_estimator.add_sample(&imu, true);
        }

        assert!(bias_estimator.is_initialized);
        assert!(bias_estimator.sample_count() > 100);

        // Verify biases are non-zero after initialization
        assert!(bias_estimator.gyro_bias.norm() > imu_fl!(0.0));
        assert!(bias_estimator.accel_bias.norm() > imu_fl!(0.0));

        // Reset
        bias_estimator.reset();

        // Verify reset state
        assert!(!bias_estimator.is_initialized);
        assert_eq!(bias_estimator.sample_count(), 0);
        assert_eq!(bias_estimator.gyro_bias, ImuVector3::zeros());
        assert_eq!(bias_estimator.accel_bias, ImuVector3::zeros());

        // After reset, adding new samples should work
        for i in 0..150 {
            let _ts = (i * 5000) as i64;
            let imu = ImuData {
                timestamp: _ts,
                gyro: [
                    imu_fl!(0.02) as f64,
                    imu_fl!(-0.03) as f64,
                    imu_fl!(0.01) as f64,
                ], // Different biases
                accel: [
                    imu_fl!(0.1) as f64,
                    imu_fl!(-0.05) as f64,
                    (imu_fl!(-9.81) + imu_fl!(0.2)) as f64,
                ],
            };
            bias_estimator.add_sample(&imu, true);
        }

        assert!(bias_estimator.is_initialized);
        // After reset and new samples, biases should be non-zero and different from zeros
        assert!(
            bias_estimator.gyro_bias.norm() > imu_fl!(0.0),
            "Gyro bias should be non-zero after reset and new samples"
        );
    }
}
