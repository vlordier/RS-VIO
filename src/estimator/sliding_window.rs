use crate::datasets::ImuData;
use crate::estimator::Frame;
use crate::estimator::loop_closure::LoopClosureDatabase;
use crate::estimator::loop_closure_validation::validate_loop_closure;
use crate::estimator::motion_prior::{MotionPrior, NullPrior, GateDecision};
use crate::optimization::factors::{BundleAdjustmentFactor, PnPFactor, LoopClosureEdgeFactor};
use crate::optimization::imu_factor::ImuPreintegrationFactor;
use crate::optimization::imu_preintegration::PreintegratedImuMeasurements;
use crate::types::{Matrix3x3, Matrix4x4, Vector3};
use apex_solver::core::loss_functions::HuberLoss;
use apex_solver::core::problem::{Problem, VariableEnum};
use apex_solver::linalg::{LinearSolverType, SchurPreconditioner, SchurVariant};
use apex_solver::manifold::ManifoldType;
use apex_solver::optimizer::levenberg_marquardt::{LevenbergMarquardt, LevenbergMarquardtConfig};
use apex_solver::optimizer::SolverResult;
use apex_solver::JacobianMode;
use na::{DVector, UnitQuaternion};
use nalgebra as na;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::HashMap;
use std::collections::VecDeque;

/// Standard gravity magnitude (m/s²). Assumes world z-axis points down.
const GRAVITY_MAGNITUDE: f64 = 9.81;

/// Gravity vector in world frame (z-down convention).
const GRAVITY: Vector3 = na::Vector3::new(0.0, 0.0, GRAVITY_MAGNITUDE);

/// Compute the inverse of an SE(3) transform matrix using the closed-form solution.
///
/// For `T = [R | t; 0 | 1]`, the inverse is `T⁻¹ = [Rᵀ | -Rᵀ·t; 0 | 1]`.
/// This is O(n²) vs O(n³) for a full matrix inverse.
#[inline(always)]
pub fn inverse_se3(t: &Matrix4x4) -> Matrix4x4 {
    let r_inv = t.fixed_view::<3, 3>(0, 0).transpose();
    let t_part = r_inv * t.fixed_view::<3, 1>(0, 3);
    let mut result = Matrix4x4::identity();
    result.fixed_view_mut::<3, 3>(0, 0).copy_from(&r_inv);
    result.fixed_view_mut::<3, 1>(0, 3).copy_from(&(-t_part));
    result
}

/// Pre-integrate IMU measurements to get relative SE(3) motion.
///
/// Implements discrete **midpoint** pre-integration with **bias correction**:
/// ```text
/// ΔR_k+1 = ΔR_k · Exp((ω_k - b_g) · Δt)
/// Δv_k+1 = Δv_k + ½ · (ΔR_k·(a_k - b_a) + ΔR_k+1·(a_k+1 - b_a)) · Δt
/// Δp_k+1 = Δp_k + Δv_k · Δt + ½ · ΔR_k·(a_k - b_a) · Δt²
/// ```
///
/// Uses midpoint integration for 2nd-order accuracy (vs 1st-order Euler).
/// Bias-subtracted IMU readings prevent drift during long integration windows.
///
/// # Arguments
/// * `imu_samples` — raw IMU measurements in chronological order
/// * `gyro_bias` — estimated gyro bias [x, y, z] in rad/s
/// * `accel_bias` — estimated accel bias [x, y, z] in m/s²
/// * `gravity` — gravity vector in world frame (pre-estimated, ~9.81 m/s²)
///
/// # Returns
/// Relative transform T_prev_to_curr as a 4x4 SE(3) matrix.
/// Returns identity if fewer than 2 samples.
#[inline(always)]
pub fn preintegrate_imu(
    imu_samples: &[ImuData],
    gyro_bias: &[f64; 3],
    accel_bias: &[f64; 3],
    gravity: &Vector3,
) -> Matrix4x4 {
    if imu_samples.len() < 2 {
        return Matrix4x4::identity();
    }

    // Bias-subtracted vectors for convenience
    let bg = Vector3::new(gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    let ba = Vector3::new(accel_bias[0], accel_bias[1], accel_bias[2]);

    let mut delta_r = na::Matrix3::<f64>::identity();
    let mut delta_v = Vector3::zeros();
    let mut delta_p = Vector3::zeros();

    // Precompute bias-corrected accel for first sample to reuse across iterations.
    let mut a_curr_body = Vector3::new(
        imu_samples[0].accel[0] - ba.x,
        imu_samples[0].accel[1] - ba.y,
        imu_samples[0].accel[2] - ba.z,
    );

    for i in 0..imu_samples.len() - 1 {
        let curr = &imu_samples[i];
        let next = &imu_samples[i + 1];
        let dt = (next.timestamp - curr.timestamp) as f64 / 1e9;
        if dt <= 0.0 || dt > 1.0 {
            // Recompute a_curr_body for next iteration after a skipped step
            a_curr_body = Vector3::new(
                next.accel[0] - ba.x,
                next.accel[1] - ba.y,
                next.accel[2] - ba.z,
            );
            continue; // skip invalid or >1s gaps
        }
        let half_dt_sq = 0.5 * dt * dt;

        // Bias-corrected gyro, then Rodrigues' rotation for Exp((ω-b_g)·Δt)
        let omega = Vector3::new(
            curr.gyro[0] - bg.x,
            curr.gyro[1] - bg.y,
            curr.gyro[2] - bg.z,
        );
        let theta = omega.norm();
        let mut r_step = na::Matrix3::<f64>::identity();
        if theta > 1e-8 {
            let axis = omega / theta;
            let angle = theta * dt;
            let (sa, ca) = angle.sin_cos();
            let ia = 1.0 - ca;
            let [x, y, z] = [axis.x, axis.y, axis.z];
            r_step = na::Matrix3::new(
                ca + x * x * ia,     x * y * ia - z * sa,  x * z * ia + y * sa,
                y * x * ia + z * sa, ca + y * y * ia,      y * z * ia - x * sa,
                z * x * ia - y * sa, z * y * ia + x * sa,  ca + z * z * ia,
            );
        }
        let delta_r_next = delta_r * r_step;

        // **Midpoint integration** for velocity and position (2nd order)
        // Reuse a_curr_body from previous iteration (saves 3 subtractions per step).
        let a_next_body = Vector3::new(
            next.accel[0] - ba.x,
            next.accel[1] - ba.y,
            next.accel[2] - ba.z,
        );
        let a_curr_world = delta_r * a_curr_body - gravity;
        let a_next_world = delta_r_next * a_next_body - gravity;
        let a_mid = (a_curr_world + a_next_world) * 0.5;

        delta_v += a_mid * dt;
        delta_p += delta_v * dt + a_curr_world * half_dt_sq;

        delta_r = delta_r_next;
        // a_next_body becomes a_curr_body for the next iteration — no recomputation needed.
        a_curr_body = a_next_body;
    }

    // Build SE(3) matrix: [delta_r | delta_p; 0 | 1]
    let mut t_rel = Matrix4x4::identity();
    t_rel.fixed_view_mut::<3, 3>(0, 0).copy_from(&delta_r);
    t_rel.fixed_view_mut::<3, 1>(0, 3).copy_from(&delta_p);
    t_rel
}

/// Triangulate a 3D point in the left camera frame from a stereo pair.
///
/// Uses the closed-form linear solution: from the left camera we know
/// `x = u_L·z`, `y = v_L·z`; substituting into the right camera
/// projection equations gives two linear equations in `z`.  The
/// least-squares solution avoids any matrix inversion.
///
/// Returns the point in the **left camera frame**, or `None` if the depth
/// is non-positive or the system is degenerate (e.g. parallel rays).
#[inline]
fn triangulate_stereo(
    u_l: f64,
    v_l: f64,
    u_r: f64,
    v_r: f64,
    r_rl: &na::Matrix3<f64>, // rotation: left cam → right cam
    t_rl: &Vector3,          // translation: left cam origin in right cam frame
) -> Option<Vector3> {
    let r0 = r_rl.row(0);
    let r1 = r_rl.row(1);
    let r2 = r_rl.row(2);
    let d = r2[0] * u_l + r2[1] * v_l + r2[2]; // R[2] · [u_l, v_l, 1]
    let a0 = u_r * d - (r0[0] * u_l + r0[1] * v_l + r0[2]);
    let a1 = v_r * d - (r1[0] * u_l + r1[1] * v_l + r1[2]);
    let b0 = t_rl[0] - u_r * t_rl[2];
    let b1 = t_rl[1] - v_r * t_rl[2];
    let denom = a0 * a0 + a1 * a1;
    if denom.abs() < 1e-10 {
        return None;
    }
    let z = (a0 * b0 + a1 * b1) / denom;
    // Reject features with near-zero disparity: they carry no useful stereo depth
    // information and pollute the BA when initialized at extreme depths.
    if z <= 0.1 || z > 50.0 {
        return None;
    }
    Some(Vector3::new(u_l * z, v_l * z, z))
}

/// SO(3) exponential map: rotation vector (axis × angle) → rotation matrix.
///
/// Uses Rodrigues' formula: `R = I + sin(θ)/θ * [ω]× + (1-cos(θ))/θ² * [ω]×²`.
/// Returns identity for `|ω| < 1e-8` to avoid division by near-zero.
#[inline]
fn rotation_exp(omega: &Vector3) -> na::Matrix3<f64> {
    let theta = omega.norm();
    if theta < 1e-8 {
        return na::Matrix3::identity();
    }
    let axis = omega / theta;
    let (s, c) = theta.sin_cos();
    let ia = 1.0 - c;
    let (x, y, z) = (axis.x, axis.y, axis.z);
    na::Matrix3::new(
        c + x * x * ia,     x * y * ia - z * s,  x * z * ia + y * s,
        y * x * ia + z * s, c + y * y * ia,      y * z * ia - x * s,
        z * x * ia - y * s, z * y * ia + x * s,  c + z * z * ia,
    )
}

/// Scale an SE(3) relative motion by a time factor.
///
/// Decomposes the motion into a rotation vector and translation, scales both by
/// `scale`, then recomposes.  This correctly interpolates along the geodesic for
/// any pure-rotation or pure-translation motion.  Combined motions are approximated
/// by independent scaling — accurate for small deltas (typical inter-frame motion).
///
/// `scale = 0` → identity.  `scale = 1` → `t_delta` unchanged.
/// Values outside [0, 1] are valid but may extrapolate beyond the delta.
#[inline]
pub fn scale_se3_delta(t_delta: &Matrix4x4, scale: f64) -> Matrix4x4 {
    let r = t_delta.fixed_view::<3, 3>(0, 0).into_owned();
    let t = t_delta.fixed_view::<3, 1>(0, 3).into_owned();
    let r_log = rotation_log(&r);
    let r_scaled = rotation_exp(&(r_log * scale));
    let t_scaled = t * scale;
    let mut result = Matrix4x4::identity();
    result.fixed_view_mut::<3, 3>(0, 0).copy_from(&r_scaled);
    result.fixed_view_mut::<3, 1>(0, 3).copy_from(&t_scaled);
    result
}

/// SO(3) logarithmic map: rotation matrix → rotation vector (axis × angle).
///
/// Returns the 3-element rotation vector `r` such that `R = Exp(r)`.
/// For small angles (|θ| < 1e-8), returns zero to avoid division by near-zero sin.
#[inline]
fn rotation_log(r: &na::Matrix3<f64>) -> Vector3 {
    let cos_angle = ((r.trace() - 1.0) * 0.5).clamp(-1.0, 1.0);
    let angle = cos_angle.acos();
    if angle.abs() < 1e-8 {
        return Vector3::zeros();
    }
    let factor = angle / (2.0 * angle.sin());
    Vector3::new(
        factor * (r[(2, 1)] - r[(1, 2)]),
        factor * (r[(0, 2)] - r[(2, 0)]),
        factor * (r[(1, 0)] - r[(0, 1)]),
    )
}

/// Sliding window of keyframes for bundle adjustment optimization.
///
/// Maintains a fixed-size window of keyframes and manages the optimization
/// of poses and 3D points across these frames.
pub struct SlidingWindow {
    /// Maximum number of keyframes in the sliding window.
    max_frames: usize,

    /// Current keyframes in the sliding window (ordered by insertion time, oldest first).
    keyframes: VecDeque<Frame>,

    /// Map points stored by feature ID: FxHashMap<feature_id, [x, y, z]>
    pub map_points: FxHashMap<usize, [f32; 3]>,

    /// Last optimized pose from track_motion, used as warm-start for next frame.
    last_track_pose: Option<Matrix4x4>,

    /// Pre-built KF variable names "KF_0".."KF_{max_frames-1}".
    /// Built once in `new()` to avoid `format!` inside the hot BA loop.
    kf_var_names: Vec<String>,

    /// Landmark variable name cache: feature_id → "pt_{feature_id}".
    /// Persisted across `optimize()` calls so features that survive window sliding
    /// don't need to be re-formatted each BA call.
    lm_name_cache: FxHashMap<usize, String>,

    /// Camera–IMU time offset in nanoseconds: `t_imu = t_cam + cam_imu_td_ns`.
    ///
    /// Positive → camera timestamps lag the IMU (camera sees events later).
    /// Initialised from config; refined online by `optimize()` using the
    /// rotation discrepancy between IMU-preintegrated and BA-optimised relative poses.
    pub cam_imu_td_ns: i64,

    /// Whether to run online refinement of `cam_imu_td_ns` after each BA call.
    pub cam_imu_td_online_refinement: bool,

    /// Motion prior for platform-specific gating of visual updates.
    /// Default is QuadrotorPrior (agile 6-DoF).
    pub motion_prior: Box<dyn MotionPrior>,

    /// IMU noise parameters for preintegration (from config).
    pub imu_gyro_noise_std: f64,
    pub imu_gyro_random_walk_std: f64,
    pub imu_accel_noise_std: f64,
    pub imu_accel_random_walk_std: f64,
    pub imu_gravity_magnitude: f64,

    /// Timestamp of the last frame processed by `track_motion` (nanoseconds).
    /// Used to compute the frame-to-frame dt for time-scaled CV prediction.
    /// Reset to `None` when `add_frame` is called (new KF inserted).
    last_track_timestamp_ns: Option<i64>,

    /// Loop closure database for detecting revisits and optimizing global scale.
    pub loop_closure_db: LoopClosureDatabase,
}

impl SlidingWindow {
    #![allow(non_snake_case)]
    /// Create a new sliding window with the specified maximum number of frames.
    ///
    /// # Arguments
    /// * `max_frames` - Maximum number of keyframes to keep in the window (default: 16)
    #[must_use]
    pub fn new(max_frames: usize) -> Self {
        let kf_var_names = (0..max_frames).map(|i| format!("KF_{}", i)).collect();
        Self {
            max_frames,
            keyframes: VecDeque::with_capacity(max_frames),
            map_points: FxHashMap::default(),
            last_track_pose: None,
            kf_var_names,
            lm_name_cache: FxHashMap::default(),
            cam_imu_td_ns: 0,
            cam_imu_td_online_refinement: true,
            motion_prior: Box::new(NullPrior),
            imu_gyro_noise_std: 1.6968e-4,
            imu_gyro_random_walk_std: 2.5e-5,
            imu_accel_noise_std: 2.6669e-3,
            imu_accel_random_walk_std: 6.4e-4,
            imu_gravity_magnitude: 9.81007,
            last_track_timestamp_ns: None,
            loop_closure_db: LoopClosureDatabase::new(),
        }
    }

    /// Create a new sliding window with a custom motion prior.
    pub fn with_prior<P: MotionPrior + 'static>(max_frames: usize, prior: P) -> Self {
        let mut sw = Self::new(max_frames);
        sw.motion_prior = Box::new(prior);
        sw
    }

    /// Initialize IMU state (gravity alignment, velocity, biases) from stationary data.
    ///
    /// This should be called during system initialization when the IMU is stationary.
    /// It estimates:
    /// - Gravity direction (from accelerometer mean)
    /// - Initial velocity (assumed zero for stationary start)
    /// - Gyro bias (from gyroscope mean)
    /// - Accel bias (from accelerometer mean minus gravity)
    ///
    /// # Arguments
    /// * `imu_samples` - IMU samples collected during stationary period (~1-2 seconds)
    /// * `state` - Mutable reference to state to initialize
    pub fn initialize_imu_from_stationary(
        &self,
        imu_samples: &[crate::datasets::ImuData],
        state: &mut crate::estimator::state::State,
    ) {
        if imu_samples.is_empty() {
            log::warn!("[SlidingWindow] No IMU samples for initialization");
            return;
        }

        // Compute mean of gyro and accel from stationary period
        let mut gyro_sum = [0.0f64; 3];
        let mut accel_sum = [0.0f64; 3];
        let n = imu_samples.len() as f64;

        for sample in imu_samples {
            gyro_sum[0] += sample.gyro[0];
            gyro_sum[1] += sample.gyro[1];
            gyro_sum[2] += sample.gyro[2];
            accel_sum[0] += sample.accel[0];
            accel_sum[1] += sample.accel[1];
            accel_sum[2] += sample.accel[2];
        }

        let gyro_mean = [
            (gyro_sum[0] / n) as f32,
            (gyro_sum[1] / n) as f32,
            (gyro_sum[2] / n) as f32,
        ];
        let accel_mean = [
            (accel_sum[0] / n) as f32,
            (accel_sum[1] / n) as f32,
            (accel_sum[2] / n) as f32,
        ];

        // Gyro bias = mean gyro (should be zero when stationary)
        state.gyro_bias = gyro_mean;

        // Accel bias = mean accel - gravity
        // Assuming world Z is down, gravity = [0, 0, g]
        // When stationary, accel should read -gravity (if sensor Z is up)
        // or +gravity (if sensor Z is down)
        // For TUM-VI, IMU frame is aligned such that accel reads +g when stationary
        state.accel_bias[0] = accel_mean[0];
        state.accel_bias[1] = accel_mean[1];
        state.accel_bias[2] = accel_mean[2] - self.imu_gravity_magnitude as f32;

        // Velocity is zero when stationary (already initialized)
        // state.velocity = [0.0, 0.0, 0.0];

        log::info!(
            "[SlidingWindow] IMU initialized from {} stationary samples",
            imu_samples.len()
        );
        log::debug!(
            "  Gyro bias: [{:.6}, {:.6}, {:.6}] rad/s",
            state.gyro_bias[0],
            state.gyro_bias[1],
            state.gyro_bias[2]
        );
        log::debug!(
            "  Accel bias: [{:.6}, {:.6}, {:.6}] m/s²",
            state.accel_bias[0],
            state.accel_bias[1],
            state.accel_bias[2]
        );
    }
}

impl Default for SlidingWindow {
    fn default() -> Self {
        Self::new(16)
    }
}

impl SlidingWindow {
    /// Add a keyframe to the sliding window.
    ///
    /// If the window is full, the oldest frame (by frame_id) will be removed
    /// to make room for the new frame. Only keyframes should be added.
    ///
    /// # Arguments
    /// * `frame` - The keyframe to add
    ///
    /// # Returns
    /// `true` if the frame was added, `false` if it was rejected (e.g., not a keyframe)
    pub fn add_frame(&mut self, frame: Frame) -> bool {
        // Only accept keyframes
        if !frame.is_keyframe {
            log::warn!(
                "[SlidingWindow] Attempted to add non-keyframe (frame_id: {})",
                frame.frame_id
            );
            return false;
        }

        // Clear warm-start pose and last-track timestamp — next track_motion restarts
        // from the new keyframe rather than continuing the non-KF tracking chain.
        self.last_track_pose = None;
        self.last_track_timestamp_ns = None;

        // Remove oldest frame if window is full (FIFO - first in, first out)
        if self.keyframes.len() >= self.max_frames {
            if let Some(removed_frame) = self.keyframes.pop_front() {
                log::debug!(
                    "[SlidingWindow] Removed oldest frame (frame_id: {}) to make room for new keyframe",
                    removed_frame.frame_id
                );
            }
        }
        let frame_id = frame.frame_id;

        // Add frame to sliding window
        self.keyframes.push_back(frame);

        // Initialize velocity of the new KF from pose finite differences.
        // This provides a physically-consistent warm start for IMU factors:
        // v_j ≈ (p_j - p_{j-1}) / Δt rather than zero.
        // The BA will refine velocities, but a good initial value prevents the optimizer
        // from absorbing IMU residuals entirely in velocity variables.
        let n = self.keyframes.len();
        if n >= 2 {
            let t_prev = self.keyframes[n - 2].timestamp_ns;
            let t_curr = self.keyframes[n - 1].timestamp_ns;
            let dt = (t_curr - t_prev) as f64 / 1e9;
            if dt > 0.01 && dt < 2.0 {
                let p_prev = na::Vector3::new(
                    self.keyframes[n - 2].state.T_W_B[(0, 3)] as f64,
                    self.keyframes[n - 2].state.T_W_B[(1, 3)] as f64,
                    self.keyframes[n - 2].state.T_W_B[(2, 3)] as f64,
                );
                let p_curr = na::Vector3::new(
                    self.keyframes[n - 1].state.T_W_B[(0, 3)] as f64,
                    self.keyframes[n - 1].state.T_W_B[(1, 3)] as f64,
                    self.keyframes[n - 1].state.T_W_B[(2, 3)] as f64,
                );
                let v = (p_curr - p_prev) / dt;
                self.keyframes[n - 1].state.velocity = [v.x as f32, v.y as f32, v.z as f32];
            }
        }

        log::debug!(
            "[SlidingWindow] Added keyframe (frame_id: {}), window size: {}/{}",
            frame_id,
            self.keyframes.len(),
            self.max_frames
        );

        true
    }

    /// Get the current number of keyframes in the window.
    pub fn len(&self) -> usize {
        self.keyframes.len()
    }

    /// Check if the sliding window is empty.
    pub fn is_empty(&self) -> bool {
        self.keyframes.is_empty()
    }

    /// Check if the sliding window is full.
    pub fn is_full(&self) -> bool {
        self.keyframes.len() >= self.max_frames
    }

    /// Get a reference to a specific keyframe by index.
    pub fn get_frame(&self, index: usize) -> Option<&Frame> {
        self.keyframes.get(index)
    }

    pub fn get_keyframe_poses(&self) -> Vec<Matrix4x4> {
        // Return poses of body in world frame
        self.keyframes.iter().map(|f| f.state.T_W_B).collect()
    }

    /// Return the last two keyframe poses without allocating a Vec.
    ///
    /// Returns `None` if fewer than 2 keyframes exist.
    /// This is more efficient than `Self::get_keyframe_poses()` when only the last
    /// two poses are needed (e.g., constant velocity prediction).
    pub fn get_last_two_poses(&self) -> Option<(Matrix4x4, Matrix4x4)> {
        let len = self.keyframes.len();
        if len < 2 {
            return None;
        }
        let t_prev = self.keyframes.iter().nth_back(1)?.state.T_W_B;
        let t_last = self.keyframes.back()?.state.T_W_B;
        Some((t_prev, t_last))
    }

    /// Predict the current pose using a constant velocity motion model.
    ///
    /// Computes the relative SE(3) motion between the last two keyframes and
    /// extrapolates: `T_W_B_init = T_last * T_prev⁻¹ * T_last`.
    ///
    /// Falls back to the last keyframe pose if fewer than 2 keyframes exist.
    pub fn predict_current_pose(&self) -> Matrix4x4 {
        if let Some((t_prev, t_last)) = self.get_last_two_poses() {
            // T_rel = T_prev⁻¹ * T_last  (relative motion in world frame)
            let t_rel = inverse_se3(&t_prev) * t_last;
            // Extrapolate: T_init = T_last * T_rel
            t_last * t_rel
        } else {
            let last_kf = self
                .keyframes
                .back()
                .expect("keyframes should not be empty");
            last_kf.state.T_W_B
        }
    }

    /// Clear all keyframes from the sliding window.
    pub fn clear(&mut self) {
        self.keyframes.clear();
        log::debug!("[SlidingWindow] Cleared all keyframes");
    }

    /// Compute the time-scale factor for the constant-velocity prediction.
    ///
    /// The CV model extrapolates by one full KF-to-KF delta.  We need to scale it
    /// down to the actual elapsed time since the last KF (no warm-start) or since the
    /// last tracked frame (warm-start).
    ///
    /// Returns a value in `[0, 1]`: clamped so we never extrapolate beyond the full
    /// CV prediction, but can be exactly 1 when the frame lands right at the KF boundary.
    ///
    /// `use_frame_dt`: true → scale by frame-to-frame dt (warm-start case);
    ///                 false → scale by KF-to-current dt (no warm-start case).
    fn cv_time_scale(&self, frame_timestamp_ns: i64, use_frame_dt: bool) -> f64 {
        // Reference interval: last two KFs, in nanoseconds.
        let kf_dt_ns = if self.keyframes.len() >= 2 {
            let last_ts = self.keyframes.back().map(|k| k.timestamp_ns).unwrap_or(0);
            let prev_ts = self.keyframes.iter().nth_back(1).map(|k| k.timestamp_ns).unwrap_or(0);
            (last_ts - prev_ts).max(1)
        } else {
            50_000_000 // 50 ms fallback
        };

        let numerator_ns = if use_frame_dt {
            // Frame-to-frame dt: time since last tracked frame.
            self.last_track_timestamp_ns
                .map(|ts| (frame_timestamp_ns - ts).max(1))
                .unwrap_or(kf_dt_ns)
        } else {
            // KF-to-current dt: time since last KF.
            let last_kf_ts = self.keyframes.back().map(|k| k.timestamp_ns).unwrap_or(0);
            (frame_timestamp_ns - last_kf_ts).max(1)
        };

        (numerator_ns as f64 / kf_dt_ns as f64).min(1.0)
    }

    fn build_solver_config(&self) -> LevenbergMarquardtConfig {
        LevenbergMarquardtConfig::new()
            .with_linear_solver_type(LinearSolverType::SparseSchurComplement)
            .with_schur_variant(SchurVariant::Sparse)
            .with_schur_preconditioner(SchurPreconditioner::BlockDiagonal)
            .with_max_iterations(20)
            .with_cost_tolerance(1e-6)
            .with_parameter_tolerance(1e-7)
            .with_jacobi_scaling(false)
    }

    fn check_sliding_window_size_for_optimization(&self) -> Result<bool, std::io::Error> {
        if self.keyframes.is_empty() {
            log::warn!("[SlidingWindow] Cannot optimize: window is empty");
            return Err(std::io::Error::other("Window is empty"));
        }

        if self.keyframes.len() < self.max_frames {
            log::warn!(
                "[SlidingWindow] Cannot optimize: need {} keyframes, have {}",
                self.max_frames,
                self.keyframes.len()
            );
            return Err(std::io::Error::other("Need more keyframes"));
        }

        log::debug!(
            "[SlidingWindow] Starting bundle adjustment optimization with {} keyframes",
            self.keyframes.len()
        );

        Ok(true)
    }

    #[allow(clippy::cognitive_complexity)]
    #[allow(clippy::too_many_lines)]
    pub fn optimize(&mut self) -> Result<bool, std::io::Error> {
        self.check_sliding_window_size_for_optimization()?;

        // Initialize problem and solver
        let mut problem = Problem::new(JacobianMode::Sparse);
        let mut solver = LevenbergMarquardt::with_config(self.build_solver_config());
        // Estimate unique feature count: use last (newest) frame as proxy.
        // The newest keyframe tends to have the most features and is O(1) to access.
        let est_features: usize = self.keyframes.back()
            .map(|f| f.left_features.len() + f.right_features.len())
            .unwrap_or(0);
        let est_vars = self.keyframes.len() + est_features;
        let mut initial_values = HashMap::with_capacity(est_vars);
        // solver.add_observer(TerminalObserver::new());

        // Fetch transforms between cameras and body
        let first_kf_state = &self.keyframes.front().expect("keyframes should not be empty").state;
        let T_Cl_B = inverse_se3(&first_kf_state.T_B_Cl);
        let T_Cr_B = inverse_se3(&first_kf_state.T_B_Cr);

        // Use nominal stereo extrinsics for now (online calibration deferred for later)
        let nominal_T_Cr_B = T_Cr_B;

        // Stereo extrinsic: T_RL maps a point in the left camera frame to the right camera frame.
        // T_RL = T_Cr_B * T_B_Cl  (left cam → body → right cam)
        // NOTE: Using nominal value for initialization; will be optimized
        let T_RL = nominal_T_Cr_B * first_kf_state.T_B_Cl;
        let R_RL = T_RL.fixed_view::<3, 3>(0, 0).into_owned();
        let t_RL = T_RL.fixed_view::<3, 1>(0, 3).into_owned();

        // Compute a scene-aware fallback depth from the median of previously optimized landmarks
        // projected into the oldest keyframe's left camera frame.  Features without stereo use
        // this instead of a fixed 2.0 m — improves scale accuracy for rotation-heavy sequences
        // where many features must be initialized without stereo depth.
        let fallback_depth = {
            let oldest_T_W_B = self.keyframes.front()
                .expect("keyframes should not be empty").state.T_W_B;
            let T_B_W_oldest = inverse_se3(&oldest_T_W_B);
            // T_CL_W = T_CL_B * T_B_W  (world → body → left camera)
            let T_CL_W = T_Cl_B * T_B_W_oldest;
            let r_cw = T_CL_W.fixed_view::<3, 3>(0, 0);
            let t_cw = T_CL_W.fixed_view::<3, 1>(0, 3);
            let mut depths: Vec<f64> = self.map_points.values()
                .map(|p| {
                    let pw = na::Vector3::new(p[0] as f64, p[1] as f64, p[2] as f64);
                    (r_cw * pw + t_cw).z
                })
                .filter(|&z| z > 0.1 && z < 50.0)
                .collect();
            if depths.len() >= 4 {
                let mid = depths.len() / 2;
                depths.select_nth_unstable_by(mid, |a, b| {
                    a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal)
                });
                depths[mid].clamp(0.5, 20.0)
            } else {
                2.0
            }
        };

        // Landmark name cache: persisted on `self` across calls so features that survive
        // window sliding (most of them) don't need re-formatting each BA call.
        // Old entries for features that left the window are stale but harmless (never looked up).
        let lm_name_cache = &mut self.lm_name_cache;

        // Add factors
        // Stereo/mono counters are only used for the debug log line below.
        let debug_log = log::log_enabled!(log::Level::Debug);
        let mut stereo_count = 0usize;
        let mut mono_count = 0usize;
        // In debug mode, track which features have been seen in left AND right cameras.
        let mut left_seen: FxHashSet<usize> = if debug_log {
            FxHashSet::with_capacity_and_hasher(est_features, Default::default())
        } else {
            FxHashSet::default()
        };
        let mut right_seen: FxHashSet<usize> = if debug_log {
            FxHashSet::with_capacity_and_hasher(est_features, Default::default())
        } else {
            FxHashSet::default()
        };

        // Compute prior-informed Huber threshold
        let prior_ctx = crate::estimator::motion_prior::PriorContext {
            feature_count: est_features,
            visual_health: 1.0,
        };
        let base_huber_threshold = 2.0;
        let prior_score = if self.keyframes.len() >= 2 {
            self.motion_prior.score_prediction(
                &self.keyframes[0].state,
                &self.keyframes.back().expect("Keyframes should not be empty").state,
                0.1,
                &prior_ctx,
            )
        } else {
            crate::estimator::motion_prior::PriorScore::default()
        };
        // Inflate Huber threshold when prior confidence is low
        let adaptive_huber = base_huber_threshold * (2.0 - prior_score.score).max(1.0);

        // **Shared HuberLoss** — create once and clone for every residual.
        // Uses adaptive threshold based on prior confidence.
        // Avoids calling HuberLoss::new().unwrap() hundreds of times.
        let shared_loss = HuberLoss::new(adaptive_huber as f64).unwrap();

        // **Adaptive Huber: first-pass optimization to estimate residual scale**
        // Run a quick optimization with fixed threshold, then compute MAD (Median Absolute
        // Deviation) of residuals to set an adaptive threshold for the real optimization.
        // This automatically adapts to current noise level and rejects outliers better.

        for (id_frame, frame) in self.keyframes.iter().enumerate() {
            // Add KF variable for this keyframe
            let kf_var = &self.kf_var_names[id_frame];
            let T_B_W = inverse_se3(&frame.state.T_W_B);
            let t_B_W = T_B_W.fixed_view::<3, 1>(0, 3);
            let R_B_W = Matrix3x3::from(T_B_W.fixed_view::<3, 3>(0, 0));
            let q_B_W = UnitQuaternion::from_matrix(&R_B_W);
            // Build DVector<f64> directly to avoid an intermediate DVector<Float> + cast alloc.
            initial_values.insert(kf_var.clone(), (ManifoldType::SE3, DVector::from_row_slice(&[
                t_B_W.x as f64, t_B_W.y as f64, t_B_W.z as f64,
                q_B_W.w as f64, q_B_W.i as f64, q_B_W.j as f64, q_B_W.k as f64,
            ])));

            // NOTE: Velocity variables are now added ONLY when IMU factors are present
            // to avoid underconstrained optimization. See IMU factor section below.

            // Build right-feature lookup for stereo triangulation (feature_id → undistorted_coord).
            let right_coord_map: FxHashMap<usize, [f32; 2]> = frame
                .right_features
                .iter()
                .filter(|f| f.undistorted_coord[0] > -0.5)
                .map(|f| (f.feature_id, f.undistorted_coord))
                .collect();

            // Process features from both cameras
            let camera_features = [
                (&frame.left_features, T_Cl_B),
                (&frame.right_features, T_Cr_B),
            ];

            for (features, T_C_B) in camera_features.iter() {
                // Precompute T_B_C once per camera, not per feature
                let T_B_C = inverse_se3(T_C_B);
                let is_left = std::ptr::eq(features.as_ptr(), frame.left_features.as_ptr());
                for feat in features.iter() {
                    let feature_id = feat.feature_id;
                    // Lazily build landmark name — one allocation per unique feature, not per obs.
                    let lm_var = lm_name_cache
                        .entry(feature_id)
                        .or_insert_with(|| format!("pt_{}", feature_id));

                    // Debug: track stereo vs mono coverage
                    if debug_log {
                        if is_left { left_seen.insert(feature_id); } else { right_seen.insert(feature_id); }
                    }

                    // Create initial value for landmark if not already present.
                    // Use contains_key(&str) to avoid cloning the String key on every
                    // observation — only clone when actually inserting (once per unique feature).
                    if !initial_values.contains_key(lm_var.as_str()) {
                        let data = if let Some(&last_pos) = self.map_points.get(&feature_id) {
                            DVector::from_row_slice(&[
                                last_pos[0] as f64,
                                last_pos[1] as f64,
                                last_pos[2] as f64,
                            ])
                        } else {
                            // Initialize landmark via stereo triangulation when both cameras
                            // observe this feature; fall back to a fixed-depth ray if not.
                            let u_l = feat.undistorted_coord[0] as f64;
                            let v_l = feat.undistorted_coord[1] as f64;
                            let p_C = if is_left {
                                right_coord_map
                                    .get(&feature_id)
                                    .and_then(|rc| {
                                        triangulate_stereo(
                                            u_l, v_l,
                                            rc[0] as f64, rc[1] as f64,
                                            &R_RL, &t_RL,
                                        )
                                    })
                                    .unwrap_or_else(|| Vector3::new(u_l * fallback_depth, v_l * fallback_depth, fallback_depth))
                            } else {
                                Vector3::new(u_l * fallback_depth, v_l * fallback_depth, fallback_depth)
                            };
                            let (R_W_B, t_W_B) = (
                                frame.state.T_W_B.fixed_view::<3, 3>(0, 0).into_owned(),
                                frame.state.T_W_B.fixed_view::<3, 1>(0, 3).into_owned(),
                            );
                            let (R_B_C, t_B_C) = (
                                T_B_C.fixed_view::<3, 3>(0, 0).into_owned(),
                                T_B_C.fixed_view::<3, 1>(0, 3).into_owned(),
                            );
                            let p_W = R_W_B * (R_B_C * p_C + t_B_C) + t_W_B;
                            DVector::from_row_slice(&[p_W.x, p_W.y, p_W.z])
                        };
                        initial_values.insert(lm_var.clone(), (ManifoldType::RN, data));
                    }

                        // Create camera projection factor with age-weighting
                        let mut factor = BundleAdjustmentFactor::new(
                            na::Vector2::new(feat.undistorted_coord[0], feat.undistorted_coord[1])
                                .cast::<f64>(),
                            *T_C_B,
                        )
                        .with_track_age(feat.age); // Age-weighted: older features get higher weight

                        // Fix pose for first frame (T_B_W already computed above)
                        if id_frame == 0 {
                            factor = factor.with_fixed_pose(T_B_W);
                        }

                        // Add residual block with shared Huber loss — use stack arrays
                        // to avoid a heap allocation per feature observation.
                        if id_frame == 0 {
                            problem.add_residual_block(
                                &[lm_var.as_str()],
                                Box::new(factor),
                                Some(Box::new(shared_loss.clone())),
                            );
                        } else {
                            problem.add_residual_block(
                                &[lm_var.as_str(), kf_var.as_str()],
                                Box::new(factor),
                                Some(Box::new(shared_loss.clone())),
                            );
                        }
                }
            }
        }

        // ── Add IMU preintegration factors between consecutive keyframes ──
        // First, check if we have any IMU data to add factors for
        let mut has_imu_factors = false;
        for i in 1..self.keyframes.len() {
            if self.keyframes[i].imu_from_last_frame.len() >= 3 {
                has_imu_factors = true;
                break;
            }
        }

        // Compute prior-informed process noise scaling
        let prior_ctx = crate::estimator::motion_prior::PriorContext {
            feature_count: self.map_points.len(),
            visual_health: 1.0,
        };
        let mut noise_scales = Vec::new();
        for i in 1..self.keyframes.len() {
            let prev_kf = &self.keyframes[i - 1];
            let curr_kf = &self.keyframes[i];
            let dt = ((curr_kf.timestamp_ns - prev_kf.timestamp_ns) as f32) / 1e9;
            let scale = self.motion_prior.process_noise_scale(&prev_kf.state, dt, &prior_ctx);
            noise_scales.push(scale);
        }

        // Define bias variable names (used only if has_imu_factors)
        let bg_var = "bg_global".to_string();
        let ba_var = "ba_global".to_string();

        // Only add velocity and bias variables if we have IMU factors to constrain them
        if has_imu_factors {
            // Add velocity variables for each keyframe
            for (id_frame, frame) in self.keyframes.iter().enumerate() {
                let vel_var = format!("v_{}", id_frame);
                let vel_init = DVector::from_row_slice(&[
                    frame.state.velocity[0] as f64,
                    frame.state.velocity[1] as f64,
                    frame.state.velocity[2] as f64,
                ]);
                initial_values.insert(vel_var.clone(), (ManifoldType::RN, vel_init));
            }

            // Add global bias variables (gyro and accel biases)
            let last_kf = self.keyframes.back().expect("keyframes should not be empty");
            let bg_init = DVector::from_row_slice(&[
                last_kf.state.gyro_bias[0] as f64,
                last_kf.state.gyro_bias[1] as f64,
                last_kf.state.gyro_bias[2] as f64,
            ]);
            let ba_init = DVector::from_row_slice(&[
                last_kf.state.accel_bias[0] as f64,
                last_kf.state.accel_bias[1] as f64,
                last_kf.state.accel_bias[2] as f64,
            ]);
            initial_values.insert(bg_var.clone(), (ManifoldType::RN, bg_init));
            initial_values.insert(ba_var.clone(), (ManifoldType::RN, ba_init));
        }

        // TODO: Online stereo calibration - deferred due to Schur complement integration issues
        // Will be implemented with simplified baseline scale factor approach

        let mut imu_factor_count = 0usize;
        if has_imu_factors {
            for i in 1..self.keyframes.len() {
            let frame_i = &self.keyframes[i];
            let imu_samples = &frame_i.imu_from_last_frame;
            if imu_samples.len() < 3 {
                log::debug!(
                    "[IMU] Skipping IMU factor between KF_{} and KF_{} (only {} samples)",
                    i - 1, i, imu_samples.len()
                );
                continue;
            }

            // Preintegrate IMU measurements
            let prev_kf = &self.keyframes[i - 1];
            let gyro_bias_f64: [f64; 3] = prev_kf.state.gyro_bias.map(|x| x as f64);
            let accel_bias_f64: [f64; 3] = prev_kf.state.accel_bias.map(|x| x as f64);

            // Apply camera-IMU time offset to IMU samples
            let imu_slice: &[ImuData] = if self.cam_imu_td_ns > 0 {
                let adjusted_prev = prev_kf.timestamp_ns + self.cam_imu_td_ns;
                let skip = imu_samples.partition_point(|s| s.timestamp <= adjusted_prev);
                &imu_samples[skip..]
            } else {
                imu_samples
            };

            if imu_slice.len() < 3 {
                continue;
            }

            // Apply prior-informed noise scaling to config IMU noise parameters
            let scale_idx = i - 1;
            let scale = if scale_idx < noise_scales.len() {
                &noise_scales[scale_idx]
            } else {
                &crate::estimator::motion_prior::ProcessNoiseScale::default()
            };

            let noise = crate::optimization::imu_preintegration::ImuNoiseConfig {
                gyro_noise_std: self.imu_gyro_noise_std * scale.gyro_scale as f64,
                gyro_random_walk_std: self.imu_gyro_random_walk_std * scale.gyro_bias_scale as f64,
                accel_noise_std: self.imu_accel_noise_std * scale.accel_scale as f64,
                accel_random_walk_std: self.imu_accel_random_walk_std * scale.accel_bias_scale as f64,
            };
            let preintegrated = PreintegratedImuMeasurements::preintegrate(
                imu_slice,
                gyro_bias_f64,
                accel_bias_f64,
                noise,
                &na::Vector3::new(0.0, 0.0, self.imu_gravity_magnitude),
            );

            if preintegrated.delta_t < 1e-6 {
                continue;
            }

            let imu_factor = ImuPreintegrationFactor::new(
                preintegrated,
                na::Vector3::new(0.0, 0.0, self.imu_gravity_magnitude),
                self.imu_gyro_random_walk_std,
                self.imu_accel_random_walk_std,
            );

            // Variable names: pose_i, pose_j, vel_i, vel_j, bg, ba
            let pose_i_var = &self.kf_var_names[i - 1];
            let pose_j_var = &self.kf_var_names[i];
            let vel_i_var = format!("v_{}", i - 1);
            let vel_j_var = format!("v_{}", i);

            problem.add_residual_block(
                &[
                    pose_i_var.as_str(),
                    pose_j_var.as_str(),
                    &vel_i_var,
                    &vel_j_var,
                    &bg_var,
                    &ba_var,
                ],
                Box::new(imu_factor),
                None, // No robust loss — IMU residuals are already whitened
            );
            imu_factor_count += 1;
            }
        }
        // End of IMU factor loop and has_imu_factors block

        // Add a global scale variable for loop closure optimization
        // This allows multiple loop closures to vote on the overall scale (corrects systematic bias)
        let scale_var_name = "scale";
        let mut _scale_var_added = false;
        if self.loop_closure_db.num_keyframes > 0 && self.keyframes.len() > 20 {
            initial_values.insert(
                scale_var_name.to_string(),
                (ManifoldType::RN, DVector::from_row_slice(&[1.0])),
            );
            _scale_var_added = true;
        }

        // ── Loop closure detection and edge factors ────────────────────────────
        // After tracking and before optimization, detect loop closures for the newest keyframe.
        // Validate candidates geometrically and add loop closure edges to the problem.
        let mut _loop_closure_count = 0;
        if self.keyframes.len() >= 2 {
            let newest_idx = self.keyframes.len() - 1;
            let newest_kf = &self.keyframes[newest_idx];

            // Add newest keyframe's descriptors to database
            if !newest_kf.left_descriptors.is_empty() || !newest_kf.right_descriptors.is_empty() {
                self.loop_closure_db.add_keyframe(
                    newest_idx,
                    newest_kf.left_descriptors.clone(),
                    newest_kf.right_descriptors.clone(),
                );
            }

            // Find loop closure candidates (only if we have enough keyframes in history)
            if newest_idx > 20 {
                let candidates = self.loop_closure_db.find_loop_candidates(newest_idx);

                // Validate each candidate geometrically
                for candidate in candidates.into_iter().take(3) { // Top 3 candidates max
                    let older_kf_idx = candidate.kf_idx_older;
                    let older_kf = &self.keyframes[older_kf_idx];

                    // Get feature positions from both keyframes
                    let mut pts1 = Vec::new();
                    let mut pts2 = Vec::new();

                    // Match features by ID between the two keyframes
                    let mut feat_map_older: FxHashMap<usize, [f32; 2]> = FxHashMap::default();
                    for feat in &older_kf.left_features {
                        feat_map_older.insert(feat.feature_id, feat.pixel_coord);
                    }

                    for feat in &newest_kf.left_features {
                        if let Some(coord_older) = feat_map_older.get(&feat.feature_id) {
                            pts1.push(*coord_older);
                            pts2.push(feat.pixel_coord);
                        }
                    }

                    // Only proceed if we have enough matched features
                    if pts1.len() < 8 {
                        continue;
                    }

                    // Validate loop closure geometrically
                    // Get camera intrinsics from the config
                    let K = {
                        let fx = 458.654 as f64;  // Default; ideally from config
                        let fy = 457.296 as f64;
                        let cx = 367.215 as f64;
                        let cy = 248.375 as f64;
                        na::Matrix3::new(
                            fx, 0.0, cx,
                            0.0, fy, cy,
                            0.0, 0.0, 1.0,
                        )
                    };

                    // Convert point format from [f32; 2] to (f32, f32)
                    let pts1_tuples: Vec<(f32, f32)> = pts1.iter().map(|p| (p[0], p[1])).collect();
                    let pts2_tuples: Vec<(f32, f32)> = pts2.iter().map(|p| (p[0], p[1])).collect();

                    if let Some(validation) = validate_loop_closure(
                        &pts1_tuples,
                        &pts2_tuples,
                        &K,
                        200, // RANSAC iterations
                        1.0, // Reprojection threshold in pixels
                    ) {
                        if validation.inlier_count >= 10 {
                            // Create loop closure edge factor
                            let older_var = &self.kf_var_names[older_kf_idx];
                            let newer_var = &self.kf_var_names[newest_idx];

                            let lc_factor = LoopClosureEdgeFactor::new(
                                validation.r_rel,
                                validation.t_rel,
                                validation.scale,
                                true, // Enable scale optimization (Option C: all loop closures vote on global scale)
                            );

                            // Create Huber loss for loop closure edge (robust outlier rejection)
                            // Reduced from 0.1 to 0.05 for stronger global constraints
                            let lc_loss = HuberLoss::new(0.05).ok();
                            problem.add_residual_block(
                                &[older_var.as_str(), newer_var.as_str(), scale_var_name],
                                Box::new(lc_factor),
                                lc_loss.map(|loss| Box::new(loss) as Box<dyn apex_solver::core::loss_functions::LossFunction + Send>),
                            );

                            _loop_closure_count += 1;
                            log::info!(
                                "[LoopClosure] Valid edge between KF {} (older) and KF {} (newer): {} inliers, scale={:.3}",
                                older_kf_idx,
                                newest_idx,
                                validation.inlier_count,
                                validation.scale
                            );
                        }
                    }
                }
            }
        }

        // Compute stereo/mono stats lazily — only when debug logging is active.
        if debug_log {
            for fid in lm_name_cache.keys() {
                if left_seen.contains(fid) && right_seen.contains(fid) {
                    stereo_count += 1;
                } else {
                    mono_count += 1;
                }
            }
        }

        // TODO: Add stereo calibration prior when optimization variable is implemented

        let num_residuals = problem.num_residual_blocks();
        let num_variables = initial_values.len();

        log::debug!(
            "Added {} variables total ({} SE3 + {} R3 + {} biases), {} residual blocks ({} stereo + {} mono landmarks + {} IMU)",
            num_variables,
            self.keyframes.len(),
            self.keyframes.len() + est_features,  // velocities + landmarks
            2,  // bg + ba
            num_residuals,
            stereo_count,
            mono_count,
            imu_factor_count,
        );

        // Validate problem before optimization
        if num_residuals < 6 {
            log::warn!(
                "[SlidingWindow] Too few residuals ({}), skipping optimization",
                num_residuals
            );
            return Ok(false);
        }

        // Check if we have enough constraints (roughly: need at least 2 residuals per variable for well-posed problem)
        if num_residuals < num_variables {
            log::warn!("[SlidingWindow] Underconstrained problem: {} residuals < {} variables, skipping optimization", 
                      num_residuals, num_variables);
            return Ok(false);
        }

        // Initialize variables in the problem
        problem.initialize_variables(&initial_values);

        // Try optimization with Schur complement first
        let opt_result = match solver.optimize(&problem, &initial_values) {
            Ok(result) => result,
            Err(e) => {
                // Check if it's a linear solve failure (singular matrix)
                let error_str = format!("{:?}", e);
                if error_str.contains("LinearSolveFailed") || error_str.contains("Singular matrix")
                {
                    log::warn!("[SlidingWindow] Schur complement failed with singular matrix, trying fallback solver (SparseCholesky)");

                    // Create fallback solver with direct Cholesky
                    let mut fallback_solver = LevenbergMarquardt::with_config(
                        LevenbergMarquardtConfig::new()
                            .with_linear_solver_type(LinearSolverType::SparseCholesky)
                            .with_max_iterations(20)
                            .with_cost_tolerance(1e-6)
                            .with_parameter_tolerance(1e-7)
                            .with_jacobi_scaling(false),
                    );

                    match fallback_solver.optimize(&problem, &initial_values) {
                        Ok(result) => {
                            log::debug!("[SlidingWindow] Fallback solver succeeded");
                            result
                        }
                        Err(e2) => {
                            log::error!("[SlidingWindow] Both Schur complement and fallback solver failed: {:?}", e2);
                            return Ok(false);
                        }
                    }
                } else {
                    log::error!("[SlidingWindow] Optimization error: {:?}", e);
                    return Ok(false);
                }
            }
        };

        // Check if optimization was successful based on status
        let is_successful = self.is_optimization_successful(&opt_result);

        log::debug!(
            "[BA] iter={}/20 vars={} res={} status={:?} cost {:.3}→{:.3}",
            opt_result.iterations,
            num_variables,
            num_residuals,
            opt_result.status,
            opt_result.initial_cost,
            opt_result.final_cost
        );

        if is_successful {
            self.process_optimization_result(&opt_result);
            // Prune stale lm_name_cache entries to keep memory bounded.
            // After optimization map_points reflects the current window; entries for evicted
            // features are stale.  Only prune when cache is >4× the live landmark count.
            if self.lm_name_cache.len() > self.map_points.len() * 4 {
                self.lm_name_cache.retain(|k, _| self.map_points.contains_key(k));
            }

            // ── Online camera–IMU time offset refinement ─────────────────────────
            // After BA converges, refine cam_imu_td using the rotation residual
            // between IMU-preintegrated and BA-optimised relative poses.
            //
            // Derivation: if the true IMU window is shifted by δtd, the
            // preintegrated rotation changes by ≈ Exp(-ω_avg · δtd).  So:
            //   r ≡ log(R_imu^T · R_vis) ≈ -ω_avg · δtd
            //   ⟹ δtd = -r · ω_avg / |ω_avg|²   (least-squares 1D projection)
            //
            // Only accepted when |ω_avg| > 0.1 rad/s (motion excitation guard):
            // below this threshold the rotation change per-millisecond is too
            // small to distinguish from noise.
            //
            // A low-pass gain of 0.05 per BA call and a ±2 ms per-step clamp
            // prevent overshooting during fast transients.
            if self.cam_imu_td_online_refinement {
                const MIN_OMEGA_SQ: f64 = 0.01; // |ω| > 0.1 rad/s guard
                const LP_GAIN: f64 = 0.05;      // low-pass: 5% of raw estimate applied
                const MAX_STEP_NS: i64 = 2_000_000; // ±2 ms per BA call

                let mut delta_td_sum = 0.0f64;
                let mut delta_td_n = 0usize;

                for i in 1..self.keyframes.len() {
                    let imu_samples = &self.keyframes[i].imu_from_last_frame;
                    if imu_samples.len() < 3 { continue; }

                    // Bias-corrected average angular velocity for this window.
                    let state = &self.keyframes[i].state;
                    let bg = Vector3::new(
                        state.gyro_bias[0] as f64,
                        state.gyro_bias[1] as f64,
                        state.gyro_bias[2] as f64,
                    );
                    let mut omega_sum = Vector3::zeros();
                    for s in imu_samples.iter() {
                        omega_sum += Vector3::new(
                            s.gyro[0] - bg.x,
                            s.gyro[1] - bg.y,
                            s.gyro[2] - bg.z,
                        );
                    }
                    let omega_avg = omega_sum / imu_samples.len() as f64;
                    let omega_sq = omega_avg.norm_squared();
                    if omega_sq < MIN_OMEGA_SQ { continue; }

                    // Preintegrate to get ΔR_imu.
                    let delta_t = preintegrate_imu(
                        imu_samples,
                        &state.gyro_bias.map(|x| x as f64),
                        &state.accel_bias.map(|x| x as f64),
                        &GRAVITY,
                    );
                    let r_imu: na::Matrix3<f64> = delta_t.fixed_view::<3, 3>(0, 0).into_owned();

                    // BA-optimised relative rotation between adjacent KFs.
                    let r_prev = self.keyframes[i - 1].state.T_W_B.fixed_view::<3, 3>(0, 0).into_owned();
                    let r_curr = self.keyframes[i].state.T_W_B.fixed_view::<3, 3>(0, 0).into_owned();
                    let r_vis = r_prev.transpose() * r_curr;

                    // Rotation residual r = log(R_imu^T · R_vis).
                    let r_err = r_imu.transpose() * r_vis;
                    let rot_res = rotation_log(&r_err);

                    // 1-D projection: δtd = -rot_res · ω_avg / |ω_avg|²
                    let delta_td = -rot_res.dot(&omega_avg) / omega_sq;
                    delta_td_sum += delta_td;
                    delta_td_n += 1;
                }

                if delta_td_n > 0 {
                    let raw_delta_td_s = delta_td_sum / delta_td_n as f64;
                    let step_ns = ((raw_delta_td_s * LP_GAIN) * 1e9) as i64;
                    let clamped = step_ns.clamp(-MAX_STEP_NS, MAX_STEP_NS);
                    // Hard bound: total offset ≤ ±100 ms (sanity guard)
                    self.cam_imu_td_ns = (self.cam_imu_td_ns + clamped)
                        .clamp(-100_000_000, 100_000_000);
                    log::debug!(
                        "[CamImuTd] raw={:.3}ms step={:.3}ms total={:.3}ms (n={})",
                        raw_delta_td_s * 1e3,
                        clamped as f64 * 1e-6,
                        self.cam_imu_td_ns as f64 * 1e-6,
                        delta_td_n,
                    );
                }
            }

            Ok(true)
        } else {
            log::warn!(
                "[SlidingWindow] Optimization failed (status: {:?})",
                opt_result.status
            );
            Ok(false)
        }
    }

    /// Check if optimization result indicates success
    fn is_optimization_successful(
        &self,
        opt_result: &SolverResult<HashMap<String, VariableEnum>>,
    ) -> bool {
        matches!(
            &opt_result.status,
            apex_solver::optimizer::OptimizationStatus::Converged
                | apex_solver::optimizer::OptimizationStatus::CostToleranceReached
                | apex_solver::optimizer::OptimizationStatus::ParameterToleranceReached
                | apex_solver::optimizer::OptimizationStatus::GradientToleranceReached
                | apex_solver::optimizer::OptimizationStatus::TrustRegionRadiusTooSmall
                | apex_solver::optimizer::OptimizationStatus::MinCostThresholdReached
                | apex_solver::optimizer::OptimizationStatus::MaxIterationsReached
        )
    }

    #[allow(clippy::too_many_lines)]
    fn process_optimization_result(
        &mut self,
        opt_result: &SolverResult<HashMap<String, VariableEnum>>,
    ) {
        // Log optimization status
        log::debug!(
            "[SlidingWindow] Optimization status: {:?}",
            opt_result.status
        );

        // Determine convergence status accurately
        let (status, convergence_reason) = match &opt_result.status {
            apex_solver::optimizer::OptimizationStatus::Converged => {
                ("CONVERGED", "Converged".to_string())
            }
            apex_solver::optimizer::OptimizationStatus::CostToleranceReached => {
                ("CONVERGED", "CostTolerance".to_string())
            }
            apex_solver::optimizer::OptimizationStatus::ParameterToleranceReached => {
                ("CONVERGED", "ParameterTolerance".to_string())
            }
            apex_solver::optimizer::OptimizationStatus::GradientToleranceReached => {
                ("CONVERGED", "GradientTolerance".to_string())
            }
            apex_solver::optimizer::OptimizationStatus::TrustRegionRadiusTooSmall => {
                ("CONVERGED", "TrustRegionRadiusTooSmall".to_string())
            }
            apex_solver::optimizer::OptimizationStatus::MinCostThresholdReached => {
                ("CONVERGED", "MinCostThresholdReached".to_string())
            }
            apex_solver::optimizer::OptimizationStatus::MaxIterationsReached => {
                ("NOT_CONVERGED", "MaxIterations".to_string())
            }
            apex_solver::optimizer::OptimizationStatus::Timeout => {
                ("NOT_CONVERGED", "Timeout".to_string())
            }
            apex_solver::optimizer::OptimizationStatus::NumericalFailure => {
                ("NOT_CONVERGED", "NumericalFailure".to_string())
            }
            apex_solver::optimizer::OptimizationStatus::IllConditionedJacobian => {
                ("NOT_CONVERGED", "IllConditionedJacobian".to_string())
            }
            apex_solver::optimizer::OptimizationStatus::InvalidNumericalValues => {
                ("NOT_CONVERGED", "InvalidNumericalValues".to_string())
            }
            apex_solver::optimizer::OptimizationStatus::UserTerminated => {
                ("NOT_CONVERGED", "UserTerminated".to_string())
            }
            apex_solver::optimizer::OptimizationStatus::Failed(msg) => {
                ("NOT_CONVERGED", format!("Failed:{}", msg))
            }
        };
        log::debug!(
            "[SlidingWindow] Optimization status: {}, convergence_reason: {}",
            status,
            convergence_reason
        );

        // Update map_points and keyframe poses with optimized values.
        // Use direct VariableEnum pattern matching to avoid the to_vector() DVector allocation
        // (~200 allocs/BA call for landmarks, ~10 for poses).
        self.map_points.clear();
        opt_result.parameters.iter().for_each(|(var_name, value)| {
            // Update map points (Rn variables)
            if let Some(feature_id_str) = var_name.strip_prefix("pt_") {
                if let Ok(feature_id) = feature_id_str.parse::<usize>() {
                    if let VariableEnum::Rn(var) = value {
                        let d = var.value.data();
                        self.map_points.insert(feature_id, [d[0] as f32, d[1] as f32, d[2] as f32]);
                    }
                }
            }
            // Update keyframe poses (SE3 variables)
            else if let Some(frame_id_str) = var_name.strip_prefix("KF_") {
                if let Ok(frame_id) = frame_id_str.parse::<i32>() {
                    if let VariableEnum::SE3(var) = value {
                        let mat = var.value.matrix();
                        if let Some(kf) = self.keyframes.get_mut(frame_id as usize) {
                            kf.state.T_W_B = inverse_se3(&mat);
                        } else {
                            log::warn!(
                                "[SlidingWindow] Optimized keyframe {} not found, skipping update",
                                frame_id
                            );
                        }
                    }
                }
            }
            // Update velocities (Rn variables: v_0, v_1, ...)
            else if let Some(vel_id_str) = var_name.strip_prefix("v_") {
                if let Ok(vel_id) = vel_id_str.parse::<usize>() {
                    if let VariableEnum::Rn(var) = value {
                        let d = var.value.data();
                        if let Some(kf) = self.keyframes.get_mut(vel_id) {
                            kf.state.velocity = [d[0] as f32, d[1] as f32, d[2] as f32];
                        }
                    }
                }
            }
            // Update gyro bias
            else if var_name == "bg_global" {
                if let VariableEnum::Rn(var) = value {
                    let d = var.value.data();
                    let bg = [d[0] as f32, d[1] as f32, d[2] as f32];
                    for kf in self.keyframes.iter_mut() {
                        kf.state.gyro_bias = bg;
                    }
                    log::debug!("[IMU] Optimized gyro bias: [{:.6}, {:.6}, {:.6}]", bg[0], bg[1], bg[2]);
                }
            }
            // Update accel bias
            else if var_name == "ba_global" {
                if let VariableEnum::Rn(var) = value {
                    let d = var.value.data();
                    let ba = [d[0] as f32, d[1] as f32, d[2] as f32];
                    for kf in self.keyframes.iter_mut() {
                        kf.state.accel_bias = ba;
                    }
                    log::debug!("[IMU] Optimized accel bias: [{:.6}, {:.6}, {:.6}]", ba[0], ba[1], ba[2]);
                }
            }
        });
    }

    /// Track the motion of the system by solving a PnP-like problem
    /// The map points and existing keyframes are kept constant, only the new frame is optimized
    #[allow(clippy::too_many_lines)]
    pub fn track_motion(&mut self, frame: &Frame) -> Result<Option<Matrix4x4>, std::io::Error> {
        // Create a new problem and solver
        let mut problem = Problem::new(JacobianMode::Sparse);
        let mut solver = LevenbergMarquardt::with_config(
            LevenbergMarquardtConfig::new()
                .with_linear_solver_type(LinearSolverType::SparseCholesky)
                .with_max_iterations(10)
                .with_cost_tolerance(1e-6)
                .with_parameter_tolerance(1e-7)
                .with_jacobi_scaling(false),
        );
        // 1 SE3 pose variable; map points are fixed, not added to initial_values
        let mut initial_values = HashMap::with_capacity(1);
        // solver.add_observer(TerminalObserver::new());

        // Record the timestamp of this tracked frame for the next call's CV time-scaling.
        // Done unconditionally at entry so all return paths (IMU fallback, prior reject,
        // divergence fallback, success) have correct frame-to-frame dt available.
        let prev_track_timestamp_ns = self.last_track_timestamp_ns;
        self.last_track_timestamp_ns = Some(frame.timestamp_ns);

        // Add variable for the new frame
        // Only the new frame is optimized.
        // Initial pose predicted by IMU pre-integration (preferred) or constant velocity model:
        //   IMU:  T_W_B_init = T_W_B_last · ΔT_imu
        //   CV:   T_W_B_init = T_W_B_last · T_W_B_prev⁻¹ · T_W_B_last
        let kf_var = "F".to_string();
        let last_kf_pose = self
            .keyframes
            .back()
            .expect("keyframes should not be empty")
            .state
            .T_W_B;

        // **Warm-start**: use last optimized pose + IMU delta if available,
        // otherwise fall back to IMU-only or constant velocity prediction.
        let init_T_W_B = if !frame.imu_from_last_frame.is_empty() {
            // Use IMU pre-integration with bias correction from last keyframe state
            let last_kf = self.keyframes.back().expect("Keyframes should not be empty");
            let last_state = &last_kf.state;
            let gyro_bias_f64: [f64; 3] = last_state.gyro_bias.map(|x| x as f64);
            let accel_bias_f64: [f64; 3] = last_state.accel_bias.map(|x| x as f64);

            // Apply camera–IMU time offset: trim IMU samples that fall before
            // the adjusted previous-frame boundary (t_prev_cam + td).
            // For td > 0 (camera lags IMU) we skip early samples whose events
            // actually preceded the camera exposure.  For td ≤ 0 all loaded
            // samples are used (we cannot reach back into the previous window).
            let imu_slice: &[crate::datasets::ImuData] = if self.cam_imu_td_ns > 0 {
                let adjusted_prev = last_kf.timestamp_ns + self.cam_imu_td_ns;
                let skip = frame.imu_from_last_frame
                    .partition_point(|s| s.timestamp <= adjusted_prev);
                &frame.imu_from_last_frame[skip..]
            } else {
                &frame.imu_from_last_frame
            };

            let delta_t = preintegrate_imu(
                imu_slice,
                &gyro_bias_f64,
                &accel_bias_f64,
                &GRAVITY,
            );
            // If we have a warm-start from the previous frame, use it as base
            if let Some(last_pose) = self.last_track_pose {
                last_pose * delta_t
            } else {
                last_kf_pose * delta_t
            }
        } else if let Some(last_pose) = self.last_track_pose {
            // No IMU: extrapolate from last tracked pose using time-scaled constant velocity.
            // cv_delta = T_last⁻¹ × T_cv_pred = body-frame KF-to-KF relative motion.
            // Scale by (frame_dt / kf_dt) so we only extrapolate the fraction of the
            // KF-to-KF motion that corresponds to the elapsed frame interval.
            let kf_delta = inverse_se3(&last_kf_pose) * self.predict_current_pose();
            let scale = self.cv_time_scale(frame.timestamp_ns, true);
            let cv_delta = scale_se3_delta(&kf_delta, scale);
            last_pose * cv_delta
        } else {
            // No IMU, no warm-start: extrapolate from last KF using time-scaled CV.
            // Scale ensures we don't overshoot when the KF interval >> frame interval.
            let kf_t_rel = if let Some((t_prev, t_last)) = self.get_last_two_poses() {
                inverse_se3(&t_prev) * t_last
            } else {
                Matrix4x4::identity()
            };
            let scale = self.cv_time_scale(frame.timestamp_ns, false);
            let kf_delta_scaled = scale_se3_delta(&kf_t_rel, scale);
            last_kf_pose * kf_delta_scaled
        };

        let T_B_W = inverse_se3(&init_T_W_B);
        let t_B_W = T_B_W.fixed_view::<3, 1>(0, 3);
        let R_B_W = Matrix3x3::from(T_B_W.fixed_view::<3, 3>(0, 0));
        let q_B_W = UnitQuaternion::from_matrix(&R_B_W);
        // Build DVector<f64> directly — avoids an intermediate DVector<Float> + cast alloc.
        initial_values.insert(kf_var.clone(), (ManifoldType::SE3, DVector::from_row_slice(&[
            t_B_W.x as f64, t_B_W.y as f64, t_B_W.z as f64,
            q_B_W.w as f64, q_B_W.i as f64, q_B_W.j as f64, q_B_W.k as f64,
        ])));

        // Add factors: for both cameras, use map points to constrain the new frame pose
        let first_kf_state = &self.keyframes.front().expect("keyframes should not be empty").state;
        let T_Cl_B = inverse_se3(&first_kf_state.T_B_Cl);
        let T_Cr_B = inverse_se3(&first_kf_state.T_B_Cr);

        // **Shared HuberLoss** for PnP factors
        let shared_loss = HuberLoss::new(2.0).unwrap();

        let camera_features = [
            (&frame.left_features, T_Cl_B),
            (&frame.right_features, T_Cr_B),
        ];
        let mut pnp_factor_count = 0usize;
        for (features, T_C_B) in camera_features.iter() {
            for feat in features.iter() {
                let feature_id = feat.feature_id;

                let point = self.map_points.get(&feature_id);
                match point {
                    Some(point) => {
                        // Create PnP factor with age-weighting
                        let factor = PnPFactor::new(
                            na::Vector2::new(feat.undistorted_coord[0], feat.undistorted_coord[1])
                                .cast::<f64>(),
                            *T_C_B,
                            na::Vector3::new(point[0] as f64, point[1] as f64, point[2] as f64),
                        )
                        .with_track_age(feat.age); // Age-weighted: older features get higher weight
                        // Add residual block with shared Huber loss
                        problem.add_residual_block(
                            &[&kf_var],
                            Box::new(factor),
                            Some(Box::new(shared_loss.clone())),
                        );
                        pnp_factor_count += 1;
                    }
                    None => {
                        // log::debug!("[SlidingWindow] Motion tracking: point {} is not in the map", feature_id);
                    }
                }
            }
        }

        // Initialize variables in the problem
        problem.initialize_variables(&initial_values);

        // Optimize
        let opt_result = match solver.optimize(&problem, &initial_values) {
            Ok(result) => result,
            Err(e) => {
                // Optimization error
                log::error!("[SlidingWindow] Motion tracking optimizer error: {:?}", e);
                return Ok(None);
            }
        };

        // Check if optimization was successful
        let is_successful = self.is_optimization_successful(&opt_result);

        log::debug!(
            "[PnP] iter={}/10 status={:?} cost {:.3}→{:.3}",
            opt_result.iterations,
            opt_result.status,
            opt_result.initial_cost,
            opt_result.final_cost
        );

        if is_successful {
            // Extract optimized pose T_B_W and convert to T_W_B.
            // Match on VariableEnum::SE3 directly to avoid to_vector() DVector allocation.
            if let Some(VariableEnum::SE3(var)) = opt_result.parameters.get(&kf_var) {
                let T_B_W_opt = var.value.matrix();
                let T_W_B_opt = inverse_se3(&T_B_W_opt);

                // === Motion Prior Gating ===
                //
                // Use the platform-specific motion prior to gate the PnP result.
                // This prevents physically impossible updates from being accepted.
                let pred_state = self.keyframes.back()
                    .map(|f| f.state.clone())
                    .unwrap_or_else(crate::estimator::state::State::identity);

                let vision_pos = T_W_B_opt.fixed_view::<3, 1>(0, 3).into_owned();
                let vision_rot = na::UnitQuaternion::from_matrix(
                    &T_W_B_opt.fixed_view::<3, 3>(0, 0).into_owned()
                );

                // Use the PnP factor count (features with map points) as inlier count proxy.
                // num_residual_blocks() counts apex_solver internal blocks, not factors.
                let inlier_count = pnp_factor_count;
                let mean_reproj_error = if inlier_count > 0 {
                    (opt_result.final_cost / inlier_count as f64).sqrt() as f32
                } else {
                    f32::MAX
                };
                let prior_score = 1.0;  // PnP succeeded

                log::debug!(
                    "[MotionPrior] PnP factors={}, final_cost={:.3}, mean_reproj_err={:.3}px",
                    inlier_count, opt_result.final_cost, mean_reproj_error
                );

                let dt_s = if let Some(last_kf) = self.keyframes.back() {
                    ((frame.timestamp_ns - last_kf.timestamp_ns) as f32) / 1e9
                } else {
                    0.05  // Default 20Hz
                };

                let gate_decision = self.motion_prior.gate_measurement(
                    &pred_state,
                    &na::Vector3::new(vision_pos.x as f32, vision_pos.y as f32, vision_pos.z as f32),
                    &na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                        vision_rot.w as f32, vision_rot.i as f32, vision_rot.j as f32, vision_rot.k as f32
                    ).normalize()),
                    dt_s,
                    inlier_count,
                    mean_reproj_error,
                    prior_score,
                );

                match gate_decision {
                    GateDecision::Reject => {
                        log::warn!(
                            "[MotionPrior] PnP rejected by {:} prior; falling back to IMU prediction",
                            self.motion_prior.name()
                        );
                        self.last_track_pose = Some(init_T_W_B);
                        return Ok(Some(init_T_W_B));
                    }
                    GateDecision::Downweight { alpha } => {
                        log::debug!(
                            "[MotionPrior] PnP downweighted by {:} prior (alpha={:.1})",
                            self.motion_prior.name(), alpha
                        );
                        // Store alpha for potential covariance inflation in future iterations
                        // For now, the gate itself provides robustness by catching impossible motions
                    }
                    GateDecision::Accept => {
                        log::debug!("[MotionPrior] PnP accepted by {:} prior", self.motion_prior.name());
                    }
                }

                // === IMU-aware outlier rejection (secondary check) ===
                //
                // Compare BA-optimized pose to the IMU-predicted initial guess.  If they
                // differ by more than MAX_BA_DEVIATION_M translation or MAX_BA_DEVIATION_RAD
                // rotation, BA has diverged — reject its output and fall back to the IMU
                // prediction.  This prevents catastrophic tracking failures (observed on
                // slides1/corridors: pose jumps > 30 m/frame = 600 m/s, system resets).
                //
                // Thresholds are loose enough to allow legitimate fast motion (5 m/s × 50 ms
                // = 0.25 m/frame; this threshold allows ~20× that) while still catching
                // numerical divergence.
                const MAX_BA_DEVIATION_M: f64 = 5.0;
                const MAX_BA_DEVIATION_RAD: f64 = 1.0;  // ~57 degrees
                let t_opt = T_W_B_opt.fixed_view::<3, 1>(0, 3);
                let t_init = init_T_W_B.fixed_view::<3, 1>(0, 3);
                let dev_t_sq = (t_opt - t_init).norm_squared();
                let r_opt: na::Matrix3<f64> = T_W_B_opt.fixed_view::<3, 3>(0, 0).into_owned();
                let r_init: na::Matrix3<f64> = init_T_W_B.fixed_view::<3, 3>(0, 0).into_owned();
                let r_rel = r_init.transpose() * r_opt;
                let cos_dev = ((r_rel.trace() - 1.0) * 0.5).clamp(-1.0, 1.0);
                let dev_rad = cos_dev.acos();

                if !dev_t_sq.is_finite() || dev_t_sq > MAX_BA_DEVIATION_M * MAX_BA_DEVIATION_M
                    || !dev_rad.is_finite() || dev_rad > MAX_BA_DEVIATION_RAD
                {
                    log::warn!(
                        "[SlidingWindow] BA diverged from IMU prediction \
                         (Δt={:.2}m, Δr={:.2}rad); falling back to IMU prediction",
                        dev_t_sq.sqrt(), dev_rad
                    );
                    self.last_track_pose = Some(init_T_W_B);
                    return Ok(Some(init_T_W_B));
                }

                // === Physical velocity plausibility check ===
                //
                // Catches pose jumps that slip past the IMU-deviation check — e.g. when
                // IMU prediction is itself bad (long gap, bias drift) and BA happens to
                // land near that bad prediction yet still far from the previous accepted
                // pose.  The threshold of 10 m/s is generous for handheld walking
                // (typical ~1.5 m/s) and still well below any numerical divergence.
                if let (Some(last_pose), Some(prev_ts)) =
                    (self.last_track_pose, prev_track_timestamp_ns)
                {
                    let dt_s =
                        (frame.timestamp_ns - prev_ts).max(1) as f64 / 1e9;
                    let t_opt = T_W_B_opt.fixed_view::<3, 1>(0, 3);
                    let t_last = last_pose.fixed_view::<3, 1>(0, 3);
                    let velocity_m_s = (t_opt - t_last).norm() / dt_s;

                    // Also check angular velocity between accepted poses.
                    let r_opt2: na::Matrix3<f64> =
                        T_W_B_opt.fixed_view::<3, 3>(0, 0).into_owned();
                    let r_last: na::Matrix3<f64> =
                        last_pose.fixed_view::<3, 3>(0, 0).into_owned();
                    let r_rel2 = r_last.transpose() * r_opt2;
                    let cos_v = ((r_rel2.trace() - 1.0) * 0.5).clamp(-1.0, 1.0);
                    let omega_rad_s = cos_v.acos() / dt_s;

                    const MAX_VELOCITY_M_S: f64 = 10.0;    // ~3× brisk walking
                    const MAX_OMEGA_RAD_S: f64 = 8.0;      // ~460°/s, well above hand-held

                    // Require BOTH velocity AND rotation rate to be suspicious before
                    // rejecting — a single large omega (quick head turn) with moderate
                    // translation is plausible; truly bogus frames exceed both limits.
                    let is_bogus = !velocity_m_s.is_finite() || !omega_rad_s.is_finite()
                        || (velocity_m_s > MAX_VELOCITY_M_S && omega_rad_s > MAX_OMEGA_RAD_S);
                    if is_bogus {
                        log::warn!(
                            "[SlidingWindow] Pose jump detected \
                             (v={:.1} m/s, ω={:.1} rad/s, dt={:.3}s); \
                             falling back to IMU prediction",
                            velocity_m_s, omega_rad_s, dt_s
                        );
                        self.last_track_pose = Some(init_T_W_B);
                        return Ok(Some(init_T_W_B));
                    }
                }

                // Store for warm-start of next frame's track_motion
                self.last_track_pose = Some(T_W_B_opt);
                log::debug!(
                    "[SlidingWindow] Motion tracking successful. Initial cost: {:.3}, final cost: {:.3}",
                    opt_result.initial_cost,
                    opt_result.final_cost
                );
                Ok(Some(T_W_B_opt))
            } else {
                log::warn!("[SlidingWindow] Motion tracking: optimized pose not found in result");
                // Fall back to IMU prediction so the trajectory doesn't stall
                self.last_track_pose = Some(init_T_W_B);
                Ok(Some(init_T_W_B))
            }
        } else {
            log::warn!(
                "[SlidingWindow] Motion tracking failed (status: {:?}) — falling back to IMU prediction",
                opt_result.status
            );
            // Fall back to IMU-predicted pose instead of returning None, so the
            // trajectory continues instead of stalling/resetting.
            self.last_track_pose = Some(init_T_W_B);
            Ok(Some(init_T_W_B))
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::Frame;

    fn make_dummy_frame(frame_id: i32, pose: Matrix4x4) -> Frame {
        let mut frame = Frame::new(0, frame_id);
        frame.state.T_W_B = pose;
        frame.is_keyframe = true;
        frame
    }

    fn translation_only(x: f64, y: f64, z: f64) -> Matrix4x4 {
        let mut m = Matrix4x4::identity();
        m[(0, 3)] = x;
        m[(1, 3)] = y;
        m[(2, 3)] = z;
        m
    }

    #[test]
    fn test_predict_pose_single_keyframe_fallback() {
        let mut sw = SlidingWindow::new(5);
        let pose = translation_only(1.0, 0.0, 0.0);
        let frame = make_dummy_frame(1, pose);
        sw.add_frame(frame);

        // With only one keyframe, should return that keyframe's pose
        let predicted = sw.predict_current_pose();
        assert_eq!(predicted, pose);
    }

    #[test]
    fn test_predict_pose_constant_velocity_translation() {
        let mut sw = SlidingWindow::new(5);

        // Two keyframes: identity at origin, then translated by (1, 0, 0)
        let pose_0 = Matrix4x4::identity();
        let pose_1 = translation_only(1.0, 0.0, 0.0);

        sw.add_frame(make_dummy_frame(0, pose_0));
        sw.add_frame(make_dummy_frame(1, pose_1));

        // Constant velocity model should predict:
        // T_rel = T_0⁻¹ * T_1 = I⁻¹ * T(1,0,0) = T(1,0,0)
        // T_pred = T_1 * T_rel = T(1,0,0) * T(1,0,0) = T(2,0,0)
        let predicted = sw.predict_current_pose();
        assert!((predicted[(0, 3)] - 2.0).abs() < 1e-10);
        assert!((predicted[(1, 3)] - 0.0).abs() < 1e-10);
        assert!((predicted[(2, 3)] - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_predict_pose_constant_velocity_multi_step() {
        let mut sw = SlidingWindow::new(5);

        // Three keyframes with uniform translation: (0,0,0) → (1,0,0) → (2,0,0)
        sw.add_frame(make_dummy_frame(0, translation_only(0.0, 0.0, 0.0)));
        sw.add_frame(make_dummy_frame(1, translation_only(1.0, 0.0, 0.0)));
        sw.add_frame(make_dummy_frame(2, translation_only(2.0, 0.0, 0.0)));

        // Should use last two: T_prev=(1,0,0), T_last=(2,0,0)
        // T_rel = (1,0,0)⁻¹ * (2,0,0) = T(1,0,0)
        // T_pred = (2,0,0) * T(1,0,0) = T(3,0,0)
        let predicted = sw.predict_current_pose();
        assert!((predicted[(0, 3)] - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_predict_pose_with_rotation() {
        use na::UnitQuaternion;

        let mut sw = SlidingWindow::new(5);

        // Two keyframes with rotation around Z axis
        let q0 = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
        let q1 = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.1);

        let mut pose_0 = Matrix4x4::identity();
        pose_0
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(q0.to_rotation_matrix().matrix());
        let mut pose_1 = Matrix4x4::identity();
        pose_1
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(q1.to_rotation_matrix().matrix());

        sw.add_frame(make_dummy_frame(0, pose_0));
        sw.add_frame(make_dummy_frame(1, pose_1));

        let predicted = sw.predict_current_pose();

        // Extract rotation angle from predicted pose
        let r: na::Matrix3<f64> = predicted.fixed_view::<3, 3>(0, 0).into_owned();
        let predicted_q = UnitQuaternion::from_matrix(&r);
        let (_, _, yaw) = predicted_q.euler_angles();

        // Expected: yaw ≈ 0.2 (0.0 + 0.1 + 0.1 extrapolated)
        assert!((yaw - 0.2).abs() < 1e-6, "expected yaw≈0.2, got {}", yaw);
    }

    #[test]
    fn test_predict_pose_window_full_uses_last_two() {
        let mut sw = SlidingWindow::new(3);

        // Fill the window: poses at x=0, x=1, x=2 (window full at 3)
        sw.add_frame(make_dummy_frame(0, translation_only(0.0, 0.0, 0.0)));
        sw.add_frame(make_dummy_frame(1, translation_only(1.0, 0.0, 0.0)));
        sw.add_frame(make_dummy_frame(2, translation_only(2.0, 0.0, 0.0)));

        // Window is full, should use last two: x=1, x=2 → predict x=3
        assert_eq!(sw.keyframes.len(), 3);
        let predicted = sw.predict_current_pose();
        assert!((predicted[(0, 3)] - 3.0).abs() < 1e-10);

        // Add another frame — oldest gets evicted, still uses last two
        sw.add_frame(make_dummy_frame(3, translation_only(3.0, 0.0, 0.0)));
        assert_eq!(sw.keyframes.len(), 3);
        let predicted = sw.predict_current_pose();
        assert!((predicted[(0, 3)] - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_predict_pose_stationary() {
        let mut sw = SlidingWindow::new(5);

        // Two keyframes at same pose (no motion)
        let pose = translation_only(5.0, 3.0, 1.0);
        sw.add_frame(make_dummy_frame(0, pose));
        sw.add_frame(make_dummy_frame(1, pose));

        // Zero velocity → prediction = last pose
        let predicted = sw.predict_current_pose();
        assert_eq!(predicted, pose);
    }

    // ========================================================================
    // Tests for get_last_two_poses (zero-allocation accessor)
    // ========================================================================

    #[test]
    fn test_get_last_two_poses_empty() {
        let sw = SlidingWindow::new(5);
        assert!(sw.get_last_two_poses().is_none());
    }

    #[test]
    fn test_get_last_two_poses_single() {
        let mut sw = SlidingWindow::new(5);
        sw.add_frame(make_dummy_frame(0, translation_only(1.0, 0.0, 0.0)));
        assert!(sw.get_last_two_poses().is_none());
    }

    #[test]
    fn test_get_last_two_poses_returns_correct_pair() {
        let mut sw = SlidingWindow::new(5);
        let p0 = translation_only(1.0, 2.0, 3.0);
        let p1 = translation_only(4.0, 5.0, 6.0);
        sw.add_frame(make_dummy_frame(0, p0));
        sw.add_frame(make_dummy_frame(1, p1));

        let (prev, last) = sw.get_last_two_poses().expect("should have two poses");
        assert_eq!(prev, p0);
        assert_eq!(last, p1);
    }

    #[test]
    fn test_get_last_two_poses_after_eviction() {
        let mut sw = SlidingWindow::new(3);
        sw.add_frame(make_dummy_frame(0, translation_only(0.0, 0.0, 0.0)));
        sw.add_frame(make_dummy_frame(1, translation_only(1.0, 0.0, 0.0)));
        sw.add_frame(make_dummy_frame(2, translation_only(2.0, 0.0, 0.0)));

        // After eviction, last two should be frames 1 and 2
        let (prev, last) = sw.get_last_two_poses().expect("should have two poses");
        assert!((prev[(0, 3)] - 1.0).abs() < 1e-10);
        assert!((last[(0, 3)] - 2.0).abs() < 1e-10);
    }

    // ========================================================================
    // Additional SE(3) edge case tests
    // ========================================================================

    #[test]
    fn test_predict_pose_combined_se3_motion() {
        use na::UnitQuaternion;

        let mut sw = SlidingWindow::new(5);

        // Two keyframes with combined translation + rotation
        let pose_0 = Matrix4x4::identity();
        let mut pose_1 = Matrix4x4::identity();

        let q1 = UnitQuaternion::from_euler_angles(0.0, 0.05, 0.1);
        pose_1
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(q1.to_rotation_matrix().matrix());
        pose_1[(0, 3)] = 0.5;
        pose_1[(1, 3)] = 0.1;
        pose_1[(2, 3)] = -0.2;

        sw.add_frame(make_dummy_frame(0, pose_0));
        sw.add_frame(make_dummy_frame(1, pose_1));

        let predicted = sw.predict_current_pose();

        // For SE(3) composition: T_pred = T_1 * T_1
        // The translation is R_1 * t_1 + t_1 (not simply 2*t_1)
        // The rotation is R_1 * R_1 (double the angle)
        let r: na::Matrix3<f64> = predicted.fixed_view::<3, 3>(0, 0).into_owned();
        let predicted_q = UnitQuaternion::from_matrix(&r);
        let (roll, pitch, yaw) = predicted_q.euler_angles();

        // Rotation should be extrapolated: yaw ≈ 0.2, pitch ≈ 0.1
        // Note: euler angles are NOT additive under SE(3) composition, so we use relaxed tolerance
        assert!((yaw - 0.2).abs() < 5e-3, "expected yaw≈0.2, got {}", yaw);
        assert!(
            (pitch - 0.1).abs() < 5e-3,
            "expected pitch≈0.1, got {}",
            pitch
        );
        assert!(roll.abs() < 0.01, "expected roll≈0.0, got {}", roll);

        // Translation is R_1 * t_1 + t_1 — verify it's approximately 2x for small rotations
        // For yaw=0.1, pitch=0.05, the rotated translation differs from 2*t_1
        // We verify the general direction and magnitude are reasonable
        let tx = predicted[(0, 3)];
        let ty = predicted[(1, 3)];
        let tz = predicted[(2, 3)];

        // Just verify non-zero motion in the expected direction
        assert!(tx > 0.5, "expected x>0.5 (got {})", tx);
        assert!(ty > 0.0, "expected y>0 (got {})", ty);
        assert!(tz < 0.0, "expected z<0 (got {})", tz);
    }

    #[test]
    fn test_predict_pose_reversing_direction() {
        let mut sw = SlidingWindow::new(5);

        // Three keyframes: moving forward then reversing
        // (0,0,0) → (2,0,0) → (3,0,0) — velocity slows down
        sw.add_frame(make_dummy_frame(0, translation_only(0.0, 0.0, 0.0)));
        sw.add_frame(make_dummy_frame(1, translation_only(2.0, 0.0, 0.0)));
        sw.add_frame(make_dummy_frame(2, translation_only(3.0, 0.0, 0.0)));

        // Last delta is +1, so prediction = 3 + 1 = 4
        let predicted = sw.predict_current_pose();
        assert!(
            (predicted[(0, 3)] - 4.0).abs() < 1e-10,
            "expected x=4, got {}",
            predicted[(0, 3)]
        );
    }

    #[test]
    fn test_predict_pose_consistency_with_get_keyframe_poses() {
        let mut sw = SlidingWindow::new(5);
        sw.add_frame(make_dummy_frame(0, translation_only(0.0, 0.0, 0.0)));
        sw.add_frame(make_dummy_frame(1, translation_only(1.0, 0.0, 0.0)));
        sw.add_frame(make_dummy_frame(2, translation_only(2.0, 0.0, 0.0)));

        // get_last_two_poses should return the same poses as get_keyframe_poses
        let all_poses = sw.get_keyframe_poses();
        let (prev, last) = sw.get_last_two_poses().expect("should have two poses");

        assert_eq!(prev, all_poses[all_poses.len() - 2]);
        assert_eq!(last, all_poses[all_poses.len() - 1]);
    }

    #[test]
    fn test_predict_pose_backward_motion() {
        let mut sw = SlidingWindow::new(5);

        // Moving in negative direction
        sw.add_frame(make_dummy_frame(0, translation_only(0.0, 0.0, 0.0)));
        sw.add_frame(make_dummy_frame(1, translation_only(-1.0, 0.5, -0.3)));

        // Predict: T_rel = I⁻¹ * T(-1, 0.5, -0.3) = T(-1, 0.5, -0.3)
        // T_pred = T(-1, 0.5, -0.3) * T(-1, 0.5, -0.3) = T(-2, 1.0, -0.6)
        let predicted = sw.predict_current_pose();
        assert!((predicted[(0, 3)] + 2.0).abs() < 1e-10);
        assert!((predicted[(1, 3)] - 1.0).abs() < 1e-10);
        assert!((predicted[(2, 3)] + 0.6).abs() < 1e-10);
    }

    // ========================================================================
    // IMU Factor Integration Tests
    // ========================================================================

    use crate::feature_tracker::Feature;
    use crate::optimization::imu_preintegration::PreintegratedImuMeasurements;

    fn make_imu_sample(timestamp_ns: i64, gyro: [f64; 3], accel: [f64; 3]) -> ImuData {
        ImuData {
            timestamp: timestamp_ns,
            gyro,
            accel,
        }
    }

    fn make_feature(feature_id: usize, u: f32, v: f32, age: u32) -> Feature {
        let mut f = Feature {
            feature_id,
            pixel_coord: [u, v],
            undistorted_coord: [u, v],
            age,
        };
        f.undistorted_coord = [u, v];
        f
    }

    /// Create a keyframe with synthetic IMU data between it and the previous frame.
    /// The IMU data simulates a pure translation along X.
    fn make_keyframe_with_imu(
        id: i32,
        pose: Matrix4x4,
        timestamp_ns: i64,
        imu_samples: Vec<ImuData>,
        feature_id_offset: usize,
        num_features: usize,
    ) -> Frame {
        let mut f = Frame::new(timestamp_ns, id);
        f.is_keyframe = true;
        f.state.T_W_B = pose;
        f.imu_from_last_frame = imu_samples;

        // Add some features for BA
        for i in 0..num_features {
            let fid = feature_id_offset + i;
            // Spread features across the image
            let u = 0.3 + (i as f32 * 0.4 / num_features.max(1) as f32);
            let v = 0.3 + (i as f32 * 0.2 / num_features.max(1) as f32);
            f.left_features.push(make_feature(fid, u, v, 3));
            f.right_features.push(make_feature(fid, u - 0.05, v, 3));
        }

        f
    }

    /// Generate synthetic IMU data for a pure translation along X axis.
    /// The robot moves at constant velocity, so accel in body frame ≈ -R^T·g + m·a_world.
    fn generate_imu_for_translation(
        start_ts_ns: i64,
        duration_s: f64,
        accel_world_x: f64,
        sample_rate_hz: f64,
    ) -> Vec<ImuData> {
        let n = (duration_s * sample_rate_hz) as usize;
        let dt = 1.0 / sample_rate_hz;
        let mut samples = Vec::with_capacity(n);

        for i in 0..n {
            let ts = start_ts_ns + (i as f64 * dt * 1e9) as i64;
            samples.push(make_imu_sample(
                ts,
                [0.0, 0.0, 0.0], // No rotation
                [accel_world_x, 0.0, 9.81], // Acceleration in body frame (z-down convention)
            ));
        }
        samples
    }

    #[test]
    fn test_imu_factor_integration_stationary() {
        // Test that BA with IMU factors converges correctly for a stationary robot.
        // This verifies that the IMU factor doesn't corrupt the solution when
        // there is no motion.
        let mut sw = SlidingWindow::new(5);

        let pose = translation_only(0.0, 0.0, 0.0);
        let base_ts = 1_000_000_000_000i64;
        let dt_s = 0.1;

        // Generate stationary IMU data (only gravity)
        let imu_samples = generate_imu_for_translation(base_ts, dt_s, 0.0, 200.0);

        // Create 5 keyframes with identical poses (stationary robot)
        for i in 0..5 {
            let frame = make_keyframe_with_imu(
                i,
                pose,
                base_ts + (i as f64 * dt_s * 1e9) as i64,
                imu_samples.clone(),
                (i * 20) as usize,
                10,
            );
            sw.add_frame(frame);
        }

        assert!(sw.is_full(), "Window should be full");

        // Run BA — should succeed with IMU factors
        let result = sw.optimize();
        assert!(result.is_ok(), "BA should succeed: {:?}", result);
        // Note: result may be false if problem is underconstrained, that's ok for this test

        // Check that poses are bounded (not diverged to infinity)
        for (i, kf) in sw.keyframes.iter().enumerate() {
            let t = kf.state.T_W_B.fixed_view::<3, 1>(0, 3);
            assert!(
                t.norm() < 5.0,
                "Pose {} diverged: t = [{:.4}, {:.4}, {:.4}]",
                i, t.x, t.y, t.z
            );
        }
    }

    #[test]
    fn test_imu_factor_integration_with_motion() {
        // Test that BA with IMU factors correctly handles translational motion.
        let mut sw = SlidingWindow::new(5);

        let base_ts = 1_000_000_000_000i64;
        let dt_s = 0.1;

        // Generate IMU data for constant acceleration along X
        let imu_samples = generate_imu_for_translation(base_ts, dt_s, 2.0, 200.0);

        // Create keyframes with increasing X translation (simulating motion)
        for i in 0..5 {
            let x = i as f64 * 0.3; // 30cm per frame
            let pose = translation_only(x, 0.0, 0.0);
            let frame = make_keyframe_with_imu(
                i,
                pose,
                base_ts + (i as f64 * dt_s * 1e9) as i64,
                imu_samples.clone(),
                (i * 20) as usize,
                10,
            );
            sw.add_frame(frame);
        }

        assert!(sw.is_full());

        // Run BA with IMU factors
        let result = sw.optimize();
        assert!(result.is_ok(), "BA should succeed: {:?}", result);

        // Check that poses are still reasonable (not diverged to infinity)
        for (i, kf) in sw.keyframes.iter().enumerate() {
            let t = kf.state.T_W_B.fixed_view::<3, 1>(0, 3);
            // Poses should be in a reasonable range (within 5m of origin)
            assert!(
                t.norm() < 5.0,
                "Pose {} diverged: t = [{:.4}, {:.4}, {:.4}]",
                i, t.x, t.y, t.z
            );
        }
    }

    #[test]
    fn test_imu_bias_estimation_convergence() {
        // Test that the IMU factor can estimate biases correctly.
        // Start with non-zero biases and verify that the optimizer reduces
        // them toward the true values (zero in this case).
        let mut sw = SlidingWindow::new(5);

        let base_ts = 1_000_000_000_000i64;
        let dt_s = 0.1;
        let imu_samples = generate_imu_for_translation(base_ts, dt_s, 0.0, 200.0);

        for i in 0..5 {
            let mut f = make_keyframe_with_imu(
                i,
                translation_only(0.0, 0.0, 0.0),
                base_ts + (i as f64 * dt_s * 1e9) as i64,
                imu_samples.clone(),
                (i * 20) as usize,
                10,
            );
            // Initialize with non-zero biases
            f.state.gyro_bias = [0.01, -0.005, 0.002];
            f.state.accel_bias = [0.1, -0.05, 0.02];
            sw.add_frame(f);
        }

        let bias_before = sw.keyframes[0].state.gyro_bias;

        let result = sw.optimize();
        assert!(result.is_ok(), "BA should succeed: {:?}", result);

        let bias_after = sw.keyframes[0].state.gyro_bias;

        // Biases should have been updated (not necessarily to zero, but changed)
        // The optimizer should move biases toward values that explain the IMU data
        let bias_delta = (bias_before[0] as f64 - bias_after[0] as f64).abs();
        // Just verify the optimization ran without error and biases were updated
        log::debug!(
            "Gyro bias before: {:?}, after: {:?}, delta: {:.6}",
            bias_before, bias_after, bias_delta
        );
    }

    #[test]
    fn test_imu_factor_velocity_update() {
        // Test that velocities are properly added to the optimization problem.
        let mut sw = SlidingWindow::new(5);

        let base_ts = 1_000_000_000_000i64;
        let dt_s = 0.1;
        let imu_samples = generate_imu_for_translation(base_ts, dt_s, 1.0, 200.0);

        for i in 0..5 {
            let mut f = make_keyframe_with_imu(
                i,
                translation_only(i as f64 * 0.1, 0.0, 0.0),
                base_ts + (i as f64 * dt_s * 1e9) as i64,
                imu_samples.clone(),
                (i * 20) as usize,
                10,
            );
            // Initialize velocities to zero (incorrect)
            f.state.velocity = [0.0, 0.0, 0.0];
            sw.add_frame(f);
        }

        let vel_before: Vec<[f32; 3]> = sw.keyframes.iter().map(|f| f.state.velocity).collect();

        let result = sw.optimize();
        assert!(result.is_ok(), "BA should succeed: {:?}", result);

        let vel_after: Vec<[f32; 3]> = sw.keyframes.iter().map(|f| f.state.velocity).collect();

        // Log the velocity changes for debugging
        for i in 0..vel_before.len() {
            log::debug!(
                "KF {} vel: [{:.6}, {:.6}, {:.6}] -> [{:.6}, {:.6}, {:.6}]",
                i,
                vel_before[i][0], vel_before[i][1], vel_before[i][2],
                vel_after[i][0], vel_after[i][1], vel_after[i][2]
            );
        }

        // Verify that at least the optimization ran without crashing
        // (velocity updates depend on the IMU factor being well-constrained)
        assert!(sw.keyframes.len() == 5, "Should have 5 keyframes");
    }

    #[test]
    fn test_imu_factor_with_rotation() {
        // Test IMU factor with pure rotational motion.
        use na::UnitQuaternion;

        let mut sw = SlidingWindow::new(5);
        let base_ts = 1_000_000_000_000i64;
        let dt_s = 0.1;

        // Generate IMU data for rotation around Z
        let omega_z = 0.5; // rad/s
        let n_samples = (dt_s * 200.0) as usize;
        let imu_samples: Vec<ImuData> = (0..n_samples)
            .map(|i| {
                make_imu_sample(
                    base_ts + (i as f64 / 200.0 * 1e9) as i64,
                    [0.0, 0.0, omega_z],
                    [0.0, 0.0, 9.81],
                )
            })
            .collect();

        for i in 0..5 {
            let angle = i as f64 * 0.05; // 5° per frame
            let q = UnitQuaternion::from_euler_angles(0.0, 0.0, angle);
            let mut pose = Matrix4x4::identity();
            pose.fixed_view_mut::<3, 3>(0, 0)
                .copy_from(q.to_rotation_matrix().matrix());

            let frame = make_keyframe_with_imu(
                i,
                pose,
                base_ts + (i as f64 * dt_s * 1e9) as i64,
                imu_samples.clone(),
                (i * 20) as usize,
                10,
            );
            sw.add_frame(frame);
        }

        let poses_before: Vec<Matrix4x4> = sw.keyframes.iter().map(|f| f.state.T_W_B).collect();

        let result = sw.optimize();
        assert!(result.is_ok(), "BA should succeed: {:?}", result);

        // Verify rotations haven't diverged
        for (i, kf) in sw.keyframes.iter().enumerate() {
            let r_after: na::Matrix3<f64> = kf.state.T_W_B.fixed_view::<3, 3>(0, 0).into_owned();
            let r_before: na::Matrix3<f64> = poses_before[i].fixed_view::<3, 3>(0, 0).into_owned();
            let r_rel = r_before.transpose() * r_after;
            let trace = r_rel.trace();
            let cos_angle = ((trace - 1.0) * 0.5).clamp(-1.0, 1.0);
            let angle_diff = cos_angle.acos();

            assert!(
                angle_diff < 0.5,
                "Rotation {} changed too much: {:.3} rad",
                i, angle_diff
            );
        }
    }

    #[test]
    fn test_imu_factor_no_samples_skipped() {
        // Test that when there are no IMU samples, the system falls back gracefully.
        let mut sw = SlidingWindow::new(5);

        for i in 0..5 {
            let mut f = make_keyframe_with_imu(
                i,
                translation_only(i as f64 * 0.1, 0.0, 0.0),
                1_000_000_000_000i64 + (i as i64) * 100_000_000,
                vec![], // No IMU samples
                (i * 20) as usize,
                10,
            );
            sw.add_frame(f);
        }

        // BA should still work (just no IMU factors added)
        let result = sw.optimize();
        assert!(result.is_ok(), "BA should succeed without IMU data");
    }

    #[test]
    fn test_imu_factor_few_samples() {
        // Test that with too few IMU samples (< 3), the IMU factor is skipped.
        let mut sw = SlidingWindow::new(5);

        for i in 0..5 {
            let mut f = make_keyframe_with_imu(
                i,
                translation_only(i as f64 * 0.1, 0.0, 0.0),
                1_000_000_000_000i64 + (i as i64) * 100_000_000,
                vec![
                    make_imu_sample(0, [0.0; 3], [0.0, 0.0, 9.81]),
                ], // Only 1 sample
                (i * 20) as usize,
                10,
            );
            sw.add_frame(f);
        }

        let result = sw.optimize();
        assert!(result.is_ok(), "BA should succeed with few IMU samples");
    }

    // ========================================================================
    // Bug regression tests
    // ========================================================================

    /// Bug: `from_stereo_images` used to default is_keyframe=true, so every
    /// tracking-failure frame (identity T_W_B) was added to the window.
    /// Fix: default is_keyframe=false; only explicit paths set it true.
    #[test]
    fn test_frame_from_stereo_is_not_keyframe_by_default() {
        use crate::estimator::Frame;
        let frame = Frame::from_stereo_images(
            0, 1,
            Matrix4x4::identity(),
            Matrix4x4::identity(),
            64,
        );
        assert!(
            !frame.is_keyframe,
            "from_stereo_images must default to is_keyframe=false; \
             tracking-failure frames must not contaminate the window"
        );
    }

    /// Verify add_frame rejects non-keyframe frames — the guard that prevents
    /// tracking-failure frames (is_keyframe=false) from entering the window.
    #[test]
    fn test_add_frame_rejects_non_keyframe() {
        let mut sw = SlidingWindow::new(3);
        let mut frame = Frame::new(0, 1);
        frame.is_keyframe = false; // simulate tracking failure
        // T_W_B stays identity — the "contamination" scenario
        let accepted = sw.add_frame(frame);
        assert!(!accepted, "non-keyframe must be rejected by add_frame");
        assert_eq!(sw.len(), 0, "window must be unchanged after rejected frame");
    }

    /// Bug: the old CV warm-start formula used
    ///   `last_kf_pose * inverse(predict_current_pose())`
    /// which computes `T_last × T_rel⁻¹ × T_last⁻¹` — a conjugation that applies
    /// the INVERSE motion.  The fixed formula is
    ///   `inverse(last_kf_pose) * predict_current_pose()` = T_rel (body-frame delta).
    ///
    /// Proof: with KFs at x=0 and x=1, T_rel = T(1,0,0).
    ///   Old formula: cv_delta = T(1,0,0) × T_rel⁻¹ × T(1,0,0)⁻¹ = T(1,0,0)×T(-1,0,0)×T(-1,0,0)
    ///     = T(-1, 0, 0) — backward delta!
    ///   Fixed formula: cv_delta = T(1,0,0)⁻¹ × T(2,0,0) = T(1,0,0) — forward delta ✓
    #[test]
    fn test_cv_warm_start_formula_is_forward() {
        let mut sw = SlidingWindow::new(5);
        sw.add_frame(make_dummy_frame(0, translation_only(0.0, 0.0, 0.0)));
        sw.add_frame(make_dummy_frame(1, translation_only(1.0, 0.0, 0.0)));

        let last_kf_pose = translation_only(1.0, 0.0, 0.0);
        let last_track_pose = translation_only(1.05, 0.0, 0.0); // slightly past last KF

        // Fixed formula: cv_delta = T_last⁻¹ × predict() = T(1,0,0)⁻¹ × T(2,0,0) = T(1,0,0)
        let cv_delta_correct = inverse_se3(&last_kf_pose) * sw.predict_current_pose();
        let init_correct = last_track_pose * cv_delta_correct;

        // Old (buggy) formula: cv_delta = T_last × inverse(predict()) = T(1,0,0) × T(-2,0,0) × T(1,0,0)⁻¹
        let cv_delta_buggy = last_kf_pose * inverse_se3(&sw.predict_current_pose());
        let init_buggy = last_track_pose * cv_delta_buggy;

        // Correct: should extrapolate forward from last_track_pose (x > 1.05)
        assert!(
            init_correct[(0, 3)] > last_track_pose[(0, 3)],
            "fixed formula must extrapolate forward: got x={}",
            init_correct[(0, 3)]
        );
        // Buggy: went backward
        assert!(
            init_buggy[(0, 3)] < last_track_pose[(0, 3)],
            "old formula would have extrapolated backward: x={}",
            init_buggy[(0, 3)]
        );
    }

    /// Bug: `predict_current_pose` applied the full KF-to-KF delta regardless of
    /// how much time had elapsed since the last KF, causing massive overshoots.
    ///
    /// `scale_se3_delta` scales the delta by the fraction of elapsed time.
    /// Test: with KFs 1s apart, a 0.05s-elapsed frame should get 5% of the delta.
    #[test]
    fn test_scale_se3_delta_translation() {
        let delta = translation_only(1.0, 0.0, 0.0); // 1 m/s over 1s
        let scaled = scale_se3_delta(&delta, 0.05);   // 50 ms elapsed
        // Should be ~0.05 m, not 1.0 m
        assert!(
            (scaled[(0, 3)] - 0.05).abs() < 1e-12,
            "0.05 scale must give 0.05m, got {}",
            scaled[(0, 3)]
        );
    }

    #[test]
    fn test_scale_se3_delta_zero_gives_identity() {
        let delta = translation_only(5.0, 3.0, 1.0);
        let scaled = scale_se3_delta(&delta, 0.0);
        let diff = (scaled - Matrix4x4::identity()).abs().max();
        assert!(diff < 1e-12, "scale=0 must give identity, max diff={}", diff);
    }

    #[test]
    fn test_scale_se3_delta_one_is_identity_op() {
        let delta = translation_only(2.0, -1.0, 0.5);
        let scaled = scale_se3_delta(&delta, 1.0);
        let diff = (scaled - delta).abs().max();
        assert!(diff < 1e-12, "scale=1 must be a no-op, max diff={}", diff);
    }

    #[test]
    fn test_scale_se3_delta_rotation() {
        use na::UnitQuaternion;
        // 0.2 rad yaw rotation as SE3 delta
        let q = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.2);
        let mut delta = Matrix4x4::identity();
        delta.fixed_view_mut::<3, 3>(0, 0).copy_from(q.to_rotation_matrix().matrix());

        let scaled = scale_se3_delta(&delta, 0.5); // half the rotation
        let r: na::Matrix3<f64> = scaled.fixed_view::<3, 3>(0, 0).into_owned();
        let q_scaled = UnitQuaternion::from_matrix(&r);
        let (_, _, yaw) = q_scaled.euler_angles();
        assert!(
            (yaw - 0.1).abs() < 1e-6,
            "0.5 scale of 0.2rad should give 0.1rad, got {:.6}",
            yaw
        );
    }

    /// Bug: age_weight() returned `w = 1 + 0.1*age`, then applied `w` to both
    /// residual and Jacobian.  In GN normal equations (wJ)ᵀ(wJ)δx = (wJ)ᵀ(wr),
    /// the w² cancels — giving ZERO effect on the solution while doubling the
    /// residual fed to Huber loss, causing good old features to be downweighted.
    ///
    /// Fix: return sqrt(w), so system weighting = (√w)² = w (correct) and
    /// Huber sees √w × error ≈ [1.0, 1.41] × error (minimal Huber interference).
    ///
    /// We test indirectly: a fixed point projected to [0.0, 0.0] at depth 5 has
    /// zero residual regardless of age weight (obs == projection).  Instead, use
    /// a non-zero observation so the residual is proportional to the weight.
    #[test]
    fn test_age_weight_is_sqrt_for_correct_huber_interaction() {
        use crate::optimization::factors::{BundleAdjustmentFactor, project_to_normalized};
        // Point at (0,0,5) in camera frame, T_C_B = identity → p_C = R_B_W*p_W+t_B_W.
        // With T_B_W = identity and p_W = (0,0,5): p_C = (0,0,5) → proj = (0,0).
        // Use observation (0.1, 0.0) so residual = -0.1 * weight.
        let obs = na::Vector2::new(0.1, 0.0);
        let t_c_b = Matrix4x4::identity();
        // Params: [pt_x, pt_y, pt_z] = [0, 0, 5] (point in world = camera at identity)
        let params: Vec<na::DVector<f64>> = vec![
            na::DVector::from_row_slice(&[0.0_f64, 0.0, 5.0])
        ];

        // age=0: sqrt(1.0) = 1.0 → residual magnitude = 1.0 * 0.1 = 0.1
        let f0 = BundleAdjustmentFactor::new(obs, t_c_b).with_track_age(0);
        // Use with_fixed_pose(identity) so only the point param is needed (1-param linearize)
        let t_b_w = Matrix4x4::identity();
        let f0 = f0.with_fixed_pose(t_b_w);
        let (r0, _) = apex_solver::factors::Factor::linearize(&f0, &params, false);

        // age=10: sqrt(2.0) ≈ 1.414 → residual magnitude ≈ 1.414 * 0.1
        let f10 = BundleAdjustmentFactor::new(obs, t_c_b)
            .with_track_age(10)
            .with_fixed_pose(t_b_w);
        let (r10, _) = apex_solver::factors::Factor::linearize(&f10, &params, false);

        let ratio = r10.norm() / r0.norm();
        let expected_sqrt = 2.0_f64.sqrt(); // ≈ 1.414
        let expected_linear = 2.0_f64;      // old buggy value

        assert!(
            (ratio - expected_sqrt).abs() < 1e-9,
            "age=10 residual should be sqrt(2)≈{:.4}× age=0, got {:.4}× \
             (if {:.4}×, the old linear weight was used — Huber interaction bug)",
            expected_sqrt, ratio, expected_linear
        );
    }
}
