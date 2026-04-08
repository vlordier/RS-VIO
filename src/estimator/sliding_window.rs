use crate::datasets::ImuData;
use crate::estimator::Frame;
use crate::optimization::factors::{BundleAdjustmentFactor, PnPFactor};
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
use std::collections::HashMap;
use std::collections::VecDeque;

/// Compute the inverse of an SE(3) transform matrix using the closed-form solution.
///
/// For `T = [R | t; 0 | 1]`, the inverse is `T⁻¹ = [Rᵀ | -Rᵀ·t; 0 | 1]`.
/// This is O(n²) vs O(n³) for a full matrix inverse.
#[inline]
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
/// Implements discrete pre-integration:
/// ```text
/// ΔR_k+1 = ΔR_k · Exp(ω_k · Δt)
/// Δv_k+1 = Δv_k + ΔR_k · a_k · Δt
/// Δp_k+1 = Δp_k + Δv_k · Δt + ½ · ΔR_k · a_k · Δt²
/// ```
///
/// Returns the relative transform T_prev_to_curr as a 4x4 matrix.
/// If fewer than 2 IMU samples, returns identity.
pub fn preintegrate_imu(imu_samples: &[ImuData]) -> Matrix4x4 {
    if imu_samples.len() < 2 {
        return Matrix4x4::identity();
    }

    let g = Vector3::new(0.0, 0.0, 9.81); // gravity in world z-down frame

    let mut delta_r = na::Matrix3::<f64>::identity(); // relative rotation
    let mut delta_v = Vector3::zeros(); // relative velocity
    let mut delta_p = Vector3::zeros(); // relative position

    for i in 0..imu_samples.len() - 1 {
        let curr = &imu_samples[i];
        let next = &imu_samples[i + 1];
        let dt = (next.timestamp - curr.timestamp) as f64 / 1e9; // ns → s
        if dt <= 0.0 {
            continue;
        }

        // Gyroscope: integrate rotation via exponential map (first-order approx)
        let omega = Vector3::new(curr.gyro[0], curr.gyro[1], curr.gyro[2]);
        let theta = omega.norm();
        if theta > 1e-8 {
            // Rodrigues' rotation formula for Exp(ω·Δt)
            let axis = omega / theta;
            let angle = theta * dt;
            let ca = angle.cos();
            let sa = angle.sin();
            let ia = 1.0 - ca;
            let [x, y, z] = [axis.x, axis.y, axis.z];
            let r_step = na::Matrix3::new(
                ca + x * x * ia,     x * y * ia - z * sa,  x * z * ia + y * sa,
                y * x * ia + z * sa, ca + y * y * ia,      y * z * ia - x * sa,
                z * x * ia - y * sa, z * y * ia + x * sa,  ca + z * z * ia,
            );
            delta_r *= r_step;
        }

        // Accelerometer: integrate velocity and position (gravity compensated in world frame)
        let a_body = Vector3::new(curr.accel[0], curr.accel[1], curr.accel[2]);
        let a_world = delta_r * a_body - g;

        delta_p = delta_p + delta_v * dt + 0.5 * a_world * dt * dt;
        delta_v += a_world * dt;
    }

    // Build SE(3) matrix: [delta_r | delta_p; 0 | 1]
    let mut t_rel = Matrix4x4::identity();
    t_rel.fixed_view_mut::<3, 3>(0, 0).copy_from(&delta_r);
    t_rel.fixed_view_mut::<3, 1>(0, 3).copy_from(&delta_p);
    t_rel
}

/// Sliding window of keyframes for bundle adjustment optimization.
///
/// Maintains a fixed-size window of keyframes and manages the optimization
/// of poses and 3D points across these frames.
#[derive(Debug)]
pub struct SlidingWindow {
    /// Maximum number of keyframes in the sliding window.
    max_frames: usize,

    /// Current keyframes in the sliding window (ordered by insertion time, oldest first).
    keyframes: VecDeque<Frame>,

    /// Map points stored by feature ID: HashMap<feature_id, [x, y, z]>
    pub map_points: HashMap<usize, [f32; 3]>,
}

impl SlidingWindow {
    #![allow(non_snake_case)]
    /// Create a new sliding window with the specified maximum number of frames.
    ///
    /// # Arguments
    /// * `max_frames` - Maximum number of keyframes to keep in the window (default: 16)
    pub fn new(max_frames: usize) -> Self {
        Self {
            max_frames,
            keyframes: VecDeque::with_capacity(max_frames),
            map_points: HashMap::new(),
        }
    }
}

impl Default for SlidingWindow {
    fn default() -> Self {
        Self::new(8)
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
    /// This is more efficient than [`get_keyframe_poses`] when only the last
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

    fn build_solver_config(&self) -> LevenbergMarquardtConfig {
        LevenbergMarquardtConfig::new()
            .with_linear_solver_type(LinearSolverType::SparseSchurComplement)
            .with_schur_variant(SchurVariant::Sparse)
            .with_schur_preconditioner(SchurPreconditioner::BlockDiagonal)
            .with_max_iterations(20)
            .with_cost_tolerance(1e-6)
            .with_parameter_tolerance(1e-9)
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

    pub fn optimize(&mut self) -> Result<bool, std::io::Error> {
        self.check_sliding_window_size_for_optimization()?;

        // Initialize problem and solver
        let mut problem = Problem::new(JacobianMode::Sparse);
        let mut solver = LevenbergMarquardt::with_config(self.build_solver_config());
        let mut initial_values = HashMap::new();
        // solver.add_observer(TerminalObserver::new());

        // Initialize maps for tracking and counting observations
        let mut map_feature_to_landmark: HashMap<usize, String> = HashMap::new();
        // Count observations per feature_id, separately for left and right cameras
        let mut left_obs_count: HashMap<usize, usize> = HashMap::new();
        let mut right_obs_count: HashMap<usize, usize> = HashMap::new();

        // Fetch transforms between cameras and body
        let T_Cl_B = self
            .keyframes
            .front()
            .expect("keyframes should not be empty")
            .state
            .T_B_Cl
            .try_inverse()
            .expect("T_B_Cl should be invertible");
        let T_Cr_B = self
            .keyframes
            .front()
            .expect("keyframes should not be empty")
            .state
            .T_B_Cr
            .try_inverse()
            .expect("T_B_Cr should be invertible");

        // Count observations for each landmark across all frames, separately for left and right cameras
        for frame in self.keyframes.iter() {
            // Count left camera observations
            for feat in frame.left_features.iter() {
                map_feature_to_landmark
                    .entry(feat.feature_id)
                    .or_insert_with(|| format!("pt_{}", feat.feature_id));
                *left_obs_count.entry(feat.feature_id).or_insert(0) += 1;
            }

            // Count right camera observations
            for feat in frame.right_features.iter() {
                map_feature_to_landmark
                    .entry(feat.feature_id)
                    .or_insert_with(|| format!("pt_{}", feat.feature_id));
                *right_obs_count.entry(feat.feature_id).or_insert(0) += 1;
            }
        }

        // Add factors
        let mut stereo_count = 0usize;
        let mut mono_count = 0usize;
        let mut counted_landmarks: std::collections::HashSet<usize> = std::collections::HashSet::new();
        for (id_frame, frame) in self.keyframes.iter().enumerate() {
            // Add KF poses
            let kf_var = format!("KF_{}", id_frame);
            let T_B_W = frame
                .state
                .T_W_B
                .try_inverse()
                .expect("T_W_B should be invertible");
            let t_B_W = T_B_W.fixed_view::<3, 1>(0, 3);
            let R_B_W = Matrix3x3::from(T_B_W.fixed_view::<3, 3>(0, 0));
            let q_B_W = UnitQuaternion::from_matrix(&R_B_W);
            let se3_data = DVector::from_row_slice(&[
                t_B_W.x, t_B_W.y, t_B_W.z, q_B_W.w, q_B_W.i, q_B_W.j, q_B_W.k,
            ]);
            // println!("KF_{} initial pose: {:?}", frame.frame_id, se3_data);
            initial_values.insert(kf_var.clone(), (ManifoldType::SE3, se3_data.cast::<f64>()));

            // Process features from both cameras
            let camera_features = [
                (&frame.left_features, T_Cl_B),
                (&frame.right_features, T_Cr_B),
            ];

            for (features, T_C_B) in camera_features.iter() {
                for feat in features.iter() {
                    let feature_id = feat.feature_id;
                    let lm_var = map_feature_to_landmark
                        .get(&feature_id)
                        .expect("feature_id should be in map from counting phase");

                    // Accept landmarks seen in at least one camera.
                    // Stereo landmarks (both cameras) are preferred but mono is allowed
                    // to prevent "no landmark variables" errors in the Schur solver.
                    let count_left = left_obs_count.get(&feature_id).copied().unwrap_or(0);
                    let count_right = right_obs_count.get(&feature_id).copied().unwrap_or(0);
                    let is_stereo = count_left > 0 && count_right > 0;
                    let has_any = count_left > 0 || count_right > 0;

                    if has_any {
                        // Count unique landmarks (stereo vs mono)
                        if counted_landmarks.insert(feature_id) {
                            if is_stereo { stereo_count += 1; } else { mono_count += 1; }
                        }

                        // Create initial value for landmark if not already present
                        initial_values.entry(lm_var.clone()).or_insert_with(|| {
                            let data = if let Some(&last_pos) = self.map_points.get(&feature_id) {
                                DVector::from_row_slice(&[
                                    last_pos[0] as f64,
                                    last_pos[1] as f64,
                                    last_pos[2] as f64,
                                ])
                            } else {
                                // Default initialization if not in map_points
                                // TODO Triangulate insrtead of assigning depth 4.0 (quick and dirty way to get going)
                                let p_C = Vector3::new(
                                    feat.undistorted_coord[0] as f64,
                                    feat.undistorted_coord[1] as f64,
                                    2.0_f64,
                                );
                                let (R_W_B, t_W_B) = (
                                    frame.state.T_W_B.fixed_view::<3, 3>(0, 0).into_owned(),
                                    frame.state.T_W_B.fixed_view::<3, 1>(0, 3).into_owned(),
                                );
                                let T_B_C =
                                    T_C_B.try_inverse().expect("T_C_B should be invertible");
                                let (R_B_C, t_B_C) = (
                                    T_B_C.fixed_view::<3, 3>(0, 0).into_owned(),
                                    T_B_C.fixed_view::<3, 1>(0, 3).into_owned(),
                                );
                                let p_W = R_W_B * (R_B_C * p_C + t_B_C) + t_W_B;
                                DVector::from_row_slice(&[p_W.x, p_W.y, p_W.z])
                            };
                            (ManifoldType::RN, data)
                        });

                        // Create camera projection factor
                        let mut factor = BundleAdjustmentFactor::new(
                            na::Vector2::new(feat.undistorted_coord[0], feat.undistorted_coord[1])
                                .cast::<f64>(),
                            *T_C_B,
                        );

                        // Fix pose for first frame
                        if id_frame == 0 {
                            // Factor expects T_B_W (Body-from-World), but frame.state.T_W_B is World-from-Body
                            let T_B_W = frame
                                .state
                                .T_W_B
                                .try_inverse()
                                .expect("T_W_B should be invertible");
                            factor = factor.with_fixed_pose(T_B_W);
                        }

                        // Determine variable names based on frame
                        let var_names: Vec<&str> = if id_frame == 0 {
                            vec![&lm_var]
                        } else {
                            vec![&lm_var, &kf_var]
                        };

                        // Add residual block with Huber loss
                        let huber_loss = HuberLoss::new(2.0).unwrap();
                        problem.add_residual_block(
                            &var_names,
                            Box::new(factor),
                            Some(Box::new(huber_loss)),
                        );
                    }
                }
            }
        }

        let num_residuals = problem.num_residual_blocks();
        let num_variables = initial_values.len();

        log::debug!(
            "Added {} variables total ({} SE3 + {} R3), {} residual blocks ({} stereo + {} mono landmarks)",
            num_variables,
            self.keyframes.len(),
            stereo_count + mono_count,
            num_residuals,
            stereo_count,
            mono_count
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

        // Save state for rollback (only after validation passes — skip if early return above)
        let saved_keyframe_poses: Vec<Matrix4x4> =
            self.keyframes.iter().map(|f| f.state.T_W_B).collect();
        let saved_map_points = self.map_points.clone();

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
                            .with_parameter_tolerance(1e-9)
                            .with_jacobi_scaling(false),
                    );

                    match fallback_solver.optimize(&problem, &initial_values) {
                        Ok(result) => {
                            log::debug!("[SlidingWindow] Fallback solver succeeded");
                            result
                        }
                        Err(e2) => {
                            log::error!("[SlidingWindow] Both Schur complement and fallback solver failed: {:?} - reverting to previous state", e2);
                            self.revert_to_saved_state(&saved_keyframe_poses, &saved_map_points);
                            return Ok(false);
                        }
                    }
                } else {
                    // Other optimization errors - revert to saved state
                    log::error!(
                        "[SlidingWindow] Optimization error: {:?} - reverting to previous state",
                        e
                    );
                    self.revert_to_saved_state(&saved_keyframe_poses, &saved_map_points);
                    return Ok(false);
                }
            }
        };

        // Check if optimization was successful based on status
        let is_successful = self.is_optimization_successful(&opt_result);

        if is_successful {
            // Process successful optimization result
            self.process_optimization_result(&opt_result);
            log::debug!(
                "[SlidingWindow] Optimization successful. Initial cost: {:.3}, final cost: {:.3}",
                opt_result.initial_cost,
                opt_result.final_cost
            );
            Ok(true)
        } else {
            // Optimization failed - revert to saved state
            log::warn!(
                "[SlidingWindow] Optimization failed (status: {:?}) - reverting to previous state",
                opt_result.status
            );
            self.revert_to_saved_state(&saved_keyframe_poses, &saved_map_points);
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

    /// Revert keyframe poses and map points to saved state
    fn revert_to_saved_state(
        &mut self,
        saved_keyframe_poses: &[Matrix4x4],
        saved_map_points: &HashMap<usize, [f32; 3]>,
    ) {
        // Restore keyframe poses
        for (i, frame) in self.keyframes.iter_mut().enumerate() {
            if i < saved_keyframe_poses.len() {
                frame.state.T_W_B = saved_keyframe_poses[i];
            }
        }

        // Restore map points
        self.map_points.clear();
        self.map_points
            .extend(saved_map_points.iter().map(|(k, v)| (*k, *v)));

        log::debug!(
            "[SlidingWindow] Reverted {} keyframe poses and {} map points",
            saved_keyframe_poses.len(),
            saved_map_points.len()
        );
    }

    fn process_optimization_result(
        &mut self,
        opt_result: &SolverResult<HashMap<String, VariableEnum>>,
    ) {
        // TODO: handle error properly

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

        // Update map_points and keyframe poses with optimized values
        self.map_points.clear();
        opt_result.parameters.iter().for_each(|(var_name, value)| {
            // Update map points
            if let Some(feature_id_str) = var_name.strip_prefix("pt_") {
                if let Ok(feature_id) = feature_id_str.parse::<usize>() {
                    let vec = value.to_vector();
                    // TODO check if points are estimated at obviously wrong locations (negative depths, etc.)
                    self.map_points
                        .insert(feature_id, [vec[0] as f32, vec[1] as f32, vec[2] as f32]);
                }
            }
            // Update keyframe poses
            else if let Some(frame_id_str) = var_name.strip_prefix("KF_") {
                if let Ok(frame_id) = frame_id_str.parse::<i32>() {
                    let mat = apex_solver::manifold::se3::SE3::from(value.to_vector()).matrix();
                    if let Some(kf) = self.keyframes.get_mut(frame_id as usize) {
                        kf.state.T_W_B = mat.try_inverse().expect("T_W_B should be invertible");
                    } else {
                        log::warn!(
                            "[SlidingWindow] Optimized keyframe {} not found, skipping update",
                            frame_id
                        );
                    }
                }
            }
        });
    }

    /// Track the motion of the system by solving a PnP-like problem
    /// The map points and existing keyframes are kept constant, only the new frame is optimized
    pub fn track_motion(&mut self, frame: &Frame) -> Result<Option<Matrix4x4>, std::io::Error> {
        // Create a new problem and solver
        let mut problem = Problem::new(JacobianMode::Sparse);
        let mut solver = LevenbergMarquardt::with_config(
            LevenbergMarquardtConfig::new()
                .with_linear_solver_type(LinearSolverType::SparseCholesky)
                .with_max_iterations(10)
                .with_cost_tolerance(1e-6)
                .with_parameter_tolerance(1e-9)
                .with_jacobi_scaling(false),
        );
        let mut initial_values = HashMap::new();
        // solver.add_observer(TerminalObserver::new());

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

        let init_T_W_B = if !frame.imu_from_last_frame.is_empty() {
            let delta_t = preintegrate_imu(&frame.imu_from_last_frame);
            last_kf_pose * delta_t
        } else {
            self.predict_current_pose()
        };

        let T_B_W = inverse_se3(&init_T_W_B);
        let t_B_W = T_B_W.fixed_view::<3, 1>(0, 3);
        let R_B_W = Matrix3x3::from(T_B_W.fixed_view::<3, 3>(0, 0));
        let q_B_W = UnitQuaternion::from_matrix(&R_B_W);
        let se3_data = DVector::from_row_slice(&[
            t_B_W.x, t_B_W.y, t_B_W.z, q_B_W.w, q_B_W.i, q_B_W.j, q_B_W.k,
        ]);
        initial_values.insert(kf_var.clone(), (ManifoldType::SE3, se3_data.cast::<f64>()));

        // Add factors: for both the left and right cameras, each point that was already in the map is used to optimize the new frame
        // Fetch transforms between cameras and body
        let T_Cl_B = self
            .keyframes
            .front()
            .expect("keyframes should not be empty")
            .state
            .T_B_Cl
            .try_inverse()
            .expect("T_B_Cl should be invertible");
        let T_Cr_B = self
            .keyframes
            .front()
            .expect("keyframes should not be empty")
            .state
            .T_B_Cr
            .try_inverse()
            .expect("T_B_Cr should be invertible");
        let camera_features = [
            (&frame.left_features, T_Cl_B),
            (&frame.right_features, T_Cr_B),
        ];
        for (features, T_C_B) in camera_features.iter() {
            for feat in features.iter() {
                let feature_id = feat.feature_id;

                let point = self.map_points.get(&feature_id);
                match point {
                    Some(point) => {
                        // Create PnP factor
                        let factor = PnPFactor::new(
                            na::Vector2::new(feat.undistorted_coord[0], feat.undistorted_coord[1])
                                .cast::<f64>(),
                            *T_C_B,
                            na::Vector3::new(point[0] as f64, point[1] as f64, point[2] as f64),
                        );
                        // Add residual block with Huber loss
                        let huber_loss = HuberLoss::new(2.0).unwrap();
                        problem.add_residual_block(
                            &[&kf_var],
                            Box::new(factor),
                            Some(Box::new(huber_loss)),
                        );
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

        if is_successful {
            // Extract optimized pose T_B_W and convert to T_W_B
            if let Some(value) = opt_result.parameters.get(&kf_var) {
                let T_B_W_opt = apex_solver::manifold::se3::SE3::from(value.to_vector()).matrix();
                let T_W_B_opt = T_B_W_opt
                    .try_inverse()
                    .expect("Optimized T_B_W should be invertible");
                log::debug!(
                    "[SlidingWindow] Motion tracking successful. Initial cost: {:.3}, final cost: {:.3}",
                    opt_result.initial_cost,
                    opt_result.final_cost
                );
                Ok(Some(T_W_B_opt))
            } else {
                log::warn!("[SlidingWindow] Motion tracking: optimized pose not found in result");
                Ok(None)
            }
        } else {
            log::warn!(
                "[SlidingWindow] Motion tracking failed (status: {:?})",
                opt_result.status
            );
            Ok(None)
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
}
