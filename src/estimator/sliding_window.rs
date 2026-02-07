//! Sliding window optimization for visual-inertial SLAM
//!
//! Uses Levenberg-Marquardt optimization with Schur complement for efficient bundle adjustment.
//! Panics on unwrap/expect are acceptable in optimization code - they indicate data corruption.

#![allow(clippy::unwrap_used, clippy::expect_used)] // Optimization code - panics indicate data corruption

use crate::estimator::Frame;
use crate::optimization::factors::{BundleAdjustmentFactor, PnPFactor};
use crate::types::{Matrix3x3, Matrix4x4, Vector3};
use apex_solver::core::loss_functions::HuberLoss;
use apex_solver::core::problem::{Problem, VariableEnum};
use apex_solver::linalg::LinearSolverType;
use apex_solver::manifold::ManifoldType;
use apex_solver::optimizer::levenberg_marquardt::{LevenbergMarquardt, LevenbergMarquardtConfig};
use apex_solver::optimizer::SolverResult;
use na::{DVector, UnitQuaternion};
use nalgebra as na;
use rayon::prelude::*;
use std::collections::HashMap;
use std::collections::VecDeque;
use std::sync::Arc;

/// Type alias for bundle adjustment residual tuples to improve readability
/// (landmark_var, keyframe_var, factor, loss_function)
type ResidualTuple = (
    Arc<String>,
    Option<Arc<String>>,
    BundleAdjustmentFactor,
    Option<Box<dyn apex_solver::core::loss_functions::LossFunction + Send>>,
);

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

    /// Create a new sliding window with the default size of 8 keyframes.
    pub fn with_default_size() -> Self {
        Self::new(8)
    }

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
    #[inline]
    pub fn len(&self) -> usize {
        self.keyframes.len()
    }

    /// Check if the sliding window is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.keyframes.is_empty()
    }

    /// Check if the sliding window is full.
    #[inline]
    pub fn is_full(&self) -> bool {
        self.keyframes.len() >= self.max_frames
    }

    pub fn get_keyframe_poses(&self) -> Vec<Matrix4x4> {
        // Return poses of body in world frame
        self.keyframes.iter().map(|f| f.state.T_W_B).collect()
    }

    /// Get the pose of the last keyframe without allocating.
    pub fn last_keyframe_pose(&self) -> Option<Matrix4x4> {
        self.keyframes.back().map(|f| f.state.T_W_B)
    }

    /// Clear all keyframes from the sliding window.
    pub fn clear(&mut self) {
        self.keyframes.clear();
        log::debug!("[SlidingWindow] Cleared all keyframes");
    }

    fn build_solver_config(&self) -> LevenbergMarquardtConfig {
        LevenbergMarquardtConfig::new()
            .with_linear_solver_type(LinearSolverType::SparseCholesky)
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

    pub fn build_optimization_problem(
        &self,
    ) -> (Problem, HashMap<String, (ManifoldType, DVector<f64>)>) {
        let mut problem = Problem::new();

        // PART 1: Count observations and find first observation frame (Parallel)
        // Map: feature_id -> (left_count, right_count, first_frame_idx)
        // Sequential scan — with only 8-16 keyframes, rayon overhead exceeds benefit
        let mut landmark_stats: HashMap<usize, (usize, usize, usize)> = HashMap::new();
        for (frame_idx, frame) in self.keyframes.iter().enumerate() {
            for feat in &frame.left_features {
                let entry = landmark_stats
                    .entry(feat.feature_id)
                    .or_insert((0, 0, usize::MAX));
                entry.0 += 1;
                if frame_idx < entry.2 {
                    entry.2 = frame_idx;
                }
            }
            for feat in &frame.right_features {
                let entry = landmark_stats
                    .entry(feat.feature_id)
                    .or_insert((0, 0, usize::MAX));
                entry.1 += 1;
                if frame_idx < entry.2 {
                    entry.2 = frame_idx;
                }
            }
        }

        // Pre-generate feature variable strings
        // This avoids calling format!("LM_{}") thousands of times during building
        let lm_string_cache: HashMap<usize, Arc<String>> = landmark_stats
            .keys()
            .map(|&id| (id, Arc::new(format!("LM_{}", id))))
            .collect();

        // Fetch transforms
        let first_frame = self
            .keyframes
            .front()
            .expect("Keyframes should not be empty");
        #[allow(clippy::unwrap_used, clippy::expect_used)]
        let T_Cl_B = Arc::new(
            first_frame
                .state
                .T_B_Cl
                .try_inverse()
                .expect("T_B_Cl should be invertible"),
        );
        #[allow(clippy::unwrap_used, clippy::expect_used)]
        let T_Cr_B = Arc::new(
            first_frame
                .state
                .T_B_Cr
                .try_inverse()
                .expect("T_B_Cr should be invertible"),
        );

        // Pre-compute body-from-camera transforms once (these are the originals, no double-inversion)
        let T_B_Cl = first_frame.state.T_B_Cl;
        let T_B_Cr = first_frame.state.T_B_Cr;

        // PART 2: Build Factors (Parallel)
        let results: Vec<_> = self
            .keyframes
            .par_iter()
            .enumerate()
            .map(|(id_frame, frame)| {
                // Pre-allocate to avoid re-allocations
                let mut local_initials = Vec::with_capacity(300);

                // Use a tuple structure for residuals to avoid allocating Vec<String>
                // (lm_var, Option<kf_var>, factor, loss)
                // Stores Arc<String> instead of String to avoid duplication
                let mut local_residuals: Vec<ResidualTuple> = Vec::with_capacity(300);

                // Add KF pose
                let kf_var = Arc::new(format!("KF_{}", id_frame));
                #[allow(clippy::expect_used)]
                let T_B_W = frame
                    .state
                    .T_W_B
                    .try_inverse()
                    .expect("T_W_B should be invertible");
                let t_B_W = T_B_W.fixed_view::<3, 1>(0, 3);
                let R_B_W = Matrix3x3::from(T_B_W.fixed_view::<3, 3>(0, 0));
                let q_B_W = UnitQuaternion::from_matrix(&R_B_W);
                let se3_data = DVector::from_column_slice(&[
                    t_B_W.x, t_B_W.y, t_B_W.z, q_B_W.w, q_B_W.i, q_B_W.j, q_B_W.k,
                ]);
                local_initials.push(((*kf_var).clone(), (ManifoldType::SE3, se3_data)));

                // Pre-compute fixed pose Arc for frame 0 (avoids recomputing inversion per-feature)
                let fixed_pose_arc = if id_frame == 0 {
                    Some(Arc::new(T_B_W))
                } else {
                    None
                };

                // Use pre-computed body-from-camera transforms (hoisted before par_iter,
                // avoids double-inversion of T_C_B→T_B_C per frame)
                let camera_features = [
                    (&frame.left_features, T_Cl_B.clone(), &T_B_Cl),
                    (&frame.right_features, T_Cr_B.clone(), &T_B_Cr),
                ];

                for (features, T_C_B, T_B_C) in camera_features.iter() {
                    for feat in features.iter() {
                        let feature_id = feat.feature_id;

                        // Use stats lookup
                        if let Some((count_left, count_right, first_frame_idx)) =
                            landmark_stats.get(&feature_id)
                        {
                            if *count_left > 0 && *count_right > 0 {
                                // Retrieve cached string
                                let lm_var_arc =
                                    lm_string_cache.get(&feature_id).expect("Cache sync error");

                                // Initial Value logic: Only if not already known AND this is the first responsible frame
                                if !self.map_points.contains_key(&feature_id)
                                    && id_frame == *first_frame_idx
                                {
                                    let default_depth = 2.0_f64;
                                    let p_C = Vector3::new(
                                        feat.undistorted_coord[0] as f64 * default_depth,
                                        feat.undistorted_coord[1] as f64 * default_depth,
                                        default_depth,
                                    );
                                    let (R_W_B, t_W_B) = (
                                        frame.state.T_W_B.fixed_view::<3, 3>(0, 0).into_owned(),
                                        frame.state.T_W_B.fixed_view::<3, 1>(0, 3).into_owned(),
                                    );
                                    let (R_B_C, t_B_C) = (
                                        T_B_C.fixed_view::<3, 3>(0, 0).into_owned(),
                                        T_B_C.fixed_view::<3, 1>(0, 3).into_owned(),
                                    );
                                    let p_W = R_W_B * (R_B_C * p_C + t_B_C) + t_W_B;
                                    let data = DVector::from_column_slice(&[p_W.x, p_W.y, p_W.z]);
                                    local_initials
                                        .push(((**lm_var_arc).clone(), (ManifoldType::RN, data)));
                                }

                                // Factor logic
                                let mut factor = BundleAdjustmentFactor::new(
                                    na::Vector2::new(
                                        feat.undistorted_coord[0],
                                        feat.undistorted_coord[1],
                                    )
                                    .cast::<f64>(),
                                    (*T_C_B).clone(),
                                );

                                if let Some(ref pose_arc) = fixed_pose_arc {
                                    factor = factor.with_fixed_pose(Arc::clone(pose_arc));
                                }

                                let kf_var_opt = if id_frame == 0 {
                                    None
                                } else {
                                    Some(kf_var.clone())
                                };

                                let huber_loss = HuberLoss::new(2.0).ok();
                                // Store Arc<String>
                                local_residuals.push((
                                    lm_var_arc.clone(),
                                    kf_var_opt,
                                    factor,
                                    huber_loss.map(|l| {
                                        Box::new(l)
                                            as Box<
                                                dyn apex_solver::core::loss_functions::LossFunction
                                                    + Send,
                                            >
                                    }),
                                ));
                            }
                        }
                    }
                }
                (local_initials, local_residuals)
            })
            .collect();

        // PART 3: Aggregate into Problem and Initial Values
        let (all_initials_maps, all_residuals_vecs): (Vec<_>, Vec<_>) = results.into_iter().unzip();

        let mut initial_values =
            all_initials_maps
                .into_iter()
                .fold(HashMap::new(), |mut acc, block_initials| {
                    for (k, v) in block_initials {
                        acc.entry(k).or_insert(v);
                    }
                    acc
                });

        // Add already known landmarks (avoid re-computing them in every frame)
        // Use the cache to filter only ACTIVE landmarks
        for (id, lm_var_arc) in &lm_string_cache {
            if let Some(pos) = self.map_points.get(id) {
                // Only add if not already present (new initial values take precedence if collision - but shouldn't happen due to if check above)
                initial_values
                    .entry((**lm_var_arc).clone())
                    .or_insert_with(|| {
                        (
                            ManifoldType::RN,
                            DVector::from_column_slice(&[
                                pos[0] as f64,
                                pos[1] as f64,
                                pos[2] as f64,
                            ]),
                        )
                    });
            }
        }

        for residuals in all_residuals_vecs {
            for (lm_var, kf_var_opt, factor, loss) in residuals {
                if let Some(kf_val) = kf_var_opt {
                    // Pass &str to solver
                    problem.add_residual_block(&[&lm_var, &*kf_val], Box::new(factor), loss);
                } else {
                    problem.add_residual_block(&[&lm_var], Box::new(factor), loss);
                }
            }
        }

        (problem, initial_values)
    }

    pub fn optimize(&mut self) -> Result<bool, std::io::Error> {
        self.check_sliding_window_size_for_optimization()?;

        // Save current state before optimization for potential rollback
        let saved_keyframe_poses: Vec<Matrix4x4> =
            self.keyframes.iter().map(|f| f.state.T_W_B).collect();
        let saved_map_points = self.map_points.clone();

        // Initialize problem and solver
        let mut solver = LevenbergMarquardt::with_config(self.build_solver_config());
        // solver.add_observer(TerminalObserver::new());

        let (problem, initial_values) = self.build_optimization_problem();

        let num_residuals = problem.num_residual_blocks();
        let num_variables = initial_values.len();

        log::debug!(
            "Added SE3 and R3 variables, now {} variables total, {} residual blocks",
            num_variables,
            num_residuals
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

        let landmark_variables = initial_values
            .keys()
            .filter(|k| k.starts_with("LM_"))
            .count();
        if landmark_variables == 0 {
            log::warn!("[SlidingWindow] No landmark variables found; skipping optimization");
            return Ok(false);
        }

        // Try optimization with Schur complement (fallback to SparseCholesky on singular matrix)
        let opt_result = match solver.optimize(&problem, &initial_values) {
            Ok(result) => result,
            Err(e) => {
                // Check if it's a linear solve failure (singular matrix)
                let error_str = format!("{:?}", e);
                if error_str.contains("LinearSolveFailed") || error_str.contains("Singular matrix")
                {
                    log::warn!("[SlidingWindow] Schur complement failed with singular matrix, trying fallback solver (SparseQR)");

                    // Create fallback solver with SparseQR (handles rank-deficient systems)
                    let mut fallback_solver = LevenbergMarquardt::with_config(
                        LevenbergMarquardtConfig::new()
                            .with_linear_solver_type(LinearSolverType::SparseQR)
                            .with_max_iterations(20)
                            .with_cost_tolerance(1e-6)
                            .with_parameter_tolerance(1e-9)
                            .with_jacobi_scaling(false),
                    );

                    match fallback_solver.optimize(&problem, &initial_values) {
                        Ok(result) => {
                            log::debug!("[SlidingWindow] Fallback solver succeeded");
                            result
                        },
                        Err(e2) => {
                            log::error!("[SlidingWindow] Both Schur complement and fallback solver failed: {:?} - reverting to previous state", e2);
                            self.revert_to_saved_state(&saved_keyframe_poses, &saved_map_points);
                            return Ok(false);
                        },
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
            },
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
    const fn is_optimization_successful(
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

        // Restore map points (clone_from reuses existing allocation)
        self.map_points.clone_from(saved_map_points);

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
        // Note: error handling deferred — opt_result status is logged below;
        // non-converged results are already handled by is_optimization_successful().

        // Determine convergence status accurately
        let (status, convergence_reason): (&str, &str) = match &opt_result.status {
            apex_solver::optimizer::OptimizationStatus::Converged => ("CONVERGED", "Converged"),
            apex_solver::optimizer::OptimizationStatus::CostToleranceReached => {
                ("CONVERGED", "CostTolerance")
            },
            apex_solver::optimizer::OptimizationStatus::ParameterToleranceReached => {
                ("CONVERGED", "ParameterTolerance")
            },
            apex_solver::optimizer::OptimizationStatus::GradientToleranceReached => {
                ("CONVERGED", "GradientTolerance")
            },
            apex_solver::optimizer::OptimizationStatus::TrustRegionRadiusTooSmall => {
                ("CONVERGED", "TrustRegionRadiusTooSmall")
            },
            apex_solver::optimizer::OptimizationStatus::MinCostThresholdReached => {
                ("CONVERGED", "MinCostThresholdReached")
            },
            apex_solver::optimizer::OptimizationStatus::MaxIterationsReached => {
                ("NOT_CONVERGED", "MaxIterations")
            },
            apex_solver::optimizer::OptimizationStatus::Timeout => ("NOT_CONVERGED", "Timeout"),
            apex_solver::optimizer::OptimizationStatus::NumericalFailure => {
                ("NOT_CONVERGED", "NumericalFailure")
            },
            apex_solver::optimizer::OptimizationStatus::IllConditionedJacobian => {
                ("NOT_CONVERGED", "IllConditionedJacobian")
            },
            apex_solver::optimizer::OptimizationStatus::InvalidNumericalValues => {
                ("NOT_CONVERGED", "InvalidNumericalValues")
            },
            apex_solver::optimizer::OptimizationStatus::UserTerminated => {
                ("NOT_CONVERGED", "UserTerminated")
            },
            apex_solver::optimizer::OptimizationStatus::Failed(_msg) => ("NOT_CONVERGED", "Failed"),
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
            if let Some(feature_id_str) = var_name.strip_prefix("LM_") {
                if let Ok(feature_id) = feature_id_str.parse::<usize>() {
                    let vec = value.to_vector();
                    // NaN/Inf guard: skip corrupted landmarks to prevent cascading failures
                    if !vec.iter().all(|v| v.is_finite()) {
                        log::warn!("[SlidingWindow] Skipping NaN/Inf landmark LM_{feature_id}");
                        return;
                    }
                    self.map_points
                        .insert(feature_id, [vec[0] as f32, vec[1] as f32, vec[2] as f32]);
                }
            }
            // Update keyframe poses
            else if let Some(frame_id_str) = var_name.strip_prefix("KF_") {
                if let Ok(frame_id) = frame_id_str.parse::<usize>() {
                    let mat = apex_solver::manifold::se3::SE3::from(value.to_vector()).matrix();
                    // NaN/Inf guard: skip corrupted poses
                    if !mat.iter().all(|v| v.is_finite()) {
                        log::warn!("[SlidingWindow] Skipping NaN/Inf keyframe KF_{frame_id}");
                        return;
                    }
                    let Some(kf) = self.keyframes.get_mut(frame_id) else {
                        log::warn!("Optimizer returned unknown keyframe index {frame_id}");
                        return;
                    };
                    match mat.try_inverse() {
                        Some(inv) => kf.state.T_W_B = inv,
                        None => {
                            log::warn!("[SlidingWindow] Near-singular matrix for KF_{frame_id}, skipping update");
                            return;
                        }
                    }
                }
            }
        });
    }

    /// Track the motion of the system by solving a PnP-like problem.
    /// The map points and existing keyframes are kept constant, only the new frame is optimized.
    ///
    /// # Arguments
    /// * `frame` - The current frame with detected features
    /// * `predicted_pose` - Optional predicted T_W_B from constant-velocity model.
    ///   Falls back to last keyframe pose if `None`.
    pub fn track_motion(
        &mut self,
        frame: &Frame,
        predicted_pose: Option<Matrix4x4>,
    ) -> Result<Option<Matrix4x4>, std::io::Error> {
        // Guard: need at least one keyframe
        if self.keyframes.is_empty() {
            return Ok(None);
        }

        // Create a new problem and solver
        let mut problem = Problem::new();
        let mut solver = LevenbergMarquardt::with_config(
            LevenbergMarquardtConfig::new()
                .with_linear_solver_type(LinearSolverType::SparseCholesky)
                .with_max_iterations(20)
                .with_cost_tolerance(1e-6)
                .with_parameter_tolerance(1e-9)
                .with_jacobi_scaling(false),
        );
        let mut initial_values = HashMap::new();

        // Add variable for the new frame
        // Use predicted pose if available, otherwise fall back to last keyframe
        let kf_var = "F".to_string();
        let T_W_B_init =
            predicted_pose.unwrap_or_else(|| self.keyframes.back().unwrap().state.T_W_B);
        let T_B_W = T_W_B_init
            .try_inverse()
            .expect("T_W_B should be invertible");
        let t_B_W = T_B_W.fixed_view::<3, 1>(0, 3);
        let R_B_W = Matrix3x3::from(T_B_W.fixed_view::<3, 3>(0, 0));
        let q_B_W = UnitQuaternion::from_matrix(&R_B_W);
        let se3_data = DVector::from_column_slice(&[
            t_B_W.x, t_B_W.y, t_B_W.z, q_B_W.w, q_B_W.i, q_B_W.j, q_B_W.k,
        ]);
        initial_values.insert(kf_var.clone(), (ManifoldType::SE3, se3_data));

        // Add factors: for both the left and right cameras, each point that was already in the map is used to optimize the new frame
        // Fetch transforms between cameras and body
        let T_Cl_B = self
            .keyframes
            .front()
            .unwrap()
            .state
            .T_B_Cl
            .try_inverse()
            .expect("T_B_Cl should be invertible");
        let T_Cr_B = self
            .keyframes
            .front()
            .unwrap()
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

                if let Some(point) = self.map_points.get(&feature_id) {
                    // Create PnP factor
                    let factor = PnPFactor::new(
                        na::Vector2::new(feat.undistorted_coord[0], feat.undistorted_coord[1])
                            .cast::<f64>(),
                        *T_C_B,
                        na::Vector3::new(point[0] as f64, point[1] as f64, point[2] as f64),
                    );
                    // Add residual block with Huber loss
                    let huber_loss =
                        HuberLoss::new(2.0).expect("HuberLoss threshold must be positive");
                    problem.add_residual_block(
                        &[&kf_var],
                        Box::new(factor),
                        Some(Box::new(huber_loss)),
                    );
                }
            }
        }

        // Minimum correspondences check: PnP needs at least 6 2D-3D correspondences
        let num_residuals = problem.num_residual_blocks();
        if num_residuals < 6 {
            log::warn!(
                "[SlidingWindow] Motion tracking: too few 2D-3D correspondences ({num_residuals}), need >= 6"
            );
            return Ok(None);
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
            },
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
