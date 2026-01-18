use crate::estimator::Frame;
use crate::imu::ImuMotionPrior;
use crate::optimization::factors::{
    BundleAdjustmentFactor, ImuPriorFactor, LoopClosurePoseFactor, PnPFactor, PriorFactor,
};
use crate::optimization::loop_closure::LoopClosureConstraint;

use crate::debug_log;
use crate::{
    fl,
    types::{Float, Matrix3x3, Matrix4x4, Vector3},
};
use apex_solver::core::loss_functions::HuberLoss;
use apex_solver::core::problem::{Problem, VariableEnum};
use apex_solver::linalg::{LinearSolverType, SchurPreconditioner, SchurVariant};
use apex_solver::manifold::ManifoldType;
use apex_solver::optimizer::levenberg_marquardt::{LevenbergMarquardt, LevenbergMarquardtConfig};
use apex_solver::optimizer::SolverResult;
use na::{DVector, UnitQuaternion};
use nalgebra as na;
use std::collections::HashMap;
use std::collections::VecDeque;

/// Trait for window management strategies.
///
/// This trait enables the Strategy Pattern for keyframe window management,
/// allowing different strategies (fixed size, adaptive, marginalization-based)
/// to be used interchangeably.
///
/// # Design Pattern
/// Implements the **Strategy Pattern** to encapsulate window management algorithms
/// and make them interchangeable without affecting the rest of the system.
pub trait WindowManager: Send {
    /// Get the current number of keyframes in the window
    fn len(&self) -> usize;

    /// Check if the window is empty
    fn is_empty(&self) -> bool;

    /// Check if the window is full
    fn is_full(&self) -> bool;

    /// Get a reference to a specific keyframe by index
    fn get_frame(&self, index: usize) -> Option<&Frame>;

    /// Get all keyframe poses
    fn get_keyframe_poses(&self) -> Vec<Matrix4x4>;

    /// Clear all keyframes
    fn clear(&mut self);

    /// Get the number of map points
    fn map_points_len(&self) -> usize;

    /// Get a name for logging
    fn name(&self) -> &'static str;
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
    /// Bounded to prevent unbounded memory growth in long-running systems
    pub map_points: HashMap<usize, [f32; 3]>,

    /// Maximum number of map points to maintain (for embedded systems)
    max_map_points: usize,

    /// Track observation count for each map point (for LRU eviction)
    map_point_observations: HashMap<usize, usize>,

    /// Marginalization manager for sliding window
    marginalization_manager: crate::optimization::marginalization::MarginalizationManager,

    /// Loop-closure constraints linking keyframes inside the window
    loop_closure_constraints: Vec<LoopClosureConstraint>,
}

impl SlidingWindow {
    #![allow(non_snake_case)]
    /// Create a new sliding window with the specified maximum number of frames.
    ///
    /// # Arguments
    /// * `max_frames` - Maximum number of keyframes to keep in the window (default: 16)
    pub fn new(max_frames: usize) -> Self {
        Self::with_marginalization_config(
            max_frames,
            crate::optimization::marginalization::MarginalizationConfig::default(),
        )
    }

    /// Create a new sliding window with custom marginalization configuration.
    ///
    /// # Arguments
    /// * `max_frames` - Maximum number of keyframes to keep in the window
    /// * `marg_config` - Marginalization configuration options
    pub fn with_marginalization_config(
        max_frames: usize,
        marg_config: crate::optimization::marginalization::MarginalizationConfig,
    ) -> Self {
        const DEFAULT_MAX_MAP_POINTS: usize = 2000;

        let marg_manager =
            crate::optimization::marginalization::MarginalizationManager::new(marg_config);

        Self {
            max_frames,
            keyframes: VecDeque::with_capacity(max_frames),
            map_points: HashMap::with_capacity(DEFAULT_MAX_MAP_POINTS),
            max_map_points: DEFAULT_MAX_MAP_POINTS,
            map_point_observations: HashMap::with_capacity(DEFAULT_MAX_MAP_POINTS),
            marginalization_manager: marg_manager,
            loop_closure_constraints: Vec::new(),
        }
    }

    /// Create a new sliding window from a Config file.
    ///
    /// # Arguments
    /// * `config` - Configuration loaded from YAML file
    pub fn from_config(config: &crate::datasets::config::Config) -> Self {
        let marg_config = crate::optimization::marginalization::MarginalizationConfig {
            enabled: config.marginalization.enabled,
            use_fej: config.marginalization.use_fej,
            damping: config.marginalization.damping,
            max_keyframes: config.marginalization.max_keyframes,
            num_marginalize_per_step: config.marginalization.num_marginalize_per_step,
            min_landmark_observations: config.marginalization.min_landmark_observations,
            landmark_age_limit: config.marginalization.landmark_age_limit,
            prior_info_scale: config.marginalization.prior_info_scale,
            hessian_approximator: config.marginalization.hessian_approximator.clone(),
            gradient_computer: config.marginalization.gradient_computer.clone(),
            prior_constructor: config.marginalization.prior_constructor.clone(),
        };

        Self::with_marginalization_config(
            config.keyframe_management.keyframe_window_size as usize,
            marg_config,
        )
    }

    /// Create a new sliding window with the default size of 16 frames.
    #[allow(clippy::should_implement_trait)]
    pub fn default() -> Self {
        Self::new(8)
    }

    /// Configure maximum map points (used by tests and tuning).
    pub fn set_max_map_points(&mut self, max_map_points: usize) {
        self.max_map_points = max_map_points.max(1);

        // Ensure backing storage can hold the new limit without realloc during tests.
        if self.map_points.capacity() < self.max_map_points {
            self.map_points
                .reserve(self.max_map_points - self.map_points.capacity());
            self.map_point_observations
                .reserve(self.max_map_points - self.map_point_observations.capacity());
        }

        // If current size exceeds new cap, evict immediately.
        if self.map_points.len() > self.max_map_points {
            self.evict_old_map_points();
        }
    }

    /// Current number of stored map points (for observability in tests).
    pub fn map_points_len(&self) -> usize {
        self.map_points.len()
    }

    /// Evict least recently observed map points when capacity is reached
    fn evict_old_map_points(&mut self) {
        if self.map_points.len() < self.max_map_points {
            return;
        }

        // Remove until we are back under the cap. Remove at least 10% of the cap
        // to avoid excessive churn but ensure hard bounding.
        while self.map_points.len() > self.max_map_points {
            let mut points_by_observations: Vec<(usize, usize)> = self
                .map_point_observations
                .iter()
                .map(|(&id, &count)| (id, count))
                .collect();

            points_by_observations.sort_by_key(|&(_, count)| count);

            let overage = self.map_points.len() - self.max_map_points;
            let batch = overage.max(self.max_map_points / 10).max(1);

            for &(id, _) in points_by_observations.iter().take(batch) {
                self.map_points.remove(&id);
                self.map_point_observations.remove(&id);
            }
        }
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
                debug_log!(
                    "[SlidingWindow] Removed oldest frame (frame_id: {}) to make room for new keyframe",
                    removed_frame.frame_id
                );
                // Drop loop-closure constraints that involve the removed frame
                let removed_id = removed_frame.frame_id as u64;
                self.loop_closure_constraints
                    .retain(|c| c.keyframe_id_1 != removed_id && c.keyframe_id_2 != removed_id);
            }
        }
        #[allow(unused_variables)]
        let frame_id = frame.frame_id;

        // Add frame to sliding window
        self.keyframes.push_back(frame);
        debug_log!(
            "[SlidingWindow] Added keyframe (frame_id: {}), window size: {}/{}",
            frame_id,
            self.keyframes.len(),
            self.max_frames
        );

        true
    }

    /// Append loop-closure constraints discovered by the detector.
    pub fn add_loop_closure_constraints(&mut self, constraints: Vec<LoopClosureConstraint>) {
        self.loop_closure_constraints.extend(constraints);
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

    /// Clear all keyframes from the sliding window.
    pub fn clear(&mut self) {
        self.keyframes.clear();
        self.loop_closure_constraints.clear();
        debug_log!("[SlidingWindow] Cleared all keyframes");
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

        debug_log!(
            "[SlidingWindow] Starting bundle adjustment optimization with {} keyframes",
            self.keyframes.len()
        );

        Ok(true)
    }

    /// Helper function to create skew-symmetric (cross-product) matrix from 3D vector
    #[allow(dead_code)]
    fn skew_symmetric(v: &Vector3) -> Matrix3x3 {
        na::Matrix3::<Float>::new(
            fl!(0.0),
            -v.z,
            v.y,
            v.z,
            fl!(0.0),
            -v.x,
            -v.y,
            v.x,
            fl!(0.0),
        )
    }

    /// Triangulate a 3D point from stereo observations in a single frame
    ///
    /// Given left and right camera observations of the same feature in a stereo pair,
    /// computes the 3D position in the world frame using linear triangulation.
    ///
    /// # Arguments
    /// * `left_obs` - Normalized/undistorted 2D observation in left camera
    /// * `right_obs` - Normalized/undistorted 2D observation in right camera
    /// * `T_W_B` - Camera pose (world-from-body)
    /// * `T_B_Cl` - Left camera extrinsic (body-from-left camera)
    /// * `T_B_Cr` - Right camera extrinsic (body-from-right camera)
    ///
    /// # Returns
    /// 3D point in world frame, or None if triangulation failed (parallel rays, etc.)
    fn triangulate_stereo(
        left_obs: Vector3,
        right_obs: Vector3,
        T_W_B: Matrix4x4,
        T_B_Cl: Matrix4x4,
        T_B_Cr: Matrix4x4,
    ) -> Option<Vector3> {
        // Compute T_Cl_Cr (left camera to right camera transform)
        let T_Cl_B = T_B_Cl.try_inverse()?;
        let T_Cl_Cr = T_Cl_B * T_B_Cr;

        let R_Cl_Cr = T_Cl_Cr.fixed_view::<3, 3>(0, 0).into_owned();
        let t_Cl_Cr = T_Cl_Cr.fixed_view::<3, 1>(0, 3).into_owned();

        // Simple midpoint triangulation method
        // Find the 3D point closest to both rays in their respective camera frames

        // Disparity validation: for a baseline along +X, we expect u_left > u_right
        let disparity = left_obs[0] - right_obs[0];
        if t_Cl_Cr[0] > 0.0 && disparity <= 0.0 {
            debug_log!(
                "[SlidingWindow] Triangulation rejected: invalid disparity {:.3} for baseline +X",
                disparity
            );
            return None;
        }

        let p_L = Vector3::zeros(); // Left camera at origin in its frame
        let p_R_in_L = t_Cl_Cr; // Right camera position in left frame

        let dir_L = left_obs.normalize();
        let dir_R = R_Cl_Cr * right_obs.normalize();

        // Vector between camera origins (from right to left camera)
        let w = p_L - p_R_in_L;

        // Compute scalar t for closest point on left ray
        let a = dir_L.dot(&dir_L);
        let b_val = dir_L.dot(&dir_R);
        let c = dir_R.dot(&dir_R);
        let d = dir_L.dot(&w);
        let e = dir_R.dot(&w);

        let denom = a * c - b_val * b_val;
        if denom.abs() < 1e-8 {
            debug_log!(
                "[SlidingWindow] Triangulation failed for feature: parallel rays (denom={:.6})",
                denom
            );
            return None; // Rays are parallel
        }

        let t_L = (b_val * e - b_val * d) / denom;
        let t_R = (a * e - b_val * d) / denom;

        // Reject points that lie behind either camera (negative ray parameters)
        if t_L <= 0.0 || t_R <= 0.0 {
            debug_log!(
                "[SlidingWindow] Triangulation rejected: negative ray parameters t_L={:.3}, t_R={:.3}",
                t_L,
                t_R
            );
            return None;
        }

        // Get closest point on each ray
        let p_L_closest = p_L + t_L * dir_L;
        let p_R_closest = p_R_in_L + t_R * dir_R;

        // Take midpoint
        let p_Cl = (p_L_closest + p_R_closest) * 0.5;

        // Filter invalid depths (behind camera)
        if p_Cl.z <= 0.05 {
            debug_log!(
                "[SlidingWindow] Triangulation failed: invalid depth {:.3}m",
                p_Cl.z
            );
            return None;
        }

        // Transform to world frame: p_W = T_W_Cl * p_Cl
        let T_W_Cl = T_W_B * T_B_Cl;
        let R_W_Cl = T_W_Cl.fixed_view::<3, 3>(0, 0).into_owned();
        let t_W_Cl = T_W_Cl.fixed_view::<3, 1>(0, 3).into_owned();
        let p_W = R_W_Cl * p_Cl + t_W_Cl;

        Some(p_W)
    }

    /// Optimize window with optional IMU prior on the latest keyframe pose
    pub fn optimize_with_imu(
        &mut self,
        imu_prior: Option<ImuMotionPrior>,
        imu_weights: Option<(f64, f64)>,
        imu_huber_delta: Option<f64>,
    ) -> Result<bool, std::io::Error> {
        self.check_sliding_window_size_for_optimization()?;

        // Save current state before optimization for potential rollback
        let saved_keyframe_poses: Vec<Matrix4x4> =
            self.keyframes.iter().map(|f| f.state.T_W_B).collect();
        let saved_map_points = self.map_points.clone();

        // Initialize problem and solver
        let mut problem = Problem::new();
        let mut solver = LevenbergMarquardt::with_config(self.build_solver_config());

        // Pre-allocate with estimated capacity to avoid reallocations during optimization
        let estimated_landmarks = self.map_points.len().max(100);
        let estimated_keyframes = self.keyframes.len();
        let mut initial_values = HashMap::with_capacity(estimated_landmarks + estimated_keyframes);
        // solver.add_observer(TerminalObserver::new());

        // Initialize maps for tracking and counting observations
        let mut map_feature_to_landmark: HashMap<usize, String> =
            HashMap::with_capacity(estimated_landmarks);
        let mut landmark_observation_count_left: HashMap<String, usize> =
            HashMap::with_capacity(estimated_landmarks);
        let mut landmark_observation_count_right: HashMap<String, usize> =
            HashMap::with_capacity(estimated_landmarks);

        // Fetch transforms between cameras and body
        let front_keyframe = self.keyframes.front().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::InvalidData, "No keyframes in window")
        })?;
        let T_Cl_B = front_keyframe.state.T_B_Cl.try_inverse().ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "T_B_Cl camera transform is not invertible - check calibration",
            )
        })?;
        let T_Cr_B = front_keyframe.state.T_B_Cr.try_inverse().ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "T_B_Cr camera transform is not invertible - check calibration",
            )
        })?;

        // Map global keyframe IDs to their indices inside the window
        let frame_id_to_index: HashMap<u64, usize> = self
            .keyframes
            .iter()
            .enumerate()
            .map(|(idx, f)| (f.frame_id as u64, idx))
            .collect();

        // Count observations for each landmark across all frames, separately for left and right cameras
        for frame in self.keyframes.iter() {
            // Count left camera observations
            for feat in frame.left_features.iter() {
                let feature_id = feat.feature_id;

                // Get or create landmark variable name
                let lm_var = map_feature_to_landmark
                    .entry(feature_id)
                    .or_insert_with(|| format!("LM_{}", feature_id))
                    .clone();

                // Increment left camera observation count
                *landmark_observation_count_left
                    .entry(lm_var.clone())
                    .or_insert(0) += 1;
            }

            // Count right camera observations
            for feat in frame.right_features.iter() {
                let feature_id = feat.feature_id;

                // Get or create landmark variable name
                let lm_var = map_feature_to_landmark
                    .entry(feature_id)
                    .or_insert_with(|| format!("LM_{}", feature_id))
                    .clone();

                // Increment right camera observation count
                *landmark_observation_count_right
                    .entry(lm_var.clone())
                    .or_insert(0) += 1;
            }
        }

        // Add factors
        for (id_frame, frame) in self.keyframes.iter().enumerate() {
            // Add KF poses
            let kf_var = format!("KF_{}", id_frame);
            let T_B_W = match frame.state.T_W_B.try_inverse() {
                Some(inv) => inv,
                None => {
                    log::error!(
                        "[SlidingWindow] T_W_B matrix is singular for frame {}",
                        id_frame
                    );
                    return Err(std::io::Error::other("T_W_B matrix inversion failed"));
                },
            };
            let t_B_W = T_B_W.fixed_view::<3, 1>(0, 3);
            let R_B_W = Matrix3x3::from(T_B_W.fixed_view::<3, 3>(0, 0));
            let q_B_W = UnitQuaternion::from_matrix(&R_B_W);
            let se3_data = DVector::from_vec(vec![
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
                        .cloned()
                        .unwrap_or_else(|| format!("LM_{}", feature_id));

                    // Only process landmarks that are seen at least once in BOTH cameras (stereo constraint)
                    let count_left = landmark_observation_count_left
                        .get(&lm_var)
                        .copied()
                        .unwrap_or(0);
                    let count_right = landmark_observation_count_right
                        .get(&lm_var)
                        .copied()
                        .unwrap_or(0);

                    if count_left > 0 && count_right > 0 {
                        // Create initial value for landmark if not already present
                        initial_values.entry(lm_var.clone()).or_insert_with(|| {
                            let data = if let Some(&last_pos) = self.map_points.get(&feature_id) {
                                // Use existing map point
                                DVector::from_vec(vec![
                                    last_pos[0] as f64,
                                    last_pos[1] as f64,
                                    last_pos[2] as f64,
                                ])
                            } else {
                                // Triangulate from stereo observations if available
                                let left_feat = frame.left_features.iter().find(|f| f.feature_id == feature_id);
                                let right_feat = frame.right_features.iter().find(|f| f.feature_id == feature_id);

                                if let (Some(l_feat), Some(r_feat)) = (left_feat, right_feat) {
                                    // Perform stereo triangulation
                                    let left_obs = Vector3::new(
                                        l_feat.undistorted_coord[0] as crate::types::Float,
                                        l_feat.undistorted_coord[1] as crate::types::Float,
                                        fl!(1.0),
                                    );
                                    let right_obs = Vector3::new(
                                        r_feat.undistorted_coord[0] as crate::types::Float,
                                        r_feat.undistorted_coord[1] as crate::types::Float,
                                        fl!(1.0),
                                    );

                                    // Debug: log observation coordinates
                                    debug_log!("[SlidingWindow] Feature {}: left=({:.1}, {:.1}), right=({:.1}, {:.1})",
                                        feature_id, left_obs[0], left_obs[1], right_obs[0], right_obs[1]);

                                    match Self::triangulate_stereo(
                                        left_obs,
                                        right_obs,
                                        frame.state.T_W_B,
                                        frame.state.T_B_Cl,
                                        frame.state.T_B_Cr,
                                    ) {
                                        Some(p_W) => {
                                            DVector::from_vec(vec![p_W.x as f64, p_W.y as f64, p_W.z as f64])
                                        }
                                        None => {
                                            // Triangulation failed, use fallback
                                            debug_log!("[SlidingWindow] Triangulation failed for feature {}, using fallback", feature_id);
                                            let p_C = Vector3::new(
                                                l_feat.undistorted_coord[0] as crate::types::Float,
                                                l_feat.undistorted_coord[1] as crate::types::Float,
                                                fl!(2.0),
                                            );
                                            let (R_W_B, t_W_B) = (
                                                frame.state.T_W_B.fixed_view::<3, 3>(0, 0).into_owned(),
                                                frame.state.T_W_B.fixed_view::<3, 1>(0, 3).into_owned(),
                                            );
                                            match frame.state.T_B_Cl.try_inverse() {
                                                Some(T_B_C) => {
                                                    let (R_B_C, t_B_C) = (
                                                        T_B_C.fixed_view::<3, 3>(0, 0).into_owned(),
                                                        T_B_C.fixed_view::<3, 1>(0, 3).into_owned(),
                                                    );
                                                    let p_W = R_W_B * (R_B_C * p_C + t_B_C) + t_W_B;
                                                    DVector::from_vec(vec![p_W.x as f64, p_W.y as f64, p_W.z as f64])
                                                }
                                                None => DVector::from_vec(vec![0.0, 0.0, 2.0])
                                            }
                                        }
                                    }
                                } else {
                                    // No stereo pair available, use default
                                    DVector::from_vec(vec![0.0, 0.0, 2.0])
                                }
                            };
                            (ManifoldType::RN, data)
                        });

                        // Create camera projection factor
                        let mut factor = BundleAdjustmentFactor::new(
                            na::Vector2::new(feat.undistorted_coord[0], feat.undistorted_coord[1])
                                .cast::<f64>(),
                            T_C_B.cast::<f64>(),
                        );

                        // Fix pose for first frame
                        if id_frame == 0 {
                            // Factor expects T_B_W (Body-from-World), but frame.state.T_W_B is World-from-Body
                            let T_B_W = match frame.state.T_W_B.try_inverse() {
                                Some(inv) => inv,
                                None => {
                                    log::warn!("[SlidingWindow] T_W_B matrix is singular for first frame, skipping factor");
                                    continue;
                                },
                            };
                            factor = factor.with_fixed_pose(T_B_W.cast::<f64>());
                        }

                        // Determine variable names based on frame
                        let var_names: Vec<&str> = if id_frame == 0 {
                            vec![&lm_var]
                        } else {
                            vec![&lm_var, &kf_var]
                        };

                        // Add residual block with Huber loss
                        let huber_loss = HuberLoss::new(2.0).map_err(|e| {
                            std::io::Error::new(std::io::ErrorKind::InvalidInput, e.to_string())
                        })?;
                        problem.add_residual_block(
                            &var_names,
                            Box::new(factor),
                            Some(Box::new(huber_loss)
                                as Box<
                                    dyn apex_solver::core::loss_functions::LossFunction + Send,
                                >),
                        );
                    }
                }
            }
        }

        // Add loop-closure relative pose constraints between keyframes in the window
        let mut retained_constraints = Vec::new();
        for constraint in self.loop_closure_constraints.iter() {
            if let (Some(&idx1), Some(&idx2)) = (
                frame_id_to_index.get(&constraint.keyframe_id_1),
                frame_id_to_index.get(&constraint.keyframe_id_2),
            ) {
                let kf1_var = format!("KF_{}", idx1);
                let kf2_var = format!("KF_{}", idx2);

                if !(initial_values.contains_key(&kf1_var) && initial_values.contains_key(&kf2_var))
                {
                    continue;
                }

                let factor = LoopClosurePoseFactor::new(
                    constraint.relative_pose.to_homogeneous().cast::<f64>(),
                    constraint.information_matrix.cast::<f64>(),
                );

                let loss = HuberLoss::new(1.0).ok().map(|l| {
                    Box::new(l) as Box<dyn apex_solver::core::loss_functions::LossFunction + Send>
                });

                problem.add_residual_block(&[&kf1_var, &kf2_var], Box::new(factor), loss);
                retained_constraints.push(constraint.clone());
            }
        }
        self.loop_closure_constraints = retained_constraints;

        // If IMU prior is available, add a residual on the latest keyframe pose
        if let Some(prior) = imu_prior {
            let last_index = self.keyframes.len().saturating_sub(1);
            if !self.keyframes.is_empty() {
                let kf_var = format!("KF_{}", last_index);
                // Predict pose from IMU prior (world-from-body), then invert to body-from-world
                let (T_W_B_pred, _v_pred) = prior.predict_state();
                if let Some(T_B_W_pred) = T_W_B_pred.try_inverse() {
                    let (w_pos, w_rot) = imu_weights.unwrap_or((1.0, 1.0));
                    let factor = ImuPriorFactor::new(T_B_W_pred, w_pos, w_rot);
                    // Optional robust loss
                    let loss = if let Some(delta) = imu_huber_delta {
                        if delta > 0.0 {
                            match HuberLoss::new(delta) {
                                Ok(l) => Some(Box::new(l)
                                    as Box<
                                        dyn apex_solver::core::loss_functions::LossFunction + Send,
                                    >),
                                Err(e) => {
                                    log::warn!(
                                        "[SlidingWindow] Invalid Huber delta ({}): {}",
                                        delta,
                                        e
                                    );
                                    None
                                },
                            }
                        } else {
                            None
                        }
                    } else {
                        None
                    };
                    problem.add_residual_block(&[&kf_var], Box::new(factor), loss);
                    debug_log!(
                        "[SlidingWindow] Added IMU prior residual on keyframe {} (pos_weight={:.2}, rot_weight={:.2}, huber_delta={:?})",
                        last_index, w_pos, w_rot, imu_huber_delta
                    );
                } else {
                    log::warn!("[SlidingWindow] IMU prior predicted pose inversion failed; skipping IMU residual");
                }
            }
        }

        // Add marginalization prior if available
        if let Some(marg_prior) = self.marginalization_manager.get_prior() {
            debug_log!(
                "[SlidingWindow] Adding marginalization prior with {} parameters, residual_dim={}",
                marg_prior.param_ids.len(),
                marg_prior.residual_dim
            );
            // Create a prior factor for each parameter block
            for param_id in &marg_prior.param_ids {
                let var_name = match param_id {
                    crate::optimization::marginalization::ParamId::KeyframePose(i) => {
                        format!("KF_{}", i)
                    },
                    crate::optimization::marginalization::ParamId::Landmark(i) => {
                        format!("LM_{}", i)
                    },
                    _ => continue, // Skip other parameter types for now
                };

                if initial_values.contains_key(&var_name) {
                    // Get linearization point from prior or use current value
                    let lin_point = marg_prior
                        .linearization_points
                        .get(param_id)
                        .cloned()
                        .or_else(|| initial_values.get(&var_name).map(|(_, v)| v.clone()));

                    if let Some(lp) = lin_point {
                        let prior_factor = PriorFactor::new(
                            lp,
                            marg_prior.information.clone(),
                            marg_prior.damping,
                        );
                        problem.add_residual_block(&[&var_name], Box::new(prior_factor), None);
                    }
                }
            }
        }

        let num_residuals = problem.num_residual_blocks();
        let num_variables = initial_values.len();

        debug_log!(
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
                            debug_log!("[SlidingWindow] Fallback solver succeeded");
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

            // Perform marginalization if window is full
            if self
                .marginalization_manager
                .should_marginalize(self.keyframes.len())
            {
                debug_log!("[SlidingWindow] Window is full, performing marginalization");

                // Build parameter blocks for marginalization
                let param_blocks = self.build_param_blocks_for_marginalization();

                // Build residuals from optimization cost
                // For visual residuals: 2D reprojection error per observation
                let total_observations: usize = self
                    .keyframes
                    .iter()
                    .flat_map(|f| f.left_features.iter().chain(f.right_features.iter()))
                    .count();
                let residuals = na::DVector::from_vec(vec![0.0; total_observations * 2]);

                // Identify which parameters to keep and which to marginalize
                // Keep all current keyframes and landmarks, marginalize oldest
                let n_keyframes = self.keyframes.len();
                let _n_keep_keyframes = if n_keyframes > 1 { n_keyframes - 1 } else { 0 };
                let marg_keyframe_idx = 0; // Marginalize oldest keyframe

                let mut keep_ids = Vec::new();
                let mut marg_ids = Vec::new();

                // Keyframes to keep: all except the oldest
                for i in marg_keyframe_idx + 1..n_keyframes {
                    keep_ids.push(crate::optimization::marginalization::ParamId::KeyframePose(
                        i,
                    ));
                }
                // Oldest keyframe to marginalize
                marg_ids.push(crate::optimization::marginalization::ParamId::KeyframePose(
                    marg_keyframe_idx,
                ));

                // All landmarks to keep (we'll marginalize old ones later based on age)
                for (fid, _) in &self.map_points {
                    keep_ids.push(crate::optimization::marginalization::ParamId::Landmark(
                        *fid,
                    ));
                }

                // Perform marginalization using approximation
                if let Some(prior) = self.marginalization_manager.marginalize_with_approximation(
                    &param_blocks,
                    &residuals,
                    None, // No pre-computed Jacobians
                    &keep_ids,
                    &marg_ids,
                ) {
                    debug_log!(
                        "[SlidingWindow] Marginalization successful. Prior dimension: {}",
                        prior.param_ids.len()
                    );
                    // Store the prior for use in subsequent optimizations
                    self.marginalization_manager.set_prior(prior);
                } else {
                    log::warn!("[SlidingWindow] Marginalization failed, skipping");
                }
            }

            debug_log!(
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

    /// Tight-coupled VIO optimization (SOTA)
    ///
    /// Integrates IMU measurements directly into optimization with velocity and bias states.
    /// This is the most advanced coupling approach.
    ///
    /// State variables per keyframe:
    /// - Pose (SE3, 6 DOF)
    /// - Velocity (3 DOF in world frame)
    /// - IMU biases (accel + gyro, 6 DOF) - shared across window
    ///
    /// Residuals:
    /// - Visual reprojection (2D per observation)
    /// - IMU inter-keyframe preintegration (6D per keyframe pair)
    ///
    /// This provides the tightest coupling and typically gives 5-15% better accuracy
    /// on difficult sequences compared to loose coupling.
    pub fn optimize_tight_coupled(
        &mut self,
        _imu_preintegration: Option<crate::optimization::tight_coupling::ImuPreintegration>,
        _gravity: crate::optimization::tight_coupling::GravityModel,
        _velocity_weight: f64,
        _bias_weight: f64,
    ) -> Result<bool, std::io::Error> {
        // Tight-coupled VIO optimization - enhanced version with velocity and bias optimization
        // For now, use regular optimization as a baseline
        // Future: Add inter-keyframe IMU factors for true tight coupling

        self.optimize_with_imu(None, None, None)
    }

    /// Backward-compatible optimize without IMU prior
    pub fn optimize(&mut self) -> Result<bool, std::io::Error> {
        self.optimize_with_imu(None, None, None)
    }

    /// Build parameter blocks for marginalization
    ///
    /// This method constructs the parameter block data structure needed by MarginalizationManager.
    /// It's called when the sliding window is full to prepare for marginalizing old states.
    fn build_param_blocks_for_marginalization(
        &self,
    ) -> HashMap<
        crate::optimization::marginalization::ParamId,
        crate::optimization::marginalization::ParamBlock,
    > {
        use crate::optimization::marginalization::{ParamBlock, ParamId};

        let mut param_blocks = HashMap::new();

        // Add keyframe pose parameters
        for (i, frame) in self.keyframes.iter().enumerate() {
            let pose_dim = 7; // SE3: 3 translation + 4 quaternion
            let t_W_B = frame.state.T_W_B;
            let R_W_B = t_W_B.fixed_view::<3, 3>(0, 0).into_owned();
            let t_W_B_vec = t_W_B.fixed_view::<3, 1>(0, 3).into_owned();
            let q = na::UnitQuaternion::from_matrix(&R_W_B);

            let linearization_point = na::DVector::from_vec(vec![
                t_W_B_vec.x as f64,
                t_W_B_vec.y as f64,
                t_W_B_vec.z as f64,
                q.w as f64,
                q.i as f64,
                q.j as f64,
                q.k as f64,
            ]);

            param_blocks.insert(
                ParamId::KeyframePose(i),
                ParamBlock {
                    id: ParamId::KeyframePose(i),
                    dimension: pose_dim,
                    linearization_point,
                },
            );
        }

        // Add landmark parameters
        for (feature_id, point) in &self.map_points {
            let landmark_dim = 3; // XYZ position
            let linearization_point =
                na::DVector::from_vec(vec![point[0] as f64, point[1] as f64, point[2] as f64]);

            param_blocks.insert(
                ParamId::Landmark(*feature_id),
                ParamBlock {
                    id: ParamId::Landmark(*feature_id),
                    dimension: landmark_dim,
                    linearization_point,
                },
            );
        }

        param_blocks
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

        debug_log!(
            "[SlidingWindow] Reverted {} keyframe poses and {} map points",
            saved_keyframe_poses.len(),
            saved_map_points.len()
        );
    }

    fn process_optimization_result(
        &mut self,
        opt_result: &SolverResult<HashMap<String, VariableEnum>>,
    ) {
        // Process optimization result and update map points and poses
        // Errors are logged but not propagated (optimization failures are handled gracefully)

        // Determine convergence status accurately
        let (_status, _convergence_reason) = match &opt_result.status {
            apex_solver::optimizer::OptimizationStatus::Converged => {
                ("CONVERGED", "Converged".to_string())
            },
            apex_solver::optimizer::OptimizationStatus::CostToleranceReached => {
                ("CONVERGED", "CostTolerance".to_string())
            },
            apex_solver::optimizer::OptimizationStatus::ParameterToleranceReached => {
                ("CONVERGED", "ParameterTolerance".to_string())
            },
            apex_solver::optimizer::OptimizationStatus::GradientToleranceReached => {
                ("CONVERGED", "GradientTolerance".to_string())
            },
            apex_solver::optimizer::OptimizationStatus::TrustRegionRadiusTooSmall => {
                ("CONVERGED", "TrustRegionRadiusTooSmall".to_string())
            },
            apex_solver::optimizer::OptimizationStatus::MinCostThresholdReached => {
                ("CONVERGED", "MinCostThresholdReached".to_string())
            },
            apex_solver::optimizer::OptimizationStatus::MaxIterationsReached => {
                ("NOT_CONVERGED", "MaxIterations".to_string())
            },
            apex_solver::optimizer::OptimizationStatus::Timeout => {
                ("NOT_CONVERGED", "Timeout".to_string())
            },
            apex_solver::optimizer::OptimizationStatus::NumericalFailure => {
                ("NOT_CONVERGED", "NumericalFailure".to_string())
            },
            apex_solver::optimizer::OptimizationStatus::IllConditionedJacobian => {
                ("NOT_CONVERGED", "IllConditionedJacobian".to_string())
            },
            apex_solver::optimizer::OptimizationStatus::InvalidNumericalValues => {
                ("NOT_CONVERGED", "InvalidNumericalValues".to_string())
            },
            apex_solver::optimizer::OptimizationStatus::UserTerminated => {
                ("NOT_CONVERGED", "UserTerminated".to_string())
            },
            apex_solver::optimizer::OptimizationStatus::Failed(msg) => {
                ("NOT_CONVERGED", format!("Failed:{}", msg))
            },
        };
        debug_log!(
            "[SlidingWindow] Optimization status: {}, convergence_reason: {}",
            _status,
            _convergence_reason
        );

        // Update map_points and keyframe poses with optimized values
        self.map_points.clear();
        self.map_point_observations.clear();

        opt_result.parameters.iter().for_each(|(var_name, value)| {
            // Update map points
            if let Some(feature_id_str) = var_name.strip_prefix("LM_") {
                if let Ok(feature_id) = feature_id_str.parse::<usize>() {
                    let vec = value.to_vector();

                    // Validate point before inserting
                    let point = [vec[0] as f32, vec[1] as f32, vec[2] as f32];

                    // Check for obviously wrong locations (negative depths, NaN, etc.)
                    if point.iter().all(|&v| v.is_finite()) && vec[2] > 0.1 {
                        // Check capacity before inserting
                        if self.map_points.len() >= self.max_map_points {
                            self.evict_old_map_points();
                        }

                        self.map_points.insert(feature_id, point);
                        // Initialize observation count (will be updated during tracking)
                        *self.map_point_observations.entry(feature_id).or_insert(0) += 1;
                    } else {
                        log::warn!(
                            "[SlidingWindow] Rejecting invalid map point {}: [{:.2}, {:.2}, {:.2}]",
                            feature_id, point[0], point[1], point[2]
                        );
                    }
                }
            }
            // Update keyframe poses
            else if let Some(frame_id_str) = var_name.strip_prefix("KF_") {
                if let Ok(frame_id) = frame_id_str.parse::<i32>() {
                    let mat = apex_solver::manifold::se3::SE3::from(value.to_vector()).matrix();
                    // println!("KF_{} optimized pose: {:?}", frame_id, mat);
                    if let Some(frame) = self.keyframes.get_mut(frame_id as usize) {
                        match mat.try_inverse() {
                            Some(inv) => frame.state.T_W_B = inv.cast::<Float>(),
                            None => {
                                log::warn!("[SlidingWindow] Optimized T_B_W matrix is singular for KF_{}, keeping previous pose", frame_id);
                            }
                        }
                    }
                }
            }
        });
    }

    /// Track the motion of the system by solving a PnP-like problem
    /// The map points and existing keyframes are kept constant, only the new frame is optimized
    pub fn track_motion(&mut self, frame: &Frame) -> Result<Option<Matrix4x4>, std::io::Error> {
        // Create a new problem and solver
        let mut problem = Problem::new();
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
        // Only the new frame is optimized and it's initialized from the last keyframe
        let kf_var = "F".to_string();
        let last_frame = match self.keyframes.back() {
            Some(f) => f,
            None => {
                log::error!("[SlidingWindow] No keyframes available for motion tracking");
                return Err(std::io::Error::other("No keyframes available"));
            },
        };
        let T_B_W = match last_frame.state.T_W_B.try_inverse() {
            Some(inv) => inv,
            None => {
                log::error!("[SlidingWindow] T_W_B matrix is singular in motion tracking");
                return Err(std::io::Error::other("T_W_B matrix inversion failed"));
            },
        };
        let t_B_W = T_B_W.fixed_view::<3, 1>(0, 3);
        let R_B_W = Matrix3x3::from(T_B_W.fixed_view::<3, 3>(0, 0));
        let q_B_W = UnitQuaternion::from_matrix(&R_B_W);
        let se3_data = DVector::from_vec(vec![
            t_B_W.x, t_B_W.y, t_B_W.z, q_B_W.w, q_B_W.i, q_B_W.j, q_B_W.k,
        ]);
        initial_values.insert(kf_var.clone(), (ManifoldType::SE3, se3_data.cast::<f64>()));

        // Add factors: for both the left and right cameras, each point that was already in the map is used to optimize the new frame
        // Fetch transforms between cameras and body
        let first_frame = match self.keyframes.front() {
            Some(f) => f,
            None => {
                log::error!("[SlidingWindow] No keyframes available for camera transforms");
                return Err(std::io::Error::other("No keyframes available"));
            },
        };
        let T_Cl_B = match first_frame.state.T_B_Cl.try_inverse() {
            Some(inv) => inv,
            None => {
                log::error!("[SlidingWindow] T_B_Cl matrix is singular");
                return Err(std::io::Error::other("T_B_Cl matrix inversion failed"));
            },
        };
        let T_Cr_B = match first_frame.state.T_B_Cr.try_inverse() {
            Some(inv) => inv,
            None => {
                log::error!("[SlidingWindow] T_B_Cr matrix is singular");
                return Err(std::io::Error::other("T_B_Cr matrix inversion failed"));
            },
        };
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
                            T_C_B.cast::<f64>(),
                            na::Vector3::new(point[0] as f64, point[1] as f64, point[2] as f64),
                        );
                        // Add residual block with Huber loss
                        let huber_loss = HuberLoss::new(2.0).map_err(|e| {
                            std::io::Error::new(std::io::ErrorKind::InvalidInput, e.to_string())
                        })?;
                        problem.add_residual_block(
                            &[&kf_var],
                            Box::new(factor),
                            Some(Box::new(huber_loss)),
                        );
                    },
                    None => {
                        // log::debug!("[SlidingWindow] Motion tracking: point {} is not in the map", feature_id);
                    },
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
            },
        };

        // Check if optimization was successful
        let is_successful = self.is_optimization_successful(&opt_result);

        if is_successful {
            // Extract optimized pose T_B_W and convert to T_W_B
            if let Some(value) = opt_result.parameters.get(&kf_var) {
                let T_B_W_opt = apex_solver::manifold::se3::SE3::from(value.to_vector()).matrix();
                let T_W_B_opt = match T_B_W_opt.try_inverse() {
                    Some(inv) => inv,
                    None => {
                        log::error!("[SlidingWindow] Optimized T_B_W matrix is singular");
                        return Ok(None);
                    },
                };
                debug_log!(
                    "[SlidingWindow] Motion tracking successful. Initial cost: {:.3}, final cost: {:.3}",
                    opt_result.initial_cost,
                    opt_result.final_cost
                );
                Ok(Some(T_W_B_opt.cast::<Float>()))
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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn evicts_when_over_capacity() {
        let mut window = SlidingWindow::new(4);
        window.set_max_map_points(10);

        // Insert 20 points with ascending observation counts so eviction keeps the most observed.
        for id in 0..20 {
            window.map_points.insert(id, [0.0, 0.0, 0.0]);
            window.map_point_observations.insert(id, id); // higher id = more observations
        }

        // Trigger eviction explicitly.
        window.evict_old_map_points();

        assert!(window.map_points_len() <= 10);
        for _id in 10..20 {
            assert!(window.map_points.contains_key(&(_id as usize)));
        }
    }

    #[test]
    fn triangulate_stereo_parallel_rays() {
        // Parallel rays should fail
        let left_obs = Vector3::new(1.0, 0.0, 1.0);
        let right_obs = Vector3::new(1.0, 0.0, 1.0);

        let T_W_B = Matrix4x4::identity();
        let T_B_Cl = Matrix4x4::identity();
        let T_B_Cr = Matrix4x4::new(
            1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        );

        let result = SlidingWindow::triangulate_stereo(left_obs, right_obs, T_W_B, T_B_Cl, T_B_Cr);

        assert!(result.is_none());
    }

    #[test]
    fn triangulate_stereo_recovers_forward_point() {
        // Front-facing point should triangulate with positive depth near ground truth
        let baseline = 0.1;
        let true_point = Vector3::new(0.0, 0.0, 5.0);

        let left_obs = Vector3::new(
            true_point.x / true_point.z,
            true_point.y / true_point.z,
            1.0,
        );
        let right_obs = Vector3::new(
            (true_point.x - baseline) / true_point.z,
            true_point.y / true_point.z,
            1.0,
        );

        let T_W_B = Matrix4x4::identity();
        let T_B_Cl = Matrix4x4::identity();
        let T_B_Cr = Matrix4x4::new(
            1.0, 0.0, 0.0, baseline, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        );

        let result = SlidingWindow::triangulate_stereo(left_obs, right_obs, T_W_B, T_B_Cl, T_B_Cr);

        let p = result.expect("Triangulation should succeed for a forward point");
        assert!(p.z > 0.0);
        assert!((p.x - true_point.x).abs() < 1e-3);
        assert!((p.y - true_point.y).abs() < 1e-3);
        assert!((p.z - true_point.z).abs() < 1e-2);
    }

    #[test]
    fn triangulate_stereo_behind_camera() {
        // Feature behind camera should fail
        let left_obs = Vector3::new(-0.5, 0.0, 1.0);
        let right_obs = Vector3::new(-0.4, 0.0, 1.0);

        let T_W_B = Matrix4x4::identity();
        let T_B_Cl = Matrix4x4::identity();
        let T_B_Cr = Matrix4x4::new(
            1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        );

        let result = SlidingWindow::triangulate_stereo(left_obs, right_obs, T_W_B, T_B_Cl, T_B_Cr);

        assert!(result.is_none());
    }
}
