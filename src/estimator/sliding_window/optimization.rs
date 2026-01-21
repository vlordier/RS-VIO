use super::SlidingWindow;
use crate::debug_log;
use crate::estimator::Frame;
use crate::imu::ImuMotionPrior;
use crate::optimization::factors::{
    BundleAdjustmentFactor, ImuPriorFactor, JointPriorFactor, LoopClosurePoseFactor, PnPFactor,
};
use crate::optimization::marginalization::{ParamBlock, ParamId};
use crate::optimization::tight_coupling::{
    GravityModel, ImuPreintegration, InterKeyframeImuFactor,
};
use crate::vision::motion_aware_depth_optimization::{
    MotionAwareDepthOptimizer, TriangulationConstraints,
};
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
use na::{DMatrix, DVector, UnitQuaternion};
use nalgebra as na;
use std::collections::HashMap;

struct OptimizationWorkspace {
    problem: Problem,
    initial_values: HashMap<String, (ManifoldType, DVector<f64>)>,
    map_feature_to_landmark: HashMap<usize, String>,
    landmark_observation_count_left: HashMap<String, usize>,
    landmark_observation_count_right: HashMap<String, usize>,
    frame_id_to_index: HashMap<u64, usize>,
    T_Cl_B: Matrix4x4,
    T_Cr_B: Matrix4x4,
}

impl OptimizationWorkspace {
    fn new(
        estimated_landmarks: usize,
        estimated_keyframes: usize,
        frame_id_to_index: HashMap<u64, usize>,
        T_Cl_B: Matrix4x4,
        T_Cr_B: Matrix4x4,
    ) -> Self {
        let initial_values =
            HashMap::with_capacity(estimated_landmarks.saturating_add(estimated_keyframes));

        Self {
            problem: Problem::new(),
            initial_values,
            map_feature_to_landmark: HashMap::with_capacity(estimated_landmarks),
            landmark_observation_count_left: HashMap::with_capacity(estimated_landmarks),
            landmark_observation_count_right: HashMap::with_capacity(estimated_landmarks),
            frame_id_to_index,
            T_Cl_B,
            T_Cr_B,
        }
    }
}

impl SlidingWindow {
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

    fn build_optimization_workspace(
        &self,
        estimated_landmarks: usize,
        estimated_keyframes: usize,
    ) -> Result<OptimizationWorkspace, std::io::Error> {
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

        let frame_id_to_index: HashMap<u64, usize> = self
            .keyframes
            .iter()
            .enumerate()
            .map(|(idx, f)| (f.frame_id as u64, idx))
            .collect();

        Ok(OptimizationWorkspace::new(
            estimated_landmarks,
            estimated_keyframes,
            frame_id_to_index,
            T_Cl_B,
            T_Cr_B,
        ))
    }

    fn count_landmark_observations(&self, workspace: &mut OptimizationWorkspace) {
        for frame in self.keyframes.iter() {
            for feat in frame.left_features.iter() {
                let feature_id = feat.feature_id;
                let lm_var = workspace
                    .map_feature_to_landmark
                    .entry(feature_id)
                    .or_insert_with(|| format!("LM_{}", feature_id))
                    .clone();

                *workspace
                    .landmark_observation_count_left
                    .entry(lm_var.clone())
                    .or_insert(0) += 1;
            }

            for feat in frame.right_features.iter() {
                let feature_id = feat.feature_id;
                let lm_var = workspace
                    .map_feature_to_landmark
                    .entry(feature_id)
                    .or_insert_with(|| format!("LM_{}", feature_id))
                    .clone();

                *workspace
                    .landmark_observation_count_right
                    .entry(lm_var.clone())
                    .or_insert(0) += 1;
            }
        }
    }

    fn add_keyframe_states_and_visual_factors(
        &self,
        workspace: &mut OptimizationWorkspace,
    ) -> Result<(), std::io::Error> {
        for (id_frame, frame) in self.keyframes.iter().enumerate() {
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

            workspace
                .initial_values
                .insert(kf_var.clone(), (ManifoldType::SE3, se3_data.cast::<f64>()));

            let vel_var = format!("VEL_{}", id_frame);
            let vel_data = DVector::from_vec(vec![
                frame.state.velocity.x as f64,
                frame.state.velocity.y as f64,
                frame.state.velocity.z as f64,
            ]);
            workspace
                .initial_values
                .insert(vel_var.clone(), (ManifoldType::RN, vel_data));

            let camera_features = [
                (&frame.left_features, &workspace.T_Cl_B),
                (&frame.right_features, &workspace.T_Cr_B),
            ];

            for (features, T_C_B) in camera_features.iter() {
                for feat in features.iter() {
                    let feature_id = feat.feature_id;
                    let lm_var = workspace
                        .map_feature_to_landmark
                        .get(&feature_id)
                        .cloned()
                        .unwrap_or_else(|| format!("LM_{}", feature_id));

                    let count_left = workspace
                        .landmark_observation_count_left
                        .get(&lm_var)
                        .copied()
                        .unwrap_or(0);
                    let count_right = workspace
                        .landmark_observation_count_right
                        .get(&lm_var)
                        .copied()
                        .unwrap_or(0);

                    if count_left == 0 || count_right == 0 {
                        continue;
                    }

                    workspace
                        .initial_values
                        .entry(lm_var.clone())
                        .or_insert_with(|| {
                            let data = if let Some(&last_pos) = self.map_points.get(&feature_id) {
                                DVector::from_vec(vec![
                                    last_pos[0] as f64,
                                    last_pos[1] as f64,
                                    last_pos[2] as f64,
                                ])
                            } else {
                                let left_feat =
                                    frame.left_features.iter().find(|f| f.feature_id == feature_id);
                                let right_feat = frame
                                    .right_features
                                    .iter()
                                    .find(|f| f.feature_id == feature_id);

                                if let (Some(l_feat), Some(r_feat)) = (left_feat, right_feat) {
                                    let left_obs = Vector3::new(
                                        l_feat.undistorted_coord[0] as Float,
                                        l_feat.undistorted_coord[1] as Float,
                                        fl!(1.0),
                                    );
                                    let right_obs = Vector3::new(
                                        r_feat.undistorted_coord[0] as Float,
                                        r_feat.undistorted_coord[1] as Float,
                                        fl!(1.0),
                                    );

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
                                            debug_log!("[SlidingWindow] Triangulation failed for feature {}, using fallback", feature_id);
                                            let p_C = Vector3::new(
                                                l_feat.undistorted_coord[0] as Float,
                                                l_feat.undistorted_coord[1] as Float,
                                                fl!(2.0),
                                            );
                                            let (R_W_B, t_W_B) = (
                                                frame
                                                    .state
                                                    .T_W_B
                                                    .fixed_view::<3, 3>(0, 0)
                                                    .into_owned(),
                                                frame
                                                    .state
                                                    .T_W_B
                                                    .fixed_view::<3, 1>(0, 3)
                                                    .into_owned(),
                                            );
                                            match frame.state.T_B_Cl.try_inverse() {
                                                Some(T_B_C) => {
                                                    let (R_B_C, t_B_C) = (
                                                        T_B_C.fixed_view::<3, 3>(0, 0).into_owned(),
                                                        T_B_C.fixed_view::<3, 1>(0, 3).into_owned(),
                                                    );
                                                    let p_W = R_W_B * (R_B_C * p_C + t_B_C) + t_W_B;
                                                    DVector::from_vec(vec![
                                                        p_W.x as f64,
                                                        p_W.y as f64,
                                                        p_W.z as f64,
                                                    ])
                                                }
                                                None => DVector::from_vec(vec![0.0, 0.0, 2.0]),
                                            }
                                        }
                                    }
                                } else {
                                    DVector::from_vec(vec![0.0, 0.0, 2.0])
                                }
                            };
                            (ManifoldType::RN, data)
                        });

                    let mut factor = BundleAdjustmentFactor::new(
                        na::Vector2::new(feat.undistorted_coord[0], feat.undistorted_coord[1])
                            .cast::<f64>(),
                        (*T_C_B).cast::<f64>(),
                    );

                    if id_frame == 0 {
                        let T_B_W = match frame.state.T_W_B.try_inverse() {
                            Some(inv) => inv,
                            None => {
                                log::warn!(
                                    "[SlidingWindow] T_W_B matrix is singular for first frame, skipping factor"
                                );
                                continue;
                            },
                        };
                        factor = factor.with_fixed_pose(T_B_W.cast::<f64>());
                    }

                    let var_names: Vec<&str> = if id_frame == 0 {
                        vec![&lm_var]
                    } else {
                        vec![&lm_var, &kf_var]
                    };

                    let huber_loss = HuberLoss::new(2.0).map_err(|e| {
                        std::io::Error::new(std::io::ErrorKind::InvalidInput, e.to_string())
                    })?;
                    workspace.problem.add_residual_block(
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

        Ok(())
    }

    fn add_loop_closure_factors(&mut self, workspace: &mut OptimizationWorkspace) {
        let mut retained_constraints = Vec::new();
        for constraint in self.loop_closure_constraints.iter() {
            if let (Some(&idx1), Some(&idx2)) = (
                workspace.frame_id_to_index.get(&constraint.keyframe_id_1),
                workspace.frame_id_to_index.get(&constraint.keyframe_id_2),
            ) {
                let kf1_var = format!("KF_{}", idx1);
                let kf2_var = format!("KF_{}", idx2);

                if !(workspace.initial_values.contains_key(&kf1_var)
                    && workspace.initial_values.contains_key(&kf2_var))
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

                workspace
                    .problem
                    .add_residual_block(&[&kf1_var, &kf2_var], Box::new(factor), loss);
                retained_constraints.push(constraint.clone());
            }
        }
        self.loop_closure_constraints = retained_constraints;
    }

    fn add_imu_factors(&self, workspace: &mut OptimizationWorkspace) {
        if self.imu_preintegrations.is_empty() || self.keyframes.len() < 2 {
            return;
        }

        let gravity = GravityModel::earth();
        let n_keyframes = self.keyframes.len();

        for i in 0..n_keyframes - 1 {
            if i >= self.imu_preintegrations.len() {
                continue;
            }

            let preintegration = &self.imu_preintegrations[i];
            if preintegration.dt <= 0.0 || preintegration.dt > 10.0 {
                debug_log!(
                    "[SlidingWindow] Skipping IMU factor for pair ({}, {}): invalid dt={:.3}",
                    i,
                    i + 1,
                    preintegration.dt
                );
                continue;
            }

            let delta_v_norm = preintegration.delta_v.norm();
            let delta_p_norm = preintegration.delta_p.norm();
            if delta_v_norm > 1000.0 || delta_p_norm > 1000.0 {
                debug_log!(
                    "[SlidingWindow] Skipping IMU factor for pair ({}, {}): large deltas (v={:.2}, p={:.2})",
                    i,
                    i + 1,
                    delta_v_norm,
                    delta_p_norm
                );
                continue;
            }

            let kf_i_var = format!("KF_{}", i);
            let vel_i_var = format!("VEL_{}", i);
            let kf_j_var = format!("KF_{}", i + 1);
            let vel_j_var = format!("VEL_{}", i + 1);

            if !workspace.initial_values.contains_key(&kf_i_var)
                || !workspace.initial_values.contains_key(&vel_i_var)
                || !workspace.initial_values.contains_key(&kf_j_var)
                || !workspace.initial_values.contains_key(&vel_j_var)
            {
                debug_log!(
                    "[SlidingWindow] Skipping IMU factor for pair ({}, {}): missing variables",
                    i,
                    i + 1
                );
                continue;
            }

            let imu_factor =
                InterKeyframeImuFactor::new(preintegration.dt, preintegration.clone(), gravity);

            let imu_huber_delta = 2.0;
            let imu_loss =
                match HuberLoss::new(imu_huber_delta) {
                    Ok(l) => Some(Box::new(l)
                        as Box<dyn apex_solver::core::loss_functions::LossFunction + Send>),
                    Err(e) => {
                        log::warn!(
                            "[SlidingWindow] Invalid IMU Huber delta ({}): {}",
                            imu_huber_delta,
                            e
                        );
                        None
                    },
                };

            workspace.problem.add_residual_block(
                &[&kf_i_var, &vel_i_var, &kf_j_var, &vel_j_var],
                Box::new(imu_factor),
                imu_loss,
            );

            debug_log!(
                "[SlidingWindow] Added InterKeyframeImuFactor for pair ({}, {}) with dt={:.3}s",
                i,
                i + 1,
                preintegration.dt
            );
        }
    }

    fn add_imu_prior_factor(
        &self,
        workspace: &mut OptimizationWorkspace,
        imu_prior: Option<ImuMotionPrior>,
        imu_weights: Option<(f64, f64)>,
        imu_huber_delta: Option<f64>,
    ) {
        let Some(prior) = imu_prior else {
            return;
        };

        if self.keyframes.is_empty() {
            return;
        }

        let last_index = self.keyframes.len().saturating_sub(1);
        let kf_var = format!("KF_{}", last_index);
        if !workspace.initial_values.contains_key(&kf_var) {
            return;
        }

        let (T_W_B_pred, _v_pred) = prior.predict_state();
        if let Some(T_B_W_pred) = T_W_B_pred.try_inverse() {
            let (w_pos, w_rot) = imu_weights.unwrap_or((1.0, 1.0));
            let factor = ImuPriorFactor::new(T_B_W_pred, w_pos, w_rot);

            let loss = if let Some(delta) = imu_huber_delta {
                if delta > 0.0 {
                    match HuberLoss::new(delta) {
                        Ok(l) => Some(Box::new(l)
                            as Box<dyn apex_solver::core::loss_functions::LossFunction + Send>),
                        Err(e) => {
                            log::warn!("[SlidingWindow] Invalid Huber delta ({}): {}", delta, e);
                            None
                        },
                    }
                } else {
                    None
                }
            } else {
                None
            };

            workspace
                .problem
                .add_residual_block(&[&kf_var], Box::new(factor), loss);
            debug_log!(
                "[SlidingWindow] Added IMU prior residual on keyframe {} (pos_weight={:.2}, rot_weight={:.2}, huber_delta={:?})",
                last_index, w_pos, w_rot, imu_huber_delta
            );
        } else {
            log::warn!(
                "[SlidingWindow] IMU prior predicted pose inversion failed; skipping IMU residual"
            );
        }
    }

    fn add_marginalization_prior_factor(&self, workspace: &mut OptimizationWorkspace) {
        if let Some(marg_prior) = self.marginalization_manager.get_prior() {
            debug_log!(
                "[SlidingWindow] Adding marginalization prior with {} parameters, residual_dim={}",
                marg_prior.param_ids.len(),
                marg_prior.residual_dim
            );

            // Build mapping of which parameters from prior exist in workspace
            // and compute their positions in both the prior and workspace
            let mut param_mapping: Vec<(String, usize)> = Vec::new(); // (var_name, dim)
            let mut prior_param_indices: Vec<usize> = Vec::new(); // indices in marg_prior.param_ids

            for (param_idx, param_id) in marg_prior.param_ids.iter().enumerate() {
                let var_name = match param_id {
                    ParamId::KeyframePose(i) => format!("KF_{}", i),
                    ParamId::KeyframeVelocity(i) => format!("VEL_{}", i),
                    ParamId::Landmark(i) => format!("LM_{}", i),
                    _ => continue,
                };

                // Check if this parameter exists in the workspace
                if let Some((_, param_val)) = workspace.initial_values.get(&var_name) {
                    param_mapping.push((var_name, param_val.len()));
                    prior_param_indices.push(param_idx);
                }
            }

            // Only add prior if we have kept parameters that exist in workspace
            if !param_mapping.is_empty() {
                // Compute original offsets in the information matrix
                let mut orig_offsets = vec![0];
                let mut offset = 0;
                for param_id in &marg_prior.param_ids {
                    if let Some(lin_pt) = marg_prior.linearization_points.get(param_id) {
                        offset += lin_pt.len();
                    }
                    orig_offsets.push(offset);
                }

                // Extract rows and columns from information matrix for existing params
                let mut info_rows = Vec::new();
                let mut info_cols = Vec::new();
                for &param_idx in &prior_param_indices {
                    let start = orig_offsets[param_idx];
                    let end = orig_offsets[param_idx + 1];
                    for i in start..end {
                        info_rows.push(i);
                        info_cols.push(i);
                    }
                }

                // Build reduced information matrix
                let total_dim = info_rows.len();
                let mut info_reduced = DMatrix::zeros(total_dim, total_dim);
                for (new_i, &old_i) in info_rows.iter().enumerate() {
                    for (new_j, &old_j) in info_cols.iter().enumerate() {
                        info_reduced[(new_i, new_j)] = marg_prior.information[(old_i, old_j)];
                    }
                }

                // Extract residual for existing parameters
                let mut residual_reduced = DVector::zeros(total_dim);
                for (new_i, &old_i) in info_rows.iter().enumerate() {
                    residual_reduced[new_i] = marg_prior.residual[old_i];
                }

                // Collect linearization points in order of existing parameters
                let mut lin_point_concat = DVector::zeros(total_dim);
                let mut offset = 0;
                for &param_idx in &prior_param_indices {
                    let param_id = &marg_prior.param_ids[param_idx];
                    if let Some(lin_point) = marg_prior.linearization_points.get(param_id) {
                        let dim = lin_point.len();
                        lin_point_concat.rows_mut(offset, dim).copy_from(lin_point);
                        offset += dim;
                    }
                }

                // Collect dimension info and variable names
                let var_names: Vec<String> = param_mapping.iter().map(|x| x.0.clone()).collect();
                let param_dims: Vec<usize> = param_mapping.iter().map(|x| x.1).collect();

                // Create joint prior factor with reduced dimensions
                let joint_prior = JointPriorFactor::new(
                    lin_point_concat,
                    info_reduced,
                    marg_prior.damping,
                    param_dims,
                );

                // Add as single residual block with existing variables
                let var_refs: Vec<&str> = var_names.iter().map(|s| s.as_str()).collect();
                workspace.problem.add_residual_block(
                    &var_refs.iter().map(|s| *s).collect::<Vec<_>>(),
                    Box::new(joint_prior),
                    None,
                );

                debug_log!(
                    "[SlidingWindow] Added joint prior factor with {} variables (total dim: {})",
                    var_names.len(),
                    total_dim
                );
            } else {
                debug_log!("[SlidingWindow] No kept parameters in optimization, skipping prior");
            }
        }
    }

    fn validate_problem_feasibility(
        &self,
        problem: &Problem,
        num_variables: usize,
        num_residuals: usize,
    ) -> Result<bool, std::io::Error> {
        if num_residuals < 6 {
            log::warn!(
                "[SlidingWindow] Too few residuals ({}), skipping optimization",
                num_residuals
            );
            return Ok(false);
        }

        if num_residuals < num_variables {
            log::warn!(
                "[SlidingWindow] Underconstrained problem: {} residuals < {} variables, skipping optimization",
                num_residuals,
                num_variables
            );
            return Ok(false);
        }

        if problem.num_residual_blocks() == 0 {
            return Err(std::io::Error::other(
                "Optimization problem has no residuals",
            ));
        }

        Ok(true)
    }

    fn solve_with_fallback_solver(
        &mut self,
        solver: &mut LevenbergMarquardt,
        problem: &Problem,
        initial_values: &HashMap<String, (ManifoldType, DVector<f64>)>,
        saved_keyframe_poses: &[Matrix4x4],
        saved_map_points: &HashMap<usize, [f32; 3]>,
    ) -> Option<SolverResult<HashMap<String, VariableEnum>>> {
        match solver.optimize(problem, initial_values) {
            Ok(result) => Some(result),
            Err(e) => {
                let error_str = format!("{:?}", e);
                if error_str.contains("LinearSolveFailed") || error_str.contains("Singular matrix")
                {
                    log::warn!(
                        "[SlidingWindow] Schur complement failed with singular matrix, trying fallback solver (SparseCholesky)"
                    );

                    let mut fallback_solver = LevenbergMarquardt::with_config(
                        LevenbergMarquardtConfig::new()
                            .with_linear_solver_type(LinearSolverType::SparseCholesky)
                            .with_max_iterations(20)
                            .with_cost_tolerance(1e-6)
                            .with_parameter_tolerance(1e-9)
                            .with_jacobi_scaling(false),
                    );

                    match fallback_solver.optimize(problem, initial_values) {
                        Ok(result) => {
                            debug_log!("[SlidingWindow] Fallback solver succeeded");
                            Some(result)
                        },
                        Err(e2) => {
                            log::error!(
                                "[SlidingWindow] Both Schur complement and fallback solver failed: {:?} - reverting to previous state",
                                e2
                            );
                            self.revert_to_saved_state(saved_keyframe_poses, saved_map_points);
                            None
                        },
                    }
                } else {
                    log::error!(
                        "[SlidingWindow] Optimization error: {:?} - reverting to previous state",
                        e
                    );
                    self.revert_to_saved_state(saved_keyframe_poses, saved_map_points);
                    None
                }
            },
        }
    }

    fn marginalize_oldest_keyframe(&mut self) {
        let param_blocks = self.build_param_blocks_for_marginalization();

        let total_observations: usize = self
            .keyframes
            .iter()
            .flat_map(|f| f.left_features.iter().chain(f.right_features.iter()))
            .count();
        let residuals = na::DVector::from_vec(vec![0.0; total_observations * 2]);

        let n_keyframes = self.keyframes.len();
        let marg_keyframe_idx = 0;

        let mut keep_ids = Vec::new();
        let mut marg_ids = Vec::new();

        for i in marg_keyframe_idx + 1..n_keyframes {
            keep_ids.push(ParamId::KeyframePose(i));
            keep_ids.push(ParamId::KeyframeVelocity(i));
        }

        marg_ids.push(ParamId::KeyframePose(marg_keyframe_idx));
        marg_ids.push(ParamId::KeyframeVelocity(marg_keyframe_idx));

        debug_log!(
            "[SlidingWindow] Marginalizing keyframe {} (pose + velocity), keeping {} keyframes",
            marg_keyframe_idx,
            n_keyframes.saturating_sub(1)
        );

        for (fid, _) in &self.map_points {
            keep_ids.push(ParamId::Landmark(*fid));
        }

        if let Some(prior) = self.marginalization_manager.marginalize_with_approximation(
            &param_blocks,
            &residuals,
            None,
            &keep_ids,
            &marg_ids,
        ) {
            debug_log!(
                "[SlidingWindow] Marginalization successful. Prior dimension: {}",
                prior.param_ids.len()
            );
            self.marginalization_manager.set_prior(prior);
            self.cleanup_after_marginalization(marg_keyframe_idx);
        } else {
            log::warn!("[SlidingWindow] Marginalization failed, skipping");
        }
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

    /// Helper function to create skew-symmetric (cross-product) matrix from 3D vector.
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

    pub(crate) fn triangulate_stereo(
        left_obs: Vector3,
        right_obs: Vector3,
        T_W_B: Matrix4x4,
        T_B_Cl: Matrix4x4,
        T_B_Cr: Matrix4x4,
    ) -> Option<Vector3> {
        let T_Cl_B = T_B_Cl.try_inverse()?;
        let T_Cl_Cr = T_Cl_B * T_B_Cr;

        let R_Cl_Cr = T_Cl_Cr.fixed_view::<3, 3>(0, 0).into_owned();
        let t_Cl_Cr = T_Cl_Cr.fixed_view::<3, 1>(0, 3).into_owned();

        let disparity = left_obs[0] - right_obs[0];
        if t_Cl_Cr[0] > 0.0 && disparity <= 0.0 {
            debug_log!(
                "[SlidingWindow] Triangulation rejected: invalid disparity {:.3} for baseline +X",
                disparity
            );
            return None;
        }

        let p_L = Vector3::zeros();
        let p_R_in_L = t_Cl_Cr;

        let dir_L = left_obs.normalize();
        let dir_R = R_Cl_Cr * right_obs.normalize();

        let w = p_L - p_R_in_L;

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
            return None;
        }

        let t_L = (b_val * e - b_val * d) / denom;
        let t_R = (a * e - b_val * d) / denom;

        if t_L <= 0.0 || t_R <= 0.0 {
            debug_log!(
                "[SlidingWindow] Triangulation rejected: negative ray parameters t_L={:.3}, t_R={:.3}",
                t_L,
                t_R
            );
            return None;
        }

        let p_L_closest = p_L + t_L * dir_L;
        let p_R_closest = p_R_in_L + t_R * dir_R;

        let p_Cl = (p_L_closest + p_R_closest) * 0.5;

        if p_Cl.z <= 0.05 {
            debug_log!(
                "[SlidingWindow] Triangulation failed: invalid depth {:.3}m",
                p_Cl.z
            );
            return None;
        }

        let T_W_Cl = T_W_B * T_B_Cl;
        let R_W_Cl = T_W_Cl.fixed_view::<3, 3>(0, 0).into_owned();
        let t_W_Cl = T_W_Cl.fixed_view::<3, 1>(0, 3).into_owned();
        let p_W = R_W_Cl * p_Cl + t_W_Cl;

        Some(p_W)
    }

    #[allow(dead_code)]
    fn triangulate_stereo_motion_aware(
        left_obs: Vector3,
        right_obs: Vector3,
        T_W_B: Matrix4x4,
        T_B_Cl: Matrix4x4,
        T_B_Cr: Matrix4x4,
        velocity: Vector3,
        angular_velocity: Vector3,
    ) -> Option<Vector3> {
        let baseline = T_B_Cl
            .try_inverse()
            .and_then(|T_Cl_B| Some((T_Cl_B * T_B_Cr).fixed_view::<3, 1>(0, 3).into_owned()))
            .map(|t| t.norm())
            .unwrap_or(0.1);

        let constraints = TriangulationConstraints::from_motion(
            na::Vector3::new(velocity.x as f64, velocity.y as f64, velocity.z as f64),
            na::Vector3::new(
                angular_velocity.x as f64,
                angular_velocity.y as f64,
                angular_velocity.z as f64,
            ),
            baseline,
            0.01,
        );

        let standard_result = Self::triangulate_stereo(left_obs, right_obs, T_W_B, T_B_Cl, T_B_Cr)?;

        let mut optimizer = MotionAwareDepthOptimizer::new();
        let position = na::Point3::new(
            standard_result.x as f64,
            standard_result.y as f64,
            standard_result.z as f64,
        );
        let velocity_magnitude =
            (velocity.x.powi(2) + velocity.y.powi(2) + velocity.z.powi(2)).sqrt() as f32;
        let constrained_depth =
            optimizer.optimize_depth(position, 0.1, &constraints, velocity_magnitude);

        if constrained_depth.confidence > 0.7 {
            Some(na::Vector3::new(
                constrained_depth.position.x as Float,
                constrained_depth.position.y as Float,
                constrained_depth.position.z as Float,
            ))
        } else {
            Some(standard_result)
        }
    }

    pub fn optimize_with_imu(
        &mut self,
        imu_prior: Option<ImuMotionPrior>,
        imu_weights: Option<(f64, f64)>,
        imu_huber_delta: Option<f64>,
    ) -> Result<bool, std::io::Error> {
        self.check_sliding_window_size_for_optimization()?;

        let saved_keyframe_poses: Vec<Matrix4x4> =
            self.keyframes.iter().map(|f| f.state.T_W_B).collect();
        let saved_map_points = self.map_points.clone();

        let mut solver = LevenbergMarquardt::with_config(self.build_solver_config());
        let estimated_landmarks = self.map_points.len().max(100);
        let estimated_keyframes = self.keyframes.len();

        let mut workspace =
            self.build_optimization_workspace(estimated_landmarks, estimated_keyframes)?;

        self.count_landmark_observations(&mut workspace);
        self.add_keyframe_states_and_visual_factors(&mut workspace)?;
        self.add_loop_closure_factors(&mut workspace);
        self.add_imu_factors(&mut workspace);
        self.add_imu_prior_factor(&mut workspace, imu_prior, imu_weights, imu_huber_delta);
        self.add_marginalization_prior_factor(&mut workspace);

        let num_residuals = workspace.problem.num_residual_blocks();
        let num_variables = workspace.initial_values.len();

        debug_log!(
            "Added SE3 and R3 variables, now {} variables total, {} residual blocks",
            num_variables,
            num_residuals
        );

        if !self.validate_problem_feasibility(&workspace.problem, num_variables, num_residuals)? {
            return Ok(false);
        }

        workspace
            .problem
            .initialize_variables(&workspace.initial_values);

        let opt_result = match self.solve_with_fallback_solver(
            &mut solver,
            &workspace.problem,
            &workspace.initial_values,
            &saved_keyframe_poses,
            &saved_map_points,
        ) {
            Some(result) => result,
            None => return Ok(false),
        };

        let is_successful = self.is_optimization_successful(&opt_result);

        if is_successful {
            self.process_optimization_result(&opt_result);

            if self
                .marginalization_manager
                .should_marginalize(self.keyframes.len())
            {
                debug_log!("[SlidingWindow] Window is full, performing marginalization");
                self.marginalize_oldest_keyframe();
            }

            debug_log!(
                "[SlidingWindow] Optimization successful. Initial cost: {:.3}, final cost: {:.3}",
                opt_result.initial_cost,
                opt_result.final_cost
            );
            Ok(true)
        } else {
            log::warn!(
                "[SlidingWindow] Optimization failed (status: {:?}) - reverting to previous state",
                opt_result.status
            );
            self.revert_to_saved_state(&saved_keyframe_poses, &saved_map_points);
            Ok(false)
        }
    }

    pub fn optimize_tight_coupled(
        &mut self,
        _imu_preintegration: Option<ImuPreintegration>,
        _gravity: GravityModel,
        _velocity_weight: f64,
        _bias_weight: f64,
    ) -> Result<bool, std::io::Error> {
        self.optimize_with_imu(None, None, None)
    }

    pub fn optimize(&mut self) -> Result<bool, std::io::Error> {
        self.optimize_with_imu(None, None, None)
    }

    pub(crate) fn build_param_blocks_for_marginalization(&self) -> HashMap<ParamId, ParamBlock> {
        let mut param_blocks = HashMap::new();

        for (i, frame) in self.keyframes.iter().enumerate() {
            let pose_dim = 7;
            let t_W_B = frame.state.T_W_B;
            let R_W_B = t_W_B.fixed_view::<3, 3>(0, 0).into_owned();
            let t_W_B_vec = t_W_B.fixed_view::<3, 1>(0, 3).into_owned();
            let q = na::UnitQuaternion::from_matrix(&R_W_B);

            let pose_linearization_point = na::DVector::from_vec(vec![
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
                    linearization_point: pose_linearization_point,
                },
            );

            let vel_dim = 3;
            let vel_linearization_point = na::DVector::from_vec(vec![
                frame.state.velocity.x as f64,
                frame.state.velocity.y as f64,
                frame.state.velocity.z as f64,
            ]);

            param_blocks.insert(
                ParamId::KeyframeVelocity(i),
                ParamBlock {
                    id: ParamId::KeyframeVelocity(i),
                    dimension: vel_dim,
                    linearization_point: vel_linearization_point,
                },
            );
        }

        for (feature_id, point) in &self.map_points {
            let landmark_dim = 3;
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

    fn revert_to_saved_state(
        &mut self,
        saved_keyframe_poses: &[Matrix4x4],
        saved_map_points: &HashMap<usize, [f32; 3]>,
    ) {
        for (i, frame) in self.keyframes.iter_mut().enumerate() {
            if i < saved_keyframe_poses.len() {
                frame.state.T_W_B = saved_keyframe_poses[i];
            }
        }

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

        self.map_points.clear();
        self.map_point_observations.clear();

        opt_result.parameters.iter().for_each(|(var_name, value)| {
            if let Some(feature_id_str) = var_name.strip_prefix("LM_") {
                if let Ok(feature_id) = feature_id_str.parse::<usize>() {
                    let vec = value.to_vector();
                    let point = [vec[0] as f32, vec[1] as f32, vec[2] as f32];

                    if point.iter().all(|&v| v.is_finite()) && vec[2] > 0.1 {
                        if self.map_points.len() >= self.max_map_points {
                            self.evict_old_map_points();
                        }

                        self.map_points.insert(feature_id, point);
                        *self.map_point_observations.entry(feature_id).or_insert(0) += 1;
                    } else {
                        log::warn!(
                            "[SlidingWindow] Rejecting invalid map point {}: [{:.2}, {:.2}, {:.2}]",
                            feature_id, point[0], point[1], point[2]
                        );
                    }
                }
            } else if let Some(frame_id_str) = var_name.strip_prefix("KF_") {
                if let Ok(frame_id) = frame_id_str.parse::<i32>() {
                    let mat = apex_solver::manifold::se3::SE3::from(value.to_vector()).matrix();
                    if let Some(frame) = self.keyframes.get_mut(frame_id as usize) {
                        match mat.try_inverse() {
                            Some(inv) => frame.state.T_W_B = inv.cast::<Float>(),
                            None => {
                                log::warn!(
                                    "[SlidingWindow] Optimized T_B_W matrix is singular for KF_{}, keeping previous pose",
                                    frame_id
                                );
                            }
                        }
                    }
                }
            } else if let Some(frame_id_str) = var_name.strip_prefix("VEL_") {
                if let Ok(frame_id) = frame_id_str.parse::<i32>() {
                    let vec = value.to_vector();
                    if let Some(frame) = self.keyframes.get_mut(frame_id as usize) {
                        frame.state.velocity = na::Vector3::new(
                            vec[0] as Float,
                            vec[1] as Float,
                            vec[2] as Float,
                        );
                        debug_log!(
                            "[SlidingWindow] Updated VEL_{} from optimization: [{:.2}, {:.2}, {:.2}]",
                            frame_id, vec[0], vec[1], vec[2]
                        );
                    }
                }
            }
        });
    }

    pub fn track_motion(&mut self, frame: &Frame) -> Result<Option<Matrix4x4>, std::io::Error> {
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
                if let Some(point) = point {
                    let factor = PnPFactor::new(
                        na::Vector2::new(feat.undistorted_coord[0], feat.undistorted_coord[1])
                            .cast::<f64>(),
                        T_C_B.cast::<f64>(),
                        na::Vector3::new(point[0] as f64, point[1] as f64, point[2] as f64),
                    );
                    let huber_loss = HuberLoss::new(2.0).map_err(|e| {
                        std::io::Error::new(std::io::ErrorKind::InvalidInput, e.to_string())
                    })?;
                    problem.add_residual_block(
                        &[&kf_var],
                        Box::new(factor),
                        Some(Box::new(huber_loss)),
                    );
                }
            }
        }

        problem.initialize_variables(&initial_values);

        let opt_result = match solver.optimize(&problem, &initial_values) {
            Ok(result) => result,
            Err(e) => {
                log::error!("[SlidingWindow] Motion tracking optimizer error: {:?}", e);
                return Ok(None);
            },
        };

        let is_successful = self.is_optimization_successful(&opt_result);

        if is_successful {
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
