/// Global bundle adjustment optimizer for the pose graph backend.
///
/// This module implements full-graph bundle adjustment for SLAM, building on
/// the sliding window optimization patterns but operating over all historical poses.
///
/// The optimizer supports:
/// - Loop closure constraints (Phase 1)
/// - Visual reprojection factors (Phase 2A)
/// - IMU preintegration factors (Phase 2B)

use crate::estimator::global_pose_graph::GlobalPoseGraph;
use crate::optimization::factors::{LoopClosurePoseFactor, BundleAdjustmentFactor};
use crate::optimization::tight_coupling::{InterKeyframeImuFactor, GravityModel};
use crate::types::{Float, Matrix3x3, Matrix4x4, Vector3};
use apex_solver::core::loss_functions::HuberLoss;
use apex_solver::core::problem::Problem;
use apex_solver::linalg::{LinearSolverType, SchurPreconditioner, SchurVariant};
use apex_solver::manifold::ManifoldType;
use apex_solver::optimizer::levenberg_marquardt::{LevenbergMarquardt, LevenbergMarquardtConfig};
use na::{DVector, UnitQuaternion};
use nalgebra as na;
use std::collections::HashMap;
use std::time::Instant;

impl GlobalPoseGraph {
    /// Run full global bundle adjustment optimization
    ///
    /// This method builds an optimization problem from the global pose graph
    /// (all historical keyframes, loop closure constraints, and map points) and
    /// solves it using Levenberg-Marquardt optimization.
    ///
    /// The optimization follows the same pattern as sliding window BA but operates
    /// on the full pose graph rather than a limited window.
    ///
    /// # Algorithm
    /// 1. Extract all keyframe poses as SE(3) variables
    /// 2. Extract all map points as 3D point variables
    /// 3. Add visual factors (reprojection errors)
    /// 4. Add loop closure factors (relative pose constraints)
    /// 5. Solve with LM optimizer using Schur complement
    /// 6. Extract and apply optimized poses
    pub fn optimize(&mut self) -> Result<crate::estimator::OptimizationResult, String> {
        let start_time = Instant::now();

        if self.keyframe_poses.is_empty() {
            return Err("No keyframes to optimize".to_string());
        }

        if self.config.enable_logging {
            log::info!(
                "[GlobalOptimizer] Starting: {} poses, {} loop closures, {} points",
                self.keyframe_poses.len(),
                self.loop_closure_edges.len(),
                self.map_points.len()
            );
        }

        // Build optimization problem
        let mut problem = Problem::new();
        let mut id_to_var: HashMap<u64, String> = HashMap::new();
        let mut initial_values: HashMap<String, (ManifoldType, DVector<f64>)> = HashMap::new();
        let mut map_point_ids: Vec<usize> = Vec::new();

        // Phase 1: Add all keyframe pose variables
        for (idx, (&id, keyframe)) in self.keyframe_poses.iter().enumerate() {
            let var_name = format!("KF_{}", idx);
            id_to_var.insert(id, var_name.clone());

            // Convert T_W_B (world to body) to body to world for optimization
            let T_B_W = match keyframe.T_W_B.clone().try_inverse() {
                Some(inv) => inv,
                None => {
                    log::warn!("[GlobalOptimizer] T_W_B inversion failed for keyframe {}", id);
                    continue;
                }
            };

            let t_B_W = T_B_W.fixed_view::<3, 1>(0, 3);
            let R_B_W = Matrix3x3::from(T_B_W.fixed_view::<3, 3>(0, 0));
            let q_B_W = UnitQuaternion::from_matrix(&R_B_W);

            let se3_data = DVector::from_vec(vec![
                t_B_W.x as f64,
                t_B_W.y as f64,
                t_B_W.z as f64,
                q_B_W.w as f64,
                q_B_W.i as f64,
                q_B_W.j as f64,
                q_B_W.k as f64,
            ]);

            initial_values.insert(var_name, (ManifoldType::SE3, se3_data));
        }

        if self.config.enable_logging {
            log::debug!("[GlobalOptimizer] Added {} keyframe pose variables", initial_values.len());
        }

        // Phase 2: Add map point variables from observations
        for (&point_id, point) in self.map_points.iter() {
            map_point_ids.push(point_id);
            let mp_var = format!("MP_{}", point_id);

            let mp_data = DVector::from_vec(vec![
                point.position.x as f64,
                point.position.y as f64,
                point.position.z as f64,
            ]);

            initial_values.insert(mp_var, (ManifoldType::RN, mp_data));
        }

        if self.config.enable_logging {
            log::debug!("[GlobalOptimizer] Added {} map point variables", map_point_ids.len());
        }

        // Phase 3: Add visual reprojection factors (Phase 2A)
        // These factors constrain both pose and landmark variables using feature observations
        let mut num_visual_factors = 0;
        for (keyframe_id, keyframe) in self.keyframe_poses.iter() {
            let kf_var = match id_to_var.get(keyframe_id) {
                Some(v) => v.clone(),
                None => continue,
            };

            // Get camera calibration transforms (inverted: T_C_B not T_B_C)
            let T_Cl_B = match keyframe.T_B_Cl.try_inverse() {
                Some(inv) => inv.cast::<f64>(),
                None => {
                    log::warn!("[GlobalOptimizer] T_B_Cl inversion failed for keyframe {}", keyframe_id);
                    continue;
                }
            };

            let T_Cr_B = match keyframe.T_B_Cr.try_inverse() {
                Some(inv) => inv.cast::<f64>(),
                None => {
                    log::warn!("[GlobalOptimizer] T_B_Cr inversion failed for keyframe {}", keyframe_id);
                    continue;
                }
            };

            // Process left camera observations
            for (feature_id, obs_coord) in &keyframe.left_feature_observations {
                // Find the corresponding map point
                let mp_var = format!("MP_{}", feature_id);
                if !initial_values.contains_key(&mp_var) {
                    continue;  // Map point not in optimization
                }

                // Create observation as 2D normalized coordinates
                let observation = na::Vector2::new(obs_coord.0, obs_coord.1);
                
                let factor = BundleAdjustmentFactor::new(observation, T_Cl_B.clone())
                    .with_weight(1.0);

                let loss = HuberLoss::new(1.0)
                    .ok()
                    .map(|l| Box::new(l) as Box<dyn apex_solver::core::loss_functions::LossFunction + Send>);

                problem.add_residual_block(&[&kf_var, &mp_var], Box::new(factor), loss);
                num_visual_factors += 1;
            }

            // Process right camera observations
            for (feature_id, obs_coord) in &keyframe.right_feature_observations {
                let mp_var = format!("MP_{}", feature_id);
                if !initial_values.contains_key(&mp_var) {
                    continue;
                }

                let observation = na::Vector2::new(obs_coord.0, obs_coord.1);

                let factor = BundleAdjustmentFactor::new(observation, T_Cr_B.clone())
                    .with_weight(1.0);

                let loss = HuberLoss::new(1.0)
                    .ok()
                    .map(|l| Box::new(l) as Box<dyn apex_solver::core::loss_functions::LossFunction + Send>);

                problem.add_residual_block(&[&kf_var, &mp_var], Box::new(factor), loss);
                num_visual_factors += 1;
            }
        }

        if self.config.enable_logging {
            log::debug!("[GlobalOptimizer] Added {} visual reprojection factors", num_visual_factors);
        }

        // Phase 4: Add loop closure factors
        let mut num_closure_factors = 0;
        for edge in self.loop_closure_edges.iter() {
            let var1 = match id_to_var.get(&edge.from_id) {
                Some(v) => v.clone(),
                None => continue,
            };
            let var2 = match id_to_var.get(&edge.to_id) {
                Some(v) => v.clone(),
                None => continue,
            };

            if !initial_values.contains_key(&var1) || !initial_values.contains_key(&var2) {
                continue;
            }

            let factor = LoopClosurePoseFactor::new(
                edge.T_from_to.to_homogeneous().cast::<f64>(),
                edge.covariance.cast::<f64>(),
            );

            let loss = HuberLoss::new(1.0)
                .ok()
                .map(|l| Box::new(l) as Box<dyn apex_solver::core::loss_functions::LossFunction + Send>);

            problem.add_residual_block(&[&var1, &var2], Box::new(factor), loss);
            num_closure_factors += 1;
        }

        if self.config.enable_logging {
            log::debug!("[GlobalOptimizer] Added {} loop closure factors", num_closure_factors);
        }

        // Phase 5: Add velocity variables for each keyframe (Phase 2B)
        let mut velocity_var_map: HashMap<u64, String> = HashMap::new();
        for (idx, (&id, keyframe)) in self.keyframe_poses.iter().enumerate() {
            let vel_var = format!("VEL_{}", idx);
            velocity_var_map.insert(id, vel_var.clone());

            // Store velocity as R3 (3D vector)
            let vel_data = DVector::from_vec(vec![
                keyframe.velocity.x as f64,
                keyframe.velocity.y as f64,
                keyframe.velocity.z as f64,
            ]);

            initial_values.insert(vel_var, (ManifoldType::RN, vel_data));
        }

        if self.config.enable_logging {
            log::debug!("[GlobalOptimizer] Added {} velocity variables", velocity_var_map.len());
        }

        // Phase 5.1: Add IMU preintegration factors
        // These factors constrain consecutive poses and velocities using IMU measurements
        let mut num_imu_factors = 0;
        let gravity = GravityModel::earth();

        for edge in self.imu_edges.iter() {
            let var_i = match id_to_var.get(&edge.from_id) {
                Some(v) => v.clone(),
                None => continue,
            };
            let var_j = match id_to_var.get(&edge.to_id) {
                Some(v) => v.clone(),
                None => continue,
            };

            let vel_i = match velocity_var_map.get(&edge.from_id) {
                Some(v) => v.clone(),
                None => continue,
            };
            let vel_j = match velocity_var_map.get(&edge.to_id) {
                Some(v) => v.clone(),
                None => continue,
            };

            // Create IMU factor with 4 variables: pose_i, vel_i, pose_j, vel_j
            let factor = InterKeyframeImuFactor::new(
                edge.preintegration.dt,
                edge.preintegration.clone(),
                gravity.clone(),
            );

            let loss = HuberLoss::new(1.0)
                .ok()
                .map(|l| Box::new(l) as Box<dyn apex_solver::core::loss_functions::LossFunction + Send>);

            problem.add_residual_block(
                &[&var_i, &vel_i, &var_j, &vel_j],
                Box::new(factor),
                loss,
            );
            num_imu_factors += 1;
        }

        if self.config.enable_logging {
            log::debug!("[GlobalOptimizer] Added {} IMU preintegration factors", num_imu_factors);
        }

        // Phase 6: Solver configuration
        let config = LevenbergMarquardtConfig::new()
            .with_linear_solver_type(LinearSolverType::SparseSchurComplement)
            .with_schur_variant(SchurVariant::Sparse)
            .with_schur_preconditioner(SchurPreconditioner::BlockDiagonal)
            .with_max_iterations(self.config.max_iterations)
            .with_cost_tolerance(self.config.cost_tolerance)
            .with_parameter_tolerance(1e-9)
            .with_jacobi_scaling(false);

        // Phase 7: Initialize solver and solve
        let mut solver = LevenbergMarquardt::with_config(config);

        let result = match solver.optimize(&problem, &initial_values) {
            Ok(r) => r,
            Err(e) => {
                log::warn!("[GlobalOptimizer] Solver failed: {:?}", e);
                let optimization_time = start_time.elapsed().as_secs_f64() * 1000.0;
                self.new_closures_since_last_opt = 0;
                self.last_optimization_time = Some(Instant::now());
                self.stats.last_optimization_time_ms = optimization_time;
                self.stats.total_optimizations += 1;

                return Ok(crate::estimator::OptimizationResult {
                    iterations: 0,
                    final_cost: 0.0,
                    converged: false,
                    optimization_time_ms: optimization_time,
                });
            }
        };

        // Phase 8: Extract optimized poses
        let num_iterations = result.iterations as usize;
        let converged = matches!(
            &result.status,
            apex_solver::optimizer::OptimizationStatus::Converged
                | apex_solver::optimizer::OptimizationStatus::CostToleranceReached
                | apex_solver::optimizer::OptimizationStatus::ParameterToleranceReached
        );

        for (id, var_name) in id_to_var.iter() {
            if let Some(var_enum) = result.parameters.get(var_name) {
                // Convert VariableEnum to SE3 vector and back to matrix
                let T_B_W_opt = apex_solver::manifold::se3::SE3::from(var_enum.to_vector()).matrix();
                if let Some(T_W_B) = T_B_W_opt.try_inverse() {
                    // Update pose in graph
                    if let Some(keyframe) = self.keyframe_poses.get_mut(id) {
                        keyframe.T_W_B = T_W_B.cast::<Float>();
                    }
                }
            }
        }

        // Phase 9: Extract optimized map points
        for point_id in map_point_ids {
            let mp_var = format!("MP_{}", point_id);
            if let Some(var_enum) = result.parameters.get(&mp_var) {
                let vec = var_enum.to_vector();
                if vec.len() >= 3 {
                    if let Some(point) = self.map_points.get_mut(&point_id) {
                        point.position = Vector3::new(
                            vec[0] as Float,
                            vec[1] as Float,
                            vec[2] as Float,
                        );
                    }
                }
            }
        }

        // Phase 10: Extract optimized velocities (Phase 2B)
        for (id, vel_var) in velocity_var_map.iter() {
            if let Some(var_enum) = result.parameters.get(vel_var) {
                let vec = var_enum.to_vector();
                if vec.len() >= 3 {
                    if let Some(keyframe) = self.keyframe_poses.get_mut(id) {
                        keyframe.velocity = Vector3::new(
                            vec[0] as Float,
                            vec[1] as Float,
                            vec[2] as Float,
                        );
                    }
                }
            }
        }

        let optimization_time = start_time.elapsed().as_secs_f64() * 1000.0;

        // Update statistics
        self.new_closures_since_last_opt = 0;
        self.last_optimization_time = Some(Instant::now());
        self.stats.last_optimization_time_ms = optimization_time;
        self.stats.last_optimization_iterations = num_iterations;
        self.stats.total_optimizations += 1;

        if self.config.enable_logging {
            log::info!(
                "[GlobalOptimizer] Completed: {:.1}ms, {} iterations, converged={}",
                optimization_time, num_iterations, converged
            );
        }

        Ok(crate::estimator::OptimizationResult {
            iterations: num_iterations,
            final_cost: result.final_cost,
            converged,
            optimization_time_ms: optimization_time,
        })
    }

    /// Get all optimized poses from the global graph
    ///
    /// Returns a vector of (keyframe_id, pose) pairs that can be applied
    /// back to the sliding window for consistency.
    pub fn get_optimized_poses(&self) -> Vec<(u64, Matrix4x4)> {
        self.keyframe_poses
            .iter()
            .map(|(id, kf)| (*id, kf.T_W_B))
            .collect()
    }
}
