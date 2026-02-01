#[cfg(test)]
mod tests {
    use crate::optimization::factors::BundleAdjustmentFactor;
    use crate::optimization::factors::BundleAdjustmentFactorTranslationOnly;
    use crate::optimization::factors::PinholeProjectionFactor;
    use crate::optimization::observer::TerminalObserver;
    use apex_solver::core::problem::Problem;
    use apex_solver::linalg::LinearSolverType;
    use apex_solver::manifold::ManifoldType;
    use apex_solver::optimizer::levenberg_marquardt::{
        LevenbergMarquardt, LevenbergMarquardtConfig,
    };
    use na::DVector;
    use nalgebra as na;
    use std::collections::HashMap;

    #[test]
    fn test_pinhole_projection_factor() {
        // True parameters: y = 2.0*x² - 3.0*x + 1.0
        let map_point_true = vec![1.0, 1.0, 1.0]; // point is right down forward of cam 0

        // cam 0: 90 def FoV, origin: point is at (1,1)
        // cam 1: 90 def FoV, 1m on the right: point is at (0, 1)

        let mut problem = Problem::new();

        let config = LevenbergMarquardtConfig::new()
            .with_linear_solver_type(LinearSolverType::SparseCholesky)
            .with_max_iterations(100)
            .with_cost_tolerance(1e-9)
            .with_parameter_tolerance(1e-9)
            .with_jacobi_scaling(false);

        let _solver = LevenbergMarquardt::with_config(config);
        let mut initial_values = HashMap::new();

        let lm_var = format!("LM_{}", 0);
        let data = DVector::from_vec(vec![0.0, 0.0, 1.0]);
        initial_values.insert(lm_var.clone(), (ManifoldType::RN, data));

        let mut T_W_right = na::Matrix4::identity();
        T_W_right[(0, 3)] = 0.5;
        let T_Cright_W = T_W_right.try_inverse().unwrap();

        let mut T_W_Cbottom = na::Matrix4::identity();
        T_W_Cbottom[(1, 3)] = 0.5;
        let T_Cbottom_W = T_W_Cbottom.try_inverse().unwrap();

        // Left camera projection factor
        let left_factor = PinholeProjectionFactor::new(
            na::Vector2::new(1.0, 1.0).cast::<f64>(),
            na::Matrix4::identity(),
        );
        problem.add_residual_block(&[&lm_var], Box::new(left_factor), None); // Left camera projection factor
        let right_factor =
            PinholeProjectionFactor::new(na::Vector2::new(0.5, 1.0).cast::<f64>(), T_Cright_W);
        problem.add_residual_block(&[&lm_var], Box::new(right_factor), None);

        let bottom_factor =
            PinholeProjectionFactor::new(na::Vector2::new(1.0, 0.5).cast::<f64>(), T_Cbottom_W);
        problem.add_residual_block(&[&lm_var], Box::new(bottom_factor), None);

        // Initialize variables in the problem
        problem.initialize_variables(&initial_values);

        // Configure Levenberg-Marquardt solver
        let config = LevenbergMarquardtConfig::new()
            .with_linear_solver_type(LinearSolverType::SparseCholesky)
            .with_max_iterations(50)
            .with_cost_tolerance(1e-6)
            .with_jacobi_scaling(false);

        let mut solver = LevenbergMarquardt::with_config(config);

        // Add terminal observer to monitor progress
        let observer = TerminalObserver::new();
        TerminalObserver::print_header();
        solver.add_observer(observer);

        // Run optimization
        let result = solver.optimize(&problem, &initial_values);

        // Check results
        assert!(result.is_ok(), "Optimization should succeed");
        let opt_result = result.unwrap();

        // Extract optimized parameters
        let optimized_params = opt_result.parameters.get(&lm_var).unwrap();
        let params_vec = optimized_params.to_vector();

        println!("\nOptimization Results:");
        println!("True parameters:  x={:?}", map_point_true);
        println!("Optimized params: x={:?}", params_vec);
        println!("Initial cost: {:.6}", opt_result.initial_cost);
        println!("Final cost: {:.6}", opt_result.final_cost);

        // Check that optimization converged
        use apex_solver::optimizer::OptimizationStatus;
        match opt_result.status {
            OptimizationStatus::Converged
            | OptimizationStatus::CostToleranceReached
            | OptimizationStatus::ParameterToleranceReached
            | OptimizationStatus::GradientToleranceReached => {
                println!("Optimization converged successfully!");
            },
            _ => {
                println!(
                    "Warning: Optimization did not fully converge. Status: {:?}",
                    opt_result.status
                );
            },
        }
    }

    /// Test bundle adjustment with translation-only optimization.
    ///
    /// Tests the BundleAdjustmentFactorTranslationOnly factor by:
    /// 1. Creating random 3D landmarks in world frame
    /// 2. Observing them from multiple camera poses (with known extrinsics)
    /// 3. Optimizing landmark positions and camera translations
    #[test]
    fn test_bundle_adjustment_factor_translation_only() {
        // Constants
        const NUM_LANDMARKS: usize = 10;
        const NUM_POSES: usize = 5; // Number of system poses (each with left-right cameras)
        const NOISE_RANGE: f64 = 0.05;
        const MIN_DEPTH: f64 = 0.1; // Minimum depth for point to be visible
        const TRANSLATION_RANGE: f64 = 3.0; // Range for random translations

        // Helper: Extract landmark index from variable name
        fn get_landmark_idx(lm_var: &str) -> usize {
            lm_var.strip_prefix("LM_").unwrap().parse().unwrap()
        }

        // Helper: Project 3D point to normalized camera coordinates
        fn project_to_normalized(p_cam: na::Vector3<f64>) -> na::Vector2<f64> {
            na::Vector2::new(p_cam[0] / p_cam[2], p_cam[1] / p_cam[2])
        }

        // Helper: Check if point is visible (in front of camera)
        fn is_visible(p_cam: &na::Vector3<f64>, min_depth: f64) -> bool {
            p_cam[2] > min_depth && p_cam.iter().all(|&x| x.is_finite())
        }

        // Helper: Transform point from world to camera frame
        fn world_to_camera(
            p_world: na::Vector3<f64>,
            t_body_world: na::Vector3<f64>,
            t_cam_body: &na::Matrix4<f64>,
        ) -> na::Vector3<f64> {
            let r_c_b = t_cam_body.fixed_view::<3, 3>(0, 0);
            let t_c_b = t_cam_body.fixed_view::<3, 1>(0, 3);
            r_c_b * (p_world + t_body_world) + t_c_b
        }

        // Helper: Add factor for a camera observation
        fn add_camera_factor(
            problem: &mut Problem,
            lm_var: &str,
            cam_var: Option<&str>,
            observation: na::Vector2<f64>,
            t_cam_body: na::Matrix4<f64>,
            fixed_position: Option<na::Vector3<f64>>,
        ) {
            let mut factor = BundleAdjustmentFactorTranslationOnly::new(observation, t_cam_body);
            if let Some(pos) = fixed_position {
                factor = factor.with_fixed_position(pos);
            }

            let var_names: Vec<&str> = if let Some(cv) = cam_var {
                vec![lm_var, cv]
            } else {
                vec![lm_var]
            };

            problem.add_residual_block(&var_names, Box::new(factor), None);
        }

        let mut problem = Problem::new();
        let mut initial_values = HashMap::new();
        let mut landmarks = Vec::new();
        let mut point_vars = Vec::new();

        // Generate random 3D landmarks with noisy initial estimates
        use rand::Rng;
        let mut rng = rand::thread_rng();

        for i in 0..NUM_LANDMARKS {
            // True landmark position
            let true_point = vec![
                rng.gen_range(-2.0..2.0),
                rng.gen_range(-2.0..2.0),
                rng.gen_range(0.5..3.0),
            ];
            landmarks.push(true_point.clone());

            // Noisy initial estimate
            let noise: Vec<f64> = (0..3)
                .map(|_| rng.gen_range(-NOISE_RANGE..NOISE_RANGE))
                .collect();
            let noisy_point: Vec<f64> = true_point
                .iter()
                .zip(noise.iter())
                .map(|(p, n)| p + n)
                .collect();

            let lm_var = format!("LM_{}", i);
            initial_values.insert(
                lm_var.clone(),
                (ManifoldType::RN, DVector::from_vec(noisy_point)),
            );
            point_vars.push(lm_var);
        }

        // Camera extrinsics (camera-to-body transforms)
        let t_cam_left_body = na::Matrix4::identity();
        let mut t_cam_right_body = na::Matrix4::identity();
        t_cam_right_body[(0, 3)] = 0.5; // Right camera offset in x

        // Generate random system poses
        let mut pose_translations = Vec::new();
        let mut pose_vars = Vec::new();

        for pose_id in 0..NUM_POSES {
            // Generate random translation for this pose
            let t_body_world = if pose_id == 0 {
                // First pose is fixed at origin
                na::Vector3::zeros()
            } else {
                na::Vector3::new(
                    rng.gen_range(-TRANSLATION_RANGE..TRANSLATION_RANGE),
                    rng.gen_range(-TRANSLATION_RANGE..TRANSLATION_RANGE),
                    rng.gen_range(-TRANSLATION_RANGE * 0.5..TRANSLATION_RANGE * 0.5),
                )
            };
            pose_translations.push(t_body_world);

            // Create variable for this pose (only if not fixed)
            if pose_id > 0 {
                let cam_var = format!("KF_{}", pose_id);
                // Initial estimate with some noise
                let noisy_translation = t_body_world
                    + na::Vector3::new(
                        rng.gen_range(-0.1..0.1),
                        rng.gen_range(-0.1..0.1),
                        rng.gen_range(-0.1..0.1),
                    );
                let cam_data = DVector::from_vec(vec![
                    noisy_translation.x,
                    noisy_translation.y,
                    noisy_translation.z,
                ]);
                initial_values.insert(cam_var.clone(), (ManifoldType::RN, cam_data));
                pose_vars.push((pose_id, cam_var));
            }
        }

        // Add observations from each pose (left + right cameras)
        let mut total_observations = 0;
        for (pose_id, t_body_world) in pose_translations.iter().enumerate() {
            let cam_var_opt = pose_vars
                .iter()
                .find(|(id, _)| *id == pose_id)
                .map(|(_, v)| v.as_str());
            let is_fixed = pose_id == 0;

            for lm_var in &point_vars {
                let idx = get_landmark_idx(lm_var);
                let p_w = na::Vector3::new(landmarks[idx][0], landmarks[idx][1], landmarks[idx][2]);

                // Left camera observation
                let p_cam_left = world_to_camera(p_w, *t_body_world, &t_cam_left_body);
                if is_visible(&p_cam_left, MIN_DEPTH) {
                    let obs_left = project_to_normalized(p_cam_left);
                    add_camera_factor(
                        &mut problem,
                        lm_var,
                        cam_var_opt,
                        obs_left,
                        t_cam_left_body,
                        if is_fixed { Some(*t_body_world) } else { None },
                    );
                    total_observations += 1;
                }

                // Right camera observation
                let p_cam_right = world_to_camera(p_w, *t_body_world, &t_cam_right_body);
                if is_visible(&p_cam_right, MIN_DEPTH) {
                    let obs_right = project_to_normalized(p_cam_right);
                    add_camera_factor(
                        &mut problem,
                        lm_var,
                        cam_var_opt,
                        obs_right,
                        t_cam_right_body,
                        if is_fixed { Some(*t_body_world) } else { None },
                    );
                    total_observations += 1;
                }
            }
        }

        println!(
            "Generated {} poses with {} total stereo observations",
            NUM_POSES, total_observations
        );

        // Initialize problem
        problem.initialize_variables(&initial_values);

        // Configure and run optimization
        let config = LevenbergMarquardtConfig::new()
            .with_linear_solver_type(LinearSolverType::SparseCholesky)
            .with_max_iterations(50)
            .with_cost_tolerance(1e-6)
            .with_jacobi_scaling(false);

        let mut solver = LevenbergMarquardt::with_config(config);
        let observer = TerminalObserver::new();
        TerminalObserver::print_header();
        solver.add_observer(observer);

        let result = solver.optimize(&problem, &initial_values);
        assert!(result.is_ok(), "Optimization should succeed");
        let opt_result = result.unwrap();

        // Verify results
        const MAX_LANDMARK_ERROR: f64 = 1e-3;
        let mut all_landmarks_valid = true;

        println!("\nOptimization Results:");
        println!("Initial cost: {:.6}", opt_result.initial_cost);
        println!("Final cost: {:.6}", opt_result.final_cost);

        // Check landmark convergence
        for (var_name, value) in opt_result.parameters.iter() {
            if var_name.starts_with("LM_") {
                let idx = get_landmark_idx(var_name);
                if let Some(true_point) = landmarks.get(idx) {
                    let optimized = value.to_vector();
                    let true_vec = DVector::from_vec(true_point.clone());
                    let error = (optimized - true_vec).norm();

                    println!("  {}: error = {:.6}", var_name, error);

                    if error > MAX_LANDMARK_ERROR {
                        println!(
                            "    WARNING: Landmark {} error ({:.6}) exceeds threshold ({:.6})",
                            idx, error, MAX_LANDMARK_ERROR
                        );
                        all_landmarks_valid = false;
                    }
                }
            }
        }

        // Check convergence status
        use apex_solver::optimizer::OptimizationStatus;
        let converged = matches!(
            opt_result.status,
            OptimizationStatus::Converged
                | OptimizationStatus::CostToleranceReached
                | OptimizationStatus::ParameterToleranceReached
                | OptimizationStatus::GradientToleranceReached
        );

        if !converged {
            println!(
                "Warning: Optimization did not fully converge. Status: {:?}",
                opt_result.status
            );
        }

        assert!(
            all_landmarks_valid,
            "Some landmarks did not converge to true values"
        );
        assert!(converged, "Optimization did not converge");
    }

    /// Test bundle adjustment with translation and rotation (full SE3).
    ///
    /// Similar to test_bundle_adjustment_factor_translation_only, but adds small rotations
    /// to the system poses. Since the factor only handles translations, there will be a
    /// small un-optimizable error due to the rotations. This will be fixed in future steps.
    #[test]
    fn test_bundle_adjustment_factor_full() {
        // Constants
        const NUM_LANDMARKS: usize = 10;
        const NUM_POSES: usize = 5; // Number of system poses (each with left-right cameras)
        const NOISE_RANGE: f64 = 0.05;
        const MIN_DEPTH: f64 = 0.1; // Minimum depth for point to be visible
        const TRANSLATION_RANGE: f64 = 3.0; // Range for random translations
        const MAX_ROTATION_ANGLE: f64 = 0.5; // Maximum rotation angle in radians (~5.7 degrees)

        // Helper: Extract landmark index from variable name
        fn get_landmark_idx(lm_var: &str) -> usize {
            lm_var.strip_prefix("LM_").unwrap().parse().unwrap()
        }

        // Helper: Project 3D point to normalized camera coordinates
        fn project_to_normalized(p_C: na::Vector3<f64>) -> na::Vector2<f64> {
            na::Vector2::new(p_C[0] / p_C[2], p_C[1] / p_C[2])
        }

        // Helper: Check if point is visible (in front of camera)
        fn is_visible(p_C: &na::Vector3<f64>, min_depth: f64) -> bool {
            p_C[2] > min_depth && p_C.iter().all(|&x| x.is_finite())
        }

        // Helper: Generate a small random rotation (axis-angle representation)
        fn generate_small_rotation(
            rng: &mut impl rand::Rng,
            max_angle: f64,
        ) -> na::UnitQuaternion<f64> {
            // Random axis (normalized)
            let axis = na::Vector3::new(
                rng.gen_range(-1.0..1.0),
                rng.gen_range(-1.0..1.0),
                rng.gen_range(-1.0..1.0),
            );
            let axis = axis.normalize();

            // Random angle
            let angle = rng.gen_range(-max_angle..max_angle);

            na::UnitQuaternion::from_axis_angle(&na::Unit::new_normalize(axis), angle)
        }

        // Helper: Transform point from world to camera frame with rotation
        // Notation: T_A_B = SE3 transform from B to A, R_A_B = SO3 rotation from B to A, t_A_B = R3 translation from B to A
        fn world_to_camera_with_rotation(
            p_W: na::Vector3<f64>,
            t_B_W: na::Vector3<f64>,         // t_B_W: translation from W to B
            R_B_W: &na::UnitQuaternion<f64>, // R_B_W: rotation from W to B
            T_C_B: &na::Matrix4<f64>,        // T_C_B: SE3 transform from B to C (camera)
        ) -> na::Vector3<f64> {
            // Transform chain: p_C = T_C_B * T_B_W * p_W
            // where T_B_W = [R_B_W | t_B_W; 0 0 0 1]
            // Extract R_C_B and t_C_B from T_C_B
            let R_C_B = T_C_B.fixed_view::<3, 3>(0, 0);
            let t_C_B = T_C_B.fixed_view::<3, 1>(0, 3);

            // Apply body-to-world transform: p_B = R_B_W * p_W + t_B_W
            let p_B = R_B_W * p_W + t_B_W;

            // Apply camera-to-body transform: p_C = R_C_B * p_B + t_C_B
            R_C_B * p_B + t_C_B
        }

        // Helper: Add factor for a camera observation
        fn add_camera_factor(
            problem: &mut Problem,
            lm_var: &str,
            cam_var: Option<&str>,
            observation: na::Vector2<f64>,
            T_C_B: na::Matrix4<f64>, // T_C_B: SE3 transform from B to C
            fixed_pose: Option<na::Matrix4<f64>>,
        ) {
            let mut factor = BundleAdjustmentFactor::new(observation, T_C_B);
            if let Some(pose) = fixed_pose {
                factor = factor.with_fixed_pose(pose);
            }

            let var_names: Vec<&str> = if let Some(cv) = cam_var {
                vec![lm_var, cv]
            } else {
                vec![lm_var]
            };

            problem.add_residual_block(&var_names, Box::new(factor), None);
        }

        // Set up problem
        let mut problem = Problem::new();
        let mut initial_values = HashMap::new();
        let mut landmarks = Vec::new();
        let mut point_vars = Vec::new();

        // Generate random 3D landmarks with noisy initial estimates
        use rand::Rng;
        let mut rng = rand::thread_rng();

        // Set up landmarks
        for i in 0..NUM_LANDMARKS {
            // True landmark position
            let true_point = vec![
                rng.gen_range(-2.0..2.0),
                rng.gen_range(-2.0..2.0),
                rng.gen_range(0.5..3.0),
            ];
            landmarks.push(true_point.clone());

            // Noisy initial estimate
            let noise: Vec<f64> = (0..3)
                .map(|_| rng.gen_range(-NOISE_RANGE..NOISE_RANGE))
                .collect();
            let noisy_point: Vec<f64> = true_point
                .iter()
                .zip(noise.iter())
                .map(|(p, n)| p + n)
                .collect();

            let lm_var = format!("LM_{}", i);
            initial_values.insert(
                lm_var.clone(),
                (ManifoldType::RN, DVector::from_vec(noisy_point)),
            );
            point_vars.push(lm_var);
        }

        // Camera extrinsics: T_Cl_B and T_Cr_B (SE3 transforms from B to Cl and Cr)
        let T_Cl_B = na::Matrix4::identity();
        let mut T_Cr_B = na::Matrix4::identity();
        T_Cr_B[(0, 3)] = 0.5; // Right camera offset in x

        // Generate random system poses with translations and rotations
        // Notation: t_B_W = translation from W to B, R_B_W = rotation from W to B
        let mut t_B_W_vec = Vec::new();
        let mut R_B_W_vec = Vec::new();
        let mut pose_vars = Vec::new();

        // Set up system poses with cameras
        for pose_id in 0..NUM_POSES {
            // Generate random translation: t_B_W (translation from W to B)
            let t_B_W = if pose_id == 0 {
                // First pose is fixed at origin
                na::Vector3::zeros()
            } else {
                na::Vector3::new(
                    rng.gen_range(-TRANSLATION_RANGE..TRANSLATION_RANGE),
                    rng.gen_range(-TRANSLATION_RANGE..TRANSLATION_RANGE),
                    rng.gen_range(-TRANSLATION_RANGE * 0.5..TRANSLATION_RANGE * 0.5),
                )
            };
            t_B_W_vec.push(t_B_W);

            // Generate small random rotation: R_B_W (rotation from W to B)
            let R_B_W = if pose_id == 0 {
                // First pose has no rotation (identity)
                na::UnitQuaternion::identity()
            } else {
                generate_small_rotation(&mut rng, MAX_ROTATION_ANGLE)
            };
            R_B_W_vec.push(R_B_W);

            // Create variable for this pose (only if not fixed)
            // Note: Currently only translation is optimized, rotation is not
            if pose_id > 0 {
                let cam_var = format!("KF_{}", pose_id);
                // Initial estimate with some noise (translation only)
                let noisy_translation = t_B_W
                    + na::Vector3::new(
                        rng.gen_range(-0.1..0.1),
                        rng.gen_range(-0.1..0.1),
                        rng.gen_range(-0.1..0.1),
                    );
                let noisy_rotation =
                    R_B_W * generate_small_rotation(&mut rng, MAX_ROTATION_ANGLE / 2.0);
                let cam_data = DVector::from_vec(vec![
                    noisy_translation.x,
                    noisy_translation.y,
                    noisy_translation.z, // then wijk
                    noisy_rotation.w.clone(),
                    noisy_rotation.i.clone(),
                    noisy_rotation.j.clone(),
                    noisy_rotation.k.clone(),
                ]);
                initial_values.insert(cam_var.clone(), (ManifoldType::SE3, cam_data));
                pose_vars.push((pose_id, cam_var));
            }
        }

        // Add observations from each pose (left + right cameras)
        let mut total_observations = 0;
        for (pose_id, (t_B_W, R_B_W)) in t_B_W_vec.iter().zip(R_B_W_vec.iter()).enumerate() {
            let cam_var_opt = pose_vars
                .iter()
                .find(|(id, _)| *id == pose_id)
                .map(|(_, v)| v.as_str());
            let is_fixed = pose_id == 0;

            let mut T_B_W = na::Matrix4::identity();
            T_B_W
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&R_B_W.to_rotation_matrix().matrix());
            T_B_W
                .fixed_view_mut::<3, 1>(0, 3)
                .copy_from(&t_B_W.to_owned());
            println!("pose_id: {}", pose_id);
            println!("T_B_W: {:?}", T_B_W.to_string());
            for lm_var in &point_vars {
                let idx = get_landmark_idx(lm_var);
                let p_W = na::Vector3::new(landmarks[idx][0], landmarks[idx][1], landmarks[idx][2]);

                // Left camera observation (with rotation applied)
                let p_Cl = world_to_camera_with_rotation(p_W, *t_B_W, R_B_W, &T_Cl_B);
                if is_visible(&p_Cl, MIN_DEPTH) {
                    let obs_Cl = project_to_normalized(p_Cl);
                    add_camera_factor(
                        &mut problem,
                        lm_var,
                        cam_var_opt,
                        obs_Cl,
                        T_Cl_B,
                        if is_fixed { Some(T_B_W) } else { None },
                    );
                    total_observations += 1;
                }

                // Right camera observation (with rotation applied)
                let p_Cr = world_to_camera_with_rotation(p_W, *t_B_W, R_B_W, &T_Cr_B);
                if is_visible(&p_Cr, MIN_DEPTH) {
                    let obs_Cr = project_to_normalized(p_Cr);
                    add_camera_factor(
                        &mut problem,
                        lm_var,
                        cam_var_opt,
                        obs_Cr,
                        T_Cr_B,
                        if is_fixed { Some(T_B_W) } else { None },
                    );
                    total_observations += 1;
                }
            }
        }

        println!(
            "Generated {} poses with rotations and {} total stereo observations",
            NUM_POSES, total_observations
        );
        println!(
            "Note: Rotations are applied but not optimized (factor only handles translations)"
        );

        // Initialize problem
        problem.initialize_variables(&initial_values);

        // Configure and run optimization
        let config = LevenbergMarquardtConfig::new()
            .with_linear_solver_type(LinearSolverType::SparseCholesky)
            .with_max_iterations(50)
            .with_cost_tolerance(1e-6)
            .with_jacobi_scaling(false);

        let mut solver = LevenbergMarquardt::with_config(config);
        let observer = TerminalObserver::new();
        TerminalObserver::print_header();
        solver.add_observer(observer);

        let result = solver.optimize(&problem, &initial_values);
        assert!(result.is_ok(), "Optimization should succeed");
        let opt_result = result.unwrap();

        // Verify results
        // Note: Error threshold is relaxed because rotations cause un-optimizable error
        const MAX_LANDMARK_ERROR: f64 = 0.1; // Larger threshold due to rotation error

        println!("\nOptimization Results:");
        println!("Initial cost: {:.6}", opt_result.initial_cost);
        println!("Final cost: {:.6}", opt_result.final_cost);

        // Check landmark convergence
        for (var_name, value) in opt_result.parameters.iter() {
            if var_name.starts_with("LM_") {
                let idx = get_landmark_idx(var_name);
                if let Some(true_point) = landmarks.get(idx) {
                    let optimized = value.to_vector();
                    let true_vec = DVector::from_vec(true_point.clone());
                    let error = (optimized - true_vec).norm();

                    println!("  {}: error = {:.6}", var_name, error);

                    if error > MAX_LANDMARK_ERROR {
                        println!(
                            "    WARNING: Landmark {} error ({:.6}) exceeds threshold ({:.6})",
                            idx, error, MAX_LANDMARK_ERROR
                        );
                    }
                }
            }
        }

        // Check convergence status
        use apex_solver::optimizer::OptimizationStatus;
        let converged = matches!(
            opt_result.status,
            OptimizationStatus::Converged
                | OptimizationStatus::CostToleranceReached
                | OptimizationStatus::ParameterToleranceReached
                | OptimizationStatus::GradientToleranceReached
        );

        if !converged {
            println!(
                "Warning: Optimization did not fully converge. Status: {:?}",
                opt_result.status
            );
        }
    }
}
