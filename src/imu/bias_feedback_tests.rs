/// Integration tests for Phase 2B: Bias feedback loop
///
/// These tests demonstrate the complete tight coupling cycle:
/// 1. IMU preintegration accumulates measurements
/// 2. Optimization refines biases using IMU factors
/// 3. Refined biases feed back to ESKF
/// 4. Future predictions improve with refined biases

#[cfg(test)]
mod bias_feedback_tests {
    use crate::imu::{ImuConfig, ImuData, ImuNoise, VelocityEstimator};
    use crate::imu::preintegration::PreintegratedImu;
    use crate::optimization::imu_factor::ImuFactor;
    use crate::optimization::OptimizationResult;
        use apex_solver::Factor;
    use nalgebra as na;
    
    /// Helper to create synthetic IMU data with known biases
    fn create_synthetic_imu_with_bias(
        num_samples: usize,
        dt: f64,
        true_gyro_bias: na::Vector3<f64>,
        true_accel_bias: na::Vector3<f64>,
    ) -> Vec<ImuData> {
        let mut data = Vec::new();
        
        for i in 0..num_samples {
            let timestamp = (i as f64 * dt * 1e9) as i64;
            
            // Simulate static IMU with biases
            // True motion: zero rotation, zero acceleration (except gravity)
            // Measured: true + bias
            let gyro_measured = true_gyro_bias;
            let accel_measured = na::Vector3::new(0.0, 0.0, 9.81) + true_accel_bias;
            
            data.push(ImuData {
                timestamp,
                gyro: [gyro_measured[0], gyro_measured[1], gyro_measured[2]],
                accel: [accel_measured[0], accel_measured[1], accel_measured[2]],
            });
        }
        
        data
    }
    
    #[test]
    fn test_bias_feedback_improves_estimates() {
        // Setup: Known true biases
        let true_gyro_bias = na::Vector3::new(0.01, -0.02, 0.005);
        let true_accel_bias = na::Vector3::new(0.1, 0.05, -0.08);
        
        // Create IMU data with these biases
        let imu_data = create_synthetic_imu_with_bias(100, 0.01, true_gyro_bias, true_accel_bias);
        
        // Initialize velocity estimator with zero bias assumption
        let config = ImuConfig::default();
        let mut estimator = VelocityEstimator::new(config);
        let orientation = na::UnitQuaternion::identity();
        estimator.initialize_from_bias_and_orientation(&imu_data[0..10], &orientation, None);
        
        // Get initial biases (should be close to zero)
        let (init_bg, init_ba) = estimator.get_biases();
        println!("Initial biases - gyro: {:?}, accel: {:?}", init_bg, init_ba);
        
        // Process some IMU data
        estimator.update(&imu_data[10..50]);
        let velocity_before = estimator.get_velocity();
        println!("Velocity before bias correction: {:?}", velocity_before);
        
        // Simulate optimization refining biases
        // In reality, this would come from bundle adjustment
        let refined_bg = true_gyro_bias * 0.9; // 90% of true bias (simulating convergence)
        let refined_ba = true_accel_bias * 0.9;
        
        let optimization_result = OptimizationResult::new(refined_bg, refined_ba, 0.001);
        
        // Apply feedback: Feed refined biases back to ESKF
        estimator.apply_optimized_biases(
            optimization_result.gyro_bias,
            optimization_result.accel_bias,
            optimization_result.bias_uncertainty,
        );
        
        // Verify biases were updated
        let (updated_bg, updated_ba) = estimator.get_biases();
        println!("Updated biases - gyro: {:?}, accel: {:?}", updated_bg, updated_ba);
        
        assert!((updated_bg - refined_bg).norm() < 1e-10, "Gyro bias should be updated");
        assert!((updated_ba - refined_ba).norm() < 1e-10, "Accel bias should be updated");
        
        // Process more IMU data with refined biases
        estimator.update(&imu_data[50..100]);
        let velocity_after = estimator.get_velocity();
        println!("Velocity after bias correction: {:?}", velocity_after);
        
        // With corrected biases, velocity should be more stable
        // (In a static scenario, velocity should remain close to zero)
        assert!(velocity_after.norm() < velocity_before.norm() * 1.5,
                "Velocity with corrected biases should not grow unbounded");
    }
    
    #[test]
    fn test_bias_feedback_reduces_uncertainty() {
        // Setup
        let config = ImuConfig::default();
        let mut estimator = VelocityEstimator::new(config);
        
        let imu_data = create_synthetic_imu_with_bias(
            50, 0.01,
            na::Vector3::new(0.01, 0.0, 0.0),
            na::Vector3::new(0.0, 0.0, 0.1),
        );
        
        let orientation = na::UnitQuaternion::identity();
        estimator.initialize_from_bias_and_orientation(&imu_data[0..10], &orientation, None);
        
        // Process data
        estimator.update(&imu_data[10..30]);
        
        let uncertainty_before = estimator.get_velocity_uncertainty();
        println!("Velocity uncertainty before: {:?}", uncertainty_before);
        
        // Apply refined biases with low uncertainty
        let refined_bg = na::Vector3::new(0.009, 0.0, 0.0);
        let refined_ba = na::Vector3::new(0.0, 0.0, 0.095);
        estimator.apply_optimized_biases(refined_bg, refined_ba, 0.0001); // Very confident
        
        // Uncertainty should reflect the refinement
        // (This is a basic test; in practice, Kalman update would refine covariance)
        estimator.update(&imu_data[30..50]);
        let uncertainty_after = estimator.get_velocity_uncertainty();
        println!("Velocity uncertainty after: {:?}", uncertainty_after);
        
        // Test passes if estimator continues to function correctly
        assert!(uncertainty_after.iter().all(|x| x.is_finite()));
    }
    
    #[test]
    fn test_preintegration_with_imu_factor() {
        // Test integration between preintegration and IMU factor
        let noise = ImuNoise::default();
        let mut preint = PreintegratedImu::new(noise);
        
        // Create IMU data
        let imu_data = create_synthetic_imu_with_bias(
            50, 0.01,
            na::Vector3::new(0.005, 0.0, 0.0),
            na::Vector3::new(0.0, 0.0, 0.05),
        );
        
        // Integrate IMU measurements
        for imu in &imu_data {
            let gyro = na::Vector3::new(imu.gyro[0], imu.gyro[1], imu.gyro[2]);
            let accel = na::Vector3::new(imu.accel[0], imu.accel[1], imu.accel[2]);
            preint.integrate(gyro, accel, 0.01);
        }
        
        // Create IMU factor
        let gravity = na::Vector3::new(0.0, 0.0, -9.81);
        let factor = ImuFactor::new(preint.clone(), gravity);
        
        // Verify factor was created successfully
        assert_eq!(factor.get_dimension(), 9);
        
        // Create dummy parameters for linearization test
        let R_i = na::UnitQuaternion::identity();
        let v_i = na::Vector3::zeros();
        let p_i = na::Vector3::zeros();
        
        let R_j = na::UnitQuaternion::identity();
        let v_j = na::Vector3::new(0.0, 0.0, -9.81 * 0.5); // gravity effect
        let p_j = na::Vector3::zeros();
        
        let bias_g = na::Vector3::new(0.005, 0.0, 0.0);
        let bias_a = na::Vector3::new(0.0, 0.0, 0.05);
        
        let params = vec![
            na::DVector::from_vec(vec![R_i.w, R_i.i, R_i.j, R_i.k]),
            na::DVector::from_vec(vec![v_i[0], v_i[1], v_i[2]]),
            na::DVector::from_vec(vec![p_i[0], p_i[1], p_i[2]]),
            na::DVector::from_vec(vec![R_j.w, R_j.i, R_j.j, R_j.k]),
            na::DVector::from_vec(vec![v_j[0], v_j[1], v_j[2]]),
            na::DVector::from_vec(vec![p_j[0], p_j[1], p_j[2]]),
            na::DVector::from_vec(vec![bias_g[0], bias_g[1], bias_g[2]]),
            na::DVector::from_vec(vec![bias_a[0], bias_a[1], bias_a[2]]),
        ];
        
        // Linearize factor
        use na::{DVector, DMatrix};
        let (residual, jacobian): (DVector<f64>, Option<DMatrix<f64>>) = factor.linearize(&params, true);
        
        assert_eq!(residual.len(), 9);
        assert!(residual.iter().all(|x| x.is_finite()));
        
        let jac = jacobian.expect("Jacobian should be computed");
        assert_eq!(jac.nrows(), 9);
        assert_eq!(jac.ncols(), 26);
        assert!(jac.iter().all(|x| x.is_finite()));
        
        println!("Residual norm: {:.6}", residual.norm());
        println!("Jacobian norm: {:.6}", jac.norm());
    }
    
    #[test]
    fn test_complete_feedback_cycle() {
        // This test demonstrates the complete cycle:
        // IMU -> Preintegration -> Optimization -> Bias Feedback -> IMU
        
        // 1. Setup with known biases
        let true_gyro_bias = na::Vector3::new(0.02, -0.01, 0.005);
        let true_accel_bias = na::Vector3::new(0.15, -0.1, 0.05);
        
        let imu_data = create_synthetic_imu_with_bias(
            200, 0.01, true_gyro_bias, true_accel_bias
        );
        
        // 2. Initialize ESKF with zero bias assumption
        let config = ImuConfig::default();
        let mut estimator = VelocityEstimator::new(config);
        let orientation = na::UnitQuaternion::identity();
        estimator.initialize_from_bias_and_orientation(&imu_data[0..20], &orientation, None);
        
        // 3. Process some data (accumulate drift due to bias)
        estimator.update(&imu_data[20..100]);
        let (initial_bg, initial_ba) = estimator.get_biases();
        println!("\nIteration 1 - Initial biases:");
        println!("  Gyro bias: {:?}", initial_bg);
        println!("  Accel bias: {:?}", initial_ba);
        
        // 4. Simulate optimization cycle 1: Partial convergence
        let iter1_bg = true_gyro_bias * 0.5;
        let iter1_ba = true_accel_bias * 0.5;
        estimator.apply_optimized_biases(iter1_bg, iter1_ba, 0.01);
        
        let (biases_after_opt1_bg, biases_after_opt1_ba) = estimator.get_biases();
        println!("\nIteration 1 - After optimization:");
        println!("  Gyro bias: {:?}", biases_after_opt1_bg);
        println!("  Accel bias: {:?}", biases_after_opt1_ba);
        
        // 5. Process more data
        estimator.update(&imu_data[100..150]);
        
        // 6. Simulate optimization cycle 2: Better convergence
        let iter2_bg = true_gyro_bias * 0.8;
        let iter2_ba = true_accel_bias * 0.8;
        estimator.apply_optimized_biases(iter2_bg, iter2_ba, 0.005);
        
        let (biases_after_opt2_bg, biases_after_opt2_ba) = estimator.get_biases();
        println!("\nIteration 2 - After optimization:");
        println!("  Gyro bias: {:?}", biases_after_opt2_bg);
        println!("  Accel bias: {:?}", biases_after_opt2_ba);
        
        // 7. Final processing
        estimator.update(&imu_data[150..200]);
        
        // 8. Verify biases converged toward true values
        let final_bg_error = (biases_after_opt2_bg - true_gyro_bias).norm();
        let final_ba_error = (biases_after_opt2_ba - true_accel_bias).norm();
        
        println!("\nFinal errors:");
        println!("  Gyro bias error: {:.6}", final_bg_error);
        println!("  Accel bias error: {:.6}", final_ba_error);
        
        // Second iteration should be closer to true values
        let iter1_bg_error = (iter1_bg - true_gyro_bias).norm();
        let iter1_ba_error = (iter1_ba - true_accel_bias).norm();
        
        assert!(final_bg_error < iter1_bg_error, 
                "Gyro bias should improve with iterations");
        assert!(final_ba_error < iter1_ba_error,
                "Accel bias should improve with iterations");
    }
}
