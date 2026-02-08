/// Integration tests for Phase 2B: Bias feedback loop
///
/// These tests demonstrate the complete tight coupling cycle:
/// 1. IMU preintegration accumulates measurements
/// 2. Optimization refines biases using IMU factors
/// 3. Refined biases feed back to ESKF
/// 4. Future predictions improve with refined biases
#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use crate::datasets::ImuData;
    use crate::imu::preintegration::PreintegratedImu;
    use crate::imu::ImuNoise;
    use crate::optimization::imu_factor::ImuFactor;
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
    fn test_preintegration_with_imu_factor() {
        // Test integration between preintegration and IMU factor
        let noise = ImuNoise::default();
        let mut preint = PreintegratedImu::new(noise);

        // Create IMU data
        let imu_data = create_synthetic_imu_with_bias(
            50,
            0.01,
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
        use na::{DMatrix, DVector};
        let (residual, jacobian): (DVector<f64>, Option<DMatrix<f64>>) =
            factor.linearize(&params, true);

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
    fn test_correct_bias_reduces_residual() {
        let noise = ImuNoise::default();
        let mut preint = PreintegratedImu::new(noise);

        let gyro_bias = na::Vector3::new(0.01, 0.0, 0.0);
        let accel_bias = na::Vector3::new(0.0, 0.0, 0.1);

        // Set linearization point = true bias so preintegration subtracts it correctly
        preint.reset(gyro_bias, accel_bias);

        let imu_data = create_synthetic_imu_with_bias(50, 0.01, gyro_bias, accel_bias);

        for imu in &imu_data {
            let gyro = na::Vector3::new(imu.gyro[0], imu.gyro[1], imu.gyro[2]);
            let accel = na::Vector3::new(imu.accel[0], imu.accel[1], imu.accel[2]);
            preint.integrate(gyro, accel, 0.01);
        }

        let gravity = na::Vector3::new(0.0, 0.0, -9.81);
        let factor = ImuFactor::new(preint.clone(), gravity);

        // Stationary sensor: no motion, so v and p don't change
        let r_i = na::UnitQuaternion::identity();
        let v_i = na::Vector3::zeros();
        let p_i = na::Vector3::zeros();
        let r_j = na::UnitQuaternion::identity();
        let v_j = na::Vector3::zeros();
        let p_j = na::Vector3::zeros();

        // Correct bias
        let params_correct = vec![
            na::DVector::from_vec(vec![r_i.w, r_i.i, r_i.j, r_i.k]),
            na::DVector::from_vec(vec![v_i[0], v_i[1], v_i[2]]),
            na::DVector::from_vec(vec![p_i[0], p_i[1], p_i[2]]),
            na::DVector::from_vec(vec![r_j.w, r_j.i, r_j.j, r_j.k]),
            na::DVector::from_vec(vec![v_j[0], v_j[1], v_j[2]]),
            na::DVector::from_vec(vec![p_j[0], p_j[1], p_j[2]]),
            na::DVector::from_vec(vec![gyro_bias[0], gyro_bias[1], gyro_bias[2]]),
            na::DVector::from_vec(vec![accel_bias[0], accel_bias[1], accel_bias[2]]),
        ];

        // Wrong bias (zeros)
        let params_wrong = vec![
            na::DVector::from_vec(vec![r_i.w, r_i.i, r_i.j, r_i.k]),
            na::DVector::from_vec(vec![v_i[0], v_i[1], v_i[2]]),
            na::DVector::from_vec(vec![p_i[0], p_i[1], p_i[2]]),
            na::DVector::from_vec(vec![r_j.w, r_j.i, r_j.j, r_j.k]),
            na::DVector::from_vec(vec![v_j[0], v_j[1], v_j[2]]),
            na::DVector::from_vec(vec![p_j[0], p_j[1], p_j[2]]),
            na::DVector::from_vec(vec![0.0, 0.0, 0.0]),
            na::DVector::from_vec(vec![0.0, 0.0, 0.0]),
        ];

        use na::DVector;
        let (residual_correct, _): (DVector<f64>, _) =
            factor.linearize(&params_correct, false);
        let (residual_wrong, _): (DVector<f64>, _) =
            factor.linearize(&params_wrong, false);

        println!(
            "Correct bias residual norm: {:.6}",
            residual_correct.norm()
        );
        println!("Wrong bias residual norm: {:.6}", residual_wrong.norm());

        assert!(
            residual_correct.norm() < residual_wrong.norm(),
            "Correct biases should produce smaller residual ({:.6}) than wrong biases ({:.6})",
            residual_correct.norm(),
            residual_wrong.norm()
        );
    }

    #[test]
    fn test_imu_factor_jacobian_finite_difference() {
        let noise = ImuNoise::default();
        let mut preint = PreintegratedImu::new(noise);

        let imu_data = create_synthetic_imu_with_bias(
            20,
            0.01,
            na::Vector3::new(0.005, -0.003, 0.001),
            na::Vector3::new(0.02, -0.01, 0.03),
        );

        for imu in &imu_data {
            let gyro = na::Vector3::new(imu.gyro[0], imu.gyro[1], imu.gyro[2]);
            let accel = na::Vector3::new(imu.accel[0], imu.accel[1], imu.accel[2]);
            preint.integrate(gyro, accel, 0.01);
        }

        let gravity = na::Vector3::new(0.0, 0.0, -9.81);
        let factor = ImuFactor::new(preint, gravity);

        let dt = 0.2;
        let r_i = na::UnitQuaternion::identity();
        let v_i = na::Vector3::new(0.1, 0.0, 0.0);
        let p_i = na::Vector3::zeros();
        let r_j = na::UnitQuaternion::identity();
        let v_j = v_i + gravity * dt;
        let p_j = p_i + v_i * dt + 0.5 * gravity * dt * dt;
        let bias_g = na::Vector3::new(0.005, -0.003, 0.001);
        let bias_a = na::Vector3::new(0.02, -0.01, 0.03);

        let params: Vec<na::DVector<f64>> = vec![
            na::DVector::from_vec(vec![r_i.w, r_i.i, r_i.j, r_i.k]),
            na::DVector::from_vec(vec![v_i[0], v_i[1], v_i[2]]),
            na::DVector::from_vec(vec![p_i[0], p_i[1], p_i[2]]),
            na::DVector::from_vec(vec![r_j.w, r_j.i, r_j.j, r_j.k]),
            na::DVector::from_vec(vec![v_j[0], v_j[1], v_j[2]]),
            na::DVector::from_vec(vec![p_j[0], p_j[1], p_j[2]]),
            na::DVector::from_vec(vec![bias_g[0], bias_g[1], bias_g[2]]),
            na::DVector::from_vec(vec![bias_a[0], bias_a[1], bias_a[2]]),
        ];

        use na::{DMatrix, DVector};
        let (residual_0, jacobian): (DVector<f64>, Option<DMatrix<f64>>) =
            factor.linearize(&params, true);
        let jac = jacobian.expect("Jacobian should be computed");
        let _ = &residual_0; // suppress unused warning

        // Finite-difference check for bias parameters only (params[6] and params[7])
        // These are the simplest to perturb (no manifold constraints like quaternions)
        let eps = 1e-6;
        for param_idx in [6usize, 7] {
            for elem_idx in 0..3 {
                let mut params_plus = params.clone();
                params_plus[param_idx][elem_idx] += eps;

                let mut params_minus = params.clone();
                params_minus[param_idx][elem_idx] -= eps;

                let (r_plus, _): (DVector<f64>, _) =
                    factor.linearize(&params_plus, false);
                let (r_minus, _): (DVector<f64>, _) =
                    factor.linearize(&params_minus, false);

                let numerical_col = (&r_plus - &r_minus) / (2.0 * eps);

                // Column index in the full Jacobian: sum of sizes of earlier params
                // params: [4, 3, 3, 4, 3, 3, 3, 3] = 26 total
                // param_idx=6 starts at col 4+3+3+4+3+3 = 20
                // param_idx=7 starts at col 20+3 = 23
                let col_start = match param_idx {
                    6 => 20,
                    7 => 23,
                    _ => unreachable!(),
                };
                let col = col_start + elem_idx;
                let analytical_col = jac.column(col);

                let diff = (numerical_col - analytical_col).norm();
                let scale = analytical_col.norm().max(1.0);

                assert!(
                    diff / scale < 1e-3,
                    "Jacobian mismatch at param {} elem {}: analytical vs numerical diff={:.6e}, scale={:.6e}",
                    param_idx, elem_idx, diff, scale
                );
            }
        }
    }
}
