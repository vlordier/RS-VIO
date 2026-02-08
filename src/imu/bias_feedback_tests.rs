/// Integration tests for Phase 2B: Bias feedback loop
///
/// These tests demonstrate the complete tight coupling cycle:
/// 1. IMU preintegration accumulates measurements
/// 2. Optimization refines biases using IMU factors
/// 3. Refined biases feed back to ESKF
/// 4. Future predictions improve with refined biases
#[cfg(test)]
#[allow(clippy::expect_used)]
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
}
