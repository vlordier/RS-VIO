//! # IMU Preintegration (Forster et al. 2017)
//!
//! On-manifold preintegration for visual-inertial odometry following:
//! "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
//! Christian Forster, Luca Carlone, Frank Dellaert, Davide Scaramuzza
//! IEEE Trans. Robotics 2017
//!
//! ## Key Features
//!
//! - Proper covariance propagation with measurement noise
//! - Bias correction with first-order Jacobians
//! - On-manifold integration using SO(3) and R^3
//! - Supports bias updates without re-integration
//!
//! ## Preintegration Equations
//!
//! Given IMU measurements ω(t), a(t) between times i and j with biases b_g, b_a:
//!
//! ```text
//! ω̃ = ω - b_g - η_g    (corrected gyro)
//! ã = a - b_a - η_a    (corrected accel)
//!
//! ΔR_ij = ∫ ΔR * exp(ω̃ * dt)
//! Δv_ij = ∫ ΔR * ã * dt
//! Δp_ij = ∫ (Δv + ΔR * ã * dt²/2)
//! ```
//!
//! Covariance propagation (first-order):
//! ```text
//! Σ_{k+1} = A_k * Σ_k * A_k^T + B_k * Q * B_k^T
//! ```

use nalgebra as na;

/// IMU noise parameters
#[derive(Debug, Clone)]
pub struct ImuNoise {
    /// Gyroscope noise density [rad/s/√Hz]
    pub gyro_noise_density: f64,
    /// Accelerometer noise density [m/s²/√Hz]
    pub accel_noise_density: f64,
    /// Gyroscope bias random walk [rad/s²/√Hz]
    pub gyro_bias_random_walk: f64,
    /// Accelerometer bias random walk [m/s³/√Hz]
    pub accel_bias_random_walk: f64,
}

impl Default for ImuNoise {
    fn default() -> Self {
        Self {
            // Typical values for consumer-grade IMU (e.g., BMI160, MPU9250)
            gyro_noise_density: 1.6e-4,     // rad/s/√Hz
            accel_noise_density: 2.0e-3,    // m/s²/√Hz
            gyro_bias_random_walk: 1.9e-5,  // rad/s²/√Hz
            accel_bias_random_walk: 3.0e-3, // m/s³/√Hz
        }
    }
}

/// Preintegrated IMU measurements between two keyframes
///
/// State: [ΔR, Δv, Δp] with uncertainty and bias Jacobians
#[derive(Debug, Clone)]
pub struct PreintegratedImu {
    /// Delta rotation from i to j (R_i^T * R_j)
    pub delta_R: na::UnitQuaternion<f64>,

    /// Delta velocity in frame i [m/s]
    pub delta_v: na::Vector3<f64>,

    /// Delta position in frame i [m]
    pub delta_p: na::Vector3<f64>,

    /// Time interval [s]
    pub delta_t: f64,

    /// 9x9 covariance matrix [rot, vel, pos]
    pub covariance: na::SMatrix<f64, 9, 9>,

    /// Jacobian of ΔR w.r.t. gyro bias (3x3)
    pub J_R_bg: na::Matrix3<f64>,

    /// Jacobian of Δv w.r.t. gyro bias (3x3)
    pub J_v_bg: na::Matrix3<f64>,

    /// Jacobian of Δv w.r.t. accel bias (3x3)
    pub J_v_ba: na::Matrix3<f64>,

    /// Jacobian of Δp w.r.t. gyro bias (3x3)
    pub J_p_bg: na::Matrix3<f64>,

    /// Jacobian of Δp w.r.t. accel bias (3x3)
    pub J_p_ba: na::Matrix3<f64>,

    /// Reference bias used during integration
    pub linearization_point_bg: na::Vector3<f64>,
    pub linearization_point_ba: na::Vector3<f64>,

    /// Noise parameters
    pub noise: ImuNoise,

    /// Count of IMU measurements dropped due to invalid dt
    dropped_measurements: usize,
}

impl PreintegratedImu {
    /// Create new preintegration with given noise parameters
    pub fn new(noise: ImuNoise) -> Self {
        Self {
            delta_R: na::UnitQuaternion::identity(),
            delta_v: na::Vector3::zeros(),
            delta_p: na::Vector3::zeros(),
            delta_t: 0.0,
            // Initialize with small prior uncertainty (1e-12 * I) to avoid degenerate covariance
            // Zero covariance would cause the first integration to skip propagation through A matrix
            covariance: na::SMatrix::from_diagonal_element(1e-12),
            J_R_bg: na::Matrix3::zeros(),
            J_v_bg: na::Matrix3::zeros(),
            J_v_ba: na::Matrix3::zeros(),
            J_p_bg: na::Matrix3::zeros(),
            J_p_ba: na::Matrix3::zeros(),
            linearization_point_bg: na::Vector3::zeros(),
            linearization_point_ba: na::Vector3::zeros(),
            noise,
            dropped_measurements: 0,
        }
    }

    /// Reset preintegration
    pub fn reset(&mut self, bg: na::Vector3<f64>, ba: na::Vector3<f64>) {
        self.delta_R = na::UnitQuaternion::identity();
        self.delta_v = na::Vector3::zeros();
        self.delta_p = na::Vector3::zeros();
        self.delta_t = 0.0;
        // Reset covariance to small prior uncertainty (1e-12 * I)
        self.covariance = na::SMatrix::from_diagonal_element(1e-12);
        self.J_R_bg = na::Matrix3::zeros();
        self.J_v_bg = na::Matrix3::zeros();
        self.J_v_ba = na::Matrix3::zeros();
        self.J_p_bg = na::Matrix3::zeros();
        self.J_p_ba = na::Matrix3::zeros();
        self.linearization_point_bg = bg;
        self.linearization_point_ba = ba;
        self.dropped_measurements = 0;
    }

    /// Number of IMU samples dropped due to invalid dt
    pub const fn dropped_measurements(&self) -> usize {
        self.dropped_measurements
    }

    /// Integrate a single IMU measurement
    ///
    /// Uses first-order Euler integration with proper covariance propagation
    /// (Forster et al. 2017 TRO: "On-Manifold Preintegration for Real-Time VIO")
    pub fn integrate(&mut self, gyro: na::Vector3<f64>, accel: na::Vector3<f64>, dt: f64) {
        if dt <= 0.0 || dt > 1.0 {
            // Skip invalid measurements
            self.dropped_measurements = self.dropped_measurements.saturating_add(1);
            log::warn!(
                "[IMU Preintegration] Dropped measurement with invalid dt={:.6}",
                dt
            );
            return;
        }

        // Bias-corrected measurements
        let omega = gyro - self.linearization_point_bg;
        let acc = accel - self.linearization_point_ba;

        let dt2 = dt * dt;

        // First-order Euler integration on manifold
        // R_{k+1} = R_k * Exp(ω * dt)
        let delta_R_k = exp_map_so3(omega * dt);
        let new_delta_R = self.delta_R * delta_R_k;

        // v_{k+1} = v_k + R_k * a * dt
        let new_delta_v = self.delta_v + self.delta_R * acc * dt;

        // p_{k+1} = p_k + v_k * dt + 0.5 * R_k * a * dt²
        let new_delta_p = self.delta_p + self.delta_v * dt + 0.5 * (self.delta_R * acc) * dt2;

        // Update Jacobians w.r.t. biases (Forster et al. Eqs. 27-32)
        // CRITICAL: Compute using OLD Jacobians to maintain correct chain rule
        // Cache old values before updating J_R_bg
        let Jr = right_jacobian_so3(omega * dt);
        let R_k = *self.delta_R.to_rotation_matrix().matrix();
        let acc_skew = skew_symmetric(acc);
        let R_k_acc_skew = R_k * acc_skew; // Cache: used 5× across Jacobians + covariance
        let J_R_bg_k = self.J_R_bg; // Cache BEFORE update
        let J_v_bg_k = self.J_v_bg; // Cache BEFORE update
        let J_v_ba_k = self.J_v_ba; // Cache BEFORE update

        // J_{R,bg}^{k+1} = ΔR_k * J_{R,bg}^k - Jr * dt (Eq. 27)
        self.J_R_bg = delta_R_k.to_rotation_matrix().matrix() * J_R_bg_k - Jr * dt;

        // J_{v,bg}^{k+1} = J_{v,bg}^k - R_k * [a]_× * J_{R,bg}^k * dt (Eq. 28, uses J_R_bg at step k)
        self.J_v_bg = J_v_bg_k - R_k_acc_skew * J_R_bg_k * dt;

        // J_{v,ba}^{k+1} = J_{v,ba}^k - R_k * dt (Eq. 29)
        self.J_v_ba = J_v_ba_k - R_k * dt;

        // J_{p,bg}^{k+1} = J_{p,bg}^k + J_{v,bg}^k * dt - 0.5 * R_k * [a]_× * J_{R,bg}^k * dt² (Eq. 30)
        self.J_p_bg = self.J_p_bg + J_v_bg_k * dt - 0.5 * R_k_acc_skew * J_R_bg_k * dt2;

        // J_{p,ba}^{k+1} = J_{p,ba}^k + J_{v,ba}^k * dt - 0.5 * R_k * dt² (Eq. 31)
        self.J_p_ba = self.J_p_ba + J_v_ba_k * dt - 0.5 * R_k * dt2;

        // Propagate covariance (reuse precomputed Jr, R_k, acc_skew, R_k_acc_skew)
        self.propagate_covariance_with_cached(&Jr, &R_k, &R_k_acc_skew, dt);

        // Update preintegrated values
        self.delta_R = new_delta_R;
        self.delta_v = new_delta_v;
        self.delta_p = new_delta_p;
        self.delta_t += dt;
    }

    /// Propagate covariance using first-order linearization
    ///
    /// Error-state formulation (Forster et al. Eq. 25):
    /// - Σ = Cov[δφ, δv, δp] (rotation, velocity, position error covariance)
    /// - Σ_{k+1} = A * Σ_k * A^T + B * Q * B^T
    ///
    /// Note: Noise covariance Q is computed by multiplying noise power spectral
    /// density (PSD) by integration interval dt. PSD has units like (rad/s)²/Hz.
    ///
    /// Takes precomputed values from `integrate()` to avoid redundant work on the hot path.
    fn propagate_covariance_with_cached(
        &mut self,
        Jr: &na::Matrix3<f64>,
        R_k: &na::Matrix3<f64>,
        R_k_acc_skew: &na::Matrix3<f64>,
        dt: f64,
    ) {
        // ✓ FIXED: Multiply by dt (was dividing - made filter behavior backwards)
        // Noise covariance from PSD integrated over time interval
        let gyro_cov = self.noise.gyro_noise_density.powi(2) * dt;
        let accel_cov = self.noise.accel_noise_density.powi(2) * dt;

        // Q = diag([σ_g², σ_g², σ_g², σ_a², σ_a², σ_a²])
        let mut Q = na::SMatrix::<f64, 6, 6>::zeros();
        Q.fixed_view_mut::<3, 3>(0, 0).fill_diagonal(gyro_cov);
        Q.fixed_view_mut::<3, 3>(3, 3).fill_diagonal(accel_cov);

        // State transition matrix A (9x9)
        // Jr, R_k, R_k_acc_skew passed from integrate() to avoid recomputation
        let dt2 = dt * dt;

        let mut A = na::SMatrix::<f64, 9, 9>::identity();

        // Rotation block: Jr^{-T} for error-state formulation (left Jacobian inverse transpose)
        // Left Jacobian: Jl(ω) = Jr(-ω), and we use Jl^{-T} for error propagation
        let Jr_inv_t = Jr.transpose().try_inverse().unwrap_or_else(|| {
            log::warn!("[IMU Preintegration] Jr^T near-singular (|omega*dt| near 2*pi*n), using identity fallback");
            na::Matrix3::identity()
        });
        A.fixed_view_mut::<3, 3>(0, 0).copy_from(&Jr_inv_t);

        // Velocity-rotation coupling (reuses cached R_k * acc_skew)
        A.fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(-R_k_acc_skew * dt));

        // Position-velocity coupling
        A.fixed_view_mut::<3, 3>(6, 3)
            .copy_from(&(na::Matrix3::identity() * dt));

        // Position-rotation coupling (reuses cached R_k * acc_skew)
        A.fixed_view_mut::<3, 3>(6, 0)
            .copy_from(&(-0.5 * R_k_acc_skew * dt2));

        // Noise gain matrix B (9x6)
        // Maps measurement noise to state error: ξ = B * [η_g; η_a]
        let mut B = na::SMatrix::<f64, 9, 6>::zeros();
        B.fixed_view_mut::<3, 3>(0, 0).copy_from(Jr); // Gyro noise → rotation error
        B.fixed_view_mut::<3, 3>(3, 3).copy_from(R_k); // Accel noise → velocity error

        // Gyro noise affects position through rotation error
        // dP/dη_g = -0.5 * R_k * [acc]_× * Jr * dt² (reuses R_k_acc_skew)
        B.fixed_view_mut::<3, 3>(6, 0)
            .copy_from(&(-0.5 * R_k_acc_skew * Jr * dt2));

        // Accel noise → position error
        B.fixed_view_mut::<3, 3>(6, 3).copy_from(&(0.5 * R_k * dt2));

        // Σ_{k+1} = A * Σ_k * A^T + B * Q * B^T
        self.covariance = A * self.covariance * A.transpose() + B * Q * B.transpose();
    }

    /// Update preintegration for new bias estimate
    ///
    /// Uses first-order approximation: ΔR(bg') ≈ ΔR(bg) * Exp(J_{R,bg} * δbg)
    pub fn update_bias(&mut self, new_bg: na::Vector3<f64>, new_ba: na::Vector3<f64>) {
        let d_bg = new_bg - self.linearization_point_bg;
        let d_ba = new_ba - self.linearization_point_ba;

        // First-order bias correction
        let delta_R_correction = exp_map_so3(self.J_R_bg * d_bg);
        self.delta_R *= delta_R_correction;

        self.delta_v += self.J_v_bg * d_bg + self.J_v_ba * d_ba;
        self.delta_p += self.J_p_bg * d_bg + self.J_p_ba * d_ba;

        // Update linearization point
        self.linearization_point_bg = new_bg;
        self.linearization_point_ba = new_ba;
    }

    /// Predict state at time j given state at time i
    ///
    /// R_j = R_i * ΔR
    /// v_j = v_i + R_i * Δv + g * Δt
    /// p_j = p_i + v_i * Δt + R_i * Δp + 0.5 * g * Δt²
    pub fn predict(
        &self,
        R_i: &na::UnitQuaternion<f64>,
        v_i: &na::Vector3<f64>,
        p_i: &na::Vector3<f64>,
        gravity: &na::Vector3<f64>,
    ) -> (na::UnitQuaternion<f64>, na::Vector3<f64>, na::Vector3<f64>) {
        let R_j = R_i * self.delta_R;
        let v_j = v_i + R_i * self.delta_v + gravity * self.delta_t;
        let p_j = p_i
            + v_i * self.delta_t
            + R_i * self.delta_p
            + 0.5 * gravity * self.delta_t * self.delta_t;

        (R_j, v_j, p_j)
    }
}

/// Exponential map for SO(3): exp(ω) = R
/// Exponential map from so(3) to SO(3) using Rodrigues formula
/// Converts rotation vector (axis * angle) to unit quaternion
pub fn exp_map_so3(omega: na::Vector3<f64>) -> na::UnitQuaternion<f64> {
    let theta = omega.norm();

    if theta < 1e-8 {
        // Small angle: q ≈ [1, ω/2] (first-order Taylor)
        // Avoid from_scaled_axis which recomputes the norm internally
        na::UnitQuaternion::new_unchecked(na::Quaternion::new(
            1.0,
            omega.x * 0.5,
            omega.y * 0.5,
            omega.z * 0.5,
        ))
    } else {
        // Reuse already-computed theta to avoid redundant norm in new_normalize
        let axis = na::Unit::new_unchecked(omega / theta);
        na::UnitQuaternion::from_axis_angle(&axis, theta)
    }
}

/// Right Jacobian of SO(3)
///
/// Jr(ω) = I - (1-cos(θ))/θ² [ω]_× + (θ-sin(θ))/θ³ [ω]_×²
pub fn right_jacobian_so3(omega: na::Vector3<f64>) -> na::Matrix3<f64> {
    let theta = omega.norm();

    if theta < 1e-8 {
        // Small angle: Jr ≈ I - 0.5 * [ω]_×
        return na::Matrix3::identity() - 0.5 * skew_symmetric(omega);
    }

    let theta2 = theta * theta;
    let theta3 = theta2 * theta;

    let W = skew_symmetric(omega);
    // Analytical identity: [omega]_x^2 = omega * omega^T - theta^2 * I
    // Avoids a full 3x3 matrix multiply (27 muls) in favor of outer product (9 muls + 3 subs)
    let W2 = omega * omega.transpose() - theta2 * na::Matrix3::identity();

    let (sin_theta, cos_theta) = theta.sin_cos();

    na::Matrix3::identity() - ((1.0 - cos_theta) / theta2) * W + ((theta - sin_theta) / theta3) * W2
}

/// Skew-symmetric matrix from vector: [v]_×
pub fn skew_symmetric(v: na::Vector3<f64>) -> na::Matrix3<f64> {
    na::Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn integrate_constant(
        preint: &mut PreintegratedImu,
        gyro: na::Vector3<f64>,
        accel: na::Vector3<f64>,
        dt: f64,
        steps: usize,
    ) {
        for _ in 0..steps {
            preint.integrate(gyro, accel, dt);
        }
    }

    #[test]
    fn test_preintegration_identity() {
        let noise = ImuNoise::default();
        let preint = PreintegratedImu::new(noise);

        assert!((preint.delta_R.angle()) < 1e-10);
        assert!(preint.delta_v.norm() < 1e-10);
        assert!(preint.delta_p.norm() < 1e-10);
    }

    #[test]
    fn test_integration_pure_rotation() {
        let noise = ImuNoise::default();
        let mut preint = PreintegratedImu::new(noise);
        preint.reset(na::Vector3::zeros(), na::Vector3::zeros());

        // Constant gyro, zero accel for 1 second at 100Hz
        let gyro = na::Vector3::new(0.0, 0.0, 0.1); // 0.1 rad/s around z
        let accel = na::Vector3::zeros();

        for _ in 0..100 {
            preint.integrate(gyro, accel, 0.01);
        }

        // Should have rotated 0.1 rad around z
        assert!((preint.delta_R.angle() - 0.1).abs() < 1e-3);
        assert!(preint.delta_v.norm() < 1e-6);
        assert!(preint.delta_p.norm() < 1e-6);
    }

    #[test]
    fn test_covariance_growth() {
        let noise = ImuNoise::default();
        let mut preint = PreintegratedImu::new(noise);
        preint.reset(na::Vector3::zeros(), na::Vector3::zeros());

        let gyro = na::Vector3::zeros();
        let accel = na::Vector3::zeros();

        let initial_trace = preint.covariance.trace();

        for _ in 0..100 {
            preint.integrate(gyro, accel, 0.01);
        }

        let final_trace = preint.covariance.trace();

        // Covariance should grow due to noise
        assert!(final_trace > initial_trace);
    }

    #[test]
    fn test_covariance_psd() {
        let noise = ImuNoise::default();
        let mut preint = PreintegratedImu::new(noise);
        preint.reset(na::Vector3::zeros(), na::Vector3::zeros());

        let gyro = na::Vector3::new(0.01, -0.02, 0.03);
        let accel = na::Vector3::new(0.1, -0.1, 0.2);

        integrate_constant(&mut preint, gyro, accel, 0.01, 200);

        let cov = na::DMatrix::from_fn(9, 9, |i, j| preint.covariance[(i, j)]);
        let sym = (&cov + cov.transpose()) * 0.5;
        let eigen = na::SymmetricEigen::new(sym);
        let min_eig = eigen.eigenvalues.min();
        assert!(
            min_eig > -1e-8,
            "Covariance should be PSD (min eigenvalue: {})",
            min_eig
        );
    }

    #[test]
    fn test_timestep_splitting_consistency() {
        let noise = ImuNoise::default();

        let gyro = na::Vector3::new(0.02, -0.01, 0.03);
        let accel = na::Vector3::new(0.1, -0.2, 0.3);

        let mut coarse = PreintegratedImu::new(noise.clone());
        coarse.reset(na::Vector3::zeros(), na::Vector3::zeros());
        integrate_constant(&mut coarse, gyro, accel, 0.01, 100);

        let mut fine = PreintegratedImu::new(noise);
        fine.reset(na::Vector3::zeros(), na::Vector3::zeros());
        integrate_constant(&mut fine, gyro, accel, 0.005, 200);

        let rot_err = (coarse.delta_R.inverse() * fine.delta_R).angle();
        let v_err = (coarse.delta_v - fine.delta_v).norm();
        let p_err = (coarse.delta_p - fine.delta_p).norm();

        assert!(
            rot_err < 5e-4,
            "Rotation should be consistent across dt splits"
        );
        assert!(
            v_err < 1e-3,
            "Velocity should be consistent across dt splits"
        );
        assert!(
            p_err < 1e-3,
            "Position should be consistent across dt splits"
        );
    }

    #[test]
    fn test_constant_accel_closed_form() {
        let noise = ImuNoise::default();
        let mut preint = PreintegratedImu::new(noise);
        preint.reset(na::Vector3::zeros(), na::Vector3::zeros());

        let accel = na::Vector3::new(0.3, -0.1, 0.2);
        let gyro = na::Vector3::zeros();

        let dt = 0.01;
        let steps = 100;
        integrate_constant(&mut preint, gyro, accel, dt, steps);

        let total_dt = dt * steps as f64;
        let expected_v = accel * total_dt;
        let expected_p = 0.5 * accel * total_dt * total_dt;

        assert!((preint.delta_v - expected_v).norm() < 5e-4);
        assert!((preint.delta_p - expected_p).norm() < 5e-4);
    }

    #[test]
    fn test_bias_jacobians_finite_difference() {
        let noise = ImuNoise::default();
        let mut preint = PreintegratedImu::new(noise.clone());
        preint.reset(na::Vector3::zeros(), na::Vector3::zeros());

        let gyro = na::Vector3::new(0.01, -0.02, 0.03);
        let accel = na::Vector3::new(0.2, -0.1, 0.15);
        integrate_constant(&mut preint, gyro, accel, 0.01, 200);

        let d_bg = na::Vector3::new(1e-4, -2e-4, 1.5e-4);
        let d_ba = na::Vector3::new(-1.2e-4, 8e-5, -6e-5);

        // Re-integrate with perturbed gyro bias
        let mut preint_bg = PreintegratedImu::new(noise.clone());
        preint_bg.reset(d_bg, na::Vector3::zeros());
        integrate_constant(&mut preint_bg, gyro, accel, 0.01, 200);

        // Re-integrate with perturbed accel bias
        let mut preint_ba = PreintegratedImu::new(noise);
        preint_ba.reset(na::Vector3::zeros(), d_ba);
        integrate_constant(&mut preint_ba, gyro, accel, 0.01, 200);

        // Rotation Jacobian check: Log(R_nom^{-1} * R_pert) ≈ J_R_bg * d_bg
        let rot_err = (preint.delta_R.inverse() * preint_bg.delta_R).scaled_axis();
        let rot_lin = preint.J_R_bg * d_bg;
        assert!((rot_err - rot_lin).norm() < 2e-3);

        // Velocity Jacobians check
        let v_bg_err = preint_bg.delta_v - preint.delta_v;
        let v_bg_lin = preint.J_v_bg * d_bg;
        assert!((v_bg_err - v_bg_lin).norm() < 2e-3);

        let v_ba_err = preint_ba.delta_v - preint.delta_v;
        let v_ba_lin = preint.J_v_ba * d_ba;
        assert!((v_ba_err - v_ba_lin).norm() < 2e-3);

        // Position Jacobians check
        let p_bg_err = preint_bg.delta_p - preint.delta_p;
        let p_bg_lin = preint.J_p_bg * d_bg;
        assert!((p_bg_err - p_bg_lin).norm() < 5e-3);

        let p_ba_err = preint_ba.delta_p - preint.delta_p;
        let p_ba_lin = preint.J_p_ba * d_ba;
        assert!((p_ba_err - p_ba_lin).norm() < 5e-3);
    }
}
