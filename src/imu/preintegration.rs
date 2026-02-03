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
}

impl PreintegratedImu {
    /// Create new preintegration with given noise parameters
    pub fn new(noise: ImuNoise) -> Self {
        Self {
            delta_R: na::UnitQuaternion::identity(),
            delta_v: na::Vector3::zeros(),
            delta_p: na::Vector3::zeros(),
            delta_t: 0.0,
            covariance: na::SMatrix::zeros(),
            J_R_bg: na::Matrix3::zeros(),
            J_v_bg: na::Matrix3::zeros(),
            J_v_ba: na::Matrix3::identity(),
            J_p_bg: na::Matrix3::zeros(),
            J_p_ba: na::Matrix3::zeros(),
            linearization_point_bg: na::Vector3::zeros(),
            linearization_point_ba: na::Vector3::zeros(),
            noise,
        }
    }

    /// Reset preintegration
    pub fn reset(&mut self, bg: na::Vector3<f64>, ba: na::Vector3<f64>) {
        self.delta_R = na::UnitQuaternion::identity();
        self.delta_v = na::Vector3::zeros();
        self.delta_p = na::Vector3::zeros();
        self.delta_t = 0.0;
        self.covariance = na::SMatrix::zeros();
        self.J_R_bg = na::Matrix3::zeros();
        self.J_v_bg = na::Matrix3::zeros();
        self.J_v_ba = na::Matrix3::identity();
        self.J_p_bg = na::Matrix3::zeros();
        self.J_p_ba = na::Matrix3::zeros();
        self.linearization_point_bg = bg;
        self.linearization_point_ba = ba;
    }

    /// Integrate a single IMU measurement
    ///
    /// Uses midpoint integration with proper covariance propagation
    pub fn integrate(&mut self, gyro: na::Vector3<f64>, accel: na::Vector3<f64>, dt: f64) {
        if dt <= 0.0 || dt > 1.0 {
            // Skip invalid measurements
            return;
        }

        // Bias-corrected measurements
        let omega = gyro - self.linearization_point_bg;
        let acc = accel - self.linearization_point_ba;

        // Midpoint integration
        // R_{k+1} = R_k * Exp(ω * dt)
        let delta_R_k = exp_map_so3(omega * dt);
        let new_delta_R = self.delta_R * delta_R_k;

        // v_{k+1} = v_k + R_k * a * dt
        let new_delta_v = self.delta_v + self.delta_R * acc * dt;

        // p_{k+1} = p_k + v_k * dt + 0.5 * R_k * a * dt²
        let new_delta_p = self.delta_p + self.delta_v * dt + 0.5 * (self.delta_R * acc) * dt * dt;

        // Update Jacobians w.r.t. biases
        // These follow from chain rule differentiation

        // Right Jacobian of SO(3) for omega * dt
        let Jr = right_jacobian_so3(omega * dt);

        // J_{R,bg}^{k+1} = J_{R,bg}^k - Jr * dt
        self.J_R_bg = delta_R_k.to_rotation_matrix().matrix() * self.J_R_bg - Jr * dt;

        // J_{v,bg}^{k+1} = J_{v,bg}^k - R_k * [a]_× * J_{R,bg}^k * dt
        self.J_v_bg = self.J_v_bg
            - self.delta_R.to_rotation_matrix().matrix() * skew_symmetric(acc) * self.J_R_bg * dt;

        // J_{v,ba}^{k+1} = J_{v,ba}^k - R_k * dt
        self.J_v_ba = self.J_v_ba - self.delta_R.to_rotation_matrix().matrix() * dt;

        // J_{p,bg}^{k+1} = J_{p,bg}^k + J_{v,bg}^k * dt - 0.5 * R_k * [a]_× * J_{R,bg}^k * dt²
        self.J_p_bg = self.J_p_bg + self.J_v_bg * dt
            - 0.5
                * self.delta_R.to_rotation_matrix().matrix()
                * skew_symmetric(acc)
                * self.J_R_bg
                * dt
                * dt;

        // J_{p,ba}^{k+1} = J_{p,ba}^k + J_{v,ba}^k * dt - 0.5 * R_k * dt²
        self.J_p_ba = self.J_p_ba + self.J_v_ba * dt
            - 0.5 * self.delta_R.to_rotation_matrix().matrix() * dt * dt;

        // Propagate covariance
        self.propagate_covariance(omega, acc, dt);

        // Update preintegrated values
        self.delta_R = new_delta_R;
        self.delta_v = new_delta_v;
        self.delta_p = new_delta_p;
        self.delta_t += dt;
    }

    /// Propagate covariance using first-order linearization
    ///
    /// Σ_{k+1} = A * Σ_k * A^T + B * Q * B^T
    ///
    /// Note: Noise covariance is computed by multiplying noise power spectral
    /// density (PSD) by integration interval dt. PSD has units like (rad/s)²/Hz.
    fn propagate_covariance(&mut self, omega: na::Vector3<f64>, acc: na::Vector3<f64>, dt: f64) {
        // ✓ FIXED: Multiply by dt (was dividing - made filter behavior backwards)
        // Noise covariance from PSD integrated over time interval
        let gyro_cov = self.noise.gyro_noise_density.powi(2) * dt;
        let accel_cov = self.noise.accel_noise_density.powi(2) * dt;

        // Q = diag([σ_g², σ_g², σ_g², σ_a², σ_a², σ_a²])
        let mut Q = na::SMatrix::<f64, 6, 6>::zeros();
        Q.fixed_view_mut::<3, 3>(0, 0).fill_diagonal(gyro_cov);
        Q.fixed_view_mut::<3, 3>(3, 3).fill_diagonal(accel_cov);

        // State transition matrix A (9x9)
        let R_k_rot = self.delta_R.to_rotation_matrix();
        let R_k = R_k_rot.matrix();
        let Jr = right_jacobian_so3(omega * dt);

        let mut A = na::SMatrix::<f64, 9, 9>::identity();
        // Rotation block
        A.fixed_view_mut::<3, 3>(0, 0).copy_from(&Jr.transpose());

        // Velocity-rotation coupling
        A.fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(-R_k * skew_symmetric(acc) * dt));

        // Position-velocity coupling
        A.fixed_view_mut::<3, 3>(6, 3)
            .copy_from(&(na::Matrix3::identity() * dt));

        // Position-rotation coupling
        A.fixed_view_mut::<3, 3>(6, 0)
            .copy_from(&(-0.5 * R_k * skew_symmetric(acc) * dt * dt));

        // Noise gain matrix B (9x6)
        let mut B = na::SMatrix::<f64, 9, 6>::zeros();
        B.fixed_view_mut::<3, 3>(0, 0).copy_from(&Jr);
        B.fixed_view_mut::<3, 3>(3, 3).copy_from(&(-R_k * dt));
        B.fixed_view_mut::<3, 3>(6, 3)
            .copy_from(&(-0.5 * R_k * dt * dt));

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
        self.delta_R = self.delta_R * delta_R_correction;

        self.delta_v = self.delta_v + self.J_v_bg * d_bg + self.J_v_ba * d_ba;
        self.delta_p = self.delta_p + self.J_p_bg * d_bg + self.J_p_ba * d_ba;

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
        // Small angle approximation
        na::UnitQuaternion::from_scaled_axis(omega)
    } else {
        na::UnitQuaternion::from_axis_angle(&na::Unit::new_normalize(omega), theta)
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
    let W2 = W * W;

    na::Matrix3::identity() - ((1.0 - theta.cos()) / theta2) * W
        + ((theta - theta.sin()) / theta3) * W2
}

/// Skew-symmetric matrix from vector: [v]_×
pub fn skew_symmetric(v: na::Vector3<f64>) -> na::Matrix3<f64> {
    na::Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
