//! IMU preintegration with full covariance propagation and analytical Jacobians.
//!
//! Implements the on-manifold preintegration theory from:
//!   Forster, Carl, et al. "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry."
//!   IEEE Transactions on Robotics (T-RO), 2017.
//!
//! The key insight: integrate IMU measurements between two keyframes *once* at measurement
//! time, then reuse the preintegrated result as a factor in bundle adjustment. This avoids
//! re-integrating at every optimization iteration.
//!
//! ## Preintegrated measurements
//!
//! Given IMU samples between keyframes i-1 and i:
//! - `ΔR` : relative rotation (SO(3))
//! - `Δv` : relative velocity (R3, in frame i-1)
//! - `Δp` : relative position (R3, in frame i-1)
//!
//! ## Jacobians w.r.t. biases
//! - `∂ΔR/∂b_g` : 3×3 matrix (rotation change per gyro bias unit)
//! - `∂Δv/∂b_g`, `∂Δv/∂b_a` : 3×3 matrices
//! - `∂Δp/∂b_g`, `∂Δp/∂b_a` : 3×3 matrices
//!
//! ## Covariance propagation
//! The covariance Σ is propagated incrementally and captures the uncertainty
//! accumulated during preintegration. The information matrix used in the factor
//! is Σ⁻¹.

use nalgebra::{Matrix3, Matrix6, SMatrix, Vector3 as NaVector3};
use crate::constants::tolerance;
use crate::datasets::ImuData;

/// Helper type aliases for fixed-size matrices used in preintegration.
type Matrix9<T> = SMatrix<T, 9, 9>;
type Matrix9x6<T> = SMatrix<T, 9, 6>;
pub type Matrix15<T> = SMatrix<T, 15, 15>;
pub type Matrix15x3<T> = SMatrix<T, 15, 3>;
pub type Matrix15x6<T> = SMatrix<T, 15, 6>;

/// SO(3) exponential map: rotation vector → rotation matrix via Rodrigues' formula.
#[inline]
pub fn exp_so3(r: &NaVector3<f64>) -> Matrix3<f64> {
    let theta = r.norm();
    if theta < tolerance::ROTATION_ZERO {
        return Matrix3::identity();
    }
    let axis = r / theta;
    let (sa, ca) = theta.sin_cos();
    let ia = 1.0 - ca;
    let [x, y, z] = [axis.x, axis.y, axis.z];
    Matrix3::new(
        ca + x * x * ia,
        x * y * ia - z * sa,
        x * z * ia + y * sa,
        y * x * ia + z * sa,
        ca + y * y * ia,
        y * z * ia - x * sa,
        z * x * ia - y * sa,
        z * y * ia + x * sa,
        ca + z * z * ia,
    )
}

/// SO(3) logarithmic map: rotation matrix → rotation vector.
#[inline]
pub fn log_so3(r: &Matrix3<f64>) -> NaVector3<f64> {
    let cos_angle = ((r.trace() - 1.0) * 0.5).clamp(-1.0, 1.0);
    let angle = cos_angle.acos();
    if angle.abs() < tolerance::ROTATION_ZERO {
        return NaVector3::zeros();
    }
    let factor = angle / (2.0 * angle.sin());
    NaVector3::new(
        factor * (r[(2, 1)] - r[(1, 2)]),
        factor * (r[(0, 2)] - r[(2, 0)]),
        factor * (r[(1, 0)] - r[(0, 1)]),
    )
}

/// Right Jacobian of SO(3): J_r(φ) = I - (1-cos(θ))/θ² [φ]× + (θ-sin(θ))/θ³ [φ]×²
///
/// Used to map perturbations in the Lie algebra to the group manifold.
#[inline]
fn right_jacobian_so3(phi: &NaVector3<f64>) -> Matrix3<f64> {
    let theta = phi.norm();
    if theta < tolerance::ROTATION_ZERO {
        return Matrix3::identity();
    }
    let axis = phi / theta;
    let [x, y, z] = [axis.x, axis.y, axis.z];
    let sk = Matrix3::new(0.0, -z, y, z, 0.0, -x, -y, x, 0.0);
    let sk2 = sk * sk;
    let (sa, ca) = theta.sin_cos();
    Matrix3::identity() - ((1.0 - ca) / theta) * sk + ((theta - sa) / theta) * sk2
}

/// Inverse of the right Jacobian of SO(3).
///
/// Computed via the series expansion for numerical stability.
/// (Currently unused but kept for future Jacobian computations)
#[inline]
#[allow(dead_code)]
fn right_jacobian_inv_so3(phi: &NaVector3<f64>) -> Matrix3<f64> {
    let theta = phi.norm();
    if theta < tolerance::ROTATION_ZERO {
        return Matrix3::identity();
    }
    let axis = phi / theta;
    let [x, y, z] = [axis.x, axis.y, axis.z];
    let sk = Matrix3::new(0.0, -z, y, z, 0.0, -x, -y, x, 0.0);
    let half_theta = theta / 2.0;
    let cot = half_theta / half_theta.tan();
    Matrix3::identity() - 0.5 * sk + (1.0 - theta * cot / (2.0 * half_theta)) * sk * sk
}

/// Result of IMU preintegration: preintegrated measurements, Jacobians, and covariance.
#[derive(Debug, Clone)]
pub struct PreintegratedImuMeasurements {
    /// Preintegrated relative rotation (in frame i-1).
    pub delta_r: Matrix3<f64>,
    /// Preintegrated relative velocity (in frame i-1).
    pub delta_v: NaVector3<f64>,
    /// Preintegrated relative position (in frame i-1).
    pub delta_p: NaVector3<f64>,
    /// Total integration time.
    pub delta_t: f64,

    /// Biases at the linearization point (when preintegration was computed).
    /// The Jacobians below are w.r.t. *delta* from these values.
    pub bg_lin: NaVector3<f64>,
    pub ba_lin: NaVector3<f64>,

    /// Jacobian of ΔR w.r.t. gyro bias: ∂ΔR/∂b_g (3×3).
    /// Maps δbg → tangent-space perturbation of ΔR.
    pub jacobian_delta_r_wrt_bg: Matrix3<f64>,
    /// Jacobian of Δv w.r.t. gyro bias: ∂Δv/∂b_g (3×3).
    pub jacobian_delta_v_wrt_bg: Matrix3<f64>,
    /// Jacobian of Δv w.r.t. accel bias: ∂Δv/∂b_a (3×3).
    pub jacobian_delta_v_wrt_ba: Matrix3<f64>,
    /// Jacobian of Δp w.r.t. gyro bias: ∂Δp/∂b_g (3×3).
    pub jacobian_delta_p_wrt_bg: Matrix3<f64>,
    /// Jacobian of Δp w.r.t. accel bias: ∂Δp/∂b_a (3×3).
    pub jacobian_delta_p_wrt_ba: Matrix3<f64>,

    /// Full 9×9 covariance matrix of the preintegrated measurements.
    /// Order: [ΔR (3), Δv (3), Δp (3)].
    pub covariance: Matrix9<f64>,

    /// Number of IMU samples used.
    pub num_samples: usize,
}

/// IMU noise configuration for preintegration.
#[derive(Debug, Clone, Copy)]
pub struct ImuNoiseConfig {
    pub gyro_noise_std: f64,
    pub gyro_random_walk_std: f64,
    pub accel_noise_std: f64,
    pub accel_random_walk_std: f64,
}

/// IMU residual computation state.
pub struct ImuResidualState<'a> {
    pub r_i: &'a Matrix3<f64>,
    pub r_j: &'a Matrix3<f64>,
    pub v_i: &'a NaVector3<f64>,
    pub v_j: &'a NaVector3<f64>,
    pub p_i: &'a NaVector3<f64>,
    pub p_j: &'a NaVector3<f64>,
    pub bg: &'a NaVector3<f64>,
    pub ba: &'a NaVector3<f64>,
    pub gravity: &'a NaVector3<f64>,
}

impl PreintegratedImuMeasurements {
    /// Preintegrate IMU measurements between two keyframes.
    ///
    /// # Arguments
    /// * `imu_samples` — IMU samples in chronological order
    /// * `gyro_bias` — current gyro bias estimate [rad/s]
    /// * `accel_bias` — current accel bias estimate [m/s²]
    /// * `noise` — IMU noise configuration
    /// * `gravity` — gravity vector in world frame
    #[allow(clippy::too_many_lines)]
    pub fn preintegrate(
        imu_samples: &[ImuData],
        gyro_bias: [f64; 3],
        accel_bias: [f64; 3],
        noise: ImuNoiseConfig,
        _gravity: &NaVector3<f64>,
    ) -> Self {
        // Early return for insufficient samples
        if imu_samples.len() < 2 {
            return Self {
                delta_r: Matrix3::identity(),
                delta_v: NaVector3::zeros(),
                delta_p: NaVector3::zeros(),
                delta_t: 0.0,
                bg_lin: NaVector3::new(gyro_bias[0], gyro_bias[1], gyro_bias[2]),
                ba_lin: NaVector3::new(accel_bias[0], accel_bias[1], accel_bias[2]),
                jacobian_delta_r_wrt_bg: Matrix3::zeros(),
                jacobian_delta_v_wrt_bg: Matrix3::zeros(),
                jacobian_delta_v_wrt_ba: Matrix3::zeros(),
                jacobian_delta_p_wrt_bg: Matrix3::zeros(),
                jacobian_delta_p_wrt_ba: Matrix3::zeros(),
                covariance: Matrix9::<f64>::zeros(),
                num_samples: imu_samples.len(),
            };
        }
        let bg = NaVector3::new(gyro_bias[0], gyro_bias[1], gyro_bias[2]);
        let ba = NaVector3::new(accel_bias[0], accel_bias[1], accel_bias[2]);

        let mut delta_r = Matrix3::identity();
        let mut delta_v = NaVector3::zeros();
        let mut delta_p = NaVector3::zeros();

        // Jacobians w.r.t. biases
        let mut j_dr_bg = Matrix3::zeros(); // ∂ΔR/∂b_g
        let mut j_dv_bg = Matrix3::zeros(); // ∂Δv/∂b_g
        let mut j_dv_ba = Matrix3::zeros(); // ∂Δv/∂b_a
        let mut j_dp_bg = Matrix3::zeros(); // ∂Δp/∂b_g
        let mut j_dp_ba = Matrix3::zeros(); // ∂Δp/∂b_a

        // Covariance matrix: [ΔR(3), Δv(3), Δp(3)] = 9×9
        let mut covariance = Matrix9::<f64>::zeros();

        // Total integration time
        let mut total_dt = 0.0;

        // Previous bias-corrected accel in body frame
        let mut a_k_body = if imu_samples.is_empty() {
            NaVector3::zeros()
        } else {
            NaVector3::new(
                imu_samples[0].accel[0] - ba.x,
                imu_samples[0].accel[1] - ba.y,
                imu_samples[0].accel[2] - ba.z,
            )
        };

        // Noise covariance for incremental propagation (9×9)
        // Order: [gyro noise (3), accel noise (3)]
        // The incremental covariance Σ_n = diag(σ²_g I₃, σ²_a I₃, 0₃) for the
        // [ΔR, Δv, Δp] part — position is not directly affected by measurement noise.
        let sigma_g2 = noise.gyro_noise_std * noise.gyro_noise_std;
        let sigma_a2 = noise.accel_noise_std * noise.accel_noise_std;
        let sigma_bg2 = noise.gyro_random_walk_std * noise.gyro_random_walk_std;
        let sigma_ba2 = noise.accel_random_walk_std * noise.accel_random_walk_std;

        // F matrix (9×9) and Q matrix (9×6) for covariance propagation:
        // Σ_{k+1} = F_k Σ_k F_k^T + Q_k Σ_n Q_k^T
        //
        // The state is [ΔR, Δv, Δp] and the noise input is [η_gd, η_ad, η_bg, η_ba].
        //
        // F_k = [ I       0       0 ]    Q_k (w.r.t. [η_gd, η_ad]) =
        //       [ A·dt   I       0 ]                                [ J_r·dt    0      ]
        //       [ 0      I·dt   I  ]                                [ 0       I·dt   ]
        //                                                              [ 0       0      ]
        //
        // where A = -[ΔR_k · (a_k - b_a)]× (skew-symmetric of rotated accel)

        for k in 0..imu_samples.len() - 1 {
            let curr = &imu_samples[k];
            let next = &imu_samples[k + 1];
            let dt = (next.timestamp - curr.timestamp) as f64 / 1e9;

            if dt <= 1e-9 || dt > 1.0 {
                // Skip invalid or gap samples; recompute a_k_body
                a_k_body = NaVector3::new(
                    next.accel[0] - ba.x,
                    next.accel[1] - ba.y,
                    next.accel[2] - ba.z,
                );
                continue;
            }

            total_dt += dt;

            // ── Bias-corrected angular velocity ──
            let omega = NaVector3::new(
                curr.gyro[0] - bg.x,
                curr.gyro[1] - bg.y,
                curr.gyro[2] - bg.z,
            );

            // ── Incremental rotation via Exp map ──
            let delta_theta = omega * dt;
            let delta_r_k = exp_so3(&delta_theta);
            let delta_r_next = delta_r * delta_r_k;

            // ── Bias-corrected accel in body frame ──
            let a_next_body = NaVector3::new(
                next.accel[0] - ba.x,
                next.accel[1] - ba.y,
                next.accel[2] - ba.z,
            );

            // Rotate accel into initial body frame (Forster-style: no gravity subtraction here;
            // gravity appears only in the residual, not the preintegrated measurement).
            let a_k_world = delta_r * a_k_body;
            let a_next_world = delta_r_next * a_next_body;
            let a_mid = (a_k_world + a_next_world) * 0.5;

            // ── Integrate velocity and position ──
            delta_v += a_mid * dt;
            delta_p += delta_v * dt + a_k_world * 0.5 * dt * dt;

            // ── Update Jacobians w.r.t. biases ──
            // ∂ΔR/∂b_g: chain rule through incremental rotation
            // d(ΔR_{k+1})/db_g = d(ΔR_k)/db_g · Exp(δθ) + ΔR_k · ∂Exp(δθ)/∂b_g
            // ∂Exp(δθ)/∂b_g = -J_r(δθ) · dt  (since δθ = (ω - b_g)·dt)
            let jr = right_jacobian_so3(&delta_theta);
            j_dr_bg = delta_r_k.transpose() * j_dr_bg - jr * dt;

            // A = -skew(ΔR_k · (a_k - b_a)) = -skew(a_k_world + gravity)
            // but we use the body-frame version: -skew(ΔR_k · a_k_body)
            let a_k_rotated = delta_r * a_k_body;
            let skew_a_k = Matrix3::new(
                0.0,
                -a_k_rotated.z,
                a_k_rotated.y,
                a_k_rotated.z,
                0.0,
                -a_k_rotated.x,
                -a_k_rotated.y,
                a_k_rotated.x,
                0.0,
            );

            // ∂Δv/∂b_g update:
            // ∂Δv_{k+1}/∂b_g = ∂Δv_k/∂b_g + ½·∂(ΔR_k·a_k)/∂b_g·dt + ½·∂(ΔR_{k+1}·a_{k+1})/∂b_g·dt
            // ∂(ΔR·a)/∂b_g = -skew(ΔR·a) · ∂ΔR/∂b_g  (in tangent space approx)
            // Simplified: ∂Δv/∂b_g += -skew(a_k_rotated) · j_dr_bg · dt * 0.5
            //   plus the same for the next term
            let a_next_rotated = delta_r_next * a_next_body;
            let skew_a_next = Matrix3::new(
                0.0,
                -a_next_rotated.z,
                a_next_rotated.y,
                a_next_rotated.z,
                0.0,
                -a_next_rotated.x,
                -a_next_rotated.y,
                a_next_rotated.x,
                0.0,
            );
            j_dv_bg += -(skew_a_k * j_dr_bg + skew_a_next * (delta_r_k.transpose() * j_dr_bg))
                * 0.5
                * dt;
            j_dv_ba -= Matrix3::identity() * dt; // direct: ∂Δv/∂b_a = -I·dt (first order)

            // ∂Δp/∂b_g update:
            // ∂Δp_{k+1}/∂b_g = ∂Δp_k/∂b_g + ∂Δv_k/∂b_g·dt
            //   + ½·∂(ΔR_k·a_k)/∂b_g·dt²
            j_dp_bg += j_dv_bg * dt - skew_a_k * j_dr_bg * 0.5 * dt * dt;
            j_dp_ba += j_dv_ba * dt - Matrix3::identity() * 0.5 * dt * dt;

            // ── Covariance propagation ──
            // State transition matrix F (9×9):
            // F = [ I        0      0 ]
            //     [ F_vr·dt  I      0 ]
            //     [ 0       I·dt   I ]
            // where F_vr = -skew(a_mid_world) ≈ -skew(a_k_world)
            let f_vr = Matrix3::new(
                0.0,
                a_k_world.z,
                -a_k_world.y,
                -a_k_world.z,
                0.0,
                a_k_world.x,
                a_k_world.y,
                -a_k_world.x,
                0.0,
            ); // = -skew(a_k_world)

            // Noise-driven Jacobian Q (9×6): maps [η_gd, η_ad] → [ΔR, Δv, Δp]
            // Q = [ J_r·dt    0     ]
            //     [ 0        I·dt   ]
            //     [ 0        0      ]
            let mut q = Matrix9x6::<f64>::zeros();
            // Top-left: ∂ΔR/∂η_gd = J_r · dt  (but we use Jr·dt as an approximation)
            q.fixed_view_mut::<3, 3>(0, 0).copy_from(&(jr * dt));
            // Middle-right: ∂Δv/∂η_ad = I · dt (approx; rotated by ΔR)
            q.fixed_view_mut::<3, 3>(3, 3).copy_from(&(delta_r * dt));

            // Measurement noise covariance (6×6): diag(σ²_g, σ²_a)
            // Plus bias random walk contributions
            let mut sigma_n = Matrix6::<f64>::zeros();
            for i in 0..3 {
                sigma_n[(i, i)] = sigma_g2 / dt + sigma_bg2 * dt; // gyro: white noise + random walk
                sigma_n[(i + 3, i + 3)] = sigma_a2 / dt + sigma_ba2 * dt;
            }

            // Build F matrix (9×9)
            let mut f = Matrix9::<f64>::identity();
            f.fixed_view_mut::<3, 3>(3, 0).copy_from(&(f_vr * dt));
            f.fixed_view_mut::<3, 3>(6, 3).copy_from(&(Matrix3::identity() * dt));

            // Σ_{k+1} = F · Σ_k · F^T + Q · Σ_n · Q^T
            let q_sigma_qt = q * sigma_n * q.transpose();
            covariance = f * covariance * f.transpose() + q_sigma_qt;

            // Update state
            delta_r = delta_r_next;
            a_k_body = a_next_body;
        }

        Self {
            delta_r,
            delta_v,
            delta_p,
            delta_t: total_dt,
            bg_lin: bg,
            ba_lin: ba,
            jacobian_delta_r_wrt_bg: j_dr_bg,
            jacobian_delta_v_wrt_bg: j_dv_bg,
            jacobian_delta_v_wrt_ba: j_dv_ba,
            jacobian_delta_p_wrt_bg: j_dp_bg,
            jacobian_delta_p_wrt_ba: j_dp_ba,
            covariance,
            num_samples: imu_samples.len(),
        }
    }

    /// Compute the IMU residual given current estimates of poses, velocities, and biases.
    ///
    /// # Arguments
    /// * `r_i`, `r_j` — rotation matrices of keyframes i and j (world-to-body)
    /// * `v_i` — velocity of keyframe i (in world frame)
    /// * `p_i`, `p_j` — positions of keyframes i and j (in world frame)
    /// * `bg` — current gyro bias estimate
    /// * `ba` — current accel bias estimate
    /// * `gravity` — gravity vector
    ///
    /// # Returns
    /// 15-element residual vector: [r_res(3), v_res(3), p_res(3), bg_res(3), ba_res(3)]
    pub fn compute_residual(&self, state: ImuResidualState) -> [f64; 15] {
        let ImuResidualState { r_i, r_j, v_i, v_j, p_i, p_j, bg, ba, gravity } = state;
        let mut residual = [0.0f64; 15];

        // Delta biases from linearization point
        let db_g = bg - self.bg_lin;
        let db_a = ba - self.ba_lin;

        // Rotation residual: r_res = log(ΔR_corrected^T · R_i^T · R_j)
        // ΔR_corrected = ΔR_0 · Exp(J_dr_bg · δbg)
        let delta_r_corrected = self.delta_r * exp_so3(&(self.jacobian_delta_r_wrt_bg * db_g));
        let r_err = delta_r_corrected.transpose() * r_i.transpose() * r_j;
        let r_res = log_so3(&r_err);
        residual[0..3].copy_from_slice(r_res.as_slice());

        // Velocity residual: v_res = Δv_corrected - R_i^T · (v_j - v_i - g · Δt)
        // Δv_corrected = Δv_0 + J_dv_bg · δbg + J_dv_ba · δba
        let delta_v_corrected = self.delta_v
            + self.jacobian_delta_v_wrt_bg * db_g
            + self.jacobian_delta_v_wrt_ba * db_a;
        let v_expected = r_i.transpose() * (v_j - v_i + gravity * self.delta_t);
        let v_res = delta_v_corrected - v_expected;
        residual[3..6].copy_from_slice(v_res.as_slice());

        // Position residual: p_res = Δp_corrected - R_i^T · (p_j - p_i - v_i · Δt - ½ · g · Δt²)
        let delta_p_corrected = self.delta_p
            + self.jacobian_delta_p_wrt_bg * db_g
            + self.jacobian_delta_p_wrt_ba * db_a;
        let p_expected =
            r_i.transpose() * (p_j - p_i - v_i * self.delta_t + gravity * 0.5 * self.delta_t * self.delta_t);
        let p_res = delta_p_corrected - p_expected;
        residual[6..9].copy_from_slice(p_res.as_slice());

        // Bias residuals: random walk prior from linearization point
        residual[9..12].copy_from_slice(db_g.as_slice());
        residual[12..15].copy_from_slice(db_a.as_slice());

        residual
    }

    /// Compute the 15×15 information matrix (inverse covariance).
    ///
    /// The full covariance includes:
    /// - Preintegration covariance (9×9 for ΔR, Δv, Δp)
    /// - Bias random walk covariance (6×6 for b_g, b_a)
    ///
    /// The bias noise parameters can be tuned; defaults are reasonable for
    /// typical MEMS IMUs (≈200 Hz).
    pub fn information_matrix(&self, gyro_random_walk_std: f64, accel_random_walk_std: f64) -> Matrix15<f64> {
        // Build the full 15×15 covariance matrix
        let mut cov = Matrix15::<f64>::zeros();

        // Copy the 9×9 preintegration covariance
        cov.fixed_view_mut::<9, 9>(0, 0).copy_from(&self.covariance);

        // Bias random walk covariance (diagonal, scaled by integration time)
        // Σ_bg = σ²_bg · Δt · I,  Σ_ba = σ²_ba · Δt · I
        let sigma_bg2 = gyro_random_walk_std * gyro_random_walk_std;
        let sigma_ba2 = accel_random_walk_std * accel_random_walk_std;
        for i in 0..3 {
            cov[(9 + i, 9 + i)] = sigma_bg2 * self.delta_t;
            cov[(12 + i, 12 + i)] = sigma_ba2 * self.delta_t;
        }

        // Invert to get information matrix
        // For numerical stability, use Cholesky decomposition
        if let Some(cholesky) = cov.cholesky() {
            let info: Matrix15<f64> = cholesky.inverse();
            info
        } else {
            // Fallback: add small regularization
            for i in 0..15 {
                cov[(i, i)] += tolerance::EPSILON;
            }
            if let Some(cholesky) = cov.cholesky() {
                let info: Matrix15<f64> = cholesky.inverse();
                info
            } else {
                Matrix15::<f64>::identity()
            }
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datasets::ImuData;

    fn make_imu_sample(timestamp_ns: i64, gyro: [f64; 3], accel: [f64; 3]) -> ImuData {
        ImuData {
            timestamp: timestamp_ns,
            gyro,
            accel,
        }
    }

    #[test]
    fn test_exp_so3_identity() {
        // Zero rotation should give identity matrix
        let r = NaVector3::<f64>::zeros();
        let R = exp_so3(&r);
        assert!((R - Matrix3::identity()).norm() < 1e-10);
    }

    #[test]
    fn test_exp_log_roundtrip() {
        // exp(log(R)) ≈ R for a rotation matrix
        let phi = NaVector3::new(0.1, -0.2, 0.3);
        let R = exp_so3(&phi);
        let phi_back = log_so3(&R);
        assert!((phi - phi_back).norm() < 1e-8);
    }

    #[test]
    fn test_preintegrate_stationary_no_motion() {
        // Stationary IMU: gyro ≈ 0, accel ≈ [0, 0, g] in world frame
        // In body frame with z-down, accel should read [0, 0, g] ≈ [0, 0, 9.81]
        let gravity = NaVector3::new(0.0, 0.0, 9.81);
        let gyro_bias = [0.0; 3];
        let accel_bias = [0.0; 3];

        // Generate 100 samples at 200 Hz over 0.5 seconds, stationary
        let dt_ns = 5_000_000; // 5ms = 200 Hz
        let samples: Vec<ImuData> = (0..100)
            .map(|i| {
                make_imu_sample(
                    i as i64 * dt_ns,
                    [0.0, 0.0, 0.0],       // gyro: no rotation
                    [0.0, 0.0, 9.81],      // accel: gravity in body frame (z-down)
                )
            })
            .collect();

        let noise = ImuNoiseConfig {
            gyro_noise_std: 1e-3,
            gyro_random_walk_std: 1e-4,
            accel_noise_std: 1e-2,
            accel_random_walk_std: 1e-3,
        };
        let result = PreintegratedImuMeasurements::preintegrate(
            &samples,
            gyro_bias,
            accel_bias,
            noise,
            &gravity,
        );

        // After integration (Forster convention: gravity NOT subtracted during integration):
        // - delta_r should be identity (no rotation)
        // - delta_v ≈ [0, 0, g*T] (raw IMU gravity reading integrated, 100 samples × 5ms = 99 steps)
        // - delta_p ≈ [0, 0, 0.5*g*T²] (double integral of IMU gravity reading)
        // Gravity is handled in the residual, not in the preintegration.
        let n_steps = 99usize; // 100 samples → 99 integration pairs
        let total_t = n_steps as f64 * 0.005;
        let g = 9.81;
        assert!((result.delta_r - Matrix3::identity()).norm() < 1e-4, "delta_r not identity: {:?}", result.delta_r);
        assert!((result.delta_v.z - g * total_t).abs() < 0.1, "delta_v.z expected ≈ {:.3}, got {:.3}", g * total_t, result.delta_v.z);
        assert!((result.delta_p.z - 0.5 * g * total_t * total_t).abs() < 0.1, "delta_p.z expected ≈ {:.3}, got {:.3}", 0.5 * g * total_t * total_t, result.delta_p.z);
    }

    #[test]
    fn test_preintegrate_pure_rotation() {
        // Pure rotation around Z axis at constant rate
        let gravity = NaVector3::new(0.0, 0.0, 9.81);
        let gyro_bias = [0.0; 3];
        let accel_bias = [0.0; 3];

        // 100 samples at 200 Hz, rotating at 0.5 rad/s around Z
        let dt_ns = 5_000_000;
        let omega_z = 0.5;
        let total_angle = omega_z * 0.5; // 0.25 rad over 0.5s
        let samples: Vec<ImuData> = (0..100)
            .map(|i| {
                make_imu_sample(
                    i as i64 * dt_ns,
                    [0.0, 0.0, omega_z],
                    [0.0, 0.0, 9.81],
                )
            })
            .collect();

        let noise = ImuNoiseConfig {
            gyro_noise_std: 1e-3,
            gyro_random_walk_std: 1e-4,
            accel_noise_std: 1e-2,
            accel_random_walk_std: 1e-3,
        };
        let result = PreintegratedImuMeasurements::preintegrate(
            &samples,
            gyro_bias,
            accel_bias,
            noise,
            &gravity,
        );

        // Check that delta_r ≈ rotation by total_angle around Z
        let expected_r = exp_so3(&NaVector3::new(0.0, 0.0, total_angle));
        let r_err = log_so3(&(result.delta_r.transpose() * expected_r));
        assert!(r_err.norm() < 0.05, "delta_r error too large: {:?} (expected ≈ {:.4} rad)", r_err, total_angle);
    }

    #[test]
    fn test_preintegrate_covariance_positive_definite() {
        let gravity = NaVector3::new(0.0, 0.0, 9.81);
        let samples: Vec<ImuData> = (0..50)
            .map(|i| {
                make_imu_sample(
                    i as i64 * 5_000_000,
                    [0.0, 0.01, 0.0],
                    [0.0, 0.0, 9.81],
                )
            })
            .collect();

        let noise = ImuNoiseConfig {
            gyro_noise_std: 1e-3,
            gyro_random_walk_std: 1e-4,
            accel_noise_std: 1e-2,
            accel_random_walk_std: 1e-3,
        };
        let result = PreintegratedImuMeasurements::preintegrate(
            &samples,
            [0.0; 3],
            [0.0; 3],
            noise,
            &gravity,
        );

        // Covariance should be positive semi-definite (check diagonal is non-negative)
        for i in 0..9 {
            assert!(result.covariance[(i, i)] > 0.0, "Covariance diagonal not positive at {}", i);
        }
    }

    #[test]
    fn test_residual_zero_for_matching_state() {
        // When the state is kinematically consistent with the preintegration, residual should be ≈ 0.
        // The preintegration computes Δv, Δp assuming the IMU is in motion (gravity subtracted).
        // The kinematic equations are:
        //   v_j - v_i = R_i · Δv + g · Δt
        //   p_j - p_i = v_i · Δt + ½ · g · Δt² + R_i · Δp
        // For stationary IMU (Δv = Δp = 0), we need v_j = v_i + g·Δt and p_j = p_i + v_i·Δt + ½g·Δt².
        let gravity = NaVector3::new(0.0, 0.0, 9.81);

        // Stationary IMU: 20 samples at 200 Hz, Δt = 0.1s
        let samples: Vec<ImuData> = (0..20)
            .map(|i| {
                make_imu_sample(
                    i as i64 * 5_000_000,
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 9.81],
                )
            })
            .collect();

        let noise = ImuNoiseConfig {
            gyro_noise_std: 1e-3,
            gyro_random_walk_std: 1e-4,
            accel_noise_std: 1e-2,
            accel_random_walk_std: 1e-3,
        };
        let preint = PreintegratedImuMeasurements::preintegrate(
            &samples,
            [0.0; 3],
            [0.0; 3],
            noise,
            &gravity,
        );

        let dt = preint.delta_t;

        // State consistent with preintegration (stationary IMU, no motion) in Forster convention.
        // In Forster: Δv = g·Δt, Δp = ½g·Δt² (gravity reading not subtracted in integration).
        // Residual r_v = Δv - R_i^T·(v_j - v_i + g·Δt) = g·Δt - (v_j + g·Δt) = 0 → v_j = 0
        // Residual r_p = Δp - R_i^T·(p_j - p_i - v_i·Δt + ½g·Δt²) = ½g·Δt² - ½g·Δt² = 0 → p_j = 0
        let r_i = Matrix3::identity();
        let r_j = Matrix3::identity();
        let v_i = NaVector3::zeros();
        let v_j = NaVector3::zeros();  // stationary: no velocity change in world frame
        let p_i = NaVector3::zeros();
        let p_j = NaVector3::zeros();  // stationary: no position change in world frame
        let bg = NaVector3::zeros();
        let ba = NaVector3::zeros();

        let state = ImuResidualState {
            r_i: &r_i,
            r_j: &r_j,
            v_i: &v_i,
            v_j: &v_j,
            p_i: &p_i,
            p_j: &p_j,
            bg: &bg,
            ba: &ba,
            gravity: &gravity,
        };
        let res = preint.compute_residual(state);

        let res_rot = NaVector3::from_column_slice(&res[0..3]);
        let res_vel = NaVector3::from_column_slice(&res[3..6]);
        let res_pos = NaVector3::from_column_slice(&res[6..9]);
        let res_bg = NaVector3::from_column_slice(&res[9..12]);
        let res_ba = NaVector3::from_column_slice(&res[12..15]);

        assert!(res_rot.norm() < 0.01, "Rotation residual too large: {:?}", res_rot);
        assert!(res_vel.norm() < 0.01, "Velocity residual too large: {:?}", res_vel);
        assert!(res_pos.norm() < 0.01, "Position residual too large: {:?}", res_pos);
        assert!(res_bg.norm() < 1e-10, "Bias residual (bg) not zero: {:?}", res_bg);
        assert!(res_ba.norm() < 1e-10, "Bias residual (ba) not zero: {:?}", res_ba);
    }

    #[test]
    fn test_bias_correction_residual() {
        // Preintegrate with zero biases but a non-zero gyro reading.
        // When we evaluate the residual with a matching bias, it should be SMALLER
        // (because the bias corrects the gyro reading back toward zero).
        let gravity = NaVector3::new(0.0, 0.0, 9.81);
        let gyro_reading = 0.01; // non-zero gyro
        let samples: Vec<ImuData> = (0..50)
            .map(|i| {
                make_imu_sample(
                    i as i64 * 5_000_000,
                    [gyro_reading, 0.0, 0.0],
                    [0.0, 0.0, 9.81],
                )
            })
            .collect();

        // Preintegrate at zero bias (linearization point)
        let noise = ImuNoiseConfig {
            gyro_noise_std: 1e-3,
            gyro_random_walk_std: 1e-4,
            accel_noise_std: 1e-2,
            accel_random_walk_std: 1e-3,
        };
        let preint = PreintegratedImuMeasurements::preintegrate(
            &samples,
            [0.0; 3],
            [0.0; 3],
            noise,
            &gravity,
        );

        // Zero-bias residual (gyro reading not corrected)
        let state_zero = ImuResidualState {
            r_i: &Matrix3::identity(),
            r_j: &Matrix3::identity(),
            v_i: &NaVector3::zeros(),
            v_j: &NaVector3::zeros(),
            p_i: &NaVector3::zeros(),
            p_j: &NaVector3::zeros(),
            bg: &NaVector3::zeros(),
            ba: &NaVector3::zeros(),
            gravity: &gravity,
        };
        let res_zero = preint.compute_residual(state_zero);

        // Matching bias: bg = gyro_reading cancels the gyro
        let bg_match = NaVector3::new(gyro_reading, 0.0, 0.0);
        let state_bias = ImuResidualState {
            r_i: &Matrix3::identity(),
            r_j: &Matrix3::identity(),
            v_i: &NaVector3::zeros(),
            v_j: &NaVector3::zeros(),
            p_i: &NaVector3::zeros(),
            p_j: &NaVector3::zeros(),
            bg: &bg_match,
            ba: &NaVector3::zeros(),
            gravity: &gravity,
        };
        let res_bias = preint.compute_residual(state_bias);

        // The bias-matching residual should be SMALLER (bias corrects the gyro)
        let res_zero_rot = NaVector3::from_column_slice(&res_zero[0..3]);
        let res_bias_rot = NaVector3::from_column_slice(&res_bias[0..3]);
        assert!(res_bias_rot.norm() < res_zero_rot.norm(),
            "Bias-corrected residual ({:.6}) should be smaller than zero-bias ({:.6})",
            res_bias_rot.norm(), res_zero_rot.norm());
    }

    #[test]
    fn test_information_matrix_symmetric() {
        let gravity = NaVector3::new(0.0, 0.0, 9.81);
        let samples: Vec<ImuData> = (0..20)
            .map(|i| make_imu_sample(i as i64 * 5_000_000, [0.0; 3], [0.0, 0.0, 9.81]))
            .collect();

        let noise = ImuNoiseConfig {
            gyro_noise_std: 1e-3,
            gyro_random_walk_std: 1e-4,
            accel_noise_std: 1e-2,
            accel_random_walk_std: 1e-3,
        };
        let preint = PreintegratedImuMeasurements::preintegrate(
            &samples, [0.0; 3], [0.0; 3], noise, &gravity,
        );

        let info = preint.information_matrix(1e-4, 1e-3);

        // Information matrix should be symmetric (floating-point tolerance for matrix inversion)
        for i in 0..15 {
            for j in (i + 1)..15 {
                assert!(
                    (info[(i, j)] - info[(j, i)]).abs() < 1e-6,
                    "Info matrix not symmetric at ({}, {}): {} vs {}",
                    i, j, info[(i, j)], info[(j, i)]
                );
            }
        }
    }

    #[test]
    fn test_preintegrate_empty_samples() {
        let gravity = NaVector3::new(0.0, 0.0, 9.81);
        let noise = ImuNoiseConfig {
            gyro_noise_std: 1e-3,
            gyro_random_walk_std: 1e-4,
            accel_noise_std: 1e-2,
            accel_random_walk_std: 1e-3,
        };
        let result = PreintegratedImuMeasurements::preintegrate(
            &[],
            [0.0; 3],
            [0.0; 3],
            noise,
            &gravity,
        );

        assert!(result.num_samples == 0);
        assert!(result.delta_t.abs() < 1e-10);
        assert!((result.delta_r - Matrix3::identity()).norm() < 1e-10);
        assert!(result.delta_v.norm() < 1e-10);
        assert!(result.delta_p.norm() < 1e-10);
    }

    #[test]
    fn test_preintegrate_single_sample() {
        let gravity = NaVector3::new(0.0, 0.0, 9.81);
        let samples = vec![make_imu_sample(0, [0.0; 3], [0.0, 0.0, 9.81])];

        let noise = ImuNoiseConfig {
            gyro_noise_std: 1e-3,
            gyro_random_walk_std: 1e-4,
            accel_noise_std: 1e-2,
            accel_random_walk_std: 1e-3,
        };
        let result = PreintegratedImuMeasurements::preintegrate(
            &samples,
            [0.0; 3],
            [0.0; 3],
            noise,
            &gravity,
        );

        // Single sample: no integration possible
        assert!(result.num_samples == 1);
        assert!(result.delta_t.abs() < 1e-10);
        assert!((result.delta_r - Matrix3::identity()).norm() < 1e-10);
    }
}
