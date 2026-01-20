use apex_solver::factors::Factor;
use apex_solver::manifold::se3;
use na::{DMatrix, DVector, Matrix4, Matrix6};
use nalgebra as na;

/// Relative pose factor for loop-closure constraints between two keyframes (SE3 × SE3 → R⁶).
///
/// Residual is computed as:
/// r = sqrt_info * Log(T_meas^{-1} * T_1 * T_2^{-1})
/// where Log returns [translation; rotation_axis_angle].
#[derive(Debug, Clone)]
pub struct LoopClosurePoseFactor {
    /// Measured transform taking frame 2 into frame 1 (T_1_2).
    pub T_1_2: Matrix4<f64>,
    /// Information matrix (inverse covariance) for the 6D residual.
    pub information: Matrix6<f64>,
}

impl LoopClosurePoseFactor {
    pub fn new(T_1_2: Matrix4<f64>, information: Matrix6<f64>) -> Self {
        Self { T_1_2, information }
    }

    fn sqrt_information(&self) -> Option<DMatrix<f64>> {
        if let Some(chol) = self.information.cholesky() {
            let l = chol.l();
            Some(DMatrix::from_fn(6, 6, |r, c| l[(r, c)]))
        } else {
            // Fallback to raw information if not SPD
            Some(DMatrix::from_fn(6, 6, |r, c| self.information[(r, c)]))
        }
    }

    fn compute_relative(
        R1: &na::Matrix3<f64>,
        t1: &na::Vector3<f64>,
        R2: &na::Matrix3<f64>,
        t2: &na::Vector3<f64>,
    ) -> (na::Matrix3<f64>, na::Vector3<f64>) {
        // Relative rotation R_rel = R1 * R2^T
        let R_rel = R1 * R2.transpose();
        // Relative translation t_rel = t1 - R_rel * t2
        let t_rel = t1 - R_rel * t2;
        (R_rel, t_rel)
    }

    fn log_so3(R: na::Matrix3<f64>) -> na::Vector3<f64> {
        let q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_matrix_unchecked(R));
        let angle = q.angle();
        if angle < 1e-12 {
            return na::Vector3::zeros();
        }

        if let Some(axis) = q.axis() {
            axis.into_inner() * angle
        } else {
            na::Vector3::zeros()
        }
    }
}

impl Factor for LoopClosurePoseFactor {
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        assert_eq!(
            params.len(),
            2,
            "LoopClosurePoseFactor requires 2 parameter vectors (KF1, KF2)"
        );
        assert_eq!(
            params[0].len(),
            7,
            "Pose 1 must have 7 parameters (tx, ty, tz, qw, qx, qy, qz)"
        );
        assert_eq!(
            params[1].len(),
            7,
            "Pose 2 must have 7 parameters (tx, ty, tz, qw, qx, qy, qz)"
        );

        let T1 = se3::SE3::from(params[0].clone());
        let T2 = se3::SE3::from(params[1].clone());

        let R1 = T1.rotation_so3().rotation_matrix();
        let t1 = T1.translation();
        let R2 = T2.rotation_so3().rotation_matrix();
        let t2 = T2.translation();

        let (R_rel, t_rel) = Self::compute_relative(&R1, &t1, &R2, &t2);

        // Measurement
        let R_meas = self.T_1_2.fixed_view::<3, 3>(0, 0).into_owned();
        let t_meas = self.T_1_2.fixed_view::<3, 1>(0, 3).into_owned();

        // Pose error
        let t_err = t_rel - t_meas;
        let R_err = R_meas.transpose() * R_rel;
        let rot_err = Self::log_so3(R_err);

        // Unweighted residual
        let mut residuals = DVector::zeros(6);
        residuals.view_mut((0, 0), (3, 1)).copy_from(&t_err);
        residuals.view_mut((3, 0), (3, 1)).copy_from(&rot_err);

        // Apply sqrt information
        if let Some(sqrt_info) = self.sqrt_information() {
            residuals = sqrt_info * residuals;
        }

        let jacobian_matrix = if compute_jacobian {
            // Base jacobian (6x12) before weighting: translation/rotation blocks with small-angle approximation
            let mut base_jac = DMatrix::zeros(6, 12);
            // ∂t/∂t1 = I
            for i in 0..3 {
                base_jac[(i, i)] = 1.0;
            }
            // ∂t/∂t2 = -I
            for i in 0..3 {
                base_jac[(i, 6 + i)] = -1.0;
            }
            // ∂rot/∂ω1 ≈ I
            for i in 0..3 {
                base_jac[(3 + i, 3 + i)] = 1.0;
            }
            // ∂rot/∂ω2 ≈ -I
            for i in 0..3 {
                base_jac[(3 + i, 9 + i)] = -1.0;
            }

            let weighted_jac = if let Some(sqrt_info) = self.sqrt_information() {
                sqrt_info * base_jac
            } else {
                base_jac
            };

            Some(weighted_jac)
        } else {
            None
        };

        (residuals, jacobian_matrix)
    }

    fn get_dimension(&self) -> usize {
        6
    }
}
