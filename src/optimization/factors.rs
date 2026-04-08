//! Optimization factors for bundle adjustment and PnP.
//!
//! Each factor implements the [`Factor`](apex_solver::factors::Factor) trait and computes
//! reprojection residuals with optional Jacobians for SE(3) manifold optimization.
//!
//! ## Factor types
//!
//! - [`PinholeProjectionFactor`] — optimizes only the 3D point, camera pose is fixed
//! - [`BundleAdjustmentFactor`] — full BA: optimizes both 3D point and camera pose
//! - [`BundleAdjustmentFactorTranslationOnly`] — BA with translation-only optimization
//! - [`PnPFactor`] — Perspective-n-Point: optimizes camera pose given known 3D points

use apex_solver::factors::Factor;
use apex_solver::manifold::se3;
use na::{DMatrix, DVector, Matrix3, Matrix4, Vector2, Vector3};
use nalgebra as na;

// ============================================================================
// Shared projection helpers (used by all factors below)
// ============================================================================

/// Project a 3D point in camera frame to normalized coordinates: `[x/z, y/z]`.
#[inline]
pub fn project_to_normalized(p_C: Vector3<f64>) -> Vector2<f64> {
    let inv_z = 1.0 / p_C[2];
    Vector2::new(p_C[0] * inv_z, p_C[1] * inv_z)
}

/// Jacobian of `[x/z, y/z]` w.r.t. the 3D point in camera frame.
///
/// ```text
/// ∂(x/z)/∂x = 1/z    ∂(x/z)/∂y = 0    ∂(x/z)/∂z = -x/z²
/// ∂(y/z)/∂x = 0      ∂(y/z)/∂y = 1/z  ∂(y/z)/∂z = -y/z²
/// ```
#[inline]
pub fn jacobian_proj_wrt_p_C(p_C: Vector3<f64>) -> na::Matrix2x3<f64> {
    let x = p_C[0];
    let y = p_C[1];
    let z = p_C[2];
    let inv_z = 1.0 / z;
    let inv_z_sq = inv_z * inv_z;

    na::Matrix2x3::new(inv_z, 0.0, -x * inv_z_sq, 0.0, inv_z, -y * inv_z_sq)
}

/// Skew-symmetric matrix of a 3D vector for cross-product: `[v]× * u = v × u`.
#[inline]
pub fn skew_symmetric(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

// ============================================================================
// PinholeProjectionFactor
// ============================================================================

/// Pinhole projection factor for optimizing **only** the 3D point position.
///
/// The camera pose is held fixed. Residual is 2D reprojection error in normalized
/// coordinates: `[x/z, y/z] - observation`.
///
/// # Variables
/// - 3D point in world frame (3 params: x, y, z)
///
/// # Fixed
/// - Camera pose `T_C_W` (world-to-camera transform)
#[derive(Debug, Clone)]
pub struct PinholeProjectionFactor {
    /// Observed 2D point in normalized/undistorted coordinates.
    pub observation: Vector2<f64>,
    /// Transform from world to camera frame.
    pub T_C_W: Matrix4<f64>,
}

impl PinholeProjectionFactor {
    /// Create a new pinhole projection factor.
    pub fn new(observation: Vector2<f64>, T_C_W: Matrix4<f64>) -> Self {
        Self { observation, T_C_W }
    }
}

impl Factor for PinholeProjectionFactor {
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        assert_eq!(
            params.len(),
            1,
            "PinholeProjectionFactor requires 1 parameter vector"
        );
        assert_eq!(params[0].len(), 3, "3D point must have 3 parameters");

        let point_world = Vector3::new(params[0][0], params[0][1], params[0][2]);

        // Transform point from world to camera frame
        let R_C_W = self.T_C_W.fixed_view::<3, 3>(0, 0);
        let t_C_W = self.T_C_W.fixed_view::<3, 1>(0, 3);
        let point_camera = R_C_W * point_world + t_C_W;

        let proj = project_to_normalized(point_camera);
        let residuals = DVector::from_row_slice(&[
            proj[0] - self.observation[0],
            proj[1] - self.observation[1],
        ]);

        let jacobian = if compute_jacobian {
            let jac = jacobian_proj_wrt_p_C(point_camera) * R_C_W;
            let mut j = DMatrix::zeros(2, 3);
            j.copy_from(&jac);
            Some(j)
        } else {
            None
        };

        (residuals, jacobian)
    }

    fn get_dimension(&self) -> usize {
        2
    }
}

// ============================================================================
// BundleAdjustmentFactorTranslationOnly
// ============================================================================

/// Bundle adjustment factor that optimizes **only translation** of a camera pose.
/// Useful for development and debugging.
///
/// # Variables
/// - 3D point in world frame (3 params)
/// - Translation `t_B_W` (3 params), unless [`with_fixed_position`](Self::with_fixed_position) is set
#[derive(Debug, Clone)]
pub struct BundleAdjustmentFactorTranslationOnly {
    /// Observed 2D point in normalized/undistorted coordinates.
    pub observation: Vector2<f64>,
    /// Transform from body to camera frame.
    pub T_C_B: Matrix4<f64>,
    /// If set, the 3D point is fixed to this position.
    pub fixed_position: Option<Vector3<f64>>,
}

impl BundleAdjustmentFactorTranslationOnly {
    /// Create a new translation-only BA factor.
    pub fn new(observation: Vector2<f64>, T_C_B: Matrix4<f64>) -> Self {
        Self {
            observation,
            T_C_B,
            fixed_position: None,
        }
    }

    /// Fix the 3D point position — only translation will be optimized.
    #[must_use]
    pub fn with_fixed_position(mut self, position: Vector3<f64>) -> Self {
        self.fixed_position = Some(position);
        self
    }
}

impl Factor for BundleAdjustmentFactorTranslationOnly {
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        let p_W = Vector3::new(params[0][0], params[0][1], params[0][2]);

        let t_B_W: Vector3<f64>;
        if let Some(fixed_position) = self.fixed_position {
            t_B_W = fixed_position;
            assert_eq!(
                params.len(),
                1,
                "TranslationOnly with fixed position requires 1 parameter"
            );
        } else {
            t_B_W = Vector3::new(params[1][0], params[1][1], params[1][2]);
            assert_eq!(
                params.len(),
                2,
                "TranslationOnly requires 2 parameter vectors"
            );
        }

        let R_C_B = self.T_C_B.fixed_view::<3, 3>(0, 0);
        let t_C_B = self.T_C_B.fixed_view::<3, 1>(0, 3);
        let p_C = R_C_B * (p_W + t_B_W) + t_C_B;

        let proj = project_to_normalized(p_C);
        let residuals = DVector::from_row_slice(&[
            proj[0] - self.observation[0],
            proj[1] - self.observation[1],
        ]);

        let jacobian = if compute_jacobian {
            let jac_r_p_C = jacobian_proj_wrt_p_C(p_C);
            let jac_r_p_W = jac_r_p_C * R_C_B;

            if self.fixed_position.is_some() {
                let mut j = DMatrix::zeros(2, 3);
                j.copy_from(&jac_r_p_W);
                Some(j)
            } else {
                let jac_r_t = jac_r_p_C * R_C_B;
                let mut j = DMatrix::zeros(2, 6);
                j.view_mut((0, 0), (2, 3)).copy_from(&jac_r_p_W);
                j.view_mut((0, 3), (2, 3)).copy_from(&jac_r_t);
                Some(j)
            }
        } else {
            None
        };

        (residuals, jacobian)
    }

    fn get_dimension(&self) -> usize {
        2
    }
}

// ============================================================================
// BundleAdjustmentFactor
// ============================================================================

/// Full bundle adjustment factor: optimizes both 3D point and camera pose.
///
/// # Variables
/// - 3D point in world frame (3 params)
/// - System pose `T_B_W` as SE(3) (7 params: tx, ty, tz, qw, qx, qy, qz),
///   unless [`with_fixed_pose`](Self::with_fixed_pose) is set
#[derive(Debug, Clone)]
pub struct BundleAdjustmentFactor {
    /// Observed 2D point in normalized/undistorted coordinates.
    pub observation: Vector2<f64>,
    /// Transform from body to camera frame.
    pub T_C_B: Matrix4<f64>,
    /// Fixed pose `T_B_W` if provided; `None` means the pose is optimized.
    pub fixed_pose: Option<Matrix4<f64>>,
}

impl BundleAdjustmentFactor {
    /// Create a new bundle adjustment factor.
    pub fn new(observation: Vector2<f64>, T_C_B: Matrix4<f64>) -> Self {
        Self {
            observation,
            T_C_B,
            fixed_pose: None,
        }
    }

    /// Set a fixed pose `T_B_W`. When set, only the 3D point is optimized.
    #[must_use]
    pub fn with_fixed_pose(mut self, T_B_W: Matrix4<f64>) -> Self {
        self.fixed_pose = Some(T_B_W);
        self
    }
}

impl Factor for BundleAdjustmentFactor {
    #![allow(non_snake_case)]
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        let p_W = Vector3::new(params[0][0], params[0][1], params[0][2]);

        // Extract T_B_W: either from fixed pose or from SE(3) parameters
        let (R_B_W, t_B_W) = if let Some(T_B_W) = self.fixed_pose {
            assert_eq!(params.len(), 1, "BA with fixed pose requires 1 parameter");
            (
                T_B_W.fixed_view::<3, 3>(0, 0).into_owned(),
                T_B_W.fixed_view::<3, 1>(0, 3).into_owned(),
            )
        } else {
            assert_eq!(params.len(), 2, "BA requires 2 parameter vectors");
            assert_eq!(params[1].len(), 7, "SE(3) pose must have 7 parameters");
            let se3_pose = se3::SE3::from(params[1].clone());
            (
                se3_pose.rotation_so3().rotation_matrix(),
                se3_pose.translation(),
            )
        };

        let R_C_B = self.T_C_B.fixed_view::<3, 3>(0, 0);
        let t_C_B = self.T_C_B.fixed_view::<3, 1>(0, 3);

        // Transform: p_W -> p_B -> p_C
        let p_B = R_B_W * p_W + t_B_W;
        let p_C = R_C_B * p_B + t_C_B;

        let proj = project_to_normalized(p_C);
        let residuals = DVector::from_row_slice(&[
            proj[0] - self.observation[0],
            proj[1] - self.observation[1],
        ]);

        let jacobian = if compute_jacobian {
            let jac_proj = jacobian_proj_wrt_p_C(p_C);
            let jac_proj_R_C_B = jac_proj * R_C_B;
            let jac_r_p_W = jac_proj_R_C_B * R_B_W;

            if self.fixed_pose.is_some() {
                let mut j = DMatrix::zeros(2, 3);
                j.copy_from(&jac_r_p_W);
                Some(j)
            } else {
                // [∂r/∂p_W (2x3) | ∂r/∂t (2x3) | ∂r/∂ω (2x3)]
                let p_W_skew = skew_symmetric(&p_W);
                let jac_r_rot = jac_proj_R_C_B * (-&R_B_W * p_W_skew);

                let mut j = DMatrix::zeros(2, 9);
                j.view_mut((0, 0), (2, 3)).copy_from(&jac_r_p_W); // ∂r/∂p_W
                j.view_mut((0, 3), (2, 3)).copy_from(&jac_r_p_W); // ∂r/∂t
                j.view_mut((0, 6), (2, 3)).copy_from(&jac_r_rot); // ∂r/∂ω
                Some(j)
            }
        } else {
            None
        };

        (residuals, jacobian)
    }

    fn get_dimension(&self) -> usize {
        2
    }
}

// ============================================================================
// PnPFactor
// ============================================================================

/// Perspective-n-Point factor: optimizes camera pose given known 3D points.
///
/// # Variables
/// - System pose `T_B_W` as SE(3) (7 params: tx, ty, tz, qw, qx, qy, qz)
///
/// # Fixed
/// - 3D point `p_W` in world frame
#[derive(Debug, Clone)]
pub struct PnPFactor {
    /// Observed 2D point in normalized/undistorted coordinates.
    pub observation: Vector2<f64>,
    /// Transform from body to camera frame.
    pub T_C_B: Matrix4<f64>,
    /// Known 3D point in world frame.
    pub p_W: Vector3<f64>,
}

impl PnPFactor {
    /// Create a new PnP factor.
    pub fn new(observation: Vector2<f64>, T_C_B: Matrix4<f64>, p_W: Vector3<f64>) -> Self {
        Self {
            observation,
            T_C_B,
            p_W,
        }
    }
}

impl Factor for PnPFactor {
    #![allow(non_snake_case)]
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        assert_eq!(params.len(), 1, "PnPFactor requires 1 parameter vector");
        assert_eq!(params[0].len(), 7, "SE(3) pose must have 7 parameters");

        let T_B_W = se3::SE3::from(params[0].clone());
        let R_B_W: Matrix3<f64> = T_B_W.rotation_so3().rotation_matrix();
        let t_B_W: Vector3<f64> = T_B_W.translation();

        let R_C_B = self.T_C_B.fixed_view::<3, 3>(0, 0);
        let t_C_B = self.T_C_B.fixed_view::<3, 1>(0, 3);

        // Transform: p_W -> p_B -> p_C
        let p_B = R_B_W * self.p_W + t_B_W;
        let p_C = R_C_B * p_B + t_C_B;

        let proj = project_to_normalized(p_C);
        let residuals = DVector::from_row_slice(&[
            proj[0] - self.observation[0],
            proj[1] - self.observation[1],
        ]);

        let jacobian = if compute_jacobian {
            let jac_proj = jacobian_proj_wrt_p_C(p_C);
            let jac_proj_R_C_B = jac_proj * R_C_B;
            let jac_r_p_W = jac_proj_R_C_B * R_B_W;

            // [∂r/∂t (2x3) | ∂r/∂ω (2x3)]
            let p_W_skew = skew_symmetric(&self.p_W);
            let jac_r_rot = jac_proj_R_C_B * (-&R_B_W * p_W_skew);

            let mut j = DMatrix::zeros(2, 6);
            j.view_mut((0, 0), (2, 3)).copy_from(&jac_r_p_W); // ∂r/∂t
            j.view_mut((0, 3), (2, 3)).copy_from(&jac_r_rot); // ∂r/∂ω
            Some(j)
        } else {
            None
        };

        (residuals, jacobian)
    }

    fn get_dimension(&self) -> usize {
        2
    }
}
