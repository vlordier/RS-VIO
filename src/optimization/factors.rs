//! Optimization factors for bundle adjustment and PnP.
//!
//! Each factor implements the `Factor` trait from apex_solver and computes
//! reprojection residuals with optional Jacobians for SE(3) manifold optimization.
//!
//! ## Factor types
//!
//! - [`PinholeProjectionFactor`] — optimizes only the 3D point, camera pose is fixed
//! - [`BundleAdjustmentFactor`] — full BA: optimizes both 3D point and camera pose
//! - [`BundleAdjustmentFactorTranslationOnly`] — BA with translation-only optimization
//! - [`PnPFactor`] — Perspective-n-Point: optimizes camera pose given known 3D points

use apex_solver::factors::Factor;
use na::{DMatrix, DVector, Matrix3, Matrix4, Quaternion, Vector2, Vector3};
use nalgebra as na;

// ============================================================================
// Shared projection helpers (used by all factors below)
// ============================================================================

/// Minimum depth (metres) below which a point is considered behind / on the camera plane.
/// Both `project_to_normalized` and `jacobian_proj_wrt_p_C` return `None` for z ≤ this value.
const MIN_DEPTH: f64 = 1e-6;

/// Project a 3D point in camera frame to normalized coordinates: `[x/z, y/z]`.
///
/// Returns `None` when the point is at or behind the camera plane (`z ≤ MIN_DEPTH`),
/// preventing inf/NaN residuals from destabilising the solver.
#[inline(always)]
pub fn project_to_normalized(p_C: Vector3<f64>) -> Option<Vector2<f64>> {
    if p_C[2] <= MIN_DEPTH {
        return None;
    }
    let inv_z = 1.0 / p_C[2];
    Some(Vector2::new(p_C[0] * inv_z, p_C[1] * inv_z))
}

/// Jacobian of `[x/z, y/z]` w.r.t. the 3D point in camera frame.
///
/// ```text
/// ∂(x/z)/∂x = 1/z    ∂(x/z)/∂y = 0    ∂(x/z)/∂z = -x/z²
/// ∂(y/z)/∂x = 0      ∂(y/z)/∂y = 1/z  ∂(y/z)/∂z = -y/z²
/// ```
///
/// Returns `None` when `z ≤ MIN_DEPTH` (point behind / on camera plane).
#[inline(always)]
pub fn jacobian_proj_wrt_p_C(p_C: Vector3<f64>) -> Option<na::Matrix2x3<f64>> {
    if p_C[2] <= MIN_DEPTH {
        return None;
    }
    let x = p_C[0];
    let y = p_C[1];
    let z = p_C[2];
    let inv_z = 1.0 / z;
    let inv_z_sq = inv_z * inv_z;

    Some(na::Matrix2x3::new(inv_z, 0.0, -x * inv_z_sq, 0.0, inv_z, -y * inv_z_sq))
}

/// Compute `[x/z, y/z]` AND `jac_proj × mat` in a single pass, sharing the
/// one expensive division `1/z`.
///
/// `jac_proj = [[1/z, 0, -x/z²], [0, 1/z, -y/z²]]` is sparse; exploiting that
/// saves one full 2×3 × 3×3 matrix multiply and one extra `1/z` division compared
/// to calling `project_to_normalized` and `jacobian_proj_wrt_p_C` separately.
///
/// Returns `None` when `z ≤ MIN_DEPTH`.
#[inline(always)]
fn project_and_jac_mul(
    p_C: Vector3<f64>,
    mat: &Matrix3<f64>,
) -> Option<(Vector2<f64>, na::Matrix2x3<f64>)> {
    let z = p_C[2];
    if z <= MIN_DEPTH {
        return None;
    }
    let inv_z = 1.0 / z;
    let inv_z_sq = inv_z * inv_z;
    let xz2 = p_C[0] * inv_z_sq;
    let yz2 = p_C[1] * inv_z_sq;
    let proj = Vector2::new(p_C[0] * inv_z, p_C[1] * inv_z);
    // Row r of (jac_proj × mat): inv_z * mat.row(r) - [xz2 or yz2] * mat.row(2)
    let jac = na::Matrix2x3::new(
        inv_z * mat[(0, 0)] - xz2 * mat[(2, 0)],
        inv_z * mat[(0, 1)] - xz2 * mat[(2, 1)],
        inv_z * mat[(0, 2)] - xz2 * mat[(2, 2)],
        inv_z * mat[(1, 0)] - yz2 * mat[(2, 0)],
        inv_z * mat[(1, 1)] - yz2 * mat[(2, 1)],
        inv_z * mat[(1, 2)] - yz2 * mat[(2, 2)],
    );
    Some((proj, jac))
}

/// Compute `-(mat × skew(v))` exploiting the 9 zero entries in the skew-symmetric matrix.
///
/// `skew(v) = [[0, -vz, vy], [vz, 0, -vx], [-vy, vx, 0]]`
///
/// Saves 6 multiplies and 6 additions vs. building the full 3×3 skew matrix and
/// doing a standard 2×3 × 3×3 multiply (18 muls + 12 adds → 12 muls + 6 adds).
#[inline(always)]
fn mat2x3_neg_mul_skew(m: &na::Matrix2x3<f64>, v: &Vector3<f64>) -> na::Matrix2x3<f64> {
    let (vx, vy, vz) = (v[0], v[1], v[2]);
    // (mat × skew(v))_{r,0} =  m[r,1]*vz - m[r,2]*vy  → negated: m[r,2]*vy - m[r,1]*vz
    // (mat × skew(v))_{r,1} = -m[r,0]*vz + m[r,2]*vx  → negated: m[r,0]*vz - m[r,2]*vx
    // (mat × skew(v))_{r,2} =  m[r,0]*vy - m[r,1]*vx  → negated: m[r,1]*vx - m[r,0]*vy
    na::Matrix2x3::new(
        m[(0, 2)] * vy - m[(0, 1)] * vz,
        m[(0, 0)] * vz - m[(0, 2)] * vx,
        m[(0, 1)] * vx - m[(0, 0)] * vy,
        m[(1, 2)] * vy - m[(1, 1)] * vz,
        m[(1, 0)] * vz - m[(1, 2)] * vx,
        m[(1, 1)] * vx - m[(1, 0)] * vy,
    )
}

/// Extract `(R_B_W, t_B_W)` directly from a SE(3) parameter `DVector<f64>` without cloning it.
///
/// Layout: `[tx, ty, tz, qw, qi, qj, qk]` (matches `initial_values` construction in optimize()).
/// Avoids `SE3::from(params.clone())` which heap-allocates a new DVector for ownership.
#[inline(always)]
fn r_t_from_se3_params(p: &DVector<f64>) -> (Matrix3<f64>, Vector3<f64>) {
    let t = Vector3::new(p[0], p[1], p[2]);
    let q = na::UnitQuaternion::from_quaternion(
        Quaternion::new(p[3], p[4], p[5], p[6]).normalize(),
    );
    (q.to_rotation_matrix().into_inner(), t)
}

/// Skew-symmetric matrix of a 3D vector for cross-product: `[v]× * u = v × u`.
#[inline(always)]
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
        let p_C = R_C_W * point_world + t_C_W;

        if !compute_jacobian {
            let Some(proj) = project_to_normalized(p_C) else {
                return (DVector::zeros(2), None);
            };
            let residuals = DVector::from_row_slice(&[
                proj[0] - self.observation[0],
                proj[1] - self.observation[1],
            ]);
            return (residuals, None);
        }

        let R_C_W_owned: Matrix3<f64> = R_C_W.into_owned();
        let Some((proj, jac)) = project_and_jac_mul(p_C, &R_C_W_owned) else {
            // Point behind camera: zero contribution (don't fail the solve).
            return (DVector::zeros(2), Some(DMatrix::zeros(2, 3)));
        };
        let residuals = DVector::from_row_slice(&[
            proj[0] - self.observation[0],
            proj[1] - self.observation[1],
        ]);
        let mut j = DMatrix::zeros(2, 3);
        j.copy_from(&jac);
        (residuals, Some(j))
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

        if !compute_jacobian {
            let Some(proj) = project_to_normalized(p_C) else {
                return (DVector::zeros(2), None);
            };
            let residuals = DVector::from_row_slice(&[
                proj[0] - self.observation[0],
                proj[1] - self.observation[1],
            ]);
            return (residuals, None);
        }

        let R_C_B_owned: Matrix3<f64> = R_C_B.into_owned();
        let Some((proj, jac_r_p_W)) = project_and_jac_mul(p_C, &R_C_B_owned) else {
            // Point behind camera: zero contribution.
            let ncols = if self.fixed_position.is_some() { 3 } else { 6 };
            return (DVector::zeros(2), Some(DMatrix::zeros(2, ncols)));
        };
        let residuals = DVector::from_row_slice(&[
            proj[0] - self.observation[0],
            proj[1] - self.observation[1],
        ]);

        let jacobian = if self.fixed_position.is_some() {
            let mut j = DMatrix::zeros(2, 3);
            j.copy_from(&jac_r_p_W);
            Some(j)
        } else {
            let mut j = DMatrix::zeros(2, 6);
            j.view_mut((0, 0), (2, 3)).copy_from(&jac_r_p_W);
            j.view_mut((0, 3), (2, 3)).copy_from(&jac_r_p_W); // ∂r/∂t same as ∂r/∂p_W here
            Some(j)
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
///
/// # Age-weighted optimization
/// When `track_age > 0`, residuals are weighted by `1 + 0.1 * min(age, 10)`.
/// Older (more reliable) features contribute more to the solution.
#[derive(Debug, Clone)]
pub struct BundleAdjustmentFactor {
    /// Observed 2D point in normalized/undistorted coordinates.
    pub observation: Vector2<f64>,
    /// Rotation part of T_C_B (body → camera). Stored split from translation
    /// to avoid re-extracting fixed_view on every linearize() call and to
    /// reduce struct size: 96 B (R+t) vs 256 B (Matrix4).
    r_c_b: Matrix3<f64>,
    /// Translation part of T_C_B.
    t_c_b: Vector3<f64>,
    /// Fixed pose `(R_B_W, t_B_W)` if provided; `None` means the pose is optimized.
    fixed_pose: Option<(Matrix3<f64>, Vector3<f64>)>,
    /// Number of frames this feature has been tracked. Used for age-weighted BA.
    pub track_age: u32,
}

impl BundleAdjustmentFactor {
    /// Create a new bundle adjustment factor.
    pub fn new(observation: Vector2<f64>, T_C_B: Matrix4<f64>) -> Self {
        Self {
            observation,
            r_c_b: T_C_B.fixed_view::<3, 3>(0, 0).into_owned(),
            t_c_b: T_C_B.fixed_view::<3, 1>(0, 3).into_owned(),
            fixed_pose: None,
            track_age: 0,
        }
    }

    /// Set a fixed pose `T_B_W`. When set, only the 3D point is optimized.
    #[must_use]
    pub fn with_fixed_pose(mut self, T_B_W: Matrix4<f64>) -> Self {
        self.fixed_pose = Some((
            T_B_W.fixed_view::<3, 3>(0, 0).into_owned(),
            T_B_W.fixed_view::<3, 1>(0, 3).into_owned(),
        ));
        self
    }

    /// Set the track age for age-weighted optimization.
    /// Older features (higher age) get higher weight, improving stability.
    #[must_use]
    pub fn with_track_age(mut self, age: u32) -> Self {
        self.track_age = age;
        self
    }

    /// Compute the square-root weight for this residual based on track age.
    ///
    /// Applying `sqrt(w)` to both residual and Jacobian yields a system weighting of
    /// `(sqrt(w)*J)ᵀ(sqrt(w)*J) = w*JᵀJ`, giving older features `w`-times more influence
    /// in the normal equations.  Using the full `w` would cancel in the normal equations
    /// (leaving zero benefit) while doubling the residual magnitude fed to the Huber loss —
    /// wrongly classifying good old features as outliers.
    ///
    /// Range: `sqrt(1.0)` to `sqrt(2.0)` ≈ [1.00, 1.41].
    #[inline(always)]
    fn age_weight(&self) -> f64 {
        (1.0 + 0.1 * (self.track_age as f64).min(10.0)).sqrt()
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
        let (R_B_W, t_B_W) = if let Some((r, t)) = self.fixed_pose {
            assert_eq!(params.len(), 1, "BA with fixed pose requires 1 parameter");
            (r, t)
        } else {
            assert_eq!(params.len(), 2, "BA requires 2 parameter vectors");
            assert_eq!(params[1].len(), 7, "SE(3) pose must have 7 parameters");
            // Direct extraction avoids SE3::from(params[1].clone()) heap alloc.
            r_t_from_se3_params(&params[1])
        };

        let R_C_B = self.r_c_b;
        let t_C_B = self.t_c_b;

        // Transform: p_W -> p_B -> p_C
        let p_B = R_B_W * p_W + t_B_W;
        let p_C = R_C_B * p_B + t_C_B;

        // **Age-weighted residuals**: older features (higher track_age) get higher weight
        let w = self.age_weight();

        if !compute_jacobian {
            let Some(proj) = project_to_normalized(p_C) else {
                return (DVector::zeros(2), None);
            };
            let residuals = DVector::from_row_slice(&[
                w * (proj[0] - self.observation[0]),
                w * (proj[1] - self.observation[1]),
            ]);
            return (residuals, None);
        }

        // Precompute R_C_W = R_C_B * R_B_W so project_and_jac_mul needs only one pass.
        let R_C_W = R_C_B * R_B_W;
        let Some((proj, jac_r_p_W)) = project_and_jac_mul(p_C, &R_C_W) else {
            // Point behind camera: zero contribution (don't fail the solve).
            let ncols = if self.fixed_pose.is_some() { 3 } else { 9 };
            return (DVector::zeros(2), Some(DMatrix::zeros(2, ncols)));
        };
        let residuals = DVector::from_row_slice(&[
            w * (proj[0] - self.observation[0]),
            w * (proj[1] - self.observation[1]),
        ]);

        let jacobian = if self.fixed_pose.is_some() {
            let mut j = DMatrix::zeros(2, 3);
            j.copy_from(&(jac_r_p_W * w));
            Some(j)
        } else {
            // SE3 right-multiplication: T_B_W * Exp(δξ), δξ = (δv, δω)
            // ∂r/∂p_W = ∂r/∂t_B_W = w · jac_proj · R_C_B · R_B_W = w · jac_r_p_W
            // ∂r/∂ω   = -w · jac_r_p_W · skew(p_W)  (via mat2x3_neg_mul_skew)
            let jac_r_rot_w = mat2x3_neg_mul_skew(&jac_r_p_W, &p_W) * w;
            let jac_r_p_W_w = jac_r_p_W * w;
            let mut j = DMatrix::zeros(2, 9);
            j.view_mut((0, 0), (2, 3)).copy_from(&jac_r_p_W_w);   // ∂r/∂p_W
            j.view_mut((0, 3), (2, 3)).copy_from(&jac_r_p_W_w);   // ∂r/∂t_B_W
            j.view_mut((0, 6), (2, 3)).copy_from(&jac_r_rot_w);   // ∂r/∂ω
            Some(j)
        };

        (residuals, jacobian)
    }

    fn get_dimension(&self) -> usize {
        2
    }
}

// ============================================================================
// BundleAdjustmentFactorWithStereoCalib
// ============================================================================

/// Bundle adjustment factor that also optimizes stereo baseline calibration (T_B_Cr).
///
/// This factor is used for stereo observations to allow online calibration of the
/// stereo camera extrinsics. It optimizes the 3D point, camera pose, AND the stereo
/// baseline transformation that maps points from body frame to right camera frame.
///
/// # Variables
/// - 3D point in world frame (3 params: p_W)
/// - System pose `T_B_W` as SE(3) (7 params: tx, ty, tz, qw, qx, qy, qz), unless fixed
/// - Stereo baseline `T_B_Cr` as SE(3) (7 params: tx, ty, tz, qw, qx, qy, qz)
///
/// The stereo baseline variable allows the optimizer to correct for miscalibration
/// in the stereo camera separation.
#[derive(Debug, Clone)]
pub struct BundleAdjustmentFactorWithStereoCalib {
    /// Observed 2D point in normalized/undistorted coordinates.
    pub observation: Vector2<f64>,
    /// Rotation part of T_C_B for the camera observing this point (body → camera).
    r_c_b: Matrix3<f64>,
    /// Translation part of T_C_B.
    t_c_b: Vector3<f64>,
    /// Whether this is observed from right camera (stereo).
    /// If true, the factor uses the optimized T_B_Cr to compute effective T_C_B.
    /// If false, uses the nominal T_B_Cl.
    is_right_camera: bool,
    /// Fixed pose `(R_B_W, t_B_W)` if provided; `None` means the pose is optimized.
    fixed_pose: Option<(Matrix3<f64>, Vector3<f64>)>,
    /// Number of frames this feature has been tracked.
    pub track_age: u32,
}

impl BundleAdjustmentFactorWithStereoCalib {
    /// Create a new bundle adjustment factor with stereo calibration optimization.
    pub fn new(
        observation: Vector2<f64>,
        T_C_B: Matrix4<f64>,
        is_right_camera: bool,
    ) -> Self {
        Self {
            observation,
            r_c_b: T_C_B.fixed_view::<3, 3>(0, 0).into_owned(),
            t_c_b: T_C_B.fixed_view::<3, 1>(0, 3).into_owned(),
            is_right_camera,
            fixed_pose: None,
            track_age: 0,
        }
    }

    /// Set a fixed pose `T_B_W`. When set, only the 3D point is optimized.
    #[must_use]
    pub fn with_fixed_pose(mut self, T_B_W: Matrix4<f64>) -> Self {
        self.fixed_pose = Some((
            T_B_W.fixed_view::<3, 3>(0, 0).into_owned(),
            T_B_W.fixed_view::<3, 1>(0, 3).into_owned(),
        ));
        self
    }

    /// Set the track age for age-weighted optimization.
    #[must_use]
    pub fn with_track_age(mut self, age: u32) -> Self {
        self.track_age = age;
        self
    }

    /// Compute the square-root weight for this residual based on track age.
    #[inline(always)]
    fn age_weight(&self) -> f64 {
        (1.0 + 0.1 * (self.track_age as f64).min(10.0)).sqrt()
    }
}

impl Factor for BundleAdjustmentFactorWithStereoCalib {
    #![allow(non_snake_case)]
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        let p_W = Vector3::new(params[0][0], params[0][1], params[0][2]);

        // Extract T_B_W
        let (R_B_W, t_B_W, pose_idx) = if let Some((r, t)) = self.fixed_pose {
            assert_eq!(params.len(), 2, "BA with fixed pose + stereo calib requires 2 parameters");
            (r, t, None)
        } else {
            assert_eq!(params.len(), 3, "BA with stereo calib requires 3 parameter vectors");
            assert_eq!(params[1].len(), 7, "SE(3) pose must have 7 parameters");
            let (r, t) = r_t_from_se3_params(&params[1]);
            (r, t, Some(1))
        };

        // Extract T_B_Cr (stereo baseline) - always optimized
        assert_eq!(params.last().unwrap().len(), 7, "SE(3) T_B_Cr must have 7 parameters");
        let (R_B_Cr, t_B_Cr) = r_t_from_se3_params(params.last().unwrap());

        // Compute effective R_C_B, t_C_B using the optimized T_B_Cr for right camera
        let (R_C_B, t_C_B, is_right_optimized) = if self.is_right_camera {
            // T_Cr_B = inv(T_B_Cr), then use it as the camera transform
            // T_Cr_B * p_B gives p_Cr
            // So R_C_B = R_Cr_B and t_C_B = -R_Cr_B * t_B_Cr
            let R_Cr_B = R_B_Cr.transpose();
            let t_Cr_B = -R_Cr_B * t_B_Cr;
            (R_Cr_B, t_Cr_B, true)
        } else {
            // Left camera uses nominal calibration
            (self.r_c_b, self.t_c_b, false)
        };

        // Transform: p_W -> p_B -> p_C
        let p_B = R_B_W * p_W + t_B_W;
        let p_C = R_C_B * p_B + t_C_B;

        let w = self.age_weight();

        if !compute_jacobian {
            let Some(proj) = project_to_normalized(p_C) else {
                return (DVector::zeros(2), None);
            };
            let residuals = DVector::from_row_slice(&[
                w * (proj[0] - self.observation[0]),
                w * (proj[1] - self.observation[1]),
            ]);
            return (residuals, None);
        }

        // With Jacobian
        let R_C_W = R_C_B * R_B_W;
        let Some((proj, jac_r_p_W)) = project_and_jac_mul(p_C, &R_C_W) else {
            let ncols = if self.fixed_pose.is_some() { 7 } else { 14 };
            return (DVector::zeros(2), Some(DMatrix::zeros(2, ncols)));
        };

        let residuals = DVector::from_row_slice(&[
            w * (proj[0] - self.observation[0]),
            w * (proj[1] - self.observation[1]),
        ]);

        // Build Jacobian: dimensions are w.r.t. manifold PERTURBATIONS, not full parameters
        // p_W: 3D (no manifold, just R³)
        // pose (SE(3) if optimized): 6D perturbation (3 translation + 3 rotation)
        // T_B_Cr (SE(3)): 6D perturbation (3 translation + 3 rotation)
        // Fixed pose: 3 + 6 = 9 cols
        // Not fixed: 3 + 6 + 6 = 15 cols
        let ncols = if self.fixed_pose.is_some() { 9 } else { 15 };
        let mut j = DMatrix::zeros(2, ncols);

        // ∂r/∂p_W: always present
        j.view_mut((0, 0), (2, 3)).copy_from(&(jac_r_p_W * w));

        if pose_idx.is_some() {
            // ∂r/∂t_B_W
            let jac_r_p_W_w = jac_r_p_W * w;
            j.view_mut((0, 3), (2, 3)).copy_from(&jac_r_p_W_w);

            // ∂r/∂ω (rotation perturbation)
            let jac_r_rot_w = mat2x3_neg_mul_skew(&jac_r_p_W, &p_W) * w;
            j.view_mut((0, 6), (2, 3)).copy_from(&jac_r_rot_w);
        }

        // ∂r/∂T_B_Cr (right camera only, zeros for left camera)
        if is_right_optimized {
            // When fixed_pose: [p_W (3) | T_B_Cr (7)] => T_B_Cr starts at col 3
            // When not fixed: [p_W (3) | pose (6) | T_B_Cr (7)] => T_B_Cr starts at col 9
            let cr_calib_col_start = if self.fixed_pose.is_some() { 3 } else { 9 };

            // ∂r/∂t_B_Cr: use chain rule
            // p_C = R_C_B * (R_B_W * p_W + t_B_W) + t_C_B
            // where R_C_B = R_Cr_B and t_C_B = -R_Cr_B * t_B_Cr
            // ∂p_C/∂t_B_Cr = -R_C_B (since ∂t_C_B/∂t_B_Cr = -R_C_B)
            let jac_p_C_t_Cr = -R_C_B;
            let jac_r_t_Cr = jac_r_p_W * jac_p_C_t_Cr * w;
            j.view_mut((0, cr_calib_col_start), (2, 3)).copy_from(&jac_r_t_Cr);

            // ∂r/∂ω_Cr (rotation perturbation of T_B_Cr)
            // Approximate using p_B sensitivity
            let jac_r_rot_Cr = mat2x3_neg_mul_skew(&jac_r_p_W, &p_B) * w;
            j.view_mut((0, cr_calib_col_start + 3), (2, 3)).copy_from(&jac_r_rot_Cr);
        }

        (residuals, Some(j))
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
/// # Age-weighted optimization
/// When `track_age > 0`, residuals are weighted by `1 + 0.1 * min(age, 10)`.
/// Older (more reliable) features contribute more to the solution.
#[derive(Debug, Clone)]
pub struct PnPFactor {
    /// Observed 2D point in normalized/undistorted coordinates.
    pub observation: Vector2<f64>,
    /// Rotation part of T_C_B. Stored split to avoid per-linearize fixed_view extractions.
    r_c_b: Matrix3<f64>,
    /// Translation part of T_C_B.
    t_c_b: Vector3<f64>,
    /// Known 3D point in world frame.
    pub p_W: Vector3<f64>,
    /// Number of frames this feature has been tracked. Used for age-weighted PnP.
    pub track_age: u32,
}

impl PnPFactor {
    /// Create a new PnP factor.
    pub fn new(observation: Vector2<f64>, T_C_B: Matrix4<f64>, p_W: Vector3<f64>) -> Self {
        Self {
            observation,
            r_c_b: T_C_B.fixed_view::<3, 3>(0, 0).into_owned(),
            t_c_b: T_C_B.fixed_view::<3, 1>(0, 3).into_owned(),
            p_W,
            track_age: 0,
        }
    }

    /// Set the track age for age-weighted optimization.
    #[must_use]
    pub fn with_track_age(mut self, age: u32) -> Self {
        self.track_age = age;
        self
    }

    /// Square-root weight for this residual based on track age (same reasoning as
    /// `BundleAdjustmentFactor::age_weight`).  Range: [1.00, 1.41].
    #[inline(always)]
    fn age_weight(&self) -> f64 {
        (1.0 + 0.1 * (self.track_age as f64).min(10.0)).sqrt()
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

        // Direct extraction avoids SE3::from(params[0].clone()) heap alloc.
        let (R_B_W, t_B_W) = r_t_from_se3_params(&params[0]);

        let R_C_B = self.r_c_b;
        let t_C_B = self.t_c_b;

        // Transform: p_W -> p_B -> p_C
        let p_B = R_B_W * self.p_W + t_B_W;
        let p_C = R_C_B * p_B + t_C_B;

        let w = self.age_weight();

        if !compute_jacobian {
            let Some(proj) = project_to_normalized(p_C) else {
                return (DVector::zeros(2), None);
            };
            let residuals = DVector::from_row_slice(&[
                w * (proj[0] - self.observation[0]),
                w * (proj[1] - self.observation[1]),
            ]);
            return (residuals, None);
        }

        // Precompute R_C_W = R_C_B * R_B_W so project_and_jac_mul needs only one pass.
        let R_C_W = R_C_B * R_B_W;
        let Some((proj, jac_r_p_W)) = project_and_jac_mul(p_C, &R_C_W) else {
            // Point behind camera: zero contribution (don't fail the solve).
            return (DVector::zeros(2), Some(DMatrix::zeros(2, 6)));
        };
        let residuals = DVector::from_row_slice(&[
            w * (proj[0] - self.observation[0]),
            w * (proj[1] - self.observation[1]),
        ]);

        // [∂r/∂t (2x3) | ∂r/∂ω (2x3)]
        // ∂r/∂ω = -jac_r_p_W * skew(p_W)  (via mat2x3_neg_mul_skew)
        let jac_r_rot_w = mat2x3_neg_mul_skew(&jac_r_p_W, &self.p_W) * w;
        let mut j = DMatrix::zeros(2, 6);
        j.view_mut((0, 0), (2, 3)).copy_from(&(jac_r_p_W * w)); // ∂r/∂t
        j.view_mut((0, 3), (2, 3)).copy_from(&jac_r_rot_w);     // ∂r/∂ω
        (residuals, Some(j))
    }

    fn get_dimension(&self) -> usize {
        2
    }
}

// ============================================================================
// LoopClosureEdgeFactor
// ============================================================================

/// Loop closure edge factor: constrains the relative pose between two keyframes.
///
/// # Variables
/// - Pose 1 `T_B1_W` as SE(3) (7 params: tx, ty, tz, qw, qx, qy, qz)
/// - Pose 2 `T_B2_W` as SE(3) (7 params: tx, ty, tz, qw, qx, qy, qz)
/// - Optional: scale parameter (1 param) if `optimize_scale = true`
///
/// # Fixed
/// - Loop closure constraint: relative rotation `R_rel`, relative translation direction `t_rel_unit`
/// - Initial scale estimate `scale_init`
///
/// # Residual
/// 6-component residual:
/// - 3 components: rotation error (log-map of rotation difference)
/// - 3 components: translation error (measured vs expected scaled translation)
#[derive(Debug, Clone)]
pub struct LoopClosureEdgeFactor {
    /// Relative rotation from loop closure (R_2_1)
    r_rel: Matrix3<f64>,
    /// Relative translation direction (unit norm) from loop closure
    t_rel_unit: Vector3<f64>,
    /// Initial scale estimate from loop closure
    scale_init: f64,
    /// Whether to optimize scale as a variable
    optimize_scale: bool,
}

impl LoopClosureEdgeFactor {
    /// Create a new loop closure edge factor.
    ///
    /// # Arguments
    /// * `r_rel` - Relative rotation matrix from loop closure validation
    /// * `t_rel_unit` - Unit-norm relative translation direction
    /// * `scale` - Estimated scale of the translation
    /// * `optimize_scale` - If true, scale is treated as an optimization variable
    pub fn new(r_rel: Matrix3<f64>, t_rel_unit: Vector3<f64>, scale: f64, optimize_scale: bool) -> Self {
        Self {
            r_rel,
            t_rel_unit,
            scale_init: scale,
            optimize_scale,
        }
    }
}

impl Factor for LoopClosureEdgeFactor {
    #![allow(non_snake_case)]
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        // Parse parameters based on whether we're optimizing scale
        let (R_B1_W, t_B1_W, scale) = if self.optimize_scale {
            assert_eq!(params.len(), 3, "LoopClosureEdge with scale requires 3 parameters");
            assert_eq!(params[0].len(), 7, "Pose 1 must have 7 parameters");
            assert_eq!(params[1].len(), 7, "Pose 2 must have 7 parameters");
            assert_eq!(params[2].len(), 1, "Scale must have 1 parameter");

            let (R1, t1) = r_t_from_se3_params(&params[0]);
            let (_, _) = r_t_from_se3_params(&params[1]); // Only used for computing relative pose
            (R1, t1, params[2][0].max(0.01)) // Clamp scale to minimum 0.01
        } else {
            assert_eq!(params.len(), 2, "LoopClosureEdge without scale requires 2 parameters");
            assert_eq!(params[0].len(), 7, "Pose 1 must have 7 parameters");
            assert_eq!(params[1].len(), 7, "Pose 2 must have 7 parameters");

            let (R1, t1) = r_t_from_se3_params(&params[0]);
            let (_, _) = r_t_from_se3_params(&params[1]);
            (R1, t1, self.scale_init)
        };

        // Extract second pose
        let (R_B2_W, t_B2_W) = r_t_from_se3_params(&params[1]);

        // Compute measured relative pose: T_B2_B1 = inv(T_B1_W) * T_B2_W
        // In body frame: p_B1 = R_B1_W * p_W + t_B1_W, so inv gives:
        // p_W = R_B1_W^T * (p_B1 - t_B1_W)
        // Therefore: p_B2 = R_B2_W * p_W + t_B2_W = R_B2_W * R_B1_W^T * (p_B1 - t_B1_W) + t_B2_W
        let R_B1_W_T = R_B1_W.transpose();
        let R_measured = R_B2_W * R_B1_W_T;
        let t_measured = t_B2_W - R_B2_W * R_B1_W_T * t_B1_W;

        // Compute residuals
        let mut residuals = DVector::zeros(6);

        // Rotation residual: log-map of (R_measured^T * R_rel)
        // The log-map gives a 3D vector whose norm is the rotation angle
        // and direction is the rotation axis
        let R_error = R_measured.transpose() * self.r_rel;
        let (ax, angle) = rotation_to_axis_angle(&R_error);
        residuals[0] = ax[0] * angle;
        residuals[1] = ax[1] * angle;
        residuals[2] = ax[2] * angle;

        // Translation residual: measured vs expected scaled translation
        // Expected: scale * t_rel_unit (direction-scaled magnitude)
        let t_expected = scale * self.t_rel_unit;
        let t_error = t_measured - t_expected;
        residuals[3] = t_error[0];
        residuals[4] = t_error[1];
        residuals[5] = t_error[2];

        if !compute_jacobian {
            return (residuals, None);
        }

        // Compute Jacobians
        let ncols = if self.optimize_scale { 15 } else { 14 };
        let mut jac = DMatrix::zeros(6, ncols);

        // Numerical differentiation for simplicity (can be optimized later)
        let eps = 1e-8;

        // Jacobian w.r.t. pose 1 (7 parameters)
        for i in 0..7 {
            let mut params_pert = params.to_vec();
            params_pert[0][i] += eps;

            let (residuals_pert, _) = self.linearize(&params_pert, false);
            let jac_col = (residuals_pert - &residuals) / eps;
            for j in 0..6 {
                jac[(j, i)] = jac_col[j];
            }
        }

        // Jacobian w.r.t. pose 2 (7 parameters)
        for i in 0..7 {
            let mut params_pert = params.to_vec();
            params_pert[1][i] += eps;

            let (residuals_pert, _) = self.linearize(&params_pert, false);
            let jac_col = (residuals_pert - &residuals) / eps;
            for j in 0..6 {
                jac[(j, 7 + i)] = jac_col[j];
            }
        }

        // Jacobian w.r.t. scale if optimizing
        if self.optimize_scale {
            let mut params_pert = params.to_vec();
            params_pert[2][0] += eps;

            let (residuals_pert, _) = self.linearize(&params_pert, false);
            let jac_col = (residuals_pert - &residuals) / eps;
            for j in 0..6 {
                jac[(j, 14)] = jac_col[j];
            }
        }

        (residuals, Some(jac))
    }

    fn get_dimension(&self) -> usize {
        6
    }
}

/// Convert rotation matrix to axis-angle representation.
/// Returns (unit_axis, angle_in_radians).
#[inline(always)]
fn rotation_to_axis_angle(R: &Matrix3<f64>) -> (Vector3<f64>, f64) {
    // angle = arccos((trace(R) - 1) / 2)
    let trace = R[(0, 0)] + R[(1, 1)] + R[(2, 2)];
    let cos_angle = (trace - 1.0) / 2.0;
    let cos_angle = cos_angle.clamp(-1.0, 1.0); // Clamp for numerical stability
    let angle = cos_angle.acos();

    // For small angles, use series expansion to avoid numerical issues
    if angle.abs() < 1e-6 {
        // For small angles: sin(θ)/θ ≈ 1, so axis ≈ [R32-R23, R13-R31, R21-R12] / 2
        let axis = Vector3::new(
            (R[(2, 1)] - R[(1, 2)]) / 2.0,
            (R[(0, 2)] - R[(2, 0)]) / 2.0,
            (R[(1, 0)] - R[(0, 1)]) / 2.0,
        );
        return (axis.normalize(), angle);
    }

    // For regular angles: extract axis from skew part
    let sin_angle = angle.sin();
    let axis = Vector3::new(
        (R[(2, 1)] - R[(1, 2)]) / (2.0 * sin_angle),
        (R[(0, 2)] - R[(2, 0)]) / (2.0 * sin_angle),
        (R[(1, 0)] - R[(0, 1)]) / (2.0 * sin_angle),
    );

    (axis.normalize(), angle)
}

// ============================================================================
// StereoCalibraPriorFactor
// ============================================================================

/// Prior factor that regularizes the stereo baseline calibration T_B_Cr.
///
/// This factor penalizes deviation from the nominal/reference stereo calibration,
/// preventing the optimizer from drifting too far from the factory calibration.
/// It acts as a soft constraint that keeps T_B_Cr close to its nominal value
/// with a configurable strength.
///
/// # Variables
/// - Stereo baseline `T_B_Cr` as SE(3) (7 params: tx, ty, tz, qw, qx, qy, qz)
///
/// The cost is: weight * ||T_B_Cr - T_B_Cr_nominal||²
#[derive(Debug, Clone)]
pub struct StereoCalibraPriorFactor {
    /// Nominal (reference) stereo calibration.
    nominal_T_B_Cr: Matrix4<f64>,
    /// Weight of this prior. Higher weight keeps T_B_Cr closer to nominal.
    /// Typical value: 0.1-1.0 depending on confidence in factory calibration.
    weight: f64,
}

impl StereoCalibraPriorFactor {
    /// Create a new prior factor for stereo calibration.
    pub fn new(nominal_T_B_Cr: Matrix4<f64>, weight: f64) -> Self {
        Self {
            nominal_T_B_Cr,
            weight,
        }
    }
}

impl Factor for StereoCalibraPriorFactor {
    #![allow(non_snake_case)]
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        assert_eq!(params.len(), 1, "Stereo calib prior requires 1 parameter (T_B_Cr)");
        assert_eq!(params[0].len(), 7, "T_B_Cr must be SE(3) with 7 parameters");

        // Extract current T_B_Cr
        let (R_current, t_current) = r_t_from_se3_params(&params[0]);

        // Extract nominal values
        let R_nominal = self.nominal_T_B_Cr.fixed_view::<3, 3>(0, 0).into_owned();
        let t_nominal = self.nominal_T_B_Cr.fixed_view::<3, 1>(0, 3).into_owned();

        // Compute perturbation
        let dt = t_current - t_nominal;
        let dR = R_current * R_nominal.transpose();

        // Extract rotation axis and angle for regularization
        let (axis, angle) = rotation_to_axis_angle(&dR);

        // Weighted residuals: translation + rotation
        let w_sqrt = self.weight.sqrt();
        let residuals = DVector::from_row_slice(&[
            w_sqrt * dt.x,
            w_sqrt * dt.y,
            w_sqrt * dt.z,
            w_sqrt * axis.x * angle,
            w_sqrt * axis.y * angle,
            w_sqrt * axis.z * angle,
        ]);

        if !compute_jacobian {
            return (residuals, None);
        }

        // Compute Jacobian using finite differences for simplicity
        // Since this is just a regularization prior, approximate Jacobian is sufficient
        let eps = 1e-8;
        let mut jac = DMatrix::zeros(6, 7);

        for col in 0..7 {
            let mut params_pert = params[0].clone();
            params_pert[col] += eps;

            let (R_pert, t_pert) = r_t_from_se3_params(&params_pert);
            let dR_pert = R_pert * R_nominal.transpose();
            let (axis_pert, angle_pert) = rotation_to_axis_angle(&dR_pert);
            let dt_pert = t_pert - t_nominal;

            let res_pert = DVector::from_row_slice(&[
                w_sqrt * dt_pert.x,
                w_sqrt * dt_pert.y,
                w_sqrt * dt_pert.z,
                w_sqrt * axis_pert.x * angle_pert,
                w_sqrt * axis_pert.y * angle_pert,
                w_sqrt * axis_pert.z * angle_pert,
            ]);

            for row in 0..6 {
                jac[(row, col)] = (res_pert[row] - residuals[row]) / eps;
            }
        }

        (residuals, Some(jac))
    }

    fn get_dimension(&self) -> usize {
        6
    }
}
