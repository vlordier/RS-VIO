use apex_solver::factors::Factor;
use na::{DMatrix, DVector, Matrix4, Quaternion, UnitQuaternion, Vector2, Vector3};
use nalgebra as na;
use std::sync::Arc;

// ─── Shared projection helpers (used by all visual factors) ───

/// Project a 3D point in camera frame to normalized coordinates: `[x/z, y/z]`.
///
/// Caller must ensure `p_C.z > 0` (cheirality).
#[inline]
fn project_normalized(p_C: Vector3<f64>) -> Vector2<f64> {
    Vector2::new(p_C[0] / p_C[2], p_C[1] / p_C[2])
}

/// Jacobian of normalized pinhole projection w.r.t. 3D point in camera frame.
///
/// `∂[x/z, y/z] / ∂[x, y, z]`
///
/// Caller must ensure `p_C.z > 0`.
#[inline]
fn jacobian_proj_wrt_p_C(p_C: Vector3<f64>) -> na::Matrix2x3<f64> {
    let inv_z = 1.0 / p_C[2];
    let inv_z_sq = inv_z * inv_z;
    na::Matrix2x3::new(
        inv_z,
        0.0,
        -p_C[0] * inv_z_sq,
        0.0,
        inv_z,
        -p_C[1] * inv_z_sq,
    )
}

/// Pinhole projection factor for optimizing 3D point positions from camera observations.
///
/// This factor computes the reprojection error for a 3D point observed in a camera.
/// It optimizes only the 3D point position, with camera pose held fixed.
///
/// - Variables: 3D point in world/camera frame (3 params: x, y, z)
/// - Fixed parameters: Camera pose (T_world_to_camera, 4x4 matrix),
///   Observation (2D normalized/undistorted)
///
/// The residual is 2D: [u, v] in normalized coordinates
///
/// # Mathematical Formulation
///
/// Given a 3D point `X` and camera pose `T_cam_world`, the residual is:
///
/// ```text
/// r = proj(T_cam_world * X) - obs
/// ```
///
/// where `proj` is the pinhole projection to normalized coordinates: `[x/z, y/z]`
#[derive(Debug, Clone)]
pub struct PinholeProjectionFactor {
    /// Observed 2D point in camera (normalized/undistorted coordinates: x, y)
    pub observation: Vector2<f64>,

    /// Transform from world to camera frame (T_world_to_camera, 4x4 matrix)
    pub T_C_W: Matrix4<f64>,
}

impl PinholeProjectionFactor {
    /// Create a new pinhole projection factor.
    ///
    /// # Arguments
    /// * `observation` - Observed 2D point in camera (normalized/undistorted: x, y)
    /// * `t_world_to_camera` - Transform from world to camera frame (4x4 matrix)
    pub const fn new(observation: Vector2<f64>, T_C_W: Matrix4<f64>) -> Self {
        Self { observation, T_C_W }
    }
}

impl Factor for PinholeProjectionFactor {
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        // params[0] = 3D point in world frame (3 params: x, y, z)
        assert_eq!(
            params.len(),
            1,
            "PinholeProjectionFactor requires 1 parameter vector"
        );
        assert_eq!(params[0].len(), 3, "3D point must have 3 parameters");

        let point_world = Vector3::new(params[0][0], params[0][1], params[0][2]);

        // Transform 3D point from world to camera frame
        let R_C_W = self.T_C_W.fixed_view::<3, 3>(0, 0);
        let t_C_W = self.T_C_W.fixed_view::<3, 1>(0, 3);
        let point_camera = R_C_W * point_world + t_C_W;

        // Cheirality check: point must be in front of camera
        if point_camera.z <= 1e-6 {
            let residuals = DVector::from_column_slice(&[1e6, 1e6]);
            if compute_jacobian {
                // Gradient pushes the 3D point toward positive camera z in world frame
                let mut jac = DMatrix::zeros(2, 3);
                // Direction that increases p_C.z w.r.t. p_W is R_C_W[2, :]
                // Negate so GN descent moves point *in front of* camera
                for c in 0..3 {
                    jac[(0, c)] = -R_C_W[(2, c)] * 1e3;
                    jac[(1, c)] = -R_C_W[(2, c)] * 1e3;
                }
                return (residuals, Some(jac));
            }
            return (residuals, None);
        }

        // Project to normalized coordinates (simple pinhole: x/z, y/z)
        let proj = project_normalized(point_camera);

        // Compute residuals (2D: u, v)
        let residuals = DVector::from_column_slice(&[
            proj[0] - self.observation[0],
            proj[1] - self.observation[1],
        ]);

        let jacobian_matrix = if compute_jacobian {
            let jac_proj_wrt_point_cam = jacobian_proj_wrt_p_C(point_camera);
            // Chain rule: ∂r/∂point_world = ∂proj/∂point_cam * R_world_to_camera
            let jac_wrt_point = jac_proj_wrt_point_cam * R_C_W;

            // as_slice() is column-major, matching DMatrix default storage
            Some(DMatrix::from_column_slice(2, 3, jac_wrt_point.as_slice()))
        } else {
            None
        };

        (residuals, jacobian_matrix)
    }

    fn get_dimension(&self) -> usize {
        2 // 2D residual (u, v)
    }
}

/// BA factor for optimizing only the translation of a camera pose - development purposes
/// Observation: 2D point
/// Data: Transform from camera to body
/// Variables: Translation of system pose t_B_W, p_W
/// Residual: 2D point - project(T_B_C^{-1} * T_B_W *p_W)
#[derive(Debug, Clone)]
pub struct BundleAdjustmentFactorTranslationOnly {
    /// Observed 2D point in camera (normalized/undistorted coordinates: x, y)
    pub observation: Vector2<f64>,

    pub T_C_B: Matrix4<f64>,

    pub fixed_position: Option<Vector3<f64>>,
}

impl BundleAdjustmentFactorTranslationOnly {
    pub const fn new(observation: Vector2<f64>, T_C_B: Matrix4<f64>) -> Self {
        Self {
            observation,
            T_C_B,
            fixed_position: None,
        }
    }

    pub const fn with_fixed_position(mut self, position: Vector3<f64>) -> Self {
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
        // params[0] = 3D point in world frame (3 params: x, y, z)

        let (p_W, t_B_W) = if let Some(fixed_position) = self.fixed_position {
            assert_eq!(params.len(), 1, "BundleAdjustmentFactorTranslationOnly with fixed position requires 1 parameter vector");
            assert_eq!(params[0].len(), 3, "3D point must have 3 parameters");
            (
                Vector3::new(params[0][0], params[0][1], params[0][2]),
                fixed_position,
            )
        } else {
            assert_eq!(
                params.len(),
                2,
                "BundleAdjustmentFactorTranslationOnly requires 2 parameter vectors"
            );
            assert_eq!(params[0].len(), 3, "3D point must have 3 parameters");
            assert_eq!(params[1].len(), 3, "Translation must have 3 parameters");
            (
                Vector3::new(params[0][0], params[0][1], params[0][2]),
                Vector3::new(params[1][0], params[1][1], params[1][2]),
            )
        };

        // Transform 3D point from world to camera frame
        let R_C_B = self.T_C_B.fixed_view::<3, 3>(0, 0);
        let t_C_B = self.T_C_B.fixed_view::<3, 1>(0, 3);
        let p_C = R_C_B * (p_W + t_B_W) + t_C_B;

        // Cheirality check: point must be in front of camera
        if p_C.z <= 1e-6 {
            let residuals = DVector::from_column_slice(&[1e6, 1e6]);
            if compute_jacobian {
                // Gradient pushes the 3D point toward positive camera z
                let ncols = if self.fixed_position.is_some() { 3 } else { 6 };
                let mut jac = DMatrix::zeros(2, ncols);
                for c in 0..3 {
                    jac[(0, c)] = -R_C_B[(2, c)] * 1e3;
                    jac[(1, c)] = -R_C_B[(2, c)] * 1e3;
                }
                if ncols == 6 {
                    // Translation Jacobian: same direction since ∂p_C/∂t_B_W = R_C_B
                    for c in 0..3 {
                        jac[(0, 3 + c)] = -R_C_B[(2, c)] * 1e3;
                        jac[(1, 3 + c)] = -R_C_B[(2, c)] * 1e3;
                    }
                }
                return (residuals, Some(jac));
            }
            return (residuals, None);
        }

        // Project to normalized coordinates (simple pinhole: x/z, y/z)
        let proj = project_normalized(p_C);

        // Compute residuals (2D: u, v)
        let residuals = DVector::from_column_slice(&[
            proj[0] - self.observation[0],
            proj[1] - self.observation[1],
        ]);

        let jacobian_matrix = if compute_jacobian {
            let jac_r_wrt_p_C = jacobian_proj_wrt_p_C(p_C); // 2x3
            let jac_r_wrt_p_W = jac_r_wrt_p_C * R_C_B; // 2x3

            if self.fixed_position.is_some() {
                let mut jac = DMatrix::zeros(2, 3);
                jac.copy_from(&jac_r_wrt_p_W);
                Some(jac)
            } else {
                // ∂r/∂t_B_W = ∂r/∂p_W (same since p_C depends on p_W + t_B_W)
                let mut jac = DMatrix::zeros(2, 6);
                jac.view_mut((0, 0), (2, 3)).copy_from(&jac_r_wrt_p_W);
                jac.view_mut((0, 3), (2, 3)).copy_from(&jac_r_wrt_p_W);
                Some(jac)
            }
        } else {
            None
        };

        (residuals, jacobian_matrix)
    }

    fn get_dimension(&self) -> usize {
        2 // 2D residual (u, v)
    }
}

/// BA factor
/// Observation: 2D point
/// Data: Transform from camera to body (T_C_B)
/// Variables: System pose T_B_W (or t_B_W if rotation is fixed), p_W
/// Residual: 2D point - project(T_C_B * T_B_W * p_W)
#[derive(Debug, Clone)]
pub struct BundleAdjustmentFactor {
    /// Observed 2D point in camera (normalized/undistorted coordinates: x, y)
    pub observation: Vector2<f64>,

    /// Transform from body to camera (T_C_B: SE3 transform from B to C)
    pub T_C_B: Arc<Matrix4<f64>>,

    /// Fixed pose T_B_W (SE3 transform from W to B) if provided, None if pose is optimized
    pub fixed_pose: Option<Arc<Matrix4<f64>>>,
}

impl BundleAdjustmentFactor {
    pub const fn new(observation: Vector2<f64>, T_C_B: Arc<Matrix4<f64>>) -> Self {
        Self {
            observation,
            T_C_B,
            fixed_pose: None,
        }
    }

    /// Set a fixed pose T_B_W (SE3 transform from W to B).
    /// When set, the pose is not optimized and only the 3D point is optimized.
    pub fn with_fixed_pose(mut self, T_B_W: Arc<Matrix4<f64>>) -> Self {
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
        // Extract 3D point in world frame
        let p_W = Vector3::new(params[0][0], params[0][1], params[0][2]);

        // Extract T_B_W (SE3 transform from W to B)
        let (R_B_W, t_B_W) = if let Some(T_B_W) = &self.fixed_pose {
            assert_eq!(
                params.len(),
                1,
                "BundleAdjustmentFactor with fixed pose requires 1 parameter vector"
            );
            assert_eq!(params[0].len(), 3, "3D point must have 3 parameters");
            (
                T_B_W.fixed_view::<3, 3>(0, 0).into_owned(),
                T_B_W.fixed_view::<3, 1>(0, 3).into_owned(),
            )
        } else {
            assert_eq!(
                params.len(),
                2,
                "BundleAdjustmentFactor requires 2 parameter vectors"
            );
            assert_eq!(params[0].len(), 3, "3D point must have 3 parameters");
            assert_eq!(
                params[1].len(),
                7,
                "System pose must have 7 parameters (tx, ty, tz, qw, qx, qy, qz)"
            );

            // Optimized extraction avoiding heap allocation and cloning
            let t_B_W = Vector3::new(params[1][0], params[1][1], params[1][2]);
            let quat = UnitQuaternion::new_normalize(Quaternion::new(
                params[1][3],
                params[1][4],
                params[1][5],
                params[1][6],
            ));
            (quat.to_rotation_matrix().into_inner(), t_B_W)
        };

        // Pre-compute camera transform components (reused in jacobian)
        let R_C_B = self.T_C_B.fixed_view::<3, 3>(0, 0);
        let t_C_B = self.T_C_B.fixed_view::<3, 1>(0, 3);

        // Transform: p_W -> p_B -> p_C
        let p_B = R_B_W * p_W + t_B_W;
        let p_C = R_C_B * p_B + t_C_B;

        // Check cheirality: point behind camera gets a large residual with
        // a finite Jacobian so the optimizer can recover (zero Jacobian
        // would stall the solver).  Threshold 1e-6 (not 0.0) to prevent
        // Inf from project_normalized dividing by ~0.
        if p_C.z <= 1e-6 {
            let residuals = DVector::from_column_slice(&[1e6, 1e6]);
            if !compute_jacobian {
                return (residuals, None);
            }
            // R_total = R_C_B * R_B_W.  Row 2 of R_total is the direction in
            // world coords that increases p_C.z.  We NEGATE so that GN descent
            // (δ = -(J^T J + λI)^{-1} J^T r) moves in the +camera-z direction
            // when r = [+1e6, +1e6].
            let R_total = R_C_B * R_B_W;
            if self.fixed_pose.is_some() {
                let mut jac = DMatrix::zeros(2, 3);
                for c in 0..3 {
                    jac[(0, c)] = -R_total[(2, c)] * 1e3;
                    jac[(1, c)] = -R_total[(2, c)] * 1e3;
                }
                return (residuals, Some(jac));
            } else {
                let mut jac = DMatrix::zeros(2, 9);
                // Point Jacobian (columns 0-2)
                for c in 0..3 {
                    jac[(0, c)] = -R_total[(2, c)] * 1e3;
                    jac[(1, c)] = -R_total[(2, c)] * 1e3;
                }
                // Pose translation Jacobian (columns 3-5): same direction under
                // right-perturbation since ∂p_C/∂δρ = R_C_B * R_B_W = R_total
                for c in 0..3 {
                    jac[(0, 3 + c)] = -R_total[(2, c)] * 1e3;
                    jac[(1, 3 + c)] = -R_total[(2, c)] * 1e3;
                }
                // Rotation Jacobian (columns 6-8): leave zero (under-determined)
                return (residuals, Some(jac));
            }
        }

        // Project and compute residuals
        let proj = project_normalized(p_C);
        let residuals = DVector::from_column_slice(&[
            proj[0] - self.observation[0],
            proj[1] - self.observation[1],
        ]);

        let jacobian_matrix = if compute_jacobian {
            // Optimized Jacobian computation
            let inv_z = 1.0 / p_C.z;
            let inv_z_sq = inv_z * inv_z;
            let fx = inv_z;
            let fy = inv_z;
            let fz_x = -p_C.x * inv_z_sq;
            let fz_y = -p_C.y * inv_z_sq;

            // Compute R_total = R_C_B * R_B_W (R_C_B already extracted above)
            let R_total = R_C_B * R_B_W;

            // ∂r/∂p_W = jac_proj * R_total
            // Manually computed to exploit sparsity of jac_proj
            // jac_proj = [fx, 0, fz_x; 0, fy, fz_y]
            let mut jac_r_wrt_p_W = na::Matrix2x3::<f64>::zeros();
            for i in 0..3 {
                jac_r_wrt_p_W[(0, i)] = fx * R_total[(0, i)] + fz_x * R_total[(2, i)];
                jac_r_wrt_p_W[(1, i)] = fy * R_total[(1, i)] + fz_y * R_total[(2, i)];
            }

            if self.fixed_pose.is_some() {
                // Only optimize 3D point
                let mut jac = DMatrix::zeros(2, 3);
                jac.copy_from(&jac_r_wrt_p_W);
                Some(jac)
            } else {
                // Compute ∂r/∂ω = -∂r/∂p_W * [p_W]x
                // Manually computed to exploit sparsity of skew matrix [p_W]x
                // [p_W]x = [0, -z, y; z, 0, -x; -y, x, 0]
                // Row0 = J00, J01, J02 * Cols
                // C0 = J01*z - J02*y
                // C1 = J02*x - J00*z
                // C2 = J00*y - J01*x
                let x = p_W.x;
                let y = p_W.y;
                let z = p_W.z;

                let mut jac_r_wrt_rot = na::Matrix2x3::<f64>::zeros();

                // Row 0
                let j00 = jac_r_wrt_p_W[(0, 0)];
                let j01 = jac_r_wrt_p_W[(0, 1)];
                let j02 = jac_r_wrt_p_W[(0, 2)];
                jac_r_wrt_rot[(0, 0)] = j01 * z - j02 * y;
                jac_r_wrt_rot[(0, 1)] = j02 * x - j00 * z;
                jac_r_wrt_rot[(0, 2)] = j00 * y - j01 * x;

                // Row 1
                let j10 = jac_r_wrt_p_W[(1, 0)];
                let j11 = jac_r_wrt_p_W[(1, 1)];
                let j12 = jac_r_wrt_p_W[(1, 2)];
                jac_r_wrt_rot[(1, 0)] = j11 * z - j12 * y;
                jac_r_wrt_rot[(1, 1)] = j12 * x - j10 * z;
                jac_r_wrt_rot[(1, 2)] = j10 * y - j11 * x;

                // Negate result (formula is -J * skew)
                // We computed J * skew, so negate
                jac_r_wrt_rot.neg_mut();

                // ∂r/∂δρ = jac_proj * R_C_B * R_B_W = jac_proj * R_total
                // Under SE3 right-perturbation: t' = t + R_B_W * δρ, so ∂t/∂δρ = R_B_W
                // ∂r/∂t_B_W equals ∂r/∂p_W (both = jac_proj * R_total), reuse it

                let mut jac = DMatrix::zeros(2, 9);
                jac.view_mut((0, 0), (2, 3)).copy_from(&jac_r_wrt_p_W); // ∂r/∂p_W
                jac.view_mut((0, 3), (2, 3)).copy_from(&jac_r_wrt_p_W); // ∂r/∂t_B_W
                jac.view_mut((0, 6), (2, 3)).copy_from(&jac_r_wrt_rot); // ∂r/∂ω
                Some(jac)
            }
        } else {
            None
        };

        (residuals, jacobian_matrix)
    }

    fn get_dimension(&self) -> usize {
        2 // 2D residual (u, v)
    }
}

/// PnP factor
/// Observation: 2D point
/// Data: Transform from camera to body (T_C_B), 3D point p_W
/// Variables: System pose T_B_W
/// Residual: 2D point - project(T_C_B * T_B_W * p_W)
#[derive(Debug, Clone)]
pub struct PnPFactor {
    pub observation: Vector2<f64>,
    pub T_C_B: Matrix4<f64>,
    pub p_W: Vector3<f64>,
}

impl PnPFactor {
    pub const fn new(observation: Vector2<f64>, T_C_B: Matrix4<f64>, p_W: Vector3<f64>) -> Self {
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
        assert_eq!(
            params[0].len(),
            7,
            "System pose must have 7 parameters (tx, ty, tz, qw, qx, qy, qz)"
        );

        // OPTIMIZATION: Manually extract params to avoid SE3 allocation
        // let T_B_W = se3::SE3::from(params[0].clone());
        let tx = params[0][0];
        let ty = params[0][1];
        let tz = params[0][2];
        let qw = params[0][3];
        let qx = params[0][4];
        let qy = params[0][5];
        let qz = params[0][6];

        // Construct rotation/translation manually
        let t_B_W = Vector3::new(tx, ty, tz);
        let q_B_W = UnitQuaternion::new_normalize(Quaternion::new(qw, qx, qy, qz));
        let R_B_W = q_B_W.to_rotation_matrix();

        // Pre-compute camera transform components (reused in jacobian)
        let R_C_B = self.T_C_B.fixed_view::<3, 3>(0, 0);
        let t_C_B = self.T_C_B.fixed_view::<3, 1>(0, 3);

        // Transform: p_W -> p_B -> p_C
        let p_B = R_B_W * self.p_W + t_B_W;
        let p_C = R_C_B * p_B + t_C_B;

        // Cheirality check: point behind camera gets large residual with
        // gradient signal so the optimizer can recover.  Threshold 1e-6
        // (not 0.0) to prevent Inf from project_normalized dividing by ~0.
        if p_C.z <= 1e-6 {
            let residuals = DVector::from_column_slice(&[1e6, 1e6]);
            if compute_jacobian {
                // Negate so GN descent (δ = -(J^TJ+λI)^{-1} J^T r) pushes
                // the pose to move the point in front of the camera.
                let R_total = R_C_B * R_B_W.matrix();
                let mut jac = DMatrix::zeros(2, 6);
                for c in 0..3 {
                    jac[(0, c)] = -R_total[(2, c)] * 1e3;
                    jac[(1, c)] = -R_total[(2, c)] * 1e3;
                }
                // Rotation Jacobian (columns 3-5): leave zero
                return (residuals, Some(jac));
            }
            return (residuals, None);
        }

        // Project and compute residuals
        let proj = project_normalized(p_C);
        let residuals = DVector::from_column_slice(&[
            proj[0] - self.observation[0],
            proj[1] - self.observation[1],
        ]);

        let jacobian_matrix = if compute_jacobian {
            let jac_proj = jacobian_proj_wrt_p_C(p_C); // 2x3

            // Dense multiplication optimization: Expand jac_proj * R_C_B manually
            // R_C_B is 3x3, jac_proj is 2x3.
            // R_C_B columns
            let r00 = R_C_B[(0, 0)];
            let r01 = R_C_B[(0, 1)];
            let r02 = R_C_B[(0, 2)];
            let r10 = R_C_B[(1, 0)];
            let r11 = R_C_B[(1, 1)];
            let r12 = R_C_B[(1, 2)];
            let r20 = R_C_B[(2, 0)];
            let r21 = R_C_B[(2, 1)];
            let r22 = R_C_B[(2, 2)];

            // jac_proj elements
            let j00 = jac_proj[(0, 0)];
            let j01 = jac_proj[(0, 1)];
            let j02 = jac_proj[(0, 2)];
            let j10 = jac_proj[(1, 0)];
            let j11 = jac_proj[(1, 1)];
            let j12 = jac_proj[(1, 2)];

            // jac_proj_R_C_B = jac_proj * R_C_B
            let jpR00 = j00 * r00 + j01 * r10 + j02 * r20;
            let jpR01 = j00 * r01 + j01 * r11 + j02 * r21;
            let jpR02 = j00 * r02 + j01 * r12 + j02 * r22;

            let jpR10 = j10 * r00 + j11 * r10 + j12 * r20;
            let jpR11 = j10 * r01 + j11 * r11 + j12 * r21;
            let jpR12 = j10 * r02 + j11 * r12 + j12 * r22;

            // Transpose R_B_W for multiplication if needed, but we need jac * R_B_W
            // R_B_W (3x3)
            let rb00 = R_B_W[(0, 0)];
            let rb01 = R_B_W[(0, 1)];
            let rb02 = R_B_W[(0, 2)];
            let rb10 = R_B_W[(1, 0)];
            let rb11 = R_B_W[(1, 1)];
            let rb12 = R_B_W[(1, 2)];
            let rb20 = R_B_W[(2, 0)];
            let rb21 = R_B_W[(2, 1)];
            let rb22 = R_B_W[(2, 2)];

            // Translation Jacobian: ∂r/∂t = jac_proj * R_C_B * R_B_W
            // = [jpR] * [R_B_W] (2x3 * 3x3 = 2x3)
            let jt00 = jpR00 * rb00 + jpR01 * rb10 + jpR02 * rb20;
            let jt01 = jpR00 * rb01 + jpR01 * rb11 + jpR02 * rb21;
            let jt02 = jpR00 * rb02 + jpR01 * rb12 + jpR02 * rb22;

            let jt10 = jpR10 * rb00 + jpR11 * rb10 + jpR12 * rb20;
            let jt11 = jpR10 * rb01 + jpR11 * rb11 + jpR12 * rb21;
            let jt12 = jpR10 * rb02 + jpR11 * rb12 + jpR12 * rb22;

            // Rotation Jacobian: ∂r/∂ω = jac_proj * R_C_B * (-R_B_W * [p_W]×)
            // = (Jacobian_t) * (-1 * [p_W]x)
            let px = self.p_W[0];
            let py = self.p_W[1];
            let pz = self.p_W[2];

            // skew(p_W) = [0, -z, y; z, 0, -x; -y, x, 0]
            // -skew(p_W) = [0, z, -y; -z, 0, x; y, -x, 0]
            // J_rot = J_trans * (-skew(p_W))
            /*
                [jt00 jt01 jt02] * [ 0  z -y]
                [jt10 jt11 jt12]   [-z  0  x]
                                   [ y -x  0]

                col0 = jt00(0) + jt01(-z) + jt02(y)
                col1 = jt00(z) + jt01(0) + jt02(-x)
                col2 = jt00(-y) + jt01(x) + jt02(0)
            */
            let jr00 = -jt01 * pz + jt02 * py;
            let jr01 = jt00 * pz - jt02 * px;
            let jr02 = -jt00 * py + jt01 * px;

            let jr10 = -jt11 * pz + jt12 * py;
            let jr11 = jt10 * pz - jt12 * px;
            let jr12 = -jt10 * py + jt11 * px;

            let mut jac = DMatrix::zeros(2, 6);
            // Translate part
            jac[(0, 0)] = jt00;
            jac[(0, 1)] = jt01;
            jac[(0, 2)] = jt02;
            jac[(1, 0)] = jt10;
            jac[(1, 1)] = jt11;
            jac[(1, 2)] = jt12;
            // Rotate part
            jac[(0, 3)] = jr00;
            jac[(0, 4)] = jr01;
            jac[(0, 5)] = jr02;
            jac[(1, 3)] = jr10;
            jac[(1, 4)] = jr11;
            jac[(1, 5)] = jr12;

            Some(jac)
        } else {
            None
        };

        (residuals, jacobian_matrix)
    }

    fn get_dimension(&self) -> usize {
        2 // 2D residual (u, v)
    }
}
