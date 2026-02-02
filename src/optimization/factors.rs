use apex_solver::factors::Factor;
use na::{DMatrix, DVector, Matrix3, Matrix4, Vector2, Vector3, UnitQuaternion, Quaternion};
use nalgebra as na;
use std::sync::Arc;

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

    /// Project a 3D point in camera frame to normalized coordinates (simple pinhole: x/z, y/z).
    fn project_normalized(&self, point_3d_cam: Vector3<f64>) -> Vector2<f64> {
        let x = point_3d_cam[0] / point_3d_cam[2];
        let y = point_3d_cam[1] / point_3d_cam[2];
        Vector2::new(x, y)
    }

    /// Compute Jacobian of normalized projection w.r.t. 3D point in camera frame.
    /// For pinhole: [x/z, y/z], so ∂[x/z, y/z]/∂[x, y, z]
    fn jacobian_proj_wrt_point(&self, point_3d_cam: Vector3<f64>) -> na::Matrix2x3<f64> {
        let x = point_3d_cam[0];
        let y = point_3d_cam[1];
        let z = point_3d_cam[2];

        // ∂(x/z)/∂x = 1/z, ∂(x/z)/∂y = 0, ∂(x/z)/∂z = -x/z²
        // ∂(y/z)/∂x = 0, ∂(y/z)/∂y = 1/z, ∂(y/z)/∂z = -y/z²
        let inv_z = 1.0 / z;
        let inv_z_sq = inv_z * inv_z;

        let mut jac = na::Matrix2x3::zeros();
        jac[(0, 0)] = inv_z; // ∂(x/z)/∂x
        jac[(0, 1)] = 0.0; // ∂(x/z)/∂y
        jac[(0, 2)] = -x * inv_z_sq; // ∂(x/z)/∂z
        jac[(1, 0)] = 0.0; // ∂(y/z)/∂x
        jac[(1, 1)] = inv_z; // ∂(y/z)/∂y
        jac[(1, 2)] = -y * inv_z_sq; // ∂(y/z)/∂z

        jac
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
        //println!("t_C_W: {:?}", t_C_W.to_owned().to_string());
        let point_camera = R_C_W * point_world + t_C_W;

        // Project to normalized coordinates (simple pinhole: x/z, y/z)
        let proj = self.project_normalized(point_camera);

        // Compute residuals (2D: u, v)
        let mut residuals = DVector::zeros(2);
        residuals[0] = proj[0] - self.observation[0];
        residuals[1] = proj[1] - self.observation[1];

        let jacobian_matrix = if compute_jacobian {
            let jac_proj_wrt_point_cam = self.jacobian_proj_wrt_point(point_camera);
            // Chain rule: ∂r/∂point_world = ∂proj/∂point_cam * R_world_to_camera
            let jac_wrt_point = jac_proj_wrt_point_cam * R_C_W;

            let mut jac = DMatrix::zeros(2, 3);
            jac.copy_from(&jac_wrt_point);
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

#[inline]
pub fn skew_symmetric(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
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

    /// Project a 3D point in camera frame to normalized coordinates (simple pinhole: x/z, y/z).
    fn project_normalized(&self, point_3d_cam: Vector3<f64>) -> Vector2<f64> {
        let x = point_3d_cam[0] / point_3d_cam[2];
        let y = point_3d_cam[1] / point_3d_cam[2];
        Vector2::new(x, y)
    }

    /// Compute Jacobian of normalized projection w.r.t. 3D point in camera frame.
    /// For pinhole: [x/z, y/z], so ∂[x/z, y/z]/∂[x, y, z]
    fn jacobian_r_wrt_p_C(&self, point_3d_cam: Vector3<f64>) -> na::Matrix2x3<f64> {
        let x = point_3d_cam[0];
        let y = point_3d_cam[1];
        let z = point_3d_cam[2];

        // ∂(x/z)/∂x = 1/z, ∂(x/z)/∂y = 0, ∂(x/z)/∂z = -x/z²
        // ∂(y/z)/∂x = 0, ∂(y/z)/∂y = 1/z, ∂(y/z)/∂z = -y/z²
        let inv_z = 1.0 / z;
        let inv_z_sq = inv_z * inv_z;

        let mut jac = na::Matrix2x3::zeros();
        jac[(0, 0)] = inv_z; // ∂(x/z)/∂x
        jac[(0, 1)] = 0.0; // ∂(x/z)/∂y
        jac[(0, 2)] = -x * inv_z_sq; // ∂(x/z)/∂z
        jac[(1, 0)] = 0.0; // ∂(y/z)/∂x
        jac[(1, 1)] = inv_z; // ∂(y/z)/∂y
        jac[(1, 2)] = -y * inv_z_sq; // ∂(y/z)/∂z

        jac
    }
}

impl Factor for BundleAdjustmentFactorTranslationOnly {
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        // params[0] = 3D point in world frame (3 params: x, y, z)

        let p_W = Vector3::new(params[0][0], params[0][1], params[0][2]);
        let t_B_W: Vector3<f64>;
        if let Some(fixed_position) = self.fixed_position {
            t_B_W = fixed_position;
            assert_eq!(params.len(), 1, "BundleAdjustmentFactorTranslationOnly with fixed position requires 1 parameter vector");
            assert_eq!(params[0].len(), 3, "3D point must have 3 parameters");
        } else {
            t_B_W = Vector3::new(params[1][0], params[1][1], params[1][2]);
            assert_eq!(
                params.len(),
                2,
                "BundleAdjustmentFactorTranslationOnly requires 2 parameter vectors"
            );
            assert_eq!(params[0].len(), 3, "3D point must have 3 parameters");
            assert_eq!(params[1].len(), 3, "Translation must have 3 parameters");
        }

        // Transform 3D point from world to camera frame
        let R_C_B: nalgebra::Matrix<
            f64,
            nalgebra::Const<3>,
            nalgebra::Const<3>,
            nalgebra::ViewStorage<
                '_,
                f64,
                nalgebra::Const<3>,
                nalgebra::Const<3>,
                nalgebra::Const<1>,
                nalgebra::Const<4>,
            >,
        > = self.T_C_B.fixed_view::<3, 3>(0, 0);
        let t_C_B = self.T_C_B.fixed_view::<3, 1>(0, 3);
        //println!("t_C_W: {:?}", t_C_W.to_owned().to_string());
        let p_C = R_C_B * (p_W + t_B_W) + t_C_B;

        // Project to normalized coordinates (simple pinhole: x/z, y/z)
        let proj = self.project_normalized(p_C);

        // Compute residuals (2D: u, v)
        let mut residuals = DVector::zeros(2);
        residuals[0] = proj[0] - self.observation[0];
        residuals[1] = proj[1] - self.observation[1];

        let jacobian_matrix = if compute_jacobian {
            let jac_r_wrt_p_C = self.jacobian_r_wrt_p_C(p_C); // 2x3
            let jac_r_wrt_p_W = jac_r_wrt_p_C * R_C_B; // 2x3

            if self.fixed_position.is_some() {
                let mut jac = DMatrix::zeros(2, 3);
                jac.copy_from(&jac_r_wrt_p_W);
                Some(jac)
            } else {
                let jac_r_wrt_t_B_W = jac_r_wrt_p_C * R_C_B;
                let mut jac = DMatrix::zeros(2, 6);
                jac.view_mut((0, 0), (2, 3)).copy_from(&jac_r_wrt_p_W);
                jac.view_mut((0, 3), (2, 3)).copy_from(&jac_r_wrt_t_B_W);
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

    /// Project a 3D point in camera frame to normalized coordinates (simple pinhole: x/z, y/z).
    fn project_normalized(&self, point_3d_cam: Vector3<f64>) -> Vector2<f64> {
        let x = point_3d_cam[0] / point_3d_cam[2];
        let y = point_3d_cam[1] / point_3d_cam[2];
        Vector2::new(x, y)
    }

    /// Compute Jacobian of normalized projection w.r.t. 3D point in camera frame.
    /// For pinhole: [x/z, y/z], so ∂[x/z, y/z]/∂[x, y, z]
    #[allow(dead_code)]
    fn jacobian_r_wrt_p_C(&self, point_3d_cam: Vector3<f64>) -> na::Matrix2x3<f64> {
        let x = point_3d_cam[0];
        let y = point_3d_cam[1];
        let z = point_3d_cam[2];

        // ∂(x/z)/∂x = 1/z, ∂(x/z)/∂y = 0, ∂(x/z)/∂z = -x/z²
        // ∂(y/z)/∂x = 0, ∂(y/z)/∂y = 1/z, ∂(y/z)/∂z = -y/z²
        let inv_z = 1.0 / z;
        let inv_z_sq = inv_z * inv_z;

        let mut jac = na::Matrix2x3::zeros();
        jac[(0, 0)] = inv_z; // ∂(x/z)/∂x
        jac[(0, 1)] = 0.0; // ∂(x/z)/∂y
        jac[(0, 2)] = -x * inv_z_sq; // ∂(x/z)/∂z
        jac[(1, 0)] = 0.0; // ∂(y/z)/∂x
        jac[(1, 1)] = inv_z; // ∂(y/z)/∂y
        jac[(1, 2)] = -y * inv_z_sq; // ∂(y/z)/∂z

        jac
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
            let quat = UnitQuaternion::new_normalize(Quaternion::new(params[1][3], params[1][4], params[1][5], params[1][6]));
            (quat.to_rotation_matrix().into_inner(), t_B_W)
        };

        // Pre-compute camera transform components (reused in jacobian)
        let R_C_B = self.T_C_B.fixed_view::<3, 3>(0, 0);
        let t_C_B = self.T_C_B.fixed_view::<3, 1>(0, 3);

        // Transform: p_W -> p_B -> p_C
        let p_B = R_B_W * p_W + t_B_W;
        let p_C = R_C_B * p_B + t_C_B;

        //println!("p_C: {:?}", p_C.to_owned().to_string());
        // Check cheirality of the 3D point
        // TODO fix this because it does not help
        if p_C.z <= 0.0 {
            // log::warn!("3D point is behind the camera, skipping optimization");
            let residuals = DVector::from_vec(vec![1e6, 1e6]);
            if self.fixed_pose.is_some() {
                // Only optimize 3D point
                let jac = DMatrix::zeros(2, 3);
                return (residuals, Some(jac));
            } else {
                let jac = DMatrix::zeros(2, 9);
                return (residuals, Some(jac));
            }
        }

        // Project and compute residuals
        let proj = self.project_normalized(p_C);
        let residuals = DVector::from_vec(vec![
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

            // Compute R_total = R_C_B * R_B_W
            let R_C_B = self.T_C_B.fixed_view::<3, 3>(0, 0);
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

                let mut jac = DMatrix::zeros(2, 9);
                jac.view_mut((0, 0), (2, 3)).copy_from(&jac_r_wrt_p_W); // ∂r/∂p_W
                jac.view_mut((0, 3), (2, 3)).copy_from(&jac_r_wrt_p_W); // ∂r/∂t (Following original logic)
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

    /// Project a 3D point in camera frame to normalized coordinates (simple pinhole: x/z, y/z).
    fn project_normalized(&self, p_C: Vector3<f64>) -> Vector2<f64> {
        let x = p_C[0] / p_C[2];
        let y = p_C[1] / p_C[2];
        Vector2::new(x, y)
    }

    /// Compute Jacobian of normalized projection w.r.t. 3D point in camera frame.
    /// For pinhole: [x/z, y/z], so ∂[x/z, y/z]/∂[x, y, z]
    fn jacobian_r_wrt_p_C(&self, p_C: Vector3<f64>) -> na::Matrix2x3<f64> {
        let x = p_C[0];
        let y = p_C[1];
        let z = p_C[2];

        // ∂(x/z)/∂x = 1/z, ∂(x/z)/∂y = 0, ∂(x/z)/∂z = -x/z²
        // ∂(y/z)/∂x = 0, ∂(y/z)/∂y = 1/z, ∂(y/z)/∂z = -y/z²
        let inv_z = 1.0 / z;
        let inv_z_sq = inv_z * inv_z;

        let mut jac = na::Matrix2x3::zeros();
        jac[(0, 0)] = inv_z; // ∂(x/z)/∂x
        jac[(0, 1)] = 0.0; // ∂(x/z)/∂y
        jac[(0, 2)] = -x * inv_z_sq; // ∂(x/z)/∂z
        jac[(1, 0)] = 0.0; // ∂(y/z)/∂x
        jac[(1, 1)] = inv_z; // ∂(y/z)/∂y
        jac[(1, 2)] = -y * inv_z_sq; // ∂(y/z)/∂z

        jac
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
        // We assume valid unit quaternion from solver
        let q_B_W = UnitQuaternion::new_unchecked(Quaternion::new(qw, qx, qy, qz)); 
        let R_B_W = q_B_W.to_rotation_matrix();

        // Pre-compute camera transform components (reused in jacobian)
        let R_C_B = self.T_C_B.fixed_view::<3, 3>(0, 0);
        let t_C_B = self.T_C_B.fixed_view::<3, 1>(0, 3);

        // Transform: p_W -> p_B -> p_C
        let p_B = R_B_W * self.p_W + t_B_W;
        let p_C = R_C_B * p_B + t_C_B;

        // Project and compute residuals
        let proj = self.project_normalized(p_C);
        let residuals = DVector::from_vec(vec![
            proj[0] - self.observation[0],
            proj[1] - self.observation[1],
        ]);

        let jacobian_matrix = if compute_jacobian {
            // Using helper method but could inline for further speed (helper is small though)
            let jac_proj = self.jacobian_r_wrt_p_C(p_C); // 2x3

            // Dense multiplication optimization: Expand jac_proj * R_C_B manually
            // R_C_B is 3x3, jac_proj is 2x3.
            // R_C_B columns
            let r00 = R_C_B[(0,0)]; let r01 = R_C_B[(0,1)]; let r02 = R_C_B[(0,2)];
            let r10 = R_C_B[(1,0)]; let r11 = R_C_B[(1,1)]; let r12 = R_C_B[(1,2)];
            let r20 = R_C_B[(2,0)]; let r21 = R_C_B[(2,1)]; let r22 = R_C_B[(2,2)];

            // jac_proj elements
            let j00 = jac_proj[(0,0)]; let j01 = jac_proj[(0,1)]; let j02 = jac_proj[(0,2)];
            let j10 = jac_proj[(1,0)]; let j11 = jac_proj[(1,1)]; let j12 = jac_proj[(1,2)];

            // jac_proj_R_C_B = jac_proj * R_C_B
            let jpR00 = j00*r00 + j01*r10 + j02*r20;
            let jpR01 = j00*r01 + j01*r11 + j02*r21;
            let jpR02 = j00*r02 + j01*r12 + j02*r22;

            let jpR10 = j10*r00 + j11*r10 + j12*r20;
            let jpR11 = j10*r01 + j11*r11 + j12*r21;
            let jpR12 = j10*r02 + j11*r12 + j12*r22;

            // Transpose R_B_W for multiplication if needed, but we need jac * R_B_W
            // R_B_W (3x3)
            let rb00 = R_B_W[(0,0)]; let rb01 = R_B_W[(0,1)]; let rb02 = R_B_W[(0,2)];
            let rb10 = R_B_W[(1,0)]; let rb11 = R_B_W[(1,1)]; let rb12 = R_B_W[(1,2)];
            let rb20 = R_B_W[(2,0)]; let rb21 = R_B_W[(2,1)]; let rb22 = R_B_W[(2,2)];

            // Translation Jacobian: ∂r/∂t = jac_proj * R_C_B * R_B_W
            // = [jpR] * [R_B_W] (2x3 * 3x3 = 2x3)
            let jt00 = jpR00*rb00 + jpR01*rb10 + jpR02*rb20;
            let jt01 = jpR00*rb01 + jpR01*rb11 + jpR02*rb21;
            let jt02 = jpR00*rb02 + jpR01*rb12 + jpR02*rb22;

            let jt10 = jpR10*rb00 + jpR11*rb10 + jpR12*rb20;
            let jt11 = jpR10*rb01 + jpR11*rb11 + jpR12*rb21;
            let jt12 = jpR10*rb02 + jpR11*rb12 + jpR12*rb22;

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
            let jr00 = -jt01*pz + jt02*py;
            let jr01 =  jt00*pz - jt02*px;
            let jr02 = -jt00*py + jt01*px;

            let jr10 = -jt11*pz + jt12*py;
            let jr11 =  jt10*pz - jt12*px;
            let jr12 = -jt10*py + jt11*px;

            let mut jac = DMatrix::zeros(2, 6);
            // Translate part
            jac[(0,0)] = jt00; jac[(0,1)] = jt01; jac[(0,2)] = jt02;
            jac[(1,0)] = jt10; jac[(1,1)] = jt11; jac[(1,2)] = jt12;
            // Rotate part
            jac[(0,3)] = jr00; jac[(0,4)] = jr01; jac[(0,5)] = jr02;
            jac[(1,3)] = jr10; jac[(1,4)] = jr11; jac[(1,5)] = jr12;
            
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
