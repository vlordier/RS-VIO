use apex_solver::factors::Factor;
use apex_solver::manifold::se3;
use na::{DMatrix, DVector, Matrix3, Matrix4, Matrix6, Vector2, Vector3};
use nalgebra as na;

// Import shared projection utilities to eliminate duplication
use super::projection;

/// Pinhole projection factor for optimizing 3D point positions from camera observations.
///
/// This factor computes the reprojection error for a 3D point observed in a camera.
/// It optimizes only the 3D point position, with camera pose held fixed.
///
/// - Variables: 3D point in world/camera frame (3 params: x, y, z)
/// - Fixed parameters:
///   - Camera pose (T_world_to_camera, 4x4 matrix)
///   - Observation (2D normalized/undistorted)
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

        // Project to normalized coordinates using shared projection utility
        let proj = projection::project_normalized(point_camera);

        // Compute residuals (2D: u, v)
        let mut residuals = DVector::zeros(2);
        residuals[0] = proj[0] - self.observation[0];
        residuals[1] = proj[1] - self.observation[1];

        let jacobian_matrix = if compute_jacobian {
            // Use shared jacobian utility
            let jac_proj_wrt_point_cam = projection::jacobian_proj_wrt_point(point_camera);
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
    pub fn new(observation: Vector2<f64>, T_C_B: Matrix4<f64>) -> Self {
        Self {
            observation,
            T_C_B,
            fixed_position: None,
        }
    }

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
        // params[0] = 3D point in world frame (3 params: x, y, z)

        let p_W = Vector3::new(params[0][0], params[0][1], params[0][2]);
        let t_B_W: Vector3<f64> = if let Some(fixed_position) = self.fixed_position {
            assert_eq!(params.len(), 1, "BundleAdjustmentFactorTranslationOnly with fixed position requires 1 parameter vector");
            assert_eq!(params[0].len(), 3, "3D point must have 3 parameters");
            fixed_position
        } else {
            assert_eq!(
                params.len(),
                2,
                "BundleAdjustmentFactorTranslationOnly requires 2 parameter vectors"
            );
            assert_eq!(params[0].len(), 3, "3D point must have 3 parameters");
            assert_eq!(params[1].len(), 3, "Translation must have 3 parameters");
            Vector3::new(params[1][0], params[1][1], params[1][2])
        };

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

        // Project to normalized coordinates using shared projection utility
        let proj = projection::project_normalized(p_C);

        // Compute residuals (2D: u, v)
        let mut residuals = DVector::zeros(2);
        residuals[0] = proj[0] - self.observation[0];
        residuals[1] = proj[1] - self.observation[1];

        let jacobian_matrix = if compute_jacobian {
            let jac_r_wrt_p_C = projection::jacobian_proj_wrt_point(p_C); // 2x3
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
    pub T_C_B: Matrix4<f64>,

    /// Fixed pose T_B_W (SE3 transform from W to B) if provided, None if pose is optimized
    pub fixed_pose: Option<Matrix4<f64>>,
}

impl BundleAdjustmentFactor {
    pub fn new(observation: Vector2<f64>, T_C_B: Matrix4<f64>) -> Self {
        Self {
            observation,
            T_C_B,
            fixed_pose: None,
        }
    }

    /// Set a fixed pose T_B_W (SE3 transform from W to B).
    /// When set, the pose is not optimized and only the 3D point is optimized.
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
        // Extract 3D point in world frame
        let p_W = Vector3::new(params[0][0], params[0][1], params[0][2]);

        // Extract T_B_W (SE3 transform from W to B)
        let (R_B_W, t_B_W) = if let Some(T_B_W) = self.fixed_pose {
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
            let T_B_W = se3::SE3::from(params[1].clone());
            (T_B_W.rotation_so3().rotation_matrix(), T_B_W.translation())
        };

        // Pre-compute camera transform components (reused in jacobian)
        let R_C_B = self.T_C_B.fixed_view::<3, 3>(0, 0);
        let t_C_B = self.T_C_B.fixed_view::<3, 1>(0, 3);

        // Transform: p_W -> p_B -> p_C
        let p_B = R_B_W * p_W + t_B_W;
        let p_C = R_C_B * p_B + t_C_B;

        //println!("p_C: {:?}", p_C.to_owned().to_string());
        // Check cheirality of the 3D point (must be in front of camera)
        // Note: Using large residuals doesn't effectively penalize the optimization.
        // A proper implementation would use a soft constraint or reject the measurement.
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

        // Project and compute residuals using shared projection utility
        let proj = projection::project_normalized(p_C);
        let residuals = DVector::from_vec(vec![
            proj[0] - self.observation[0],
            proj[1] - self.observation[1],
        ]);

        let jacobian_matrix = if compute_jacobian {
            let jac_proj = projection::jacobian_proj_wrt_point(p_C); // 2x3

            // Pre-compute: jac_proj * R_C_B (reused for both translation and rotation jacobians)
            let jac_proj_R_C_B = jac_proj * R_C_B; // 2x3

            // ∂r/∂p_W = jac_proj * R_C_B * R_B_W
            let jac_r_wrt_p_W = jac_proj_R_C_B * R_B_W; // 2x3

            if self.fixed_pose.is_some() {
                // Only optimize 3D point
                let mut jac = DMatrix::zeros(2, 3);
                jac.copy_from(&jac_r_wrt_p_W);
                Some(jac)
            } else {
                // Optimize both 3D point and pose: [∂r/∂p_W (2x3) | ∂r/∂T_B_W (2x6)]
                // where T_B_W SE3 tangent = [t; ω] (3 translation + 3 rotation)

                // Compute rotation jacobian: ∂r/∂ω = jac_proj * R_C_B * (-R_B_W * [p_W]×)
                let p_W_skew = skew_symmetric(&p_W);
                let jac_r_wrt_rot = jac_proj_R_C_B * (-&R_B_W * p_W_skew); // 2x3

                // Translation jacobian: ∂r/∂t = jac_proj * R_C_B * R_B_W (same as ∂r/∂p_W)
                // Concatenate: [∂r/∂p_W (2x3) | ∂r/∂t (2x3) | ∂r/∂ω (2x3)] = [2x3 | 2x6]
                let mut jac = DMatrix::zeros(2, 9);
                jac.view_mut((0, 0), (2, 3)).copy_from(&jac_r_wrt_p_W); // ∂r/∂p_W
                jac.view_mut((0, 3), (2, 3)).copy_from(&jac_r_wrt_p_W); // ∂r/∂t
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
        assert_eq!(
            params[0].len(),
            7,
            "System pose must have 7 parameters (tx, ty, tz, qw, qx, qy, qz)"
        );
        let T_B_W = se3::SE3::from(params[0].clone());
        let R_B_W: na::Matrix3<f64> = T_B_W.rotation_so3().rotation_matrix();
        let t_B_W: na::Vector3<f64> = T_B_W.translation();

        // Pre-compute camera transform components (reused in jacobian)
        let R_C_B = self.T_C_B.fixed_view::<3, 3>(0, 0);
        let t_C_B = self.T_C_B.fixed_view::<3, 1>(0, 3);

        // Transform: p_W -> p_B -> p_C
        let p_B = R_B_W * self.p_W + t_B_W;
        let p_C = R_C_B * p_B + t_C_B;

        // Project and compute residuals using shared projection utility
        let proj = projection::project_normalized(p_C);
        let residuals = DVector::from_vec(vec![
            proj[0] - self.observation[0],
            proj[1] - self.observation[1],
        ]);

        let jacobian_matrix = if compute_jacobian {
            let jac_proj = projection::jacobian_proj_wrt_point(p_C); // 2x3

            // Pre-compute: jac_proj * R_C_B (reused for both translation and rotation jacobians)
            let jac_proj_R_C_B = jac_proj * R_C_B; // 2x3

            // ∂r/∂p_W = jac_proj * R_C_B * R_B_W
            let jac_r_wrt_p_W = jac_proj_R_C_B * R_B_W; // 2x3

            // Optimize both 3D point and pose: [∂r/∂p_W (2x3) | ∂r/∂T_B_W (2x6)]
            // where T_B_W SE3 tangent = [t; ω] (3 translation + 3 rotation)

            // Compute rotation jacobian: ∂r/∂ω = jac_proj * R_C_B * (-R_B_W * [p_W]×)
            let p_W_skew = skew_symmetric(&self.p_W);
            let jac_r_wrt_rot = jac_proj_R_C_B * (-&R_B_W * p_W_skew); // 2x3

            // Translation jacobian: ∂r/∂t = jac_proj * R_C_B * R_B_W (same as ∂r/∂p_W)
            // Concatenate: [∂r/∂p_W (2x3) | ∂r/∂t (2x3) | ∂r/∂ω (2x3)] = [2x3 | 2x6]
            let mut jac = DMatrix::zeros(2, 6);
            jac.view_mut((0, 0), (2, 3)).copy_from(&jac_r_wrt_p_W); // ∂r/∂t
            jac.view_mut((0, 3), (2, 3)).copy_from(&jac_r_wrt_rot); // ∂r/∂ω
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

/// IMU prior factor
/// Data: Predicted body-from-world pose `T_B_W_pred` from IMU preintegration
/// Variables: System pose `T_B_W` (SE3)
/// Residual: 6D vector combining position error and rotation error (axis-angle)
#[derive(Debug, Clone)]
pub struct ImuPriorFactor {
    /// Predicted body-from-world pose from IMU preintegration
    pub T_B_W_pred: Matrix4<f64>,
    /// Weight for position residual components
    pub weight_pos: f64,
    /// Weight for rotation residual components
    pub weight_rot: f64,
}

impl ImuPriorFactor {
    pub fn new(T_B_W_pred: Matrix4<f64>, weight_pos: f64, weight_rot: f64) -> Self {
        Self {
            T_B_W_pred,
            weight_pos,
            weight_rot,
        }
    }
}

impl Factor for ImuPriorFactor {
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        assert_eq!(
            params.len(),
            1,
            "ImuPriorFactor requires 1 parameter vector"
        );
        assert_eq!(
            params[0].len(),
            7,
            "System pose must have 7 parameters (tx, ty, tz, qw, qx, qy, qz)",
        );

        // Variable pose (body-from-world)
        let T_B_W_var = se3::SE3::from(params[0].clone());
        let R_B_W_var: na::Matrix3<f64> = T_B_W_var.rotation_so3().rotation_matrix();
        let t_B_W_var: na::Vector3<f64> = T_B_W_var.translation();

        // Predicted pose components
        let R_B_W_pred = self.T_B_W_pred.fixed_view::<3, 3>(0, 0).into_owned();
        let t_B_W_pred = self.T_B_W_pred.fixed_view::<3, 1>(0, 3).into_owned();

        // Position residual
        let t_err = t_B_W_var - t_B_W_pred;

        // Rotation residual: axis-angle from R_pred^T * R_var
        let R_err = R_B_W_pred.transpose() * R_B_W_var;
        let q_err =
            na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_matrix_unchecked(R_err));
        let angle = q_err.angle();
        // For small angles, axis might be ill-defined; handle gracefully
        let axis = if angle > 1e-12 {
            q_err
                .axis()
                .map(|u| u.into_inner())
                .unwrap_or(na::Vector3::zeros())
        } else {
            na::Vector3::zeros()
        };
        let rot_vec = axis * angle;

        // Build 6D residual [pos; rot] with weights
        let mut residuals = DVector::zeros(6);
        residuals[0] = self.weight_pos * t_err.x;
        residuals[1] = self.weight_pos * t_err.y;
        residuals[2] = self.weight_pos * t_err.z;
        residuals[3] = self.weight_rot * rot_vec.x;
        residuals[4] = self.weight_rot * rot_vec.y;
        residuals[5] = self.weight_rot * rot_vec.z;

        let jacobian_matrix = if compute_jacobian {
            // Approximate Jacobian w.r.t. SE3 tangent as identity scaled by weights
            let mut jac = DMatrix::zeros(6, 6);
            // Position components map primarily to translation tangent
            jac[(0, 0)] = self.weight_pos;
            jac[(1, 1)] = self.weight_pos;
            jac[(2, 2)] = self.weight_pos;
            // Rotation components map to rotation tangent
            jac[(3, 3)] = self.weight_rot;
            jac[(4, 4)] = self.weight_rot;
            jac[(5, 5)] = self.weight_rot;
            Some(jac)
        } else {
            None
        };

        (residuals, jacobian_matrix)
    }

    fn get_dimension(&self) -> usize {
        6 // 3 position + 3 rotation
    }
}

/// Prior factor for marginalization.
///
/// This factor encodes the information from marginalized states as a prior
/// on the remaining parameters. It implements a quadratic prior:
///
/// ```text
/// r = prior_weight * (x - x0)
/// J = prior_weight
/// H = J^T * Information * J = prior_weight^2 * Information
/// ```
///
/// - Variables: Parameters with prior (variable dimension)
/// - Data: Linearization point `x0`, information matrix `Omega`
/// - Residual: `sqrt(Omega) * (x - x0)` scaled by prior_weight
///
/// # Mathematical Formulation
///
/// Given parameter vector `x` and prior information `x0`, `Omega`:
///
/// ```text
/// r = prior_weight * (x - x0)
/// Cost = 0.5 * r^T * Omega * r
/// ```
///
/// For Gaussian priors, `Omega` is the information matrix (inverse covariance).
#[derive(Debug, Clone)]
pub struct PriorFactor {
    /// Linearization point (prior mean)
    pub linearization_point: DVector<f64>,
    /// Information matrix (inverse covariance)
    pub information: DMatrix<f64>,
    /// Prior weight (scales the prior strength)
    pub prior_weight: f64,
}

impl PriorFactor {
    /// Create a new prior factor.
    ///
    /// # Arguments
    /// * `linearization_point` - Prior mean vector
    /// * `information` - Information matrix (must be square, matching parameter dimension)
    /// * `prior_weight` - Scaling factor for prior strength
    pub fn new(
        linearization_point: DVector<f64>,
        information: DMatrix<f64>,
        prior_weight: f64,
    ) -> Self {
        assert_eq!(
            linearization_point.len(),
            information.nrows(),
            "Linearization point dimension must match information matrix"
        );
        assert_eq!(
            information.nrows(),
            information.ncols(),
            "Information matrix must be square"
        );

        Self {
            linearization_point,
            information,
            prior_weight,
        }
    }

    /// Create a prior factor for SE3 pose (7 parameters).
    pub fn for_se3_pose(T_B_W: Matrix4<f64>, information: DMatrix<f64>, prior_weight: f64) -> Self {
        // Extract SE3 parameters: [tx, ty, tz, qw, qx, qy, qz]
        let t_B_W = T_B_W.fixed_view::<3, 1>(0, 3).into_owned();
        let R_B_W = T_B_W.fixed_view::<3, 3>(0, 0).into_owned();
        let q = na::UnitQuaternion::from_matrix(&R_B_W);

        let linearization_point =
            DVector::from_vec(vec![t_B_W.x, t_B_W.y, t_B_W.z, q.w, q.i, q.j, q.k]);

        Self::new(linearization_point, information, prior_weight)
    }
}

impl Factor for PriorFactor {
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        assert_eq!(params.len(), 1, "PriorFactor requires 1 parameter vector");

        let param = &params[0];
        assert_eq!(
            param.len(),
            self.linearization_point.len(),
            "Parameter dimension must match prior"
        );

        // Residual: r = prior_weight * (x - x0)
        let delta = param - self.linearization_point.clone();
        let residuals = &delta * self.prior_weight;

        // Jacobian: J = prior_weight * I
        let jacobian_matrix = if compute_jacobian {
            let dim = param.len();
            let mut jac = DMatrix::zeros(dim, dim);
            for i in 0..dim {
                jac[(i, i)] = self.prior_weight;
            }
            Some(jac)
        } else {
            None
        };

        (residuals, jacobian_matrix)
    }

    fn get_dimension(&self) -> usize {
        self.linearization_point.len()
    }
}

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

#[cfg(test)]
#[allow(clippy::all)]
mod tests {
    use super::*;

    fn se3_identity_vec() -> DVector<f64> {
        DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    }

    #[test]
    fn loop_closure_zero_residual_for_perfect_measurement() {
        let factor = LoopClosurePoseFactor::new(na::Matrix4::identity(), na::Matrix6::identity());
        let (res, jac) = factor.linearize(&[se3_identity_vec(), se3_identity_vec()], true);
        assert!(
            res.amax() < 1e-9,
            "residual should be zero for identity measurement"
        );
        let jac = jac.expect("jacobian should be present");
        assert_eq!(jac.nrows(), 6);
        assert_eq!(jac.ncols(), 12);
    }

    #[test]
    fn loop_closure_translation_error_propagates() {
        let T_1_2 = na::Isometry3::translation(0.0, 0.0, 0.0).to_homogeneous();
        let info = na::Matrix6::identity();
        let factor = LoopClosurePoseFactor::new(T_1_2, info);

        let pose1 = DVector::from_vec(vec![1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        let pose2 = se3_identity_vec();
        let (res, _) = factor.linearize(&[pose1, pose2], false);
        assert!((res[0] - 1.0).abs() < 1e-6);
    }
}
