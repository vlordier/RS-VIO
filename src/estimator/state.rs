use crate::traits::{StateTransform, StateView};
use crate::types::{Float, Matrix4x4, Vector3};
use crate::ok_or_log;

#[derive(Debug, Clone)]
pub struct State {
    /// World-from-body pose as a 4x4 row-major matrix (T_w_b).
    pub T_W_B: Matrix4x4,

    pub T_B_Cl: Matrix4x4,
    pub T_B_Cr: Matrix4x4,

    /// Body-frame linear velocity in world coordinates.
    pub velocity: Vector3,

    /// Accelerometer bias.
    pub accel_bias: Vector3,

    /// Gyroscope bias.
    pub gyro_bias: Vector3,
}

impl StateView for State {
    fn pose(&self) -> &Matrix4x4 {
        &self.T_W_B
    }

    fn velocity(&self) -> &Vector3 {
        &self.velocity
    }

    fn accel_bias(&self) -> &Vector3 {
        &self.accel_bias
    }

    fn gyro_bias(&self) -> &Vector3 {
        &self.gyro_bias
    }

    fn camera_left_extrinsics(&self) -> &Matrix4x4 {
        &self.T_B_Cl
    }

    fn camera_right_extrinsics(&self) -> &Matrix4x4 {
        &self.T_B_Cr
    }
}

impl StateTransform for State {
    fn compose(&self, other: &Self) -> Self {
        Self {
            T_W_B: self.T_W_B * other.T_W_B,
            T_B_Cl: self.T_B_Cl,
            T_B_Cr: self.T_B_Cr,
            velocity: self.velocity + other.velocity,
            accel_bias: self.accel_bias,
            gyro_bias: self.gyro_bias,
        }
    }

    fn inverse(&self) -> Self {
        let T_W_B_inv = ok_or_log!(
            self.T_W_B.try_inverse(),
            Matrix4x4::identity(),
            "[State] T_W_B inversion failed, using identity"
        );
        Self {
            T_W_B: T_W_B_inv,
            T_B_Cl: self.T_B_Cl,
            T_B_Cr: self.T_B_Cr,
            velocity: -self.velocity,
            accel_bias: -self.accel_bias,
            gyro_bias: -self.gyro_bias,
        }
    }

    fn interpolate(&self, other: &Self, alpha: Float) -> Self {
        let translation = self.translation() + (other.translation() - self.translation()) * alpha;

        let mut pose = Matrix4x4::identity();
        pose[(0, 3)] = translation.x;
        pose[(1, 3)] = translation.y;
        pose[(2, 3)] = translation.z;

        Self {
            T_W_B: pose,
            T_B_Cl: self.T_B_Cl,
            T_B_Cr: self.T_B_Cr,
            velocity: self.velocity + (other.velocity - self.velocity) * alpha,
            accel_bias: self.accel_bias,
            gyro_bias: self.gyro_bias,
        }
    }
}

impl State {
    pub fn new(T_B_Cl: Matrix4x4, T_B_Cr: Matrix4x4) -> Self {
        Self {
            T_W_B: Matrix4x4::identity(),
            T_B_Cl,
            T_B_Cr,
            velocity: Vector3::zeros(),
            accel_bias: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
        }
    }

    /// Identity pose, zero velocity and zero biases.
    pub fn identity() -> Self {
        Self {
            T_W_B: Matrix4x4::identity(),
            T_B_Cl: Matrix4x4::identity(),
            T_B_Cr: Matrix4x4::identity(),
            velocity: Vector3::zeros(),
            accel_bias: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
        }
    }

    // ========================================================================
    // Backward compatibility methods (delegate to traits)
    // ========================================================================

    /// Deprecated: Use StateView::pose() instead
    pub fn pose(&self) -> Matrix4x4 {
        *StateView::pose(self)
    }

    /// Deprecated: Use StateView::velocity() instead
    pub fn velocity_vec(&self) -> Vector3 {
        *StateView::velocity(self)
    }

    /// Deprecated: Use StateView::accel_bias() instead
    pub fn accel_bias_vec(&self) -> Vector3 {
        *StateView::accel_bias(self)
    }

    /// Deprecated: Use StateView::gyro_bias() instead
    pub fn gyro_bias_vec(&self) -> Vector3 {
        *StateView::gyro_bias(self)
    }

    /// Deprecated: Use StateView::camera_left_extrinsics() instead
    pub fn T_B_Cl(&self) -> Matrix4x4 {
        *StateView::camera_left_extrinsics(self)
    }

    /// Deprecated: Use StateView::camera_right_extrinsics() instead
    pub fn T_B_Cr(&self) -> Matrix4x4 {
        *StateView::camera_right_extrinsics(self)
    }

    /// Deprecated: Use StateTransform::inverse() instead
    pub fn inverse_pose(&self) -> Matrix4x4 {
        StateTransform::inverse(self).T_W_B
    }
}
