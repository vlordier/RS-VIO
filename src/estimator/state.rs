use crate::types::{Matrix4x4, Vector3};

/// Trait for state operations.
///
/// This trait provides a unified interface for state manipulations,
/// enabling consistent operations across different state representations
/// and easier testing/mocking.
///
/// # Design Pattern
/// Implements the **Strategy Pattern** to encapsulate state operations
/// and make them interchangeable.
pub trait StateOperations {
    /// Get the world-from-body pose
    fn pose(&self) -> Matrix4x4;

    /// Get body linear velocity in world coordinates
    fn velocity(&self) -> Vector3;

    /// Get accelerometer bias
    fn accel_bias(&self) -> Vector3;

    /// Get gyroscope bias
    fn gyro_bias(&self) -> Vector3;

    /// Get camera extrinsics (body to left camera)
    fn T_B_Cl(&self) -> Matrix4x4;

    /// Get camera extrinsics (body to right camera)
    fn T_B_Cr(&self) -> Matrix4x4;

    /// Compose with another state (relative motion)
    fn compose(&self, other: &Self) -> Self;

    /// Invert the pose (world from body -> body from world)
    fn inverse_pose(&self) -> Matrix4x4;

    /// Interpolate between two states
    fn interpolate(&self, other: &Self, alpha: f64) -> Self;

    /// Get translation component of pose
    fn translation(&self) -> Vector3;

    /// Get rotation as unit quaternion
    fn rotation(&self) -> crate::types::UnitQuaternion;
}

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

impl StateOperations for State {
    fn pose(&self) -> Matrix4x4 {
        self.T_W_B
    }

    fn velocity(&self) -> Vector3 {
        self.velocity
    }

    fn accel_bias(&self) -> Vector3 {
        self.accel_bias
    }

    fn gyro_bias(&self) -> Vector3 {
        self.gyro_bias
    }

    fn T_B_Cl(&self) -> Matrix4x4 {
        self.T_B_Cl
    }

    fn T_B_Cr(&self) -> Matrix4x4 {
        self.T_B_Cr
    }

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

    fn inverse_pose(&self) -> Matrix4x4 {
        self.T_W_B.try_inverse().unwrap_or(Matrix4x4::identity())
    }

    fn interpolate(&self, other: &Self, alpha: f64) -> Self {
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

    fn translation(&self) -> Vector3 {
        Vector3::new(self.T_W_B[(0, 3)], self.T_W_B[(1, 3)], self.T_W_B[(2, 3)])
    }

    fn rotation(&self) -> crate::types::UnitQuaternion {
        let r = self.T_W_B.fixed_view::<3, 3>(0, 0);
        let rotmat = nalgebra::Rotation3::from_matrix_unchecked(r.into_owned());
        crate::types::UnitQuaternion::from_rotation_matrix(&rotmat)
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
}
