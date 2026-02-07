//! Per-frame VIO state: pose, velocity, and IMU biases.
//!
//! Stored on each [`super::Frame`] and updated by the estimator
//! after optimization or IMU propagation.

use crate::types::{Float, Matrix4x4};

#[derive(Debug, Clone)]
pub struct State {
    /// World-from-body pose as a 4x4 row-major matrix (T_w_b).
    pub T_W_B: Matrix4x4,

    pub T_B_Cl: Matrix4x4,
    pub T_B_Cr: Matrix4x4,

    /// Body-frame linear velocity in world coordinates.
    pub velocity: [Float; 3],

    /// Accelerometer bias.
    pub accel_bias: [Float; 3],

    /// Gyroscope bias.
    pub gyro_bias: [Float; 3],
}

impl State {
    pub fn new(T_B_Cl: Matrix4x4, T_B_Cr: Matrix4x4) -> Self {
        Self {
            T_W_B: Matrix4x4::identity(),
            T_B_Cl,
            T_B_Cr,
            velocity: [0.0, 0.0, 0.0],
            accel_bias: [0.0, 0.0, 0.0],
            gyro_bias: [0.0, 0.0, 0.0],
        }
    }

    /// Identity pose, zero velocity and zero biases.
    pub fn identity() -> Self {
        Self {
            T_W_B: Matrix4x4::identity(),
            T_B_Cl: Matrix4x4::identity(),
            T_B_Cr: Matrix4x4::identity(),
            velocity: [0.0, 0.0, 0.0],
            accel_bias: [0.0, 0.0, 0.0],
            gyro_bias: [0.0, 0.0, 0.0],
        }
    }
}
