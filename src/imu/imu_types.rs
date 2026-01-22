//! IMU-specific type definitions.
//!
//! This module defines float precision for IMU calculations,
//! which can be different from the global `Float` type.
//! For embedded systems, `f32` is often preferred for IMU computations
//! due to performance considerations and hardware capabilities.

/// Float type for IMU-specific computations.
/// Set to `f32` for potential performance gains on embedded platforms,
/// especially with SIMD.
pub type ImuFloat = f32;

/// Re-export nalgebra types with the configured ImuFloat precision
use nalgebra as na;
pub type ImuVector3 = na::Vector3<ImuFloat>;
pub type ImuUnitQuaternion = na::UnitQuaternion<ImuFloat>;

/// Macro to cast literals to ImuFloat type (f32)
#[macro_export]
macro_rules! imu_fl {
    ($val:expr) => {
        $val as $crate::imu::imu_types::ImuFloat
    };
}
