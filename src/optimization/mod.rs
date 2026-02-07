//! Nonlinear optimization factors and utilities for bundle adjustment.
//!
//! Includes visual reprojection factors, IMU preintegration factors,
//! parallel batch operations, and observer instrumentation.

pub mod factors;
pub mod imu_factor;
pub mod observer;
pub mod parallel_factors;
pub mod result;

pub use imu_factor::ImuFactorSe3;
pub use result::OptimizationResult;

#[cfg(test)]
mod tests;
