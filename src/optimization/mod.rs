//! Nonlinear least-squares optimization for VIO.
//!
//! Provides [`Factor`](apex_solver::factors::Factor) implementations for reprojection
//! residuals on the SE(3) manifold, used by the sliding window bundle adjustment.
//!
//! ## Factor types
//!
//! - [`factors::PinholeProjectionFactor`] — optimizes only 3D point, pose is fixed
//! - [`factors::BundleAdjustmentFactor`] — full BA: optimizes both point and pose
//! - [`factors::BundleAdjustmentFactorTranslationOnly`] — translation-only BA
//! - [`factors::PnPFactor`] — PnP: optimizes pose given known 3D points
//! - [`factors::LoopClosureEdgeFactor`] — loop closure: constrains relative pose between two keyframes

pub mod factors;
pub mod imu_factor;
pub mod imu_preintegration;
pub mod observer;

#[cfg(test)]
mod tests;
