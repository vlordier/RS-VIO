//! Factor graph components for optimization.
//!
//! This module contains different types of factors used in visual-inertial optimization:
//! - Visual factors for camera reprojection errors
//! - IMU factors for inertial constraints
//! - Loop closure factors for global consistency
//! - Prior factors for marginalization

mod imu;
mod loop_closure;
mod prior;
mod utils;
mod visual;

#[cfg(test)]
mod tests;

pub use imu::ImuPriorFactor;
pub use loop_closure::LoopClosurePoseFactor;
pub use prior::{JointPriorFactor, PriorFactor};
pub use utils::skew_symmetric;
pub use visual::{
    BundleAdjustmentFactor, BundleAdjustmentFactorTranslationOnly, PinholeProjectionFactor,
    PnPFactor,
};
