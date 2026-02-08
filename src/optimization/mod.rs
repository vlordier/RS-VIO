//! Optimization module: bundle adjustment, IMU factors, and solver integration.

pub mod factors;
pub mod imu_factor;
pub mod observer;
pub mod parallel_factors;
pub mod result;

pub use imu_factor::ImuFactorSe3;
pub use result::OptimizationResult;

use apex_solver::optimizer::OptimizationStatus;

/// Returns `true` if the optimization status indicates convergence.
///
/// Covers all definitive convergence criteria but NOT `MaxIterationsReached`
/// (which may or may not be acceptable depending on context — callers can
/// check for that separately).
pub const fn optimization_converged(status: &OptimizationStatus) -> bool {
    matches!(
        status,
        OptimizationStatus::Converged
            | OptimizationStatus::CostToleranceReached
            | OptimizationStatus::ParameterToleranceReached
            | OptimizationStatus::GradientToleranceReached
            | OptimizationStatus::TrustRegionRadiusTooSmall
            | OptimizationStatus::MinCostThresholdReached
    )
}

#[cfg(test)]
mod tests;
