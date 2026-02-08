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

/// Return a `("CONVERGED"|"NOT_CONVERGED", reason)` pair for logging.
pub const fn optimization_status_label(status: &OptimizationStatus) -> (&'static str, &'static str) {
    match status {
        OptimizationStatus::Converged => ("CONVERGED", "Converged"),
        OptimizationStatus::CostToleranceReached => ("CONVERGED", "CostTolerance"),
        OptimizationStatus::ParameterToleranceReached => ("CONVERGED", "ParameterTolerance"),
        OptimizationStatus::GradientToleranceReached => ("CONVERGED", "GradientTolerance"),
        OptimizationStatus::TrustRegionRadiusTooSmall => {
            ("CONVERGED", "TrustRegionRadiusTooSmall")
        },
        OptimizationStatus::MinCostThresholdReached => ("CONVERGED", "MinCostThresholdReached"),
        OptimizationStatus::MaxIterationsReached => ("NOT_CONVERGED", "MaxIterations"),
        OptimizationStatus::Timeout => ("NOT_CONVERGED", "Timeout"),
        OptimizationStatus::NumericalFailure => ("NOT_CONVERGED", "NumericalFailure"),
        OptimizationStatus::IllConditionedJacobian => {
            ("NOT_CONVERGED", "IllConditionedJacobian")
        },
        OptimizationStatus::InvalidNumericalValues => {
            ("NOT_CONVERGED", "InvalidNumericalValues")
        },
        OptimizationStatus::UserTerminated => ("NOT_CONVERGED", "UserTerminated"),
        OptimizationStatus::Failed(_) => ("NOT_CONVERGED", "Failed"),
    }
}

#[cfg(test)]
mod tests;
