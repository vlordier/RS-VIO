//!
//! Implements sliding window marginalization for visual-inertial odometry.
//! Uses Schur complement to efficiently marginalize old states while preserving
//! information from marginalized measurements.
//!
//! ## Architecture
//!
//! This module uses trait-oriented programming for flexibility and extensibility:
//!
//! - `HessianApproximator`: Trait for computing Hessian approximations
//! - `GradientComputer`: Trait for computing gradients
//! - `MarginalizationStrategy`: Combined trait for full marginalization
//! - `PriorConstructor`: Trait for constructing priors from Schur complement
//!
//! ## Algorithm
//!
//! When the sliding window is full, the oldest keyframe is marginalized:
//!
//! 1. **Identify states to marginalize**: Oldest keyframe poses and old landmarks
//! 2. **Extract linearized problem**: Get Hessian H and gradient b from optimizer
//! 3. **Schur complement**: S = H_aa - H_ab * H_bb^-1 * H_ba
//! 4. **Construct prior**: Create MarginalizationPriorFactor from S and b_eff
//!
//! ## Benefits
//!
//! - Enables large-scale SLAM without unbounded state growth
//! - Preserves information from marginalized measurements
//! - First-Estimate Jacobian (FEJ) prevents consistency issues
//! - Trait-based design allows easy swapping of approximation strategies
//!
//! ## Embedded VIO Optimization (Drones)
//!
//! This module is optimized for resource-constrained embedded platforms (Jetson Xavier, Snapdragon):
//!
//! ### Real-Time Constraints
//! - **Target latency**: <5ms per marginalization @ 30 Hz (Jetson Xavier)
//! - **Memory budget**: <500MB total (incl. sliding window, priors, feature map)
//! - **Strategy**: Fast Cholesky solve with graceful degradation to LU/pseudo-inverse
//!
//! ### Key Optimizations
//! 1. **Condition number estimation**: O(n) fast heuristic (Frobenius/trace) instead of O(n³) SVD
//! 2. **Solve pipeline**: Try Cholesky first, escalate damping, fall back to LU then pseudo-inverse
//! 3. **Memory efficiency**: Minimize matrix clones; re-clone only on actual solve attempts
//! 4. **Damping bounds**: Cap regularization at 1e3 to prevent unbounded solution degradation
//!
//! ### Configuration Recommendations
//!
//! For typical embedded drone VIO:
//! ```ignore
//! MarginalizationConfig {
//!     damping: 1e-5,                      // Conservative: prevent ill-conditioning
//!     max_keyframes: 8,                   // Tight window: 8 frames × 7 DOF = 56 states
//!     hessian_approximator: "Diagonal",   // Fast: O(n) vs O(n²) Gauss-Newton
//!     prior_info_scale: 0.9,              // Slight downweight: prevent over-constraint
//!     use_fej: true,                      // Always use FEJ for consistency
//! }
//! ```
//!
//! For GPS-denied tight spaces (tunnels, canyons):
//! ```ignore
//! MarginalizationConfig {
//!     damping: 1e-4,                      // Stronger regularization: expect poor conditioning
//!     max_keyframes: 5,                   // Ultra-tight: minimize state growth
//!     hessian_approximator: "Diagonal",
//!     prior_info_scale: 0.7,              // Downweight to avoid over-constraint
//!     use_fej: true,
//! }
//! ```
//!
//! See [MARGINALIZATION_EMBEDDED_AUDIT.md](../MARGINALIZATION_EMBEDDED_AUDIT.md) for detailed
//! robustness audit and performance measurements.

pub mod approximators;
pub mod config;
pub mod manager;
pub mod prior;
pub mod traits;
pub mod utils;

#[cfg(test)]
mod tests;

// Re-export public types
pub use approximators::{
    DiagonalApproximator, ExactHessianApproximator, GaussNewtonApproximator, IdentityApproximator,
    LevenbergMarquardtApproximator, RegularizedPriorConstructor, StandardGradientComputer,
    StandardPriorConstructor, ZeroGradientComputer,
};
pub use config::{
    MarginalizationConfig, MarginalizationInfo, MarginalizationResult, ParamBlock, ParamId,
};
pub use manager::MarginalizationManager;
pub use prior::{FejCache, MarginalizationPrior, MarginalizationStats};
pub use traits::{GradientComputer, HessianApproximator, PriorConstructor};
pub use utils::select_marginalization_candidates;
