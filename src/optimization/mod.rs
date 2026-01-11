//! # Optimization Module
//!
//! Bundle adjustment optimization for camera poses and 3D map points.
//!
//! ## Overview
//!
//! This module implements sliding window bundle adjustment for real-time visual-inertial
//! odometry. It optimizes camera poses and 3D landmark positions using Levenberg-Marquardt
//! optimization with sparse Cholesky decomposition.
//!
//! ## Key Components
//!
//! - [`Solver`] - Trait for pluggable optimization algorithms
//! - [`PinholeProjectionFactor`](factors::PinholeProjectionFactor) - Reprojection error factor
//! - [`BundleAdjustmentFactor`](factors::BundleAdjustmentFactor) - Multi-view optimization factor
//! - [`TerminalObserver`](crate::optimization::observer::TerminalObserver) - Progress and diagnostics
//!
//! ## Optimization Problem
//!
//! Minimizes total reprojection error across all keyframes and landmarks:
//!
//! ```text
//! min Σ ||proj(T_C_W * p_W) - obs||²
//! ```
//!
//! Where:
//! - `T_C_W` = Camera pose (SE(3), optimized)
//! - `p_W` = 3D landmark position in world frame (optimized)
//! - `proj()` = Pinhole projection to normalized camera coordinates
//! - `obs` = Observed 2D feature (fixed)
//!
//! ## Factors
//!
//! ### PinholeProjectionFactor
//! - **Dimension**: 2D residual (reprojection error)
//! - **Variables**: 3D point position (3 DOF)
//! - **Parameters**: Camera pose, observation
//! - **Use case**: Optimize landmark positions with fixed poses
//!
//! ### BundleAdjustmentFactor
//! - **Dimension**: 2D residual per observation
//! - **Variables**: Camera poses (6 DOF each via SE(3) tangent space)
//! - **Parameters**: 3D landmarks, observations
//! - **Use case**: Full bundle adjustment optimization
//!
//! ## Solver Configuration
//!
//! ```rust,ignore
//! // Optimization configuration is handled through OptimizationConfig
//! use rs_vio::optimization::OptimizationConfig;
//!
//! let config = OptimizationConfig::default();
//! // Configuration specifies solver parameters and optimization strategies
//! ```
//!
//! ## Performance
//!
//! - **5-frame window**: <1ms optimization time
//! - **10-frame window**: 1-3ms optimization time
//! - **20-frame window**: 3-10ms optimization time
//! - Scales quadratically with window size and landmark density
//!
//! ## Safety
//!
//! All optimization maintains numerical stability:
//! - Validates depth ranges (0.1m to 1000m)
//! - Checks for NaN/Inf propagation
//! - Handles rank-deficient systems gracefully
//!
//! ## See Also
//! - [estimator::sliding_window](crate::estimator::sliding_window) - Keyframe management
//! - [feature_tracker](crate::feature_tracker) - Feature observations

pub mod factors;
pub mod observer;

#[cfg(test)]
mod tests;
