//! Math utilities for robust numerical algorithms.
//!
//! This module provides building blocks for reliable, real-time optimization:
//! - Robust solvers with graceful degradation
//! - Condition number estimation
//! - Stable matrix decompositions

pub mod robust_solver;

pub use robust_solver::{estimate_condition_number, RobustSolver, SolveMethod, SolveQuality};
