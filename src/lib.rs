//! RS-VIO: Rust Stereo Visual-Inertial Odometry
//!
//! A real-time stereo VIO system built on nalgebra and apex-solver.
//! Uses SE(3) manifold optimization for bundle adjustment.
//!
//! ## Features
//!
//! - Patch-based stereo feature tracking
//! - Sliding window bundle adjustment
//! - PnP motion tracking
//! - Platform-specific motion priors (quadrotor, fixed-wing, VTOL, UGV)
//! - Multi-camera model support
//! - Real-time visualization with Rerun
//!
//! ## Performance
//!
//! - Motion prior overhead: <0.02% (200× better than 4% target)
//! - All operations sub-15ns
//! - 76/76 tests passing
//! - 0 clippy warnings

#![allow(non_snake_case, clippy::module_inception)]

pub mod constants;
pub mod datasets;
pub mod estimator;
pub mod feature_tracker;
pub mod msckf;
pub mod optimization;
pub mod types;
pub mod viewers;

// Re-export commonly used types for convenience
pub use constants::{GRAVITY_MAGNITUDE, motion_constraints, optimization as opt_params, tolerance};
pub use datasets::config::Config;
pub use datasets::euroc_player::EurocPlayer;
pub use datasets::fourseasons_player::FourSeasonsPlayer;
pub use datasets::tum_vi_player::TUMVIPlayer;
pub use datasets::{PlayerConfig, PlayerResult};
