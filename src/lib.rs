//! RS-VIO: Rust Stereo Visual-Inertial Odometry
//!
//! A real-time stereo VIO system built on nalgebra and apex-solver.
//! Uses SE(3) manifold optimization for bundle adjustment.

#![allow(non_snake_case, clippy::module_inception)]

pub mod datasets;
pub mod estimator;
pub mod feature_tracker;
pub mod optimization;
pub mod types;
pub mod viewers;

// Re-export commonly used types for convenience
pub use datasets::config::Config;
pub use datasets::euroc_player::EurocPlayer;
pub use datasets::fourseasons_player::FourSeasonsPlayer;
pub use datasets::tum_vi_player::TUMVIPlayer;
pub use datasets::{PlayerConfig, PlayerResult};
