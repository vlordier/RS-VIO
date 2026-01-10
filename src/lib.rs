//! # RS-VIO
//!
//! A Rust implementation of Visual-Inertial Odometry (VIO) for stereo cameras.
//!
//! This library provides tools for processing stereo image sequences with IMU data
//! to estimate camera motion and build 3D maps. It includes dataset players for
//! standard benchmarks like EuRoC, 4Seasons, and TUM-VI.
//!
//! ## Features
//!
//! - Stereo visual feature tracking
//! - IMU integration for motion estimation
//! - Bundle adjustment optimization
//! - Real-time visualization support
//! - Dataset players for common VIO benchmarks

use thiserror::Error;

/// Custom error type for RS-VIO operations.
#[derive(Error, Debug)]
pub enum VIOError {
    #[error("Configuration error: {0}")]
    Config(String),
    #[error("Image processing error: {0}")]
    Image(String),
    #[error("Optimization error: {0}")]
    Optimization(String),
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("Anyhow error: {0}")]
    Anyhow(#[from] anyhow::Error),
}

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
