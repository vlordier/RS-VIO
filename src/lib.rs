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
//!
//! ## Usage
//!
//! ```rust,no_run
//! use rs_vio::{datasets::config::Config, estimator::Estimator};
//! use serde_yaml;
//!
//! // Load configuration
//! let config_yaml = std::fs::read_to_string("config.yaml")?;
//! let config: Config = serde_yaml::from_str(&config_yaml)?;
//!
//! // Create estimator
//! let mut estimator = Estimator::new(config, None);
//!
//! // Process frames...
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

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

/// Initialize logging for RS-VIO.
///
/// This function sets up structured logging with appropriate levels and formatting.
/// Call this at the beginning of your application.
///
/// # Examples
///
/// ```rust
/// use rs_vio::init_logging;
///
/// init_logging();
/// ```
pub fn init_logging() {
    use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

    tracing_subscriber::registry()
        .with(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "rs_vio=info".into()),
        )
        .with(tracing_subscriber::fmt::layer().json())
        .init();
}

pub mod datasets;
pub mod estimator;
pub mod feature_tracker;
pub mod optimization;
pub mod types;
pub mod viewers;
pub mod validation;

// Re-export commonly used types for convenience
pub use datasets::config::Config;
pub use datasets::euroc_player::EurocPlayer;
pub use datasets::fourseasons_player::FourSeasonsPlayer;
pub use datasets::tum_vi_player::TUMVIPlayer;
pub use datasets::{PlayerConfig, PlayerResult};
