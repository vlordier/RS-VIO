//! # RS-VIO: Real-time Stereo Visual-Inertial Odometry
//!
//! A tightly-coupled stereo VIO system designed for embedded real-time
//! operation on resource-constrained platforms (drones, robots).
//!
//! ## Architecture
//!
//! - **Feature tracker**: Patch-based stereo optical flow with pyramid tracking
//! - **Estimator**: Sliding-window bundle adjustment with Schur complement
//! - **IMU integration**: Preintegration factors with ESKF for high-rate prediction
//! - **Async pipeline**: Non-blocking frame processing with priority scheduling
//!
//! ## Precision
//!
//! Float precision is configurable at compile time via the `use_f32` feature flag.
//! Default is `f64` (double precision).

pub mod calibration;
pub mod datasets;
pub mod estimator;
pub mod evaluation;
pub mod feature_tracker;
pub mod imu;
pub mod logging;
pub mod optimization;
pub mod types;
pub mod viewers;

/// Macro for timing a block of code and returning (result, elapsed_ms).
/// Eliminates repetitive Instant::now() + elapsed().as_secs_f64() * 1000.0 patterns.
#[macro_export]
macro_rules! timed_ms {
    ($block:expr) => {{
        let _timed_start = std::time::Instant::now();
        let _timed_result = $block;
        (_timed_result, _timed_start.elapsed().as_secs_f64() * 1000.0)
    }};
}

#[allow(clippy::crate_in_macro_def)]
#[macro_export]
macro_rules! fl {
    ($val:expr) => {
        $val as crate::types::Float
    };
}

// Re-export commonly used types for convenience

// --- Core pipeline types ---
pub use estimator::{Estimator, MotionModel};
pub use feature_tracker::{Feature, StereoPatchTracker};
pub use viewers::Viewer;

// --- Dataset player types ---
pub use datasets::config::Config;
pub use datasets::euroc_player::EurocPlayer;
pub use datasets::fourseasons_player::FourSeasonsPlayer;
pub use datasets::player::DatasetPlayer;
pub use datasets::tum_vi_player::TUMVIPlayer;
pub use datasets::{PlayerConfig, PlayerResult};

// --- Evaluation types ---
pub use evaluation::{
    calculate_ate, calculate_rpe, EstimatedTrajectory, GroundTruthPose, GroundTruthTrajectory,
    TrajectoryEvaluation,
};

// --- Logging ---
pub use logging::init_logger;
pub use logging::PerformanceMetrics;
