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

#[allow(clippy::crate_in_macro_def)]
#[macro_export]
macro_rules! fl {
    ($val:expr) => {
        $val as crate::types::Float
    };
}

// Re-export commonly used types for convenience
pub use datasets::config::Config;
pub use datasets::euroc_player::EurocPlayer;
pub use datasets::fourseasons_player::FourSeasonsPlayer;
pub use datasets::tum_vi_player::TUMVIPlayer;
pub use datasets::{PlayerConfig, PlayerResult};
pub use evaluation::{
    calculate_ate, calculate_rpe, EstimatedTrajectory, GroundTruthPose, GroundTruthTrajectory,
    TrajectoryEvaluation,
};
pub use logging::PerformanceMetrics;
