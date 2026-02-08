//! RS-VIO: Real-time stereo visual-inertial odometry for embedded platforms.

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
pub use datasets::config::Config;
pub use datasets::euroc_player::EurocPlayer;
pub use datasets::fourseasons_player::FourSeasonsPlayer;
pub use datasets::player::DatasetPlayer;
pub use datasets::tum_vi_player::TUMVIPlayer;
pub use datasets::{PlayerConfig, PlayerResult};
pub use evaluation::{
    calculate_ate, calculate_rpe, EstimatedTrajectory, GroundTruthPose, GroundTruthTrajectory,
    TrajectoryEvaluation,
};
pub use logging::PerformanceMetrics;

/// Initialize the standard colored logger used by all dataset-runner binaries.
///
/// Sets the default log level to `debug`, silences the `rerun` crate to `Warn`,
/// and produces timestamped, ANSI-colored output.
pub fn init_logger() {
    use env_logger::{Builder, Env};
    use log::LevelFilter;

    Builder::from_env(Env::default().default_filter_or("debug"))
        .filter_module("rerun", LevelFilter::Warn)
        .format(|buf, record| {
            use std::io::Write;
            let level = match record.level() {
                log::Level::Error => "\x1b[31mERROR\x1b[0m",
                log::Level::Warn => "\x1b[33mWARN\x1b[0m",
                log::Level::Info => "\x1b[32mINFO\x1b[0m",
                log::Level::Debug => "\x1b[34mDEBUG\x1b[0m",
                log::Level::Trace => "\x1b[36mTRACE\x1b[0m",
            };
            writeln!(
                buf,
                "[{}] [{}] {}",
                buf.timestamp_millis(),
                level,
                record.args()
            )
        })
        .init();
}

/// Shared entry point for all dataset-runner binaries.
///
/// Creates a [`PlayerConfig`] from the given paths, runs the player,
/// and returns the appropriate [`std::process::ExitCode`].
pub fn run_dataset<P: DatasetPlayer + Default>(
    config_path: &str,
    dataset_path: &str,
) -> std::process::ExitCode {
    let player_config = PlayerConfig {
        config_path: config_path.to_owned(),
        dataset_path: dataset_path.to_owned(),
        enable_statistics: true,
        enable_console_statistics: true,
        step_mode: false,
    };

    let player = P::default();
    let result = player.run(player_config);

    if result.success {
        log::info!("[Main] processing completed successfully!");
        std::process::ExitCode::SUCCESS
    } else {
        log::error!("[Main] processing failed: {}", result.error_message);
        std::process::ExitCode::FAILURE
    }
}
