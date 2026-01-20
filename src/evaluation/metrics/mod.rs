/// Modular metrics for distance and speed-aware evaluation
///
/// Organized into focused modules:
/// - `types`: Core metric data structures
/// - `distance`: Distance binning and calculations
/// - `speed`: Speed/velocity binning and calculations
/// - `accuracy`: Residual and tracking quality metrics
/// - `trajectory`: Combined distance-speed analysis
/// - `calibration`: Calibration-aware performance metrics
/// - `distance_speed`: Unified distance-speed analyzer

pub mod accuracy;
pub mod calibration;
pub mod distance;
pub mod distance_speed;
pub mod speed;
pub mod trajectory;
pub mod types;

#[cfg(test)]
mod tests;

// Re-export commonly used types for convenience
pub use accuracy::{
    compute_residual_stats, compute_track_survival, TrackSurvivalStats, WeightedResidualStats,
};
pub use calibration::{CalibrationAwareAnalyzer, CalibrationAwareMetrics};
pub use distance::{bin_by_distance, DistanceBinnedImprovement, DistanceBinnedMetrics};
pub use distance_speed::DistanceSpeedAnalyzer;
pub use speed::{bin_by_speed, SpeedBinnedImprovement, SpeedBinnedMetrics};
pub use trajectory::{DistanceSpeedAnalysis, DistanceSpeedMatrix, WorstCase};
pub use types::BinMetrics;
