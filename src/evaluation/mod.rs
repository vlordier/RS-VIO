/// VIO evaluation metrics and benchmarking tools
/// 
/// Provides comprehensive evaluation of:
/// - Trajectory accuracy (ATE, RPE)
/// - Depth/disparity accuracy  
/// - Feature tracking quality
/// - Robustness metrics
/// - Motion estimation error
/// - Distance and speed-aware metrics

pub mod trajectory_metrics;
pub mod depth_metrics;
pub mod feature_metrics;
pub mod robustness_metrics;
pub mod results;
pub mod distance_speed_metrics;
pub mod calibration_quality;

pub use trajectory_metrics::{TrajectoryMetrics, compute_ate, compute_rpe};
pub use depth_metrics::{DepthMetrics, compute_depth_rmse};
pub use feature_metrics::{FeatureMetrics, compute_reprojection_error};
pub use robustness_metrics::{RobustnessMetrics, track_failure_rate};
pub use results::{Configuration, ConfigurationResults, compare_configurations};
pub use distance_speed_metrics::{
    DistanceBinnedMetrics, SpeedBinnedMetrics, BinMetrics, DistanceSpeedMatrix,
    DistanceSpeedAnalysis, DistanceSpeedAnalyzer,
};
pub use calibration_quality::{
    CalibrationConfidenceFactors, CalibrationQualityStats, CalibrationImprovementMetrics,
};
