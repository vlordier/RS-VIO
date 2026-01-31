pub mod calibration_monitor;
pub mod calibration_quality;
pub mod depth_metrics;
pub mod feature_metrics;
pub mod metrics;
pub mod results;
pub mod robustness_metrics;
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

pub mod trajectory_evaluation;
pub use calibration_monitor::{
    CalibrationHealth, CalibrationHealthAssessment, CalibrationHealthThresholds,
    CalibrationMonitor, CalibrationTrend, HealthIssue, IssueSeverity,
};
pub use calibration_quality::{
    CalibrationConfidenceFactors, CalibrationImprovementMetrics, CalibrationQualityStats,
};
pub use depth_metrics::{compute_depth_rmse, DepthMetrics};
pub use feature_metrics::{compute_reprojection_error, FeatureMetrics};
pub use metrics::{
    BinMetrics, CalibrationAwareAnalyzer, CalibrationAwareMetrics, DistanceBinnedImprovement,
    DistanceBinnedMetrics, DistanceSpeedAnalysis, DistanceSpeedAnalyzer, DistanceSpeedMatrix,
    SpeedBinnedImprovement, SpeedBinnedMetrics, TrackSurvivalStats, WeightedResidualStats,
};
pub use results::{compare_configurations, Configuration, ConfigurationResults};
pub use robustness_metrics::{track_failure_rate, RobustnessMetrics};
pub use trajectory_metrics::{compute_ate, compute_rpe, TrajectoryMetrics};

pub use trajectory_evaluation::{
    calculate_ate, calculate_rpe, EstimatedTrajectory, GroundTruthPose, GroundTruthTrajectory,
    TrajectoryEvaluation,
};
