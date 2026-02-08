//! VIO evaluation metrics and benchmarking tools
//!
//! Evaluation tools for:
//! - Trajectory accuracy (ATE, RPE)
//! - Ground truth trajectory loading and analysis
//! - Visual-inertial odometry evaluation infrastructure

pub mod trajectory_evaluation;

pub use trajectory_evaluation::{
    calculate_ate, calculate_rpe, EstimatedTrajectory, GroundTruthPose, GroundTruthTrajectory,
    TrajectoryEvaluation,
};
