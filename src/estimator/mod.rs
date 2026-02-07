//! State estimation and VIO pipeline.
//!
//! Contains the sliding-window estimator, per-frame state, motion models,
//! and async wrappers for real-time operation.

pub mod async_enhancements;
pub mod async_optimization;
pub mod async_wrapper;
pub mod constant_velocity_model;
#[allow(clippy::module_inception)] // Re-exported as crate::estimator::Estimator
pub mod estimator;
pub mod frame;
pub mod motion_model;
pub mod sliding_window;
pub mod state;

pub use async_enhancements::{
    DeadlineTracker, FailureRecoveryTracker, LatencyHistogram, PriorityFrameEntry,
    ProcessingMetrics, StreamingPatternAnalyzer,
};
pub use async_optimization::AsyncOptimizer;
pub use async_wrapper::AsyncEstimator;
pub use constant_velocity_model::{ConstantVelocityConfig, ConstantVelocityModel};
pub use estimator::Estimator;
pub use frame::Frame;
pub use motion_model::MotionModel;
pub use sliding_window::SlidingWindow;
pub use state::State;
