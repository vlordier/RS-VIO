pub mod async_enhancements;
pub mod async_optimization;
pub mod async_wrapper;
#[allow(clippy::module_inception)] // Re-exported as crate::estimator::Estimator
pub mod constant_velocity_model;
pub mod estimator;
pub mod frame;
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
pub use sliding_window::SlidingWindow;
pub use state::State;
