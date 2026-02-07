//! # Motion Model Trait
//!
//! Defines the interface for motion prediction in VIO systems.
//! Different motion models (constant velocity, IMU-based, hybrid) implement
//! this trait, allowing the estimator to use any prediction strategy.
//!
//! ## Design Rationale
//!
//! In tightly-coupled VIO, the motion model provides an initial guess for
//! frame-to-frame pose estimation. The quality of this guess directly impacts:
//! - Convergence speed of optimization
//! - Robustness to fast motion
//! - Tracking through texture-less regions
//!
//! By defining a trait, we enable:
//! - Compile-time or runtime selection of prediction strategy
//! - Clean testing via mock implementations
//! - Future extension (e.g., learned motion priors)

use anyhow::Result;
use nalgebra as na;

/// Motion model for VIO pose prediction.
///
/// Provides next-frame pose predictions based on history or sensor data.
/// Implementations must be safe for real-time use (bounded compute, no unbounded allocation).
pub trait MotionModel: Send {
    /// Initialize model with the first observed pose.
    ///
    /// # Arguments
    /// * `pose` - World-to-body transformation (T_w_b)
    /// * `timestamp` - Frame timestamp in nanoseconds
    fn initialize(&mut self, pose: na::Isometry3<f64>, timestamp: i64) -> Result<()>;

    /// Update model with a new pose observation.
    ///
    /// Returns the innovation (prediction error in meters) for diagnostics.
    ///
    /// # Arguments
    /// * `pose` - Current world-to-body transformation (T_w_b)
    /// * `timestamp` - Frame timestamp in nanoseconds
    fn update(&mut self, pose: na::Isometry3<f64>, timestamp: i64) -> Result<f64>;

    /// Predict the next pose after `time_delta` seconds.
    ///
    /// # Arguments
    /// * `time_delta` - Time into the future to predict (seconds)
    fn predict_pose(&self, time_delta: f64) -> Result<na::Isometry3<f64>>;

    /// Whether the model has been initialized and can make predictions.
    fn is_initialized(&self) -> bool;

    /// Prediction confidence in [0.0, 1.0].
    ///
    /// Higher values indicate more reliable predictions. The estimator may
    /// use this to weight the motion prior in optimization.
    fn confidence(&self) -> f64;

    /// Reset model to uninitialized state.
    fn reset(&mut self);
}
