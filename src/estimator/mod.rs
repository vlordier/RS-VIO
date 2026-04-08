//! State estimation pipeline for stereo VIO.
//!
//! The estimator receives stereo image pairs, tracks features across frames,
//! and manages a sliding window of keyframes for bundle adjustment optimization.
//!
//! ## Core types
//!
//! - [`Estimator`] — main entry point, orchestrates the full VIO pipeline
//! - [`Frame`] — represents a stereo frame with tracked features
//! - [`State`] — pose, velocity, and IMU bias state for a frame
//! - [`SlidingWindow`] — fixed-size window of keyframes for BA optimization

pub mod estimator;
pub mod frame;
pub mod sliding_window;
pub mod state;

pub use estimator::Estimator;
pub use frame::Frame;
pub use sliding_window::{inverse_se3, preintegrate_imu, SlidingWindow};
pub use state::State;
