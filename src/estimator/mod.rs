//! # Estimator Module
//!
//! Visual-inertial odometry pipeline for real-time pose and map estimation.
//!
//! ## Overview
//!
//! The estimator implements the core VIO pipeline, managing:
//! - Frame processing and feature association
//! - Pose estimation via PnP and bundle adjustment
//! - Sliding window keyframe management
//! - 3D map point tracking and optimization
//!
//! ## Pipeline Architecture
//!
//! ```text
//! Input Frame (stereo pair)
//!     ↓
//! Feature Detection & Tracking
//!     ↓
//! Stereo Matching (left-right consistency)
//!     ↓
//! 3D Triangulation
//!     ↓
//! Pose Estimation (PnP / Motion Model)
//!     ↓
//! Keyframe Decision
//!     ↓
//! Update Sliding Window
//!     ↓
//! Bundle Adjustment Optimization
//!     ↓
//! Output: Camera Pose + Map Points
//! ```
//!
//! ## Key Components
//!
//! - [`Estimator`](estimator::Estimator) - Main pipeline orchestrator
//! - [`Frame`](frame::Frame) - Frame representation with features
//! - [`State`](state::State) - System state (poses and landmarks)
//! - [`SlidingWindow`](sliding_window::SlidingWindow) - Keyframe management
//!
//! ## Frame Types
//!
//! - **Stereo Frame**: Left and right synchronized images
//! - Features detected in left image, matched in right image
//! - Baseline: ~10cm (configurable)
//!
//! ## Pose Representation
//!
//! - **World Frame**: Global reference frame
//! - **Body Frame**: IMU/camera body
//! - **Camera Frames**: Left and right stereo cameras
//! - **Transformations**: SE(3) matrices with rotation + translation
//!
//! ## Performance Targets
//!
//! - **Single Frame**: <33ms @ 30 FPS (640×480 resolution)
//! - **State Accumulation**: Linear growth for <50 frames
//! - **Memory**: <50MB for 50-frame window
//! - **Deterministic**: Fixed execution order for reproducibility
//!
//! ## Initialization
//!
//! System initializes from configuration YAML:
//! - Camera intrinsics and distortion models
//! - Stereo baseline and extrinsics
//! - Feature detection parameters
//! - Optimization settings
//!
//! ## Thread Safety
//!
//! `Estimator` is not thread-safe. For real-time processing:
//! - Process frames sequentially
//! - No internal parallelism (deterministic performance)
//! - IMU data integrated via separate fusion module (future)
//!
//! ## Usage
//!
//! ```rust,ignore
//! use rs_vio::{estimator::Estimator, datasets::config::Config};
//!
//! let config = Config::load("config.yaml")?;
//! let mut estimator = Estimator::new(config, None);
//!
//! let left_img = vec![0u8; 640*480];
//! let right_img = vec![0u8; 640*480];
//! let timestamp = 0i64;
//! estimator.process_frame(&left_img, &right_img, timestamp, None);
//! ```
//!
//! ## See Also
//! - [optimization](crate::optimization) - Optimization algorithms
//! - [feature_tracker](crate::feature_tracker) - Feature tracking

#![allow(clippy::module_inception)]

pub mod estimator;
pub mod frame;
pub mod sliding_window;
pub mod state;

pub use estimator::Estimator;
pub use frame::Frame;
pub use sliding_window::SlidingWindow;
pub use state::State;
