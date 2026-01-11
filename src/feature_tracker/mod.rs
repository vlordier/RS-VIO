//! # Feature Tracker Module
//!
//! Real-time stereo feature tracking using patch-based optical flow.
//!
//! ## Overview
//!
//! This module implements patch-based stereo feature tracking for visual-inertial odometry.
//! Features are detected in the left image and tracked in both left and right frames
//! using a 52-point patch pattern with optical flow optimization.
//!
//! ## Key Components
//!
//! - [`FeatureTracker`](feature_tracker::FeatureTracker) - Main tracker managing both cameras
//! - [`StereoPatchTracker`](feature_tracker::StereoPatchTracker) - Stereo tracking implementation
//! - [`Pattern52`](patch::Pattern52) - 52-point patch for optical flow
//! - [`Feature`](feature_tracker::Feature) - Individual feature with pixel and undistorted coordinates
//!
//! ## Algorithm
//!
//! ### Feature Detection
//! 1. FAST corner detection on left image
//! 2. Grid-based distribution for uniform spatial coverage
//! 3. Multiple pyramid levels for multi-scale tracking
//!
//! ### Feature Tracking
//! 1. Optical flow using Lucas-Kanade on 52-point pattern
//! 2. Left-right consistency check for stereo matching
//! 3. Iterative refinement with convergence threshold
//!
//! ### Stereo Matching
//! 1. Features tracked left-to-right with epipolar constraint
//! 2. Disparity-based depth computation
//! 3. Invalid features filtered by depth and visibility
//!
//! ## Performance
//!
//! - **Detection**: ~5-20ms depending on grid size (5×5 to 20×20)
//! - **Tracking**: ~10-30ms for typical frame-to-frame motion
//! - **Stereo matching**: Integrated with tracking
//!
//! ## Usage
//!
//! ```rust,ignore
//! use rs_vio::feature_tracker::StereoPatchTracker;
//!
//! let mut tracker: StereoPatchTracker<3> = StereoPatchTracker::new(640, 480);
//! // Features are tracked by calling process_frame with stereo images
//! ```
//!
//! ## Thread Safety
//!
//! `FeatureTracker` is not thread-safe and should be used sequentially for
//! deterministic real-time performance. For parallel processing, create one
//! tracker per thread.
//!
//! ## See Also
//! - [optimization](crate::optimization) - Bundle adjustment optimization
//! - [estimator](crate::estimator) - VIO pipeline integration

#![allow(clippy::module_inception)]

pub mod feature_tracker;
pub mod image_utilities;
pub mod patch;

pub use feature_tracker::*;
pub use patch::Pattern52;
