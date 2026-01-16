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
//! - StereoPatchTracker - Stereo tracking implementation
//! - PatchTracker - Monocular patch tracker used for utilities
//! - Pattern52 - 52-point patch for optical flow
//! - Feature - Individual feature with pixel and undistorted coordinates
//!
//! ## Performance Optimizations
//!
//! - `patch_simd` - SIMD-accelerated patch operations (AVX2/SSE4.1)
//! - `frame_skip` - Adaptive frame skipping for real-time constraints
//!
//! ## Trait
//!
//! The module provides a `FeatureTracker` trait for pluggable tracking algorithms.
//! See `feature_tracker::FeatureTracker` for details.
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
//! use rs_vio::feature_tracker::{FeatureTracker, StereoPatchTracker};
//!
//! let tracker = StereoPatchTracker::new();
//! // Process frames through the FeatureTracker trait
//! ```
//!
//! ## Thread Safety
//!
//! `FeatureTracker` implementations are not thread-safe and should be used sequentially for
//! deterministic real-time performance. For parallel processing, create one
//! tracker per thread.
//!
//! ## See Also
//! - [optimization](crate::optimization) - Bundle adjustment optimization
//! - [estimator](crate::estimator) - VIO pipeline integration

#![allow(clippy::module_inception)]

pub mod feature_tracker;
pub mod frame_skip;
pub mod image_utilities;
pub mod patch;
pub mod patch_simd;
pub mod ransac;

pub use feature_tracker::*;
pub use frame_skip::AdaptiveFrameSkipper;
pub use patch::Pattern52;
pub use patch_simd::compute_residuals_simd;
