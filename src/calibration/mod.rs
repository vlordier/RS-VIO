//! # Stereo Camera Auto-Calibration
//!
//! This module provides automatic calibration of stereo camera intrinsics and extrinsics
//! using optimization-based techniques. It estimates camera parameters from stereo image pairs
//! by minimizing reprojection errors and epipolar constraints.
//!
//! ## Features
//!
//! - **Intrinsic Calibration**: Estimates focal lengths, principal points, and distortion parameters
//! - **Extrinsic Calibration**: Estimates stereo baseline and relative pose between cameras
//! - **Bundle Adjustment**: Joint optimization of camera parameters and 3D point positions
//! - **Robust Estimation**: Handles outliers using robust loss functions
//!
//! ## Usage
//!
//! ```rust
//! use rs_vio::calibration::{StereoCalibrator, CalibrationConfig};
//!
//! // Create calibrator with configuration
//! let config = CalibrationConfig::default();
//! let mut calibrator = StereoCalibrator::new(config);
//!
//! // Add stereo image pairs
//! for (left_img, right_img) in stereo_pairs {
//!     calibrator.add_stereo_pair(left_img, right_img);
//! }
//!
//! // Run calibration
//! let result = calibrator.calibrate()?;
//!
//! // Get calibrated intrinsics
//! let left_intrinsics = result.left_intrinsics;
//! let right_intrinsics = result.right_intrinsics;
//! ```
//!
//! ## Mathematical Formulation
//!
//! The calibration minimizes the total reprojection error:
//!
//! ```text
//! E = Σᵢ ρ(rᵢ)²
//! ```
//!
//! where ρ is a robust loss function and rᵢ are reprojection residuals.
//!
//! ### Parameters Optimized
//! - **Intrinsics**: [fₓ, fᵧ, cₓ, cᵧ, k₁, k₂, p₁, p₂, k₃] (OpenCV model)
//! - **Extrinsics**: [R, t] - rotation and translation between cameras
//! - **3D Points**: [X, Y, Z] - triangulated feature positions

pub mod stereo_calibrator;
pub mod factors;
pub mod config;

pub use stereo_calibrator::{StereoCalibrator, CalibrationResult, CalibrationStatus};
pub use config::CalibrationConfig;