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
//! // The calibrator is ready to accept stereo image pairs
//! // for automatic calibration of camera intrinsics
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

pub mod camera_models;
pub mod config;
pub mod factors;
pub mod guidance;
pub mod multi_camera;
pub mod quality;
pub mod rolling_shutter;
pub mod stereo_calibrator;
pub mod triangulation;

pub use camera_models::{
    CameraConfig, CameraModel, DistortionModel, FisheyeCamera, FisheyeModel, PinholeCamera,
};
pub use config::CalibrationConfig;
pub use factors::{
    CameraGraphFactor, EpipolarFactor, MultiCameraReprojectionFactor, RollingShutterFactor,
    StereoReprojectionFactor, TemporalConsistencyFactor, TemporalSuperResolutionFactor,
};
pub use guidance::{CalibrationCoverage, CalibrationGuidance, CalibrationSuggestion, MovementType};
pub use multi_camera::{
    create_camera_graph, CameraGraph, CameraPose, MultiCameraCalibrationConfig,
    MultiCameraCalibrationResult, MultiCameraCalibrationStatus, MultiCameraCalibrator,
    MultiViewObservation,
};
pub use quality::{CalibrationLogger, CalibrationQualityMetrics, QualityThresholds};
pub use stereo_calibrator::{
    CalibrationResult, CalibrationStatus, RollingShutterDetectionInfo, StereoCalibrator,
};
