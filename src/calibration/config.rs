//! Configuration for stereo camera calibration

use serde::{Deserialize, Serialize};

/// Configuration for stereo camera auto-calibration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationConfig {
    /// Maximum number of stereo pairs to use for calibration
    pub max_stereo_pairs: usize,

    /// Minimum number of feature matches required per stereo pair
    pub min_feature_matches: usize,

    /// Maximum reprojection error threshold (pixels)
    pub max_reprojection_error: f64,

    /// Robust loss function parameter (for Huber loss)
    pub huber_delta: f64,

    /// Maximum iterations for bundle adjustment
    pub max_iterations: usize,

    /// Convergence tolerance for parameter updates
    pub parameter_tolerance: f64,

    /// Convergence tolerance for cost function
    pub cost_tolerance: f64,

    /// Whether to optimize distortion parameters
    pub optimize_distortion: bool,

    /// Whether to optimize principal point
    pub optimize_principal_point: bool,

    /// Initial focal length guess (if not provided)
    pub initial_focal_length: f64,

    /// Initial principal point guess (if not provided)
    pub initial_principal_point: (f64, f64),

    /// Image dimensions
    pub image_width: u32,
    pub image_height: u32,
}

impl Default for CalibrationConfig {
    fn default() -> Self {
        Self {
            max_stereo_pairs: 50,
            min_feature_matches: 20,
            max_reprojection_error: 2.0,
            huber_delta: 1.0,
            max_iterations: 100,
            parameter_tolerance: 1e-6,
            cost_tolerance: 1e-6,
            optimize_distortion: true,
            optimize_principal_point: true,
            initial_focal_length: 500.0,
            initial_principal_point: (320.0, 240.0),
            image_width: 640,
            image_height: 480,
        }
    }
}