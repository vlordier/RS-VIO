//! Configuration for feature detection strategies
//!
//! YAML-based configuration for selecting and tuning detectors, trackers,
//! and descriptors based on platform capabilities and performance requirements.

use super::FeatureError;
use serde::{Deserialize, Serialize};

/// Feature detection strategy selection
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "kebab-case")]
pub enum DetectionStrategy {
    /// Shi-Tomasi (GFTT) corners with KLT tracking
    #[serde(rename = "gftt")]
    GFTTCorners,
    /// FAST corners with non-maximum suppression
    #[serde(rename = "fast")]
    FASTCorners,
    /// ORB (ORiented BRIEF)
    #[serde(rename = "orb")]
    ORBCorners,
    /// AKAZE corners
    #[serde(rename = "akaze")]
    AKAZECorners,
    /// SuperPoint (GPU-based, requires ONNX Runtime)
    #[serde(rename = "superpoint")]
    SuperPoint,
}

impl std::fmt::Display for DetectionStrategy {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::GFTTCorners => write!(f, "gftt"),
            Self::FASTCorners => write!(f, "fast"),
            Self::ORBCorners => write!(f, "orb"),
            Self::AKAZECorners => write!(f, "akaze"),
            Self::SuperPoint => write!(f, "superpoint"),
        }
    }
}

impl Default for DetectionStrategy {
    fn default() -> Self {
        Self::GFTTCorners
    }
}

/// Tracking strategy
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum TrackingStrategy {
    /// Pyramidal Kanade-Lucas-Tomasi
    #[serde(rename = "klt")]
    KLT,
    /// Optical flow-based tracking
    #[serde(rename = "optical-flow")]
    OpticalFlow,
}

impl Default for TrackingStrategy {
    fn default() -> Self {
        Self::KLT
    }
}

/// GFTT (Shi-Tomasi) corner configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GFTTConfig {
    /// Quality level for corner detection [0-1]
    #[serde(default = "default_quality_level")]
    pub quality_level: f64,
    /// Minimum distance between corners
    #[serde(default = "default_min_distance")]
    pub min_distance: u32,
    /// Grid width for spatial distribution
    #[serde(default = "default_grid_size")]
    pub grid_size: u32,
    /// Maximum features per grid cell
    #[serde(default = "default_max_per_grid")]
    pub max_per_grid: u32,
    /// Number of pyramid levels for scale robustness
    #[serde(default = "default_pyramid_levels")]
    pub pyramid_levels: u32,
}

fn default_quality_level() -> f64 {
    0.01
}
fn default_min_distance() -> u32 {
    10
}
fn default_grid_size() -> u32 {
    30
}
fn default_max_per_grid() -> u32 {
    200
}
fn default_pyramid_levels() -> u32 {
    4
}

impl Default for GFTTConfig {
    fn default() -> Self {
        Self {
            quality_level: default_quality_level(),
            min_distance: default_min_distance(),
            grid_size: default_grid_size(),
            max_per_grid: default_max_per_grid(),
            pyramid_levels: default_pyramid_levels(),
        }
    }
}

impl GFTTConfig {
    /// Validate configuration
    pub fn validate(&self) -> Result<(), FeatureError> {
        if self.quality_level <= 0.0 || self.quality_level > 1.0 {
            return Err(FeatureError::InvalidConfig(
                "quality_level must be (0, 1]".to_string(),
            ));
        }
        if self.min_distance == 0 {
            return Err(FeatureError::InvalidConfig(
                "min_distance must be > 0".to_string(),
            ));
        }
        if self.grid_size == 0 {
            return Err(FeatureError::InvalidConfig(
                "grid_size must be > 0".to_string(),
            ));
        }
        Ok(())
    }
}

/// KLT tracking configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KLTConfig {
    /// Window size for tracking patch (pixels)
    #[serde(default = "default_window_size")]
    pub window_size: u32,
    /// Maximum iterations for convergence
    #[serde(default = "default_max_iterations")]
    pub max_iterations: u32,
    /// Convergence threshold (pixels)
    #[serde(default = "default_convergence_threshold")]
    pub convergence_threshold: f64,
}

fn default_window_size() -> u32 {
    15
}
fn default_max_iterations() -> u32 {
    30
}
fn default_convergence_threshold() -> f64 {
    0.001
}

impl Default for KLTConfig {
    fn default() -> Self {
        Self {
            window_size: default_window_size(),
            max_iterations: default_max_iterations(),
            convergence_threshold: default_convergence_threshold(),
        }
    }
}

impl KLTConfig {
    /// Validate configuration
    pub fn validate(&self) -> Result<(), FeatureError> {
        if self.window_size < 3 {
            return Err(FeatureError::InvalidConfig(
                "window_size must be >= 3".to_string(),
            ));
        }
        if self.max_iterations == 0 {
            return Err(FeatureError::InvalidConfig(
                "max_iterations must be > 0".to_string(),
            ));
        }
        Ok(())
    }
}

/// FAST corner configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FASTConfig {
    /// Intensity threshold (0-255)
    #[serde(default = "default_threshold")]
    pub threshold: u8,
    /// Non-maximum suppression radius
    #[serde(default = "default_nms_radius")]
    pub nms_radius: u32,
    /// Maximum corners to detect
    #[serde(default = "default_max_corners")]
    pub max_corners: u32,
}

fn default_threshold() -> u8 {
    20
}
fn default_nms_radius() -> u32 {
    3
}
fn default_max_corners() -> u32 {
    500
}

impl Default for FASTConfig {
    fn default() -> Self {
        Self {
            threshold: default_threshold(),
            nms_radius: default_nms_radius(),
            max_corners: default_max_corners(),
        }
    }
}

/// ORB descriptor configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ORBConfig {
    /// Number of features
    #[serde(default = "default_num_features")]
    pub num_features: u32,
    /// Scale factor between levels
    #[serde(default = "default_scale_factor")]
    pub scale_factor: f64,
    /// Number of levels in scale pyramid
    #[serde(default = "default_num_levels")]
    pub num_levels: u32,
    /// Initial threshold for FAST
    #[serde(default = "default_fast_threshold")]
    pub fast_threshold: u8,
}

fn default_num_features() -> u32 {
    500
}
fn default_scale_factor() -> f64 {
    1.2
}
fn default_num_levels() -> u32 {
    8
}
fn default_fast_threshold() -> u8 {
    20
}

impl Default for ORBConfig {
    fn default() -> Self {
        Self {
            num_features: default_num_features(),
            scale_factor: default_scale_factor(),
            num_levels: default_num_levels(),
            fast_threshold: default_fast_threshold(),
        }
    }
}

/// SuperPoint configuration (GPU-based)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SuperPointConfig {
    /// Enable SuperPoint (requires GPU/ONNX Runtime)
    #[serde(default)]
    pub enabled: bool,
    /// Path to ONNX model
    #[serde(default = "default_model_path")]
    pub model_path: String,
    /// Confidence threshold for keypoint detection
    #[serde(default = "default_confidence_threshold")]
    pub confidence_threshold: f32,
    /// Non-maximum suppression radius
    #[serde(default = "default_superpoint_nms")]
    pub nms_radius: u32,
    /// Maximum keypoints to extract
    #[serde(default = "default_max_keypoints")]
    pub max_keypoints: u32,
}

fn default_model_path() -> String {
    "models/superpoint.onnx".to_string()
}
fn default_confidence_threshold() -> f32 {
    0.015
}
fn default_superpoint_nms() -> u32 {
    4
}
fn default_max_keypoints() -> u32 {
    1000
}

impl Default for SuperPointConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            model_path: default_model_path(),
            confidence_threshold: default_confidence_threshold(),
            nms_radius: default_superpoint_nms(),
            max_keypoints: default_max_keypoints(),
        }
    }
}

/// Master feature detection configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FeatureDetectionConfig {
    /// Detection strategy
    #[serde(default)]
    pub detector: DetectionStrategy,
    /// Tracking strategy
    #[serde(default)]
    pub tracker: TrackingStrategy,
    /// Enable descriptor computation
    #[serde(default)]
    pub compute_descriptors: bool,
    /// GFTT configuration
    #[serde(default)]
    pub gftt: GFTTConfig,
    /// FAST configuration
    #[serde(default)]
    pub fast: FASTConfig,
    /// ORB configuration
    #[serde(default)]
    pub orb: ORBConfig,
    /// KLT tracking configuration
    #[serde(default)]
    pub klt: KLTConfig,
    /// SuperPoint configuration
    #[serde(default)]
    pub superpoint: SuperPointConfig,
}

impl FeatureDetectionConfig {
    /// Validate configuration
    pub fn validate(&self) -> Result<(), FeatureError> {
        self.gftt.validate()?;
        self.klt.validate()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gftt_config_validation() {
        let mut config = GFTTConfig::default();
        assert!(config.validate().is_ok());

        config.quality_level = 0.0;
        assert!(config.validate().is_err());

        config.quality_level = 0.01;
        config.min_distance = 0;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_klt_config_validation() {
        let mut config = KLTConfig::default();
        assert!(config.validate().is_ok());

        config.window_size = 1;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_master_config_validation() {
        let config = FeatureDetectionConfig::default();
        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_detection_strategy_display() {
        assert_eq!(DetectionStrategy::GFTTCorners.to_string(), "gftt");
        assert_eq!(DetectionStrategy::FASTCorners.to_string(), "fast");
        assert_eq!(DetectionStrategy::ORBCorners.to_string(), "orb");
    }
}
