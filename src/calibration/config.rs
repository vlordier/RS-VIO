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

    /// Rolling shutter compensation
    pub rolling_shutter_enabled: Option<bool>, // None = auto-detect, Some(true/false) = manual
    pub rolling_shutter_readout_time: f64, // seconds for full frame readout
    pub motion_blur_compensation: bool,

    /// Robust feature detection for low-quality cameras
    pub feature_quality_threshold: f64,
    pub min_feature_sharpness: f64,
    pub adaptive_threshold_enabled: bool,

    /// Multi-frame calibration
    pub temporal_smoothing_enabled: bool,
    pub max_temporal_window: usize,

    /// Temporal super resolution
    pub temporal_super_resolution_enabled: bool,
    pub temporal_sequence_min_pairs: usize,
    pub temporal_sequence_max_duration: f64, // seconds
    pub temporal_consistency_weight: f64,

    /// Adaptive guidance system
    pub adaptive_guidance_enabled: bool,
    pub auto_calibration_enabled: bool,
    pub guidance_update_frequency: usize, // Update guidance every N pairs
    pub min_quality_for_auto_calibration: f64, // Quality threshold for auto-calibration

    /// Calibration quality logging
    pub log_calibration_metrics: bool,
    pub reprojection_error_threshold: f64,

    /// Enhanced feature detection (ORB descriptors)
    pub enhanced_features_enabled: bool,
    pub orb_max_features: usize,
    pub orb_fast_threshold: u8,
    pub orb_scale_factor: f32,
    pub orb_pyramid_levels: usize,

    /// Stereo matching configuration
    pub stereo_matcher_max_distance: u32,
    pub stereo_matcher_ratio_threshold: f32,
    pub stereo_matcher_ransac_iterations: usize,

    /// Advanced optimization features
    pub enable_bundle_adjustment: bool,
    pub enable_temporal_tracking: bool,
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
            rolling_shutter_enabled: None, // Auto-detect rolling shutter
            rolling_shutter_readout_time: 0.033, // ~30fps readout time
            motion_blur_compensation: true,
            feature_quality_threshold: 0.01,
            min_feature_sharpness: 0.1,
            adaptive_threshold_enabled: true,
            temporal_smoothing_enabled: true,
            max_temporal_window: 10,
            temporal_super_resolution_enabled: true,
            temporal_sequence_min_pairs: 5,
            temporal_sequence_max_duration: 2.0, // 2 seconds max sequence
            temporal_consistency_weight: 0.1,
            adaptive_guidance_enabled: true,
            auto_calibration_enabled: true,
            guidance_update_frequency: 2, // Update every 2 pairs
            min_quality_for_auto_calibration: 0.8,
            log_calibration_metrics: true,
            reprojection_error_threshold: 1.0,
            enhanced_features_enabled: true,
            orb_max_features: 1000,
            orb_fast_threshold: 20,
            orb_scale_factor: 1.2,
            orb_pyramid_levels: 3,
            stereo_matcher_max_distance: 64,
            stereo_matcher_ratio_threshold: 0.8,
            stereo_matcher_ransac_iterations: 1000,
            enable_bundle_adjustment: true,
            enable_temporal_tracking: true,
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::field_reassign_with_default)]
mod tests {
    use super::*;

    #[test]
    fn default_has_sensible_values() {
        let cfg = CalibrationConfig::default();
        assert_eq!(cfg.max_stereo_pairs, 50);
        assert_eq!(cfg.min_feature_matches, 20);
        assert!((cfg.max_reprojection_error - 2.0).abs() < f64::EPSILON);
        assert!((cfg.initial_focal_length - 500.0).abs() < f64::EPSILON);
        assert_eq!(cfg.image_width, 640);
        assert_eq!(cfg.image_height, 480);
        assert!(cfg.auto_calibration_enabled);
        assert!(cfg.optimize_distortion);
    }

    #[test]
    fn default_rolling_shutter_is_autodetect() {
        let cfg = CalibrationConfig::default();
        assert!(cfg.rolling_shutter_enabled.is_none());
    }

    #[test]
    fn serde_round_trip() {
        let cfg = CalibrationConfig::default();
        let yaml = serde_yaml::to_string(&cfg).expect("serialize");
        let deserialized: CalibrationConfig =
            serde_yaml::from_str(&yaml).expect("deserialize");
        assert_eq!(deserialized.max_stereo_pairs, cfg.max_stereo_pairs);
        assert_eq!(deserialized.min_feature_matches, cfg.min_feature_matches);
        assert!((deserialized.initial_focal_length - cfg.initial_focal_length).abs() < f64::EPSILON);
        assert_eq!(deserialized.rolling_shutter_enabled, cfg.rolling_shutter_enabled);
    }

    #[test]
    fn clone_preserves_values() {
        let mut cfg = CalibrationConfig::default();
        cfg.max_stereo_pairs = 99;
        cfg.rolling_shutter_enabled = Some(true);
        let cloned = cfg.clone();
        assert_eq!(cloned.max_stereo_pairs, 99);
        assert_eq!(cloned.rolling_shutter_enabled, Some(true));
    }

    #[test]
    fn fields_are_independently_modifiable() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        cfg.adaptive_guidance_enabled = false;
        cfg.rolling_shutter_enabled = Some(false);
        cfg.max_iterations = 42;
        assert!(!cfg.auto_calibration_enabled);
        assert!(!cfg.adaptive_guidance_enabled);
        assert_eq!(cfg.rolling_shutter_enabled, Some(false));
        assert_eq!(cfg.max_iterations, 42);
        // Other fields unchanged
        assert_eq!(cfg.max_stereo_pairs, 50);
    }
}
