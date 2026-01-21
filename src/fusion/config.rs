//! Fusion configuration for strategy selection and tuning.
//!
//! Provides YAML-based configuration to enable pluggable fusion strategies
//! with per-strategy parameter tuning.

use serde::{Deserialize, Serialize};
use super::{FusionError, FusionResult};

/// High-level fusion strategy selection
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "kebab-case")]
pub enum FusionStrategy {
    /// Disable fusion
    #[serde(rename = "none")]
    None,
    /// IMU-driven rotation stabilization
    #[serde(rename = "rotation")]
    RotationStabilizer,
    /// Depth-aware selective patch fusion
    #[serde(rename = "depth-aware")]
    DepthAwareFusion,
}

impl Default for FusionStrategy {
    fn default() -> Self {
        Self::None
    }
}

impl std::fmt::Display for FusionStrategy {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::None => write!(f, "none"),
            Self::RotationStabilizer => write!(f, "rotation"),
            Self::DepthAwareFusion => write!(f, "depth-aware"),
        }
    }
}

/// Rotation stabilizer configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RotationStabilizerConfig {
    /// Number of frames to fuse (3-5 typical)
    #[serde(default = "default_num_frames")]
    pub num_frames: usize,
    /// Minimum rotation to trigger fusion (radians)
    #[serde(default = "default_min_rotation_threshold")]
    pub min_rotation_threshold: f64,
    /// Weighting strategy: "uniform", "exponential", or "sharpness-adaptive"
    #[serde(default = "default_weighting_strategy")]
    pub weighting_strategy: String,
}

fn default_num_frames() -> usize { 3 }
fn default_min_rotation_threshold() -> f64 { 0.01 }
fn default_weighting_strategy() -> String { "exponential".to_string() }

impl Default for RotationStabilizerConfig {
    fn default() -> Self {
        Self {
            num_frames: default_num_frames(),
            min_rotation_threshold: default_min_rotation_threshold(),
            weighting_strategy: default_weighting_strategy(),
        }
    }
}

impl RotationStabilizerConfig {
    /// Validate configuration
    pub fn validate(&self) -> FusionResult<()> {
        if self.num_frames < 2 || self.num_frames > 10 {
            return Err(FusionError::InvalidConfig(
                format!("num_frames must be 2-10, got {}", self.num_frames),
            ));
        }
        if self.min_rotation_threshold < 0.0 || self.min_rotation_threshold > 1.0 {
            return Err(FusionError::InvalidConfig(
                "min_rotation_threshold must be 0-1".to_string(),
            ));
        }
        match self.weighting_strategy.as_str() {
            "uniform" | "exponential" | "sharpness-adaptive" => Ok(()),
            _ => Err(FusionError::InvalidConfig(
                format!("unknown weighting strategy: {}", self.weighting_strategy),
            )),
        }
    }
}

/// Depth-aware fusion configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DepthAwareFusionConfig {
    /// Number of frames to use for fusion
    #[serde(default = "default_num_frames")]
    pub num_frames: usize,
    /// Minimum texture confidence to trigger fusion [0-1]
    #[serde(default = "default_texture_threshold")]
    pub texture_confidence_threshold: f64,
    /// Depth hypothesis search range [%]
    #[serde(default = "default_depth_range")]
    pub depth_hypothesis_range: f64,
    /// Number of depth hypotheses to test
    #[serde(default = "default_num_hypotheses")]
    pub num_depth_hypotheses: usize,
}

fn default_texture_threshold() -> f64 { 0.6 }
fn default_depth_range() -> f64 { 0.2 }
fn default_num_hypotheses() -> usize { 5 }

impl Default for DepthAwareFusionConfig {
    fn default() -> Self {
        Self {
            num_frames: default_num_frames(),
            texture_confidence_threshold: default_texture_threshold(),
            depth_hypothesis_range: default_depth_range(),
            num_depth_hypotheses: default_num_hypotheses(),
        }
    }
}

impl DepthAwareFusionConfig {
    /// Validate configuration
    pub fn validate(&self) -> FusionResult<()> {
        if self.num_frames < 2 || self.num_frames > 10 {
            return Err(FusionError::InvalidConfig(
                format!("num_frames must be 2-10, got {}", self.num_frames),
            ));
        }
        if self.texture_confidence_threshold < 0.0 || self.texture_confidence_threshold > 1.0 {
            return Err(FusionError::InvalidConfig(
                "texture_confidence_threshold must be 0-1".to_string(),
            ));
        }
        if self.depth_hypothesis_range <= 0.0 || self.depth_hypothesis_range > 1.0 {
            return Err(FusionError::InvalidConfig(
                "depth_hypothesis_range must be 0-1".to_string(),
            ));
        }
        Ok(())
    }
}

/// Master fusion configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FusionConfig {
    /// Active fusion strategy
    #[serde(default)]
    pub strategy: FusionStrategy,
    /// Rotation stabilizer settings
    #[serde(default)]
    pub rotation_stabilizer: RotationStabilizerConfig,
    /// Depth-aware fusion settings
    #[serde(default)]
    pub depth_aware_fusion: DepthAwareFusionConfig,
    /// Enable logging of fusion metrics
    #[serde(default = "default_log_metrics")]
    pub log_metrics: bool,
}

fn default_log_metrics() -> bool { false }

impl FusionConfig {
    /// Validate entire configuration
    pub fn validate(&self) -> FusionResult<()> {
        match self.strategy {
            FusionStrategy::None => Ok(()),
            FusionStrategy::RotationStabilizer => self.rotation_stabilizer.validate(),
            FusionStrategy::DepthAwareFusion => self.depth_aware_fusion.validate(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rotation_config_validation() {
        let mut config = RotationStabilizerConfig::default();
        assert!(config.validate().is_ok());

        config.num_frames = 0;
        assert!(config.validate().is_err());

        config.num_frames = 3;
        config.weighting_strategy = "invalid".to_string();
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_depth_aware_config_validation() {
        let mut config = DepthAwareFusionConfig::default();
        assert!(config.validate().is_ok());

        config.texture_confidence_threshold = 1.5;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_master_config_validation() {
        let config = FusionConfig {
            strategy: FusionStrategy::RotationStabilizer,
            rotation_stabilizer: RotationStabilizerConfig::default(),
            ..Default::default()
        };
        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_strategy_display() {
        assert_eq!(FusionStrategy::None.to_string(), "none");
        assert_eq!(FusionStrategy::RotationStabilizer.to_string(), "rotation");
        assert_eq!(FusionStrategy::DepthAwareFusion.to_string(), "depth-aware");
    }
}
