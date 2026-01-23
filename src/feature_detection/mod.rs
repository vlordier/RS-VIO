//! Feature detection and tracking module
//!
//! Provides pluggable, trait-based feature detection and tracking strategies
//! for VIO pipeline. Supports classical detectors (GFTT, FAST, ORB) and
//! learned approaches (SuperPoint integration placeholder).
//!
//! ## Architecture
//!
//! - **KeypointDetector trait:** Pluggable detection strategies
//! - **Descriptor trait:** Descriptor computation and matching
//! - **FeatureTracker trait:** Multi-frame tracking
//! - **Configuration:** YAML-based strategy selection

pub mod config;
pub mod fast_corners;
pub mod gftt_klt;
pub mod orb_descriptor;

pub use config::FeatureDetectionConfig;
pub use gftt_klt::{GFTTDetector, PyramidalKLTTracker};

use crate::types::Float;

/// Result type for feature detection operations
pub type FeatureResult<T> = Result<T, FeatureError>;

/// Feature detection errors
#[derive(Debug, Clone)]
pub enum FeatureError {
    /// Invalid image data
    InvalidImage(String),
    /// Invalid configuration
    InvalidConfig(String),
    /// Computation error
    ComputationError(String),
    /// Tracking error
    TrackingError(String),
}

impl std::fmt::Display for FeatureError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidImage(msg) => write!(f, "Invalid image: {}", msg),
            Self::InvalidConfig(msg) => write!(f, "Invalid config: {}", msg),
            Self::ComputationError(msg) => write!(f, "Computation error: {}", msg),
            Self::TrackingError(msg) => write!(f, "Tracking error: {}", msg),
        }
    }
}

impl std::error::Error for FeatureError {}

/// Detected keypoint in image
#[derive(Debug, Clone, Copy)]
pub struct Keypoint {
    /// X coordinate (pixels)
    pub x: Float,
    /// Y coordinate (pixels)
    pub y: Float,
    /// Detection score / strength
    pub score: Float,
    /// Scale / octave
    pub scale: Float,
    /// Orientation (degrees, if applicable)
    pub angle: Option<Float>,
}

/// Feature track across frames
#[derive(Debug, Clone)]
pub struct FeatureTrack {
    /// Keypoint in current frame
    pub keypoint: Keypoint,
    /// Previous keypoint (if tracking)
    pub prev_keypoint: Option<Keypoint>,
    /// Tracking error / residual
    pub tracking_error: Float,
    /// Track ID (persistent across frames)
    pub track_id: u32,
    /// Number of frames tracked
    pub track_length: u32,
    /// Estimated measurement uncertainty (from residuals)
    pub uncertainty: Float,
}

/// Trait for pluggable keypoint detectors
pub trait KeypointDetector: Send + Sync {
    /// Detect keypoints in image
    fn detect(&self, image: &[u8], width: u32, height: u32) -> FeatureResult<Vec<Keypoint>>;

    /// Get detector name
    fn name(&self) -> &str;
}

/// Trait for pluggable feature tracking
pub trait FeatureTracker: Send + Sync {
    /// Track features from previous to current frame
    fn track(
        &mut self,
        image: &[u8],
        prev_image: &[u8],
        width: u32,
        height: u32,
        keypoints: &[Keypoint],
    ) -> FeatureResult<Vec<FeatureTrack>>;

    /// Reset tracker state
    fn reset(&mut self);

    /// Get tracker name
    fn name(&self) -> &str;
}

/// Descriptor data (binary or floating-point)
#[derive(Debug, Clone)]
pub enum DescriptorData {
    /// Binary descriptor (e.g., ORB, BRIEF)
    Binary(Vec<u8>),
    /// Floating-point descriptor
    Float(Vec<Float>),
}

/// Trait for pluggable descriptors
pub trait Descriptor: Send + Sync {
    /// Compute descriptors for keypoints
    fn compute(
        &self,
        image: &[u8],
        keypoints: &[Keypoint],
        width: u32,
        height: u32,
    ) -> FeatureResult<Vec<DescriptorData>>;

    /// Distance between two descriptors
    fn distance(&self, desc1: &DescriptorData, desc2: &DescriptorData) -> u32;

    /// Get descriptor name
    fn name(&self) -> &str;
}

/// Feature detection quality metrics
#[derive(Debug, Clone, Default)]
pub struct FeatureMetrics {
    /// Number of keypoints detected
    pub num_keypoints: usize,
    /// Number of tracks (matched features)
    pub num_tracks: usize,
    /// Average tracking error
    pub avg_tracking_error: Float,
    /// Percentage of features successfully tracked
    pub tracking_success_rate: Float,
    /// Average measurement uncertainty
    pub avg_uncertainty: Float,
    /// Computation time (milliseconds)
    pub computation_time_ms: Float,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_feature_error_display() {
        let err = FeatureError::InvalidImage("test".to_string());
        assert!(err.to_string().contains("Invalid image"));
    }

    #[test]
    fn test_keypoint_creation() {
        let kp = Keypoint {
            x: 100.5,
            y: 200.5,
            score: 0.8,
            scale: 1.0,
            angle: Some(45.0),
        };
        assert_eq!(kp.x, 100.5);
    }

    #[test]
    fn test_feature_track_creation() {
        let track = FeatureTrack {
            keypoint: Keypoint {
                x: 100.0,
                y: 200.0,
                score: 0.8,
                scale: 1.0,
                angle: None,
            },
            prev_keypoint: None,
            tracking_error: 0.5,
            track_id: 1,
            track_length: 1,
            uncertainty: 0.1,
        };
        assert_eq!(track.track_length, 1);
    }
}
