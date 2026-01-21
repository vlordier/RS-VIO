//! # Multi-frame Fusion Module
//!
//! Implements advanced geometric fusion strategies for improving image quality and tracking robustness.
//!
//! ## Overview
//!
//! This module provides:
//! - **Rotation-only stabilization**: IMU-driven frame warping using gyro-derived rotation
//! - **Depth-aware patch fusion**: Selective per-patch refinement using sparse depth
//! - **Configuration**: YAML-based fusion strategy selection and tuning
//!
//! ## Architecture
//!
//! Traits-oriented design for pluggability:
//! ```text
//! FusionStrategy (trait)
//!   ├─ RotationStabilizer
//!   └─ DepthAwareFusion
//!
//! FusionConfig → FusionStrategy → Enhanced Frame
//! ```
//!
//! ## Performance
//!
//! - **Rotation stabilization**: ~5-15ms for N=3-5 frames @ 640×480
//! - **Patch fusion**: ~2-5ms for sparse depth (~500 points)
//! - **Memory**: Linear with frame buffer size

pub mod config;
pub mod depth_aware_fusion;
pub mod rotation_stabilizer;

pub use config::{FusionConfig, FusionStrategy};
pub use depth_aware_fusion::DepthAwareFusion;
pub use rotation_stabilizer::RotationStabilizer;

use crate::estimator::Frame;
use crate::types::Float;
use std::fmt::Debug;

/// Result type for fusion operations
pub type FusionResult<T> = Result<T, FusionError>;

/// Fusion module errors
#[derive(Debug, Clone)]
pub enum FusionError {
    /// Insufficient frames for fusion
    InsufficientFrames { required: usize, available: usize },
    /// Invalid configuration
    InvalidConfig(String),
    /// Computation error
    ComputationError(String),
    /// IMU data missing or invalid
    ImuDataError(String),
}

impl std::fmt::Display for FusionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InsufficientFrames { required, available } => {
                write!(f, "Insufficient frames: need {}, have {}", required, available)
            }
            Self::InvalidConfig(msg) => write!(f, "Invalid config: {}", msg),
            Self::ComputationError(msg) => write!(f, "Computation error: {}", msg),
            Self::ImuDataError(msg) => write!(f, "IMU error: {}", msg),
        }
    }
}

impl std::error::Error for FusionError {}

/// Trait for pluggable fusion strategies
pub trait FusionStrategyImpl: Debug + Send + Sync {
    /// Process buffered frames and return fused/enhanced output
    fn fuse(&mut self, frames: &[Frame]) -> FusionResult<FusedFrame>;

    /// Reset internal state
    fn reset(&mut self);

    /// Get strategy name for logging
    fn name(&self) -> &str;
}

/// Output of a fusion operation
#[derive(Debug, Clone)]
pub struct FusedFrame {
    /// Reference frame ID
    pub reference_frame_id: i32,
    /// Enhanced image data (if applicable)
    pub enhanced_image: Option<Vec<u8>>,
    /// Per-feature confidence scores (0-1)
    pub feature_confidence: Vec<Float>,
    /// Estimated depth for sparse points
    pub sparse_depth: Vec<Option<Float>>,
    /// Quality metrics
    pub metrics: FusionMetrics,
}

/// Quality metrics from fusion operation
#[derive(Debug, Clone, Default)]
pub struct FusionMetrics {
    /// Number of frames used
    pub num_frames_used: usize,
    /// Computation time in milliseconds
    pub computation_time_ms: Float,
    /// Estimated SNR improvement
    pub snr_improvement_db: Option<Float>,
    /// Inlier ratio for depth estimates
    pub depth_inlier_ratio: Option<Float>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fusion_error_display() {
        let err = FusionError::InsufficientFrames {
            required: 3,
            available: 1,
        };
        assert!(err.to_string().contains("Insufficient"));
    }
}
