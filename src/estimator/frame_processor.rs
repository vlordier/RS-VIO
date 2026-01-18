//! Frame Processing Module
//!
//! Handles the core frame processing pipeline with clear separation of concerns:
//! - Image loading and buffer management
//! - Feature tracking coordination
//! - Keyframe decision logic
//!
//! This module optimizes the critical path by minimizing allocations and
//! separating concerns for better testability and maintainability.

use crate::estimator::Frame;
use crate::feature_tracker::StereoPatchTracker;
use crate::types::{Float, Matrix4x4};
use crate::{Result, VIOError};
use image::GrayImage;
use std::time::Instant;

/// Image buffer manager for zero-copy frame loading
///
/// Manages reusable buffers to avoid allocations in the hotpath
pub struct ImageBufferManager {
    left_buffer: Vec<u8>,
    right_buffer: Vec<u8>,
    image_width: u32,
    image_height: u32,
}

impl ImageBufferManager {
    /// Create new buffer manager with preallocated buffers
    pub fn new(width: u32, height: u32) -> Self {
        let capacity = (width * height) as usize;
        Self {
            left_buffer: Vec::with_capacity(capacity),
            right_buffer: Vec::with_capacity(capacity),
            image_width: width,
            image_height: height,
        }
    }

    /// Load left image into preallocated buffer (zero-copy)
    #[inline]
    pub fn load_left(&mut self, image_data: &[u8]) -> Result<()> {
        let expected_size = (self.image_width * self.image_height) as usize;
        if image_data.len() != expected_size {
            return Err(VIOError::Image(format!(
                "Invalid left image size: expected {}, got {}",
                expected_size,
                image_data.len()
            )));
        }
        self.left_buffer.clear();
        self.left_buffer.extend_from_slice(image_data);
        Ok(())
    }

    /// Load right image into preallocated buffer (zero-copy)
    #[inline]
    pub fn load_right(&mut self, image_data: &[u8]) -> Result<()> {
        let expected_size = (self.image_width * self.image_height) as usize;
        if image_data.len() != expected_size {
            return Err(VIOError::Image(format!(
                "Invalid right image size: expected {}, got {}",
                expected_size,
                image_data.len()
            )));
        }
        self.right_buffer.clear();
        self.right_buffer.extend_from_slice(image_data);
        Ok(())
    }

    /// Take ownership of left buffer and create GrayImage
    pub fn take_left_image(&mut self) -> Result<GrayImage> {
        let buf = std::mem::take(&mut self.left_buffer);
        GrayImage::from_raw(self.image_width, self.image_height, buf).ok_or_else(|| {
            VIOError::Image("Failed to create left GrayImage: size mismatch".to_string())
        })
    }

    /// Take ownership of right buffer and create GrayImage
    pub fn take_right_image(&mut self) -> Result<GrayImage> {
        let buf = std::mem::take(&mut self.right_buffer);
        GrayImage::from_raw(self.image_width, self.image_height, buf).ok_or_else(|| {
            VIOError::Image("Failed to create right GrayImage: size mismatch".to_string())
        })
    }
}

/// Keyframe decision result
#[derive(Debug, Clone)]
pub struct KeyframeDecision {
    pub is_keyframe: bool,
    pub reason: String,
    pub translation_norm: Float,
    pub rotation_norm: Float,
}

/// Keyframe decision maker
///
/// Determines when a new keyframe should be created based on:
/// - Visual criteria (translation/rotation thresholds)
/// - IMU criteria (from IMU-aided selector)
/// - Combined criteria
pub struct KeyframeDecider {
    translation_threshold: Float,
    rotation_threshold: Float,
}

impl KeyframeDecider {
    pub fn new(translation_threshold: Float, rotation_threshold: Float) -> Self {
        Self {
            translation_threshold,
            rotation_threshold,
        }
    }

    /// Determine if current pose should be a keyframe
    ///
    /// Returns decision with detailed reasoning for debugging
    pub fn decide(
        &self,
        current_pose: &Matrix4x4,
        last_keyframe_pose: &Matrix4x4,
        imu_decision: Option<(bool, String)>,
    ) -> Result<KeyframeDecision> {
        // Compute relative transformation
        let pose_inv = last_keyframe_pose
            .try_inverse()
            .ok_or_else(|| VIOError::Optimization("Failed to invert pose matrix".to_string()))?;

        let T_rel = current_pose * pose_inv;

        // Extract translation
        let t_rel = T_rel.fixed_view::<3, 1>(0, 3).into_owned();
        let translation_norm = t_rel.norm();

        // Extract rotation (using Euler angles for threshold checking)
        let R_rel = T_rel.fixed_view::<3, 3>(0, 0).into_owned();
        let rotation = nalgebra::Rotation3::from_matrix_unchecked(R_rel);
        let euler = rotation.euler_angles();
        let rotation_norm = (euler.0.abs() + euler.1.abs() + euler.2.abs()).abs();

        // Visual criteria
        let visual_keyframe = translation_norm > self.translation_threshold
            || rotation_norm > self.rotation_threshold;

        // IMU criteria
        let (is_imu_keyframe, imu_reason) = imu_decision.unwrap_or((false, String::new()));

        // Combined decision
        let is_keyframe = is_imu_keyframe || visual_keyframe;

        let reason = if is_imu_keyframe && !visual_keyframe {
            format!("IMU-aided: {}", imu_reason)
        } else if visual_keyframe {
            format!(
                "Visual: trans={:.3}m, rot={:.3}rad",
                translation_norm, rotation_norm
            )
        } else {
            imu_reason
        };

        Ok(KeyframeDecision {
            is_keyframe,
            reason,
            translation_norm,
            rotation_norm,
        })
    }
}

/// Feature tracking coordinator
///
/// Manages the feature tracking pipeline with timing and error handling
pub struct FeatureTrackingCoordinator;

impl FeatureTrackingCoordinator {
    /// Execute feature tracking on stereo pair
    ///
    /// Returns tracking duration for performance monitoring
    #[inline]
    pub fn track_stereo_frame<const LEVELS: u32>(
        tracker: &mut StereoPatchTracker<LEVELS>,
        left_image: &GrayImage,
        right_image: &GrayImage,
        frame: &mut Frame,
    ) -> std::time::Duration {
        let start = Instant::now();
        tracker.process_frame(left_image, right_image, frame);
        start.elapsed()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_image_buffer_manager_creation() {
        let manager = ImageBufferManager::new(640, 480);
        assert_eq!(manager.image_width, 640);
        assert_eq!(manager.image_height, 480);
    }

    #[test]
    fn test_image_buffer_load_valid() {
        let mut manager = ImageBufferManager::new(10, 10);
        let data = vec![128u8; 100];
        assert!(manager.load_left(&data).is_ok());
        assert!(manager.load_right(&data).is_ok());
    }

    #[test]
    fn test_image_buffer_load_invalid_size() {
        let mut manager = ImageBufferManager::new(10, 10);
        let data = vec![128u8; 50]; // Wrong size
        assert!(manager.load_left(&data).is_err());
    }

    #[test]
    fn test_keyframe_decider_creation() {
        let decider = KeyframeDecider::new(0.5, 0.1);
        assert_eq!(decider.translation_threshold, 0.5);
        assert_eq!(decider.rotation_threshold, 0.1);
    }

    #[test]
    fn test_keyframe_decision_no_motion() {
        let decider = KeyframeDecider::new(0.5, 0.1);
        let pose = Matrix4x4::identity();
        let decision = decider.decide(&pose, &pose, None).unwrap();
        assert!(!decision.is_keyframe);
        assert!(decision.translation_norm < 0.01);
    }

    #[test]
    fn test_keyframe_decision_large_translation() {
        let decider = KeyframeDecider::new(0.5, 0.1);
        let pose1 = Matrix4x4::identity();
        let mut pose2 = Matrix4x4::identity();
        pose2[(0, 3)] = 1.0; // 1m translation in X
        let decision = decider.decide(&pose2, &pose1, None).unwrap();
        assert!(decision.is_keyframe);
        assert!(decision.translation_norm > 0.5);
    }
}
