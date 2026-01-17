//! Frame processing workspace with preallocated buffers.
//!
//! This module provides reusable buffer management for per-frame processing,
//! eliminating allocation overhead from the hot path.
//!
//! ## Motivation
//!
//! Visual-inertial odometry processes one frame at a time in a tight loop:
//! ```text
//! Loop (30-120 Hz):
//!   1. Read stereo images [O(W×H) bytes]
//!   2. Feature detection [O(features) allocations]
//!   3. Feature tracking [O(features) work buffers]
//!   4. Optimization [O(state_size²) matrices]
//! ```
//!
//! Each allocation adds latency variance. A workspace pre-reserves all buffers
//! once during init, then reuses them across frames via `.reset()` methods.
//!
//! ## Design
//!
//! - **Ownership**: Estimator owns the workspace; passes &mut to frame pipeline.
//! - **Borrowing**: Subsystems borrow buffers during frame processing, return borrows to workspace.
//! - **Reset semantics**: Buffers are cleared (not deallocated) between frames.
//! - **Capacity bounds**: Workspace caps feature count, image size, etc. based on config.

use crate::datasets::ImuData;
use crate::feature_tracker::Feature;
use crate::types::Float;
use nalgebra as na;

/// Configuration for workspace buffer sizes.
#[derive(Debug, Clone)]
pub struct WorkspaceConfig {
    /// Maximum image width (pixels)
    pub max_image_width: u32,
    /// Maximum image height (pixels)
    pub max_image_height: u32,
    /// Maximum features per frame
    pub max_features_per_frame: usize,
    /// Maximum IMU samples per frame
    pub max_imu_samples: usize,
    /// Reserve additional capacity (default 1.2x = 20% headroom)
    pub capacity_headroom: f32,
}

impl Default for WorkspaceConfig {
    fn default() -> Self {
        Self {
            max_image_width: 1280,
            max_image_height: 720,
            max_features_per_frame: 300,
            max_imu_samples: 100,
            capacity_headroom: 1.2,
        }
    }
}

/// Preallocated buffers for frame processing.
///
/// **Ownership**: Created once per Estimator, reused across all frames.
/// **Lifetime**: 'static relative to containing Estimator.
pub struct FrameWorkspace {
    config: WorkspaceConfig,

    // Image buffers (RGBA or grayscale, depending on source)
    left_image_buffer: Vec<u8>,
    right_image_buffer: Vec<u8>,

    // Feature detection / tracking buffers
    left_features: Vec<Feature>,
    right_features: Vec<Feature>,

    // IMU sample buffer
    imu_samples: Vec<ImuData>,

    // Temporary RANSAC / matching workspace
    ransac_buffer: Vec<(usize, usize)>, // match pairs
    descriptor_buffer: Vec<Float>,      // descriptors
    residual_buffer: Vec<Float>,        // for scoring

    // RANSAC-specific buffers (for hypothesis sampling and inlier tracking)
    ransac_hypothesis_samples: Vec<usize>, // sampled point indices (used per RANSAC iteration)
    ransac_inlier_mask: Vec<bool>,         // inlier flags per point
    ransac_residuals: Vec<Float>,          // residual per point for scoring

    // Scratch matrices for optimization
    scratch_matrix_6x6: na::Matrix6<Float>,
    scratch_vector_6: na::Vector6<Float>,
}

impl FrameWorkspace {
    /// Create a new workspace with configuration.
    pub fn new(config: WorkspaceConfig) -> Self {
        let img_capacity = ((config.max_image_width as usize)
            * (config.max_image_height as usize) as usize) as f32
            * config.capacity_headroom;
        let features_capacity =
            (config.max_features_per_frame as f32 * config.capacity_headroom) as usize;
        let imu_capacity = (config.max_imu_samples as f32 * config.capacity_headroom) as usize;

        Self {
            config,
            left_image_buffer: Vec::with_capacity(img_capacity as usize),
            right_image_buffer: Vec::with_capacity(img_capacity as usize),
            left_features: Vec::with_capacity(features_capacity),
            right_features: Vec::with_capacity(features_capacity),
            imu_samples: Vec::with_capacity(imu_capacity),
            ransac_buffer: Vec::with_capacity(features_capacity),
            descriptor_buffer: Vec::with_capacity(features_capacity * 256), // ORB: 256 bits
            residual_buffer: Vec::with_capacity(features_capacity),
            ransac_hypothesis_samples: Vec::with_capacity(features_capacity),
            ransac_inlier_mask: Vec::with_capacity(features_capacity),
            ransac_residuals: Vec::with_capacity(features_capacity),
            scratch_matrix_6x6: na::Matrix6::zeros(),
            scratch_vector_6: na::Vector6::zeros(),
        }
    }

    /// Reset all buffers for next frame (clear, do not deallocate).
    pub fn reset(&mut self) {
        self.left_image_buffer.clear();
        self.right_image_buffer.clear();
        self.left_features.clear();
        self.right_features.clear();
        self.imu_samples.clear();
        self.ransac_buffer.clear();
        self.descriptor_buffer.clear();
        self.residual_buffer.clear();
        self.ransac_hypothesis_samples.clear();
        self.ransac_inlier_mask.clear();
        self.ransac_residuals.clear();
        // Scratch matrices are overwritten, no need to clear
    }
}

impl Default for FrameWorkspace {
    fn default() -> Self {
        Self::new(WorkspaceConfig::default())
    }
}

impl FrameWorkspace {
    /// Load left image into workspace buffer (borrowed ownership).
    ///
    /// # Returns
    /// - `Ok(())` if image fits within capacity
    /// - `Err` if image size exceeds configured max
    pub fn load_left_image(&mut self, pixels: &[u8]) -> Result<(), String> {
        if pixels.len()
            > self.config.max_image_width as usize * self.config.max_image_height as usize
        {
            return Err(format!(
                "Left image too large: {} bytes > {} bytes max",
                pixels.len(),
                self.config.max_image_width as usize * self.config.max_image_height as usize
            ));
        }
        self.left_image_buffer.clear();
        self.left_image_buffer.extend_from_slice(pixels);
        Ok(())
    }

    /// Load right image into workspace buffer.
    pub fn load_right_image(&mut self, pixels: &[u8]) -> Result<(), String> {
        if pixels.len()
            > self.config.max_image_width as usize * self.config.max_image_height as usize
        {
            return Err(format!(
                "Right image too large: {} bytes > {} bytes max",
                pixels.len(),
                self.config.max_image_width as usize * self.config.max_image_height as usize
            ));
        }
        self.right_image_buffer.clear();
        self.right_image_buffer.extend_from_slice(pixels);
        Ok(())
    }

    /// Borrow left image buffer as slice.
    pub fn left_image(&self) -> &[u8] {
        &self.left_image_buffer
    }

    /// Move out the left image buffer (caller takes ownership).
    pub fn take_left_image_buffer(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.left_image_buffer)
    }

    /// Return an owned left image buffer to the workspace for reuse.
    pub fn put_left_image_buffer(&mut self, buf: Vec<u8>) {
        self.left_image_buffer = buf;
    }

    /// Borrow right image buffer as slice.
    pub fn right_image(&self) -> &[u8] {
        &self.right_image_buffer
    }

    /// Move out the right image buffer (caller takes ownership).
    pub fn take_right_image_buffer(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.right_image_buffer)
    }

    /// Return an owned right image buffer to the workspace for reuse.
    pub fn put_right_image_buffer(&mut self, buf: Vec<u8>) {
        self.right_image_buffer = buf;
    }

    /// Borrow mutable left features.
    pub fn left_features_mut(&mut self) -> &mut Vec<Feature> {
        &mut self.left_features
    }

    /// Borrow immutable left features.
    pub fn left_features(&self) -> &[Feature] {
        &self.left_features
    }

    /// Borrow mutable right features.
    pub fn right_features_mut(&mut self) -> &mut Vec<Feature> {
        &mut self.right_features
    }

    /// Borrow immutable right features.
    pub fn right_features(&self) -> &[Feature] {
        &self.right_features
    }

    /// Borrow mutable IMU samples.
    pub fn imu_samples_mut(&mut self) -> &mut Vec<ImuData> {
        &mut self.imu_samples
    }

    /// Borrow immutable IMU samples.
    pub fn imu_samples(&self) -> &[ImuData] {
        &self.imu_samples
    }

    /// Add IMU sample to buffer.
    pub fn push_imu_sample(&mut self, sample: &ImuData) -> Result<(), String> {
        if self.imu_samples.len() >= self.config.max_imu_samples {
            return Err("IMU buffer full".to_string());
        }
        self.imu_samples.push(sample.clone());
        Ok(())
    }

    /// Borrow scratch RANSAC match buffer.
    pub fn ransac_buffer_mut(&mut self) -> &mut Vec<(usize, usize)> {
        &mut self.ransac_buffer
    }

    /// Borrow scratch descriptor buffer.
    pub fn descriptor_buffer_mut(&mut self) -> &mut Vec<Float> {
        &mut self.descriptor_buffer
    }

    /// Borrow scratch residual buffer.
    pub fn residual_buffer_mut(&mut self) -> &mut Vec<Float> {
        &mut self.residual_buffer
    }

    /// Borrow RANSAC hypothesis sample buffer (point indices for sampling).
    pub fn ransac_hypothesis_samples_mut(&mut self) -> &mut Vec<usize> {
        &mut self.ransac_hypothesis_samples
    }

    /// Borrow RANSAC inlier mask buffer (boolean flags per point).
    pub fn ransac_inlier_mask_mut(&mut self) -> &mut Vec<bool> {
        &mut self.ransac_inlier_mask
    }

    /// Borrow immutable RANSAC inlier mask.
    pub fn ransac_inlier_mask(&self) -> &[bool] {
        &self.ransac_inlier_mask
    }

    /// Borrow RANSAC residual buffer (scores for inlier verification).
    pub fn ransac_residuals_mut(&mut self) -> &mut Vec<Float> {
        &mut self.ransac_residuals
    }

    /// Borrow immutable RANSAC residuals.
    pub fn ransac_residuals(&self) -> &[Float] {
        &self.ransac_residuals
    }

    /// Get mutable references to both inlier mask and residuals (for RANSAC processing).
    /// This avoids borrow checker issues with getting both mut refs separately.
    pub fn ransac_buffers_mut(&mut self) -> (&mut Vec<bool>, &mut Vec<Float>) {
        (&mut self.ransac_inlier_mask, &mut self.ransac_residuals)
    }

    /// Get mutable references to hypothesis samples and inlier mask (for feature tracker RANSAC).
    /// This avoids borrow checker issues with getting both mut refs separately.
    pub fn feature_ransac_buffers_mut(&mut self) -> (&mut Vec<usize>, &mut Vec<bool>) {
        (
            &mut self.ransac_hypothesis_samples,
            &mut self.ransac_inlier_mask,
        )
    }

    /// Borrow scratch 6×6 matrix.
    pub fn scratch_matrix_6x6_mut(&mut self) -> &mut na::Matrix6<Float> {
        &mut self.scratch_matrix_6x6
    }

    /// Borrow scratch 6-vector.
    pub fn scratch_vector_6_mut(&mut self) -> &mut na::Vector6<Float> {
        &mut self.scratch_vector_6
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn workspace_creation_default() {
        let ws = FrameWorkspace::new(WorkspaceConfig::default());
        // Preallocated with capacity headroom (1.2x default = 1280*720*1.2)
        let expected_img_capacity = ((1280 * 720) as f32 * 1.2) as usize;
        assert_eq!(ws.left_image_buffer.capacity(), expected_img_capacity);
        assert!(ws.left_features.is_empty()); // empty but reserved capacity
        assert!(ws.imu_samples.is_empty());
    }

    #[test]
    fn workspace_load_image() {
        let mut ws = FrameWorkspace::new(WorkspaceConfig::default());
        let pixels = vec![0u8; 1280 * 720];
        assert!(ws.load_left_image(&pixels).is_ok());
        assert_eq!(ws.left_image().len(), 1280 * 720);
    }

    #[test]
    fn workspace_reject_oversized_image() {
        let mut config = WorkspaceConfig::default();
        config.max_image_width = 640;
        config.max_image_height = 480;
        let mut ws = FrameWorkspace::new(config);

        let oversized = vec![0u8; 1280 * 720]; // 3× capacity
        assert!(ws.load_left_image(&oversized).is_err());
    }

    #[test]
    fn workspace_reset() {
        let mut ws = FrameWorkspace::new(WorkspaceConfig::default());
        let pixels = vec![0u8; 640];
        ws.load_left_image(&pixels).unwrap();

        ws.reset();
        assert!(ws.left_image().is_empty());
        assert!(ws.left_features().is_empty());
    }

    #[test]
    fn workspace_imu_buffer_overflow() {
        let mut config = WorkspaceConfig::default();
        config.max_imu_samples = 2;
        let mut ws = FrameWorkspace::new(config);

        let sample = ImuData {
            timestamp: 0,
            accel: [0.0, 0.0, 0.0],
            gyro: [0.0, 0.0, 0.0],
        };

        assert!(ws.push_imu_sample(&sample).is_ok());
        assert!(ws.push_imu_sample(&sample).is_ok());
        assert!(ws.push_imu_sample(&sample).is_err()); // Full
    }
}
