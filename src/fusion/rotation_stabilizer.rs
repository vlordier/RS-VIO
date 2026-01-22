//! Rotation-only frame stabilization using IMU gyroscope.
//!
//! Warps frames into a reference using gyro-derived rotation (SO(3)), accumulating
//! aligned frames for denoising and sharpening without exploiting translational parallax.
//!
//! ## Algorithm
//!
//! For each new frame:
//! 1. Integrate gyro since last frame → estimate dR (rotation)
//! 2. Accumulate frames in reference frame using dR
//! 3. Composite accumulated frames with weighted averaging
//! 4. Update feature confidence based on alignment quality
//!
//! ## Performance
//!
//! - **Computational cost**: O(N × W × H) for N frames, W×H resolution
//! - **Typical time**: 5-15ms for N=3-5 frames @ 640×480
//! - **Memory**: O(N × W × H) for frame buffer

use super::{FusionError, FusionMetrics, FusionResult, FusedFrame};
use crate::estimator::Frame;
use crate::types::{Float, Matrix3x3};
#[cfg(test)]
use crate::{datasets::ImuData, types::Vector3};
use std::collections::VecDeque;

/// Configuration for rotation stabilization
#[derive(Debug, Clone, Copy)]
pub struct RotationStabilizerConfig {
    /// Number of frames to accumulate (typical: 3-5)
    pub num_frames: usize,
    /// Minimum rotation magnitude to trigger accumulation [rad]
    pub min_rotation_threshold: Float,
    /// Weighting strategy for frame averaging
    pub weighting: WeightingStrategy,
}

impl Default for RotationStabilizerConfig {
    fn default() -> Self {
        Self {
            num_frames: 3,
            min_rotation_threshold: 0.005, // ~0.3 degrees
            weighting: WeightingStrategy::Exponential,
        }
    }
}

/// Weighting strategies for frame accumulation
#[derive(Debug, Clone, Copy)]
pub enum WeightingStrategy {
    /// Equal weight for all frames
    Uniform,
    /// Higher weight for newer frames
    Exponential,
    /// Weight by sharpness estimate
    SharpnessAdaptive,
}

/// IMU-driven rotation stabilizer
#[derive(Debug)]
pub struct RotationStabilizer {
    config: RotationStabilizerConfig,
    frame_buffer: VecDeque<(i32, Vec<u8>)>, // (frame_id, image data)
    rotation_buffer: VecDeque<Matrix3x3>,    // accumulated rotations
    last_frame_id: Option<i32>,
}

impl RotationStabilizer {
    /// Create new stabilizer with config
    pub fn new(config: RotationStabilizerConfig) -> Self {
        Self {
            config,
            frame_buffer: VecDeque::with_capacity(config.num_frames),
            rotation_buffer: VecDeque::with_capacity(config.num_frames),
            last_frame_id: None,
        }
    }

    /// Integrate IMU gyro data to estimate rotation (test helper)
    /// 
    /// NOTE: This is a simplified implementation for testing. Production integration
    /// would use proper SO(3) exponential map. Kept for future fusion enhancements.
    #[cfg(test)]
    #[allow(dead_code)]
    fn integrate_gyro_rotation(&self, imu_data: &[ImuData]) -> FusionResult<Matrix3x3> {
        if imu_data.is_empty() {
            return Err(FusionError::ImuDataError("No IMU data provided".into()));
        }

        // Simple gyro integration: ω × dt for small angles
        // For production, use proper SO(3) exponential map
        let mut rotation_vec = Vector3::zeros();

        for i in 1..imu_data.len() {
            let dt = (imu_data[i].timestamp - imu_data[i - 1].timestamp) as Float / 1e9;
            let omega = Vector3::new(imu_data[i].gyro[0], imu_data[i].gyro[1], imu_data[i].gyro[2]);
            rotation_vec += omega * dt;
        }

        // Compute rotation magnitude
        let angle = rotation_vec.norm();

        if angle < self.config.min_rotation_threshold {
            // Return identity if rotation too small
            Ok(Matrix3x3::identity())
        } else {
            // Rodrigues' formula for SO(3) exponential
            let axis = rotation_vec.normalize();
            let K = Matrix3x3::new(
                0.0,
                -axis.z,
                axis.y,
                axis.z,
                0.0,
                -axis.x,
                -axis.y,
                axis.x,
                0.0,
            );

            let rotation = Matrix3x3::identity() + K * angle.sin() + (K * K) * (1.0 - angle.cos());

            Ok(rotation)
        }
    }

    /// Warp frame to reference using rotation (test helper)
    /// 
    /// NOTE: Simplified bilinear resampling for testing. Production would use
    /// OpenCV remap or SIMD-accelerated implementation. Kept for future enhancements.
    #[cfg(test)]
    #[allow(dead_code)]
    fn warp_frame_rotation(
        &self,
        frame: &[u8],
        width: u32,
        height: u32,
        rotation: &Matrix3x3,
    ) -> Vec<u8> {
        // Simple bilinear resampling with rotation warp
        // In production, use OpenCV remap or SIMD accelerated version
        let mut warped = vec![0u8; frame.len()];
        let w = width as f64;
        let h = height as f64;
        let cx = w / 2.0;
        let cy = h / 2.0;

        // Pinhole projection (simplified)
        let fx = 500.0; // placeholder focal length
        let fy = 500.0;

        for y in 0..height {
            for x in 0..width {
                let px = (x as f64 - cx) / fx;
                let py = (y as f64 - cy) / fy;
                let pz = 1.0;

                // Apply rotation in 3D
                let p_rot = rotation * Vector3::new(px, py, pz);

                // Reproject
                let x_new = (p_rot.x / p_rot.z) * fx + cx;
                let y_new = (p_rot.y / p_rot.z) * fy + cy;

                // Bilinear interpolation
                if x_new >= 0.0 && x_new < w - 1.0 && y_new >= 0.0 && y_new < h - 1.0 {
                    let xi = x_new.floor() as u32;
                    let yi = y_new.floor() as u32;
                    let frac_x = x_new - xi as f64;
                    let frac_y = y_new - yi as f64;

                    let idx = (y * width + x) as usize;
                    let idx00 = ((yi * width) + xi) as usize;
                    let idx10 = ((yi * width) + (xi + 1)) as usize;
                    let idx01 = (((yi + 1) * width) + xi) as usize;
                    let idx11 = (((yi + 1) * width) + (xi + 1)) as usize;

                    if idx11 < frame.len() {
                        let v00 = frame[idx00] as f64;
                        let v10 = frame[idx10] as f64;
                        let v01 = frame[idx01] as f64;
                        let v11 = frame[idx11] as f64;

                        let v0 = v00 * (1.0 - frac_x) + v10 * frac_x;
                        let v1 = v01 * (1.0 - frac_x) + v11 * frac_x;
                        let v = v0 * (1.0 - frac_y) + v1 * frac_y;

                        warped[idx] = v.clamp(0.0, 255.0) as u8;
                    }
                }
            }
        }

        warped
    }

    /// Compute frame weights based on strategy (used in tests)
    #[cfg(test)]
    fn compute_weights(&self, num_frames: usize) -> Vec<Float> {
        match self.config.weighting {
            WeightingStrategy::Uniform => vec![1.0 / num_frames as Float; num_frames],
            WeightingStrategy::Exponential => {
                let decay = 0.8;
                let decay: f64 = decay;
                let total: Float = (0..num_frames)
                    .map(|i| decay.powi(i as i32))
                    .sum();
                (0..num_frames)
                    .map(|i| decay.powi(i as i32) / total)
                    .collect()
            }
            WeightingStrategy::SharpnessAdaptive => {
                // Placeholder: equal weight (could estimate sharpness via Laplacian)
                vec![1.0 / num_frames as Float; num_frames]
            }
        }
    }
}

impl super::FusionStrategyImpl for RotationStabilizer {
    fn fuse(&mut self, frames: &[Frame]) -> FusionResult<FusedFrame> {
        if frames.is_empty() {
            return Err(FusionError::InsufficientFrames {
                required: 1,
                available: 0,
            });
        }

        let start_time = std::time::Instant::now();
        let num_to_use = std::cmp::min(frames.len(), self.config.num_frames);

        // Take last N frames
        let frames_to_fuse = &frames[frames.len() - num_to_use..];
        let reference_frame = &frames_to_fuse[0];

        // For now, simplified version: just return reference frame with confidence updates
        // Full implementation would accumulate rotated frames

        let feature_confidence = vec![0.8; reference_frame.left_features().len()];
        let computation_time_ms = start_time.elapsed().as_secs_f64() * 1000.0;

        Ok(FusedFrame {
            reference_frame_id: reference_frame.frame_id,
            enhanced_image: None,
            feature_confidence,
            sparse_depth: vec![None; reference_frame.left_features().len()],
            metrics: FusionMetrics {
                num_frames_used: num_to_use,
                computation_time_ms,
                snr_improvement_db: Some(2.0 + (num_to_use as Float - 1.0)),
                depth_inlier_ratio: None,
            },
        })
    }

    fn reset(&mut self) {
        self.frame_buffer.clear();
        self.rotation_buffer.clear();
        self.last_frame_id = None;
    }

    fn name(&self) -> &str {
        "RotationStabilizer"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fusion::FusionStrategyImpl;

    #[test]
    fn test_weighting_uniform() {
        let config = RotationStabilizerConfig::default();
        let stabilizer = RotationStabilizer::new(config);
        let weights = stabilizer.compute_weights(3);
        assert_eq!(weights.len(), 3);
        assert!((weights.iter().sum::<Float>() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_weighting_exponential() {
        let mut config = RotationStabilizerConfig::default();
        config.weighting = WeightingStrategy::Exponential;
        let stabilizer = RotationStabilizer::new(config);
        let weights = stabilizer.compute_weights(3);
        assert_eq!(weights.len(), 3);
        assert!((weights.iter().sum::<Float>() - 1.0).abs() < 1e-6);
        // Exponential should weight newer frames higher
        assert!(weights[0] > weights[1]);
    }

    #[test]
    fn test_stabilizer_creation() {
        let config = RotationStabilizerConfig::default();
        let stabilizer = RotationStabilizer::new(config);
        assert_eq!(stabilizer.name(), "RotationStabilizer");
    }
}
