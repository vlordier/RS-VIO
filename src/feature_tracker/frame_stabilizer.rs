//! Rotation-only multi-frame stabilization for improved feature detection
//!
//! This module implements geometric super-resolution using gyro-derived rotation
//! to warp and accumulate multiple frames. This approach:
//! - Stabilizes image sequences for better feature detection
//! - Denoises through temporal averaging
//! - Sharpens features through sub-pixel alignment
//! - Works without depth (rotation-only SO(3) warping)
//!
//! Perfect for drone VIO where IMU provides accurate rotation measurements.

use image::{GrayImage, ImageBuffer, Luma};
use nalgebra as na;
use std::collections::VecDeque;

/// Configuration for rotation-based frame stabilization
#[derive(Debug, Clone)]
pub struct StabilizerConfig {
    /// Number of frames to accumulate (recommended: 3-5)
    pub buffer_size: usize,
    /// Weight for exponential moving average (0.0-1.0, higher = more current frame)
    pub accumulation_weight: f32,
    /// Enable/disable stabilization (for A/B testing)
    pub enabled: bool,
    /// Minimum rotation magnitude to trigger accumulation (rad)
    pub min_rotation_threshold: f32,
}

impl Default for StabilizerConfig {
    fn default() -> Self {
        Self {
            buffer_size: 5,
            accumulation_weight: 0.7,
            enabled: true,
            min_rotation_threshold: 0.001, // ~0.057 degrees
        }
    }
}

/// Frame with associated IMU rotation
#[derive(Clone)]
struct StabilizedFrame {
    /// Grayscale image
    image: GrayImage,
    /// Rotation from reference frame (SO(3))
    rotation: na::Rotation3<f32>,
    /// Timestamp for temporal ordering
    #[allow(dead_code)]
    timestamp: f64,
}

/// Rotation-only multi-frame stabilizer
///
/// Accumulates N frames by warping them into a reference coordinate frame using
/// gyro-derived rotation. The output is a stabilized, denoised image suitable
/// for high-quality feature detection.
pub struct FrameStabilizer {
    config: StabilizerConfig,
    /// Ring buffer of recent frames
    frame_buffer: VecDeque<StabilizedFrame>,
    /// Reference frame (oldest in buffer)
    reference_frame: Option<StabilizedFrame>,
    /// Accumulated stabilized image
    accumulated: Option<ImageBuffer<Luma<f32>, Vec<f32>>>,
    /// Camera intrinsics for projection
    fx: f32,
    fy: f32,
    cx: f32,
    cy: f32,
}

impl FrameStabilizer {
    /// Create a new frame stabilizer
    pub fn new(config: StabilizerConfig, fx: f32, fy: f32, cx: f32, cy: f32) -> Self {
        let buffer_size = config.buffer_size;
        Self {
            config,
            frame_buffer: VecDeque::with_capacity(buffer_size),
            reference_frame: None,
            accumulated: None,
            fx,
            fy,
            cx,
            cy,
        }
    }

    /// Process a new frame with its gyro-derived rotation
    ///
    /// # Arguments
    /// * `image` - Input grayscale image
    /// * `rotation` - Rotation relative to world frame (from IMU integration)
    /// * `timestamp` - Frame timestamp
    ///
    /// # Returns
    /// Stabilized, accumulated image (or original if stabilization disabled)
    pub fn process_frame(
        &mut self,
        image: &GrayImage,
        rotation: na::Rotation3<f32>,
        timestamp: f64,
    ) -> GrayImage {
        if !self.config.enabled {
            return image.clone();
        }

        let current = StabilizedFrame {
            image: image.clone(),
            rotation,
            timestamp,
        };

        // Initialize reference frame on first call
        if self.reference_frame.is_none() {
            self.reference_frame = Some(current.clone());
            self.accumulated = Some(self.image_to_f32(image));
            return image.clone();
        }

        // Add to buffer
        self.frame_buffer.push_back(current.clone());
        if self.frame_buffer.len() > self.config.buffer_size {
            self.frame_buffer.pop_front();
            // Update reference to oldest frame in buffer
            if let Some(oldest) = self.frame_buffer.front() {
                self.reference_frame = Some(oldest.clone());
            }
        }

        // Warp current frame to reference and accumulate
        let reference = self.reference_frame.as_ref().unwrap();
        let warped = self.warp_to_reference(&current, reference);

        // Exponentially weighted moving average
        let weight = self.config.accumulation_weight;
        if let Some(ref mut acc) = self.accumulated {
            Self::blend_images_static(acc, &warped, weight);
        } else {
            self.accumulated = Some(warped);
        }

        // Convert accumulated float image back to u8
        self.f32_to_image(self.accumulated.as_ref().unwrap())
    }

    /// Warp frame to reference coordinate system using rotation only
    fn warp_to_reference(
        &self,
        current: &StabilizedFrame,
        reference: &StabilizedFrame,
    ) -> ImageBuffer<Luma<f32>, Vec<f32>> {
        let width = current.image.width();
        let height = current.image.height();

        // Compute relative rotation: R_current_to_ref
        let r_rel = reference.rotation.inverse() * current.rotation;

        // Create output image
        let mut warped =
            ImageBuffer::<Luma<f32>, Vec<f32>>::from_pixel(width, height, Luma([0.0]));

        // For each pixel in reference frame, find corresponding pixel in current
        for y in 0..height {
            for x in 0..width {
                // Back-project pixel to unit ray in reference frame
                let x_norm = ((x as f32) - self.cx) / self.fx;
                let y_norm = ((y as f32) - self.cy) / self.fy;
                let ray_ref = na::Vector3::new(x_norm, y_norm, 1.0).normalize();

                // Rotate ray to current frame
                let ray_cur = r_rel * ray_ref;

                // Project back to image plane
                let x_cur = self.fx * (ray_cur.x / ray_cur.z) + self.cx;
                let y_cur = self.fy * (ray_cur.y / ray_cur.z) + self.cy;

                // Bilinear interpolation from current image
                if let Some(value) = self.bilinear_sample(&current.image, x_cur, y_cur) {
                    warped.put_pixel(x, y, Luma([value]));
                }
            }
        }

        warped
    }

    /// Bilinear interpolation sampling
    fn bilinear_sample(&self, image: &GrayImage, x: f32, y: f32) -> Option<f32> {
        let width = image.width() as f32;
        let height = image.height() as f32;

        // Check bounds
        if x < 0.0 || y < 0.0 || x >= width - 1.0 || y >= height - 1.0 {
            return None;
        }

        let x0 = x.floor() as u32;
        let y0 = y.floor() as u32;
        let x1 = x0 + 1;
        let y1 = y0 + 1;

        let dx = x - (x0 as f32);
        let dy = y - (y0 as f32);

        let v00 = image.get_pixel(x0, y0).0[0] as f32;
        let v10 = image.get_pixel(x1, y0).0[0] as f32;
        let v01 = image.get_pixel(x0, y1).0[0] as f32;
        let v11 = image.get_pixel(x1, y1).0[0] as f32;

        let v0 = v00 * (1.0 - dx) + v10 * dx;
        let v1 = v01 * (1.0 - dx) + v11 * dx;
        let v = v0 * (1.0 - dy) + v1 * dy;

        Some(v)
    }

    /// Blend two float images with exponential weighting
    fn blend_images_static(
        accumulator: &mut ImageBuffer<Luma<f32>, Vec<f32>>,
        new_frame: &ImageBuffer<Luma<f32>, Vec<f32>>,
        weight: f32,
    ) {
        assert_eq!(accumulator.dimensions(), new_frame.dimensions());

        for (acc_pixel, new_pixel) in accumulator.pixels_mut().zip(new_frame.pixels()) {
            acc_pixel.0[0] = acc_pixel.0[0] * (1.0 - weight) + new_pixel.0[0] * weight;
        }
    }

    /// Convert u8 image to f32 for accumulation
    fn image_to_f32(&self, image: &GrayImage) -> ImageBuffer<Luma<f32>, Vec<f32>> {
        ImageBuffer::from_fn(image.width(), image.height(), |x, y| {
            Luma([image.get_pixel(x, y).0[0] as f32])
        })
    }

    /// Convert f32 accumulated image back to u8
    fn f32_to_image(&self, image: &ImageBuffer<Luma<f32>, Vec<f32>>) -> GrayImage {
        ImageBuffer::from_fn(image.width(), image.height(), |x, y| {
            let value = image.get_pixel(x, y).0[0].clamp(0.0, 255.0) as u8;
            Luma([value])
        })
    }

    /// Reset the stabilizer (useful when scene changes dramatically)
    pub fn reset(&mut self) {
        self.frame_buffer.clear();
        self.reference_frame = None;
        self.accumulated = None;
    }

    /// Get current buffer size
    pub fn buffer_len(&self) -> usize {
        self.frame_buffer.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stabilizer_creation() {
        let config = StabilizerConfig::default();
        let stabilizer = FrameStabilizer::new(config, 500.0, 500.0, 320.0, 240.0);
        assert_eq!(stabilizer.buffer_len(), 0);
    }

    #[test]
    fn test_stabilizer_disabled() {
        let mut config = StabilizerConfig::default();
        config.enabled = false;

        let mut stabilizer = FrameStabilizer::new(config, 500.0, 500.0, 320.0, 240.0);
        let input = GrayImage::new(640, 480);
        let rotation = na::Rotation3::identity();

        let output = stabilizer.process_frame(&input, rotation, 0.0);
        assert_eq!(output.dimensions(), input.dimensions());
    }

    #[test]
    fn test_frame_accumulation() {
        let config = StabilizerConfig::default();
        let mut stabilizer = FrameStabilizer::new(config, 500.0, 500.0, 320.0, 240.0);

        let rotation = na::Rotation3::identity();

        // Add multiple frames
        for i in 0..7 {
            let image = GrayImage::new(640, 480);
            stabilizer.process_frame(&image, rotation, i as f64 * 0.033);
        }

        // Should maintain buffer size limit
        assert_eq!(stabilizer.buffer_len(), 5);
    }

    #[test]
    fn test_bilinear_interpolation() {
        let config = StabilizerConfig::default();
        let stabilizer = FrameStabilizer::new(config, 500.0, 500.0, 320.0, 240.0);

        let mut image = GrayImage::new(10, 10);
        image.put_pixel(5, 5, Luma([100]));
        image.put_pixel(6, 5, Luma([200]));
        image.put_pixel(5, 6, Luma([100]));
        image.put_pixel(6, 6, Luma([200]));

        // Sample at center of 4 pixels
        let value = stabilizer.bilinear_sample(&image, 5.5, 5.5);
        assert!(value.is_some());
        assert!((value.unwrap() - 150.0).abs() < 1.0); // Average should be 150
    }
}
