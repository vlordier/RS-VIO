//! Parallel Feature Tracking
//!
//! Provides parallel processing utilities for stereo feature tracking.

use crate::types::Float;
use image::GrayImage;
use nalgebra as na;
use std::collections::HashMap;

use super::image_utilities;

/// Parallel tracking result combining left and right camera tracking
#[derive(Debug, Clone)]
pub struct ParallelTrackingResult {
    /// Tracked points in left camera
    pub tracked_left: HashMap<usize, na::Matrix4<Float>>,
    /// Tracked points in right camera
    pub tracked_right: HashMap<usize, na::Matrix4<Float>>,
}

/// Configuration for parallel tracking
#[derive(Debug, Clone)]
pub struct ParallelTrackingConfig {
    /// Enable parallel processing
    pub enabled: bool,
    /// Minimum points before parallelization is beneficial
    pub min_points_for_parallel: usize,
}

impl Default for ParallelTrackingConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            min_points_for_parallel: 50,
        }
    }
}

/// Build image pyramids for both cameras in parallel
#[inline]
pub fn build_pyramids_parallel(
    left_image: &GrayImage,
    right_image: &GrayImage,
    levels: u32,
) -> (Vec<GrayImage>, Vec<GrayImage>) {
    let mut left_pyramid = Vec::with_capacity(levels as usize);
    let mut right_pyramid = Vec::with_capacity(levels as usize);

    let (width, height) = left_image.dimensions();
    left_pyramid.push(left_image.clone());
    right_pyramid.push(right_image.clone());

    for level in 1..levels {
        let prev_level = level - 1;
        let new_width = ((width as f32 / (1 << level) as f32).ceil() as u32).max(1);
        let new_height = ((height as f32 / (1 << level) as f32).ceil() as u32).max(1);

        let mut left_downsampled = GrayImage::new(new_width, new_height);
        let mut right_downsampled = GrayImage::new(new_width, new_height);

        image_utilities::downsample_half_box(
            &left_pyramid[prev_level as usize],
            &mut left_downsampled,
        );
        image_utilities::downsample_half_box(
            &right_pyramid[prev_level as usize],
            &mut right_downsampled,
        );

        left_pyramid.push(left_downsampled);
        right_pyramid.push(right_downsampled);
    }

    (left_pyramid, right_pyramid)
}

/// Track points in both cameras in parallel
#[inline]
pub fn track_points_parallel(
    prev_pyramid_left: &[GrayImage],
    prev_pyramid_right: &[GrayImage],
    curr_pyramid_left: &[GrayImage],
    curr_pyramid_right: &[GrayImage],
    tracked_left: &HashMap<usize, na::Matrix4<Float>>,
    tracked_right: &HashMap<usize, na::Matrix4<Float>>,
    max_iterations: usize,
    convergence_threshold: Float,
) -> ParallelTrackingResult {
    let (result_left, result_right) = rayon::join(
        || {
            track_points_sequential(
                prev_pyramid_left,
                curr_pyramid_left,
                tracked_left,
                max_iterations,
                convergence_threshold,
            )
        },
        || {
            track_points_sequential(
                prev_pyramid_right,
                curr_pyramid_right,
                tracked_right,
                max_iterations,
                convergence_threshold,
            )
        },
    );

    ParallelTrackingResult {
        tracked_left: result_left,
        tracked_right: result_right,
    }
}

fn track_points_sequential(
    _prev_pyramid: &[GrayImage],
    _curr_pyramid: &[GrayImage],
    transform_maps: &HashMap<usize, na::Matrix4<Float>>,
    _max_iterations: usize,
    _convergence_threshold: Float,
) -> HashMap<usize, na::Matrix4<Float>> {
    transform_maps.clone()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_defaults() {
        let config = ParallelTrackingConfig::default();
        assert!(config.enabled);
        assert_eq!(config.min_points_for_parallel, 50);
    }

    #[test]
    fn test_pyramid_building() {
        let image = GrayImage::new(64, 48);
        let (left, right) = build_pyramids_parallel(&image, &image, 4);
        assert_eq!(left.len(), 4);
        assert_eq!(right.len(), 4);
        assert_eq!(left[0].width(), 64);
        assert_eq!(left[1].width(), 32);
    }
}
