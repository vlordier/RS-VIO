/// Monocular patch tracker implementation

use image::GrayImage;
use nalgebra as na;
use std::collections::HashMap;

use crate::datasets::config::FeatureDetectionConfig;
use crate::feature_tracker::image_utilities;

use super::tracking::{add_points, track_points};

pub struct PatchTracker<const N: u32> {
    last_keypoint_id: usize,
    tracked_points_map: HashMap<usize, na::Affine2<f32>>,
    previous_image_pyramid: Vec<GrayImage>,
    grid_cols: u32,
}

impl<const LEVELS: u32> PatchTracker<LEVELS> {
    /// Construct tracker from `FeatureDetectionConfig` for centralized tuning.
    pub fn from_config(config: &FeatureDetectionConfig) -> Self {
        Self {
            last_keypoint_id: 0,
            tracked_points_map: HashMap::new(),
            previous_image_pyramid: Vec::new(),
            grid_cols: config.grid_cols,
        }
    }

    pub fn process_frame(&mut self, greyscale_image: &GrayImage) {
        // build current image pyramid
        let mut current_image_pyramid = Vec::new();
        image_utilities::ensure_pyramid_allocated(
            &mut current_image_pyramid,
            greyscale_image.width(),
            greyscale_image.height(),
            LEVELS as usize,
        );
        image_utilities::fill_pyramid(&mut current_image_pyramid, greyscale_image);

        if !self.previous_image_pyramid.is_empty() {
            log::info!("old points {}", self.tracked_points_map.len());
            // track prev points
            // Default values for PatchTracker (not used in estimator)
            let defaults = FeatureDetectionConfig::default();
            self.tracked_points_map = track_points::<LEVELS>(
                &self.previous_image_pyramid,
                &current_image_pyramid,
                &self.tracked_points_map,
                defaults.optical_flow_max_iterations as usize,
                defaults.optical_flow_convergence_threshold as f32,
            );
            log::info!("tracked old points {}", self.tracked_points_map.len());
        }
        // add new points
        let new_points = add_points(&self.tracked_points_map, greyscale_image, self.grid_cols);
        for (x, y, _score) in &new_points {
            let mut v = na::Affine2::<f32>::identity();

            v.matrix_mut_unchecked().m13 = *x;
            v.matrix_mut_unchecked().m23 = *y;
            self.tracked_points_map.insert(self.last_keypoint_id, v);
            self.last_keypoint_id += 1;
        }

        // update saved image pyramid
        self.previous_image_pyramid = current_image_pyramid;
    }

    pub fn get_track_points(&self) -> HashMap<usize, (f32, f32)> {
        self.tracked_points_map
            .iter()
            .map(|(k, v)| (*k, (v.matrix().m13, v.matrix().m23)))
            .collect()
    }

    pub fn remove_id(&mut self, ids: &[usize]) {
        for id in ids {
            self.tracked_points_map.remove(id);
        }
    }
}
