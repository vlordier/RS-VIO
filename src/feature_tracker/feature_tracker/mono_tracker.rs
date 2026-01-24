/// Monocular patch tracker implementation
use image::GrayImage;
use nalgebra as na;
use std::collections::HashMap;

use crate::common::FeatureTrackingArena;
use crate::datasets::config::FeatureDetectionConfig;
use crate::feature_tracker::image_utilities;

use super::tracking::{add_points, track_points};

pub struct PatchTracker<const N: u32> {
    last_keypoint_id: usize,
    tracked_points_map: HashMap<usize, na::Affine2<f32>>,
    previous_image_pyramid: Vec<GrayImage>,
    grid_cols: u32,
    max_features_per_grid: u32,
}

impl<const LEVELS: u32> PatchTracker<LEVELS> {
    /// Construct tracker from `FeatureDetectionConfig` for centralized tuning.
    pub fn from_config(config: &FeatureDetectionConfig) -> Self {
        Self {
            last_keypoint_id: 0,
            tracked_points_map: HashMap::new(),
            previous_image_pyramid: Vec::new(),
            grid_cols: config.grid_cols,
            max_features_per_grid: config.max_features_per_grid,
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
        let new_points = add_points(
            &self.tracked_points_map,
            greyscale_image,
            self.grid_cols,
            self.max_features_per_grid,
        );
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

/// Arena-backed monocular tracker for zero-copy allocation
///
/// This implementation optimizes allocation patterns for feature tracking,
/// preparing for integration into high-frequency loops.
/// Currently alongside PatchTracker for gradual migration.
#[allow(dead_code)]
pub struct ArenaPatchTracker<const N: u32> {
    last_keypoint_id: usize,
    /// Maps feature ID to (point, velocity, age, confidence)
    /// Using tuple instead of ArenaTrack to avoid lifetime issues
    tracked_points: HashMap<usize, ([f32; 2], Option<[f32; 2]>, u32, f32)>,
    previous_image_pyramid: Vec<GrayImage>,
    grid_cols: u32,
    max_features_per_grid: u32,
    /// Arena allocator for feature track data (for future optimization)
    _arena: FeatureTrackingArena,
}

#[allow(dead_code)]
impl<const LEVELS: u32> ArenaPatchTracker<LEVELS> {
    /// Create new arena-backed tracker with capacity hint
    pub fn from_config(config: &FeatureDetectionConfig) -> Self {
        // Pre-allocate arena for typical frame size
        let estimated_features = (config.grid_cols * config.max_features_per_grid) as usize;

        Self {
            last_keypoint_id: 0,
            tracked_points: HashMap::new(),
            previous_image_pyramid: Vec::new(),
            grid_cols: config.grid_cols,
            max_features_per_grid: config.max_features_per_grid,
            _arena: FeatureTrackingArena::new(estimated_features),
        }
    }

    /// Process frame and track features with optimized allocation
    pub fn process_frame(&mut self, greyscale_image: &GrayImage) {
        // Build current image pyramid
        let mut current_image_pyramid = Vec::new();
        image_utilities::ensure_pyramid_allocated(
            &mut current_image_pyramid,
            greyscale_image.width(),
            greyscale_image.height(),
            LEVELS as usize,
        );
        image_utilities::fill_pyramid(&mut current_image_pyramid, greyscale_image);

        if !self.previous_image_pyramid.is_empty() {
            log::info!("old points {}", self.tracked_points.len());

            // Convert tracked points to Affine2 for tracking algorithm
            let mut affine_map = HashMap::new();
            for (id, (point, _, _, _)) in &self.tracked_points {
                let mut v = na::Affine2::<f32>::identity();
                v.matrix_mut_unchecked().m13 = point[0];
                v.matrix_mut_unchecked().m23 = point[1];
                affine_map.insert(*id, v);
            }

            // Track points using standard algorithm
            let defaults = FeatureDetectionConfig::default();
            let tracked = track_points::<LEVELS>(
                &self.previous_image_pyramid,
                &current_image_pyramid,
                &affine_map,
                defaults.optical_flow_max_iterations as usize,
                defaults.optical_flow_convergence_threshold as f32,
            );

            // Update tracked points with new positions and velocities
            let mut updated = HashMap::new();
            for (id, new_affine) in tracked {
                let new_point = [new_affine.matrix().m13, new_affine.matrix().m23];

                if let Some((old_point, _, old_age, old_conf)) = self.tracked_points.get(&id) {
                    // Calculate velocity from old to new position
                    let velocity = Some([new_point[0] - old_point[0], new_point[1] - old_point[1]]);
                    // Increment age and maintain confidence
                    updated.insert(id, (new_point, velocity, old_age + 1, *old_conf));
                } else {
                    // New track
                    updated.insert(id, (new_point, None, 1, 1.0));
                }
            }
            self.tracked_points = updated;
            log::info!("tracked old points {}", self.tracked_points.len());
        }

        // Add new points
        let mut affine_map = HashMap::new();
        for (id, (point, _, _, _)) in &self.tracked_points {
            let mut v = na::Affine2::<f32>::identity();
            v.matrix_mut_unchecked().m13 = point[0];
            v.matrix_mut_unchecked().m23 = point[1];
            affine_map.insert(*id, v);
        }

        let new_points = add_points(
            &affine_map,
            greyscale_image,
            self.grid_cols,
            self.max_features_per_grid,
        );
        for (x, y, _score) in &new_points {
            self.tracked_points
                .insert(self.last_keypoint_id, ([*x, *y], None, 1, 0.5));
            self.last_keypoint_id += 1;
        }

        // Update saved image pyramid
        self.previous_image_pyramid = current_image_pyramid;
    }

    /// Get tracked point coordinates
    pub fn get_track_points(&self) -> HashMap<usize, (f32, f32)> {
        self.tracked_points
            .iter()
            .map(|(k, (point, _, _, _))| (*k, (point[0], point[1])))
            .collect()
    }

    /// Remove tracked features by ID
    pub fn remove_id(&mut self, ids: &[usize]) {
        for id in ids {
            self.tracked_points.remove(id);
        }
    }

    /// Get allocation statistics
    pub fn allocation_stats(&self) -> crate::common::arena::ArenaStats {
        self._arena.stats()
    }
}
