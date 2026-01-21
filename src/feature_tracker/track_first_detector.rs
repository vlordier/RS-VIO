//! Track-first, detect-to-fill feature management pattern
//!
//! This module implements the "SOTA twist" for classical features:
//! - Keep existing tracks alive as long as possible
//! - Only detect new features when spatial coverage drops below threshold
//! - Maintain per-feature uncertainty from image gradients and tracking residuals
//!
//! Benefits:
//! - Longer feature tracks = better BA conditioning
//! - Reduced computational cost (detection only when needed)
//! - Better spatial distribution of features

use image::GrayImage;
use nalgebra as na;
use std::collections::HashMap;

/// Configuration for track-first detector
#[derive(Debug, Clone)]
pub struct TrackFirstConfig {
    /// Minimum feature count to maintain
    pub min_features: usize,
    /// Maximum feature count (for performance)
    pub max_features: usize,
    /// Grid cell size for spatial distribution (pixels)
    pub grid_cell_size: u32,
    /// Minimum features per grid cell
    pub min_features_per_cell: usize,
    /// Quality threshold for corner detection
    pub corner_quality_threshold: f64,
    /// Minimum distance between features (pixels)
    pub min_feature_distance: f32,
}

impl Default for TrackFirstConfig {
    fn default() -> Self {
        Self {
            min_features: 150,
            max_features: 300,
            grid_cell_size: 32,
            min_features_per_cell: 2,
            corner_quality_threshold: 0.01,
            min_feature_distance: 20.0,
        }
    }
}

/// Feature with uncertainty tracking
#[derive(Debug, Clone)]
pub struct TrackedFeature {
    /// Feature ID (persistent across frames)
    pub id: usize,
    /// Current pixel coordinates
    pub position: na::Vector2<f32>,
    /// Feature age (number of frames tracked)
    pub age: usize,
    /// Tracking uncertainty from residuals
    pub uncertainty: f32,
    /// Image gradient magnitude at feature location
    pub gradient_magnitude: f32,
    /// Grid cell this feature belongs to
    pub grid_cell: (u32, u32),
}

/// Track-first feature detector and manager
pub struct TrackFirstDetector {
    config: TrackFirstConfig,
    /// Currently tracked features
    tracked_features: HashMap<usize, TrackedFeature>,
    /// Next feature ID to assign
    next_id: usize,
    /// Image dimensions for grid calculations
    image_width: u32,
    image_height: u32,
}

impl TrackFirstDetector {
    /// Create a new track-first detector
    pub fn new(config: TrackFirstConfig, image_width: u32, image_height: u32) -> Self {
        Self {
            config,
            tracked_features: HashMap::new(),
            next_id: 0,
            image_width,
            image_height,
        }
    }

    /// Update existing tracks and detect new features if needed
    ///
    /// # Arguments
    /// * `image` - Current grayscale image
    /// * `tracked_points` - Successfully tracked points from KLT
    /// * `tracking_residuals` - Optional tracking error for each point
    ///
    /// # Returns
    /// Complete set of features (existing + newly detected if needed)
    pub fn update_tracks(
        &mut self,
        image: &GrayImage,
        tracked_points: &[(usize, na::Vector2<f32>)],
        tracking_residuals: Option<&[f32]>,
    ) -> Vec<TrackedFeature> {
        // Update existing features with tracking results
        self.update_existing_features(tracked_points, tracking_residuals);

        // Compute image gradients for uncertainty estimation
        let gradients = self.compute_image_gradients(image);

        // Update gradient magnitudes for tracked features
        for feature in self.tracked_features.values_mut() {
            let x = feature.position.x as u32;
            let y = feature.position.y as u32;
            feature.gradient_magnitude = Self::get_gradient_at_static(
                &gradients,
                x,
                y,
                self.image_width,
                self.image_height,
            );
        }

        // Check if we need to detect new features
        if self.needs_new_features() {
            self.detect_and_fill(image, &gradients);
        }

        // Return all current features
        self.tracked_features.values().cloned().collect()
    }

    /// Update positions and ages of successfully tracked features
    fn update_existing_features(
        &mut self,
        tracked_points: &[(usize, na::Vector2<f32>)],
        residuals: Option<&[f32]>,
    ) {
        // Create set of successfully tracked IDs
        let tracked_ids: HashMap<usize, (na::Vector2<f32>, f32)> = tracked_points
            .iter()
            .enumerate()
            .map(|(idx, &(id, pos))| {
                let residual = residuals.map(|r| r[idx]).unwrap_or(0.0);
                (id, (pos, residual))
            })
            .collect();

        // Remove lost tracks and update successful ones
        self.tracked_features.retain(|id, feature| {
            if let Some((new_pos, residual)) = tracked_ids.get(id) {
                feature.position = *new_pos;
                feature.age += 1;
                // Update uncertainty based on tracking residual
                feature.uncertainty = feature.uncertainty * 0.9 + residual * 0.1;
                true
            } else {
                false // Remove lost track
            }
        });
    }

    /// Determine if new features need to be detected
    fn needs_new_features(&self) -> bool {
        // Check total feature count
        if self.tracked_features.len() < self.config.min_features {
            return true;
        }

        // Check spatial coverage (grid-based)
        let grid_coverage = self.compute_grid_coverage();
        let (grid_width, grid_height) = self.grid_dimensions();
        let total_cells = grid_width * grid_height;
        let filled_cells = grid_coverage.values().filter(|&&count| count > 0).count();

        // If less than 70% of cells have features, we need more
        (filled_cells as f32) / (total_cells as f32) < 0.7
    }

    /// Detect new features in under-covered regions
    fn detect_and_fill(&mut self, image: &GrayImage, gradients: &[f32]) {
        let grid_coverage = self.compute_grid_coverage();
        let (grid_width, grid_height) = self.grid_dimensions();

        // Find cells that need more features
        let mut cells_to_fill = Vec::new();
        for gy in 0..grid_height {
            for gx in 0..grid_width {
                let cell = (gx, gy);
                let count = grid_coverage.get(&cell).copied().unwrap_or(0);
                if count < self.config.min_features_per_cell {
                    cells_to_fill.push(cell);
                }
            }
        }

        // Detect features in each under-covered cell
        for (gx, gy) in cells_to_fill {
            if self.tracked_features.len() >= self.config.max_features {
                break;
            }

            self.detect_in_cell(image, gradients, gx, gy);
        }
    }

    /// Detect features within a specific grid cell using Shi-Tomasi (GFTT)
    fn detect_in_cell(&mut self, _image: &GrayImage, gradients: &[f32], gx: u32, gy: u32) {
        let cell_x = gx * self.config.grid_cell_size;
        let cell_y = gy * self.config.grid_cell_size;
        let cell_w = self.config.grid_cell_size.min(self.image_width - cell_x);
        let cell_h = self.config.grid_cell_size.min(self.image_height - cell_y);

        // Find best corner in cell based on gradient magnitude
        let mut best_response = 0.0f32;
        let mut best_pos = None;

        for y in cell_y..(cell_y + cell_h) {
            for x in cell_x..(cell_x + cell_w) {
                let grad_mag = self.get_gradient_at(gradients, x, y);

                // Check if this is better than current best and far from existing features
                if grad_mag > best_response && grad_mag > self.config.corner_quality_threshold as f32
                {
                    let pos = na::Vector2::new(x as f32, y as f32);
                    if self.is_far_from_existing_features(&pos) {
                        best_response = grad_mag;
                        best_pos = Some(pos);
                    }
                }
            }
        }

        // Add new feature if found
        if let Some(position) = best_pos {
            let feature = TrackedFeature {
                id: self.next_id,
                position,
                age: 0,
                uncertainty: 0.0,
                gradient_magnitude: best_response,
                grid_cell: (gx, gy),
            };
            self.tracked_features.insert(self.next_id, feature);
            self.next_id += 1;
        }
    }

    /// Check if a position is far enough from existing features
    fn is_far_from_existing_features(&self, pos: &na::Vector2<f32>) -> bool {
        let min_dist_sq = self.config.min_feature_distance * self.config.min_feature_distance;

        for feature in self.tracked_features.values() {
            let dist_sq = (feature.position - pos).norm_squared();
            if dist_sq < min_dist_sq {
                return false;
            }
        }
        true
    }

    /// Compute grid coverage (features per cell)
    fn compute_grid_coverage(&self) -> HashMap<(u32, u32), usize> {
        let mut coverage = HashMap::new();

        for feature in self.tracked_features.values() {
            *coverage.entry(feature.grid_cell).or_insert(0) += 1;
        }

        coverage
    }

    /// Get grid dimensions
    fn grid_dimensions(&self) -> (u32, u32) {
        let grid_width = (self.image_width + self.config.grid_cell_size - 1)
            / self.config.grid_cell_size;
        let grid_height = (self.image_height + self.config.grid_cell_size - 1)
            / self.config.grid_cell_size;
        (grid_width, grid_height)
    }

    /// Compute image gradients using Sobel operator
    fn compute_image_gradients(&self, image: &GrayImage) -> Vec<f32> {
        let width = image.width();
        let height = image.height();
        let mut gradients = vec![0.0f32; (width * height) as usize];

        for y in 1..(height - 1) {
            for x in 1..(width - 1) {
                // Sobel kernels
                let gx = (-1.0 * image.get_pixel(x - 1, y - 1).0[0] as f32)
                    + (-2.0 * image.get_pixel(x - 1, y).0[0] as f32)
                    + (-1.0 * image.get_pixel(x - 1, y + 1).0[0] as f32)
                    + (1.0 * image.get_pixel(x + 1, y - 1).0[0] as f32)
                    + (2.0 * image.get_pixel(x + 1, y).0[0] as f32)
                    + (1.0 * image.get_pixel(x + 1, y + 1).0[0] as f32);

                let gy = (-1.0 * image.get_pixel(x - 1, y - 1).0[0] as f32)
                    + (-2.0 * image.get_pixel(x, y - 1).0[0] as f32)
                    + (-1.0 * image.get_pixel(x + 1, y - 1).0[0] as f32)
                    + (1.0 * image.get_pixel(x - 1, y + 1).0[0] as f32)
                    + (2.0 * image.get_pixel(x, y + 1).0[0] as f32)
                    + (1.0 * image.get_pixel(x + 1, y + 1).0[0] as f32);

                let magnitude = (gx * gx + gy * gy).sqrt();
                gradients[(y * width + x) as usize] = magnitude;
            }
        }

        gradients
    }

    /// Get gradient magnitude at a specific position
    fn get_gradient_at(&self, gradients: &[f32], x: u32, y: u32) -> f32 {
        Self::get_gradient_at_static(gradients, x, y, self.image_width, self.image_height)
    }

    /// Static version of get_gradient_at
    fn get_gradient_at_static(gradients: &[f32], x: u32, y: u32, width: u32, height: u32) -> f32 {
        if x >= width || y >= height {
            return 0.0;
        }
        gradients[(y * width + x) as usize]
    }

    /// Get current feature count
    pub fn feature_count(&self) -> usize {
        self.tracked_features.len()
    }

    /// Reset all tracks (useful on keyframe)
    pub fn reset(&mut self) {
        self.tracked_features.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_track_first_creation() {
        let config = TrackFirstConfig::default();
        let detector = TrackFirstDetector::new(config, 640, 480);
        assert_eq!(detector.feature_count(), 0);
    }

    #[test]
    fn test_needs_new_features() {
        let config = TrackFirstConfig {
            min_features: 10,
            ..Default::default()
        };
        let detector = TrackFirstDetector::new(config, 640, 480);

        // Should need features when empty
        assert!(detector.needs_new_features());
    }

    #[test]
    fn test_grid_dimensions() {
        let config = TrackFirstConfig {
            grid_cell_size: 32,
            ..Default::default()
        };
        let detector = TrackFirstDetector::new(config, 640, 480);
        let (gw, gh) = detector.grid_dimensions();

        assert_eq!(gw, 20); // 640 / 32 = 20
        assert_eq!(gh, 15); // 480 / 32 = 15
    }

    #[test]
    fn test_feature_distance_check() {
        let config = TrackFirstConfig {
            min_feature_distance: 20.0,
            ..Default::default()
        };
        let mut detector = TrackFirstDetector::new(config, 640, 480);

        // Add a feature
        let feature = TrackedFeature {
            id: 0,
            position: na::Vector2::new(100.0, 100.0),
            age: 0,
            uncertainty: 0.0,
            gradient_magnitude: 0.0,
            grid_cell: (0, 0),
        };
        detector.tracked_features.insert(0, feature);

        // Check nearby point (should be too close)
        let nearby = na::Vector2::new(105.0, 105.0);
        assert!(!detector.is_far_from_existing_features(&nearby));

        // Check far point (should be OK)
        let far = na::Vector2::new(150.0, 150.0);
        assert!(detector.is_far_from_existing_features(&far));
    }
}
