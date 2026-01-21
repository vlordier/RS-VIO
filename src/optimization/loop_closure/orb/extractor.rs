//! ORB feature extractor main implementation

use super::descriptor::extract_brief;
use super::detector::{compute_orientation, fast_corner_score};
use super::types::{OrbConfig, OrbFeature};
use crate::debug_log;
use crate::optimization::loop_closure::descriptor_pool::OrbBinaryPool;
use na::Vector2;
use nalgebra as na;
use std::sync::Arc;

/// ORB descriptor extractor
pub struct OrbExtractor {
    config: OrbConfig,
}

impl OrbExtractor {
    /// Create a new ORB extractor with default configuration
    pub fn new(config: OrbConfig) -> Self {
        Self { config }
    }

    /// Extract ORB descriptors from a grayscale image
    ///
    /// # Arguments
    /// * `image` - Grayscale image (row-major, u8 values)
    /// * `width` - Image width in pixels
    /// * `height` - Image height in pixels
    ///
    /// # Returns
    /// Vector of ORB features (up to `config.num_features`)
    pub fn extract(&self, image: &[u8], width: u32, height: u32) -> Vec<OrbFeature> {
        // Validate input
        if image.len() != (width * height) as usize {
            log::warn!(
                "[OrbExtractor] Image size mismatch: expected {}, got {}",
                (width * height) as usize,
                image.len()
            );
            return Vec::new();
        }

        if width < 32 || height < 32 {
            log::warn!("[OrbExtractor] Image too small: {}x{}", width, height);
            return Vec::new();
        }

        let mut features = if self.config.use_pyramid {
            // Multi-scale extraction
            self.extract_pyramid(image, width, height)
        } else {
            // Single-scale extraction
            self.extract_single_scale(image, width, height, 0)
        };

        // Sort by strength and keep top N (using total_cmp for NaN safety)
        features.sort_by(|a, b| b.strength.total_cmp(&a.strength));
        features.truncate(self.config.num_features);

        debug_log!("[OrbExtractor] Extracted {} ORB features", features.len());
        features
    }

    /// Extract ORB features from a single scale (no pyramid)
    fn extract_single_scale(
        &self,
        image: &[u8],
        width: u32,
        height: u32,
        level: usize,
    ) -> Vec<OrbFeature> {
        let mut features = Vec::new();
        let border = 16;

        // Grid-based feature extraction for uniform distribution
        let grid_size = 32;
        let grid_width = ((width as usize - 2 * border) / grid_size).max(1);
        let grid_height = ((height as usize - 2 * border) / grid_size).max(1);

        for gy in 0..grid_height {
            for gx in 0..grid_width {
                let x_start = border + gx * grid_size;
                let y_start = border + gy * grid_size;
                let x_end = (x_start + grid_size).min(width as usize - border);
                let y_end = (y_start + grid_size).min(height as usize - border);

                // Find strongest FAST corner in this grid cell
                let mut best_feature: Option<OrbFeature> = None;

                for y in y_start..y_end {
                    for x in x_start..x_end {
                        if let Some(strength) =
                            fast_corner_score(image, x, y, width as usize, &self.config)
                        {
                            // FAST score is the count of matching neighbors (0-8)
                            // If fast_corner_score returns Some, it already verified >= 3 consecutive
                            // So any positive score is a valid corner
                            if strength >= 3.0 {
                                // Valid FAST corner
                                let orientation =
                                    compute_orientation(image, x, y, width as usize, &self.config);
                                let descriptor =
                                    extract_brief(image, x, y, width as usize, orientation, None);

                                let feature = OrbFeature {
                                    position: Vector2::new(x as f64, y as f64),
                                    orientation,
                                    descriptor,
                                    level,
                                    strength,
                                };

                                best_feature = match best_feature {
                                    None => Some(feature),
                                    Some(best) => {
                                        if feature.strength > best.strength {
                                            Some(feature)
                                        } else {
                                            Some(best)
                                        }
                                    },
                                };
                            }
                        }
                    }
                }

                if let Some(f) = best_feature {
                    features.push(f);
                }
            }
        }

        features
    }

    /// Extract ORB features from image pyramid
    fn extract_pyramid(&self, image: &[u8], width: u32, height: u32) -> Vec<OrbFeature> {
        let mut all_features = Vec::new();
        let mut current_image = image.to_vec();
        let mut current_width = width;
        let mut current_height = height;

        for level in 0..self.config.num_levels {
            let features =
                self.extract_single_scale(&current_image, current_width, current_height, level);
            all_features.extend(features);

            // Downsample for next level
            if level < self.config.num_levels - 1 {
                let next_width = (current_width as f64 / self.config.scale_factor) as u32;
                let next_height = (current_height as f64 / self.config.scale_factor) as u32;

                if next_width < 32 || next_height < 32 {
                    break;
                }

                current_image = self.downsample(
                    &current_image,
                    current_width as usize,
                    current_height as usize,
                    next_width as usize,
                    next_height as usize,
                );
                current_width = next_width;
                current_height = next_height;
            }
        }

        all_features
    }

    /// Simple box filter downsampling
    fn downsample(&self, image: &[u8], w: usize, h: usize, nw: usize, nh: usize) -> Vec<u8> {
        let mut downsampled = vec![0u8; nw * nh];
        let scale_x = w as f64 / nw as f64;
        let scale_y = h as f64 / nh as f64;

        for y in 0..nh {
            for x in 0..nw {
                let src_x = (x as f64 * scale_x) as usize;
                let src_y = (y as f64 * scale_y) as usize;

                let val = image.get(src_y * w + src_x).copied().unwrap_or(0);
                downsampled[y * nw + x] = val;
            }
        }

        downsampled
    }

    /// Extract ORB descriptors with optional buffer pool for binary descriptors
    ///
    /// # Arguments
    /// * `image` - Grayscale image (row-major, u8 values)
    /// * `width` - Image width in pixels
    /// * `height` - Image height in pixels
    /// * `binary_pool` - Optional pre-allocated binary descriptor pool
    ///
    /// # Returns
    /// Vector of ORB features (up to `config.num_features`)
    ///
    /// When a pool is provided, binary descriptor allocations are reused from the pool,
    /// eliminating per-feature allocation overhead. OrbFeature descriptors are now
    /// Vec<u8>, enabling true buffer pooling and reuse across extraction calls.
    pub fn extract_with_pool(
        &self,
        image: &[u8],
        width: u32,
        height: u32,
        binary_pool: Option<&Arc<OrbBinaryPool>>,
    ) -> Vec<OrbFeature> {
        // Validate input
        if image.len() != (width * height) as usize {
            log::warn!(
                "[OrbExtractor] Image size mismatch: expected {}, got {}",
                (width * height) as usize,
                image.len()
            );
            return Vec::new();
        }

        if width < 32 || height < 32 {
            log::warn!("[OrbExtractor] Image too small: {}x{}", width, height);
            return Vec::new();
        }

        let mut features = if self.config.use_pyramid {
            // Multi-scale extraction with pooling
            self.extract_pyramid_with_pool(image, width, height, binary_pool)
        } else {
            // Single-scale extraction with pooling
            self.extract_single_scale_with_pool(image, width, height, 0, binary_pool)
        };

        // Sort by strength and keep top N (using total_cmp for NaN safety)
        features.sort_by(|a, b| b.strength.total_cmp(&a.strength));
        features.truncate(self.config.num_features);

        debug_log!(
            "[OrbExtractor] Extracted {} ORB features with pooling",
            features.len()
        );
        features
    }

    /// Extract ORB features from a single scale with optional pooling
    fn extract_single_scale_with_pool(
        &self,
        image: &[u8],
        width: u32,
        height: u32,
        level: usize,
        pool: Option<&Arc<OrbBinaryPool>>,
    ) -> Vec<OrbFeature> {
        let mut features = Vec::new();
        let border = 16;

        // Grid-based feature extraction for uniform distribution
        let grid_size = 32;
        let grid_width = ((width as usize - 2 * border) / grid_size).max(1);
        let grid_height = ((height as usize - 2 * border) / grid_size).max(1);

        for gy in 0..grid_height {
            for gx in 0..grid_width {
                let x_start = border + gx * grid_size;
                let y_start = border + gy * grid_size;
                let x_end = (x_start + grid_size).min(width as usize - border);
                let y_end = (y_start + grid_size).min(height as usize - border);

                // Find strongest FAST corner in this grid cell
                let mut best_feature: Option<OrbFeature> = None;

                for y in y_start..y_end {
                    for x in x_start..x_end {
                        if let Some(strength) =
                            fast_corner_score(image, x, y, width as usize, &self.config)
                        {
                            // FAST score is the count of matching neighbors (0-8)
                            // If fast_corner_score returns Some, it already verified >= 3 consecutive
                            // So any positive score is a valid corner
                            if strength >= 3.0 {
                                // Valid FAST corner
                                let orientation =
                                    compute_orientation(image, x, y, width as usize, &self.config);
                                let descriptor =
                                    extract_brief(image, x, y, width as usize, orientation, pool);

                                let feature = OrbFeature {
                                    position: Vector2::new(x as f64, y as f64),
                                    orientation,
                                    descriptor,
                                    level,
                                    strength,
                                };

                                best_feature = match best_feature {
                                    None => Some(feature),
                                    Some(best) => {
                                        if feature.strength > best.strength {
                                            Some(feature)
                                        } else {
                                            Some(best)
                                        }
                                    },
                                };
                            }
                        }
                    }
                }

                if let Some(f) = best_feature {
                    features.push(f);
                }
            }
        }

        features
    }

    /// Extract ORB features from image pyramid with optional pooling
    fn extract_pyramid_with_pool(
        &self,
        image: &[u8],
        width: u32,
        height: u32,
        pool: Option<&Arc<OrbBinaryPool>>,
    ) -> Vec<OrbFeature> {
        let mut all_features = Vec::new();
        let mut current_image = image.to_vec();
        let mut current_width = width;
        let mut current_height = height;

        for level in 0..self.config.num_levels {
            let features = self.extract_single_scale_with_pool(
                &current_image,
                current_width,
                current_height,
                level,
                pool,
            );
            all_features.extend(features);

            // Downsample for next level
            if level < self.config.num_levels - 1 {
                let next_width = (current_width as f64 / self.config.scale_factor) as u32;
                let next_height = (current_height as f64 / self.config.scale_factor) as u32;

                if next_width < 32 || next_height < 32 {
                    break;
                }

                current_image = self.downsample(
                    &current_image,
                    current_width as usize,
                    current_height as usize,
                    next_width as usize,
                    next_height as usize,
                );
                current_width = next_width;
                current_height = next_height;
            }
        }

        all_features
    }
}
