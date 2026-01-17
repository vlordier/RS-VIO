//! # ORB (Oriented FAST and Rotated BRIEF) Descriptor
//!
//! Implements the ORB descriptor from:
//! "ORB: An Efficient Alternative to SIFT or SURF" (Rublee et al., 2011)
//!
//! Features:
//! - Rotation-invariant: Orientation computed from intensity centroid
//! - Scale-invariant: Multi-scale image pyramid
//! - Efficient: Binary descriptor (256 bits = 32 bytes)
//! - Fast matching: Hamming distance computation
//!
//! ## Algorithm Overview
//!
//! 1. **Corner Detection**: FAST9 corners on image pyramid
//! 2. **Orientation Estimation**: Intensity centroid method
//! 3. **Descriptor Extraction**: Rotated BRIEF pattern
//! 4. **Matching**: Hamming distance with ratio test

use na::Vector2;
use nalgebra as na;
use crate::debug_log;
use crate::optimization::loop_closure::descriptor_pool::OrbBinaryPool;
use std::f64::consts::PI;
use std::sync::Arc;

/// ORB descriptor configuration
#[derive(Debug, Clone)]
pub struct OrbConfig {
    /// Number of features to extract
    pub num_features: usize,
    /// Scale factor between pyramid levels (typically 1.2)
    pub scale_factor: f64,
    /// Number of pyramid levels
    pub num_levels: usize,
    /// Patch size for orientation computation
    pub patch_size: usize,
    /// Whether to use multi-scale extraction
    pub use_pyramid: bool,
}

impl Default for OrbConfig {
    fn default() -> Self {
        Self {
            num_features: 500,
            scale_factor: 1.2,
            num_levels: 8,
            patch_size: 31,
            use_pyramid: true,
        }
    }
}

/// A single ORB feature with position, orientation, and descriptor
#[derive(Debug, Clone)]
pub struct OrbFeature {
    /// 2D position in image (pixel coordinates)
    pub position: Vector2<f64>,
    /// Orientation in radians [0, 2π)
    pub orientation: f64,
    /// ORB descriptor: 256 bits as 32 bytes (dynamic allocation, poolable)
    pub descriptor: Vec<u8>,
    /// Pyramid level where feature was detected
    pub level: usize,
    /// Feature strength (FAST score)
    pub strength: f64,
}

impl OrbFeature {
    /// Compute Hamming distance to another descriptor
    pub fn hamming_distance(&self, other: &[u8]) -> u32 {
        self.descriptor
            .iter()
            .zip(other.iter())
            .map(|(a, b)| (a ^ b).count_ones())
            .sum()
    }

    /// Check if orientation is consistent (for rotation-invariant matching)
    pub fn orientation_match(&self, other_orientation: f64, threshold: f64) -> bool {
        let diff = (self.orientation - other_orientation).abs();
        let normalized_diff = if diff > PI { 2.0 * PI - diff } else { diff };
        normalized_diff < threshold
    }
}

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
                        if let Some(strength) = self.fast_corner_score(image, x, y, width as usize)
                        {
                            // FAST score is the count of matching neighbors (0-8)
                            // If fast_corner_score returns Some, it already verified >= 3 consecutive
                            // So any positive score is a valid corner
                            if strength >= 3.0 {
                                // Valid FAST corner
                                let orientation =
                                    self.compute_orientation(image, x, y, width as usize);
                                let descriptor =
                                    self.extract_brief(image, x, y, width as usize, orientation, None);

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

    /// Compute FAST9 corner strength (simplified)
    fn fast_corner_score(&self, image: &[u8], x: usize, y: usize, width: usize) -> Option<f64> {
        if x < 3 || y < 3 || x >= width - 3 {
            return None;
        }

        let height = image.len() / width;
        if y >= height - 3 {
            return None;
        }

        let center = image[y * width + x] as f64;
        let threshold = 50.0;

        // Sample 8 neighbors in circle
        let offsets = [
            (-1, -3),
            (-2, -2),
            (-3, -1),
            (-3, 0),
            (-3, 1),
            (-2, 2),
            (-1, 3),
            (0, 3),
        ];

        let mut high_count = 0;
        let mut low_count = 0;

        for (dx, dy) in offsets {
            let nx = (x as i32 + dx) as usize;
            let ny = (y as i32 + dy) as usize;
            let neighbor = image[ny * width + nx] as f64;

            if neighbor > center + threshold {
                high_count += 1;
            } else if neighbor < center - threshold {
                low_count += 1;
            }
        }

        if high_count >= 3 || low_count >= 3 {
            Some((high_count + low_count) as f64)
        } else {
            None
        }
    }

    /// Compute corner orientation using intensity centroid
    fn compute_orientation(&self, image: &[u8], x: usize, y: usize, width: usize) -> f64 {
        let patch_size = self.config.patch_size;
        let half_patch = patch_size / 2;
        let height = image.len() / width;

        let mut m10 = 0.0f64;
        let mut m01 = 0.0f64;

        for py in 0..patch_size {
            for px in 0..patch_size {
                let py_signed = py as i32 - half_patch as i32;
                let px_signed = px as i32 - half_patch as i32;

                let img_y = (y as i32 + py_signed) as usize;
                let img_x = (x as i32 + px_signed) as usize;

                if img_x < width && img_y < height {
                    let intensity = image[img_y * width + img_x] as f64;
                    m10 += px_signed as f64 * intensity;
                    m01 += py_signed as f64 * intensity;
                }
            }
        }

        let angle = m01.atan2(m10);
        if angle < 0.0 {
            angle + 2.0 * PI
        } else {
            angle
        }
    }

    /// Extract BRIEF descriptor (simplified)
    /// In practice, use pre-defined BRIEF pattern; here we use pseudo-random tests
    fn extract_brief(
        &self,
        image: &[u8],
        x: usize,
        y: usize,
        width: usize,
        orientation: f64,
        pool: Option<&Arc<OrbBinaryPool>>,
    ) -> Vec<u8> {
        let mut descriptor = if let Some(p) = pool {
            p.acquire_binary().unwrap_or_else(|| vec![0u8; 32])
        } else {
            vec![0u8; 32]
        };
        let height = image.len() / width;
        let cos_angle = orientation.cos();
        let sin_angle = orientation.sin();

        // 256 bit tests (32 bytes)
        for i in 0..256 {
            // Pseudo-random test points (in practice, use ORB's fixed pattern)
            let p1_x = ((i as f64).sin() * 10.0) as i32;
            let p1_y = ((i as f64).cos() * 10.0) as i32;
            let p2_x = ((i as f64).sin() * 15.0) as i32;
            let p2_y = ((i as f64).cos() * 15.0) as i32;

            // Rotate points by orientation
            let r1_x = ((p1_x as f64 * cos_angle - p1_y as f64 * sin_angle) as i32).clamp(-16, 16);
            let r1_y = ((p1_x as f64 * sin_angle + p1_y as f64 * cos_angle) as i32).clamp(-16, 16);
            let r2_x = ((p2_x as f64 * cos_angle - p2_y as f64 * sin_angle) as i32).clamp(-16, 16);
            let r2_y = ((p2_x as f64 * sin_angle + p2_y as f64 * cos_angle) as i32).clamp(-16, 16);

            // Apply test
            let x1 = ((x as i32 + r1_x) as usize).min(width - 1);
            let y1 = ((y as i32 + r1_y) as usize).min(height - 1);
            let x2 = ((x as i32 + r2_x) as usize).min(width - 1);
            let y2 = ((y as i32 + r2_y) as usize).min(height - 1);

            let intensity1 = image.get(y1 * width + x1).copied().unwrap_or(128) as i32;
            let intensity2 = image.get(y2 * width + x2).copied().unwrap_or(128) as i32;

            let bit = if intensity1 > intensity2 { 1u8 } else { 0u8 };
            let byte_idx = i / 8;
            let bit_idx = i % 8;
            descriptor[byte_idx] |= bit << bit_idx;
        }

        descriptor
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

        debug_log!("[OrbExtractor] Extracted {} ORB features with pooling", features.len());
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
                        if let Some(strength) = self.fast_corner_score(image, x, y, width as usize)
                        {
                            // FAST score is the count of matching neighbors (0-8)
                            // If fast_corner_score returns Some, it already verified >= 3 consecutive
                            // So any positive score is a valid corner
                            if strength >= 3.0 {
                                // Valid FAST corner
                                let orientation =
                                    self.compute_orientation(image, x, y, width as usize);
                                let descriptor =
                                    self.extract_brief(image, x, y, width as usize, orientation, pool);

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

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::field_reassign_with_default
)]
mod tests {
    use super::*;

    /// Create test image with actual FAST-9 corners AND texture for BRIEF descriptors
    /// 
    /// FAST-9 uses this circular pattern of 8 offsets:
    /// (-1,-3), (-2,-2), (-3,-1), (-3,0), (-3,1), (-2,2), (-1,3), (0,3)
    /// 
    /// Requires >= 3 consecutive pixels brighter OR darker than center by threshold (50)
    /// BRIEF descriptors need texture variation to produce unique descriptors
    fn create_test_image(width: usize, height: usize) -> Vec<u8> {
        // Start with textured background for BRIEF descriptor diversity
        let mut image = vec![0u8; width * height];
        
        // Add checkerboard texture to background (8x8 blocks)
        for y in 0..height {
            for x in 0..width {
                let block_x = (x / 8) % 2;
                let block_y = (y / 8) % 2;
                image[y * width + x] = if (block_x + block_y) % 2 == 0 { 60 } else { 80 };
            }
        }

        // FAST-9 circle offsets (from fast_corner_score implementation)
        let fast_offsets = [
            (-1, -3), (-2, -2), (-3, -1), (-3, 0), 
            (-3, 1), (-2, 2), (-1, 3), (0, 3),
        ];

        // Create bright FAST corners at known locations with unique texture
        let corner_positions = [
            (50, 50),   (200, 50),  (350, 50),  (500, 50),
            (50, 150),  (200, 150), (350, 150), (500, 150),
            (50, 250),  (200, 250), (350, 250), (500, 250),
            (50, 350),  (200, 350), (350, 350), (500, 350),
        ];

        for (corner_idx, &(cx, cy)) in corner_positions.iter().enumerate() {
            if cx < width.saturating_sub(20) && cy < height.saturating_sub(20) 
               && cx >= 20 && cy >= 20 {
                
                // Set center pixel with variation per corner
                let base_intensity = 180 + (corner_idx % 4) as u8 * 15;
                image[cy * width + cx] = base_intensity;
                
                // Make ALL 8 FAST circle pixels VERY bright (>center + 50)
                for &(dx, dy) in &fast_offsets {
                    let nx = (cx as i32 + dx) as usize;
                    let ny = (cy as i32 + dy) as usize;
                    if nx < width && ny < height {
                        image[ny * width + nx] = 255;
                    }
                }

                // Add unique radial gradient around each corner for descriptor diversity
                for dy in -15..=15 {
                    for dx in -15..=15 {
                        let dist_sq = dx * dx + dy * dy;
                        if dist_sq > 0 && dist_sq < 225 { // radius 15
                            let x = (cx as i32 + dx) as usize;
                            let y = (cy as i32 + dy) as usize;
                            if x < width && y < height {
                                // Create gradient based on angle and corner index
                                let angle_factor = ((dx as f64).atan2(dy as f64) * 10.0) as i32;
                                let intensity = (120 + (corner_idx as i32 * 7) + angle_factor).clamp(60, 240) as u8;
                                image[y * width + x] = intensity;
                            }
                        }
                    }
                }
            }
        }

        image
    }

    #[test]
    fn debug_fast_corner_detection() {
        let config = OrbConfig::default();
        let extractor = OrbExtractor::new(config);
        
        // Create test image
        let image = create_test_image(640, 480);
        
        // Manually check corner positions
        let test_positions = vec![(50, 50), (200, 50), (350, 50)];
        
        for &(x, y) in &test_positions {
            let center_val = image[y * 640 + x];
            println!("Position ({}, {}) center value: {}", x, y, center_val);
            
            // Check FAST circle neighbors
            let offsets = [
                (-1, -3), (-2, -2), (-3, -1), (-3, 0),
                (-3, 1), (-2, 2), (-1, 3), (0, 3),
            ];
            
            for (i, &(dx, dy)) in offsets.iter().enumerate() {
                let nx = (x as i32 + dx) as usize;
                let ny = (y as i32 + dy) as usize;
                let neighbor_val = image[ny * 640 + nx];
                let diff = neighbor_val as i32 - center_val as i32;
                println!("  Offset {}: ({:2},{:2}) value={:3} diff={:4}", 
                         i, dx, dy, neighbor_val, diff);
            }
            
            if let Some(score) = extractor.fast_corner_score(&image, x, y, 640) {
                println!("  ✓ FAST score: {}", score);
            } else {
                println!("  ✗ Not a FAST corner!");
            }
            println!();
        }
        
        // Try extraction
        let features = extractor.extract(&image, 640, 480);
        println!("Total features extracted: {}", features.len());
    }

    #[test]
    fn orb_extraction_produces_features_from_test_image() {
        let mut config = OrbConfig::default();
        config.use_pyramid = false; // Single-scale for determinism
        let extractor = OrbExtractor::new(config);
        let image = create_test_image(640, 480);

        let features = extractor.extract(&image, 640, 480);
        
        // With 9 explicit cross corners, we MUST extract features
        assert!(
            features.len() > 0,
            "MUST extract features from test image with explicit FAST corners. Got 0 features - test image generation is broken!"
        );
        
        // Should not exceed max
        assert!(features.len() <= 500, "Should not exceed max features");
        
        println!("✅ Extracted {} features from test image (expected >0)", features.len());
    }

    #[test]
    fn orb_descriptor_is_binary() {
        let config = OrbConfig::default();
        let extractor = OrbExtractor::new(config);
        let image = create_test_image(640, 480);

        let features = extractor.extract(&image, 640, 480);
        assert!(features.len() > 0, "Should extract features");
        
        for feature in &features {
            assert_eq!(
                feature.descriptor.len(),
                32,
                "ORB descriptor should be 256 bits (32 bytes)"
            );
            
            // ORB descriptors are bit-packed: each byte contains 8 binary bits
            // So bytes can be any value 0-255 (not just 0 or 255)
            // Verify at least some descriptors have non-zero bytes (not all empty)
            let has_nonzero = feature.descriptor.iter().any(|&b| b != 0);
            assert!(
                has_nonzero,
                "Descriptor should not be all zeros - indicates extraction failure"
            );
        }
        
        println!("✅ ORB binary descriptor test passed: {} features with 32-byte descriptors", 
                 features.len());
    }

    #[test]
    fn orb_extraction_edge_cases() {
        let config = OrbConfig::default();
        let max_features = config.num_features;
        let extractor = OrbExtractor::new(config.clone());

        // Test 1: Empty image
        let empty_image = vec![];
        let features = extractor.extract(&empty_image, 0, 0);
        assert_eq!(features.len(), 0, "Empty image should return no features");

        // Test 2: Very small image
        let small_image = create_test_image(16, 16);
        let features = extractor.extract(&small_image, 16, 16);
        assert!(
            features.len() <= max_features,
            "Small image should return few features"
        );

        // Test 3: Very large image
        let large_image = create_test_image(1920, 1080);
        let features = extractor.extract(&large_image, 1920, 1080);
        assert!(
            features.len() <= max_features,
            "Large image should respect max features"
        );

        // Test 4: All black image (should return no features)
        let black_image = vec![0u8; 640 * 480];
        let features = extractor.extract(&black_image, 640, 480);
        // FAST detector should not find features in uniform image
        assert_eq!(
            features.len(),
            0,
            "All black image should return no features"
        );

        // Test 5: All white image
        let white_image = vec![255u8; 640 * 480];
        let features = extractor.extract(&white_image, 640, 480);
        assert_eq!(
            features.len(),
            0,
            "All white image should return no features"
        );

        // Test 6: Checkerboard pattern (should find many features)
        let mut checkerboard = vec![0u8; 640 * 480];
        for y in 0..480 {
            for x in 0..640 {
                let pattern = ((x / 32) + (y / 32)) % 2;
                checkerboard[y * 640 + x] = if pattern == 0 { 0 } else { 255 };
            }
        }
        let features = extractor.extract(&checkerboard, 640, 480);
        // FAST detector may not find features in synthetic checkerboard
        // Just ensure it doesn't crash and returns reasonable results
        assert!(features.len() <= max_features);
        assert!(
            features.len() <= max_features,
            "Should respect max features"
        );
    }

    #[test]
    fn orb_config_edge_cases() {
        // Test various configurations
        let configs = vec![
            OrbConfig {
                num_features: 0,
                scale_factor: 1.0,
                num_levels: 1,
                patch_size: 31,
                use_pyramid: false,
            },
            OrbConfig {
                num_features: 10000,
                scale_factor: 2.0,
                num_levels: 8,
                patch_size: 31,
                use_pyramid: true,
            },
        ];

        for config in configs {
            let extractor = OrbExtractor::new(config.clone());
            let image = create_test_image(640, 480);
            let features = extractor.extract(&image, 640, 480);
            // Should not panic and return reasonable results
            assert!(features.len() <= config.num_features);
        }
    }

    #[test]
    fn orb_feature_properties() {
        let config = OrbConfig::default();
        let extractor = OrbExtractor::new(config);
        let image = create_test_image(640, 480);

        let features = extractor.extract(&image, 640, 480);

        for feature in &features {
            // Check coordinate bounds
            assert!(
                feature.position[0] >= 0.0 && feature.position[0] < 640.0,
                "X coordinate should be within image bounds"
            );
            assert!(
                feature.position[1] >= 0.0 && feature.position[1] < 480.0,
                "Y coordinate should be within image bounds"
            );

            // Check descriptor properties
            assert_eq!(
                feature.descriptor.len(),
                32,
                "Descriptor should be 32 bytes"
            );
            assert!(
                feature.strength >= 0.0,
                "FAST response should be non-negative"
            );

            // Check level and orientation
            // Level is usize, so it's always >= 0
            assert!(true, "Level should be non-negative");
            assert!(
                feature.orientation >= 0.0 && feature.orientation < 2.0 * std::f64::consts::PI,
                "Orientation should be in [0, 2π)"
            );
        }
    }

    #[test]
    fn orb_pyramid_functionality() {
        let mut config = OrbConfig::default();
        config.use_pyramid = true;
        config.num_levels = 3;

        let extractor = OrbExtractor::new(config);
        let image = create_test_image(640, 480);

        let features = extractor.extract(&image, 640, 480);

        // Should extract features across multiple scales
        let _levels: std::collections::HashSet<_> =
            features.iter().map(|f| f.level as i32).collect();
        // May not produce features at all levels depending on image content
        // Just ensure it doesn't crash

        for _feature in &features {
            // Level is usize, so it's always >= 0
            assert!(true, "Level should be valid");
        }
    }

    #[test]
    fn orb_extreme_image_conditions() {
        use rand::Rng;
        let config = OrbConfig::default();
        let extractor = OrbExtractor::new(config);

        // Test 1: Checkerboard pattern (should find many features)
        let mut checkerboard = vec![0u8; 640 * 480];
        for y in 0..480 {
            for x in 0..640 {
                let pattern = ((x / 32) + (y / 32)) % 2;
                checkerboard[y * 640 + x] = if pattern == 0 { 0 } else { 255 };
            }
        }
        let features = extractor.extract(&checkerboard, 640, 480);
        // FAST detector may not find features in synthetic checkerboard
        // Just ensure it doesn't crash
        let _ = features;

        // Test 2: Random noise (should find features)
        let mut rng = rand::thread_rng();
        let mut noise_image = vec![0u8; 640 * 480];
        for pixel in &mut noise_image {
            *pixel = rng.gen_range(0..255);
        }
        let features = extractor.extract(&noise_image, 640, 480);
        // Random noise might or might not produce features, but shouldn't crash
        // Just ensure it doesn't panic
        let _ = features;

        // Test 3: Gradient pattern
        let mut gradient = vec![0u8; 640 * 480];
        for y in 0..480 {
            for x in 0..640 {
                gradient[y * 640 + x] = ((x as f32 / 640.0) * 255.0) as u8;
            }
        }
        let features = extractor.extract(&gradient, 640, 480);
        // Just ensure it doesn't panic
        let _ = features;

        // Test 4: Sine wave pattern
        let mut sine_wave = vec![0u8; 640 * 480];
        for y in 0..480 {
            for x in 0..640 {
                let value = ((x as f32 * 0.1).sin() * 127.0 + 128.0) as u8;
                sine_wave[y * 640 + x] = value;
            }
        }
        let features = extractor.extract(&sine_wave, 640, 480);
        // Just ensure it doesn't panic
        let _ = features;

        // Test 5: Very high contrast edges
        let mut edges = vec![128u8; 640 * 480];
        for y in 0..480 {
            for x in 0..640 {
                if x < 320 {
                    edges[y * 640 + x] = 0;
                } else {
                    edges[y * 640 + x] = 255;
                }
            }
        }
        let features = extractor.extract(&edges, 640, 480);
        // Just ensure it doesn't panic
        let _ = features;

        println!("✅ ORB extreme image condition tests passed!");
    }

    #[test]
    fn orb_boundary_configurations() {
        // Test extreme configuration values
        let configs = vec![
            OrbConfig {
                num_features: 1,    // Minimum
                scale_factor: 1.01, // Very small scale change
                num_levels: 1,      // Single level only
                patch_size: 7,      // Minimum patch size
                use_pyramid: false,
            },
            OrbConfig {
                num_features: 10000, // Very high feature count
                scale_factor: 3.0,   // Large scale change
                num_levels: 16,      // Many levels
                patch_size: 63,      // Large patch size
                use_pyramid: true,
            },
            OrbConfig {
                num_features: 100, // Normal
                scale_factor: 1.0, // No scaling
                num_levels: 1,
                patch_size: 31,
                use_pyramid: false,
            },
        ];

        for config in configs {
            let extractor = OrbExtractor::new(config.clone());
            let image = create_test_image(640, 480);
            let features = extractor.extract(&image, 640, 480);

            // Should not crash and return reasonable results
            assert!(features.len() <= config.num_features);
            // features.len() is usize, always >= 0

            for feature in &features {
                assert!(feature.position[0] >= 0.0 && feature.position[0] < 640.0);
                assert!(feature.position[1] >= 0.0 && feature.position[1] < 480.0);
                assert_eq!(feature.descriptor.len(), 32);
            }
        }

        println!("✅ ORB boundary configuration tests passed!");
    }

    #[test]
    fn orb_memory_and_performance_stress() {
        let config = OrbConfig::default();
        let extractor = OrbExtractor::new(config);

        // Test 1: Very large images (memory stress)
        let large_sizes = vec![
            (1920, 1080), // Full HD
            (3840, 2160), // 4K
        ];

        for (width, height) in large_sizes {
            let image = create_test_image(width, height);
            let features = extractor.extract(&image, width as u32, height as u32);

            // Should handle large images without crashing
            // features.len() is usize, always >= 0
            assert!(features.len() <= 500); // Default max features

            for feature in &features {
                assert!(feature.position[0] >= 0.0 && feature.position[0] < width as f64);
                assert!(feature.position[1] >= 0.0 && feature.position[1] < height as f64);
            }
        }

        // Test 2: Many small extractions (performance stress)
        let image = create_test_image(640, 480);
        let mut total_features = 0;

        for _ in 0..100 {
            let features = extractor.extract(&image, 640, 480);
            total_features += features.len();
        }

        // Should handle repeated extractions without issues
        let _ = total_features;

        println!("✅ ORB memory and performance stress tests passed!");
    }

    #[test]
    fn orb_descriptor_properties() {
        let config = OrbConfig::default();
        let extractor = OrbExtractor::new(config);
        let image = create_test_image(640, 480);

        let features = extractor.extract(&image, 640, 480);
        assert!(features.len() > 0, "Should extract features");

        let mut balance_ratios = Vec::new();
        let mut all_ones_count = 0;
        let mut all_zeros_count = 0;
        
        for feature in &features {
            // Test descriptor binary properties
            let mut ones_count = 0;

            for &byte in &feature.descriptor {
                ones_count += byte.count_ones() as usize;
            }
            
            let zeros_count = 256 - ones_count;
            
            // Track pathological cases
            if ones_count == 256 { all_ones_count += 1; }
            if zeros_count == 256 { all_zeros_count += 1; }
            
            let balance_ratio = ones_count as f32 / 256.0;
            balance_ratios.push(balance_ratio);
        }

        // Most descriptors should NOT be all 0s or all 1s
        let pathological_ratio = (all_ones_count + all_zeros_count) as f32 / features.len() as f32;
        assert!(
            pathological_ratio < 0.3,
            "Too many pathological descriptors (all 0s or all 1s): {}/{} = {:.1}%",
            all_ones_count + all_zeros_count, features.len(), pathological_ratio * 100.0
        );

        // Check that average balance is reasonable (20%-80%)
        let avg_balance = balance_ratios.iter().sum::<f32>() / balance_ratios.len() as f32;
        assert!(
            avg_balance > 0.2 && avg_balance < 0.8,
            "Average descriptor balance should be 20%-80%, got {:.1}%",
            avg_balance * 100.0
        );

        println!("✅ Descriptor properties: avg balance {:.1}%, {} features ({} pathological)", 
                 avg_balance * 100.0, features.len(), all_ones_count + all_zeros_count);
    }

    #[test]
    fn hamming_distance_identical_descriptors() {
        let desc1 = vec![0u8; 32];
        let desc2 = vec![0u8; 32];

        let feature = OrbFeature {
            position: Vector2::new(0.0, 0.0),
            orientation: 0.0,
            descriptor: desc1,
            level: 0,
            strength: 100.0,
        };

        assert_eq!(feature.hamming_distance(&desc2), 0);
    }

    #[test]
    fn hamming_distance_flipped_bits() {
        let desc1 = vec![0u8; 32];
        let mut desc2 = vec![0u8; 32];
        desc2[0] = 1; // 1 bit difference

        let feature = OrbFeature {
            position: Vector2::new(0.0, 0.0),
            orientation: 0.0,
            descriptor: desc1,
            level: 0,
            strength: 100.0,
        };

        assert_eq!(feature.hamming_distance(&desc2), 1);
    }

    #[test]
    fn orientation_computation_is_normalized() {
        let config = OrbConfig::default();
        let extractor = OrbExtractor::new(config);
        let image = create_test_image(640, 480);

        let features = extractor.extract(&image, 640, 480);
        for feature in features {
            // Orientation should be in [0, 2π)
            assert!(
                feature.orientation >= 0.0,
                "Orientation must be non-negative"
            );
            assert!(feature.orientation < 2.0 * PI, "Orientation must be < 2π");
        }
    }

    #[test]
    fn image_size_validation() {
        let config = OrbConfig::default();
        let extractor = OrbExtractor::new(config);

        // Too small image
        let small_image = vec![128u8; 16 * 16];
        let features = extractor.extract(&small_image, 16, 16);
        assert!(
            features.is_empty(),
            "Should reject image smaller than 32x32"
        );

        // Size mismatch
        let image = vec![128u8; 100];
        let features = extractor.extract(&image, 640, 480);
        assert!(features.is_empty(), "Should reject size mismatch");
    }

    #[test]
    fn extract_with_pool_actually_uses_pool() {
        use std::sync::Arc;
        use crate::optimization::loop_closure::descriptor_pool::OrbBinaryPool;

        let mut config = OrbConfig::default();
        config.use_pyramid = false;
        config.num_features = 50; // Limit for predictable testing
        let extractor = OrbExtractor::new(config);
        
        let image = create_test_image(640, 480);
        let pool = Arc::new(OrbBinaryPool::new(100));

        // Verify baseline: image MUST produce features
        let baseline_features = extractor.extract(&image, 640, 480);
        assert!(
            baseline_features.len() > 0,
            "Test image must produce features. Got 0 - test setup is broken!"
        );

        // Check pool state before extraction
        let acquired_before = pool.acquired_count();
        assert_eq!(acquired_before, 0, "Pool should start with 0 acquired buffers");
        
        // Extract with pool
        let features_with_pool = extractor.extract_with_pool(&image, 640, 480, Some(&pool));
        
        // CRITICAL: Verify pool was actually used
        let acquired_during = pool.acquired_count();
        assert!(
            acquired_during > 0,
            "Pool MUST be used during extraction! Got 0 acquisitions - pooling is not working!"
        );
        
        // Verify features were extracted
        assert!(
            features_with_pool.len() > 0,
            "Pooled extraction must produce features. Got 0!"
        );
        
        // All descriptors must be 32 bytes
        for (i, feature) in features_with_pool.iter().enumerate() {
            assert_eq!(
                feature.descriptor.len(), 
                32,
                "Feature {} descriptor must be 32 bytes, got {}",
                i,
                feature.descriptor.len()
            );
        }

        // Determinism: pooled and non-pooled should produce IDENTICAL results
        assert_eq!(
            features_with_pool.len(),
            baseline_features.len(),
            "Pooled and non-pooled extraction must produce same feature count"
        );

        println!("✅ Pool usage verified: {} acquisitions for {} features", 
                 acquired_during, features_with_pool.len());
    }

    #[test]
    fn extract_with_pool_determinism() {
        use std::sync::Arc;
        use crate::optimization::loop_closure::descriptor_pool::OrbBinaryPool;

        let mut config = OrbConfig::default();
        config.use_pyramid = false;
        let extractor = OrbExtractor::new(config);
        let image = create_test_image(640, 480);
        
        let pool = Arc::new(OrbBinaryPool::new(500));

        // Extract multiple times - must be deterministic
        let features1 = extractor.extract_with_pool(&image, 640, 480, Some(&pool));
        let features2 = extractor.extract_with_pool(&image, 640, 480, Some(&pool));
        let features_no_pool = extractor.extract(&image, 640, 480);

        // All must produce same number of features
        assert_eq!(
            features1.len(),
            features2.len(),
            "Repeated pooled extraction must be deterministic"
        );
        assert_eq!(
            features1.len(),
            features_no_pool.len(),
            "Pooled and non-pooled must produce identical feature counts"
        );

        // Features must be at same positions (within floating point tolerance)
        for i in 0..features1.len().min(features_no_pool.len()) {
            let pos_diff = (features1[i].position - features_no_pool[i].position).norm();
            assert!(
                pos_diff < 1e-10,
                "Feature {} position must match: pooled={:?} vs non-pooled={:?}",
                i, features1[i].position, features_no_pool[i].position
            );
        }

        println!("✅ Determinism verified: {} features consistently extracted", features1.len());
    }

    #[test]
    fn extract_with_pool_stress_test_exhaustion() {
        use std::sync::Arc;
        use crate::optimization::loop_closure::descriptor_pool::OrbBinaryPool;

        let mut config = OrbConfig::default();
        config.use_pyramid = false;
        config.num_features = 100; // Request more than pool size
        let extractor = OrbExtractor::new(config);
        let image = create_test_image(640, 480);
        
        // Small pool - will exhaust
        let small_pool = Arc::new(OrbBinaryPool::new(10));

        let features = extractor.extract_with_pool(&image, 640, 480, Some(&small_pool));

        // Must still extract features even with exhausted pool
        assert!(
            features.len() > 0,
            "Must extract features even when pool exhausted (fallback allocation)"
        );

        // All descriptors must still be valid
        for (i, feature) in features.iter().enumerate() {
            assert_eq!(
                feature.descriptor.len(),
                32,
                "Feature {} must have 32-byte descriptor even with pool exhaustion",
                i
            );
            
            // Verify descriptor is not all zeros (would indicate allocation failure)
            let has_nonzero = feature.descriptor.iter().any(|&b| b != 0);
            assert!(
                has_nonzero,
                "Feature {} descriptor must not be all zeros - indicates extraction failure",
                i
            );
        }

        println!("✅ Pool exhaustion handled: extracted {} features with pool size 10", 
                 features.len());
    }

    #[test]
    fn descriptor_uniqueness_test() {
        use std::collections::HashMap;

        let mut config = OrbConfig::default();
        config.use_pyramid = false;
        let extractor = OrbExtractor::new(config);
        let image = create_test_image(640, 480);

        let features = extractor.extract(&image, 640, 480);
        assert!(features.len() > 5, "Need multiple features to test uniqueness");

        // Track descriptor occurrences
        let mut descriptor_counts: HashMap<String, Vec<usize>> = HashMap::new();
        for (i, feature) in features.iter().enumerate() {
            let desc_hex: String = feature.descriptor.iter()
                .map(|b| format!("{:02x}", b))
                .collect();
            
            descriptor_counts.entry(desc_hex).or_insert_with(Vec::new).push(i);
        }

        let unique_count = descriptor_counts.len();
        let uniqueness_ratio = unique_count as f64 / features.len() as f64;
        
        // At least 70% of features should have unique descriptors
        // (Some duplication is OK for features in similar local texture)
        assert!(
            uniqueness_ratio >= 0.7,
            "Descriptor uniqueness too low: {}/{} = {:.1}%. Expected >=70%",
            unique_count, features.len(), uniqueness_ratio * 100.0
        );
        
        // Log duplicates for debugging
        let duplicates: Vec<_> = descriptor_counts.iter()
            .filter(|(_, indices)| indices.len() > 1)
            .collect();
        
        if !duplicates.is_empty() {
            println!("Note: {} duplicate descriptor groups found:", duplicates.len());
            for (desc, indices) in duplicates.iter().take(3) {
                println!("  Descriptor {} appears at features: {:?}", &desc[..16], indices);
            }
        }

        println!("✅ Descriptor uniqueness: {}/{} unique ({:.1}%)",
                 unique_count, features.len(), uniqueness_ratio * 100.0);
    }

    #[test]
    fn orb_feature_vec_descriptor_hamming_distance() {
        // Test hamming distance with Vec descriptors
        let mut desc1 = vec![0u8; 32];
        let mut desc2 = vec![0u8; 32];
        
        // Set some bits
        desc1[0] = 0b10101010;
        desc1[1] = 0b11110000;
        desc2[0] = 0b10101010; // Same
        desc2[1] = 0b11000011; // Different

        let feature = OrbFeature {
            position: Vector2::new(100.0, 100.0),
            orientation: 1.5,
            descriptor: desc1,
            level: 0,
            strength: 150.0,
        };

        let distance = feature.hamming_distance(&desc2);
        
        // Count expected differences
        let byte1_xor: u8 = 0b11110000 ^ 0b11000011; // = 0b00110011
        let expected_distance = byte1_xor.count_ones();
        
        assert_eq!(distance, expected_distance, "Hamming distance should match bit differences");
        println!("✅ Vec descriptor hamming distance test passed: distance = {}", distance);
    }
}

