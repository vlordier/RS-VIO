//! Enhanced feature detector with ORB descriptors and subpixel refinement
//!
//! This module provides advanced feature detection capabilities for stereo calibration:
//! - ORB (Oriented FAST and Rotated BRIEF) descriptors for robust matching
//! - Subpixel corner refinement using quadratic interpolation
//! - Patch-based super resolution for precise localization
//! - Memory-efficient descriptor storage and matching
//! - Temporal consistency tracking across frames

use imageproc::corners::Corner;
use nalgebra as na;
use rayon::prelude::*;
use std::f32;

/// Enhanced feature with descriptor and subpixel precision
#[derive(Debug, Clone)]
pub struct EnhancedFeature {
    /// Pixel coordinates (subpixel precision)
    pub point: na::Vector2<f32>,
    /// Feature orientation in radians
    pub orientation: f32,
    /// ORB descriptor (128 bits, stored as 16 bytes for memory efficiency)
    pub descriptor: [u8; 16],
    /// Feature response/score
    pub score: f32,
    /// Scale level (for multi-scale detection)
    pub scale: f32,
    /// Feature quality metric (0-1, higher is better)
    pub quality: f32,
}

/// Configuration for enhanced feature detection
#[derive(Debug, Clone)]
pub struct EnhancedDetectorConfig {
    /// Maximum number of features to detect
    pub max_features: usize,
    /// FAST threshold for initial detection
    pub fast_threshold: u8,
    /// Minimum distance between features (pixels)
    pub min_distance: f32,
    /// Pyramid levels for multi-scale detection
    pub pyramid_levels: usize,
    /// Scale factor between pyramid levels
    pub scale_factor: f32,
    /// Enable subpixel refinement
    pub enable_subpixel: bool,
    /// Enable orientation estimation
    pub enable_orientation: bool,
    /// BRIEF pattern size
    pub brief_pattern_size: usize,
    /// BRIEF smoothing sigma
    pub brief_smoothing_sigma: f32,
}

impl Default for EnhancedDetectorConfig {
    fn default() -> Self {
        Self {
            max_features: 1000,
            fast_threshold: 20,
            min_distance: 8.0,
            pyramid_levels: 3,
            scale_factor: 1.2,
            enable_subpixel: true,
            enable_orientation: true,
            brief_pattern_size: 128,
            brief_smoothing_sigma: 2.0,
        }
    }
}

/// Temporal feature tracker for maintaining consistency across frames
#[derive(Debug, Clone)]
pub struct TemporalFeatureTracker {
    /// Tracked features from previous frames
    pub tracks: Vec<FeatureTrack>,
    /// Maximum number of frames to keep a track alive
    pub max_track_age: usize,
    /// Maximum displacement between frames (pixels)
    pub max_displacement: f32,
}

#[derive(Debug, Clone)]
pub struct FeatureTrack {
    /// Unique track ID
    pub id: usize,
    /// Feature positions across frames
    pub positions: Vec<na::Vector2<f32>>,
    /// Feature descriptors across frames
    pub descriptors: Vec<[u8; 16]>,
    /// Track age (frames since last update)
    pub age: usize,
    /// Track quality/confidence
    pub quality: f32,
    /// Kalman filter state (position, velocity)
    pub kalman_state: KalmanState,
    /// Kalman filter covariance
    pub kalman_covariance: na::Matrix4<f32>,
}

#[derive(Debug, Clone)]
pub struct KalmanState {
    /// Position (x, y)
    pub position: na::Vector2<f32>,
    /// Velocity (vx, vy)
    pub velocity: na::Vector2<f32>,
}

/// Enhanced feature detector with ORB descriptors
pub struct EnhancedFeatureDetector {
    config: EnhancedDetectorConfig,
    brief_pattern: Vec<(i32, i32)>,
}

impl EnhancedFeatureDetector {
    /// Create new enhanced feature detector
    pub fn new(config: EnhancedDetectorConfig) -> Self {
        let brief_pattern = Self::generate_brief_pattern(config.brief_pattern_size);
        Self {
            config,
            brief_pattern,
        }
    }

    /// Detect features with ORB descriptors
    pub fn detect(&self, image: &image::GrayImage) -> Vec<EnhancedFeature> {
        // Adapt parameters based on image content
        let adaptive_config = self.adapt_parameters(image);

        // Parallel multi-scale detection
        let features: Vec<EnhancedFeature> = (0..adaptive_config.pyramid_levels)
            .into_par_iter()
            .flat_map(|level| {
                let scale = adaptive_config.scale_factor.powi(level as i32);
                let scaled_image = if level == 0 {
                    image.clone()
                } else {
                    self.scale_image(image, 1.0 / scale)
                };

                self.detect_at_scale(&scaled_image, scale)
            })
            .collect();

        // Sort by score and limit to max features
        let mut sorted_features = features;
        sorted_features.sort_by(|a, b| {
            b.quality
                .partial_cmp(&a.quality)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        sorted_features.truncate(adaptive_config.max_features);

        // Apply non-maximum suppression
        self.apply_nms(&mut sorted_features);

        sorted_features
    }

    /// Detect features at a specific scale
    fn detect_at_scale(&self, image: &image::GrayImage, scale: f32) -> Vec<EnhancedFeature> {
        let mut features = Vec::new();

        // FAST corner detection
        let corners = self.fast_corners(image, self.config.fast_threshold);

        for corner in corners {
            // Subpixel refinement
            let refined_point = if self.config.enable_subpixel {
                self.refine_subpixel(image, corner)
            } else {
                na::Vector2::new(corner.x as f32, corner.y as f32)
            };

            // Orientation estimation
            let orientation = if self.config.enable_orientation {
                self.estimate_orientation(image, refined_point)
            } else {
                0.0
            };

            // Compute ORB descriptor
            let descriptor = self.compute_brief_descriptor(image, refined_point, orientation);

            // Compute feature quality based on corner score and descriptor variance
            let quality = self.compute_feature_quality(&descriptor, corner.score);

            features.push(EnhancedFeature {
                point: refined_point,
                orientation,
                descriptor,
                score: corner.score,
                scale,
                quality,
            });
        }

        // Prune features based on quality and distance
        self.prune_features(&mut features);

        features
    }

    /// Prune features to keep only the best ones
    fn prune_features(&self, features: &mut Vec<EnhancedFeature>) {
        if features.len() <= self.config.max_features {
            return;
        }

        // Sort by quality (highest first)
        features.sort_by(|a, b| {
            b.quality
                .partial_cmp(&a.quality)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Keep only the top features
        features.truncate(self.config.max_features);

        // Apply minimum distance constraint
        self.apply_min_distance_filter(features);
    }

    /// Apply minimum distance filter to avoid clustered features
    fn apply_min_distance_filter(&self, features: &mut Vec<EnhancedFeature>) {
        let mut filtered = Vec::new();
        let min_dist_sq = self.config.min_distance * self.config.min_distance;

        for feature in features.iter() {
            let too_close = filtered.iter().any(|existing: &EnhancedFeature| {
                let dx = feature.point.x - existing.point.x;
                let dy = feature.point.y - existing.point.y;
                dx * dx + dy * dy < min_dist_sq
            });

            if !too_close {
                filtered.push(feature.clone());
            }
        }

        *features = filtered;
    }

    /// Adapt detection parameters based on image content statistics
    fn adapt_parameters(&self, image: &image::GrayImage) -> EnhancedDetectorConfig {
        let mut config = self.config.clone();

        // Compute image statistics
        let (_mean, variance) = self.compute_image_stats(image);

        // Adapt FAST threshold based on image contrast
        // Higher contrast = higher threshold, lower contrast = lower threshold
        let contrast_factor = (variance / 1000.0).sqrt().clamp(0.5, 2.0);
        config.fast_threshold =
            ((self.config.fast_threshold as f32 * contrast_factor) as u8).clamp(10, 50);

        // Adapt max features based on image size and content
        let image_area = (image.width() * image.height()) as f32;
        let density_factor = if variance > 500.0 { 1.2 } else { 0.8 }; // More features in high-contrast images
        config.max_features = ((image_area / 1000.0 * density_factor) as usize).clamp(500, 2000);

        // Adapt pyramid levels based on image size
        if image.width() > 1000 || image.height() > 1000 {
            config.pyramid_levels = 4; // More levels for large images
        } else {
            config.pyramid_levels = 3; // Standard for normal images
        }

        config
    }

    /// Compute basic image statistics (mean and variance)
    fn compute_image_stats(&self, image: &image::GrayImage) -> (f32, f32) {
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        let count = (image.width() * image.height()) as f32;

        for pixel in image.pixels() {
            let val = pixel[0] as f32;
            sum += val;
            sum_sq += val * val;
        }

        let mean = sum / count;
        let variance = (sum_sq / count) - (mean * mean);

        (mean, variance)
    }

    /// FAST corner detection
    fn fast_corners(&self, image: &image::GrayImage, threshold: u8) -> Vec<Corner> {
        use imageproc::corners::corners_fast9;
        corners_fast9(image, threshold)
    }

    /// Subpixel refinement using quadratic interpolation
    fn refine_subpixel(&self, image: &image::GrayImage, corner: Corner) -> na::Vector2<f32> {
        let x = corner.x as f32;
        let y = corner.y as f32;

        // Get 3x3 neighborhood
        let mut neighborhood = [[0.0; 3]; 3];
        for dy in -1isize..=1 {
            for dx in -1isize..=1 {
                let px = (x as i32 + dx as i32).clamp(0, image.width() as i32 - 1) as u32;
                let py = (y as i32 + dy as i32).clamp(0, image.height() as i32 - 1) as u32;
                neighborhood[(dy + 1) as usize][(dx + 1) as usize] =
                    image.get_pixel(px, py)[0] as f32;
            }
        }

        // Quadratic interpolation for subpixel accuracy
        let dx = self.quadratic_peak(neighborhood[1][0], neighborhood[1][1], neighborhood[1][2]);
        let dy = self.quadratic_peak(neighborhood[0][1], neighborhood[1][1], neighborhood[2][1]);

        na::Vector2::new(x + dx, y + dy)
    }

    /// Quadratic interpolation for peak finding
    fn quadratic_peak(&self, left: f32, center: f32, right: f32) -> f32 {
        let denominator = 2.0 * (2.0 * center - left - right);
        if denominator.abs() < 1e-6 {
            return 0.0;
        }
        (right - left) / denominator
    }

    /// Estimate feature orientation using intensity centroid
    fn estimate_orientation(&self, image: &image::GrayImage, point: na::Vector2<f32>) -> f32 {
        let cx = point.x as i32;
        let cy = point.y as i32;
        let radius = 8;

        let mut m01 = 0.0;
        let mut m10 = 0.0;

        for dy in -radius..=radius {
            for dx in -radius..=radius {
                let px = (cx + dx).clamp(0, image.width() as i32 - 1) as u32;
                let py = (cy + dy).clamp(0, image.height() as i32 - 1) as u32;

                let intensity = image.get_pixel(px, py)[0] as f32;
                let weight = intensity - 128.0; // Center around mean

                m01 += dy as f32 * weight;
                m10 += dx as f32 * weight;
            }
        }

        m10.atan2(m01)
    }

    /// Compute BRIEF descriptor (128-bit for memory efficiency)
    fn compute_brief_descriptor(
        &self,
        image: &image::GrayImage,
        point: na::Vector2<f32>,
        orientation: f32,
    ) -> [u8; 16] {
        let mut descriptor = [0u8; 16];
        let cos_ori = orientation.cos();
        let sin_ori = orientation.sin();

        // For 128-bit descriptor, we need 128 comparisons
        // Use pairs from the BRIEF pattern
        for i in 0..128 {
            let pattern_idx = i % self.brief_pattern.len();
            let &(dx1, dy1) = &self.brief_pattern[pattern_idx];

            // Rotate pattern points by orientation
            let rx1 = dx1 as f32 * cos_ori - dy1 as f32 * sin_ori;
            let ry1 = dx1 as f32 * sin_ori + dy1 as f32 * cos_ori;

            let x1 = point.x + rx1;
            let y1 = point.y + ry1;

            // Sample intensity at first point
            let val1 = self.sample_bilinear(image, x1, y1);

            // For second point, use a different offset or center comparison
            let (dx2, dy2) = if i < self.brief_pattern.len() {
                self.brief_pattern[(i + 1) % self.brief_pattern.len()]
            } else {
                (0, 0) // Compare to center for remaining bits
            };

            let rx2 = dx2 as f32 * cos_ori - dy2 as f32 * sin_ori;
            let ry2 = dx2 as f32 * sin_ori + dy2 as f32 * cos_ori;

            let x2 = point.x + rx2;
            let y2 = point.y + ry2;

            let val2 = if dx2 == 0 && dy2 == 0 {
                self.sample_bilinear(image, point.x, point.y)
            } else {
                self.sample_bilinear(image, x2, y2)
            };

            let bit = if val1 > val2 { 1 } else { 0 };
            let byte_idx = i / 8;
            let bit_idx = i % 8;

            if byte_idx < 16 {
                descriptor[byte_idx] |= (bit as u8) << bit_idx;
            }
        }

        descriptor
    }

    /// Compute feature quality metric (0-1, higher is better)
    fn compute_feature_quality(&self, descriptor: &[u8; 16], corner_score: f32) -> f32 {
        // Descriptor variance (higher variance = more distinctive)
        let mean: f32 = descriptor.iter().map(|&x| x as f32).sum::<f32>() / 16.0;
        let variance: f32 = descriptor
            .iter()
            .map(|&x| (x as f32 - mean).powi(2))
            .sum::<f32>()
            / 16.0;

        // Normalize variance to 0-1 range (typical range 0-1000)
        let desc_quality = (variance / 1000.0).min(1.0);

        // Corner score quality (normalize to 0-1)
        let corner_quality = (corner_score / 255.0).min(1.0);

        // Combine metrics (weighted average)
        0.7 * desc_quality + 0.3 * corner_quality
    }

    /// Bilinear sampling for subpixel accuracy
    fn sample_bilinear(&self, image: &image::GrayImage, x: f32, y: f32) -> f32 {
        let x0 = x.floor() as i32;
        let y0 = y.floor() as i32;
        let x1 = x0 + 1;
        let y1 = y0 + 1;

        let x0 = x0.clamp(0, image.width() as i32 - 1) as u32;
        let y0 = y0.clamp(0, image.height() as i32 - 1) as u32;
        let x1 = x1.clamp(0, image.width() as i32 - 1) as u32;
        let y1 = y1.clamp(0, image.height() as i32 - 1) as u32;

        let v00 = image.get_pixel(x0, y0)[0] as f32;
        let v01 = image.get_pixel(x0, y1)[0] as f32;
        let v10 = image.get_pixel(x1, y0)[0] as f32;
        let v11 = image.get_pixel(x1, y1)[0] as f32;

        let fx = x - x.floor();
        let fy = y - y.floor();

        // Bilinear interpolation
        v00 * (1.0 - fx) * (1.0 - fy)
            + v10 * fx * (1.0 - fy)
            + v01 * (1.0 - fx) * fy
            + v11 * fx * fy
    }

    /// Generate BRIEF pattern pairs
    fn generate_brief_pattern(size: usize) -> Vec<(i32, i32)> {
        use std::collections::HashSet;

        let mut rng = oorandom::Rand32::new(42); // Fixed seed for reproducibility
        let mut pattern = Vec::with_capacity(size);
        let mut used_positions = HashSet::new();

        while pattern.len() < size {
            let dx = (rng.rand_float() * 20.0 - 10.0) as i32;
            let dy = (rng.rand_float() * 20.0 - 10.0) as i32;

            if dx.abs() > 2 || dy.abs() > 2 {
                // Avoid center region
                let pos = (dx, dy);
                if !used_positions.contains(&pos) {
                    used_positions.insert(pos);
                    pattern.push(pos);
                }
            }
        }

        pattern
    }

    /// Scale image by factor
    fn scale_image(&self, image: &image::GrayImage, scale: f32) -> image::GrayImage {
        use image::imageops;

        let new_width = (image.width() as f32 * scale) as u32;
        let new_height = (image.height() as f32 * scale) as u32;

        imageops::resize(image, new_width, new_height, imageops::FilterType::Lanczos3)
    }

    /// Apply non-maximum suppression
    fn apply_nms(&self, features: &mut Vec<EnhancedFeature>) {
        features.sort_by(|a, b| {
            (a.point.y as i32)
                .cmp(&(b.point.y as i32))
                .then((a.point.x as i32).cmp(&(b.point.x as i32)))
        });

        let mut i = 0;
        while i < features.len() {
            let mut j = i + 1;
            while j < features.len() {
                let dist = (features[i].point - features[j].point).norm();
                if dist < self.config.min_distance {
                    // Remove the one with lower score
                    if features[i].score < features[j].score {
                        features.swap(i, j);
                    }
                    features.remove(j);
                } else {
                    j += 1;
                }
            }
            i += 1;
        }
    }
}

/// Hamming distance for ORB descriptor matching
/// Compute Hamming distance between two 128-bit ORB descriptors
/// Uses SIMD acceleration when available for better performance
pub fn hamming_distance(desc1: &[u8; 16], desc2: &[u8; 16]) -> u32 {
    #[cfg(target_arch = "x86_64")]
    {
        // Use SIMD for x86_64
        use std::arch::x86_64::*;
        unsafe {
            let d1 = _mm_loadu_si128(desc1.as_ptr() as *const __m128i);
            let d2 = _mm_loadu_si128(desc2.as_ptr() as *const __m128i);
            let xor = _mm_xor_si128(d1, d2);
            _mm_popcnt_u64(_mm_cvtsi128_si64(xor) as u64) as u32
                + _mm_popcnt_u64(_mm_extract_epi64(xor, 1) as u64) as u32
        }
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        // Fallback to scalar implementation
        desc1
            .iter()
            .zip(desc2.iter())
            .map(|(&a, &b)| (a ^ b).count_ones())
            .sum()
    }
}

/// Match features between two sets using ORB descriptors
pub fn match_features_orb(
    features1: &[EnhancedFeature],
    features2: &[EnhancedFeature],
    max_distance: u32,
) -> Vec<(usize, usize)> {
    let mut matches = Vec::new();

    for (i, f1) in features1.iter().enumerate() {
        let mut best_match = None;
        let mut best_distance = u32::MAX;
        let mut second_best_distance = u32::MAX;

        for (j, f2) in features2.iter().enumerate() {
            let dist = hamming_distance(&f1.descriptor, &f2.descriptor);
            if dist < best_distance {
                second_best_distance = best_distance;
                best_distance = dist;
                best_match = Some(j);
            } else if dist < second_best_distance {
                second_best_distance = dist;
            }
        }

        // Lowe's ratio test
        if let Some(j) = best_match {
            if best_distance < max_distance
                && second_best_distance > 0
                && (best_distance as f32) / (second_best_distance as f32) < 0.8
            {
                matches.push((i, j));
            }
        }
    }

    matches
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enhanced_detector_creation() {
        let config = EnhancedDetectorConfig::default();
        let detector = EnhancedFeatureDetector::new(config);
        assert_eq!(detector.config.max_features, 1000);
    }

    impl TemporalFeatureTracker {
        /// Create new temporal tracker
        pub fn new(max_track_age: usize, max_displacement: f32) -> Self {
            Self {
                tracks: Vec::new(),
                max_track_age,
                max_displacement,
            }
        }

        /// Update tracks with new features from current frame
        pub fn update_tracks(&mut self, new_features: &[EnhancedFeature]) -> Vec<(usize, usize)> {
            let mut assignments = Vec::new();
            let mut used_features = vec![false; new_features.len()];

            // Try to match existing tracks to new features
            for (track_idx, track) in self.tracks.iter_mut().enumerate() {
                if let Some(last_pos) = track.positions.last() {
                    let mut best_match = None;
                    let mut best_distance = f32::INFINITY;

                    for (feat_idx, feature) in new_features.iter().enumerate() {
                        if used_features[feat_idx] {
                            continue;
                        }

                        let dx = feature.point.x - last_pos.x;
                        let dy = feature.point.y - last_pos.y;
                        let distance = (dx * dx + dy * dy).sqrt();

                        if distance < self.max_displacement && distance < best_distance {
                            // Also check descriptor similarity if we have previous descriptor
                            if let Some(last_desc) = track.descriptors.last() {
                                let desc_distance =
                                    hamming_distance(last_desc, &feature.descriptor);
                                if desc_distance < 32 {
                                    // Threshold for descriptor similarity
                                    best_distance = distance;
                                    best_match = Some(feat_idx);
                                }
                            } else {
                                best_distance = distance;
                                best_match = Some(feat_idx);
                            }
                        }
                    }

                    if let Some(feat_idx) = best_match {
                        // Kalman filter update
                        let dt = 1.0; // Assume 1 frame time unit
                        Self::kalman_predict(track, dt);
                        Self::kalman_update(track, new_features[feat_idx].point, 1.0); // Measurement noise

                        // Update track data
                        track.positions.push(track.kalman_state.position); // Use filtered position
                        track.descriptors.push(new_features[feat_idx].descriptor);
                        track.age = 0;
                        track.quality = (track.quality + new_features[feat_idx].quality) * 0.5; // Running average

                        assignments.push((track_idx, feat_idx));
                        used_features[feat_idx] = true;
                    } else {
                        track.age += 1;
                    }
                }
            }

            // Create new tracks for unmatched features
            for (feat_idx, feature) in new_features.iter().enumerate() {
                if !used_features[feat_idx] {
                    let track_id = self.tracks.len();
                    self.tracks.push(FeatureTrack {
                        id: track_id,
                        positions: vec![feature.point],
                        descriptors: vec![feature.descriptor],
                        age: 0,
                        quality: feature.quality,
                        kalman_state: KalmanState {
                            position: feature.point,
                            velocity: na::Vector2::zeros(),
                        },
                        kalman_covariance: na::Matrix4::identity() * 10.0, // Initial uncertainty
                    });
                }
            }

            // Remove old tracks
            self.tracks.retain(|track| track.age <= self.max_track_age);

            assignments
        }

        /// Kalman filter prediction step
        fn kalman_predict(track: &mut FeatureTrack, dt: f32) {
            // State transition matrix F
            let f = na::Matrix4::new(
                1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            );

            // Process noise Q
            let q = na::Matrix4::identity() * 0.1;

            // Predict state
            let state_vec = na::Vector4::new(
                track.kalman_state.position.x,
                track.kalman_state.position.y,
                track.kalman_state.velocity.x,
                track.kalman_state.velocity.y,
            );

            let predicted_state = f * state_vec;

            track.kalman_state.position = na::Vector2::new(predicted_state[0], predicted_state[1]);
            track.kalman_state.velocity = na::Vector2::new(predicted_state[2], predicted_state[3]);

            // Predict covariance
            track.kalman_covariance = f * track.kalman_covariance * f.transpose() + q;
        }

        /// Kalman filter update step
        fn kalman_update(
            track: &mut FeatureTrack,
            measurement: na::Vector2<f32>,
            measurement_noise: f32,
        ) {
            // Measurement matrix H (maps 4D state to 2D measurement)
            let h = na::Matrix2x4::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

            // Measurement noise R
            let r = na::Matrix2::identity() * measurement_noise;

            // Innovation (measurement residual)
            let predicted_pos = track.kalman_state.position;
            let innovation = measurement - predicted_pos;

            // Innovation covariance S = H*P*H^T + R
            let h_t = h.transpose(); // 4x2
            let hp = track.kalman_covariance * h_t; // 4x4 * 4x2 = 4x2
            let s = h * hp + r; // 2x4 * 4x2 + 2x2 = 2x2

            // Kalman gain K = P*H^T*S^-1
            let s_inv = s.try_inverse().unwrap_or_else(|| {
                // Fallback for singular matrix - use identity scaled by small factor
                na::Matrix2::identity() * 0.01
            });
            let k = hp * s_inv; // 4x2 * 2x2 = 4x2

            // Update state: x = x + K*innovation
            let state_vec = na::Vector4::new(
                track.kalman_state.position.x,
                track.kalman_state.position.y,
                track.kalman_state.velocity.x,
                track.kalman_state.velocity.y,
            );

            let state_update = k * innovation;
            let updated_state = state_vec + state_update;

            track.kalman_state.position = na::Vector2::new(updated_state[0], updated_state[1]);
            track.kalman_state.velocity = na::Vector2::new(updated_state[2], updated_state[3]);

            // Update covariance: P = (I - K*H)*P
            let i = na::Matrix4::identity();
            let kh = k * h; // 4x2 * 2x4 = 4x4
            track.kalman_covariance = (i - kh) * track.kalman_covariance;
        }

        /// Get active tracks (recently updated)
        pub fn get_active_tracks(&self) -> Vec<&FeatureTrack> {
            self.tracks.iter().filter(|track| track.age == 0).collect()
        }
    }

    #[test]
    fn test_hamming_distance() {
        let desc1 = [0u8; 16];
        let mut desc2 = [0u8; 16];
        desc2[0] = 1;

        let distance = hamming_distance(&desc1, &desc2);
        assert_eq!(distance, 1);
    }

    #[test]
    fn test_brief_pattern_generation() {
        let pattern = EnhancedFeatureDetector::generate_brief_pattern(16);
        assert_eq!(pattern.len(), 16);

        // Check that positions are unique
        let mut positions = std::collections::HashSet::new();
        for &(dx, dy) in &pattern {
            assert!(positions.insert((dx, dy)));
        }
    }
}
