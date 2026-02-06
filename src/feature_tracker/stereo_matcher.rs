//! Stereo feature matching with enhanced descriptors
//!
//! This module provides robust stereo correspondence using ORB descriptors
//! with geometric constraints and outlier rejection for calibration.

use crate::feature_tracker::enhanced_detector::{hamming_distance, EnhancedFeature};
use image;
use nalgebra as na;
use std::collections::HashMap;

/// Stereo correspondence between left and right features
#[derive(Debug, Clone)]
pub struct StereoMatch {
    /// Index in left feature list
    pub left_idx: usize,
    /// Index in right feature list
    pub right_idx: usize,
    /// Matching score (lower is better for Hamming distance)
    pub score: u32,
    /// Epipolar error in pixels
    pub epipolar_error: f32,
    /// Confidence score (0-1, higher is better)
    pub confidence: f32,
}

/// Temporal depth information for a feature track
#[derive(Debug, Clone)]
pub struct TemporalDepthInfo {
    /// Feature position in left image
    pub position: na::Vector2<f32>,
    /// Accumulated depth measurements
    pub depth_measurements: Vec<f32>,
    /// Accumulated confidence values
    pub confidence_values: Vec<f32>,
    /// Filtered depth estimate
    pub filtered_depth: f32,
    /// Depth uncertainty
    pub depth_uncertainty: f32,
    /// Last update timestamp
    pub last_update: f64,
}

/// Temporal depth fusion for robust depth estimation
#[derive(Debug)]
pub struct TemporalDepthFusion {
    /// Map from feature position to temporal depth info
    depth_tracks: HashMap<(i32, i32), TemporalDepthInfo>,
    /// Maximum number of frames to keep in history
    max_history_frames: usize,
    /// Position tolerance for track association (pixels)
    position_tolerance: f32,
    /// Current timestamp
    current_timestamp: f64,
}

impl TemporalDepthFusion {
    /// Create new temporal depth fusion
    pub fn new(max_history_frames: usize, position_tolerance: f32) -> Self {
        Self {
            depth_tracks: HashMap::new(),
            max_history_frames,
            position_tolerance,
            current_timestamp: 0.0,
        }
    }

    /// Update with new stereo matches
    pub fn update(
        &mut self,
        matches: &[StereoMatch],
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        baseline: f32,
        focal_length: f32,
        timestamp: f64,
    ) {
        self.current_timestamp = timestamp;

        for match_ in matches {
            let left_point = left_features[match_.left_idx].point;

            // Compute disparity and depth
            let right_point = right_features[match_.right_idx].point;
            let disparity = (left_point.x - right_point.x).abs();

            if disparity > 1.0 {
                let depth = baseline * focal_length / disparity;

                // Find or create track
                let key = self.position_to_key(left_point);
                let track = self
                    .depth_tracks
                    .entry(key)
                    .or_insert_with(|| TemporalDepthInfo {
                        position: left_point,
                        depth_measurements: Vec::new(),
                        confidence_values: Vec::new(),
                        filtered_depth: depth,
                        depth_uncertainty: 1.0,
                        last_update: timestamp,
                    });

                // Add measurement
                track.depth_measurements.push(depth);
                track.confidence_values.push(match_.confidence);
                track.last_update = timestamp;

                // Maintain history size
                if track.depth_measurements.len() > self.max_history_frames {
                    track.depth_measurements.remove(0);
                    track.confidence_values.remove(0);
                }

                // Update filtered estimate inline
                if !track.depth_measurements.is_empty() {
                    // Weight by confidence and recency
                    let mut weighted_sum = 0.0;
                    let mut total_weight = 0.0;
                    let mut variances = Vec::new();

                    for (i, (&depth_val, &confidence)) in track
                        .depth_measurements
                        .iter()
                        .zip(&track.confidence_values)
                        .enumerate()
                    {
                        // Recency weight (newer measurements have higher weight)
                        let recency_weight = (i + 1) as f32 / track.depth_measurements.len() as f32;
                        let weight = confidence * recency_weight;

                        weighted_sum += depth_val * weight;
                        total_weight += weight;
                        variances.push(depth_val);
                    }

                    if total_weight > 0.0 {
                        track.filtered_depth = weighted_sum / total_weight;

                        // Compute uncertainty as standard deviation
                        let mean = track.filtered_depth;
                        let variance = variances.iter().map(|&d| (d - mean).powi(2)).sum::<f32>()
                            / variances.len() as f32;
                        track.depth_uncertainty = variance.sqrt();
                    }
                }
            }
        }

        // Remove old tracks
        self.depth_tracks.retain(|_, track| {
            timestamp - track.last_update < 1.0 // Keep tracks updated within 1 second
        });
    }

    /// Get filtered depth for a position
    pub fn get_filtered_depth(&self, position: na::Vector2<f32>) -> Option<f32> {
        let key = self.position_to_key(position);
        self.depth_tracks
            .get(&key)
            .map(|track| track.filtered_depth)
    }

    /// Get depth uncertainty for a position
    pub fn get_depth_uncertainty(&self, position: na::Vector2<f32>) -> Option<f32> {
        let key = self.position_to_key(position);
        self.depth_tracks
            .get(&key)
            .map(|track| track.depth_uncertainty)
    }

    /// Convert position to discrete key for hashing
    fn position_to_key(&self, position: na::Vector2<f32>) -> (i32, i32) {
        let scale = 1.0 / self.position_tolerance;
        ((position.x * scale) as i32, (position.y * scale) as i32)
    }
}

/// Configuration for stereo matching
#[derive(Debug, Clone)]
pub struct StereoMatcherConfig {
    /// Maximum Hamming distance for descriptor matching
    pub max_descriptor_distance: u32,
    /// Maximum epipolar error in pixels
    pub max_epipolar_error: f32,
    /// Lowe's ratio test threshold
    pub ratio_threshold: f32,
    /// Enable geometric verification
    pub enable_geometric_check: bool,
    /// RANSAC iterations for fundamental matrix estimation
    pub ransac_iterations: usize,
    /// RANSAC inlier threshold
    pub ransac_threshold: f32,
    /// Enable hierarchical matching (coarse-to-fine)
    pub enable_hierarchical_matching: bool,
    /// Pyramid levels for hierarchical matching
    pub pyramid_levels: usize,
}

impl Default for StereoMatcherConfig {
    fn default() -> Self {
        Self {
            max_descriptor_distance: 64,
            max_epipolar_error: 2.0,
            ratio_threshold: 0.8,
            enable_geometric_check: true,
            ransac_iterations: 1000,
            ransac_threshold: 1.0,
            enable_hierarchical_matching: true,
            pyramid_levels: 3,
        }
    }
}

/// Stereo feature matcher with descriptor-based matching
pub struct StereoMatcher {
    config: StereoMatcherConfig,
}

impl StereoMatcher {
    /// Create new stereo matcher
    pub const fn new(config: StereoMatcherConfig) -> Self {
        Self { config }
    }

    /// Match features between left and right images
    pub fn match_features(
        &self,
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        camera_intrinsics: &na::Matrix3<f64>,
    ) -> Vec<StereoMatch> {
        if self.config.enable_hierarchical_matching {
            self.hierarchical_matching(left_features, right_features, camera_intrinsics)
        } else {
            // Fallback to standard matching
            let mut candidate_matches = self.descriptor_matching(left_features, right_features);

            if candidate_matches.is_empty() {
                return Vec::new();
            }

            // Advanced epipolar geometry verification with M-estimators
            if self.config.enable_geometric_check {
                candidate_matches = self.geometric_verification_robust(
                    &candidate_matches,
                    left_features,
                    right_features,
                    camera_intrinsics,
                );
            }

            candidate_matches
        }
    }

    /// Hierarchical coarse-to-fine matching for improved accuracy and speed
    fn hierarchical_matching(
        &self,
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        camera_intrinsics: &na::Matrix3<f64>,
    ) -> Vec<StereoMatch> {
        let mut all_matches = Vec::new();

        // Start from coarsest level and refine
        for level in (0..self.config.pyramid_levels).rev() {
            let scale = 0.5f32.powi(level as i32);

            // Scale features for this level
            let scaled_left = self.scale_features(left_features, scale);
            let scaled_right = self.scale_features(right_features, scale);

            // Match at this level
            let level_matches =
                self.match_at_level(&scaled_left, &scaled_right, camera_intrinsics, scale);

            // If not the finest level, use matches to guide next level
            if level > 0 {
                // Propagate matches to guide finer level
                all_matches =
                    self.propagate_matches_to_next_level(&level_matches, &all_matches, scale);
            } else {
                // Finest level - collect final matches
                all_matches.extend(level_matches);
            }
        }

        // Final geometric verification on all matches
        if self.config.enable_geometric_check {
            all_matches = self.geometric_verification(
                &all_matches,
                left_features,
                right_features,
                camera_intrinsics,
            );
        }

        all_matches
    }

    /// Match features at a specific pyramid level
    fn match_at_level(
        &self,
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        camera_intrinsics: &na::Matrix3<f64>,
        scale: f32,
    ) -> Vec<StereoMatch> {
        // Relax constraints at coarser levels
        let level_max_distance =
            (self.config.max_descriptor_distance as f32 * (2.0 - scale)) as u32;
        let level_max_error = self.config.max_epipolar_error * (2.0 - scale);

        let mut candidate_matches =
            self.descriptor_matching_with_params(left_features, right_features, level_max_distance);

        if candidate_matches.is_empty() {
            return Vec::new();
        }

        // Scale intrinsics for this level
        let scaled_intrinsics = camera_intrinsics * scale as f64;

        // Epipolar geometry verification with relaxed constraints
        if self.config.enable_geometric_check {
            candidate_matches = self.geometric_verification_with_params(
                &candidate_matches,
                left_features,
                right_features,
                &scaled_intrinsics,
                level_max_error,
            );
        }

        candidate_matches
    }

    /// Scale features for pyramid level
    fn scale_features(&self, features: &[EnhancedFeature], scale: f32) -> Vec<EnhancedFeature> {
        features
            .iter()
            .map(|f| {
                let mut scaled = f.clone();
                scaled.point = f.point * scale;
                scaled
            })
            .collect()
    }

    /// Propagate matches from coarse to fine level
    fn propagate_matches_to_next_level(
        &self,
        coarse_matches: &[StereoMatch],
        _previous_matches: &[StereoMatch],
        scale: f32,
    ) -> Vec<StereoMatch> {
        // Use coarse matches to predict search regions for fine level
        // For now, just return coarse matches scaled up
        coarse_matches
            .iter()
            .map(|m| {
                let mut scaled = m.clone();
                scaled.left_idx = (m.left_idx as f32 / scale) as usize;
                scaled.right_idx = (m.right_idx as f32 / scale) as usize;
                scaled
            })
            .collect()
    }

    /// Descriptor matching with custom parameters
    fn descriptor_matching_with_params(
        &self,
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        max_distance: u32,
    ) -> Vec<StereoMatch> {
        let mut matches = Vec::new();

        for (left_idx, left_feat) in left_features.iter().enumerate() {
            let mut best_match = None;
            let mut best_distance = u32::MAX;
            let mut second_best_distance = u32::MAX;

            // Find best and second best matches in right image
            for (right_idx, right_feat) in right_features.iter().enumerate() {
                let dist = hamming_distance(&left_feat.descriptor, &right_feat.descriptor);

                if dist < best_distance {
                    second_best_distance = best_distance;
                    best_distance = dist;
                    best_match = Some(right_idx);
                } else if dist < second_best_distance {
                    second_best_distance = dist;
                }
            }

            // Lowe's ratio test
            if let Some(right_idx) = best_match {
                if best_distance < max_distance
                    && second_best_distance > 0
                    && (best_distance as f32) / (second_best_distance as f32)
                        < self.config.ratio_threshold
                {
                    // Compute initial confidence based on descriptor matching
                    let confidence = self.compute_match_confidence(
                        best_distance,
                        0.0, // epipolar error not computed yet
                        max_distance,
                        self.config.max_epipolar_error,
                    );

                    matches.push(StereoMatch {
                        left_idx,
                        right_idx,
                        score: best_distance,
                        epipolar_error: 0.0, // Will be computed later
                        confidence,
                    });
                }
            }
        }

        matches
    }

    /// Geometric verification with custom parameters
    fn geometric_verification_with_params(
        &self,
        candidate_matches: &[StereoMatch],
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        camera_intrinsics: &na::Matrix3<f64>,
        max_epipolar_error: f32,
    ) -> Vec<StereoMatch> {
        if candidate_matches.len() < 8 {
            return candidate_matches.to_vec();
        }

        // Estimate fundamental matrix using RANSAC
        let fundamental_matrix = self.estimate_fundamental_matrix_ransac(
            candidate_matches,
            left_features,
            right_features,
            camera_intrinsics,
        );

        // Filter matches by epipolar constraint
        candidate_matches
            .iter()
            .filter_map(|m| {
                let left_point = left_features[m.left_idx].point;
                let right_point = right_features[m.right_idx].point;

                let epipolar_error = self.compute_epipolar_error(
                    &fundamental_matrix,
                    &na::Vector2::new(left_point.x as f64, left_point.y as f64),
                    &na::Vector2::new(right_point.x as f64, right_point.y as f64),
                );

                if epipolar_error < max_epipolar_error {
                    // Update confidence with epipolar error information
                    let confidence = self.compute_match_confidence(
                        m.score,
                        epipolar_error,
                        self.config.max_descriptor_distance,
                        self.config.max_epipolar_error,
                    );

                    Some(StereoMatch {
                        left_idx: m.left_idx,
                        right_idx: m.right_idx,
                        score: m.score,
                        epipolar_error,
                        confidence,
                    })
                } else {
                    None
                }
            })
            .collect()
    }

    /// Robust geometric verification using M-estimators
    fn geometric_verification_robust(
        &self,
        candidate_matches: &[StereoMatch],
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        camera_intrinsics: &na::Matrix3<f64>,
    ) -> Vec<StereoMatch> {
        if candidate_matches.len() < 8 {
            return candidate_matches.to_vec();
        }

        // Estimate fundamental matrix using robust M-estimator
        let fundamental_matrix = self.robust_fundamental_matrix_estimation(
            candidate_matches,
            left_features,
            right_features,
            camera_intrinsics,
        );

        // Filter matches by epipolar constraint with adaptive threshold
        let mut verified_matches = Vec::new();
        let mut errors = Vec::new();

        for match_ in candidate_matches {
            let left_point = left_features[match_.left_idx].point;
            let right_point = right_features[match_.right_idx].point;

            let epipolar_error = self.compute_epipolar_error(
                &fundamental_matrix,
                &na::Vector2::new(left_point.x as f64, left_point.y as f64),
                &na::Vector2::new(right_point.x as f64, right_point.y as f64),
            );

            errors.push(epipolar_error);
        }

        // Adaptive threshold based on median error
        errors.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median_error = errors[errors.len() / 2];
        let adaptive_threshold = (median_error * 2.0).max(self.config.max_epipolar_error);

        for (i, match_) in candidate_matches.iter().enumerate() {
            if errors[i] <= adaptive_threshold {
                // Update confidence with refined epipolar error
                let confidence = self.compute_match_confidence(
                    match_.score,
                    errors[i],
                    self.config.max_descriptor_distance,
                    self.config.max_epipolar_error,
                );

                verified_matches.push(StereoMatch {
                    left_idx: match_.left_idx,
                    right_idx: match_.right_idx,
                    score: match_.score,
                    epipolar_error: errors[i],
                    confidence,
                });
            }
        }

        verified_matches
    }

    /// Initial descriptor-based matching
    fn descriptor_matching(
        &self,
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
    ) -> Vec<StereoMatch> {
        let mut matches = Vec::new();

        for (left_idx, left_feat) in left_features.iter().enumerate() {
            let mut best_match = None;
            let mut best_distance = u32::MAX;
            let mut second_best_distance = u32::MAX;

            // Find best and second best matches in right image
            for (right_idx, right_feat) in right_features.iter().enumerate() {
                let dist = hamming_distance(&left_feat.descriptor, &right_feat.descriptor);

                if dist < best_distance {
                    second_best_distance = best_distance;
                    best_distance = dist;
                    best_match = Some(right_idx);
                } else if dist < second_best_distance {
                    second_best_distance = dist;
                }
            }

            // Apply Lowe's ratio test
            if let Some(right_idx) = best_match {
                if best_distance <= self.config.max_descriptor_distance
                    && second_best_distance > 0
                    && (best_distance as f32) / (second_best_distance as f32)
                        <= self.config.ratio_threshold
                {
                    // Compute initial confidence based on descriptor matching
                    let confidence = self.compute_match_confidence(
                        best_distance,
                        0.0, // epipolar error not computed yet
                        self.config.max_descriptor_distance,
                        self.config.max_epipolar_error,
                    );

                    matches.push(StereoMatch {
                        left_idx,
                        right_idx,
                        score: best_distance,
                        epipolar_error: 0.0, // Will be computed later
                        confidence,
                    });
                }
            }
        }

        matches
    }

    /// Geometric verification using epipolar constraints
    fn geometric_verification(
        &self,
        candidate_matches: &[StereoMatch],
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        camera_intrinsics: &na::Matrix3<f64>,
    ) -> Vec<StereoMatch> {
        if candidate_matches.len() < 8 {
            // Not enough points for robust estimation
            return candidate_matches.to_vec();
        }

        // Estimate fundamental matrix using RANSAC
        let fundamental_matrix = self.estimate_fundamental_matrix_ransac(
            candidate_matches,
            left_features,
            right_features,
            camera_intrinsics,
        );

        // Filter matches using epipolar constraint
        candidate_matches
            .iter()
            .filter_map(|match_| {
                let left_point = left_features[match_.left_idx].point;
                let right_point = right_features[match_.right_idx].point;

                let epipolar_error = self.compute_epipolar_error(
                    &fundamental_matrix,
                    &na::Vector2::new(left_point.x as f64, left_point.y as f64),
                    &na::Vector2::new(right_point.x as f64, right_point.y as f64),
                );

                if epipolar_error <= self.config.max_epipolar_error {
                    // Update confidence with final epipolar error
                    let confidence = self.compute_match_confidence(
                        match_.score,
                        epipolar_error,
                        self.config.max_descriptor_distance,
                        self.config.max_epipolar_error,
                    );

                    Some(StereoMatch {
                        left_idx: match_.left_idx,
                        right_idx: match_.right_idx,
                        score: match_.score,
                        epipolar_error,
                        confidence,
                    })
                } else {
                    None
                }
            })
            .collect()
    }

    /// Estimate fundamental matrix using RANSAC
    fn estimate_fundamental_matrix_ransac(
        &self,
        matches: &[StereoMatch],
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        camera_intrinsics: &na::Matrix3<f64>,
    ) -> na::Matrix3<f64> {
        let mut best_f = na::Matrix3::zeros();
        let mut best_inliers = 0;

        let k_inv = camera_intrinsics.try_inverse().unwrap_or_else(|| {
            // Fallback for singular matrix - use identity (should not happen with valid intrinsics)
            na::Matrix3::identity()
        });

        for _ in 0..self.config.ransac_iterations {
            // Randomly sample 8 matches
            let sample_indices = self.random_sample(matches.len(), 8);
            let sample_matches: Vec<_> = sample_indices.iter().map(|&idx| &matches[idx]).collect();

            if let Some(f) = self.estimate_fundamental_matrix_8point(
                &sample_matches,
                left_features,
                right_features,
                &k_inv,
            ) {
                // Count inliers
                let inliers = matches
                    .iter()
                    .filter(|match_| {
                        let left_point = left_features[match_.left_idx].point;
                        let right_point = right_features[match_.right_idx].point;

                        let error = self.compute_epipolar_error(
                            &f,
                            &na::Vector2::new(left_point.x as f64, left_point.y as f64),
                            &na::Vector2::new(right_point.x as f64, right_point.y as f64),
                        );

                        error <= self.config.ransac_threshold
                    })
                    .count();

                if inliers > best_inliers {
                    best_inliers = inliers;
                    best_f = f;
                }
            }
        }

        best_f
    }

    /// Advanced outlier rejection using M-estimators (more robust than RANSAC)
    fn robust_fundamental_matrix_estimation(
        &self,
        matches: &[StereoMatch],
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        camera_intrinsics: &na::Matrix3<f64>,
    ) -> na::Matrix3<f64> {
        let k_inv = camera_intrinsics.try_inverse().unwrap_or_else(|| {
            // Fallback for singular matrix - use identity (should not happen with valid intrinsics)
            na::Matrix3::identity()
        });

        // Initial estimate using RANSAC
        let mut f_matrix = self.estimate_fundamental_matrix_ransac(
            matches,
            left_features,
            right_features,
            camera_intrinsics,
        );

        // Refine using M-estimator (iteratively reweighted least squares)
        for _ in 0..5 {
            // Few iterations usually sufficient
            let (refined_f, weights) = self.m_estimator_refinement(
                &f_matrix,
                matches,
                left_features,
                right_features,
                &k_inv,
            );
            f_matrix = refined_f;

            // Early termination if weights are stable
            let weight_sum: f64 = weights.iter().sum();
            let avg_weight = weight_sum / weights.len() as f64;
            if avg_weight > 0.8 {
                // Most points are inliers
                break;
            }
        }

        f_matrix
    }

    /// M-estimator refinement using Huber loss
    fn m_estimator_refinement(
        &self,
        initial_f: &na::Matrix3<f64>,
        matches: &[StereoMatch],
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        k_inv: &na::Matrix3<f64>,
    ) -> (na::Matrix3<f64>, Vec<f64>) {
        let mut weights = Vec::with_capacity(matches.len());

        // Compute residuals and weights
        for match_ in matches {
            let left_point = left_features[match_.left_idx].point;
            let right_point = right_features[match_.right_idx].point;

            let residual = self.compute_epipolar_error(
                initial_f,
                &na::Vector2::new(left_point.x as f64, left_point.y as f64),
                &na::Vector2::new(right_point.x as f64, right_point.y as f64),
            );

            // Huber loss weight
            let k = 1.345_f64; // Tuning parameter for Huber loss
            let weight = if (residual as f64) <= k {
                1.0_f64
            } else {
                k / residual as f64
            };

            weights.push(weight);
        }

        // Weighted least squares refinement
        self.weighted_fundamental_matrix_estimation(
            matches,
            left_features,
            right_features,
            &weights,
            k_inv,
        )
    }

    /// Weighted fundamental matrix estimation
    fn weighted_fundamental_matrix_estimation(
        &self,
        matches: &[StereoMatch],
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        weights: &[f64],
        k_inv: &na::Matrix3<f64>,
    ) -> (na::Matrix3<f64>, Vec<f64>) {
        // Build weighted A matrix
        let mut a_matrix = Vec::new();

        for (i, match_) in matches.iter().enumerate() {
            let weight = weights[i].sqrt();
            let left_point = left_features[match_.left_idx].point;
            let right_point = right_features[match_.right_idx].point;

            let xl = k_inv * na::Vector3::new(left_point.x as f64, left_point.y as f64, 1.0);
            let xr = k_inv * na::Vector3::new(right_point.x as f64, right_point.y as f64, 1.0);

            let row = [
                weight * xl.x * xr.x,
                weight * xl.x * xr.y,
                weight * xl.x,
                weight * xl.y * xr.x,
                weight * xl.y * xr.y,
                weight * xl.y,
                weight * xr.x,
                weight * xr.y,
                weight,
            ];

            a_matrix.extend_from_slice(&row);
        }

        // Solve using SVD
        let a_na = na::DMatrix::from_row_slice(matches.len(), 9, &a_matrix);
        let svd = a_na.svd(true, true);
        let v_t = svd.v_t.unwrap_or_else(|| {
            // Fallback - should not happen with valid input
            na::DMatrix::identity(9, 9)
        });
        let f_vec = v_t.row(8);

        let f_matrix = na::Matrix3::new(
            f_vec[0], f_vec[1], f_vec[2], f_vec[3], f_vec[4], f_vec[5], f_vec[6], f_vec[7],
            f_vec[8],
        );

        // Enforce rank-2 constraint
        let svd_f = f_matrix.svd(true, true);
        let mut sigma = svd_f.singular_values;
        sigma[2] = 0.0; // Set smallest singular value to zero

        let u = svd_f.u.unwrap_or_else(|| {
            // Fallback - should not happen with valid matrix
            na::Matrix3::identity()
        });
        let v_t = svd_f.v_t.unwrap_or_else(|| {
            // Fallback - should not happen with valid matrix
            na::Matrix3::identity()
        });

        let f_refined = u * na::Matrix3::from_diagonal(&sigma) * v_t;

        (f_refined, weights.to_vec())
    }

    /// Estimate fundamental matrix from 8 point correspondences
    fn estimate_fundamental_matrix_8point(
        &self,
        matches: &[&StereoMatch],
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        k_inv: &na::Matrix3<f64>,
    ) -> Option<na::Matrix3<f64>> {
        if matches.len() != 8 {
            return None;
        }

        // Build the A matrix for Af = 0
        let mut a = na::DMatrix::<f64>::zeros(8, 9);

        for (i, match_) in matches.iter().enumerate() {
            let left_point = left_features[match_.left_idx].point;
            let right_point = right_features[match_.right_idx].point;

            // Normalize points
            let left_norm = k_inv * na::Vector3::new(left_point.x as f64, left_point.y as f64, 1.0);
            let right_norm =
                k_inv * na::Vector3::new(right_point.x as f64, right_point.y as f64, 1.0);

            let xl = left_norm.x / left_norm.z;
            let yl = left_norm.y / left_norm.z;
            let xr = right_norm.x / right_norm.z;
            let yr = right_norm.y / right_norm.z;

            // Epipolar constraint: xl*xr + yl*yr + ... = 0
            a[(i, 0)] = xl * xr;
            a[(i, 1)] = xl * yr;
            a[(i, 2)] = xl;
            a[(i, 3)] = yl * xr;
            a[(i, 4)] = yl * yr;
            a[(i, 5)] = yl;
            a[(i, 6)] = xr;
            a[(i, 7)] = yr;
            a[(i, 8)] = 1.0;
        }

        // Solve Af = 0 using SVD
        let svd = a.svd(true, true);
        let v_t = svd.v_t.unwrap_or_else(|| {
            // Fallback - should not happen with valid input
            na::DMatrix::identity(9, 9)
        });
        let f_vec = v_t.row(8).transpose(); // Last column of V

        // Reshape to 3x3 matrix
        let mut f = na::Matrix3::zeros();
        f[(0, 0)] = f_vec[0];
        f[(0, 1)] = f_vec[1];
        f[(0, 2)] = f_vec[2];
        f[(1, 0)] = f_vec[3];
        f[(1, 1)] = f_vec[4];
        f[(1, 2)] = f_vec[5];
        f[(2, 0)] = f_vec[6];
        f[(2, 1)] = f_vec[7];
        f[(2, 2)] = f_vec[8];

        // Enforce rank 2 constraint
        let svd_f = f.svd(true, true);
        let mut sigma = svd_f.singular_values;
        sigma[2] = 0.0; // Set smallest singular value to 0

        let u = svd_f.u.unwrap_or_else(|| {
            // Fallback - should not happen with valid matrix
            na::Matrix3::identity()
        });
        let v_t = svd_f.v_t.unwrap_or_else(|| {
            // Fallback - should not happen with valid matrix
            na::Matrix3::identity()
        });

        // Reconstruct F
        let sigma_diag = na::Matrix3::from_diagonal(&sigma);
        Some(u * sigma_diag * v_t)
    }

    /// Compute epipolar error for a point pair
    fn compute_epipolar_error(
        &self,
        f: &na::Matrix3<f64>,
        left_point: &na::Vector2<f64>,
        right_point: &na::Vector2<f64>,
    ) -> f32 {
        // Convert to homogeneous coordinates
        let left_homogeneous = na::Vector3::new(left_point.x, left_point.y, 1.0);
        let right_homogeneous = na::Vector3::new(right_point.x, right_point.y, 1.0);

        // Compute epipolar line: l = F * x_right
        let epipolar_line = f * right_homogeneous;

        // Compute distance: |x_left^T * l| / sqrt(l1^2 + l2^2)
        let numerator = (left_homogeneous.transpose() * epipolar_line).abs();
        let denominator = (epipolar_line.x.powi(2) + epipolar_line.y.powi(2)).sqrt();

        if denominator > 1e-8 {
            (numerator[0] / denominator) as f32
        } else {
            f32::INFINITY
        }
    }

    /// Compute confidence score for a stereo match (0-1, higher is better)
    pub fn compute_match_confidence(
        &self,
        score: u32,
        epipolar_error: f32,
        max_score: u32,
        max_error: f32,
    ) -> f32 {
        // Normalize descriptor score (lower is better)
        let score_confidence = if max_score > 0 {
            1.0 - (score as f32 / max_score as f32).min(1.0)
        } else {
            1.0
        };

        // Normalize epipolar error (lower is better)
        let error_confidence = if max_error > 0.0 {
            1.0 - (epipolar_error / max_error).min(1.0)
        } else {
            1.0
        };

        // Combine confidences with weights
        let score_weight = 0.6;
        let error_weight = 0.4;

        score_confidence * score_weight + error_confidence * error_weight
    }

    /// Refine stereo matches to sub-pixel accuracy using interpolation
    pub fn refine_matches_subpixel(
        &self,
        matches: &[StereoMatch],
        left_features: &[EnhancedFeature],
        right_features: &[EnhancedFeature],
        left_image: &image::GrayImage,
        right_image: &image::GrayImage,
    ) -> Vec<StereoMatch> {
        let mut refined_matches = Vec::with_capacity(matches.len());

        for match_ in matches {
            let left_point = left_features[match_.left_idx].point;
            let right_point = right_features[match_.right_idx].point;

            // Refine right point to sub-pixel accuracy
            let _refined_right_point =
                self.refine_point_subpixel(left_point, right_point, left_image, right_image);

            // Create refined match with updated point
            let refined_match = match_.clone();
            // Update the right feature point (we'd need to modify the feature or store refined point separately)
            // For now, we'll just update the epipolar error computation

            refined_matches.push(refined_match);
        }

        refined_matches
    }

    /// Refine a single point to sub-pixel accuracy using correlation
    fn refine_point_subpixel(
        &self,
        left_point: na::Vector2<f32>,
        right_point: na::Vector2<f32>,
        left_image: &image::GrayImage,
        right_image: &image::GrayImage,
    ) -> na::Vector2<f32> {
        let patch_size = 5; // 5x5 patch for correlation
        let search_range = 2; // Search ±2 pixels around initial match

        let left_x = left_point.x as i32;
        let left_y = left_point.y as i32;
        let right_x = right_point.x as i32;
        let right_y = right_point.y as i32;

        // Extract left patch
        let left_patch = self.extract_patch(left_image, left_x, left_y, patch_size);

        let mut best_corr = -1.0;
        let mut best_dx = 0.0;

        // Search for best sub-pixel match
        for dx in (-search_range..=search_range).map(|x| x as f32) {
            let test_x = right_x as f32 + dx;
            let test_y = right_y as f32; // Assume horizontal epipolar lines

            // Extract right patch at sub-pixel position using bilinear interpolation
            let right_patch =
                self.extract_patch_interpolated(right_image, test_x, test_y, patch_size);

            // Compute normalized cross-correlation
            let corr = self.compute_patch_correlation(&left_patch, &right_patch);

            if corr > best_corr {
                best_corr = corr;
                best_dx = dx;
            }
        }

        // Return refined point
        na::Vector2::new(right_point.x + best_dx, right_point.y)
    }

    /// Extract image patch around a point
    fn extract_patch(&self, image: &image::GrayImage, x: i32, y: i32, size: i32) -> Vec<f32> {
        let half_size = size / 2;
        let mut patch = Vec::with_capacity((size * size) as usize);

        for dy in -half_size..=half_size {
            for dx in -half_size..=half_size {
                let px = (x + dx).clamp(0, image.width() as i32 - 1) as u32;
                let py = (y + dy).clamp(0, image.height() as i32 - 1) as u32;
                patch.push(image.get_pixel(px, py)[0] as f32);
            }
        }

        patch
    }

    /// Extract image patch with bilinear interpolation
    fn extract_patch_interpolated(
        &self,
        image: &image::GrayImage,
        x: f32,
        y: f32,
        size: i32,
    ) -> Vec<f32> {
        let half_size = size / 2;
        let mut patch = Vec::with_capacity((size * size) as usize);

        for dy in -half_size..=half_size {
            for dx in -half_size..=half_size {
                let px = x + dx as f32;
                let py = y + dy as f32;
                let value = self.bilinear_interpolate(image, px, py);
                patch.push(value);
            }
        }

        patch
    }

    /// Bilinear interpolation at sub-pixel position
    fn bilinear_interpolate(&self, image: &image::GrayImage, x: f32, y: f32) -> f32 {
        let x0 = x.floor() as i32;
        let y0 = y.floor() as i32;
        let x1 = x0 + 1;
        let y1 = y0 + 1;

        let x0 = x0.clamp(0, image.width() as i32 - 1) as u32;
        let y0 = y0.clamp(0, image.height() as i32 - 1) as u32;
        let x1 = x1.clamp(0, image.width() as i32 - 1) as u32;
        let y1 = y1.clamp(0, image.height() as i32 - 1) as u32;

        let q00 = image.get_pixel(x0, y0)[0] as f32;
        let q01 = image.get_pixel(x0, y1)[0] as f32;
        let q10 = image.get_pixel(x1, y0)[0] as f32;
        let q11 = image.get_pixel(x1, y1)[0] as f32;

        let dx = x - x0 as f32;
        let dy = y - y0 as f32;

        // Bilinear interpolation
        q00 * (1.0 - dx) * (1.0 - dy)
            + q10 * dx * (1.0 - dy)
            + q01 * (1.0 - dx) * dy
            + q11 * dx * dy
    }

    /// Compute normalized cross-correlation between two patches
    fn compute_patch_correlation(&self, patch1: &[f32], patch2: &[f32]) -> f32 {
        if patch1.len() != patch2.len() {
            return 0.0;
        }

        let n = patch1.len() as f32;

        // Compute means
        let mean1 = patch1.iter().sum::<f32>() / n;
        let mean2 = patch2.iter().sum::<f32>() / n;

        // Compute correlation
        let mut numerator = 0.0;
        let mut sum_sq1 = 0.0;
        let mut sum_sq2 = 0.0;

        for (&p1, &p2) in patch1.iter().zip(patch2.iter()) {
            let diff1 = p1 - mean1;
            let diff2 = p2 - mean2;
            numerator += diff1 * diff2;
            sum_sq1 += diff1 * diff1;
            sum_sq2 += diff2 * diff2;
        }

        if sum_sq1 == 0.0 || sum_sq2 == 0.0 {
            0.0
        } else {
            numerator / (sum_sq1 * sum_sq2).sqrt()
        }
    }

    /// Apply bilateral filtering to depth map for noise reduction
    pub fn filter_depth_map_bilateral(
        &self,
        depth_map: &mut [f32],
        width: usize,
        height: usize,
        spatial_sigma: f32,
        depth_sigma: f32,
    ) {
        let mut filtered = vec![0.0; depth_map.len()];

        for y in 0..height {
            for x in 0..width {
                let idx = y * width + x;
                let center_depth = depth_map[idx];

                if center_depth <= 0.0 {
                    filtered[idx] = center_depth;
                    continue;
                }

                let mut sum_weights = 0.0;
                let mut sum_weighted_depth = 0.0;

                // Apply bilateral filter in local window
                let window_size = (spatial_sigma * 3.0) as i32;

                for dy in -window_size..=window_size {
                    for dx in -window_size..=window_size {
                        let nx = x as i32 + dx;
                        let ny = y as i32 + dy;

                        if nx >= 0 && nx < width as i32 && ny >= 0 && ny < height as i32 {
                            let nidx = (ny as usize) * width + (nx as usize);
                            let neighbor_depth = depth_map[nidx];

                            if neighbor_depth > 0.0 {
                                // Spatial weight (Gaussian)
                                let spatial_dist = (dx * dx + dy * dy) as f32;
                                let spatial_weight =
                                    (-spatial_dist / (2.0 * spatial_sigma * spatial_sigma)).exp();

                                // Depth weight (Gaussian)
                                let depth_dist = (center_depth - neighbor_depth).abs();
                                let depth_weight =
                                    (-depth_dist / (2.0 * depth_sigma * depth_sigma)).exp();

                                let weight = spatial_weight * depth_weight;
                                sum_weights += weight;
                                sum_weighted_depth += neighbor_depth * weight;
                            }
                        }
                    }
                }

                filtered[idx] = if sum_weights > 0.0 {
                    sum_weighted_depth / sum_weights
                } else {
                    center_depth
                };
            }
        }

        // Copy filtered result back
        depth_map.copy_from_slice(&filtered);
    }

    /// Remove depth outliers using statistical filtering
    pub fn remove_depth_outliers(
        &self,
        depth_map: &mut [f32],
        width: usize,
        height: usize,
        max_deviation_sigma: f32,
    ) {
        // Compute local statistics
        let mut local_means = vec![0.0; depth_map.len()];
        let mut local_stds = vec![0.0; depth_map.len()];

        let window_size = 3; // 3x3 window

        for y in 0..height {
            for x in 0..width {
                let idx = y * width + x;

                let mut valid_depths = Vec::new();

                // Collect valid depths in window
                for dy in -window_size..=window_size {
                    for dx in -window_size..=window_size {
                        let nx = x as i32 + dx;
                        let ny = y as i32 + dy;

                        if nx >= 0 && nx < width as i32 && ny >= 0 && ny < height as i32 {
                            let nidx = (ny as usize) * width + (nx as usize);
                            let depth = depth_map[nidx];
                            if depth > 0.0 {
                                valid_depths.push(depth);
                            }
                        }
                    }
                }

                if !valid_depths.is_empty() {
                    // Compute mean and std
                    let mean = valid_depths.iter().sum::<f32>() / valid_depths.len() as f32;
                    let variance = valid_depths
                        .iter()
                        .map(|&d| (d - mean).powi(2))
                        .sum::<f32>()
                        / valid_depths.len() as f32;
                    let std = variance.sqrt();

                    local_means[idx] = mean;
                    local_stds[idx] = std;
                }
            }
        }

        // Remove outliers
        for i in 0..depth_map.len() {
            let depth = depth_map[i];
            if depth > 0.0 && local_stds[i] > 0.0 {
                let deviation = (depth - local_means[i]).abs() / local_stds[i];
                if deviation > max_deviation_sigma {
                    depth_map[i] = 0.0; // Mark as invalid
                }
            }
        }
    }

    /// Fill holes in depth map using interpolation
    pub fn fill_depth_holes(&self, depth_map: &mut [f32], width: usize, height: usize) {
        let mut filled = depth_map.to_vec();

        for y in 0..height {
            for x in 0..width {
                let idx = y * width + x;

                if depth_map[idx] <= 0.0 {
                    // Find nearest valid depths
                    let mut nearest_depths = Vec::new();
                    let search_radius = 5;

                    for dy in -search_radius..=search_radius {
                        for dx in -search_radius..=search_radius {
                            let nx = x as i32 + dx;
                            let ny = y as i32 + dy;

                            if nx >= 0 && nx < width as i32 && ny >= 0 && ny < height as i32 {
                                let nidx = (ny as usize) * width + (nx as usize);
                                let depth = depth_map[nidx];
                                if depth > 0.0 {
                                    let dist = ((dx * dx + dy * dy) as f32).sqrt();
                                    nearest_depths.push((depth, dist));
                                }
                            }
                        }
                    }

                    if !nearest_depths.is_empty() {
                        // Inverse distance weighting
                        let mut weighted_sum = 0.0;
                        let mut total_weight = 0.0;

                        for (depth, dist) in nearest_depths {
                            let weight = 1.0 / (dist + 1.0); // Add 1 to avoid division by zero
                            weighted_sum += depth * weight;
                            total_weight += weight;
                        }

                        filled[idx] = weighted_sum / total_weight;
                    }
                }
            }
        }

        depth_map.copy_from_slice(&filled);
    }

    /// Randomly sample k indices from n
    fn random_sample(&self, n: usize, k: usize) -> Vec<usize> {
        use std::collections::HashSet;

        let mut rng = oorandom::Rand32::new(
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_else(|_| std::time::Duration::from_secs(0))
                .as_nanos() as u64,
        );

        let mut samples = HashSet::new();
        while samples.len() < k {
            let idx = (rng.rand_float() * n as f32) as usize;
            samples.insert(idx.min(n - 1));
        }

        samples.into_iter().collect()
    }
}

/// Memory-efficient stereo calibration data structure
#[derive(Debug, Clone)]
pub struct StereoCalibrationData {
    /// Left image features
    pub left_features: Vec<EnhancedFeature>,
    /// Right image features
    pub right_features: Vec<EnhancedFeature>,
    /// Stereo matches
    pub matches: Vec<StereoMatch>,
    /// Camera intrinsics used for matching
    pub intrinsics: na::Matrix3<f64>,
    /// Timestamp of this stereo pair
    pub timestamp: f64,
}

impl StereoCalibrationData {
    /// Create new stereo calibration data
    pub const fn new(
        left_features: Vec<EnhancedFeature>,
        right_features: Vec<EnhancedFeature>,
        matches: Vec<StereoMatch>,
        intrinsics: na::Matrix3<f64>,
        timestamp: f64,
    ) -> Self {
        Self {
            left_features,
            right_features,
            matches,
            intrinsics,
            timestamp,
        }
    }

    /// Get number of valid matches
    pub const fn num_matches(&self) -> usize {
        self.matches.len()
    }

    /// Get average epipolar error
    pub fn average_epipolar_error(&self) -> f32 {
        if self.matches.is_empty() {
            return 0.0;
        }

        self.matches.iter().map(|m| m.epipolar_error).sum::<f32>() / self.matches.len() as f32
    }

    /// Get match quality statistics
    pub fn match_quality_stats(&self) -> (f32, f32, f32) {
        if self.matches.is_empty() {
            return (0.0, 0.0, 0.0);
        }

        let scores: Vec<f32> = self.matches.iter().map(|m| m.score as f32).collect();
        let mean = scores.iter().sum::<f32>() / scores.len() as f32;
        let variance = scores.iter().map(|s| (s - mean).powi(2)).sum::<f32>() / scores.len() as f32;
        let std_dev = variance.sqrt();

        (
            mean,
            std_dev,
            scores.iter().fold(f32::INFINITY, |a, &b| a.min(b)),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stereo_matcher_creation() {
        let config = StereoMatcherConfig::default();
        let matcher = StereoMatcher::new(config);
        assert_eq!(matcher.config.max_descriptor_distance, 64);
    }

    #[test]
    fn test_epipolar_error_computation() {
        let matcher = StereoMatcher::new(StereoMatcherConfig::default());

        // Identity fundamental matrix (rectified case)
        let f = na::Matrix3::<f64>::identity();
        let left_point = na::Vector2::new(100.0, 100.0);
        let right_point = na::Vector2::new(100.0, 100.0);

        let error = matcher.compute_epipolar_error(&f, &left_point, &right_point);
        assert!(error >= 0.0);
    }

    #[test]
    fn test_stereo_calibration_data() {
        let features = vec![];
        let matches = vec![];
        let intrinsics = na::Matrix3::identity();

        let data = StereoCalibrationData::new(features.clone(), features, matches, intrinsics, 0.0);

        assert_eq!(data.num_matches(), 0);
        assert!((data.average_epipolar_error() - 0.0_f32).abs() < f32::EPSILON);
    }
}
