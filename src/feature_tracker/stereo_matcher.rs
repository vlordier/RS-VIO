//! Stereo feature matching with enhanced descriptors
//!
//! This module provides robust stereo correspondence using ORB descriptors
//! with geometric constraints and outlier rejection for calibration.

use crate::feature_tracker::enhanced_detector::{hamming_distance, EnhancedFeature};
use nalgebra as na;
use std::cell::Cell;

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
            enable_hierarchical_matching: false,
            pyramid_levels: 3,
        }
    }
}

/// Stereo feature matcher with descriptor-based matching
pub struct StereoMatcher {
    config: StereoMatcherConfig,
    /// RNG seed counter — incremented on each call for deterministic, unique seeds
    rng_counter: Cell<u64>,
}

impl StereoMatcher {
    /// Create new stereo matcher
    pub fn new(config: StereoMatcherConfig) -> Self {
        Self {
            config,
            rng_counter: Cell::new(42),
        }
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
            let mut candidate_matches = self.descriptor_matching_with_params(
                left_features,
                right_features,
                self.config.max_descriptor_distance,
            );

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
                all_matches = self.propagate_matches_to_next_level(level_matches, scale);
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

        // Scale intrinsics for this level (only fx, fy, cx, cy — K[2][2] must stay 1)
        let mut scaled_intrinsics = *camera_intrinsics;
        let s = scale as f64;
        scaled_intrinsics[(0, 0)] *= s; // fx
        scaled_intrinsics[(1, 1)] *= s; // fy
        scaled_intrinsics[(0, 2)] *= s; // cx
        scaled_intrinsics[(1, 2)] *= s; // cy

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
    /// Re-uses a single allocation and only mutates the point field
    fn scale_features(&self, features: &[EnhancedFeature], scale: f32) -> Vec<EnhancedFeature> {
        let mut scaled: Vec<EnhancedFeature> = features.to_vec();
        for f in &mut scaled {
            f.point *= scale;
        }
        scaled
    }

    /// Propagate matches from coarse to fine level (identity — indices don't change)
    #[inline]
    fn propagate_matches_to_next_level(
        &self,
        coarse_matches: Vec<StereoMatch>,
        _scale: f32,
    ) -> Vec<StereoMatch> {
        coarse_matches
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
        let mut errors: Vec<f32> = Vec::with_capacity(candidate_matches.len());

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
        // Use an index-based partial sort to avoid cloning the errors vec
        let median_error = {
            let mut indices: Vec<usize> = (0..errors.len()).collect();
            let mid = indices.len() / 2;
            indices.select_nth_unstable_by(mid, |&a, &b| {
                errors[a]
                    .partial_cmp(&errors[b])
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            errors[indices[mid]]
        };
        let adaptive_threshold = (median_error * 2.0).min(self.config.max_epipolar_error);

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
        let mut a_matrix = Vec::with_capacity(matches.len() * 9);

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

        // Convert Essential → Fundamental so error is in pixel space.
        let f_refined = k_inv.transpose() * f_refined * k_inv;

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
        let mut a = na::SMatrix::<f64, 8, 9>::zeros();

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

        // Solve Af = 0 via the normal equations: find the eigenvector of A^T A
        // corresponding to the smallest eigenvalue (= null space of A).
        // Uses only stack-allocated 9×9 SMatrix — no heap allocation per RANSAC iteration.
        let ata = a.transpose() * a; // 9×9 SMatrix
        let eig = ata.symmetric_eigen();
        let min_idx = eig.eigenvalues.imin();
        let f_vec = eig.eigenvectors.column(min_idx);

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
        // The 8-point algorithm with K-normalized points yields the Essential
        // matrix E.  Convert to Fundamental matrix F = K^{-T} E K^{-1} so that
        // downstream epipolar-error computation works in pixel coordinates.
        let e = u * sigma_diag * v_t;
        Some(k_inv.transpose() * e * k_inv)
    }

    /// Compute epipolar error for a point pair
    #[inline]
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
        // Use dot product to get a scalar directly (avoids intermediate 1×1 matrix)
        let numerator = left_homogeneous.dot(&epipolar_line).abs();
        let denominator = (epipolar_line.x.powi(2) + epipolar_line.y.powi(2)).sqrt();

        if denominator > 1e-8 {
            (numerator / denominator) as f32
        } else {
            f32::INFINITY
        }
    }

    /// Compute confidence score for a stereo match (0-1, higher is better)
    #[inline]
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

    /// Randomly sample k indices from n (k must be <= 8)
    fn random_sample(&self, n: usize, k: usize) -> [usize; 8] {
        debug_assert!(k <= 8, "random_sample only supports k <= 8");
        let mut samples = [0usize; 8];
        if n == 0 || k == 0 {
            return samples;
        }
        if k >= n {
            for (i, s) in samples.iter_mut().enumerate().take(n.min(8)) {
                *s = i;
            }
            return samples;
        }

        let seed = self.rng_counter.get();
        self.rng_counter.set(seed.wrapping_add(1));
        let mut rng = oorandom::Rand32::new(seed);

        let mut count = 0;
        while count < k {
            let idx = (rng.rand_float() * n as f32) as usize;
            let idx = idx.min(n - 1);
            if !samples[..count].contains(&idx) {
                samples[count] = idx;
                count += 1;
            }
        }

        samples
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
}
