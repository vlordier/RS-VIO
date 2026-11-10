//! Relocalization module for visual-inertial odometry.
//!
//! Provides robust recovery from tracking failures by matching the current frame
//! against a database of keyframes and solving PnP to recover the pose.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
//! │  Current Frame  │────▶│  Feature Detect  │────▶│  Descriptor DB  │
//! │  (tracking lost)│     │  (ORB features)  │     │  (keyframe bow) │
//! └─────────────────┘     └──────────────────┘     └─────────────────┘
//!                                │                        │
//!                                ▼                        ▼
//!                         ┌──────────────────────────────────┐
//!                         │     Place Recognition (DBoW2)    │
//!                         │  - Find candidate keyframes      │
//!                         │  - Score by visual similarity    │
//!                         └──────────────────────────────────┘
//!                                         │
//!                                         ▼
//!                                ┌─────────────────┐
//!                                │  Geometric Verify│
//!                                │  - Feature match │
//!                                │  - RANSAC PnP    │
//!                                └─────────────────┘
//!                                         │
//!                                         ▼
//!                                ┌─────────────────┐
//!                                │  Recovered Pose │
//!                                │  (if successful)│
//!                                └─────────────────┘
//! ```
//!
//! ## Key design decisions for Raspberry Pi 5
//!
//! 1. **ORB features**: Fast to detect and describe, NEON-optimizable
//! 2. **DBoW2-style vocabulary**: Efficient place recognition without exhaustive search
//! 3. **Lazy descriptor storage**: Only store descriptors for keyframes, not all frames
//! 4. **Early exit**: Stop searching once a good enough match is found
//!
//! ## Usage
//!
//! ```rust
//! let mut relocalizer = Relocalizer::new(config);
//!
//! // Add keyframes as they're created
//! relocalizer.add_keyframe(&keyframe);
//!
//! // When tracking fails, attempt relocalization
//! if let Some(pose) = relocalizer.relocalize(&current_frame) {
//!     // Resume tracking from recovered pose
//!     estimator.set_pose(pose);
//! } else {
//!     // Full system reset required
//!     estimator.reset();
//! }
//! ```

use crate::estimator::Frame;
use crate::types::Matrix4x4;
use rustc_hash::FxHashMap;

/// Type alias for relocalization result: (descriptors, coords, ids, bow)
type RelocResult = (
    Vec<OrbDescriptor>,
    Vec<[f32; 2]>,
    Vec<usize>,
    FxHashMap<u32, f32>,
);

/// Configuration for the relocalization module.
#[derive(Debug, Clone)]
pub struct RelocalizationConfig {
    /// Minimum number of inliers required for successful PnP.
    pub min_pnp_inliers: usize,
    /// RANSAC threshold for inlier test (pixels).
    pub ransac_threshold: f64,
    /// Minimum similarity score for place recognition (0.0-1.0).
    pub min_similarity_score: f64,
    /// Maximum number of candidate keyframes to consider.
    pub max_candidates: usize,
    /// Whether to use ORB features (true) or reuse existing features (false).
    pub use_orb_features: bool,
    /// Number of ORB features to detect for relocalization.
    pub orb_max_features: usize,
    /// Scale factor between pyramid levels for ORB.
    pub orb_scale_factor: f64,
    /// Number of pyramid levels for ORB.
    pub orb_n_levels: usize,
}

impl Default for RelocalizationConfig {
    fn default() -> Self {
        Self {
            min_pnp_inliers: 40,  // Stricter: only accept >50% inlier ratio (min 40 from ~80 features)
            ransac_threshold: 1.5,  // Stricter: tighter geometric reprojection tolerance
            min_similarity_score: 0.35,  // Stricter: only strong place recognition candidates
            max_candidates: 10,
            use_orb_features: true,
            orb_max_features: 500,
            orb_scale_factor: 1.2,
            orb_n_levels: 8,
        }
    }
}

/// ORB descriptor: 256-bit (32 bytes) binary descriptor.
pub type OrbDescriptor = [u8; 32];

/// A keyframe entry in the relocalization database.
#[derive(Debug, Clone)]
pub struct RelocKeyframe {
    /// Keyframe ID (matches the frame_id in the sliding window).
    pub kf_id: i32,
    /// Timestamp in nanoseconds.
    pub timestamp_ns: i64,
    /// World-to-body pose.
    pub T_W_B: Matrix4x4,
    /// ORB descriptors for this keyframe (left camera).
    pub descriptors: Vec<OrbDescriptor>,
    /// Undistorted feature coordinates (left camera).
    pub feature_coords: Vec<[f32; 2]>,
    /// Feature IDs (for matching with map points).
    pub feature_ids: Vec<usize>,
    /// Bag-of-words vector (sparse representation).
    pub bow_vector: FxHashMap<u32, f32>,
}

/// Result of a relocalization attempt.
#[derive(Debug, Clone)]
pub struct RelocalizationResult {
    /// Whether relocalization was successful.
    pub success: bool,
    /// Recovered world-to-body pose (if successful).
    pub T_W_B: Option<Matrix4x4>,
    /// Number of inliers in PnP.
    pub n_inliers: usize,
    /// Similarity score with matched keyframe.
    pub similarity_score: f64,
    /// Matched keyframe ID.
    pub matched_kf_id: Option<i32>,
    /// Error message (if failed).
    pub error_message: Option<String>,
}

/// Main relocalization module.
pub struct Relocalizer {
    /// Configuration.
    config: RelocalizationConfig,
    /// Database of keyframes for place recognition.
    keyframes: FxHashMap<i32, RelocKeyframe>,
    /// Inverted index: word_id → keyframe IDs.
    inverted_index: FxHashMap<u32, Vec<i32>>,
    /// Keyframe count.
    n_keyframes: usize,
}

impl Relocalizer {
    /// Create a new relocalizer with the given configuration.
    pub fn new(config: RelocalizationConfig) -> Self {
        Self {
            config,
            keyframes: FxHashMap::default(),
            inverted_index: FxHashMap::default(),
            n_keyframes: 0,
        }
    }

    /// Create a relocalizer with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(RelocalizationConfig::default())
    }

    /// Add a keyframe to the database.
    ///
    /// This extracts ORB features and descriptors if configured, or uses
    /// existing features from the frame.
    pub fn add_keyframe(&mut self, frame: &Frame) {
        let kf_id = frame.frame_id;

        // Extract features and descriptors
        let (descriptors, coords, feature_ids, bow_vector) = if self.config.use_orb_features {
            self.extract_orb_features_and_compute_bow(&frame.left_features)
        } else {
            self.compute_bow_from_existing_features(&frame.left_features)
        };

        let n_features = descriptors.len();
        let n_bow_words = bow_vector.len();

        // Create keyframe entry
        let reloc_kf = RelocKeyframe {
            kf_id,
            timestamp_ns: frame.timestamp_ns,
            T_W_B: frame.state.T_W_B,
            descriptors,
            feature_coords: coords,
            feature_ids,
            bow_vector,
        };

        // Update inverted index
        for word_id in reloc_kf.bow_vector.keys() {
            self.inverted_index
                .entry(*word_id)
                .or_insert_with(|| Vec::with_capacity(16))
                .push(kf_id);
        }

        self.keyframes.insert(kf_id, reloc_kf);
        self.n_keyframes += 1;

        log::debug!(
            "[Relocalizer] Added keyframe {} with {} features, {} bow words",
            kf_id,
            n_features,
            n_bow_words
        );
    }

    /// Remove a keyframe from the database (when it leaves the sliding window).
    pub fn remove_keyframe(&mut self, kf_id: i32) {
        if let Some(kf) = self.keyframes.remove(&kf_id) {
            // Update inverted index
            for word_id in kf.bow_vector.keys() {
                if let Some(kf_list) = self.inverted_index.get_mut(word_id) {
                    kf_list.retain(|&id| id != kf_id);
                    if kf_list.is_empty() {
                        self.inverted_index.remove(word_id);
                    }
                }
            }
            self.n_keyframes -= 1;
            log::debug!("[Relocalizer] Removed keyframe {}", kf_id);
        }
    }

    /// Attempt to relocalize using actual map points from the sliding window.
    ///
    /// This version provides better accuracy by using the true 3D positions
    /// of map points instead of assuming fixed depth.
    ///
    /// # Arguments
    /// * `frame` - Current frame for relocalization
    /// * `map_points` - Map of feature_id → [x, y, z] positions from BA
    pub fn relocalize_with_map_points(
        &self,
        frame: &Frame,
        map_points: &FxHashMap<usize, [f32; 3]>,
    ) -> RelocalizationResult {
        if self.keyframes.is_empty() {
            return RelocalizationResult {
                success: false,
                T_W_B: None,
                n_inliers: 0,
                similarity_score: 0.0,
                matched_kf_id: None,
                error_message: Some("No keyframes in database".to_string()),
            };
        }

        // Extract features from current frame
        let (descriptors, coords, _, bow_vector) = if self.config.use_orb_features {
            self.extract_orb_features_and_compute_bow(&frame.left_features)
        } else {
            self.compute_bow_from_existing_features(&frame.left_features)
        };

        if descriptors.len() < 20 {
            return RelocalizationResult {
                success: false,
                T_W_B: None,
                n_inliers: 0,
                similarity_score: 0.0,
                matched_kf_id: None,
                error_message: Some(format!(
                    "Too few features for relocalization: {}",
                    descriptors.len()
                )),
            };
        }

        // Find candidate keyframes using bag-of-words
        let candidates = self.find_candidates(&bow_vector);

        if candidates.is_empty() {
            return RelocalizationResult {
                success: false,
                T_W_B: None,
                n_inliers: 0,
                similarity_score: 0.0,
                matched_kf_id: None,
                error_message: Some("No candidate keyframes found".to_string()),
            };
        }

        log::debug!(
            "[Relocalizer] Found {} candidate keyframes for relocalization",
            candidates.len()
        );

        // Try geometric verification on each candidate
        for (kf_id, similarity) in candidates {
            if let Some(kf) = self.keyframes.get(&kf_id) {
                // Match features between current frame and keyframe
                let matches = self.match_descriptors(&descriptors, &kf.descriptors);

                if matches.len() < 6 {
                    continue;
                }

                // Use actual map points where available, fall back to depth assumption
                let kf_points_3d: Vec<[f64; 3]> = kf.feature_ids
                    .iter()
                    .map(|&feat_id| {
                        // Look up actual 3D position from map_points
                        if let Some(&pos) = map_points.get(&feat_id) {
                            [pos[0] as f64, pos[1] as f64, pos[2] as f64]
                        } else {
                            // Fall back to depth assumption only if no map point available
                            let coord = &kf.feature_coords[kf.feature_ids.iter().position(|&id| id == feat_id).unwrap_or(0)];
                            [coord[0] as f64, coord[1] as f64, 2.0]
                        }
                    })
                    .collect();

                // Solve PnP with RANSAC
                let inliers = self.solve_pnp_ransac(
                    &kf.feature_coords,
                    &coords,
                    &matches,
                    &kf.T_W_B,
                    &kf_points_3d,
                );

                if inliers.success && inliers.n_inliers >= self.config.min_pnp_inliers {
                    log::info!(
                        "[Relocalizer] Successfully relocalized to keyframe {} with {} inliers (similarity: {:.3}, map_points: {}/{})",
                        kf_id, inliers.n_inliers, similarity,
                        kf_points_3d.iter().filter(|p| p[2].abs() > 0.1).count(),
                        kf_points_3d.len()
                    );

                    return RelocalizationResult {
                        success: true,
                        T_W_B: Some(inliers.T_W_B),
                        n_inliers: inliers.n_inliers,
                        similarity_score: similarity,
                        matched_kf_id: Some(kf_id),
                        error_message: None,
                    };
                }
            }
        }

        RelocalizationResult {
            success: false,
            T_W_B: None,
            n_inliers: 0,
            similarity_score: 0.0,
            matched_kf_id: None,
            error_message: Some("Geometric verification failed for all candidates (using map points)".to_string()),
        }
    }

    /// Attempt to relocalize the current frame.
    ///
    /// Returns the recovered pose if successful, or None if relocalization failed.
    /// Uses fixed-depth assumption for 3D points.
    pub fn relocalize(&self, frame: &Frame) -> RelocalizationResult {
        if self.keyframes.is_empty() {
            return RelocalizationResult {
                success: false,
                T_W_B: None,
                n_inliers: 0,
                similarity_score: 0.0,
                matched_kf_id: None,
                error_message: Some("No keyframes in database".to_string()),
            };
        }

        // Extract features from current frame
        let (descriptors, coords, _, bow_vector) = if self.config.use_orb_features {
            self.extract_orb_features_and_compute_bow(&frame.left_features)
        } else {
            self.compute_bow_from_existing_features(&frame.left_features)
        };

        if descriptors.len() < 20 {
            return RelocalizationResult {
                success: false,
                T_W_B: None,
                n_inliers: 0,
                similarity_score: 0.0,
                matched_kf_id: None,
                error_message: Some(format!(
                    "Too few features for relocalization: {}",
                    descriptors.len()
                )),
            };
        }

        // Find candidate keyframes using bag-of-words
        let candidates = self.find_candidates(&bow_vector);

        if candidates.is_empty() {
            return RelocalizationResult {
                success: false,
                T_W_B: None,
                n_inliers: 0,
                similarity_score: 0.0,
                matched_kf_id: None,
                error_message: Some("No candidate keyframes found".to_string()),
            };
        }

        log::debug!(
            "[Relocalizer] Found {} candidate keyframes for relocalization",
            candidates.len()
        );

        // Try geometric verification on each candidate
        for (kf_id, similarity) in candidates {
            if let Some(kf) = self.keyframes.get(&kf_id) {
                // Match features between current frame and keyframe
                let matches = self.match_descriptors(&descriptors, &kf.descriptors);

                if matches.len() < 6 {
                    continue;
                }

                // For now, use keyframe feature coordinates as approximate 3D points
                // Note: Future improvement: Use actual map points from the sliding window
                // for more accurate relocalization
                let kf_points_3d: Vec<[f64; 3]> = kf.feature_coords
                    .iter()
                    .map(|c| [c[0] as f64, c[1] as f64, 2.0])  // Assume depth=2m
                    .collect();

                // Solve PnP with RANSAC
                let inliers = self.solve_pnp_ransac(
                    &kf.feature_coords,
                    &coords,
                    &matches,
                    &kf.T_W_B,
                    &kf_points_3d,
                );

                if inliers.success && inliers.n_inliers >= self.config.min_pnp_inliers {
                    log::info!(
                        "[Relocalizer] Successfully relocalized to keyframe {} with {} inliers (similarity: {:.3})",
                        kf_id, inliers.n_inliers, similarity
                    );

                    return RelocalizationResult {
                        success: true,
                        T_W_B: Some(inliers.T_W_B),
                        n_inliers: inliers.n_inliers,
                        similarity_score: similarity,
                        matched_kf_id: Some(kf_id),
                        error_message: None,
                    };
                }
            }
        }

        RelocalizationResult {
            success: false,
            T_W_B: None,
            n_inliers: 0,
            similarity_score: 0.0,
            matched_kf_id: None,
            error_message: Some("Geometric verification failed for all candidates".to_string()),
        }
    }

    /// Get the number of keyframes in the database.
    pub fn n_keyframes(&self) -> usize {
        self.n_keyframes
    }

    // ========================================================================
    // Internal methods
    // ========================================================================

    /// Extract ORB features and compute bag-of-words vector.
    fn extract_orb_features_and_compute_bow(
        &self,
        _features: &[crate::feature_tracker::Feature],
    ) -> RelocResult {
        // Note: ORB feature detection and description to be implemented
        // Currently returns empty results as a placeholder
        // This will be filled in when ORB is implemented for better relocalization
        (Vec::new(), Vec::new(), Vec::new(), FxHashMap::default())
    }

    /// Compute bag-of-words from existing features.
    fn compute_bow_from_existing_features(
        &self,
        features: &[crate::feature_tracker::Feature],
    ) -> RelocResult {
        // For existing features, we can't compute ORB descriptors
        // This is a fallback that just uses coordinates
        let coords: Vec<[f32; 2]> = features
            .iter()
            .filter(|f| f.undistorted_coord[0] > -0.5)
            .map(|f| f.undistorted_coord)
            .collect();

        let feature_ids: Vec<usize> = features
            .iter()
            .filter(|f| f.undistorted_coord[0] > -0.5)
            .map(|f| f.feature_id)
            .collect();

        // Create a simple hash-based "bow" from feature coordinates
        // This is a placeholder until proper ORB + DBoW2 is implemented
        let mut bow_vector = FxHashMap::default();
        for &coord in coords.iter() {
            // Hash the coordinate to get a "word"
            let word = ((coord[0] * 10.0) as u32) ^ ((coord[1] * 10.0) as u32).wrapping_mul(31);
            *bow_vector.entry(word).or_insert(0.0) += 1.0;
        }

        // Normalize
        let norm: f32 = bow_vector.values().map(|v| v * v).sum::<f32>().sqrt();
        if norm > 1e-6 {
            for v in bow_vector.values_mut() {
                *v /= norm;
            }
        }

        (Vec::new(), coords, feature_ids, bow_vector)
    }

    /// Find candidate keyframes using bag-of-words similarity.
    fn find_candidates(
        &self,
        query_bow: &FxHashMap<u32, f32>,
    ) -> Vec<(i32, f64)> {
        if query_bow.is_empty() {
            return Vec::new();
        }

        // Count how many words match for each keyframe
        let mut kf_scores: FxHashMap<i32, f32> = FxHashMap::default();

        for (word_id, weight) in query_bow {
            if let Some(kf_ids) = self.inverted_index.get(word_id) {
                for &kf_id in kf_ids {
                    *kf_scores.entry(kf_id).or_insert(0.0) += weight;
                }
            }
        }

        // Convert to similarity scores and sort
        let mut candidates: Vec<(i32, f32)> = kf_scores.into_iter().collect();
        candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        // Take top candidates above threshold
        candidates
            .into_iter()
            .take(self.config.max_candidates)
            .filter_map(|(kf_id, score)| {
                if score as f64 >= self.config.min_similarity_score {
                    Some((kf_id, score as f64))
                } else {
                    None
                }
            })
            .collect()
    }

    /// Match binary descriptors using Hamming distance.
    fn match_descriptors(
        &self,
        desc1: &[OrbDescriptor],
        desc2: &[OrbDescriptor],
    ) -> Vec<(usize, usize)> {
        if desc1.is_empty() || desc2.is_empty() {
            return Vec::new();
        }

        let mut matches = Vec::new();
        let threshold = 80; // Hamming distance threshold

        for (i, d1) in desc1.iter().enumerate() {
            let mut best_dist = 256;
            let mut best_j = 0;

            for (j, d2) in desc2.iter().enumerate() {
                let dist = hamming_distance(d1, d2);
                if dist < best_dist {
                    best_dist = dist;
                    best_j = j;
                }
            }

            if best_dist < threshold {
                matches.push((i, best_j));
            }
        }

        matches
    }

    /// Solve PnP with RANSAC.
    fn solve_pnp_ransac(
        &self,
        _kf_coords: &[[f32; 2]],
        curr_coords: &[[f32; 2]],
        matches: &[(usize, usize)],
        kf_pose: &Matrix4x4,
        kf_points_3d: &[[f64; 3]],  // 3D points for matched features
    ) -> PnPResult {
        if matches.len() < 6 {
            return PnPResult {
                success: false,
                T_W_B: Matrix4x4::identity(),
                n_inliers: 0,
            };
        }

        // Build matched 3D-2D correspondences
        let mut points_3d = Vec::with_capacity(matches.len());
        let mut points_2d = Vec::with_capacity(matches.len());

        for &(kf_idx, curr_idx) in matches {
            if kf_idx < kf_points_3d.len() && curr_idx < curr_coords.len() {
                points_3d.push(kf_points_3d[kf_idx]);
                points_2d.push([
                    curr_coords[curr_idx][0] as f64,
                    curr_coords[curr_idx][1] as f64,
                ]);
            }
        }

        if points_3d.len() < 6 {
            return PnPResult {
                success: false,
                T_W_B: Matrix4x4::identity(),
                n_inliers: 0,
            };
        }

        // Use RANSAC PnP solver
        let camera = crate::estimator::pnp::CameraMatrix::new(500.0, 500.0, 256.0, 256.0);
        let ransac = crate::estimator::pnp::RansacPnP {
            threshold: self.config.ransac_threshold,
            min_inliers: self.config.min_pnp_inliers,
            max_iterations: 100,
            seed: 42,
        };

        let result = ransac.solve(&points_3d, &points_2d, &camera);

        if result.success {
            // PnP gives us T_C_W (camera-to-world in keyframe's local frame)
            // We need to transform to global world frame using the keyframe's known pose
            let T_C_W_local = result.T_C_W;
            
            // T_W_B_global = T_W_B_kf * T_B_kf_B_current = T_W_B_kf * inverse_se3(T_C_W_local)
            let T_Bkf_W_local = crate::estimator::sliding_window::inverse_se3(&T_C_W_local);
            let T_W_B = kf_pose * T_Bkf_W_local;

            PnPResult {
                success: true,
                T_W_B,
                n_inliers: result.n_inliers,
            }
        } else {
            PnPResult {
                success: false,
                T_W_B: Matrix4x4::identity(),
                n_inliers: result.n_inliers,
            }
        }
    }
}

/// Result of PnP solving.
#[derive(Debug, Clone)]
struct PnPResult {
    success: bool,
    T_W_B: Matrix4x4,
    n_inliers: usize,
}

/// Compute Hamming distance between two 32-byte descriptors.
#[inline]
fn hamming_distance(a: &OrbDescriptor, b: &OrbDescriptor) -> usize {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x ^ y).count_ones() as usize)
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hamming_distance() {
        let a = [0u8; 32];
        let b = [0u8; 32];
        assert_eq!(hamming_distance(&a, &b), 0);

        let mut c = [0u8; 32];
        c[0] = 1;
        assert_eq!(hamming_distance(&a, &c), 1);
    }

    #[test]
    fn test_relocalizer_empty() {
        let relocalizer = Relocalizer::with_defaults();
        assert_eq!(relocalizer.n_keyframes(), 0);
    }
}
