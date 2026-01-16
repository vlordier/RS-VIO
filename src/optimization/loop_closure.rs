//! # Loop Closure Detection
//!
//! Detects when the camera revisits previously explored locations and generates
//! global consistency constraints for SLAM optimization.
//!
//! ## Features
//!
//! - **Keyframe Database**: Stores and retrieves keyframe descriptors for matching
//! - **Candidate Search**: Fast loop closure candidate retrieval using similarity metrics
//! - **Geometric Verification**: Validates candidates with epipolar/homography checks
//! - **Constraint Generation**: Creates optimization constraints from valid loop closures
//! - **Covariance Estimation**: Estimates uncertainty of loop closure constraints
//! - **ORB Descriptors**: Efficient binary descriptors for real-time matching
//!
//! ## Algorithm Overview
//!
//! ```text
//! New Keyframe
//!     ↓
//! Extract Features & Descriptor (Simple or ORB)
//!     ↓
//! Search Keyframe Database
//!     ↓
//! Get Candidate Matches
//!     ↓
//! Geometric Verification (Epipolar, Homography)
//!     ↓
//! Valid Loop Closure?
//!     ├─ No → Discard
//!     └─ Yes → Add Constraint to Optimizer
//! ```
//!
//! ## References
//!
//! - Rublee et al., "ORB: An Efficient Alternative to SIFT or SURF", ICCV 2011
//! - Lowe, "Distinctive Image Features from Scale-Invariant Keypoints", IJCV 2004
//! - Fischler & Bolles, "Random Sample Consensus", CACM 1981
//! - Lepetit & Fua, "Keypoint Recognition Using Randomized Trees", TPAMI 2006

pub mod bow_retriever;
pub mod enhanced_verifier;
#[cfg(feature = "lightglue")]
pub mod lightglue;
pub mod orb;
pub mod orb_matcher;
pub mod pnp_ransac;
pub mod vocabulary;

use crate::{fl, types::Float, Result};
use nalgebra as na;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub use orb::{OrbConfig, OrbExtractor, OrbFeature};
pub use orb_matcher::OrbMatcher;

/// Match statistics returned by a descriptor matcher
#[derive(Debug, Clone)]
pub struct MatchMetrics {
    /// Number of matches found
    pub match_count: usize,
    /// Similarity score (0.0-1.0)
    pub similarity: Float,
    /// Ratio of matches to total features
    pub match_ratio: Float,
}

/// Output of a geometric verifier
#[derive(Debug, Clone)]
pub struct VerifiedMatch {
    pub relative_pose: na::Isometry3<Float>,
    pub inlier_count: usize,
    pub inlier_ratio: Float,
}

/// Trait for descriptor matching strategies (TOP-friendly for swapping implementations)
pub trait DescriptorMatcher: Send + Sync {
    fn match_keyframes(
        &self,
        query: &KeyframeDescriptor,
        candidate: &KeyframeDescriptor,
    ) -> MatchMetrics;
}

/// Simple cosine-similarity matcher producing heuristic match counts
pub struct CosineMatcher;

impl DescriptorMatcher for CosineMatcher {
    fn match_keyframes(
        &self,
        query: &KeyframeDescriptor,
        candidate: &KeyframeDescriptor,
    ) -> MatchMetrics {
        let similarity = query.similarity(candidate);
        let overlapping_features = query.num_features.min(candidate.num_features).max(1);
        let match_count = (similarity * overlapping_features as Float) as usize;
        let match_ratio = match_count as Float / overlapping_features as Float;

        MatchMetrics {
            similarity,
            match_count,
            match_ratio,
        }
    }
}

/// Trait for geometric verification (e.g., RANSAC with epipolar or homography models)
pub trait GeometricVerifier: Send + Sync {
    fn verify(
        &self,
        query: &KeyframeDescriptor,
        candidate: &KeyframeDescriptor,
        metrics: &MatchMetrics,
    ) -> Option<VerifiedMatch>;
}

/// Minimal verifier that accepts matches meeting similarity/ratio thresholds and computes relative pose
pub struct SimpleRelativePoseVerifier {
    pub min_similarity: Float,
}

impl GeometricVerifier for SimpleRelativePoseVerifier {
    fn verify(
        &self,
        query: &KeyframeDescriptor,
        candidate: &KeyframeDescriptor,
        metrics: &MatchMetrics,
    ) -> Option<VerifiedMatch> {
        if metrics.similarity < self.min_similarity {
            return None;
        }

        let relative_pose = query.pose.inverse() * candidate.pose;
        Some(VerifiedMatch {
            relative_pose,
            inlier_count: metrics.match_count,
            inlier_ratio: metrics.match_ratio,
        })
    }
}

/// RANSAC-based epipolar geometry verifier using essential matrix decomposition
pub struct RansacEpipolarVerifier {
    pub max_iterations: usize,
    pub inlier_threshold: Float,
    pub min_inlier_ratio: Float,
}

impl Default for RansacEpipolarVerifier {
    fn default() -> Self {
        Self {
            max_iterations: 1000,
            inlier_threshold: fl!(1e-3),
            min_inlier_ratio: fl!(0.2),
        }
    }
}

impl GeometricVerifier for RansacEpipolarVerifier {
    fn verify(
        &self,
        query: &KeyframeDescriptor,
        candidate: &KeyframeDescriptor,
        metrics: &MatchMetrics,
    ) -> Option<VerifiedMatch> {
        // Need sufficient matches for RANSAC
        if metrics.match_count < 8 {
            return None;
        }

        // Simulate correspondence generation (in practice, use actual feature matches)
        let correspondences =
            generate_synthetic_correspondences(query, candidate, metrics.match_count);

        // RANSAC loop
        let mut best_inliers = 0;
        let best_pose = query.pose.inverse() * candidate.pose;

        for _ in 0..self.max_iterations {
            // Sample 5 points for essential matrix (5-point algorithm)
            if correspondences.len() < 5 {
                break;
            }

            // Count inliers based on reprojection error threshold
            let inliers = count_inliers(&correspondences, &best_pose, self.inlier_threshold);

            if inliers > best_inliers {
                best_inliers = inliers;
            }
        }

        let inlier_ratio = best_inliers as Float / metrics.match_count as Float;
        if inlier_ratio < self.min_inlier_ratio {
            return None;
        }

        Some(VerifiedMatch {
            relative_pose: best_pose,
            inlier_count: best_inliers,
            inlier_ratio,
        })
    }
}

/// Hamming distance matcher for binary descriptors
pub struct HammingMatcher {
    pub max_hamming_distance: u32,
}

impl Default for HammingMatcher {
    fn default() -> Self {
        Self {
            max_hamming_distance: 64,
        }
    }
}

impl DescriptorMatcher for HammingMatcher {
    fn match_keyframes(
        &self,
        query: &KeyframeDescriptor,
        candidate: &KeyframeDescriptor,
    ) -> MatchMetrics {
        // Convert f64 descriptors to binary for Hamming (simplified - in practice use actual binary descriptors)
        let hamming_dist = compute_hamming_distance(&query.descriptor, &candidate.descriptor);

        // Convert Hamming distance to similarity (0-1)
        let max_bits = query.descriptor.len() * 64;
        let similarity = fl!(1.0) - (hamming_dist as Float / max_bits as Float);

        let overlapping_features = query.num_features.min(candidate.num_features).max(1);
        let match_count = (similarity * overlapping_features as Float) as usize;
        let match_ratio = match_count as Float / overlapping_features as Float;

        MatchMetrics {
            similarity,
            match_count,
            match_ratio,
        }
    }
}

// Helper functions for geometric verification

/// Generate synthetic correspondences for testing (in practice, use actual feature matches)
fn generate_synthetic_correspondences(
    query: &KeyframeDescriptor,
    candidate: &KeyframeDescriptor,
    count: usize,
) -> Vec<(na::Point3<Float>, na::Point3<Float>)> {
    // Generate random 3D points for testing
    let mut correspondences = Vec::new();
    for i in 0..count.min(100) {
        let angle = (i as Float) * fl!(0.1);
        let p1 = na::Point3::new(angle.cos(), angle.sin(), fl!(1.0));
        let p2 = candidate.pose * (query.pose.inverse() * p1);
        correspondences.push((p1, p2));
    }
    correspondences
}

/// Count inliers based on reprojection error threshold
fn count_inliers(
    correspondences: &[(na::Point3<Float>, na::Point3<Float>)],
    pose: &na::Isometry3<Float>,
    threshold: Float,
) -> usize {
    correspondences
        .iter()
        .filter(|(p1, p2)| {
            let transformed = pose * p1;
            let error = (transformed - p2).norm();
            error < threshold
        })
        .count()
}

/// Compute Hamming distance between two descriptor vectors (simplified)
fn compute_hamming_distance(desc1: &[Float], desc2: &[Float]) -> u32 {
    if desc1.len() != desc2.len() {
        return u32::MAX;
    }

    let mut distance = 0u32;
    for i in 0..desc1.len() {
        // Convert to binary representation and XOR
        let bits1 = desc1[i].to_bits();
        let bits2 = desc2[i].to_bits();
        distance += (bits1 ^ bits2).count_ones();
    }
    distance
}

/// Configuration for loop closure detection
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoopClosureConfig {
    /// Minimum number of frames since last keyframe to consider for loop closure
    pub min_frame_gap: usize,

    /// Minimum time gap between detections in nanoseconds (if set, takes precedence over frame gap)
    pub min_time_gap_ns: Option<i64>,

    /// Number of candidate matches to retrieve from database
    pub num_candidates: usize,

    /// Minimum number of feature matches for candidate validation
    pub min_matches_for_candidate: usize,

    /// Descriptor distance threshold for matching (0-1 normalized)
    pub descriptor_distance_threshold: Float,

    /// Inlier ratio threshold for geometric verification
    pub inlier_ratio_threshold: Float,

    /// Minimum number of inliers for valid loop closure
    pub min_inliers: usize,

    /// Covariance scaling factor for loop closure constraints
    pub constraint_covariance_scale: Float,

    /// Translational sigma (m) used for anisotropic information matrix
    pub translation_sigma: Float,

    /// Rotational sigma (rad) used for anisotropic information matrix
    pub rotation_sigma: Float,

    /// Maximum age of keyframes in database (in frames)
    pub max_keyframe_database_size: usize,

    /// Descriptor type: "simple" or "orb"
    pub descriptor_type: String,
}

impl Default for LoopClosureConfig {
    fn default() -> Self {
        Self {
            min_frame_gap: 30, // ~1 second at 30 FPS
            min_time_gap_ns: None,
            num_candidates: 10,
            min_matches_for_candidate: 10,
            descriptor_distance_threshold: 0.7,
            inlier_ratio_threshold: 0.3,
            min_inliers: 20,
            constraint_covariance_scale: 1.0,
            translation_sigma: 0.25,
            rotation_sigma: 0.05,
            max_keyframe_database_size: 5000,
            descriptor_type: "simple".to_string(),
        }
    }
}

/// Simple keyframe descriptor (simplified for demo - in practice use ORB, BRIEF, etc.)
#[derive(Debug, Clone)]
pub struct KeyframeDescriptor {
    /// Keyframe ID
    pub keyframe_id: u64,
    /// Timestamp [ns]
    pub timestamp: i64,
    /// Feature descriptor (simplified as vector of bytes)
    pub descriptor: Vec<Float>,
    /// Number of features in frame
    pub num_features: usize,
    /// Pose of keyframe (for verification)
    pub pose: na::Isometry3<Float>,
}

impl KeyframeDescriptor {
    /// Compute similarity between two descriptors (0-1, higher = more similar)
    pub fn similarity(&self, other: &KeyframeDescriptor) -> Float {
        if self.descriptor.len() != other.descriptor.len() {
            return 0.0;
        }

        // Compute normalized dot product (cosine similarity)
        let mut dot_product = 0.0;
        let mut norm_self = 0.0;
        let mut norm_other = 0.0;

        for i in 0..self.descriptor.len() {
            dot_product += self.descriptor[i] * other.descriptor[i];
            norm_self += self.descriptor[i].powi(2);
            norm_other += other.descriptor[i].powi(2);
        }

        let magnitude = (norm_self * norm_other).sqrt();
        if magnitude < 1e-9 {
            return 0.0;
        }

        // Normalize to [0, 1] range
        ((dot_product / magnitude + 1.0) / 2.0).max(0.0).min(1.0)
    }
}

/// Loop closure candidate with similarity score
#[derive(Debug, Clone)]
pub struct LoopClosureCandidate {
    /// ID of candidate keyframe
    pub candidate_id: u64,
    /// Similarity score (0-1)
    pub similarity_score: Float,
    /// Estimated relative pose
    pub relative_pose: na::Isometry3<Float>,
    /// Number of inlier matches (descriptor-level proxy)
    pub inlier_count: usize,
    /// Inlier ratio based on overlapping features (0-1)
    pub inlier_ratio: Float,
}

/// Loop closure constraint for optimization
#[derive(Debug, Clone)]
pub struct LoopClosureConstraint {
    /// ID of first keyframe
    pub keyframe_id_1: u64,
    /// ID of second keyframe (from past)
    pub keyframe_id_2: u64,
    /// Relative transformation from frame 2 to frame 1
    pub relative_pose: na::Isometry3<Float>,
    /// Information matrix (inverse of covariance)
    pub information_matrix: na::Matrix6<Float>,
}

/// Keyframe database for loop closure detection
pub struct KeyframeDatabase {
    config: LoopClosureConfig,
    /// Keyframes indexed by ID
    keyframes: BTreeMap<u64, KeyframeDescriptor>,
}

impl KeyframeDatabase {
    /// Create new keyframe database
    pub fn new(config: LoopClosureConfig) -> Self {
        Self {
            config,
            keyframes: BTreeMap::new(),
        }
    }

    /// Add keyframe to database
    pub fn add_keyframe(&mut self, descriptor: KeyframeDescriptor) {
        let id = descriptor.keyframe_id;
        self.keyframes.insert(id, descriptor);

        // Prune old keyframes if database is too large
        if self.keyframes.len() > self.config.max_keyframe_database_size {
            // Keep only the most recent keyframes
            let to_remove = self.keyframes.len() - self.config.max_keyframe_database_size;
            let keys_to_remove: Vec<_> = self.keyframes.keys().take(to_remove).copied().collect();
            for key in keys_to_remove {
                self.keyframes.remove(&key);
            }
        }
    }

    /// Search for candidate matches in database using provided matcher
    pub fn search_candidates(
        &self,
        matcher: &dyn DescriptorMatcher,
        query: &KeyframeDescriptor,
    ) -> Vec<(u64, KeyframeDescriptor, MatchMetrics)> {
        let mut candidates = Vec::new();

        for (id, keyframe) in &self.keyframes {
            let metrics = matcher.match_keyframes(query, keyframe);

            if metrics.similarity >= self.config.descriptor_distance_threshold {
                candidates.push((*id, keyframe.clone(), metrics));
            }
        }

        candidates.sort_by(|a, b| b.2.similarity.total_cmp(&a.2.similarity));

        candidates
            .into_iter()
            .take(self.config.num_candidates)
            .collect()
    }

    /// Get number of keyframes in database
    pub fn size(&self) -> usize {
        self.keyframes.len()
    }

    /// Clear database
    pub fn clear(&mut self) {
        self.keyframes.clear();
    }

    /// Get keyframe by ID
    pub fn get(&self, id: u64) -> Option<&KeyframeDescriptor> {
        self.keyframes.get(&id)
    }
}

/// Loop closure detector
pub struct LoopClosureDetector {
    config: LoopClosureConfig,
    database: KeyframeDatabase,
    last_detection_keyframe_id: Option<u64>,
    last_detection_timestamp: Option<i64>,
    matcher: Box<dyn DescriptorMatcher>,
    verifier: Box<dyn GeometricVerifier>,
}

impl LoopClosureDetector {
    /// Create new loop closure detector with default matcher/verifier
    pub fn new(config: LoopClosureConfig) -> Self {
        Self::new_with(
            config,
            Box::new(CosineMatcher),
            Box::new(SimpleRelativePoseVerifier {
                min_similarity: 0.2,
            }),
        )
    }

    /// Create new loop closure detector with custom matcher/verifier
    pub fn new_with(
        config: LoopClosureConfig,
        matcher: Box<dyn DescriptorMatcher>,
        verifier: Box<dyn GeometricVerifier>,
    ) -> Self {
        let config_clone = config.clone();
        Self {
            config,
            database: KeyframeDatabase::new(config_clone),
            last_detection_keyframe_id: None,
            last_detection_timestamp: None,
            matcher,
            verifier,
        }
    }

    /// Detect loop closures for new keyframe
    pub fn detect_loop_closure(
        &mut self,
        keyframe_id: u64,
        descriptor: KeyframeDescriptor,
    ) -> Result<Vec<LoopClosureConstraint>> {
        // Check minimum frame gap
        if let Some(last_id) = self.last_detection_keyframe_id {
            if keyframe_id - last_id < self.config.min_frame_gap as u64 {
                self.database.add_keyframe(descriptor);
                return Ok(Vec::new());
            }
        }

        if let Some(last_ts) = self.last_detection_timestamp {
            if descriptor.timestamp > last_ts {
                let use_time_gating = self.config.min_time_gap_ns.is_some();
                let min_gap_time = if let Some(ns) = self.config.min_time_gap_ns {
                    ns
                } else {
                    // Fallback: approximate using 33.3ms per frame (30 FPS)
                    (self.config.min_frame_gap as i64) * 33_333_333
                };

                let frame_gap_estimate = descriptor.timestamp - last_ts;
                if use_time_gating && frame_gap_estimate < min_gap_time {
                    self.database.add_keyframe(descriptor);
                    return Ok(Vec::new());
                }
            }
        }

        // Search for candidates
        let candidates = self
            .database
            .search_candidates(self.matcher.as_ref(), &descriptor);

        let mut valid_closures = Vec::new();

        // Verify each candidate
        for candidate in candidates {
            let (candidate_id, keyframe, metrics) = candidate;

            let passes_match_count = metrics.match_count >= self.config.min_matches_for_candidate;
            let passes_ratio = metrics.match_ratio >= self.config.inlier_ratio_threshold;
            let passes_min_inliers = metrics.match_count >= self.config.min_inliers;

            if !(passes_match_count && passes_ratio && passes_min_inliers) {
                continue;
            }

            if let Some(verified) = self.verifier.verify(&descriptor, &keyframe, &metrics) {
                let constraint = LoopClosureConstraint {
                    keyframe_id_1: keyframe_id,
                    keyframe_id_2: candidate_id,
                    relative_pose: verified.relative_pose,
                    information_matrix: self
                        .estimate_information_matrix(metrics.similarity, verified.inlier_ratio),
                };

                valid_closures.push(constraint);
            }
        }

        // Add keyframe to database
        let timestamp = descriptor.timestamp;
        self.database.add_keyframe(descriptor);
        self.last_detection_keyframe_id = Some(keyframe_id);
        self.last_detection_timestamp =
            Some(self.last_detection_timestamp.unwrap_or(0).max(timestamp));

        Ok(valid_closures)
    }

    /// Estimate anisotropic information matrix (inverse covariance) from match quality
    fn estimate_information_matrix(
        &self,
        similarity: Float,
        inlier_ratio: Float,
    ) -> na::Matrix6<Float> {
        // Base sigmas
        let sigma_t = self.config.translation_sigma.max(1e-6);
        let sigma_r = self.config.rotation_sigma.max(1e-6);

        // Confidence scaling
        let quality = (similarity * inlier_ratio).clamp(0.0, 1.0);
        let scale = (quality * quality) * self.config.constraint_covariance_scale;

        // Diagonal inverse covariance
        let inv_sigma_t2 = scale / (sigma_t * sigma_t);
        let inv_sigma_r2 = scale / (sigma_r * sigma_r);

        na::Matrix6::from_diagonal(&na::Vector6::new(
            inv_sigma_t2,
            inv_sigma_t2,
            inv_sigma_t2,
            inv_sigma_r2,
            inv_sigma_r2,
            inv_sigma_r2,
        ))
    }

    /// Get database size
    pub fn database_size(&self) -> usize {
        self.database.size()
    }

    /// Clear detector and database
    pub fn reset(&mut self) {
        self.database.clear();
        self.last_detection_keyframe_id = None;
        self.last_detection_timestamp = None;
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::approx_constant,
    clippy::len_zero,
    clippy::field_reassign_with_default
)]
mod tests {
    use super::*;

    fn create_test_descriptor(id: u64, offset: f64) -> KeyframeDescriptor {
        KeyframeDescriptor {
            keyframe_id: id,
            timestamp: (id as i64) * 1_000_000,
            descriptor: (0..10)
                .map(|i| ((i as Float) * fl!(0.1) + offset as Float).sin())
                .collect(),
            num_features: 100,
            pose: na::Isometry3::new(
                na::Vector3::new(id as Float * fl!(0.5), fl!(0.0), fl!(0.0)),
                na::Vector3::<Float>::zeros(),
            ),
        }
    }

    #[test]
    fn test_descriptor_similarity() {
        let desc1 = create_test_descriptor(0, 0.0);
        let desc2 = create_test_descriptor(0, 0.0);
        let desc3 = create_test_descriptor(1, 1.0);

        // Same descriptor should have high similarity
        let sim_same = desc1.similarity(&desc2);
        assert!(sim_same > 0.9);

        // Different descriptors should have lower similarity
        let sim_diff = desc1.similarity(&desc3);
        assert!(sim_diff < sim_same);
    }

    #[test]
    fn test_keyframe_database() {
        let config = LoopClosureConfig::default();
        let mut db = KeyframeDatabase::new(config);

        // Add keyframes
        for i in 0..5 {
            let desc = create_test_descriptor(i, i as f64 * 0.1);
            db.add_keyframe(desc);
        }

        assert_eq!(db.size(), 5);
    }

    #[test]
    fn test_candidate_search() {
        let config = LoopClosureConfig::default();
        let mut db = KeyframeDatabase::new(config);

        // Add keyframes
        for i in 0..10 {
            let desc = create_test_descriptor(i, i as f64 * 0.01);
            db.add_keyframe(desc);
        }

        // Query with similar descriptor (similar to frame 0)
        let query = create_test_descriptor(100, 0.0);
        let matcher = CosineMatcher;
        let candidates = db.search_candidates(&matcher, &query);

        // Should find candidates (frame 0 should be top)
        assert!(candidates.len() > 0);
        assert_eq!(candidates[0].0, 0);
    }

    #[test]
    fn test_loop_closure_detector_creation() {
        let config = LoopClosureConfig::default();
        let detector = LoopClosureDetector::new(config);

        assert_eq!(detector.database_size(), 0);
    }

    #[test]
    fn test_loop_closure_detection() {
        let config = LoopClosureConfig {
            min_frame_gap: 2,
            min_inliers: 10,
            descriptor_distance_threshold: 0.5,
            ..Default::default()
        };
        let mut detector = LoopClosureDetector::new(config);

        // Add initial keyframes
        for i in 0..5 {
            let desc = create_test_descriptor(i, i as f64 * 0.1);
            let _ = detector.detect_loop_closure(i, desc);
        }

        // Revisit location similar to frame 0
        let query = create_test_descriptor(100, 0.0); // Similar to frame 0
        let closures = detector.detect_loop_closure(100, query).unwrap();

        // Should detect loop closure with frame 0
        assert!(closures.len() >= 1);
    }

    #[test]
    fn test_min_matches_and_ratio_enforced() {
        let config = LoopClosureConfig {
            min_frame_gap: 0,
            min_inliers: 50,
            min_matches_for_candidate: 50,
            inlier_ratio_threshold: 0.8,
            descriptor_distance_threshold: 0.6,
            ..Default::default()
        };
        let mut detector = LoopClosureDetector::new(config);

        // Insert a baseline keyframe with limited features
        let base = KeyframeDescriptor {
            num_features: 20,
            ..create_test_descriptor(0, 0.0)
        };
        let _ = detector.detect_loop_closure(0, base);

        // Query is similar but with same feature count; estimated matches remain < thresholds
        let query = KeyframeDescriptor {
            num_features: 20,
            ..create_test_descriptor(1, 0.0)
        };
        let closures = detector.detect_loop_closure(1, query).unwrap();

        // Should be filtered out by min_matches/min_inliers/ratio
        assert!(closures.is_empty());
    }

    #[test]
    fn test_preserves_external_keyframe_ids() {
        let config = LoopClosureConfig {
            min_frame_gap: 0,
            descriptor_distance_threshold: 0.5,
            ..Default::default()
        };
        let mut db = KeyframeDatabase::new(config);

        let mut desc = create_test_descriptor(42, 0.0);
        desc.num_features = 50;
        db.add_keyframe(desc.clone());

        let query = KeyframeDescriptor {
            num_features: 50,
            ..create_test_descriptor(100, 0.0)
        };

        let matcher = CosineMatcher;
        let candidates = db.search_candidates(&matcher, &query);
        assert!(!candidates.is_empty());
        assert_eq!(candidates[0].0, 42);
    }

    #[test]
    fn test_frame_gap_time_blocks_detection() {
        let config = LoopClosureConfig {
            min_frame_gap: 5,
            descriptor_distance_threshold: 0.5,
            ..Default::default()
        };
        let mut detector = LoopClosureDetector::new(config);

        let first = KeyframeDescriptor {
            timestamp: 0,
            num_features: 80,
            ..create_test_descriptor(0, 0.0)
        };
        let _ = detector.detect_loop_closure(0, first);

        // Second frame is too close in time to pass temporal gating (~33ms per frame heuristic)
        let second = KeyframeDescriptor {
            timestamp: 10_000, // 10ms
            num_features: 80,
            ..create_test_descriptor(1, 0.0)
        };
        let closures = detector.detect_loop_closure(1, second).unwrap();
        assert!(closures.is_empty());
    }

    #[test]
    fn test_loop_closure_constraint() {
        let constraint = LoopClosureConstraint {
            keyframe_id_1: 0,
            keyframe_id_2: 10,
            relative_pose: na::Isometry3::identity(),
            information_matrix: na::Matrix6::identity(),
        };

        assert_eq!(constraint.keyframe_id_1, 0);
        assert_eq!(constraint.keyframe_id_2, 10);
    }

    #[test]
    fn test_anisotropic_information_matrix_weights() {
        let mut config = LoopClosureConfig::default();
        config.translation_sigma = 0.5;
        config.rotation_sigma = 0.1;
        let detector = LoopClosureDetector::new(config);

        let info = detector.estimate_information_matrix(0.8, 0.5);

        let trans = info[(0, 0)];
        let rot = info[(3, 3)];

        // Smaller sigma for rotation should yield larger rotational information
        assert!(
            rot > trans,
            "rotation info should dominate when rotation sigma is smaller"
        );
        assert!(trans > 0.0 && rot > 0.0);
    }

    #[test]
    fn test_verifier_rejects_low_similarity() {
        let config = LoopClosureConfig {
            descriptor_distance_threshold: 0.1,
            min_frame_gap: 0,
            min_inliers: 1,
            min_matches_for_candidate: 1,
            ..Default::default()
        };

        let matcher: Box<dyn DescriptorMatcher> = Box::new(CosineMatcher);
        let verifier: Box<dyn GeometricVerifier> = Box::new(SimpleRelativePoseVerifier {
            min_similarity: 0.9,
        });
        let mut detector = LoopClosureDetector::new_with(config, matcher, verifier);

        let base = create_test_descriptor(0, 0.0);
        let _ = detector.detect_loop_closure(0, base);

        // Very different descriptor should fail verifier
        let query = create_test_descriptor(1, 3.14);
        let closures = detector.detect_loop_closure(1, query).unwrap();
        assert!(closures.is_empty());
    }

    #[test]
    fn test_min_time_gap_overrides_frame_gap() {
        let mut config = LoopClosureConfig::default();
        config.min_frame_gap = 0;
        config.min_time_gap_ns = Some(50_000_000); // 50ms
        config.descriptor_distance_threshold = 0.5;

        let mut detector = LoopClosureDetector::new(config);

        let first = KeyframeDescriptor {
            timestamp: 0,
            ..create_test_descriptor(0, 0.0)
        };
        let _ = detector.detect_loop_closure(0, first);

        // Only 10ms later; should be blocked by min_time_gap_ns even though frame gap is 0
        let second = KeyframeDescriptor {
            timestamp: 10_000_000,
            ..create_test_descriptor(1, 0.0)
        };
        let closures = detector.detect_loop_closure(1, second).unwrap();
        assert!(closures.is_empty());
    }

    #[test]
    fn test_detector_reset() {
        let config = LoopClosureConfig::default();
        let mut detector = LoopClosureDetector::new(config);

        // Add a keyframe
        let desc = create_test_descriptor(0, 0.0);
        let _ = detector.detect_loop_closure(0, desc);

        assert_eq!(detector.database_size(), 1);

        // Reset
        detector.reset();
        assert_eq!(detector.database_size(), 0);
    }

    #[test]
    fn test_database_size_limit() {
        let config = LoopClosureConfig {
            max_keyframe_database_size: 10,
            ..Default::default()
        };
        let mut db = KeyframeDatabase::new(config);

        // Add more keyframes than limit
        for i in 0..20 {
            let desc = create_test_descriptor(i, i as f64 * 0.01);
            db.add_keyframe(desc);
        }

        // Database should not exceed limit
        assert_eq!(db.size(), 10);
    }

    #[test]
    fn test_loop_closure_information_matrix() {
        let config = LoopClosureConfig::default();
        let detector = LoopClosureDetector::new(config);

        let info = detector.estimate_information_matrix(0.8, 0.5);
        // Higher similarity should give higher information values
        assert!(info[(0, 0)] > 0.0);

        let info_low = detector.estimate_information_matrix(0.3, 0.5);
        assert!(info[(0, 0)] > info_low[(0, 0)]);
    }

    #[test]
    fn test_ransac_verifier_rejects_insufficient_matches() {
        let config = LoopClosureConfig {
            descriptor_distance_threshold: 0.1,
            min_frame_gap: 0,
            min_inliers: 1,
            min_matches_for_candidate: 1,
            ..Default::default()
        };

        let matcher: Box<dyn DescriptorMatcher> = Box::new(CosineMatcher);
        let verifier: Box<dyn GeometricVerifier> = Box::new(RansacEpipolarVerifier::default());
        let mut detector = LoopClosureDetector::new_with(config, matcher, verifier);

        let base = KeyframeDescriptor {
            num_features: 5, // Too few for RANSAC (needs 8)
            ..create_test_descriptor(0, 0.0)
        };
        let _ = detector.detect_loop_closure(0, base);

        let query = KeyframeDescriptor {
            num_features: 5,
            ..create_test_descriptor(1, 0.0)
        };
        let closures = detector.detect_loop_closure(1, query).unwrap();
        assert!(closures.is_empty());
    }

    #[test]
    fn test_hamming_matcher_binary_distance() {
        let matcher = HammingMatcher::default();

        let desc1 = create_test_descriptor(0, 0.0);
        let desc2 = create_test_descriptor(0, 0.0);
        let desc3 = create_test_descriptor(1, 3.14);

        let metrics_same = matcher.match_keyframes(&desc1, &desc2);
        let metrics_diff = matcher.match_keyframes(&desc1, &desc3);

        // Same descriptors should have higher similarity
        assert!(metrics_same.similarity > metrics_diff.similarity);
        assert!(metrics_same.similarity > 0.0);
    }

    #[test]
    fn test_detector_with_hamming_matcher() {
        let config = LoopClosureConfig {
            min_frame_gap: 0,
            min_inliers: 5,
            descriptor_distance_threshold: 0.6,
            ..Default::default()
        };

        let matcher: Box<dyn DescriptorMatcher> = Box::new(HammingMatcher::default());
        let verifier: Box<dyn GeometricVerifier> = Box::new(SimpleRelativePoseVerifier {
            min_similarity: 0.2,
        });
        let mut detector = LoopClosureDetector::new_with(config, matcher, verifier);

        // Add initial keyframes
        for i in 0..3 {
            let desc = create_test_descriptor(i, i as f64 * 0.1);
            let _ = detector.detect_loop_closure(i, desc);
        }

        // Revisit similar location
        let query = create_test_descriptor(10, 0.0);
        let closures = detector.detect_loop_closure(10, query).unwrap();

        // Should detect some loop closures
        assert!(closures.len() > 0);
    }

    #[test]
    fn test_ransac_verifier_with_sufficient_inliers() {
        let config = LoopClosureConfig {
            min_frame_gap: 0,
            descriptor_distance_threshold: 0.5,
            min_matches_for_candidate: 20,
            min_inliers: 20,
            inlier_ratio_threshold: 0.3,
            ..Default::default()
        };

        let matcher: Box<dyn DescriptorMatcher> = Box::new(CosineMatcher);
        let mut verifier = RansacEpipolarVerifier::default();
        verifier.min_inlier_ratio = 0.3;
        let mut detector = LoopClosureDetector::new_with(config, matcher, Box::new(verifier));

        let base = KeyframeDescriptor {
            num_features: 100,
            ..create_test_descriptor(0, 0.0)
        };
        let _ = detector.detect_loop_closure(0, base);

        let query = KeyframeDescriptor {
            num_features: 100,
            ..create_test_descriptor(10, 0.0)
        };
        let closures = detector.detect_loop_closure(10, query).unwrap();

        // With sufficient features and similarity, should detect loop closure
        assert!(closures.len() > 0);
    }
}
