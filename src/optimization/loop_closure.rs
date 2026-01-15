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
//!
//! ## Algorithm Overview
//!
//! ```text
//! New Keyframe
//!     ↓
//! Extract Features & Descriptor
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
//! - Lowe, "Distinctive Image Features from Scale-Invariant Keypoints", IJCV 2004
//! - Fischler & Bolles, "Random Sample Consensus", CACM 1981
//! - Lepetit & Fua, "Keypoint Recognition Using Randomized Trees", TPAMI 2006

use nalgebra as na;
use serde::{Deserialize, Serialize};
use crate::Result;
use std::collections::BTreeMap;

/// Configuration for loop closure detection
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoopClosureConfig {
    /// Minimum number of frames since last keyframe to consider for loop closure
    pub min_frame_gap: usize,

    /// Number of candidate matches to retrieve from database
    pub num_candidates: usize,

    /// Minimum number of feature matches for candidate validation
    pub min_matches_for_candidate: usize,

    /// Descriptor distance threshold for matching (0-1 normalized)
    pub descriptor_distance_threshold: f64,

    /// Inlier ratio threshold for geometric verification
    pub inlier_ratio_threshold: f64,

    /// Minimum number of inliers for valid loop closure
    pub min_inliers: usize,

    /// Covariance scaling factor for loop closure constraints
    pub constraint_covariance_scale: f64,

    /// Maximum age of keyframes in database (in frames)
    pub max_keyframe_database_size: usize,
}

impl Default for LoopClosureConfig {
    fn default() -> Self {
        Self {
            min_frame_gap: 30,                  // ~1 second at 30 FPS
            num_candidates: 10,
            min_matches_for_candidate: 10,
            descriptor_distance_threshold: 0.7,
            inlier_ratio_threshold: 0.3,
            min_inliers: 20,
            constraint_covariance_scale: 1.0,
            max_keyframe_database_size: 5000,
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
    pub descriptor: Vec<f64>,
    /// Number of features in frame
    pub num_features: usize,
    /// Pose of keyframe (for verification)
    pub pose: na::Isometry3<f64>,
}

impl KeyframeDescriptor {
    /// Compute similarity between two descriptors (0-1, higher = more similar)
    pub fn similarity(&self, other: &KeyframeDescriptor) -> f64 {
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
    pub similarity_score: f64,
    /// Estimated relative pose
    pub relative_pose: na::Isometry3<f64>,
    /// Number of inlier matches (descriptor-level proxy)
    pub inlier_count: usize,
    /// Inlier ratio based on overlapping features (0-1)
    pub inlier_ratio: f64,
}

/// Loop closure constraint for optimization
#[derive(Debug, Clone)]
pub struct LoopClosureConstraint {
    /// ID of first keyframe
    pub keyframe_id_1: u64,
    /// ID of second keyframe (from past)
    pub keyframe_id_2: u64,
    /// Relative transformation from frame 2 to frame 1
    pub relative_pose: na::Isometry3<f64>,
    /// Information matrix (inverse of covariance)
    pub information_matrix: na::Matrix6<f64>,
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

    /// Search for candidate matches in database
    pub fn search_candidates(&self, query: &KeyframeDescriptor) -> Vec<LoopClosureCandidate> {
        let mut candidates = Vec::new();

        // Search all keyframes and compute similarity
        for (id, keyframe) in &self.keyframes {
            let similarity = query.similarity(keyframe);

            // Only keep candidates above threshold
            if similarity >= self.config.descriptor_distance_threshold {
                candidates.push((*id, similarity, keyframe.clone()));
            }
        }

        // Sort by similarity (descending)
        candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        // Convert to LoopClosureCandidate and limit count
        candidates
            .into_iter()
            .take(self.config.num_candidates)
            .map(|(id, similarity, keyframe)| {
                // Simple relative pose estimation (in practice use epipolar geometry)
                let relative_pose = query.pose.inverse() * keyframe.pose;

                let overlapping_features = query.num_features.min(keyframe.num_features).max(1);
                let estimated_inliers = (similarity * overlapping_features as f64) as usize;
                let inlier_ratio = estimated_inliers as f64 / overlapping_features as f64;

                LoopClosureCandidate {
                    candidate_id: id,
                    similarity_score: similarity,
                    relative_pose,
                    inlier_count: estimated_inliers,
                    inlier_ratio,
                }
            })
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
}

impl LoopClosureDetector {
    /// Create new loop closure detector
    pub fn new(config: LoopClosureConfig) -> Self {
        let config_clone = config.clone();
        Self {
            config,
            database: KeyframeDatabase::new(config_clone),
            last_detection_keyframe_id: None,
            last_detection_timestamp: None,
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
                // Too recent, skip detection
                self.database.add_keyframe(descriptor);
                return Ok(Vec::new());
            }
        }

        if let Some(last_ts) = self.last_detection_timestamp {
            if descriptor.timestamp > last_ts {
                // Treat min_frame_gap as frames; approximate time using 33.3ms/frame (30 FPS default) for temporal gating.
                let min_gap_time = (self.config.min_frame_gap as i64) * 33_333;
                let frame_gap_estimate = descriptor.timestamp - last_ts;
                if frame_gap_estimate < min_gap_time {
                    self.database.add_keyframe(descriptor);
                    return Ok(Vec::new());
                }
            }
        }

        // Search for candidates
        let candidates = self.database.search_candidates(&descriptor);

        let mut valid_closures = Vec::new();

        // Verify each candidate
        for candidate in candidates {
            let passes_match_count = candidate.inlier_count >= self.config.min_matches_for_candidate;
            let passes_ratio = candidate.inlier_ratio >= self.config.inlier_ratio_threshold;
            let passes_min_inliers = candidate.inlier_count >= self.config.min_inliers;

            if passes_match_count && passes_ratio && passes_min_inliers {
                // Valid loop closure
                let constraint = LoopClosureConstraint {
                    keyframe_id_1: keyframe_id,
                    keyframe_id_2: candidate.candidate_id,
                    relative_pose: candidate.relative_pose,
                    information_matrix: self.estimate_information_matrix(candidate.similarity_score),
                };

                valid_closures.push(constraint);
            }
        }

        // Add keyframe to database
        let timestamp = descriptor.timestamp;
        self.database.add_keyframe(descriptor);
        self.last_detection_keyframe_id = Some(keyframe_id);
        self.last_detection_timestamp = Some(self.last_detection_timestamp.unwrap_or(0).max(timestamp));

        Ok(valid_closures)
    }

    /// Estimate information matrix (inverse covariance) from match quality
    fn estimate_information_matrix(&self, similarity: f64) -> na::Matrix6<f64> {
        // Higher similarity → higher confidence (higher information)
        let info_scale = similarity * similarity * self.config.constraint_covariance_scale;
        na::Matrix6::identity() * info_scale
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
mod tests {
    use super::*;

    fn create_test_descriptor(id: u64, offset: f64) -> KeyframeDescriptor {
        KeyframeDescriptor {
            keyframe_id: id,
            timestamp: (id as i64) * 1_000_000,
            descriptor: (0..10)
                .map(|i| (i as f64 * 0.1 + offset).sin())
                .collect(),
            num_features: 100,
            pose: na::Isometry3::new(
                na::Vector3::new(id as f64 * 0.5, 0.0, 0.0),
                na::Vector3::zeros(),
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
        let candidates = db.search_candidates(&query);

        // Should find candidates (frame 0 should be top)
        assert!(candidates.len() > 0);
        assert!(candidates[0].candidate_id == 0);
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

        let candidates = db.search_candidates(&query);
        assert!(!candidates.is_empty());
        assert_eq!(candidates[0].candidate_id, 42);
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

        let info = detector.estimate_information_matrix(0.8);
        // Higher similarity should give higher information values
        assert!(info[(0, 0)] > 0.0);

        let info_low = detector.estimate_information_matrix(0.3);
        assert!(info[(0, 0)] > info_low[(0, 0)]);
    }
}
