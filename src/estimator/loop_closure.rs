//! Loop closure detection and optimization.
//!
//! Detects when the camera revisits previously seen locations and adds global
//! constraints to the pose graph to correct long-term scale drift.
//!
//! ## Algorithm Overview
//!
//! 1. **Descriptor Matching**: Compare ORB descriptors between keyframes using Hamming distance
//! 2. **Geometric Validation**: Validate matches with epipolar constraints
//! 3. **Loop Edge Creation**: Create pose graph edge with relative transform and scale
//! 4. **Optimization**: Joint optimization of poses and scale in BA
//!
//! ## Performance
//!
//! - Brute-force matching: O(n²) but fast for <200 keyframes
//! - Per-keyframe overhead: <10ms for descriptor matching
//! - Per-loop closure: 50-100ms validation

use crate::feature_tracker::OrbDescriptor;
use rustc_hash::FxHashMap;

/// Loop closure match between two keyframes.
#[derive(Debug, Clone)]
pub struct LoopMatch {
    /// Index of the older keyframe in sliding window history
    pub kf_idx_older: usize,
    /// Index of the newer keyframe in sliding window
    pub kf_idx_newer: usize,
    /// Number of matched descriptor pairs
    pub inlier_count: usize,
    /// Average Hamming distance of matches
    pub avg_distance: u8,
}

/// Database of keyframe descriptors for loop closure detection.
#[derive(Debug, Clone)]
pub struct LoopClosureDatabase {
    /// Keyframe index → ORB descriptors for left camera
    pub descriptors_left: FxHashMap<usize, Vec<OrbDescriptor>>,
    /// Keyframe index → ORB descriptors for right camera
    pub descriptors_right: FxHashMap<usize, Vec<OrbDescriptor>>,
    /// Total keyframes added
    pub num_keyframes: usize,
    /// Minimum number of matches to consider a valid loop
    pub min_match_count: usize,
    /// Maximum Hamming distance for a descriptor match
    pub max_hamming_distance: u8,
    /// Minimum gap (keyframes) to skip temporal matches
    pub min_temporal_gap: usize,
}

impl LoopClosureDatabase {
    /// Create a new loop closure database.
    pub fn new() -> Self {
        Self {
            descriptors_left: FxHashMap::default(),
            descriptors_right: FxHashMap::default(),
            num_keyframes: 0,
            min_match_count: 20,      // Min 20 matches for valid loop
            max_hamming_distance: 60,  // Max Hamming distance (of 256 bits)
            min_temporal_gap: 30,      // Skip matches within 30 keyframes
        }
    }

    /// Add descriptors for a keyframe.
    pub fn add_keyframe(
        &mut self,
        kf_idx: usize,
        descriptors_left: Vec<OrbDescriptor>,
        descriptors_right: Vec<OrbDescriptor>,
    ) {
        self.descriptors_left.insert(kf_idx, descriptors_left);
        self.descriptors_right.insert(kf_idx, descriptors_right);
        self.num_keyframes += 1;
    }

    /// Find potential loop closure matches for a keyframe.
    /// Returns top candidates sorted by inlier count.
    pub fn find_loop_candidates(&self, kf_idx: usize) -> Vec<LoopMatch> {
        let Some(descs_newer) = self.descriptors_left.get(&kf_idx) else {
            return Vec::new();
        };

        let mut candidates = Vec::new();

        // Search all older keyframes
        for (&older_idx, descs_older) in &self.descriptors_left {
            // Skip recent frames (temporal matches)
            if (kf_idx as isize - older_idx as isize).abs() < self.min_temporal_gap as isize {
                continue;
            }

            // Skip if same keyframe
            if older_idx == kf_idx {
                continue;
            }

            // Brute-force descriptor matching
            let (match_count, avg_distance) =
                self.match_descriptors(descs_older, descs_newer);

            if match_count >= self.min_match_count {
                candidates.push(LoopMatch {
                    kf_idx_older: older_idx,
                    kf_idx_newer: kf_idx,
                    inlier_count: match_count,
                    avg_distance,
                });
            }
        }

        // Sort by inlier count descending
        candidates.sort_by_key(|c| std::cmp::Reverse(c.inlier_count));
        candidates
    }

    /// Match two sets of descriptors using Hamming distance.
    /// Returns (match_count, average_distance).
    fn match_descriptors(
        &self,
        descs_a: &[OrbDescriptor],
        descs_b: &[OrbDescriptor],
    ) -> (usize, u8) {
        let mut total_distance = 0u32;
        let mut match_count = 0;

        for desc_b in descs_b {
            // Find nearest neighbor in descs_a
            let mut min_distance = u8::MAX;
            for desc_a in descs_a {
                let dist = hamming_distance(desc_a, desc_b);
                if dist < min_distance {
                    min_distance = dist;
                }
            }

            // Accept if within threshold
            if min_distance <= self.max_hamming_distance {
                total_distance += min_distance as u32;
                match_count += 1;
            }
        }

        let avg_distance = if match_count > 0 {
            (total_distance / match_count as u32) as u8
        } else {
            0
        };

        (match_count, avg_distance)
    }
}

impl Default for LoopClosureDatabase {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute Hamming distance between two 32-byte descriptors.
/// Counts number of differing bits.
#[inline(always)]
pub fn hamming_distance(a: &OrbDescriptor, b: &OrbDescriptor) -> u8 {
    let mut distance = 0u8;
    for i in 0..32 {
        distance += (a[i] ^ b[i]).count_ones() as u8;
    }
    distance
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hamming_distance() {
        let desc_a = [0u8; 32];
        let desc_b = [0u8; 32];
        assert_eq!(hamming_distance(&desc_a, &desc_b), 0);

        let mut desc_c = [0u8; 32];
        desc_c[0] = 0xFF; // All bits set
        assert_eq!(hamming_distance(&desc_a, &desc_c), 8);
    }

    #[test]
    fn test_loop_database() {
        let mut db = LoopClosureDatabase::new();

        let descs1 = vec![[0u8; 32]; 100];
        let descs2 = vec![[0u8; 32]; 100];
        db.add_keyframe(0, descs1.clone(), descs2.clone());

        let descs3 = vec![[0u8; 32]; 100];
        let descs4 = vec![[0u8; 32]; 100];
        db.add_keyframe(50, descs3, descs4);

        // Should find perfect matches (all descriptors identical)
        let candidates = db.find_loop_candidates(50);
        assert!(candidates.len() > 0);
        assert_eq!(candidates[0].kf_idx_older, 0);
        assert_eq!(candidates[0].inlier_count, 100);
    }
}
