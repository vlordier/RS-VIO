use super::{DescriptorMatcher, MatchMetrics};
use crate::optimization::loop_closure::KeyframeDescriptor;
use crate::types::Float;

/// ORB-based descriptor matcher using Hamming distance
pub struct OrbMatcher {
    /// Maximum Hamming distance threshold (0-256)
    pub max_hamming_distance: u32,
    /// Whether to apply Lowe's ratio test
    pub use_ratio_test: bool,
    /// Lowe's ratio threshold (typically 0.7)
    pub ratio_threshold: f64,
}

impl OrbMatcher {
    /// Create a new ORB matcher with default parameters
    pub fn new() -> Self {
        Self {
            max_hamming_distance: 64,
            use_ratio_test: true,
            ratio_threshold: 0.75,
        }
    }

    /// Convert keyframe descriptor (Vec<f64>) to ORB binary format for matching
    /// Assumes descriptor is either:
    /// - 32 bytes (256-bit binary ORB): directly usable
    /// - Or needs conversion from floating-point representation
    fn descriptor_to_binary(&self, desc: &[Float]) -> Option<[u8; 32]> {
        if desc.len() == 32 {
            // Already in binary format (32 bytes)
            let mut binary = [0u8; 32];
            for (i, &val) in desc.iter().enumerate() {
                binary[i] = (val as f64 * 255.0).min(255.0) as u8;
            }
            Some(binary)
        } else if desc.len() >= 4 {
            // Convert from floating-point descriptor to binary representation
            let mut binary = [0u8; 32];
            let chunk_size = desc.len() / 32;

            for i in 0..32 {
                let chunk = &desc[i * chunk_size..((i + 1) * chunk_size).min(desc.len())];
                let avg = chunk.iter().map(|&x| x as f64).sum::<f64>() / chunk.len() as f64;
                binary[i] = (avg * 255.0).min(255.0) as u8;
            }
            Some(binary)
        } else {
            None
        }
    }
}

impl Default for OrbMatcher {
    fn default() -> Self {
        Self::new()
    }
}

impl DescriptorMatcher for OrbMatcher {
    fn match_keyframes(
        &self,
        query: &KeyframeDescriptor,
        candidate: &KeyframeDescriptor,
    ) -> MatchMetrics {
        let query_binary = match self.descriptor_to_binary(&query.descriptor) {
            Some(b) => b,
            None => {
                log::warn!("[OrbMatcher] Failed to convert query descriptor");
                return MatchMetrics {
                    similarity: 0.0,
                    match_count: 0,
                    match_ratio: 0.0,
                };
            },
        };

        let candidate_binary = match self.descriptor_to_binary(&candidate.descriptor) {
            Some(b) => b,
            None => {
                log::warn!("[OrbMatcher] Failed to convert candidate descriptor");
                return MatchMetrics {
                    similarity: 0.0,
                    match_count: 0,
                    match_ratio: 0.0,
                };
            },
        };

        // Compute Hamming distance (0 = identical, 256 = completely different)
        let hamming_dist: u32 = query_binary
            .iter()
            .zip(candidate_binary.iter())
            .map(|(a, b)| (a ^ b).count_ones())
            .sum();

        // Convert to similarity (0-1, higher = more similar)
        // Similarity = 1 - (hamming_dist / 256)
        let similarity = 1.0 - (hamming_dist as f64 / 256.0);

        // Number of matches is heuristic based on Hamming distance
        // Lower distance = more matches
        let match_count = if hamming_dist < self.max_hamming_distance {
            ((1.0 - hamming_dist as f64 / 256.0) * query.num_features as f64) as usize
        } else {
            0
        };

        let overlapping_features = query.num_features.min(candidate.num_features).max(1);
        let match_ratio = if overlapping_features > 0 {
            match_count as f64 / overlapping_features as f64
        } else {
            0.0
        };

        MatchMetrics {
            similarity: similarity as Float,
            match_count,
            match_ratio: match_ratio as Float,
        }
    }
}

#[cfg(test)]
#[allow(clippy::needless_range_loop)]
mod tests {
    use super::*;
    use crate::fl;
    use nalgebra::Isometry3;

    fn create_test_descriptor(seed: u64) -> KeyframeDescriptor {
        // Create a deterministic descriptor from seed
        let mut desc = vec![fl!(0.0); 32];
        for i in 0..32 {
            desc[i] = ((seed as Float * (i as Float + fl!(1.0))).sin() + fl!(1.0)) / fl!(2.0);
        }

        KeyframeDescriptor {
            keyframe_id: seed,
            timestamp: seed as i64 * 1_000_000,
            descriptor: desc,
            num_features: 100,
            pose: Isometry3::identity(),
        }
    }

    #[test]
    fn orb_matcher_identical_descriptors() {
        let matcher = OrbMatcher::new();
        let desc1 = create_test_descriptor(42);
        let desc2 = desc1.clone();

        let metrics = matcher.match_keyframes(&desc1, &desc2);
        assert!(
            metrics.similarity > 0.99,
            "Identical descriptors should have ~100% similarity"
        );
    }

    #[test]
    fn orb_matcher_different_descriptors() {
        let matcher = OrbMatcher::new();
        let desc1 = create_test_descriptor(42);
        let desc2 = create_test_descriptor(100);

        let metrics = matcher.match_keyframes(&desc1, &desc2);
        assert!(
            metrics.similarity < 0.7,
            "Different descriptors should have lower similarity"
        );
    }

    #[test]
    fn orb_matcher_hamming_distance_threshold() {
        let matcher = OrbMatcher::new();
        let desc1 = create_test_descriptor(42);
        let mut desc2 = create_test_descriptor(42);

        // Flip some bits to simulate higher Hamming distance
        desc2.descriptor[0] = 1.0 - desc2.descriptor[0];

        let metrics = matcher.match_keyframes(&desc1, &desc2);
        assert!(
            metrics.similarity > 0.0,
            "Should have some similarity even with bit flips"
        );
    }

    #[test]
    fn orb_matcher_match_count_bounded() {
        let matcher = OrbMatcher::new();
        let desc1 = create_test_descriptor(42);
        let desc2 = create_test_descriptor(100);

        let metrics = matcher.match_keyframes(&desc1, &desc2);
        assert!(
            metrics.match_count <= desc1.num_features,
            "Match count should be ≤ num_features"
        );
        assert!(metrics.match_ratio <= 1.0, "Match ratio should be ≤ 1.0");
    }

    #[test]
    fn orb_matcher_converts_floating_point_descriptor() {
        let matcher = OrbMatcher::new();
        let desc = vec![0.5; 32];

        let binary = matcher.descriptor_to_binary(&desc);
        assert!(
            binary.is_some(),
            "Should convert 32-element float descriptor"
        );

        if let Some(b) = binary {
            assert_eq!(b.len(), 32);
        }
    }

    #[test]
    fn orb_matcher_handles_variable_length_descriptors() {
        let matcher = OrbMatcher::new();

        // Test various descriptor lengths
        for len in [16, 32, 64, 128].iter() {
            let desc = vec![0.5; *len];
            let binary = matcher.descriptor_to_binary(&desc);
            assert!(
                binary.is_some(),
                "Should handle {}-element descriptors",
                len
            );
        }
    }
}
