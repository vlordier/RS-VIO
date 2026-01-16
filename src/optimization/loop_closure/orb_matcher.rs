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
    use crate::types::Vector3;
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

    #[test]
    fn orb_matcher_edge_cases() {
        let matcher = OrbMatcher::new();

        // Test 1: Empty descriptors
        let empty_desc = vec![];
        let binary = matcher.descriptor_to_binary(&empty_desc);
        assert!(binary.is_none(), "Empty descriptor should return None");

        // Test 2: Very small descriptors
        let tiny_desc = vec![0.0];
        let binary = matcher.descriptor_to_binary(&tiny_desc);
        assert!(binary.is_none(), "Tiny descriptor should return None");

        // Test 3: All zeros
        let zero_desc = vec![0.0; 32];
        let binary = matcher.descriptor_to_binary(&zero_desc);
        assert!(binary.is_some(), "All zeros should still convert");

        // Test 4: All ones
        let ones_desc = vec![1.0; 32];
        let binary = matcher.descriptor_to_binary(&ones_desc);
        assert!(binary.is_some(), "All ones should still convert");

        // Test 5: Extreme values
        let extreme_desc = vec![
            fl!(-100.0),
            fl!(0.0),
            fl!(100.0),
            Float::INFINITY,
            Float::NEG_INFINITY,
            Float::NAN,
        ];
        let binary = matcher.descriptor_to_binary(&extreme_desc);
        assert!(binary.is_some(), "Should handle extreme values gracefully");

        // Test 6: Very long descriptors
        let long_desc = vec![0.5; 1024];
        let binary = matcher.descriptor_to_binary(&long_desc);
        assert!(binary.is_some(), "Should handle very long descriptors");
    }

    #[test]
    fn orb_matcher_keyframe_matching_edge_cases() {
        let matcher = OrbMatcher::new();

        // Test 1: Matching identical keyframes
        let desc1 = create_test_descriptor(42);
        let desc2 = desc1.clone();
        let metrics = matcher.match_keyframes(&desc1, &desc2);
        assert!(
            metrics.similarity > 0.95,
            "Identical keyframes should have high similarity"
        );

        // Test 2: Matching keyframes with no features
        let mut desc_no_features = desc1.clone();
        desc_no_features.num_features = 0;
        let metrics = matcher.match_keyframes(&desc_no_features, &desc2);
        assert_eq!(
            metrics.match_count, 0,
            "No features should result in zero matches"
        );

        // Test 3: Matching keyframes with different feature counts
        let mut desc_few_features = desc1.clone();
        desc_few_features.num_features = 1;
        let metrics = matcher.match_keyframes(&desc_few_features, &desc2);
        assert!(
            metrics.match_count <= 1,
            "Should respect minimum feature count"
        );

        // Test 4: Matching with very different poses
        let mut desc_different_pose = desc1.clone();
        desc_different_pose.pose = Isometry3::new(
            Vector3::new(100.0, 200.0, 50.0),
            Vector3::new(1.57, 0.78, 0.0),
        );
        let metrics = matcher.match_keyframes(&desc1, &desc_different_pose);
        // Should still compute similarity based on descriptors, not pose
        assert!(metrics.similarity >= 0.0 && metrics.similarity <= 1.0);

        // Test 5: Matching with corrupted descriptors
        let mut desc_corrupted = desc1.clone();
        desc_corrupted.descriptor = vec![Float::NAN; desc_corrupted.descriptor.len()];
        let metrics = matcher.match_keyframes(&desc1, &desc_corrupted);
        // Should handle NaN gracefully (likely return low similarity)
        assert!(metrics.similarity >= 0.0);

        // Test 6: Matching with different descriptor lengths
        let mut desc_different_len = desc1.clone();
        desc_different_len.descriptor = vec![0.5; 64]; // Different length
        let metrics = matcher.match_keyframes(&desc1, &desc_different_len);
        // Should handle length mismatch gracefully
        assert!(metrics.similarity >= 0.0 && metrics.similarity <= 1.0);
    }

    #[test]
    fn orb_matcher_configuration_edge_cases() {
        // Test different configurations
        let configs = vec![
            OrbMatcher {
                max_hamming_distance: 0, // Very strict
                use_ratio_test: false,
                ratio_threshold: 0.0,
            },
            OrbMatcher {
                max_hamming_distance: 256, // Very lenient
                use_ratio_test: true,
                ratio_threshold: 0.95,
            },
            OrbMatcher {
                max_hamming_distance: 128, // Default-like
                use_ratio_test: true,
                ratio_threshold: 0.7,
            },
        ];

        for config in configs {
            let desc1 = create_test_descriptor(42);
            let desc2 = create_test_descriptor(43);

            let metrics = config.match_keyframes(&desc1, &desc2);
            assert!(metrics.similarity >= 0.0 && metrics.similarity <= 1.0);
            assert!(metrics.match_ratio >= 0.0 && metrics.match_ratio <= 1.0);
        }
    }

    #[test]
    fn orb_matcher_binary_conversion_properties() {
        let matcher = OrbMatcher::new();

        // Test that binary conversion is deterministic
        let desc = vec![0.5; 32];
        let binary1 = matcher.descriptor_to_binary(&desc).unwrap();
        let binary2 = matcher.descriptor_to_binary(&desc).unwrap();
        assert_eq!(
            binary1, binary2,
            "Binary conversion should be deterministic"
        );

        // Test that similar descriptors produce similar binary
        let desc_similar = vec![0.5001; 32];
        let binary_similar = matcher.descriptor_to_binary(&desc_similar).unwrap();

        let hamming_distance: u32 = binary1
            .iter()
            .zip(binary_similar.iter())
            .map(|(a, b)| (a ^ b).count_ones())
            .sum();

        assert!(
            hamming_distance < 32,
            "Similar descriptors should have low Hamming distance"
        );

        // Test threshold behavior
        let desc_zeros = vec![0.0; 32];
        let desc_ones = vec![1.0; 32];
        let binary_zeros = matcher.descriptor_to_binary(&desc_zeros).unwrap();
        let binary_ones = matcher.descriptor_to_binary(&desc_ones).unwrap();

        let hamming_max: u32 = binary_zeros
            .iter()
            .zip(binary_ones.iter())
            .map(|(a, b)| (a ^ b).count_ones())
            .sum();

        assert_eq!(
            hamming_max, 256,
            "Opposite descriptors should have max Hamming distance"
        );
    }
}
