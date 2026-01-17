use super::{DescriptorMatcher, MatchMetrics};
use crate::optimization::loop_closure::KeyframeDescriptor;
use crate::optimization::loop_closure::descriptor_pool::OrbBinaryPool;
use crate::types::Float;
use std::sync::Arc;

/// ORB-based descriptor matcher using Hamming distance
pub struct OrbMatcher {
    /// Maximum Hamming distance threshold (0-256)
    pub max_hamming_distance: u32,
    /// Whether to apply Lowe's ratio test
    pub use_ratio_test: bool,
    /// Lowe's ratio threshold (typically 0.7)
    pub ratio_threshold: f64,
    /// Optional binary descriptor pool for reusing conversion buffers
    pub binary_pool: Option<Arc<OrbBinaryPool>>,
}

impl OrbMatcher {
    /// Create a new ORB matcher with default parameters
    pub fn new() -> Self {
        Self {
            max_hamming_distance: 64,
            use_ratio_test: true,
            ratio_threshold: 0.75,
            binary_pool: None,
        }
    }

    /// Create a new ORB matcher with optional binary descriptor pool
    pub fn new_with_pool(binary_pool: Option<Arc<OrbBinaryPool>>) -> Self {
        Self {
            max_hamming_distance: 64,
            use_ratio_test: true,
            ratio_threshold: 0.75,
            binary_pool,
        }
    }

    /// Convert keyframe descriptor (Vec<f64>) to ORB binary format for matching
    /// Assumes descriptor is either:
    /// - 32 bytes (256-bit binary ORB): directly usable
    /// - Or needs conversion from floating-point representation
    fn descriptor_to_binary(&self, desc: &[Float]) -> Result<[u8; 32], &'static str> {
        if desc.len() == 32 {
            // Already in binary format (32 bytes)
            let mut binary = [0u8; 32];
            for (i, &val) in desc.iter().enumerate() {
                binary[i] = (val as f64 * 255.0).min(255.0) as u8;
            }
            Ok(binary)
        } else if desc.len() >= 4 {
            // Convert from floating-point descriptor to binary representation
            let mut binary = [0u8; 32];
            let chunk_size = desc.len() / 32;

            for i in 0..32 {
                let chunk = &desc[i * chunk_size..((i + 1) * chunk_size).min(desc.len())];
                let avg = chunk.iter().map(|&x| x as f64).sum::<f64>() / chunk.len() as f64;
                binary[i] = (avg * 255.0).min(255.0) as u8;
            }
            Ok(binary)
        } else {
            Err("Descriptor too short for ORB binary conversion")
        }
    }
}

impl DescriptorMatcher for OrbMatcher {
    fn match_keyframes(
        &self,
        query: &KeyframeDescriptor,
        candidate: &KeyframeDescriptor,
    ) -> MatchMetrics {
        let overlapping_features = query.num_features.min(candidate.num_features);
        if overlapping_features == 0 {
            return MatchMetrics {
                similarity: 0.0,
                match_count: 0,
                match_ratio: 0.0,
            };
        }

        let binary_query = self.descriptor_to_binary(&query.descriptor);
        let binary_candidate = self.descriptor_to_binary(&candidate.descriptor);

        let (similarity, match_count) = match (binary_query, binary_candidate) {
            (Ok(query_bits), Ok(candidate_bits)) => {
                let hamming_distance: u32 = query_bits
                    .iter()
                    .zip(candidate_bits.iter())
                    .map(|(a, b)| (a ^ b).count_ones())
                    .sum();

                if hamming_distance > self.max_hamming_distance {
                    (0.0, 0)
                } else {
                    let normalized_distance = hamming_distance as Float / 256.0;
                    let similarity = (1.0 - normalized_distance).max(0.0);
                    let match_count =
                        (similarity * overlapping_features as Float).round() as usize;
                    (similarity, match_count.min(overlapping_features))
                }
            }
            _ => (0.0, 0),
        };

        let match_ratio = match_count as Float / overlapping_features as Float;

        MatchMetrics {
            similarity,
            match_count,
            match_ratio,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fl;

    #[test]
    fn descriptor_conversion_errors_and_bounds() {
        let matcher = OrbMatcher::new();
        assert!(matcher.descriptor_to_binary(&[]).is_err());
        assert!(matcher.descriptor_to_binary(&[0.0]).is_err());
        assert!(matcher.descriptor_to_binary(&vec![0.0; 32]).is_ok());
        assert!(matcher.descriptor_to_binary(&vec![1.0; 32]).is_ok());
        assert!(matcher.descriptor_to_binary(&vec![0.5; 1024]).is_ok());

        let extreme_desc = vec![
            fl!(-100.0),
            fl!(0.0),
            fl!(100.0),
            Float::INFINITY,
            Float::NEG_INFINITY,
            Float::NAN,
        ];
        assert!(matcher.descriptor_to_binary(&extreme_desc).is_ok());
    }

    #[test]
    fn descriptor_conversion_deterministic() {
        let matcher = OrbMatcher::new();
        let desc = vec![0.5; 32];
        let a = matcher.descriptor_to_binary(&desc).expect("convert desc");
        let b = matcher.descriptor_to_binary(&desc).expect("convert desc");
        assert_eq!(a, b);
    }
}