use super::{DescriptorMatcher, MatchMetrics};
use crate::optimization::loop_closure::descriptor_pool::OrbBinaryPool;
use crate::optimization::loop_closure::KeyframeDescriptor;
use crate::traits::Strategy;
use crate::types::Float;
use std::sync::Arc;

#[cfg(feature = "gpu")]
use crate::gpu::orb_matcher::{init_wgpu, GpuOrbMatcher};
#[cfg(feature = "gpu")]
use pollster::block_on;

/// ORB-based descriptor matcher using Hamming distance
#[derive(Debug, Clone)]
pub struct OrbMatcher {
    /// Maximum Hamming distance threshold (0-256)
    pub max_hamming_distance: u32,
    /// Whether to apply Lowe's ratio test
    pub use_ratio_test: bool,
    /// Lowe's ratio threshold (typically 0.7)
    pub ratio_threshold: f64,
    /// Optional binary descriptor pool for reusing conversion buffers
    pub binary_pool: Option<Arc<OrbBinaryPool>>,
    /// Optional GPU matcher for accelerating Hamming distance calculation
    #[cfg(feature = "gpu")]
    #[cfg_attr(not(feature = "gpu"), allow(dead_code))]
    // Allow dead code if GPU feature is not enabled
    gpu_matcher: Option<Arc<GpuOrbMatcher>>,
}

impl Default for OrbMatcher {
    fn default() -> Self {
        Self {
            max_hamming_distance: 64,
            use_ratio_test: true,
            ratio_threshold: 0.75,
            binary_pool: None,
            #[cfg(feature = "gpu")]
            gpu_matcher: None,
        }
    }
}

impl Strategy for OrbMatcher {
    fn name(&self) -> &str {
        "OrbMatcher"
    }
}

impl OrbMatcher {
    /// Create a new ORB matcher with default parameters
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a new ORB matcher with optional binary descriptor pool
    pub fn new_with_pool(binary_pool: Option<Arc<OrbBinaryPool>>) -> Self {
        Self {
            max_hamming_distance: 64,
            use_ratio_test: true,
            ratio_threshold: 0.75,
            binary_pool,
            #[cfg(feature = "gpu")]
            gpu_matcher: None,
        }
    }

    /// Creates a new ORB matcher, optionally initializing GPU acceleration if the "gpu" feature is enabled.
    /// This function is asynchronous because WGPU initialization is asynchronous.
    #[cfg(feature = "gpu")]
    pub async fn new_gpu_aware(binary_pool: Option<Arc<OrbBinaryPool>>) -> Self {
        let mut matcher = Self::new_with_pool(binary_pool);
        log::debug!("Attempting to initialize GPU-aware OrbMatcher.");
        match init_wgpu().await {
            Some((device, queue, _adapter)) => {
                log::info!("WGPU initialized successfully. Creating GpuOrbMatcher.");
                let gpu_matcher = Arc::new(GpuOrbMatcher::new(device.clone(), queue.clone()).await);
                matcher.gpu_matcher = Some(gpu_matcher);
                log::info!("GpuOrbMatcher created.");
            },
            None => {
                log::warn!("WGPU initialization failed. Falling back to CPU OrbMatcher.");
            },
        }
        matcher
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

        // Try GPU matching if enabled and available
        #[cfg(feature = "gpu")]
        if let Some(gpu_matcher) = &self.gpu_matcher {
            let binary_query_desc = self.descriptor_to_binary(&query.descriptor);
            let binary_candidate_desc = self.descriptor_to_binary(&candidate.descriptor);

            if let (Ok(q_desc), Ok(c_desc)) = (binary_query_desc, binary_candidate_desc) {
                // For simplicity, converting single descriptor to a slice of 1 for GPU batching
                let query_descs = &[q_desc];
                let target_descs = &[c_desc]; // Assuming candidate is one descriptor from DB

                let gpu_matches = block_on(gpu_matcher.match_descriptors(
                    query_descs,
                    target_descs,
                    self.max_hamming_distance,
                ));

                // Process the single match result from GPU
                if let Some((_, _, hamming_distance)) = gpu_matches.first() {
                    if *hamming_distance > self.max_hamming_distance {
                        return MatchMetrics {
                            similarity: 0.0,
                            match_count: 0,
                            match_ratio: 0.0,
                        };
                    } else {
                        let normalized_distance = *hamming_distance as Float / 256.0;
                        let similarity = (1.0 - normalized_distance).max(0.0);
                        let match_count =
                            (similarity * overlapping_features as Float).round() as usize;
                        let match_ratio = match_count as Float / overlapping_features as Float;
                        return MatchMetrics {
                            similarity,
                            match_count: match_count.min(overlapping_features),
                            match_ratio,
                        };
                    }
                }
            }
        }

        // Fallback to CPU matching (or if GPU not enabled/available)
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
                    let match_count = (similarity * overlapping_features as Float).round() as usize;
                    (similarity, match_count.min(overlapping_features))
                }
            },
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
        assert!(matcher.descriptor_to_binary(&[0.0; 32]).is_ok());
        assert!(matcher.descriptor_to_binary(&[1.0; 32]).is_ok());
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
