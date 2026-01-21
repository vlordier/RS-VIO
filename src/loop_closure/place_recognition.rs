//! Place recognition using descriptor hashing and similarity search
//!
//! Implements fast place recognition for loop closure detection using
//! hierarchical hashing of SuperPoint descriptors from Phase 6.
//!
//! References:
//! - DeLone et al., "SuperPoint: Self-Supervised Interest Point Detection and Description"
//! - Ge et al., "Exploring Simple Siamese Representation Learning"

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Configuration for place recognition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlaceRecognitionConfig {
    /// Minimum temporal distance between candidate frames (in frame count)
    pub min_temporal_distance: u32,
    /// Number of top candidates to return for verification
    pub top_k_candidates: usize,
    /// Similarity threshold for considering a frame as candidate
    pub similarity_threshold: f32,
    /// Hash table size for descriptor indexing
    pub descriptor_hash_size: usize,
    /// Number of hash functions for LSH
    pub num_hash_functions: u32,
    /// Dimension reduction for efficient hashing
    pub hash_dimension: usize,
}

impl Default for PlaceRecognitionConfig {
    fn default() -> Self {
        Self {
            min_temporal_distance: 30,     // At least 30 frames apart
            top_k_candidates: 10,           // Test top 10 candidates
            similarity_threshold: 0.5,      // Min 50% descriptor match
            descriptor_hash_size: 100_000,  // Hash table size
            num_hash_functions: 8,          // 8 independent hash functions
            hash_dimension: 128,            // Hash to 128-bit codes
        }
    }
}

/// Statistics about a place (keyframe aggregate)
#[derive(Debug, Clone, Copy)]
pub struct PlaceStatistics {
    pub keyframe_id: u32,
    pub num_descriptors: u32,
    pub descriptor_mean: f32,
    pub descriptor_variance: f32,
}

/// Candidate loop closure with similarity score
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopCandidate {
    pub candidate_keyframe_id: u32,
    pub similarity_score: f32,
    pub temporal_distance: u32,
    pub descriptor_match_count: u32,
}

impl PartialOrd for LoopCandidate {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.similarity_score.partial_cmp(&other.similarity_score)
    }
}

/// Hash bucket for descriptor indexing
#[derive(Debug, Clone)]
struct HashBucket {
    keyframe_id: u32,
    descriptor_hash: Vec<u64>,
}

/// Place recognition database using descriptor hashing
pub struct PlaceRecognitionDatabase {
    config: PlaceRecognitionConfig,
    /// Store of keyframes with hashed descriptors
    index: Vec<HashBucket>,
    /// Map from keyframe ID to index position
    keyframe_map: HashMap<u32, usize>,
    /// Statistics per keyframe for faster similarity estimation
    statistics: HashMap<u32, PlaceStatistics>,
    /// Current keyframe counter
    total_keyframes: u32,
}

impl PlaceRecognitionDatabase {
    /// Create new place recognition database
    pub fn new(config: PlaceRecognitionConfig) -> Self {
        Self {
            config,
            index: Vec::new(),
            keyframe_map: HashMap::new(),
            statistics: HashMap::new(),
            total_keyframes: 0,
        }
    }

    /// Add a new keyframe with descriptors to the database
    pub fn add_keyframe(
        &mut self,
        keyframe_id: u32,
        descriptors: &[Vec<f32>],
    ) -> Result<(), String> {
        if descriptors.is_empty() {
            return Err("No descriptors provided".to_string());
        }

        // Compute statistics
        let stats = compute_descriptor_statistics(descriptors);

        // Hash descriptors
        let hashes = self.hash_descriptors(descriptors);

        // Store in index
        let index_pos = self.index.len();
        self.index.push(HashBucket {
            keyframe_id,
            descriptor_hash: hashes,
        });

        // Update maps
        self.keyframe_map.insert(keyframe_id, index_pos);
        self.statistics.insert(keyframe_id, stats);
        self.total_keyframes += 1;

        Ok(())
    }

    /// Query for loop closure candidates
    pub fn query_candidates(&self, descriptors: &[Vec<f32>]) -> Result<Vec<LoopCandidate>, String> {
        if descriptors.is_empty() {
            return Err("No descriptors provided for query".to_string());
        }

        let query_hashes = self.hash_descriptors(descriptors);
        let mut candidates = Vec::new();

        // Find similar keyframes via hash collision
        let mut hash_matches: HashMap<u32, u32> = HashMap::new();

        for query_hash in &query_hashes {
            for bucket in &self.index {
                for bucket_hash in &bucket.descriptor_hash {
                    // Count matching hashes (Hamming distance threshold)
                    if hamming_distance(*query_hash, *bucket_hash) < 16 {
                        *hash_matches.entry(bucket.keyframe_id).or_insert(0) += 1;
                    }
                }
            }
        }

        // Convert matches to candidates
        let current_frame_id = self.total_keyframes;

        for (keyframe_id, match_count) in hash_matches {
            // Temporal distance: how many frames ago was this keyframe?
            let temporal_distance = if current_frame_id > keyframe_id {
                current_frame_id - keyframe_id
            } else {
                continue; // Skip if keyframe is in future (shouldn't happen)
            };

            // Filter by temporal distance
            if temporal_distance < self.config.min_temporal_distance {
                continue;
            }

            // Compute similarity score (normalized match count)
            let max_possible_matches = (descriptors.len() * self.config.num_hash_functions as usize)
                as u32;
            let similarity_score = match_count as f32 / max_possible_matches as f32;

            // Filter by similarity threshold
            if similarity_score < self.config.similarity_threshold {
                continue;
            }

            candidates.push(LoopCandidate {
                candidate_keyframe_id: keyframe_id,
                similarity_score,
                temporal_distance,
                descriptor_match_count: match_count,
            });
        }

        // Sort by similarity (descending)
        candidates.sort_by(|a, b| {
            b.similarity_score
                .partial_cmp(&a.similarity_score)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Return top-k
        Ok(candidates.into_iter().take(self.config.top_k_candidates).collect())
    }

    /// Get best candidate (highest similarity)
    pub fn best_candidate(&self, descriptors: &[Vec<f32>]) -> Result<Option<LoopCandidate>, String> {
        let mut candidates = self.query_candidates(descriptors)?;
        Ok(candidates.pop())
    }

    /// Get database statistics
    pub fn statistics(&self) -> DatabaseStatistics {
        DatabaseStatistics {
            total_keyframes: self.total_keyframes,
            index_size: self.index.len(),
            memory_usage_bytes: self.estimate_memory_usage(),
        }
    }

    /// Hash descriptors using multiple hash functions
    fn hash_descriptors(&self, descriptors: &[Vec<f32>]) -> Vec<u64> {
        let mut hashes = Vec::new();

        for descriptor in descriptors {
            for hash_fn_id in 0..self.config.num_hash_functions {
                let hash = self.descriptor_to_hash(descriptor, hash_fn_id as usize);
                hashes.push(hash);
            }
        }

        hashes
    }

    /// Convert descriptor to hash code using random projection
    fn descriptor_to_hash(&self, descriptor: &[f32], hash_fn_id: usize) -> u64 {
        if descriptor.is_empty() {
            return 0u64;
        }

        // Simple hash: sum of descriptor elements weighted by hash function
        let mut hash_value: f32 = 0.0;
        let seed = (hash_fn_id * 73) as u32; // Deterministic seed per hash function

        for (i, &val) in descriptor.iter().enumerate() {
            // Weight by hash function ID and position
            let weight = ((seed.wrapping_mul(i as u32 + 1)) % 1000) as f32 / 1000.0;
            hash_value += val * weight;
        }

        // Convert to 64-bit hash
        let normalized = (hash_value.abs() % 1.0) * (u64::MAX as f32);
        normalized as u64
    }

    /// Estimate memory usage in bytes
    fn estimate_memory_usage(&self) -> usize {
        let mut total = 0;

        // Index size
        total += self.index.len() * std::mem::size_of::<HashBucket>();
        for bucket in &self.index {
            total += bucket.descriptor_hash.len() * std::mem::size_of::<u64>();
        }

        // Maps
        total += self.keyframe_map.len() * (std::mem::size_of::<u32>() + std::mem::size_of::<usize>());
        total += self.statistics.len() * (std::mem::size_of::<u32>() + std::mem::size_of::<PlaceStatistics>());

        total
    }
}

impl Default for PlaceRecognitionDatabase {
    fn default() -> Self {
        Self::new(PlaceRecognitionConfig::default())
    }
}

/// Compute Hamming distance between two 64-bit hash codes
fn hamming_distance(a: u64, b: u64) -> u32 {
    (a ^ b).count_ones()
}

/// Compute descriptor statistics for a keyframe
fn compute_descriptor_statistics(descriptors: &[Vec<f32>]) -> PlaceStatistics {
    if descriptors.is_empty() {
        return PlaceStatistics {
            keyframe_id: 0,
            num_descriptors: 0,
            descriptor_mean: 0.0,
            descriptor_variance: 0.0,
        };
    }

    // Compute mean
    let mut sum = 0.0f32;
    let mut count = 0usize;

    for descriptor in descriptors {
        for &val in descriptor {
            sum += val;
            count += 1;
        }
    }

    let mean = if count > 0 { sum / count as f32 } else { 0.0 };

    // Compute variance
    let mut var_sum = 0.0f32;
    for descriptor in descriptors {
        for &val in descriptor {
            let diff = val - mean;
            var_sum += diff * diff;
        }
    }

    let variance = if count > 0 { var_sum / count as f32 } else { 0.0 };

    PlaceStatistics {
        keyframe_id: 0,
        num_descriptors: descriptors.len() as u32,
        descriptor_mean: mean,
        descriptor_variance: variance,
    }
}

/// Statistics about the place recognition database
#[derive(Debug, Clone, Copy)]
pub struct DatabaseStatistics {
    pub total_keyframes: u32,
    pub index_size: usize,
    pub memory_usage_bytes: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_descriptor() -> Vec<f32> {
        vec![0.1; 256]
    }

    fn create_test_descriptors(count: usize) -> Vec<Vec<f32>> {
        vec![create_test_descriptor(); count]
    }

    #[test]
    fn test_config_defaults() {
        let config = PlaceRecognitionConfig::default();
        assert_eq!(config.min_temporal_distance, 30);
        assert_eq!(config.top_k_candidates, 10);
        assert!(config.similarity_threshold > 0.0);
    }

    #[test]
    fn test_database_creation() {
        let db = PlaceRecognitionDatabase::new(PlaceRecognitionConfig::default());
        let stats = db.statistics();
        assert_eq!(stats.total_keyframes, 0);
    }

    #[test]
    fn test_add_keyframe() {
        let mut db = PlaceRecognitionDatabase::new(PlaceRecognitionConfig::default());
        let descriptors = create_test_descriptors(100);

        let result = db.add_keyframe(0, &descriptors);
        assert!(result.is_ok());

        let stats = db.statistics();
        assert_eq!(stats.total_keyframes, 1);
    }

    #[test]
    fn test_query_empty_database() {
        let db = PlaceRecognitionDatabase::new(PlaceRecognitionConfig::default());
        let descriptors = create_test_descriptors(100);

        let candidates = db.query_candidates(&descriptors);
        assert!(candidates.is_ok());
        assert_eq!(candidates.unwrap().len(), 0);
    }

    #[test]
    fn test_temporal_filtering() {
        let mut config = PlaceRecognitionConfig::default();
        config.min_temporal_distance = 50;
        let min_temp = config.min_temporal_distance;

        let mut db = PlaceRecognitionDatabase::new(config);

        // Add several keyframes
        for i in 0..100 {
            let descriptors = create_test_descriptors(50);
            db.add_keyframe(i, &descriptors).ok();
        }

        // Query from late keyframe
        let descriptors = create_test_descriptors(50);
        let candidates = db.query_candidates(&descriptors).unwrap();

        // Should filter temporal distance
        for candidate in &candidates {
            assert!(candidate.temporal_distance >= min_temp);
        }
    }

    #[test]
    fn test_similarity_ranking() {
        let mut db = PlaceRecognitionDatabase::new(PlaceRecognitionConfig::default());

        // Add diverse keyframes
        let desc1 = create_test_descriptors(100);
        db.add_keyframe(0, &desc1).ok();

        for i in 1..60 {
            let mut desc = create_test_descriptors(100);
            // Vary slightly
            for d in &mut desc {
                for v in d.iter_mut() {
                    *v = 0.1 + (i as f32 * 0.001);
                }
            }
            db.add_keyframe(i as u32, &desc).ok();
        }

        // Query should rank by similarity
        let query_desc = create_test_descriptors(100);
        let candidates = db.query_candidates(&query_desc).unwrap();

        // Verify sorted by similarity
        for i in 1..candidates.len() {
            assert!(candidates[i - 1].similarity_score >= candidates[i].similarity_score);
        }
    }

    #[test]
    fn test_database_statistics() {
        let mut db = PlaceRecognitionDatabase::new(PlaceRecognitionConfig::default());

        for i in 0..10 {
            let descriptors = create_test_descriptors(50);
            db.add_keyframe(i, &descriptors).ok();
        }

        let stats = db.statistics();
        assert_eq!(stats.total_keyframes, 10);
        assert!(stats.memory_usage_bytes > 0);
    }

    #[test]
    fn test_best_candidate() {
        let mut config = PlaceRecognitionConfig::default();
        config.min_temporal_distance = 10;
        let min_temp = config.min_temporal_distance;

        let mut db = PlaceRecognitionDatabase::new(config);

        let descriptors = create_test_descriptors(100);
        
        // Add keyframes sequentially
        for i in 0..50 {
            db.add_keyframe(i, &descriptors).ok();
        }

        let query_desc = create_test_descriptors(100);
        let best = db.best_candidate(&query_desc).unwrap();

        // Should return Some if candidates exist, with proper temporal distance
        if let Some(candidate) = best {
            assert!(candidate.temporal_distance >= min_temp);
        }
    }

    #[test]
    fn test_hamming_distance() {
        // Test Hamming distance computation
        let a = 0b1010;
        let b = 0b1100;
        assert_eq!(hamming_distance(a, b), 2); // Two differing bits
    }
}
