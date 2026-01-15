//! Bag-of-Words (BoW) vocabulary for efficient loop closure candidate retrieval.
//!
//! This module implements a vocabulary-based approach for fast loop closure detection:
//! - K-means clustering of ORB descriptors to create visual words
//! - Inverted index mapping words to keyframes
//! - TF-IDF scoring for candidate ranking
//!
//! References:
//! - DBoW2: Appearance-based SLAM with bag of visual words (Rubio et al., 2012)
//! - FBoW: Fast Bag of Words (Galvez-Lopez & Tardos, 2018)

use std::collections::{HashMap, BTreeMap};
use crate::Result;

/// Configuration for BoW vocabulary
#[derive(Debug, Clone)]
pub struct VocabularyConfig {
    /// Number of visual words (vocabulary size)
    pub num_words: usize,
    /// Number of clusters for k-means
    pub num_clusters: usize,
    /// Maximum number of k-means iterations
    pub max_iterations: usize,
    /// Convergence threshold for k-means
    pub convergence_threshold: f64,
}

impl Default for VocabularyConfig {
    fn default() -> Self {
        Self {
            num_words: 10000,
            num_clusters: 10000,
            max_iterations: 100,
            convergence_threshold: 1e-4,
        }
    }
}

/// Visual word (cluster center in descriptor space)
#[derive(Debug, Clone)]
pub struct VisualWord {
    /// Word ID (index in vocabulary)
    pub word_id: usize,
    /// Cluster center (256-bit descriptor as bytes)
    pub center: Vec<u8>,
    /// Number of training descriptors in this cluster
    pub count: usize,
    /// Inverse document frequency (IDF) weight
    pub idf: f64,
}

/// BoW vocabulary built from training descriptors
#[derive(Debug, Clone)]
pub struct Vocabulary {
    #[allow(dead_code)]
    config: VocabularyConfig,
    words: Vec<VisualWord>,
    // Inverted index: word_id -> list of (keyframe_id, weight)
    inverted_index: HashMap<usize, Vec<(u64, f64)>>,
    // Document frequency for IDF computation
    document_frequency: HashMap<usize, usize>,
    total_documents: usize,
}

impl Vocabulary {
    /// Create a new empty vocabulary
    pub fn new(config: VocabularyConfig) -> Self {
        Self {
            config,
            words: Vec::new(),
            inverted_index: HashMap::new(),
            document_frequency: HashMap::new(),
            total_documents: 0,
        }
    }

    /// Build vocabulary from training descriptors using k-means clustering
    pub fn build_from_descriptors(
        config: VocabularyConfig,
        descriptors: Vec<Vec<u8>>,
    ) -> Result<Self> {
        if descriptors.is_empty() {
            return Ok(Self::new(config));
        }

        log::info!(
            "[Vocabulary] Building vocabulary from {} descriptors with {} clusters",
            descriptors.len(),
            config.num_clusters
        );

        // Convert byte descriptors to float for clustering
        let float_descriptors: Vec<Vec<f64>> = descriptors
            .iter()
            .map(|desc| {
                desc.iter()
                    .map(|&byte| byte as f64)
                    .collect()
            })
            .collect();

        // Run k-means clustering
        let centers = Self::kmeans_clustering(
            &float_descriptors,
            config.num_clusters,
            config.max_iterations,
            config.convergence_threshold,
        )?;

        // Create visual words from cluster centers
        let mut words = Vec::new();

        for (word_id, center) in centers.iter().enumerate() {
            // Convert center back to bytes
            let center_bytes: Vec<u8> = center
                .iter()
                .map(|&val| (val.max(0.0).min(255.0)) as u8)
                .collect();

            words.push(VisualWord {
                word_id,
                center: center_bytes,
                count: 0,
                idf: 0.0, // Will be set after adding keyframes
            });
        }

        let vocab = Self {
            config,
            words,
            inverted_index: HashMap::new(),
            document_frequency: HashMap::new(),
            total_documents: 0,
        };

        log::debug!("[Vocabulary] Built vocabulary with {} words", vocab.words.len());
        Ok(vocab)
    }

    /// Quantize a descriptor to the nearest visual word ID
    pub fn quantize(&self, descriptor: &[u8]) -> usize {
        if self.words.is_empty() {
            return 0;
        }

        let desc_float: Vec<f64> = descriptor
            .iter()
            .map(|&byte| byte as f64)
            .collect();

        let mut best_word = 0;
        let mut best_distance = f64::INFINITY;

        for (word_id, word) in self.words.iter().enumerate() {
            let word_float: Vec<f64> = word.center
                .iter()
                .map(|&byte| byte as f64)
                .collect();
            let distance = Self::hamming_distance_float(&desc_float, &word_float);
            if distance < best_distance {
                best_distance = distance;
                best_word = word_id;
            }
        }

        best_word
    }

    /// Add keyframe to inverted index with its BoW histogram
    pub fn add_keyframe(&mut self, keyframe_id: u64, words: Vec<usize>) {
        self.total_documents += 1;

        // Count word occurrences (term frequency)
        let mut word_count: HashMap<usize, usize> = HashMap::new();
        for word_id in words {
            *word_count.entry(word_id).or_insert(0) += 1;
        }

        // Add to inverted index and update document frequency
        for (word_id, count) in word_count {
            *self.document_frequency.entry(word_id).or_insert(0) += 1;

            // TF (normalized by number of words)
            let tf = count as f64 / (word_id + 1) as f64; // Simple normalization

            // Add to inverted index
            self.inverted_index
                .entry(word_id)
                .or_insert_with(Vec::new)
                .push((keyframe_id, tf));
        }
    }

    /// Compute TF-IDF weighted BoW histogram for a descriptor list
    pub fn compute_histogram(&self, words: Vec<usize>) -> BTreeMap<usize, f64> {
        let mut histogram: HashMap<usize, usize> = HashMap::new();

        for word_id in words {
            *histogram.entry(word_id).or_insert(0) += 1;
        }

        let mut result = BTreeMap::new();
        let total_words = histogram.len() as f64;

        for (word_id, count) in histogram {
            // TF: normalized term frequency
            let tf = count as f64 / total_words;

            // IDF: inverse document frequency
            let df = *self.document_frequency.get(&word_id).unwrap_or(&1) as f64;
            let idf = ((self.total_documents as f64) / (df + 1.0)).ln();

            // TF-IDF score
            let score = tf * idf;
            result.insert(word_id, score);
        }

        result
    }

    /// Compute similarity between two BoW histograms using L2 norm
    pub fn histogram_similarity(
        hist1: &BTreeMap<usize, f64>,
        hist2: &BTreeMap<usize, f64>,
    ) -> f64 {
        let mut dot_product = 0.0;
        let mut norm1 = 0.0;
        let mut norm2 = 0.0;

        // Compute norms and dot product
        for (word_id, score1) in hist1 {
            norm1 += score1 * score1;
            if let Some(score2) = hist2.get(word_id) {
                dot_product += score1 * score2;
            }
        }

        for (_, score2) in hist2 {
            norm2 += score2 * score2;
        }

        if norm1 == 0.0 || norm2 == 0.0 {
            return 0.0;
        }

        dot_product / (norm1.sqrt() * norm2.sqrt())
    }

    /// K-means clustering for vocabulary building
    fn kmeans_clustering(
        descriptors: &[Vec<f64>],
        num_clusters: usize,
        max_iterations: usize,
        convergence_threshold: f64,
    ) -> Result<Vec<Vec<f64>>> {
        if descriptors.is_empty() {
            return Ok(Vec::new());
        }

        let dim = descriptors[0].len();
        let mut centers = Self::initialize_centers(descriptors, num_clusters);

        for iteration in 0..max_iterations {
            // Assign points to nearest cluster
            let mut assignments = vec![Vec::new(); num_clusters];
            for (point_idx, point) in descriptors.iter().enumerate() {
                let mut best_cluster = 0;
                let mut best_distance = f64::INFINITY;

                for (cluster_idx, center) in centers.iter().enumerate() {
                    let distance = Self::euclidean_distance(point, center);
                    if distance < best_distance {
                        best_distance = distance;
                        best_cluster = cluster_idx;
                    }
                }

                assignments[best_cluster].push(point_idx);
            }

            // Recompute centers
            let mut new_centers = centers.clone();
            for (cluster_idx, points) in assignments.iter().enumerate() {
                if points.is_empty() {
                    continue;
                }

                let mut new_center = vec![0.0; dim];
                for &point_idx in points {
                    for (d, &val) in descriptors[point_idx].iter().enumerate() {
                        new_center[d] += val;
                    }
                }

                for val in &mut new_center {
                    *val /= points.len() as f64;
                }

                new_centers[cluster_idx] = new_center;
            }

            // Check convergence
            let max_shift = centers
                .iter()
                .zip(new_centers.iter())
                .map(|(old, new)| Self::euclidean_distance(old, new))
                .fold(0.0, f64::max);

            centers = new_centers;

            if max_shift < convergence_threshold {
                log::debug!(
                    "[Vocabulary] K-means converged after {} iterations",
                    iteration + 1
                );
                break;
            }
        }

        Ok(centers)
    }

    /// Initialize cluster centers using random selection
    fn initialize_centers(descriptors: &[Vec<f64>], num_clusters: usize) -> Vec<Vec<f64>> {
        use std::cmp::min;

        let num_clusters = min(num_clusters, descriptors.len());
        let step = descriptors.len() / num_clusters;

        (0..num_clusters)
            .map(|i| descriptors[i * step].clone())
            .collect()
    }

    /// Compute Euclidean distance between two vectors (float)
    fn euclidean_distance(a: &[f64], b: &[f64]) -> f64 {
        a.iter()
            .zip(b.iter())
            .map(|(x, y)| (x - y).powi(2))
            .sum::<f64>()
            .sqrt()
    }

    /// Compute Hamming distance between two float descriptors
    fn hamming_distance_float(a: &[f64], b: &[f64]) -> f64 {
        a.iter()
            .zip(b.iter())
            .map(|(x, y)| (x - y).abs())
            .sum()
    }

    /// Get number of words in vocabulary
    pub fn size(&self) -> usize {
        self.words.len()
    }

    /// Get reference to inverted index
    pub fn inverted_index(&self) -> &HashMap<usize, Vec<(u64, f64)>> {
        &self.inverted_index
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_descriptors(count: usize) -> Vec<Vec<u8>> {
        (0..count)
            .map(|i| {
                vec![(i % 256) as u8; 32]
            })
            .collect()
    }

    #[test]
    fn vocabulary_creation() {
        let config = VocabularyConfig {
            num_words: 100,
            num_clusters: 10,
            max_iterations: 10,
            convergence_threshold: 1e-3,
        };

        let descriptors = create_test_descriptors(100);
        let vocab = Vocabulary::build_from_descriptors(config, descriptors).unwrap();

        assert!(vocab.size() > 0, "Vocabulary should have words");
    }

    #[test]
    fn quantize_descriptor() {
        let config = VocabularyConfig::default();
        let descriptors = create_test_descriptors(50);
        let vocab = Vocabulary::build_from_descriptors(config, descriptors).unwrap();

        let test_desc = vec![100u8; 32];
        let word_id = vocab.quantize(&test_desc);

        assert!(word_id < vocab.size(), "Word ID should be valid");
    }

    #[test]
    fn histogram_similarity_identical() {
        let mut hist1 = BTreeMap::new();
        hist1.insert(0, 1.0);
        hist1.insert(1, 0.5);

        let sim = Vocabulary::histogram_similarity(&hist1, &hist1);
        assert!((sim - 1.0).abs() < 1e-6, "Identical histograms should have similarity 1.0");
    }

    #[test]
    fn histogram_similarity_orthogonal() {
        let mut hist1 = BTreeMap::new();
        hist1.insert(0, 1.0);

        let mut hist2 = BTreeMap::new();
        hist2.insert(1, 1.0);

        let sim = Vocabulary::histogram_similarity(&hist1, &hist2);
        assert!(sim < 1e-6, "Orthogonal histograms should have near-zero similarity");
    }

    #[test]
    fn add_keyframe_to_vocabulary() {
        let config = VocabularyConfig::default();
        let mut vocab = Vocabulary::new(config);

        let words = vec![0, 1, 2, 1, 0];
        vocab.add_keyframe(0, words);

        assert_eq!(vocab.total_documents, 1);
        assert!(!vocab.inverted_index.is_empty());
    }
}
