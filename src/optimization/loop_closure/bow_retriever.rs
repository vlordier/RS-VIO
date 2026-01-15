//! BoW-based efficient candidate retrieval for loop closure detection.
//!
//! This module provides fast candidate retrieval using Bag-of-Words histograms
//! instead of exhaustive descriptor comparisons.

use std::collections::BTreeMap;
use super::vocabulary::Vocabulary;
use super::{DescriptorMatcher, MatchMetrics};

/// BoW-based retriever for fast loop closure candidate detection
pub struct BowRetriever {
    vocabulary: Option<Vocabulary>,
    /// Minimum histogram similarity threshold for candidate consideration
    min_similarity_threshold: f64,
    /// Maximum number of candidates to return
    max_candidates: usize,
}

impl BowRetriever {
    /// Create new BoW retriever with optional vocabulary
    pub fn new(min_similarity_threshold: f64, max_candidates: usize) -> Self {
        Self {
            vocabulary: None,
            min_similarity_threshold,
            max_candidates,
        }
    }

    /// Set vocabulary for BoW matching
    pub fn set_vocabulary(&mut self, vocabulary: Vocabulary) {
        self.vocabulary = Some(vocabulary);
    }

    /// Check if vocabulary is available
    pub fn has_vocabulary(&self) -> bool {
        self.vocabulary.is_some()
    }

    /// Retrieve candidates using BoW histograms
    pub fn retrieve_candidates(
        &self,
        query_words: Vec<usize>,
        keyframe_database: &[(u64, Vec<usize>)],
    ) -> Vec<(u64, f64)> {
        if let Some(vocab) = &self.vocabulary {
            let query_hist = vocab.compute_histogram(query_words);

            let mut candidates: Vec<(u64, f64)> = keyframe_database
                .iter()
                .map(|(keyframe_id, words)| {
                    let db_hist = vocab.compute_histogram(words.clone());
                    let similarity = Vocabulary::histogram_similarity(&query_hist, &db_hist);
                    (*keyframe_id, similarity)
                })
                .filter(|(_, similarity)| *similarity >= self.min_similarity_threshold)
                .collect();

            // Sort by similarity (descending)
            candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

            // Limit to max candidates
            candidates.truncate(self.max_candidates);

            log::debug!(
                "[BowRetriever] Retrieved {} candidates from {} keyframes (threshold: {})",
                candidates.len(),
                keyframe_database.len(),
                self.min_similarity_threshold
            );

            candidates
        } else {
            log::warn!("[BowRetriever] No vocabulary available, returning empty candidates");
            Vec::new()
        }
    }

    /// Compute BoW histogram for descriptor list
    pub fn compute_histogram(&self, words: Vec<usize>) -> Option<BTreeMap<usize, f64>> {
        self.vocabulary.as_ref().map(|vocab| vocab.compute_histogram(words))
    }

    /// Quantize descriptor using vocabulary
    pub fn quantize_descriptor(&self, descriptor: &[u8]) -> Option<usize> {
        self.vocabulary.as_ref().map(|vocab| vocab.quantize(descriptor))
    }
}

/// Hybrid matcher that uses BoW for retrieval then DescriptorMatcher for verification
pub struct HybridMatcher {
    bow_retriever: BowRetriever,
    descriptor_matcher: Box<dyn DescriptorMatcher>,
}

impl HybridMatcher {
    /// Create new hybrid matcher
    pub fn new(
        bow_retriever: BowRetriever,
        descriptor_matcher: Box<dyn DescriptorMatcher>,
    ) -> Self {
        Self {
            bow_retriever,
            descriptor_matcher,
        }
    }

    /// Get reference to BoW retriever
    pub fn bow_retriever(&self) -> &BowRetriever {
        &self.bow_retriever
    }

    /// Get mutable reference to BoW retriever
    pub fn bow_retriever_mut(&mut self) -> &mut BowRetriever {
        &mut self.bow_retriever
    }
}

impl DescriptorMatcher for HybridMatcher {
    fn match_keyframes(
        &self,
        descriptor1: &super::KeyframeDescriptor,
        descriptor2: &super::KeyframeDescriptor,
    ) -> MatchMetrics {
        // If BoW vocabulary is available, we could add pre-filtering here
        // For now, delegate to the underlying descriptor matcher
        self.descriptor_matcher.match_keyframes(descriptor1, descriptor2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bow_retriever_creation() {
        let retriever = BowRetriever::new(0.5, 10);
        assert!(!retriever.has_vocabulary());
    }

    #[test]
    fn bow_retriever_no_candidates_without_vocab() {
        let retriever = BowRetriever::new(0.5, 10);
        let database = vec![(1u64, vec![0, 1, 2])];
        let candidates = retriever.retrieve_candidates(vec![0, 1, 2], &database);
        assert_eq!(candidates.len(), 0);
    }

    #[test]
    fn hybrid_matcher_creation() {
        use crate::optimization::loop_closure::CosineMatcher;
        let retriever = BowRetriever::new(0.5, 10);
        let matcher = HybridMatcher::new(retriever, Box::new(CosineMatcher));
        assert!(!matcher.bow_retriever().has_vocabulary());
    }
}
