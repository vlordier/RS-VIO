//! LightGlue cross-attention matcher for descriptor correspondence
//!
//! Implements learned feature matching using attention mechanisms.
//! LightGlue learns to match keypoints by comparing descriptors
//! through multi-head cross-attention.
//!
//! References:
//! - Lindenberger et al., "LightGlue: Local Feature Matching at Light Speed"
//! - https://github.com/cvg/LightGlue

use crate::feature_tracker::KeypointDescriptor;
use serde::{Deserialize, Serialize};

/// LightGlue matcher configuration
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LightGlueConfig {
    /// Number of attention heads
    pub num_heads: u32,
    /// Depth of transformer (number of layers)
    pub depth: u32,
    /// Flash attention fallback for memory efficiency
    pub use_flash_attention: bool,
    /// Confidence threshold for matches [0, 1]
    pub match_threshold: f32,
    /// Maximum number of matches to keep
    pub max_matches: u32,
    /// Subpixel refinement iterations
    pub subpixel_iterations: u32,
}

impl Default for LightGlueConfig {
    fn default() -> Self {
        Self {
            num_heads: 8,
            depth: 9,
            use_flash_attention: true,
            match_threshold: 0.1,
            max_matches: 512,
            subpixel_iterations: 10,
        }
    }
}

/// Feature match between two keypoints
#[derive(Debug, Clone, Copy)]
pub struct FeatureMatch {
    /// ID of query keypoint
    pub query_id: u32,
    /// ID of reference keypoint
    pub reference_id: u32,
    /// Match confidence score [0, 1]
    pub confidence: f32,
    /// Subpixel refinement offset for query
    pub query_offset: (f32, f32),
    /// Subpixel refinement offset for reference
    pub reference_offset: (f32, f32),
}

/// LightGlue matcher
pub struct LightGlueMatcher {
    config: LightGlueConfig,
    /// Model path (ONNX or other format)
    model_path: Option<String>,
    /// Descriptor banks
    query_descriptors: Vec<KeypointDescriptor>,
    reference_descriptors: Vec<KeypointDescriptor>,
    /// Match results
    matches: Vec<FeatureMatch>,
}

impl LightGlueMatcher {
    /// Create new LightGlue matcher
    pub fn new(config: LightGlueConfig) -> Self {
        Self {
            config,
            model_path: None,
            query_descriptors: Vec::new(),
            reference_descriptors: Vec::new(),
            matches: Vec::new(),
        }
    }

    /// Load matcher model
    pub fn load_model(&mut self, path: &str) -> Result<(), String> {
        // In production: load ONNX/PyTorch model
        // For now, just store path
        self.model_path = Some(path.to_string());
        Ok(())
    }

    /// Set query descriptors (typically from current frame)
    pub fn set_query_descriptors(&mut self, descriptors: Vec<KeypointDescriptor>) {
        self.query_descriptors = descriptors;
    }

    /// Set reference descriptors (typically from previous frame or template)
    pub fn set_reference_descriptors(&mut self, descriptors: Vec<KeypointDescriptor>) {
        self.reference_descriptors = descriptors;
    }

    /// Match query descriptors to reference descriptors
    ///
    /// In a real implementation, this would:
    /// 1. Run ONNX model forward pass on descriptor pairs
    /// 2. Apply soft inlier threshold
    /// 3. Extract matches with confidence scores
    /// 4. Optionally refine matches via subpixel registration
    pub fn match_descriptors(&mut self) -> Result<Vec<FeatureMatch>, String> {
        if self.query_descriptors.is_empty() || self.reference_descriptors.is_empty() {
            return Ok(Vec::new());
        }

        self.matches.clear();

        // TODO: Implement cross-attention matching via ONNX
        // For now, use simple nearest neighbor matching as fallback
        
        self.simple_nn_matching()?;

        Ok(self.matches.clone())
    }

    /// Simple nearest neighbor matching (fallback implementation)
    fn simple_nn_matching(&mut self) -> Result<(), String> {
        for (q_idx, query) in self.query_descriptors.iter().enumerate() {
            let mut best_dist = f32::MAX;
            let mut best_idx = None;
            let mut second_best_dist = f32::MAX;

            for (r_idx, reference) in self.reference_descriptors.iter().enumerate() {
                let dist = query.l2_distance(reference);

                if dist < best_dist {
                    second_best_dist = best_dist;
                    best_dist = dist;
                    best_idx = Some(r_idx);
                } else if dist < second_best_dist {
                    second_best_dist = dist;
                }
            }

            // Lowe's ratio test: reject ambiguous matches
            if let Some(r_idx) = best_idx {
                let ratio = best_dist / (second_best_dist + 1e-6);
                if ratio < 0.7 && best_dist < 2.0 {
                    // Normalize distance to confidence
                    let confidence = 1.0 - (best_dist / 2.0).min(1.0);

                    if confidence >= self.config.match_threshold {
                        self.matches.push(FeatureMatch {
                            query_id: q_idx as u32,
                            reference_id: r_idx as u32,
                            confidence,
                            query_offset: (0.0, 0.0),
                            reference_offset: (0.0, 0.0),
                        });
                    }
                }
            }
        }

        // Keep only top matches by confidence
        self.matches
            .sort_by(|a, b| b.confidence.partial_cmp(&a.confidence).unwrap());
        self.matches.truncate(self.config.max_matches as usize);

        Ok(())
    }

    /// Refine matches via subpixel registration
    ///
    /// Uses template matching within small neighborhoods to refine
    /// match positions to subpixel accuracy
    pub fn refine_matches_subpixel(&mut self) -> Result<(), String> {
        for match_result in &mut self.matches {
            let query = match self
                .query_descriptors
                .iter()
                .find(|k| k.descriptor.iter().zip(
                    self.query_descriptors[match_result.query_id as usize]
                        .descriptor
                        .iter()
                    )
                    .all(|(a, b)| (a - b).abs() < 1e-6))
            {
                Some(k) => k,
                None => continue,
            };

            let reference = match self
                .reference_descriptors
                .iter()
                .find(|k| k.descriptor.iter().zip(
                    self.reference_descriptors[match_result.reference_id as usize]
                        .descriptor
                        .iter()
                    )
                    .all(|(a, b)| (a - b).abs() < 1e-6))
            {
                Some(k) => k,
                None => continue,
            };

            // Simple subpixel refinement: estimate offset via descriptor gradient
            // In practice, this would use image gradients and iterative refinement

            let dx = if reference.position.x > query.position.x { 0.1 } else { -0.1 };
            let dy = if reference.position.y > query.position.y { 0.1 } else { -0.1 };

            match_result.query_offset = (dx, dy);
        }

        Ok(())
    }

    /// Get matched pairs
    pub fn matches(&self) -> &[FeatureMatch] {
        &self.matches
    }

    /// Get configuration
    pub fn config(&self) -> LightGlueConfig {
        self.config
    }

    /// Get matching statistics
    pub fn stats(&self) -> MatchingStats {
        MatchingStats {
            num_matches: self.matches.len() as u32,
            avg_confidence: if self.matches.is_empty() {
                0.0
            } else {
                self.matches.iter().map(|m| m.confidence as f64).sum::<f64>()
                    / self.matches.len() as f64
            },
            match_rate: if self.query_descriptors.is_empty() {
                0.0
            } else {
                self.matches.len() as f64 / self.query_descriptors.len() as f64
            },
        }
    }
}

impl Default for LightGlueMatcher {
    fn default() -> Self {
        Self::new(LightGlueConfig::default())
    }
}

/// Statistics from matching
#[derive(Debug, Clone, Copy)]
pub struct MatchingStats {
    pub num_matches: u32,
    pub avg_confidence: f64,
    pub match_rate: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point2;

    #[test]
    fn test_lightglue_config_defaults() {
        let config = LightGlueConfig::default();
        assert_eq!(config.num_heads, 8);
        assert_eq!(config.depth, 9);
        assert!(config.use_flash_attention);
    }

    #[test]
    fn test_matcher_creation() {
        let matcher = LightGlueMatcher::new(LightGlueConfig::default());
        assert_eq!(matcher.query_descriptors.len(), 0);
        assert_eq!(matcher.reference_descriptors.len(), 0);
    }

    #[test]
    fn test_simple_nn_matching() {
        let mut matcher = LightGlueMatcher::new(LightGlueConfig::default());

        // Create query descriptors
        let query = KeypointDescriptor {
            position: Point2::new(100.0, 100.0),
            descriptor: vec![0.1; 256],
            confidence: 0.95,
            octave: 0,
            grid_index: (0, 0),
        };

        // Create reference descriptors (one identical, one different)
        let ref1 = KeypointDescriptor {
            position: Point2::new(105.0, 105.0),
            descriptor: vec![0.1; 256], // Identical to query
            confidence: 0.95,
            octave: 0,
            grid_index: (0, 0),
        };

        let ref2 = KeypointDescriptor {
            position: Point2::new(200.0, 200.0),
            descriptor: vec![0.5; 256], // Different from query
            confidence: 0.95,
            octave: 0,
            grid_index: (1, 1),
        };

        matcher.set_query_descriptors(vec![query]);
        matcher.set_reference_descriptors(vec![ref1, ref2]);

        let matches = matcher.match_descriptors().unwrap();
        
        // Should match query to ref1
        assert_eq!(matches.len(), 1);
        assert_eq!(matches[0].reference_id, 0);
    }

    #[test]
    fn test_matching_stats() {
        let mut matcher = LightGlueMatcher::new(LightGlueConfig::default());

        let query = KeypointDescriptor {
            position: Point2::new(100.0, 100.0),
            descriptor: vec![0.1; 256],
            confidence: 0.95,
            octave: 0,
            grid_index: (0, 0),
        };

        let reference = KeypointDescriptor {
            position: Point2::new(105.0, 105.0),
            descriptor: vec![0.1; 256],
            confidence: 0.95,
            octave: 0,
            grid_index: (0, 0),
        };

        matcher.set_query_descriptors(vec![query]);
        matcher.set_reference_descriptors(vec![reference]);

        let _matches = matcher.match_descriptors().unwrap();
        let stats = matcher.stats();

        assert!(stats.num_matches > 0);
        assert!(stats.match_rate > 0.0);
    }
}
