//! SuperPoint neural network descriptor extraction
//!
//! Implements descriptor extraction using SuperPoint, a self-supervised
//! deep learning approach for keypoint detection and description.
//!
//! References:
//! - DeLone et al., "SuperPoint: Self-Supervised Interest Point Detection and Description"
//! - https://github.com/magicleap/SuperPoint

use nalgebra::Point2;
use serde::{Deserialize, Serialize};

/// SuperPoint configuration
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SuperPointConfig {
    /// Grid size for descriptor extraction (typically 8x8 grid on 64x64 cell)
    pub descriptor_grid_size: u32,
    /// Descriptor size in bytes (SuperPoint uses 256-dimensional float32)
    pub descriptor_size: usize, // 256 * 4 = 1024 bytes
    /// Confidence threshold for keypoints [0, 1]
    pub keypoint_threshold: f32,
    /// Image normalization: subtract mean and divide by std
    pub normalize_image: bool,
    /// Whether to use ONNX runtime (vs. PyTorch/TorchScript)
    pub use_onnx: bool,
}

impl Default for SuperPointConfig {
    fn default() -> Self {
        Self {
            descriptor_grid_size: 8,
            descriptor_size: 256, // float32, 4 bytes each = 1024 bytes total
            keypoint_threshold: 0.015,
            normalize_image: true,
            use_onnx: true,
        }
    }
}

/// Extracted keypoint with descriptor
#[derive(Debug, Clone)]
pub struct KeypointDescriptor {
    /// Keypoint position in image coordinates (pixels)
    pub position: Point2<f64>,
    /// Descriptor vector (256-dimensional for SuperPoint)
    pub descriptor: Vec<f32>,
    /// Keypoint confidence score [0, 1]
    pub confidence: f32,
    /// Octave/scale level from pyramid
    pub octave: u32,
    /// Grid cell index in descriptor space
    pub grid_index: (u32, u32),
}

impl KeypointDescriptor {
    /// Compute L2 distance to another descriptor
    pub fn l2_distance(&self, other: &KeypointDescriptor) -> f32 {
        self.descriptor
            .iter()
            .zip(other.descriptor.iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f32>()
            .sqrt()
    }

    /// Compute cosine similarity to another descriptor (normalized cosine)
    pub fn cosine_similarity(&self, other: &KeypointDescriptor) -> f32 {
        let dot_product: f32 = self
            .descriptor
            .iter()
            .zip(other.descriptor.iter())
            .map(|(a, b)| a * b)
            .sum();

        let self_norm: f32 = self.descriptor.iter().map(|x| x * x).sum::<f32>().sqrt();
        let other_norm: f32 = other.descriptor.iter().map(|x| x * x).sum::<f32>().sqrt();

        if self_norm > 1e-6 && other_norm > 1e-6 {
            dot_product / (self_norm * other_norm)
        } else {
            0.0
        }
    }
}

/// SuperPoint descriptor extractor (placeholder for ONNX integration)
pub struct SuperPointDescriptor {
    config: SuperPointConfig,
    /// Model path (ONNX format)
    model_path: Option<String>,
    /// Input image size expected by model
    #[allow(dead_code)]
    input_size: (u32, u32),
    /// Extracted keypoints and descriptors
    keypoints: Vec<KeypointDescriptor>,
}

impl SuperPointDescriptor {
    /// Create new SuperPoint descriptor extractor
    pub fn new(config: SuperPointConfig) -> Self {
        Self {
            config,
            model_path: None,
            input_size: (640, 480),
            keypoints: Vec::new(),
        }
    }

    /// Load ONNX model from file path
    pub fn load_model(&mut self, path: &str) -> Result<(), String> {
        // In production: load ONNX model using `ort` crate
        // For now, just store the path
        self.model_path = Some(path.to_string());
        Ok(())
    }

    /// Extract descriptors from image
    ///
    /// In a real implementation, this would:
    /// 1. Preprocess image (normalize, resize to expected input size)
    /// 2. Run ONNX model forward pass
    /// 3. Extract keypoints from confidence map
    /// 4. Extract descriptors from descriptor map
    /// 5. Match descriptors spatially to keypoints
    pub fn extract(
        &mut self,
        image_data: &[u8],
        _image_width: u32,
        _image_height: u32,
    ) -> Result<Vec<KeypointDescriptor>, String> {
        if image_data.is_empty() {
            return Err("Empty image data".to_string());
        }

        // Reset keypoints
        self.keypoints.clear();

        // TODO: Implement ONNX model loading and inference
        // For now, return empty (will be implemented with ort crate)
        
        Ok(Vec::new())
    }

    /// Extract descriptors at specific keypoints
    ///
    /// Useful for re-extracting descriptors at tracked positions
    /// without re-running full detection
    pub fn extract_at_keypoints(
        &mut self,
        _image_data: &[u8],
        _image_width: u32,
        _image_height: u32,
        _keypoints: &[Point2<f64>],
    ) -> Result<Vec<KeypointDescriptor>, String> {
        // TODO: Extract descriptors only at specified positions
        Ok(Vec::new())
    }

    /// Get extracted keypoints and descriptors
    pub fn keypoints(&self) -> &[KeypointDescriptor] {
        &self.keypoints
    }

    /// Get mutable keypoints
    pub fn keypoints_mut(&mut self) -> &mut Vec<KeypointDescriptor> {
        &mut self.keypoints
    }

    /// Get configuration
    pub fn config(&self) -> SuperPointConfig {
        self.config
    }

    /// Statistics about extraction
    pub fn stats(&self) -> ExtractionStats {
        ExtractionStats {
            num_keypoints: self.keypoints.len() as u32,
            avg_confidence: if self.keypoints.is_empty() {
                0.0
            } else {
                self.keypoints.iter().map(|k| k.confidence as f64).sum::<f64>()
                    / self.keypoints.len() as f64
            },
            descriptor_size: self.config.descriptor_size as u32,
        }
    }
}

impl Default for SuperPointDescriptor {
    fn default() -> Self {
        Self::new(SuperPointConfig::default())
    }
}

/// Statistics from descriptor extraction
#[derive(Debug, Clone, Copy)]
pub struct ExtractionStats {
    pub num_keypoints: u32,
    pub avg_confidence: f64,
    pub descriptor_size: u32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_superpoint_config_defaults() {
        let config = SuperPointConfig::default();
        assert_eq!(config.descriptor_grid_size, 8);
        assert_eq!(config.descriptor_size, 256);
        assert!(config.normalize_image);
        assert!(config.use_onnx);
    }

    #[test]
    fn test_keypoint_descriptor_creation() {
        let descriptor = vec![0.1; 256];
        let kp = KeypointDescriptor {
            position: Point2::new(100.0, 150.0),
            descriptor,
            confidence: 0.95,
            octave: 0,
            grid_index: (4, 5),
        };

        assert_eq!(kp.descriptor.len(), 256);
        assert!(kp.confidence > 0.9);
    }

    #[test]
    fn test_descriptor_distance_computation() {
        let desc1 = vec![0.1; 256];
        let desc2 = vec![0.1; 256];

        let kp1 = KeypointDescriptor {
            position: Point2::new(100.0, 150.0),
            descriptor: desc1,
            confidence: 0.95,
            octave: 0,
            grid_index: (0, 0),
        };

        let kp2 = KeypointDescriptor {
            position: Point2::new(100.0, 150.0),
            descriptor: desc2,
            confidence: 0.95,
            octave: 0,
            grid_index: (0, 0),
        };

        // Identical descriptors should have zero distance
        let dist = kp1.l2_distance(&kp2);
        assert!(dist < 1e-5);

        // Should have high cosine similarity
        let sim = kp1.cosine_similarity(&kp2);
        assert!(sim > 0.99);
    }

    #[test]
    fn test_superpoint_extractor_creation() {
        let config = SuperPointConfig::default();
        let extractor = SuperPointDescriptor::new(config);

        assert_eq!(extractor.keypoints().len(), 0);
    }

    #[test]
    fn test_extraction_stats() {
        let config = SuperPointConfig::default();
        let mut extractor = SuperPointDescriptor::new(config);

        let descriptor = vec![0.1; 256];
        extractor.keypoints_mut().push(KeypointDescriptor {
            position: Point2::new(100.0, 150.0),
            descriptor,
            confidence: 0.95,
            octave: 0,
            grid_index: (0, 0),
        });

        let stats = extractor.stats();
        assert_eq!(stats.num_keypoints, 1);
        assert!(stats.avg_confidence > 0.9);
    }
}
