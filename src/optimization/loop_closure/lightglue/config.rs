//! Configuration structures for LightGlue matcher.

use std::path::PathBuf;

/// Configuration for LightGlue matcher
#[derive(Debug, Clone)]
pub struct LightGlueConfig {
    /// Path to the ONNX model file
    pub model_path: PathBuf,
    /// Maximum number of keypoints to extract per image
    pub max_keypoints: usize,
    /// Match confidence threshold (0.0-1.0)
    pub confidence_threshold: f32,
    /// Use GPU if available
    pub use_gpu: bool,
    /// Feature extractor type ("superpoint", "disk", etc.)
    pub feature_extractor: String,
}

impl Default for LightGlueConfig {
    fn default() -> Self {
        Self {
            model_path: PathBuf::from("models/lightglue_superpoint.onnx"),
            max_keypoints: 1024,
            confidence_threshold: 0.5,
            use_gpu: false,
            feature_extractor: "superpoint".to_string(),
        }
    }
}
