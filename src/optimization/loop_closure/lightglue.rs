//! LightGlue ONNX-based feature matching for loop closure verification.
//!
//! This module provides deep learning-based feature matching using LightGlue
//! via ONNX Runtime. LightGlue is designed for robust matching in challenging
//! conditions where traditional geometric methods may fail.
//!
//! References:
//! - Lindenberger et al., "LightGlue: Local Feature Matching at Light Speed", ICCV 2023
//! - ONNX Runtime for Rust: https://github.com/pykeio/ort
//! - LightGlue-ONNX: https://github.com/fabio-sim/LightGlue-ONNX

#[cfg(feature = "lightglue")]
use ndarray::Array2;
#[cfg(feature = "lightglue")]
use ort::{
    session::{builder::GraphOptimizationLevel, Session},
    value::Value,
};
use std::path::{Path, PathBuf};

use super::{DescriptorMatcher, MatchMetrics};

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

/// LightGlue-based descriptor matcher using ONNX Runtime
pub struct LightGlueMatcher {
    config: LightGlueConfig,
    #[cfg(feature = "lightglue")]
    session: Option<Session>,
}

impl LightGlueMatcher {
    /// Create new LightGlue matcher
    pub fn new(config: LightGlueConfig) -> Result<Self, String> {
        #[cfg(feature = "lightglue")]
        {
            let session = Self::load_model(&config)?;
            Ok(Self {
                config,
                session: Some(session),
            })
        }

        #[cfg(not(feature = "lightglue"))]
        {
            log::warn!("LightGlue feature not enabled. Compile with --features lightglue");
            Ok(Self { config })
        }
    }

    #[cfg(feature = "lightglue")]
    fn load_model(config: &LightGlueConfig) -> Result<Session, String> {
        if !config.model_path.exists() {
            return Err(format!(
                "LightGlue model not found at {:?}. Download from https://github.com/fabio-sim/LightGlue-ONNX",
                config.model_path
            ));
        }

        let session = Session::builder()
            .map_err(|e| format!("Failed to create session builder: {}", e))?
            .with_optimization_level(GraphOptimizationLevel::Level3)
            .map_err(|e| format!("Failed to set optimization level: {}", e))?
            .with_intra_threads(4)
            .map_err(|e| format!("Failed to set threads: {}", e))?
            .commit_from_file(&config.model_path)
            .map_err(|e| format!("Failed to load model: {}", e))?;

        log::info!("LightGlue model loaded from {:?}", config.model_path);
        Ok(session)
    }

    #[cfg(feature = "lightglue")]
    fn run_inference(
        &self,
        keypoints0: &Array2<f32>,
        keypoints1: &Array2<f32>,
        descriptors0: &Array2<f32>,
        descriptors1: &Array2<f32>,
    ) -> Result<(Vec<(usize, usize)>, Vec<f32>), String> {
        let session = self
            .session
            .as_ref()
            .ok_or_else(|| "ONNX session not initialized".to_string())?;

        // Prepare inputs: keypoints (N, 2), descriptors (N, D)
        let kpts0 = Value::from_array(keypoints0.clone().into_owned())
            .map_err(|e| format!("Failed to create keypoints0 tensor: {}", e))?;
        let kpts1 = Value::from_array(keypoints1.clone().into_owned())
            .map_err(|e| format!("Failed to create keypoints1 tensor: {}", e))?;
        let desc0 = Value::from_array(descriptors0.clone().into_owned())
            .map_err(|e| format!("Failed to create descriptors0 tensor: {}", e))?;
        let desc1 = Value::from_array(descriptors1.clone().into_owned())
            .map_err(|e| format!("Failed to create descriptors1 tensor: {}", e))?;

        // Run inference
        let inputs = ort::inputs![
            "keypoints0" => kpts0,
            "keypoints1" => kpts1,
            "descriptors0" => desc0,
            "descriptors1" => desc1
        ]
        .map_err(|e| format!("Failed to create inputs: {}", e))?;
        
        let outputs = session
            .run(inputs)
            .map_err(|e| format!("Inference failed: {}", e))?;

        // Extract matches: indices (M, 2) and scores (M,)
        let (matches_shape, matches_data) = outputs[0]
            .try_extract_tensor::<i64>()
            .map_err(|e| format!("Failed to extract matches: {}", e))?;
        let (_scores_shape, scores_data) = outputs[1]
            .try_extract_tensor::<f32>()
            .map_err(|e| format!("Failed to extract scores: {}", e))?;

        let mut matches = Vec::new();
        let mut confidences = Vec::new();

        // Filter by confidence threshold
        let num_matches = matches_shape[0] as usize;
        for i in 0..num_matches {
            let score = scores_data[i];
            if score >= self.config.confidence_threshold {
                let idx0 = matches_data[i * 2] as usize;
                let idx1 = matches_data[i * 2 + 1] as usize;
                matches.push((idx0, idx1));
                confidences.push(score);
            }
        }

        Ok((matches, confidences))
    }

    /// Convert byte descriptors to f32 for ONNX
    #[allow(dead_code)]
    fn descriptors_to_f32(descriptors: &[Vec<u8>]) -> Array2<f32> {
        let n = descriptors.len();
        let d = descriptors[0].len();

        let mut array = Array2::zeros((n, d));
        for (i, desc) in descriptors.iter().enumerate() {
            for (j, &byte) in desc.iter().enumerate() {
                array[[i, j]] = byte as f32 / 255.0;
            }
        }
        array
    }
}

impl DescriptorMatcher for LightGlueMatcher {
    fn match_keyframes(
        &self,
        _query: &super::KeyframeDescriptor,
        _candidate: &super::KeyframeDescriptor,
    ) -> MatchMetrics {
        #[cfg(feature = "lightglue")]
        {
            // In a real implementation, this would:
            // 1. Extract SuperPoint features from query and candidate images
            // 2. Convert to ONNX-compatible tensors
            // 3. Run LightGlue inference
            // 4. Return match metrics

            log::warn!("LightGlue matching not fully implemented - requires image data");

            MatchMetrics {
                similarity: 0.0,
                match_count: 0,
                match_ratio: 0.0,
            }
        }

        #[cfg(not(feature = "lightglue"))]
        {
            log::error!("LightGlue feature not enabled. Compile with --features lightglue");
            MatchMetrics {
                similarity: 0.0,
                match_count: 0,
                match_ratio: 0.0,
            }
        }
    }
}

/// LightGlue model downloader helper
pub struct LightGlueModelDownloader;

impl LightGlueModelDownloader {
    /// Download pre-trained LightGlue ONNX model
    pub fn download_model(output_dir: &Path, feature_type: &str) -> Result<PathBuf, String> {
        let model_name = format!("lightglue_{}.onnx", feature_type);
        let output_path = output_dir.join(&model_name);

        if output_path.exists() {
            log::info!("Model already exists at {:?}", output_path);
            return Ok(output_path);
        }

        let url = format!(
            "https://github.com/fabio-sim/LightGlue-ONNX/releases/download/v1.0/{}",
            model_name
        );

        log::info!("Downloading LightGlue model from {}", url);
        log::warn!("Auto-download not implemented. Please download manually:");
        log::warn!("  1. Visit: https://github.com/fabio-sim/LightGlue-ONNX/releases");
        log::warn!("  2. Download {}", model_name);
        log::warn!("  3. Place at {:?}", output_path);

        Err("Manual download required".to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lightglue_config_default() {
        let config = LightGlueConfig::default();
        assert_eq!(config.max_keypoints, 1024);
        assert_eq!(config.confidence_threshold, 0.5);
        assert_eq!(config.feature_extractor, "superpoint");
    }

    #[test]
    fn test_lightglue_matcher_creation_without_model() {
        let config = LightGlueConfig {
            model_path: PathBuf::from("/nonexistent/model.onnx"),
            ..Default::default()
        };

        #[cfg(feature = "lightglue")]
        {
            let result = LightGlueMatcher::new(config);
            assert!(result.is_err());
        }

        #[cfg(not(feature = "lightglue"))]
        {
            let result = LightGlueMatcher::new(config);
            assert!(result.is_ok());
        }
    }

    #[test]
    fn test_descriptor_conversion() {
        let descriptors = vec![vec![0, 128, 255], vec![64, 192, 32]];

        let array = LightGlueMatcher::descriptors_to_f32(&descriptors);
        assert_eq!(array.shape(), &[2, 3]);
        assert!((array[[0, 0]] - 0.0).abs() < 1e-6);
        assert!((array[[0, 2]] - 1.0).abs() < 1e-6);
    }
}
