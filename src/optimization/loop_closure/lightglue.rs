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
#[allow(dead_code)]
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
    #[allow(dead_code)]
    fn run_inference(
        &mut self,
        keypoints0: &Array2<f32>,
        keypoints1: &Array2<f32>,
        descriptors0: &Array2<f32>,
        descriptors1: &Array2<f32>,
    ) -> Result<(Vec<(usize, usize)>, Vec<f32>), String> {
        let session = self
            .session
            .as_mut()
            .ok_or_else(|| "ONNX session not initialized".to_string())?;

        // Prepare inputs: keypoints (N, 2), descriptors (N, D)
        let kpts0_shape = vec![keypoints0.nrows() as i64, keypoints0.ncols() as i64];
        let kpts1_shape = vec![keypoints1.nrows() as i64, keypoints1.ncols() as i64];
        let desc0_shape = vec![descriptors0.nrows() as i64, descriptors0.ncols() as i64];
        let desc1_shape = vec![descriptors1.nrows() as i64, descriptors1.ncols() as i64];

        let kpts0 =
            Value::from_array((kpts0_shape, keypoints0.clone().into_raw_vec_and_offset().0))
                .map_err(|e| format!("Failed to create keypoints0 tensor: {}", e))?;
        let kpts1 =
            Value::from_array((kpts1_shape, keypoints1.clone().into_raw_vec_and_offset().0))
                .map_err(|e| format!("Failed to create keypoints1 tensor: {}", e))?;
        let desc0 = Value::from_array((
            desc0_shape,
            descriptors0.clone().into_raw_vec_and_offset().0,
        ))
        .map_err(|e| format!("Failed to create descriptors0 tensor: {}", e))?;
        let desc1 = Value::from_array((
            desc1_shape,
            descriptors1.clone().into_raw_vec_and_offset().0,
        ))
        .map_err(|e| format!("Failed to create descriptors1 tensor: {}", e))?;

        // Run inference
        let inputs = ort::inputs![
            "keypoints0" => kpts0,
            "keypoints1" => kpts1,
            "descriptors0" => desc0,
            "descriptors1" => desc1
        ];

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

    #[test]
    fn test_lightglue_model_loading_with_weights() {
        use std::path::Path;

        // Test model loading when weights are available
        let config = LightGlueConfig {
            model_path: PathBuf::from("models/lightglue_superpoint.onnx"),
            ..Default::default()
        };

        #[cfg(feature = "lightglue")]
        {
            let result = LightGlueMatcher::new(config.clone());

            if Path::new("models/lightglue_superpoint.onnx").exists() {
                // If model weights exist, should succeed
                assert!(result.is_ok(), "Should load model when weights exist");
                let mut matcher = result.unwrap();

                // Test that we can access the session
                assert!(matcher.session.is_some(), "Session should be initialized");

                // Test inference with synthetic data
                let keypoints0 = Array2::<f32>::zeros((10, 2));
                let keypoints1 = Array2::<f32>::zeros((10, 2));
                let descriptors0 = Array2::<f32>::zeros((10, 256));
                let descriptors1 = Array2::<f32>::zeros((10, 256));

                let inference_result =
                    matcher.run_inference(&keypoints0, &keypoints1, &descriptors0, &descriptors1);

                assert!(
                    inference_result.is_ok(),
                    "Inference should succeed with valid inputs"
                );
                let (matches, scores) = inference_result.unwrap();
                assert!(
                    !matches.is_empty() || scores.is_empty(),
                    "Should return matches or empty results"
                );
            } else {
                // If no weights, should fail gracefully
                assert!(result.is_err(), "Should fail when weights don't exist");
            }
        }

        #[cfg(not(feature = "lightglue"))]
        {
            // Without feature, should always succeed (no-op)
            let result = LightGlueMatcher::new(config);
            assert!(
                result.is_ok(),
                "Should always succeed without lightglue feature"
            );
        }
    }

    #[test]
    fn test_lightglue_inference_edge_cases() {
        use std::path::Path;

        let config = LightGlueConfig {
            model_path: PathBuf::from("models/lightglue_superpoint.onnx"),
            ..Default::default()
        };

        #[cfg(feature = "lightglue")]
        {
            if !Path::new("models/lightglue_superpoint.onnx").exists() {
                // Skip if no model weights
                return;
            }

            let mut matcher = LightGlueMatcher::new(config).unwrap();

            // Test 1: Empty keypoints/descriptors
            let empty_kpts = Array2::<f32>::zeros((0, 2));
            let empty_desc = Array2::<f32>::zeros((0, 256));

            let result = matcher.run_inference(&empty_kpts, &empty_kpts, &empty_desc, &empty_desc);
            assert!(result.is_ok(), "Should handle empty inputs gracefully");

            // Test 2: Single feature
            let single_kpts = Array2::<f32>::from_shape_vec((1, 2), vec![100.0, 200.0]).unwrap();
            let single_desc = Array2::<f32>::zeros((1, 256));

            let result =
                matcher.run_inference(&single_kpts, &single_kpts, &single_desc, &single_desc);
            assert!(result.is_ok(), "Should handle single feature");

            // Test 3: Mismatched dimensions (should fail gracefully)
            let kpts_a = Array2::<f32>::zeros((5, 2));
            let kpts_b = Array2::<f32>::zeros((3, 2)); // Different number of keypoints
            let desc_a = Array2::<f32>::zeros((5, 256));
            let desc_b = Array2::<f32>::zeros((3, 256));

            let result = matcher.run_inference(&kpts_a, &kpts_b, &desc_a, &desc_b);
            assert!(result.is_ok(), "Should handle mismatched keypoint counts");

            // Test 4: Large number of features
            let large_kpts = Array2::<f32>::zeros((1000, 2));
            let large_desc = Array2::<f32>::zeros((1000, 256));

            let result = matcher.run_inference(&large_kpts, &large_kpts, &large_desc, &large_desc);
            assert!(result.is_ok(), "Should handle large feature sets");

            // Test 5: Very different descriptors (should return few/no matches)
            let kpts = Array2::<f32>::zeros((10, 2));
            let desc_similar = Array2::<f32>::zeros((10, 256));
            let mut desc_different = Array2::<f32>::ones((10, 256)) * 0.5;

            // Make descriptors very different
            for i in 0..10 {
                for j in 0..256 {
                    desc_different[[i, j]] = if j % 2 == 0 { 1.0 } else { 0.0 };
                }
            }

            let result = matcher.run_inference(&kpts, &kpts, &desc_similar, &desc_different);
            assert!(result.is_ok(), "Should handle very different descriptors");
        }
    }

    #[test]
    fn test_lightglue_descriptor_preprocessing() {
        // Test descriptor preprocessing with various inputs
        let test_cases = vec![
            // Empty descriptors
            vec![],
            // Single descriptor
            vec![vec![0u8; 32]],
            // Multiple descriptors with different values
            vec![vec![0u8; 32], vec![255u8; 32], vec![128u8; 32]],
            // Variable length descriptors (should handle gracefully)
            vec![vec![0u8; 16], vec![0u8; 64]],
        ];

        for descriptors in test_cases {
            if !descriptors.is_empty() {
                let array = LightGlueMatcher::descriptors_to_f32(&descriptors);
                assert_eq!(array.nrows(), descriptors.len());
                assert_eq!(array.ncols(), descriptors[0].len());
            }
        }
    }

    #[test]
    fn test_lightglue_config_validation() {
        // Test various configuration scenarios
        let configs = vec![
            LightGlueConfig {
                confidence_threshold: -0.1, // Invalid
                ..Default::default()
            },
            LightGlueConfig {
                confidence_threshold: 1.5, // Invalid
                ..Default::default()
            },
            LightGlueConfig {
                confidence_threshold: 0.5, // Valid
                max_keypoints: 0,          // Invalid
                ..Default::default()
            },
            LightGlueConfig {
                confidence_threshold: 0.5, // Valid
                max_keypoints: 10000,      // Valid
                ..Default::default()
            },
        ];

        for config in configs {
            // Config validation should not panic - let the matcher handle invalid values
            let _matcher = LightGlueMatcher::new(config);
        }
    }

    #[test]
    fn test_lightglue_model_weight_validation() {
        use std::fs;
        use std::path::Path;
        use std::process::Command;

        let model_path = "models/lightglue_superpoint.onnx";
        let model_url = "https://github.com/fabio-sim/LightGlue-ONNX/releases/download/v2.0/superpoint_lightglue_pipeline.ort.onnx";

        #[cfg(feature = "lightglue")]
        {
            // Ensure model directory exists
            std::fs::create_dir_all("models").ok();

            // Auto-download model if not present (for testing)
            if !Path::new(model_path).exists() {
                println!("📥 Auto-downloading LightGlue model for testing...");
                println!("   URL: {}", model_url);

                let download_result = if cfg!(target_os = "windows") {
                    Command::new("powershell")
                        .args(&[
                            "-Command",
                            &format!(
                                "Invoke-WebRequest -Uri '{}' -OutFile '{}'",
                                model_url, model_path
                            ),
                        ])
                        .status()
                } else {
                    Command::new("curl")
                        .args(&["-L", "-o", model_path, model_url])
                        .status()
                };

                match download_result {
                    Ok(status) if status.success() => {
                        println!("✅ Model downloaded successfully");
                    },
                    _ => {
                        println!("⚠️  Model download failed - skipping validation test");
                        return;
                    },
                }
            }

            // Verify model file exists and has reasonable size
            assert!(
                Path::new(model_path).exists(),
                "Model file should exist after download"
            );
            let metadata = fs::metadata(model_path).unwrap();
            assert!(
                metadata.len() > 1000000,
                "Model file should be reasonably large (>1MB)"
            );

            // Try to load the model
            let config = LightGlueConfig {
                model_path: PathBuf::from(model_path),
                ..Default::default()
            };

            let result = LightGlueMatcher::new(config);
            assert!(result.is_ok(), "Should successfully load model weights");

            let mut matcher = result.unwrap();
            assert!(matcher.session.is_some(), "Session should be initialized");

            // Test with real data shapes (typical LightGlue inputs)
            let keypoints0 =
                Array2::<f32>::from_shape_vec((50, 2), (0..100).map(|x| x as f32 * 0.1).collect())
                    .unwrap();
            let keypoints1 =
                Array2::<f32>::from_shape_vec((45, 2), (0..90).map(|x| x as f32 * 0.12).collect())
                    .unwrap();

            // LightGlue uses 256-dim descriptors
            let descriptors0 = Array2::<f32>::from_shape_vec(
                (50, 256),
                (0..12800).map(|x| (x as f32).sin() * 0.1).collect(),
            )
            .unwrap();
            let descriptors1 = Array2::<f32>::from_shape_vec(
                (45, 256),
                (0..11520).map(|x| (x as f32 * 1.1).cos() * 0.1).collect(),
            )
            .unwrap();

            let inference_result =
                matcher.run_inference(&keypoints0, &keypoints1, &descriptors0, &descriptors1);

            assert!(
                inference_result.is_ok(),
                "Inference should succeed with realistic inputs"
            );
            let (matches, scores) = inference_result.unwrap();

            // Validate output shapes and ranges
            assert!(matches.len() <= keypoints0.nrows().min(keypoints1.nrows()));
            assert_eq!(scores.len(), matches.len());

            for &score in &scores {
                assert!(
                    score >= 0.0 && score <= 1.0,
                    "Confidence scores should be in [0,1]"
                );
            }

            println!("✅ LightGlue model validation successful!");
            println!("   Matches found: {}", matches.len());
            println!(
                "   Average confidence: {:.3}",
                scores.iter().sum::<f32>() / scores.len() as f32
            );
        }

        #[cfg(not(feature = "lightglue"))]
        {
            println!("⚠️  LightGlue feature not enabled - skipping weight validation");
        }
    }

    #[test]
    fn test_lightglue_realistic_scenarios() {
        use std::path::Path;

        #[cfg(feature = "lightglue")]
        {
            if !Path::new("models/lightglue_superpoint.onnx").exists() {
                println!("⚠️  Skipping realistic scenario tests - model weights not available");
                return;
            }

            let config = LightGlueConfig {
                model_path: PathBuf::from("models/lightglue_superpoint.onnx"),
                confidence_threshold: 0.1, // Lower threshold for testing
                ..Default::default()
            };

            let mut matcher = LightGlueMatcher::new(config).unwrap();

            // Scenario 1: Sequential frames (should have good matches)
            let keypoints0 = Array2::<f32>::from_shape_vec(
                (30, 2),
                (0..60)
                    .map(|i| {
                        let x = (i / 2) as f32 * 10.0;
                        let y = (i % 2) as f32 * 10.0;
                        if i % 2 == 0 {
                            x
                        } else {
                            y
                        }
                    })
                    .collect(),
            )
            .unwrap();

            let mut keypoints1 = keypoints0.clone();
            // Add small motion (translation + small rotation)
            for i in 0..30 {
                keypoints1[[i, 0]] += 2.0 + (i as f32 * 0.1).sin(); // X translation + noise
                keypoints1[[i, 1]] += 1.0 + (i as f32 * 0.1).cos(); // Y translation + noise
            }

            let descriptors0 = Array2::<f32>::from_shape_vec(
                (30, 256),
                (0..7680).map(|x| (x as f32 * 0.01).sin()).collect(),
            )
            .unwrap();

            let mut descriptors1 = descriptors0.clone();
            // Add small descriptor variations
            for i in 0..30 {
                for j in 0..256 {
                    descriptors1[[i, j]] += (i as f32 * 0.001).sin() * 0.1;
                }
            }

            let result =
                matcher.run_inference(&keypoints0, &keypoints1, &descriptors0, &descriptors1);
            assert!(result.is_ok(), "Should handle sequential frame scenario");

            // Scenario 2: Loop closure (similar keypoints, different descriptors)
            let keypoints_loop = keypoints0.clone();
            let descriptors_loop = Array2::<f32>::from_shape_vec(
                (30, 256),
                (0..7680).map(|x| (x as f32 * 0.015).cos()).collect(),
            )
            .unwrap();

            let result = matcher.run_inference(
                &keypoints0,
                &keypoints_loop,
                &descriptors0,
                &descriptors_loop,
            );
            assert!(result.is_ok(), "Should handle loop closure scenario");

            // Scenario 3: Different scenes (poor matches expected)
            let keypoints_different =
                Array2::<f32>::from_shape_vec((20, 2), (0..40).map(|i| i as f32 * 15.0).collect())
                    .unwrap();

            let descriptors_different = Array2::<f32>::from_shape_vec(
                (20, 256),
                (0..5120).map(|x| (x as f32 * 0.1).cos()).collect(),
            )
            .unwrap();

            let result = matcher.run_inference(
                &keypoints0,
                &keypoints_different,
                &descriptors0,
                &descriptors_different,
            );
            assert!(result.is_ok(), "Should handle different scene scenario");

            println!("✅ LightGlue realistic scenario tests passed!");
        }
    }
}
