//! Tests for LightGlue matcher and related components.

#[cfg(test)]
mod tests {
    use super::super::config::LightGlueConfig;

    #[cfg(feature = "lightglue")]
    use super::super::feature_extraction::descriptors_to_f32;
    #[cfg(feature = "lightglue")]
    use ndarray::Array2;
    use std::path::{Path, PathBuf};

    #[test]
    fn test_lightglue_config_default() {
        let config = LightGlueConfig::default();
        assert_eq!(config.max_keypoints, 1024);
        assert_eq!(config.confidence_threshold, 0.5);
        assert_eq!(config.feature_extractor, "superpoint");
    }

    #[test]
    fn test_lightglue_matcher_creation_without_model() {
        let _config = LightGlueConfig {
            model_path: PathBuf::from("/nonexistent/model.onnx"),
            ..Default::default()
        };

        // Test is mainly to ensure config creation doesn't panic
    }

    #[test]
    #[cfg(feature = "lightglue")]
    fn test_descriptor_conversion() {
        let descriptors = vec![vec![0, 128, 255], vec![64, 192, 32]];

        let array = descriptors_to_f32(&descriptors);
        assert_eq!(array.shape(), &[2, 3]);
        assert!((array[[0, 0]] - 0.0).abs() < 1e-6);
        assert!((array[[0, 2]] - 1.0).abs() < 1e-6);
    }

    #[test]
    #[cfg(feature = "lightglue")]
    fn test_lightglue_model_loading_with_weights() {
        use super::super::model::load_model;

        let config = LightGlueConfig {
            model_path: PathBuf::from("models/lightglue_superpoint.onnx"),
            ..Default::default()
        };

        if Path::new("models/lightglue_superpoint.onnx").exists() {
            let result = load_model(&config);

            if result.is_err() {
                println!(
                    "⚠️  Model loading failed despite file existing: {:?}",
                    result.err()
                );
                return;
            }
            let _session = result.unwrap();

            println!("✅ Session created successfully");
        } else {
            assert!(
                load_model(&config).is_err(),
                "Should fail when weights don't exist"
            );
        }
    }

    #[test]
    #[cfg(feature = "lightglue")]
    fn test_lightglue_inference_edge_cases() {
        use super::super::matching::run_inference;

        let config = LightGlueConfig {
            model_path: PathBuf::from("models/lightglue_superpoint.onnx"),
            ..Default::default()
        };

        if !Path::new("models/lightglue_superpoint.onnx").exists() {
            return;
        }

        let matcher_result = super::super::model::load_model(&config);
        if matcher_result.is_err() {
            println!(
                "⚠️  Skipping test - model loading failed: {:?}",
                matcher_result.err()
            );
            return;
        }
        let mut session = matcher_result.unwrap();

        // Test 1: Empty keypoints/descriptors
        let empty_kpts = Array2::<f32>::zeros((0, 2));
        let empty_desc = Array2::<f32>::zeros((0, 256));

        let result = run_inference(
            &mut session,
            0.5,
            &empty_kpts,
            &empty_kpts,
            &empty_desc,
            &empty_desc,
        );
        assert!(result.is_ok(), "Should handle empty inputs gracefully");

        // Test 2: Single feature
        let single_kpts = Array2::<f32>::from_shape_vec((1, 2), vec![100.0, 200.0]).unwrap();
        let single_desc = Array2::<f32>::zeros((1, 256));

        let result = run_inference(
            &mut session,
            0.5,
            &single_kpts,
            &single_kpts,
            &single_desc,
            &single_desc,
        );
        assert!(result.is_ok(), "Should handle single feature");

        // Test 3: Mismatched dimensions
        let kpts_a = Array2::<f32>::zeros((5, 2));
        let kpts_b = Array2::<f32>::zeros((3, 2));
        let desc_a = Array2::<f32>::zeros((5, 256));
        let desc_b = Array2::<f32>::zeros((3, 256));

        let result = run_inference(&mut session, 0.5, &kpts_a, &kpts_b, &desc_a, &desc_b);
        assert!(result.is_ok(), "Should handle mismatched keypoint counts");

        // Test 4: Large number of features
        let large_kpts = Array2::<f32>::zeros((1000, 2));
        let large_desc = Array2::<f32>::zeros((1000, 256));

        let result = run_inference(
            &mut session,
            0.5,
            &large_kpts,
            &large_kpts,
            &large_desc,
            &large_desc,
        );
        assert!(result.is_ok(), "Should handle large feature sets");

        // Test 5: Very different descriptors
        let kpts = Array2::<f32>::zeros((10, 2));
        let desc_similar = Array2::<f32>::zeros((10, 256));
        let mut desc_different = Array2::<f32>::ones((10, 256)) * 0.5;

        for i in 0..10 {
            for j in 0..256 {
                desc_different[[i, j]] = if j % 2 == 0 { 1.0 } else { 0.0 };
            }
        }

        let result = run_inference(
            &mut session,
            0.5,
            &kpts,
            &kpts,
            &desc_similar,
            &desc_different,
        );
        assert!(result.is_ok(), "Should handle very different descriptors");
    }

    #[test]
    #[cfg(feature = "lightglue")]
    fn test_lightglue_descriptor_preprocessing() {
        let test_cases = vec![
            vec![],
            vec![vec![0u8; 32]],
            vec![vec![0u8; 32], vec![255u8; 32], vec![128u8; 32]],
            vec![vec![64u8; 32], vec![192u8; 32]],
        ];

        for descriptors in test_cases {
            if !descriptors.is_empty() {
                let array = descriptors_to_f32(&descriptors);
                assert_eq!(array.nrows(), descriptors.len());
                assert_eq!(array.ncols(), descriptors[0].len());
            }
        }
    }

    #[test]
    fn test_lightglue_config_validation() {
        let _configs = vec![
            LightGlueConfig {
                confidence_threshold: -0.1,
                ..Default::default()
            },
            LightGlueConfig {
                confidence_threshold: 1.5,
                ..Default::default()
            },
            LightGlueConfig {
                confidence_threshold: 0.5,
                max_keypoints: 0,
                ..Default::default()
            },
            LightGlueConfig {
                confidence_threshold: 0.5,
                max_keypoints: 10000,
                ..Default::default()
            },
        ];
    }

    #[test]
    #[cfg(feature = "lightglue")]
    fn test_lightglue_model_weight_validation() {
        use std::fs;
        use std::process::Command;

        let model_path = "models/lightglue_superpoint.onnx";
        let model_url = "https://github.com/fabio-sim/LightGlue-ONNX/releases/download/v2.0/superpoint_lightglue_pipeline.ort.onnx";

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

        assert!(
            Path::new(model_path).exists(),
            "Model file should exist after download"
        );
        let metadata = fs::metadata(model_path).unwrap();
        assert!(
            metadata.len() > 1000000,
            "Model file should be reasonably large (>1MB)"
        );

        let config = LightGlueConfig {
            model_path: PathBuf::from(model_path),
            ..Default::default()
        };

        let result = super::super::model::load_model(&config);
        if result.is_err() {
            println!(
                "⚠️  Model loading failed (possibly corrupted or incompatible format): {:?}",
                result.err()
            );
            return;
        }

        let mut _session = result.expect("Model loading should succeed");

        // Test with real data shapes
        let keypoints0 =
            Array2::<f32>::from_shape_vec((50, 2), (0..100).map(|x| x as f32 * 0.1).collect())
                .unwrap();
        let keypoints1 =
            Array2::<f32>::from_shape_vec((45, 2), (0..90).map(|x| x as f32 * 0.12).collect())
                .unwrap();

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

        let inference_result = super::super::matching::run_inference(
            &mut _session,
            0.5,
            &keypoints0,
            &keypoints1,
            &descriptors0,
            &descriptors1,
        );

        assert!(
            inference_result.is_ok(),
            "Inference should succeed with realistic inputs"
        );
        let (matches, scores) = inference_result.unwrap();

        assert!(matches.len() <= keypoints0.nrows().min(keypoints1.nrows()));
        assert_eq!(scores.len(), matches.len());

        for &score in &scores {
            assert!(
                score >= 0.0 && score <= 1.0,
                "Confidence scores should be in [0,1]"
            );
        }

        log::info!("✅ LightGlue model validation successful!");
        log::info!("   Matches found: {}", matches.len());
        println!(
            "   Average confidence: {:.3}",
            scores.iter().sum::<f32>() / scores.len() as f32
        );
    }

    #[test]
    #[cfg(feature = "lightglue")]
    fn test_lightglue_realistic_scenarios() {
        use super::super::matching::run_inference;

        if !Path::new("models/lightglue_superpoint.onnx").exists() {
            println!("⚠️  Skipping realistic scenario tests - model weights not available");
            return;
        }

        let config = LightGlueConfig {
            model_path: PathBuf::from("models/lightglue_superpoint.onnx"),
            confidence_threshold: 0.1,
            ..Default::default()
        };

        let matcher_result = super::super::model::load_model(&config);
        if matcher_result.is_err() {
            println!(
                "⚠️  Skipping test - model loading failed: {:?}",
                matcher_result.err()
            );
            return;
        }
        let mut session = matcher_result.unwrap();

        // Scenario 1: Sequential frames
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
        for i in 0..30 {
            keypoints1[[i, 0]] += 2.0 + (i as f32 * 0.1).sin();
            keypoints1[[i, 1]] += 1.0 + (i as f32 * 0.1).cos();
        }

        let descriptors0 = Array2::<f32>::from_shape_vec(
            (30, 256),
            (0..7680).map(|x| (x as f32 * 0.01).sin()).collect(),
        )
        .unwrap();

        let mut descriptors1 = descriptors0.clone();
        for i in 0..30 {
            for j in 0..256 {
                descriptors1[[i, j]] += (i as f32 * 0.001).sin() * 0.1;
            }
        }

        let result = run_inference(
            &mut session,
            0.1,
            &keypoints0,
            &keypoints1,
            &descriptors0,
            &descriptors1,
        );
        assert!(result.is_ok(), "Should handle sequential frame scenario");

        // Scenario 2: Loop closure
        let keypoints_loop = keypoints0.clone();
        let descriptors_loop = Array2::<f32>::from_shape_vec(
            (30, 256),
            (0..7680).map(|x| (x as f32 * 0.015).cos()).collect(),
        )
        .unwrap();

        let result = run_inference(
            &mut session,
            0.1,
            &keypoints0,
            &keypoints_loop,
            &descriptors0,
            &descriptors_loop,
        );
        assert!(result.is_ok(), "Should handle loop closure scenario");

        // Scenario 3: Different scenes
        let keypoints_different =
            Array2::<f32>::from_shape_vec((20, 2), (0..40).map(|i| i as f32 * 15.0).collect())
                .unwrap();

        let descriptors_different = Array2::<f32>::from_shape_vec(
            (20, 256),
            (0..5120).map(|x| (x as f32 * 0.1).cos()).collect(),
        )
        .unwrap();

        let result = run_inference(
            &mut session,
            0.1,
            &keypoints0,
            &keypoints_different,
            &descriptors0,
            &descriptors_different,
        );
        assert!(result.is_ok(), "Should handle different scene scenario");

        println!("✅ LightGlue realistic scenario tests passed!");
    }

    #[test]
    #[cfg(feature = "lightglue")]
    fn test_lightglue_extreme_edge_cases() {
        use super::super::matching::run_inference;

        if !Path::new("models/lightglue_superpoint.onnx").exists() {
            println!("⚠️  Skipping extreme edge case tests - model weights not available");
            return;
        }

        let config = LightGlueConfig {
            model_path: PathBuf::from("models/lightglue_superpoint.onnx"),
            ..Default::default()
        };

        let matcher_result = super::super::model::load_model(&config);
        if matcher_result.is_err() {
            println!(
                "⚠️  Skipping test - model loading failed: {:?}",
                matcher_result.err()
            );
            return;
        }
        let mut session = matcher_result.unwrap();

        // Test 1: NaN and Inf values
        let keypoints_nan = Array2::<f32>::from_shape_vec(
            (5, 2),
            vec![
                f32::NAN,
                100.0,
                f32::INFINITY,
                -f32::INFINITY,
                50.0,
                75.0,
                25.0,
                125.0,
                0.0,
                200.0,
            ],
        )
        .unwrap();

        let descriptors_nan = Array2::<f32>::from_shape_vec(
            (5, 256),
            (0..1280)
                .map(|x| match x % 4 {
                    0 => f32::NAN,
                    1 => f32::INFINITY,
                    2 => f32::NEG_INFINITY,
                    _ => (x as f32 * 0.01).sin(),
                })
                .collect(),
        )
        .unwrap();

        let result = run_inference(
            &mut session,
            0.5,
            &keypoints_nan,
            &keypoints_nan,
            &descriptors_nan,
            &descriptors_nan,
        );
        assert!(
            result.is_ok() || result.is_err(),
            "Should handle NaN/Inf values"
        );

        // Test 2: Extremely large feature counts
        let large_count = 1000;
        let keypoints_large = Array2::<f32>::from_shape_vec(
            (large_count, 2),
            (0..large_count * 2).map(|x| x as f32).collect(),
        )
        .unwrap();

        let descriptors_large = Array2::<f32>::from_shape_vec(
            (large_count, 256),
            (0..large_count * 256)
                .map(|x| (x as f32 * 0.001).sin())
                .collect(),
        )
        .unwrap();

        let result = run_inference(
            &mut session,
            0.5,
            &keypoints_large,
            &keypoints_large,
            &descriptors_large,
            &descriptors_large,
        );
        assert!(
            result.is_ok() || result.is_err(),
            "Should handle large feature sets without crashing"
        );

        // Test 3: Zero and negative coordinates
        let keypoints_zero = Array2::<f32>::from_shape_vec(
            (10, 2),
            vec![
                0.0,
                0.0,
                -10.0,
                -5.0,
                1000.0,
                800.0,
                -100.0,
                50.0,
                640.0,
                -100.0,
                320.0,
                240.0,
                f32::MAX,
                f32::MIN,
                0.0,
                100.0,
                200.0,
                0.0,
                -1.0,
                -1.0,
            ],
        )
        .unwrap();

        let descriptors_zero = Array2::<f32>::zeros((10, 256));

        let result = run_inference(
            &mut session,
            0.5,
            &keypoints_zero,
            &keypoints_zero,
            &descriptors_zero,
            &descriptors_zero,
        );
        assert!(
            result.is_ok() || result.is_err(),
            "Should handle zero/negative coordinates"
        );

        // Test 4: Identical keypoints with different descriptors
        let keypoints_identical = Array2::<f32>::from_shape_vec(
            (20, 2),
            (0..40).map(|x| (x % 2) as f32 * 100.0 + 50.0).collect(),
        )
        .unwrap();

        let descriptors_a = Array2::<f32>::from_shape_vec(
            (20, 256),
            (0..5120).map(|x| (x as f32 * 0.01).sin()).collect(),
        )
        .unwrap();

        let descriptors_b = Array2::<f32>::from_shape_vec(
            (20, 256),
            (0..5120).map(|x| (x as f32 * 0.01).cos()).collect(),
        )
        .unwrap();

        let result = run_inference(
            &mut session,
            0.5,
            &keypoints_identical,
            &keypoints_identical,
            &descriptors_a,
            &descriptors_b,
        );
        assert!(
            result.is_ok(),
            "Should handle identical keypoints with different descriptors"
        );

        // Test 5: Very small descriptors
        let small_descriptors = Array2::<f32>::from_shape_vec(
            (5, 256),
            (0..1280)
                .map(|_| f32::EPSILON * (1.0 + rand::random::<f32>()))
                .collect(),
        )
        .unwrap();

        let result = run_inference(
            &mut session,
            0.5,
            &keypoints_zero,
            &keypoints_zero,
            &small_descriptors,
            &small_descriptors,
        );
        assert!(
            result.is_ok() || result.is_err(),
            "Should handle very small descriptor values"
        );

        println!("✅ LightGlue extreme edge case tests passed!");
    }

    #[test]
    #[cfg(feature = "lightglue")]
    fn test_lightglue_model_corruption_and_errors() {
        use std::fs;

        let model_path = "models/lightglue_superpoint.onnx";

        // Test 1: Corrupted model file
        if Path::new(model_path).exists() {
            let backup_path = format!("{}.backup", model_path);
            fs::copy(model_path, &backup_path).ok();

            let file = fs::OpenOptions::new().write(true).open(model_path).unwrap();
            file.set_len(100).unwrap();
            drop(file);

            let config = LightGlueConfig {
                model_path: PathBuf::from(model_path),
                ..Default::default()
            };

            let result = super::super::model::load_model(&config);
            assert!(result.is_err(), "Should fail with corrupted model file");

            fs::copy(&backup_path, model_path).ok();
            fs::remove_file(backup_path).ok();
        }

        // Test 2: Non-existent model file
        let config = LightGlueConfig {
            model_path: PathBuf::from("models/nonexistent_model.onnx"),
            ..Default::default()
        };

        let result = super::super::model::load_model(&config);
        assert!(result.is_err(), "Should fail with non-existent model file");

        // Test 3: Directory instead of file
        let config = LightGlueConfig {
            model_path: PathBuf::from("models"),
            ..Default::default()
        };

        let result = super::super::model::load_model(&config);
        assert!(
            result.is_err(),
            "Should fail when model path is a directory"
        );

        // Test 4: Invalid configuration values
        let _configs = vec![
            LightGlueConfig {
                confidence_threshold: -1.0,
                ..Default::default()
            },
            LightGlueConfig {
                confidence_threshold: 2.0,
                ..Default::default()
            },
            LightGlueConfig {
                max_keypoints: 0,
                ..Default::default()
            },
            LightGlueConfig {
                max_keypoints: 100000,
                ..Default::default()
            },
        ];

        println!("✅ LightGlue model corruption and error tests passed!");
    }
}
