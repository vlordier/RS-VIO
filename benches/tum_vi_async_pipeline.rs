#![allow(clippy::expect_used, clippy::unwrap_used, clippy::cast_precision_loss)]

use criterion::{criterion_group, criterion_main, Criterion};
use image::GrayImage;
use rs_vio::datasets::config::Config;
use rs_vio::datasets::{DatasetPlayer, ImageData, TUMVIPlayer};
use rs_vio::estimator::{AsyncOptimizer, Frame};
use rs_vio::feature_tracker::{AsyncDetectorConfig, AsyncFeatureDetector};
use std::sync::Arc;
use tokio::runtime::Builder;

struct StereoFrame {
    left: GrayImage,
    right: GrayImage,
    timestamp: i64,
}

fn get_env_path(var: &str) -> Option<String> {
    std::env::var(var)
        .ok()
        .filter(|p| std::path::Path::new(p).exists())
}

/// Locate a TUM-VI sequence. Preference order:
/// 1) RS_VIO_TUMVI_PATH env var
/// 2) Repo datasets/tum_vi/<sequence>/ if available
fn find_tumvi_root() -> Option<String> {
    if let Some(p) = get_env_path("RS_VIO_TUMVI_PATH") {
        return Some(p);
    }

    let repo_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("datasets")
        .join("tum_vi");
    if !repo_path.exists() {
        return None;
    }

    // Prefer a standard sequence if present
    let default_seq = repo_path.join("room1");
    if default_seq.join("mav0").exists() {
        return Some(default_seq.to_string_lossy().into_owned());
    }

    // Pick first available sequence directory (e.g., room1/, magistrale1/)
    let mut candidates = std::fs::read_dir(repo_path)
        .ok()?
        .filter_map(|e| e.ok())
        .filter(|e| e.path().join("mav0").exists())
        .collect::<Vec<_>>();
    candidates.sort_by_key(|e| e.path());
    candidates
        .first()
        .map(|e| e.path().to_string_lossy().into_owned())
}

fn gray_from_bytes(width: u32, height: u32, bytes: Vec<u8>) -> Option<GrayImage> {
    if bytes.len() != (width as usize) * (height as usize) {
        return None;
    }
    GrayImage::from_vec(width, height, bytes)
}

fn load_tumvi_frames(max_frames: usize) -> Option<(Vec<StereoFrame>, Config)> {
    let ds_path = find_tumvi_root()?;
    let vio_cfg = Config::load("config/tum_vi.yaml").ok()?;
    let (w, h) = (vio_cfg.camera.image_width, vio_cfg.camera.image_height);

    let images = TUMVIPlayer::load_image_timestamps(&ds_path).ok()?;
    if images.len() < 2 {
        return None;
    }

    let mut frames = Vec::new();
    for (idx, ImageData { filename, .. }) in images.iter().enumerate().take(max_frames) {
        let bytes_left = TUMVIPlayer::load_image(&ds_path, filename, 0).ok()?;
        let bytes_right = TUMVIPlayer::load_image(&ds_path, filename, 1).ok()?;
        let left = gray_from_bytes(w, h, bytes_left)?;
        let right = gray_from_bytes(w, h, bytes_right)?;
        frames.push(StereoFrame {
            left,
            right,
            timestamp: images[idx].timestamp,
        });
    }

    Some((frames, vio_cfg))
}

fn bench_tumvi_async_pipeline(c: &mut Criterion) {
    // Load dataset once - increased to 1000 frames for better measurement
    let Some((frames, vio_cfg)) = load_tumvi_frames(1000) else {
        println!("Skipping tumvi_async_pipeline: set RS_VIO_TUMVI_PATH to a TUM-VI sequence root (contains mav0/)");
        return;
    };

    let n_frames = frames.len();
    let grid_cell_size = (vio_cfg.camera.image_width as usize)
        .saturating_div(vio_cfg.feature_detection.grid_cols.max(1) as usize)
        .max(8);

    let detector_config = AsyncDetectorConfig {
        num_parallel_tasks: 4,
        grid_cell_size,
        max_features: (vio_cfg.feature_detection.max_features_per_grid as usize)
            .saturating_mul(vio_cfg.feature_detection.grid_cols as usize)
            .max(200),
        threshold: 20.0,
    };

    // Use current-thread runtime because AsyncOptimizer Backend is !Send
    let rt = Builder::new_current_thread()
        .enable_all()
        .build()
        .expect("tokio runtime");

    let mut group = c.benchmark_group("async_pipeline");
    group.throughput(criterion::Throughput::Elements(n_frames as u64));
    group.bench_function("tumvi_1000_frames", |b| {
        b.iter(|| {
            // Fresh detector and optimizer each iteration to avoid state carryover
            let detector = AsyncFeatureDetector::new(detector_config.clone());
            let optimizer = AsyncOptimizer::new();

            rt.block_on(async {
                for (idx, f) in frames.iter().enumerate() {
                    let mut frame = Frame::new(f.timestamp, idx as i32);
                    frame.is_keyframe = idx % 5 == 0;

                    // Process both stereo images
                    let left_bytes = Arc::new(f.left.clone().into_vec());
                    let right_bytes = Arc::new(f.right.clone().into_vec());

                    let _left_features = detector
                        .detect_async(
                            left_bytes,
                            vio_cfg.camera.image_width,
                            vio_cfg.camera.image_height,
                        )
                        .await;
                    let _right_features = detector
                        .detect_async(
                            right_bytes,
                            vio_cfg.camera.image_width,
                            vio_cfg.camera.image_height,
                        )
                        .await;

                    if frame.is_keyframe {
                        let _ = optimizer.optimize().await;
                    }
                }
            });
        })
    });
    group.finish();
}

/// Synthetic benchmark that runs without TUM-VI dataset.
///
/// Measures async detection + optimization throughput using
/// checkerboard-textured images. Always runnable in CI.
fn bench_synthetic_async(c: &mut Criterion) {
    let vio_cfg = Config::load("config/tum_vi.yaml").expect("Config should load");
    let (w, h) = (vio_cfg.camera.image_width, vio_cfg.camera.image_height);

    let n_frames = 200usize;
    let frames: Vec<StereoFrame> = (0..n_frames)
        .map(|i| {
            let phase = i as f64 * 0.05;
            let mut buf = vec![0u8; (w * h) as usize];
            for y in 0..h {
                for x in 0..w {
                    let grad = ((x as f64 / w as f64 + phase).sin() * 127.0 + 128.0) as u8;
                    let checker = if ((x / 32) + (y / 32)) % 2 == 0 { 40u8 } else { 0 };
                    let noise = ((x.wrapping_mul(7).wrapping_add(y.wrapping_mul(13))) % 17) as u8;
                    buf[(y * w + x) as usize] = grad.saturating_add(checker).saturating_add(noise);
                }
            }
            let img = GrayImage::from_vec(w, h, buf.clone()).unwrap();
            let img2 = GrayImage::from_vec(w, h, buf).unwrap();
            StereoFrame {
                left: img,
                right: img2,
                timestamp: (i as i64) * 33_333,
            }
        })
        .collect();

    let grid_cell_size = (w as usize)
        .saturating_div(vio_cfg.feature_detection.grid_cols.max(1) as usize)
        .max(8);

    let detector_config = AsyncDetectorConfig {
        num_parallel_tasks: 4,
        grid_cell_size,
        max_features: 200,
        threshold: 20.0,
    };

    let rt = Builder::new_current_thread()
        .enable_all()
        .build()
        .expect("tokio runtime");

    let mut group = c.benchmark_group("synthetic_async");
    group.throughput(criterion::Throughput::Elements(n_frames as u64));
    group.bench_function("200_frames", |b| {
        b.iter(|| {
            let detector = AsyncFeatureDetector::new(detector_config.clone());
            let optimizer = AsyncOptimizer::new();
            rt.block_on(async {
                for (idx, f) in frames.iter().enumerate() {
                    let mut frame = Frame::new(f.timestamp, idx as i32);
                    frame.is_keyframe = idx % 5 == 0;
                    let left_bytes = Arc::new(f.left.clone().into_vec());
                    let right_bytes = Arc::new(f.right.clone().into_vec());
                    let _left = detector.detect_async(left_bytes, w, h).await;
                    let _right = detector.detect_async(right_bytes, w, h).await;
                    if frame.is_keyframe {
                        let _ = optimizer.optimize().await;
                    }
                }
            });
        })
    });
    group.finish();
}

criterion_group!(benches, bench_tumvi_async_pipeline, bench_synthetic_async);
criterion_main!(benches);
