#![allow(clippy::expect_used, clippy::cast_precision_loss)]

use criterion::{criterion_group, criterion_main, Criterion};
use image::GrayImage;
use rs_vio::datasets::config::Config;
use rs_vio::datasets::{ImageData, TUMVIPlayer};
use rs_vio::estimator::{Frame, SlidingWindow};
use rs_vio::feature_tracker::{EnhancedDetectorConfig, EnhancedFeatureDetector};

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

fn bench_tumvi_sequential_baseline(c: &mut Criterion) {
    // Load dataset once - increased to 1000 frames for better measurement
    let Some((frames, vio_cfg)) = load_tumvi_frames(1000) else {
        println!("Skipping tumvi_sequential_baseline: set RS_VIO_TUMVI_PATH to a TUM-VI sequence root (contains mav0/)");
        return;
    };

    let detector_config = EnhancedDetectorConfig {
        max_features: (vio_cfg.feature_detection.max_features_per_grid as usize)
            .saturating_mul(vio_cfg.feature_detection.grid_cols as usize)
            .max(200),
        fast_threshold: 20,
        min_distance: 8.0,
        ..Default::default()
    };

    c.bench_function("tumvi_sequential_baseline_1000_frames", |b| {
        b.iter(|| {
            // Fresh detector and sliding window each iteration to avoid state carryover
            let detector = EnhancedFeatureDetector::new(detector_config.clone());
            let mut sliding_window = SlidingWindow::with_default_size();

            for (idx, f) in frames.iter().enumerate() {
                let mut frame = Frame::new(f.timestamp, idx as i32);
                frame.is_keyframe = idx % 5 == 0;

                // Sequential feature detection on both stereo images (synchronous)
                let _left_features = detector.detect(&f.left);
                let _right_features = detector.detect(&f.right);

                // Sequential optimization (only on keyframes)
                if frame.is_keyframe {
                    sliding_window.add_frame(frame.clone());
                    let _ = sliding_window.optimize();
                }
            }
        })
    });
}

criterion_group!(benches, bench_tumvi_sequential_baseline);
criterion_main!(benches);
