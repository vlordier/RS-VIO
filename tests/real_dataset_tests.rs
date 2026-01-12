#![allow(
    clippy::expect_used,
    clippy::len_zero,
    clippy::cast_precision_loss
)]

//! Real dataset smoke tests (EuRoC)
//! These tests use a few real frames and IMU samples when the environment provides a dataset path.
//! Set `RS_VIO_EUROC_PATH` to the root of an EuRoC sequence (folder containing `mav0/`).
//! Tests skip gracefully if the dataset is not available.

use rs_vio::datasets::{EurocPlayer, ImageData};
use rs_vio::datasets::player_trait::DatasetPlayer;
use rs_vio::datasets::config::{Config, FeatureDetectionConfig};
use rs_vio::feature_tracker::PatchTracker;
use rs_vio::imu::{ImuConfig, ImuPreintegrator};
use image::GrayImage;

fn get_env_path(var: &str) -> Option<String> {
    std::env::var(var).ok().filter(|p| std::path::Path::new(p).exists())
}

fn gray_from_bytes(width: u32, height: u32, bytes: Vec<u8>) -> Option<GrayImage> {
    if bytes.len() != (width as usize) * (height as usize) {
        return None;
    }
    GrayImage::from_vec(width, height, bytes)
}

#[test]
fn euroc_real_mono_patch_tracking() {
    let Some(ds_path) = get_env_path("RS_VIO_EUROC_PATH") else {
        eprintln!("Skipping euroc_real_mono_patch_tracking: RS_VIO_EUROC_PATH not set or invalid");
        return;
    };

    // Load EuRoC camera config for dimensions
    let cfg = Config::load("config/euroc_vio.yaml").expect("config/euroc_vio.yaml should exist");
    let (w, h) = (cfg.camera.image_width, cfg.camera.image_height);

    // Prepare tracker
    let ft = FeatureDetectionConfig::default();
    let mut tracker = PatchTracker::<3>::from_config(&ft);

    // Load image timestamps (cam0)
    let player = EurocPlayer::new();
    let images = player
        .load_image_timestamps(&ds_path)
        .expect("should load EuRoC cam0 timestamps");
    assert!(images.len() > 5, "EuRoC sequence should have multiple frames");

    // Pick a few indices deterministically
    let picks = [images.len() / 10, images.len() / 5, images.len() / 3];

    for &idx in &picks {
        let ImageData { filename, .. } = &images[idx];
        // Load cam0 image bytes
        let bytes = player
            .load_image(&ds_path, filename, 0)
            .expect("should load cam0 image");
        let img = gray_from_bytes(w, h, bytes).expect("image size should match config");

        // Process
        tracker.process_frame(&img);
    }
}

#[test]
fn euroc_real_imu_preintegration() {
    let Some(ds_path) = get_env_path("RS_VIO_EUROC_PATH") else {
        eprintln!("Skipping euroc_real_imu_preintegration: RS_VIO_EUROC_PATH not set or invalid");
        return;
    };

    let player = EurocPlayer::new();
    let images = player
        .load_image_timestamps(&ds_path)
        .expect("should load EuRoC cam0 timestamps");
    assert!(images.len() > 10, "EuRoC sequence should have sufficient frames");

    // Load IMU CSV into cache
    player
        .load_imu_data(&ds_path, &images, 0, images.len())
        .expect("should load EuRoC IMU data");

    // Take two consecutive image timestamps
    let i0 = images.len() / 4;
    let i1 = i0 + 1;
    let ts0 = images[i0].timestamp;
    let ts1 = images[i1].timestamp;

    // Get IMU samples between them
    let imus = player.get_imu_data_between_frames(ts0, ts1);
    assert!(imus.len() > 0, "IMU samples should exist between frames");

    // Preintegrate
    let mut pre = ImuPreintegrator::new(ImuConfig::default());
    let mut prev_ts = ts0;
    for imu in &imus {
        let dt = (imu.timestamp - prev_ts) as f64 / 1e9;
        if dt > 0.0 {
            pre.propagate(imu, dt);
            prev_ts = imu.timestamp;
        }
    }
    let result = pre.get();

    // Sanity: all values finite
    assert!(result.delta_velocity.iter().all(|x| x.is_finite()));
    assert!(result.delta_position.iter().all(|x| x.is_finite()));
    assert!(result
        .delta_rotation
        .scaled_axis()
        .iter()
        .all(|x| x.is_finite()));
}
