#![allow(clippy::expect_used, clippy::len_zero, clippy::cast_precision_loss)]

//! Real dataset smoke tests (TUM-VI)
//! Set `RS_VIO_TUMVI_PATH` to the root of a TUM-VI sequence (contains `mav0/`).
//! Tests will skip gracefully if the dataset path is missing.

use image::GrayImage;
use rs_vio::datasets::config::Config;
use rs_vio::datasets::config::FeatureDetectionConfig;
use rs_vio::datasets::player_trait::DatasetPlayer;
use rs_vio::datasets::{ImageData, TUMVIPlayer};
use rs_vio::feature_tracker::PatchTracker;
use rs_vio::imu::{ImuConfig, ImuPreintegrator};

fn get_env_path(var: &str) -> Option<String> {
    std::env::var(var)
        .ok()
        .filter(|p| std::path::Path::new(p).exists())
}

fn gray_from_bytes(width: u32, height: u32, bytes: Vec<u8>) -> Option<GrayImage> {
    if bytes.len() != (width as usize) * (height as usize) {
        return None;
    }
    GrayImage::from_vec(width, height, bytes)
}

#[test]
fn tumvi_real_mono_patch_tracking() {
    let Some(ds_path) = get_env_path("RS_VIO_TUMVI_PATH") else {
        eprintln!("Skipping tumvi_real_mono_patch_tracking: RS_VIO_TUMVI_PATH not set or invalid");
        return;
    };

    // Load camera config for dimensions
    let cfg = Config::load("config/tum_vi.yaml").expect("config/tum_vi.yaml should exist");
    let (w, h) = (cfg.camera.image_width, cfg.camera.image_height);

    let ft_cfg = FeatureDetectionConfig::default();
    let mut tracker = PatchTracker::<3>::from_config(&ft_cfg);

    let player = TUMVIPlayer::new();
    let images = player
        .load_image_timestamps(&ds_path)
        .expect("should load TUM-VI cam0 timestamps");
    assert!(
        images.len() > 5,
        "TUM-VI sequence should have multiple frames"
    );

    // Select a few frames deterministically
    let picks = [images.len() / 12, images.len() / 6, images.len() / 4];
    for &idx in &picks {
        let ImageData { filename, .. } = &images[idx];
        let bytes = player
            .load_image(&ds_path, filename, 0)
            .expect("should load cam0 image");
        let img = gray_from_bytes(w, h, bytes).expect("image size should match config");
        tracker.process_frame(&img);
    }
}

#[test]
fn tumvi_real_imu_preintegration() {
    let Some(ds_path) = get_env_path("RS_VIO_TUMVI_PATH") else {
        eprintln!("Skipping tumvi_real_imu_preintegration: RS_VIO_TUMVI_PATH not set or invalid");
        return;
    };

    let player = TUMVIPlayer::new();
    let images = player
        .load_image_timestamps(&ds_path)
        .expect("should load TUM-VI cam0 timestamps");
    assert!(
        images.len() > 10,
        "TUM-VI sequence should have sufficient frames"
    );

    // Load IMU data into cache
    player
        .load_imu_data(&ds_path, &images, 0, images.len())
        .expect("should load TUM-VI IMU data");

    let i0 = images.len() / 5;
    let i1 = i0 + 1;
    let ts0 = images[i0].timestamp;
    let ts1 = images[i1].timestamp;

    let imus = player.get_imu_data_between_frames(ts0, ts1);
    assert!(imus.len() > 0, "IMU samples should exist between frames");

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

    assert!(result.delta_velocity.iter().all(|x| x.is_finite()));
    assert!(result.delta_position.iter().all(|x| x.is_finite()));
    assert!(result
        .delta_rotation
        .scaled_axis()
        .iter()
        .all(|x| x.is_finite()));
}
