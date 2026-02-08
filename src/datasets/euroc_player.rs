//! EuRoC MAV dataset player implementation.

use crate::datasets::io::{
    load_csv_image_timestamps, load_grayscale_image, load_imu_data, ImuFormat,
};
use crate::datasets::player::DatasetPlayer;
use crate::datasets::{ImageData, ImuData};
use anyhow::Result;
use std::path::Path;

pub struct EurocPlayer;

impl Default for EurocPlayer {
    fn default() -> Self {
        Self::new()
    }
}

impl EurocPlayer {
    pub const fn new() -> Self {
        EurocPlayer
    }
}

impl DatasetPlayer for EurocPlayer {
    fn name() -> &'static str {
        "EurocPlayer"
    }

    fn step_mode_sleep_ms() -> u64 {
        30
    }

    fn load_image_timestamps(dataset_path: &str) -> Result<Vec<ImageData>> {
        let data_file = Path::new(dataset_path).join("mav0/cam0/data.csv");
        let image_data = load_csv_image_timestamps(&data_file)?;
        log::info!("[EurocPlayer] Loaded {} image timestamps", image_data.len());
        Ok(image_data)
    }

    fn load_image(dataset_path: &str, filename: &str, cam_id: u32) -> Result<Vec<u8>> {
        let cam_folder = if cam_id == 0 { "cam0" } else { "cam1" };
        let full_path = Path::new(dataset_path)
            .join("mav0")
            .join(cam_folder)
            .join("data")
            .join(filename);
        load_grayscale_image(&full_path)
    }

    fn load_imu_data(dataset_path: &str) -> Result<Vec<ImuData>> {
        let imu_file = Path::new(dataset_path).join("mav0/imu0/data.csv");
        let (imu_data, _stats) = load_imu_data(&imu_file, ImuFormat::CsvComma, "EurocPlayer")?;
        Ok(imu_data)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::datasets::test_utils::{sample_imu_data, timestamps, write_file};
    use tempfile::tempdir;

    #[test]
    fn load_imu_data_missing_file_returns_empty() {
        let dir = tempdir().expect("tempdir");
        let dataset_path = dir.path().to_str().expect("path utf-8");

        let imu_data = EurocPlayer::load_imu_data(dataset_path).expect("load imu data");
        assert!(imu_data.is_empty());
    }

    #[test]
    fn load_imu_data_skips_malformed_lines() {
        let dir = tempdir().expect("tempdir");
        let imu_path = dir.path().join("mav0/imu0/data.csv");

        write_file(
            &imu_path,
            "timestamp,w.x,w.y,w.z,a.x,a.y,a.z\n\
not_a_timestamp,0,0,0,0,0,0\n\
1,0,0,0,0,0\n\
2,0,0,0,0,0,bad\n\
3,0.1,0.2,0.3,1.0,1.1,1.2\n",
        );

        let dataset_path = dir.path().to_str().expect("path utf-8");
        let imu_data = EurocPlayer::load_imu_data(dataset_path).expect("load imu data");
        assert_eq!(imu_data.len(), 1);
        assert_eq!(imu_data[0].timestamp, 3);
    }

    #[test]
    fn get_imu_data_between_frames_boundaries() {
        let imu_data = sample_imu_data();

        let between = EurocPlayer::get_imu_data_between_frames(2, 4, &imu_data);
        assert_eq!(timestamps(&between), vec![3, 4]);

        let between = EurocPlayer::get_imu_data_between_frames(0, 1, &imu_data);
        assert_eq!(timestamps(&between), vec![1]);

        let between = EurocPlayer::get_imu_data_between_frames(3, 3, &imu_data);
        assert!(between.is_empty());
    }
}
