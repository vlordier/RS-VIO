//! Common trait for dataset players

use crate::datasets::{ImageData, ImuData};
use anyhow::Result;

/// Trait for dataset players (EuRoC, TUM-VI, 4Seasons, etc.)
pub trait DatasetPlayer {
    /// Get the name of the dataset player
    fn name() -> &'static str;

    /// Sleep duration (in milliseconds) for step mode between frames
    fn step_mode_sleep_ms() -> u64 {
        30 // Default: 30ms
    }

    /// Load image timestamps from the dataset
    fn load_image_timestamps(dataset_path: &str) -> Result<Vec<ImageData>>;

    /// Load a single image from the dataset
    ///
    /// # Arguments
    /// * `dataset_path` - Root path to the dataset
    /// * `filename` - Image filename
    /// * `cam_id` - Camera ID (0 for left/cam0, 1 for right/cam1)
    fn load_image(dataset_path: &str, filename: &str, cam_id: u32) -> Result<Vec<u8>>;

    /// Load IMU data from the dataset
    fn load_imu_data(dataset_path: &str) -> Result<Vec<ImuData>>;

    /// Get IMU measurements between two frame timestamps
    ///
    /// Returns all IMU samples with timestamps in the range (prev_ts, current_ts].
    ///
    /// # Arguments
    /// * `prev_timestamp_ns` - Previous frame timestamp (exclusive)
    /// * `current_timestamp_ns` - Current frame timestamp (inclusive)
    /// * `imu_data` - All available IMU data
    fn get_imu_data_between_frames(
        prev_timestamp_ns: i64,
        current_timestamp_ns: i64,
        imu_data: &[ImuData],
    ) -> Vec<ImuData> {
        imu_data
            .iter()
            .filter(|imu| {
                imu.timestamp > prev_timestamp_ns && imu.timestamp <= current_timestamp_ns
            })
            .cloned()
            .collect()
    }
}
