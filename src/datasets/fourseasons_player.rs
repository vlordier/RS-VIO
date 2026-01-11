use crate::datasets::{
    config::Config, player_trait::DatasetPlayer, FrameContext, ImageData, ImuData, PlayerConfig,
    PlayerResult,
};
use crate::estimator::Estimator;
use crate::{Result, VIOError};
use image::ImageReader;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

#[derive(Default)]
pub struct FourSeasonsPlayer;

impl FourSeasonsPlayer {
    pub fn new() -> Self {
        FourSeasonsPlayer
    }
}

impl DatasetPlayer for FourSeasonsPlayer {
    fn run(&self, config: PlayerConfig) -> crate::Result<PlayerResult> {
        crate::datasets::player_trait::execute(self, config, "FourSeasonsPlayer")
    }

    fn load_image_timestamps(&self, dataset_path: &str) -> Result<Vec<ImageData>> {
        let data_file = Path::new(dataset_path).join("mav0/cam0/data.csv");
        let file = File::open(&data_file).map_err(|e| {
            VIOError::Config(format!(
                "Cannot open data.csv file {}: {e}",
                data_file.display()
            ))
        })?;

        let reader = BufReader::new(file);
        let mut image_data = Vec::new();

        for (line_num, line) in reader.lines().enumerate() {
            let line = line.map_err(|e| {
                VIOError::Config(format!(
                    "Failed to read data.csv line {} ({}): {e}",
                    line_num,
                    data_file.display()
                ))
            })?;

            // Skip header and empty lines
            if line_num == 0 || line.trim().is_empty() || line.trim_start().starts_with('#') {
                continue;
            }

            let parts: Vec<&str> = line.split(',').collect();
            if parts.len() >= 2 {
                let timestamp_str = parts[0].trim();
                let filename = parts[1].trim().to_string();

                if let Ok(timestamp) = timestamp_str.parse::<i64>() {
                    image_data.push(ImageData {
                        timestamp,
                        filename,
                    });
                }
            }
        }

        log::info!(
            "[FourSeasonsPlayer] Loaded {} image timestamps",
            image_data.len()
        );
        Ok(image_data)
    }

    fn load_image(&self, dataset_path: &str, filename: &str, cam_id: u32) -> Result<Vec<u8>> {
        let cam_folder = if cam_id == 0 { "cam0" } else { "cam1" };
        let full_path = Path::new(dataset_path)
            .join("mav0")
            .join(cam_folder)
            .join("data")
            .join(filename);

        if !full_path.exists() {
            return Err(VIOError::Image(format!(
                "Cannot load image: {}",
                full_path.display()
            )));
        }

        // Load image using image crate
        let img = ImageReader::open(&full_path)
            .map_err(|e| {
                VIOError::Image(format!("Failed to open image {}: {e}", full_path.display()))
            })?
            .decode()
            .map_err(|e| {
                VIOError::Image(format!(
                    "Failed to decode image {}: {e}",
                    full_path.display()
                ))
            })?;

        // Convert to grayscale if needed
        let gray_img = img.to_luma8();

        // Return raw pixel data as Vec<u8>
        let pixel_data = gray_img.as_raw().to_vec();

        Ok(pixel_data)
    }

    fn load_imu_data(
        &self,
        _dataset_path: &str,
        _image_data: &[ImageData],
        _start_frame_idx: usize,
        _end_frame_idx: usize,
    ) -> Result<()> {
        // TODO: Implement IMU data loading
        log::info!("[FourSeasonsPlayer] IMU data loading (placeholder)");
        Ok(())
    }

    fn get_imu_data_between_frames(
        &self,
        _previous_timestamp: i64,
        _current_timestamp: i64,
    ) -> Vec<ImuData> {
        // TODO: Implement IMU data retrieval between timestamps
        Vec::new()
    }

    fn process_single_frame(
        &self,
        estimator: &mut Estimator,
        context: &mut FrameContext,
        image_data: &[ImageData],
        dataset_path: &str,
    ) -> Result<f64> {
        crate::datasets::player_trait::process_single_frame_common(
            estimator,
            context,
            image_data,
            dataset_path,
            |ds_path, filename, cam_id| self.load_image(ds_path, filename, cam_id),
            |ds_path, filename, cam_id| self.load_image(ds_path, filename, cam_id),
        )
    }

    fn save_statistics(&self, result: &PlayerResult, stats_path: &Path) {
        crate::datasets::player_trait::save_statistics_common(result, stats_path);
    }

    fn save_trajectories(
        &self,
        _estimator: &Estimator,
        _context: &FrameContext,
        _dataset_path: &str,
    ) {
        // TODO: Implement trajectory saving
        log::debug!("[FourSeasonsPlayer] Saving trajectories (placeholder)");
    }

    fn create_camera_models_from_config(
        &self,
        config: &Config,
    ) -> Result<(
        crate::datasets::CameraModelType,
        crate::datasets::CameraModelType,
    )> {
        Ok(crate::datasets::create_camera_models_from_config(config))
    }

    fn initialize_estimator(&self, _estimator: &mut Estimator, _image_data: &[ImageData]) {
        // TODO: Set initial pose if needed
        // For now, just a placeholder
        log::debug!("[FourSeasonsPlayer] Estimator initialized");
    }
}
