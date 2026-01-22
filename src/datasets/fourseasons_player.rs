use crate::datasets::{
    config::Config, player_trait::DatasetPlayer, FrameContext, ImageData, ImuData, PlayerConfig,
    PlayerResult,
};
use crate::debug_log;
use crate::estimator::Estimator;
use crate::{Result, VIOError, ok_or_log};
use image::ImageReader;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;
use std::sync::Mutex;

#[derive(Default)]
pub struct FourSeasonsPlayer {
    imu_cache: Mutex<Vec<ImuData>>,
}

impl FourSeasonsPlayer {
    pub fn new() -> Self {
        FourSeasonsPlayer {
            imu_cache: Mutex::new(Vec::new()),
        }
    }
}

impl DatasetPlayer for FourSeasonsPlayer {
    fn run(&self, config: &PlayerConfig) -> crate::Result<PlayerResult> {
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
        let pixel_data = gray_img.as_raw().clone();

        Ok(pixel_data)
    }

    fn load_imu_data(
        &self,
        dataset_path: &str,
        _image_data: &[ImageData],
        _start_frame_idx: usize,
        _end_frame_idx: usize,
    ) -> Result<()> {
        let imu_file = Path::new(dataset_path).join("imu.txt");
        if !imu_file.exists() {
            log::info!(
                "[FourSeasonsPlayer] No IMU file found at {}",
                imu_file.display()
            );
            // Clear cache when file doesn't exist
            if let Ok(mut cache) = self.imu_cache.lock() {
                *cache = Vec::new();
            }
            return Ok(());
        }

        let file = File::open(&imu_file).map_err(|e| {
            VIOError::Config(format!(
                "Cannot open IMU data file {}: {e}",
                imu_file.display()
            ))
        })?;

        let reader = BufReader::new(file);
        let mut imu_data_vec = Vec::new();

        for (line_num, line) in reader.lines().enumerate() {
            let line = line.map_err(|e| {
                VIOError::Config(format!(
                    "Failed to read IMU data line {}: {e}",
                    line_num + 1
                ))
            })?;

            // 4Seasons IMU format: timestamp_w_nanoseconds omega_x omega_y omega_z alpha_x alpha_y alpha_z
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 7 {
                continue;
            }

            // Parse timestamp (nanoseconds)
            let timestamp: i64 = ok_or_log!(
                parts[0].trim().parse(),
                0,
                "Failed to parse FourSeasons IMU timestamp on line {}",
                line_num + 1
            );

            // Parse gyroscope (rad/s)
            let gyro_x: f64 = ok_or_log!(
                parts[1].trim().parse(),
                0.0,
                "Failed to parse FourSeasons gyro_x on line {}",
                line_num + 1
            );
            let gyro_y: f64 = ok_or_log!(
                parts[2].trim().parse(),
                0.0,
                "Failed to parse FourSeasons gyro_y on line {}",
                line_num + 1
            );
            let gyro_z: f64 = ok_or_log!(
                parts[3].trim().parse(),
                0.0,
                "Failed to parse FourSeasons gyro_z on line {}",
                line_num + 1
            );

            // Parse accelerometer (m/s^2)
            let accel_x: f64 = ok_or_log!(
                parts[4].trim().parse(),
                0.0,
                "Failed to parse FourSeasons accel_x on line {}",
                line_num + 1
            );
            let accel_y: f64 = ok_or_log!(
                parts[5].trim().parse(),
                0.0,
                "Failed to parse FourSeasons accel_y on line {}",
                line_num + 1
            );
            let accel_z: f64 = ok_or_log!(
                parts[6].trim().parse(),
                0.0,
                "Failed to parse FourSeasons accel_z on line {}",
                line_num + 1
            );

            imu_data_vec.push(ImuData {
                timestamp,
                gyro: [gyro_x, gyro_y, gyro_z],
                accel: [accel_x, accel_y, accel_z],
            });
        }

        // Store in cache
        if let Ok(mut cache) = self.imu_cache.lock() {
            *cache = imu_data_vec;
            log::info!("[FourSeasonsPlayer] Loaded {} IMU samples", cache.len());
        }
        Ok(())
    }

    fn get_imu_data_between_frames(
        &self,
        previous_timestamp: i64,
        current_timestamp: i64,
    ) -> Vec<ImuData> {
        self.imu_cache
            .lock()
            .map(|cache| {
                cache
                    .iter()
                    .filter(|imu| {
                        imu.timestamp > previous_timestamp && imu.timestamp <= current_timestamp
                    })
                    .cloned()
                    .collect()
            })
            .unwrap_or_else(|e| {
                log::warn!(
                    "FourSeasons IMU cache poisoned while fetching between frames: {}",
                    e
                );
                Vec::new()
            })
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
            |prev_ts, curr_ts| self.get_imu_data_between_frames(prev_ts, curr_ts),
        )
    }

    fn save_statistics(&self, result: &PlayerResult, stats_path: &Path) {
        crate::datasets::player_trait::save_statistics_common(result, stats_path);
    }

    fn save_trajectories(
        &self,
        estimator: &Estimator,
        _context: &FrameContext,
        dataset_path: &str,
    ) {
        let trajectory_path = Path::new(dataset_path).join("trajectory.txt");

        match std::fs::File::create(&trajectory_path) {
            Ok(mut file) => {
                use std::io::Write;
                let trajectory = estimator.get_trajectory();
                let mut count = 0;

                for (timestamp_ns, pose) in trajectory.iter() {
                    let timestamp_s = *timestamp_ns as f64 / 1e9;

                    // Extract translation
                    let tx = pose[(0, 3)];
                    let ty = pose[(1, 3)];
                    let tz = pose[(2, 3)];

                    // Extract rotation as quaternion
                    let r = pose.fixed_view::<3, 3>(0, 0);
                    let rotmat = nalgebra::Rotation3::from_matrix_unchecked(r.into_owned());
                    let q = nalgebra::UnitQuaternion::from_rotation_matrix(&rotmat);

                    if writeln!(
                        file,
                        "{:.9} {:.6} {:.6} {:.6} {:.9} {:.9} {:.9} {:.9}",
                        timestamp_s, tx, ty, tz, q.i, q.j, q.k, q.w
                    )
                    .is_ok()
                    {
                        count += 1;
                    }
                }

                log::info!(
                    "[FourSeasonsPlayer] Saved trajectory with {} poses to {}",
                    count,
                    trajectory_path.display()
                );
            },
            Err(e) => {
                log::error!(
                    "[FourSeasonsPlayer] Failed to create trajectory file {}: {e}",
                    trajectory_path.display()
                );
            },
        }
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
        // FourSeasons starts at identity pose - estimator already initialized with identity
        debug_log!("[FourSeasonsPlayer] Estimator initialized with identity pose");
    }
}
