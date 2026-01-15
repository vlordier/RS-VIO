use crate::datasets::{
    config::Config,
    player_trait::{self, DatasetPlayer},
    FrameContext, ImageData, ImuData, PlayerConfig, PlayerResult,
};
use crate::estimator::Estimator;
use crate::{Result, VIOError};
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;
use std::sync::Mutex;

#[derive(Default)]
pub struct EurocPlayer {
    imu_cache: Mutex<Vec<ImuData>>,
}

impl EurocPlayer {
    pub fn new() -> Self {
        EurocPlayer {
            imu_cache: Mutex::new(Vec::new()),
        }
    }
}

impl DatasetPlayer for EurocPlayer {
    fn run(&self, config: PlayerConfig) -> crate::Result<PlayerResult> {
        player_trait::execute(self, config, "EurocPlayer")
    }

    fn load_image_timestamps(&self, dataset_path: &str) -> Result<Vec<ImageData>> {
        player_trait::load_timestamps_from_csv(dataset_path, "EurocPlayer")
    }

    fn load_image(&self, dataset_path: &str, filename: &str, cam_id: u32) -> Result<Vec<u8>> {
        player_trait::load_image_from_mav0(dataset_path, filename, cam_id)
    }

    fn load_imu_data(
        &self,
        dataset_path: &str,
        _image_data: &[ImageData],
        _start_frame_idx: usize,
        _end_frame_idx: usize,
    ) -> Result<()> {
        let imu_file = Path::new(dataset_path).join("mav0/imu0/data.csv");
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

            // Skip header line
            if line_num == 0 && line.contains("#timestamp") {
                continue;
            }

            // EuRoC IMU CSV format: timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z
            let parts: Vec<&str> = line.split(',').collect();
            if parts.len() < 7 {
                continue;
            }

            // Parse timestamp (nanoseconds)
            let timestamp: i64 = parts[0].trim().parse().unwrap_or(0);

            // Parse gyroscope (rad/s)
            let gyro_x: f64 = parts[1].trim().parse().unwrap_or(0.0);
            let gyro_y: f64 = parts[2].trim().parse().unwrap_or(0.0);
            let gyro_z: f64 = parts[3].trim().parse().unwrap_or(0.0);

            // Parse accelerometer (m/s^2)
            let accel_x: f64 = parts[4].trim().parse().unwrap_or(0.0);
            let accel_y: f64 = parts[5].trim().parse().unwrap_or(0.0);
            let accel_z: f64 = parts[6].trim().parse().unwrap_or(0.0);

            imu_data_vec.push(ImuData {
                timestamp,
                gyro: [gyro_x, gyro_y, gyro_z],
                accel: [accel_x, accel_y, accel_z],
            });
        }

        // Store in cache
        if let Ok(mut cache) = self.imu_cache.lock() {
            *cache = imu_data_vec;
            log::info!("[EurocPlayer] Loaded {} IMU samples", cache.len());
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
            .unwrap_or_default()
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

    fn save_trajectories(
        &self,
        estimator: &Estimator,
        _context: &FrameContext,
        dataset_path: &str,
    ) {
        // Save trajectory in TUM format: timestamp x y z qx qy qz qw
        let trajectory_path = Path::new(dataset_path).join("trajectory.txt");

        match std::fs::File::create(&trajectory_path) {
            Ok(mut file) => {
                use std::io::Write;
                let trajectory = estimator.get_trajectory();
                let mut count = 0;

                for (timestamp_ns, pose) in trajectory.iter() {
                    // Extract translation
                    let tx = pose[(0, 3)];
                    let ty = pose[(1, 3)];
                    let tz = pose[(2, 3)];

                    // Extract rotation as quaternion
                    let r = pose.fixed_view::<3, 3>(0, 0);
                    let rotmat = nalgebra::Rotation3::from_matrix_unchecked(r.into_owned());
                    let q = nalgebra::UnitQuaternion::from_rotation_matrix(&rotmat);

                    let timestamp_s = *timestamp_ns as f64 / 1e9;

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
                    "[EurocPlayer] Saved trajectory with {} poses to {}",
                    count,
                    trajectory_path.display()
                );
            },
            Err(e) => {
                log::error!(
                    "[EurocPlayer] Failed to create trajectory file {}: {e}",
                    trajectory_path.display()
                );
            },
        }
    }

    fn save_statistics(&self, result: &PlayerResult, stats_path: &Path) {
        crate::datasets::player_trait::save_statistics_common(result, stats_path);
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
        // EuroC starts at identity pose - estimator already initialized with identity
        // For datasets with ground truth initial poses, this could be extended to load them
        log::debug!("[EurocPlayer] Estimator initialized with identity pose");
    }
}
