use crate::datasets::io::load_grayscale_image;
use crate::datasets::{
    config::Config, FrameContext, ImageData, ImuData, PlayerConfig, PlayerResult,
};
use crate::estimator::Estimator;
use crate::viewers::{create_viewer, Viewer};
use anyhow::{Context, Result};
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;
<<<<<<< HEAD
use std::thread;
use std::time::{Duration, Instant};

pub struct FourSeasonsPlayer;

impl Default for FourSeasonsPlayer {
    fn default() -> Self {
        Self::new()
=======
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
>>>>>>> 1e93ebd0 (Implement IMU data caching and retrieval)
    }
}

impl FourSeasonsPlayer {
    pub const fn new() -> Self {
        FourSeasonsPlayer
    }

    pub fn run(&self, config: PlayerConfig) -> PlayerResult {
        let mut result = PlayerResult::default();

        // Load image timestamps
        let image_data = match Self::load_image_timestamps(&config.dataset_path) {
            Ok(data) => {
                if data.is_empty() {
                    result.error_message = "No images found in dataset".to_string();
                    return result;
                }
                data
            },
            Err(e) => {
                result.error_message = format!("Failed to load image timestamps: {}", e);
                return result;
            },
        };

        // Load IMU data
        let imu_data = match Self::load_imu_data(&config.dataset_path) {
            Ok(data) => {
                if !data.is_empty() {
                    log::info!("[FourSeasonsPlayer] Loaded {} IMU samples", data.len());
                }
                data
            },
            Err(e) => {
                log::warn!("[FourSeasonsPlayer] Failed to load IMU data: {}", e);
                Vec::new()
            },
        };

        let start_frame_idx = 0;
        let end_frame_idx = image_data.len();

        // Initialize viewer
        let mut viewer: Option<Box<dyn Viewer>> = match create_viewer() {
            Ok(v) => {
                log::info!("[FourSeasonsPlayer] Viewer initialized successfully");
                Some(v)
            },
            Err(e) => {
                log::warn!("Failed to initialize viewer: {}", e);
                None
            },
        };

        // Load full YAML config
        let cfg = match Config::load(&config.config_path) {
            Ok(c) => c,
            Err(e) => {
                result.error_message =
                    format!("Failed to load config '{}': {}", config.config_path, e);
                return result;
            },
        };

        // Create camera models from config
        let (left_cam, right_cam) = match Self::create_camera_models_from_config(&cfg) {
            Ok(cams) => cams,
            Err(e) => {
                result.error_message = format!("Failed to create camera models: {}", e);
                return result;
            },
        };

        // Give ownership of the configuration to the estimator and pass a
        // reference to the viewer (which outlives the estimator).
        let mut estimator = {
            let viewer_ref: Option<&mut dyn Viewer> =
                viewer.as_deref_mut().map(|v| v as &mut dyn Viewer);
            Estimator::new_with_cameras(cfg, viewer_ref, Some(left_cam), Some(right_cam))
        };
        Self::initialize_estimator(&mut estimator, &image_data);

        // Process frames
        let mut context = FrameContext::new(config.step_mode);

        context.current_idx = start_frame_idx;
        while context.current_idx < end_frame_idx {
            let should_process_frame = if context.auto_play {
                // Auto mode: process frame
                true
            } else {
                // Step mode: only process if advance_frame is set
                if context.advance_frame {
                    context.advance_frame = false;
                    true
                } else {
                    // In step mode with no advance request, just wait
                    thread::sleep(Duration::from_millis(1));
                    continue;
                }
            };

            if should_process_frame {
                // Process single frame
                let frame_start = Instant::now();
                let _processing_time = match Self::process_single_frame(
                    &mut estimator,
                    &mut context,
                    &image_data,
                    &config.dataset_path,
                    &imu_data,
                ) {
                    Ok(time) => time,
                    Err(e) => {
                        log::warn!("Error processing frame {}: {}", context.current_idx, e);
                        0.0
                    },
                };

                let frame_duration = frame_start.elapsed();
                let total_time_ms = frame_duration.as_secs_f64() * 1000.0;
                result.frame_processing_times.push(total_time_ms);

                context.current_idx += 1;
                context.processed_frames += 1;

                // Calculate sleep time based on actual frame intervals (only in auto mode)
                if context.auto_play && context.current_idx < end_frame_idx {
                    let current_timestamp = image_data[context.current_idx - 1].timestamp;
                    let next_timestamp = image_data[context.current_idx].timestamp;
                    let frame_interval_ms = (next_timestamp - current_timestamp) as f64 / 1e6; // nanoseconds to milliseconds

                    let sleep_time_ms = (frame_interval_ms - total_time_ms).max(0.0);
                    if sleep_time_ms > 0.0 {
                        thread::sleep(Duration::from_millis(sleep_time_ms as u64));
                    }
                }
            }
        }

        // Save results
        if config.enable_statistics {
            Self::save_trajectories(&estimator, &context, &config.dataset_path);
            Self::save_statistics(&result, &config.dataset_path);
        }

        // Calculate final statistics
        result.success = true;
        result.processed_frames = context.processed_frames;

        if !result.frame_processing_times.is_empty() {
            result.average_processing_time_ms = result.frame_processing_times.iter().sum::<f64>()
                / result.frame_processing_times.len() as f64;

            log::info!(
                "[4SeasonsPlayer] Average processing time: {:.2} ms ({:.1} fps)",
                result.average_processing_time_ms,
                1000.0 / result.average_processing_time_ms
            );
        }

        // Display final statistics summary
        if config.enable_console_statistics && result.success {
            log::info!("════════════════════════════════════════════════════════════════════");
            log::info!("                          STATISTICS                                ");
            log::info!("════════════════════════════════════════════════════════════════════");
            log::info!("");
            log::info!("                          TIMING ANALYSIS                           ");
            log::info!("════════════════════════════════════════════════════════════════════");
            log::info!(" Total Frames Processed: {}", result.processed_frames);
            log::info!(
                " Average Processing Time: {:.2}ms",
                result.average_processing_time_ms
            );
            let fps = 1000.0 / result.average_processing_time_ms;
            log::info!(" Average Frame Rate: {:.1}fps", fps);
            log::info!("════════════════════════════════════════════════════════════════════");
        }

        log::info!("[4SeasonsPlayer] Processing completed! Viewer remains open for inspection.");

        result
    }

    fn load_image_timestamps(dataset_path: &str) -> Result<Vec<ImageData>> {
        let data_file = Path::new(dataset_path).join("times.txt");
        let file = File::open(&data_file)
            .with_context(|| format!("Cannot open times.txt file: {}", data_file.display()))?;

        let reader = BufReader::new(file);
        let mut image_data = Vec::new();

        for (line_num, line) in reader.lines().enumerate() {
            let line = line?;

            // Skip header and empty lines
            if line_num == 0 || line.trim().is_empty() || line.trim_start().starts_with('#') {
                continue;
            }

            // Format: timestamp filename timestamp (space-separated)
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 2 {
                let timestamp_str = parts[0].trim();
                let filename = parts[0].trim().to_string() + ".png";
                if let Ok(timestamp) = timestamp_str.parse::<i64>() {
                    image_data.push(ImageData {
                        timestamp,
                        filename,
                    });
                }
            }
        }

        log::info!(
            "[4SeasonsPlayer] Loaded {} image timestamps",
            image_data.len()
        );
        Ok(image_data)
    }

    fn load_image(dataset_path: &str, filename: &str, cam_id: u32) -> Result<Vec<u8>> {
        let cam_folder = if cam_id == 0 { "cam0" } else { "cam1" };
        let full_path = Path::new(dataset_path)
            .join("undistorted_images")
            .join(cam_folder)
            .join(filename);
        load_grayscale_image(&full_path)
    }

    fn load_imu_data(dataset_path: &str) -> Result<Vec<ImuData>> {
        // 4Seasons dataset may have IMU data in imu.txt or imu.csv
        // Special handling: supports both comma and space-separated formats
        let mut imu_file = Path::new(dataset_path).join("imu.txt");

        if !imu_file.exists() {
<<<<<<< HEAD
            let imu_csv = Path::new(dataset_path).join("imu.csv");
            if imu_csv.exists() {
                log::debug!(
                    "[FourSeasonsPlayer] IMU file imu.txt not found, falling back to {:?}",
                    imu_csv
                );
                imu_file = imu_csv;
            } else {
                log::debug!(
                    "[FourSeasonsPlayer] IMU file not found (tried {:?} and {:?}), skipping",
                    imu_file,
                    imu_csv
                );
                return Ok(Vec::new());
            }
=======
            log::info!(
                "[FourSeasonsPlayer] No IMU file found at {}",
                imu_file.display()
            );
            // Clear cache when file doesn't exist
            *self.imu_cache.lock().unwrap() = Vec::new();
            return Ok(());
>>>>>>> 1e93ebd0 (Implement IMU data caching and retrieval)
        }

        let file = File::open(&imu_file)
            .with_context(|| format!("Cannot open IMU file: {}", imu_file.display()))?;

        let reader = BufReader::new(file);
<<<<<<< HEAD
        let mut imu_data = Vec::new();
=======
        let mut imu_data_vec = Vec::new();
>>>>>>> 1e93ebd0 (Implement IMU data caching and retrieval)

        for (line_num, line) in reader.lines().enumerate() {
            let line = line?;

            // Skip header and empty lines
            if line_num == 0 || line.trim().is_empty() || line.trim_start().starts_with('#') {
                continue;
            }

<<<<<<< HEAD
            // Try to parse both space-separated and comma-separated formats
            let parts: Vec<&str> = if line.contains(',') {
                line.split(',').map(|s| s.trim()).collect()
            } else {
                line.split_whitespace().collect()
            };

            // Format: timestamp[ns] w.x w.y w.z a.x a.y a.z
            if parts.len() >= 7 {
                if let Ok(timestamp) = parts[0].trim().parse::<i64>() {
                    if let (Ok(wx), Ok(wy), Ok(wz), Ok(ax), Ok(ay), Ok(az)) = (
                        parts[1].trim().parse::<f64>(),
                        parts[2].trim().parse::<f64>(),
                        parts[3].trim().parse::<f64>(),
                        parts[4].trim().parse::<f64>(),
                        parts[5].trim().parse::<f64>(),
                        parts[6].trim().parse::<f64>(),
                    ) {
                        // Validate ranges
                        let gyro_magnitude = (wx * wx + wy * wy + wz * wz).sqrt();
                        let accel_magnitude = (ax * ax + ay * ay + az * az).sqrt();

                        if gyro_magnitude <= 1000.0 && accel_magnitude <= 200.0 {
                            imu_data.push(ImuData {
                                timestamp,
                                gyro: [wx, wy, wz],
                                accel: [ax, ay, az],
                            });
                        } else {
                            log::debug!(
                                "[FourSeasonsPlayer] Skipped malformed IMU line {}: out-of-range values (gyro={:.1}, accel={:.1})",
                                line_num, gyro_magnitude, accel_magnitude
                            );
                        }
                    } else {
                        log::debug!("[FourSeasonsPlayer] Skipped malformed IMU line {}: invalid numeric values", line_num);
=======
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
        *self.imu_cache.lock().unwrap() = imu_data_vec;

        log::info!(
            "[FourSeasonsPlayer] Loaded {} IMU samples",
            self.imu_cache.lock().unwrap().len()
        );
        Ok(())
    }

    fn get_imu_data_between_frames(
        &self,
        previous_timestamp: i64,
        current_timestamp: i64,
    ) -> Vec<ImuData> {
        let cache = self.imu_cache.lock().unwrap();
        cache
            .iter()
            .filter(|imu| imu.timestamp > previous_timestamp && imu.timestamp <= current_timestamp)
            .cloned()
            .collect()
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

    fn save_trajectories(&self, estimator: &Estimator, context: &FrameContext, dataset_path: &str) {
        let trajectory_path = Path::new(dataset_path).join("trajectory.txt");

        match std::fs::File::create(&trajectory_path) {
            Ok(mut file) => {
                use std::io::Write;
                let trajectory = estimator.get_trajectory();
                let mut count = 0;

                for pose in trajectory.iter() {
                    let timestamp_s = context.previous_frame_timestamp as f64 / 1e9;

                    // Extract translation
                    let tx = pose[(0, 3)] as f64;
                    let ty = pose[(1, 3)] as f64;
                    let tz = pose[(2, 3)] as f64;

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
>>>>>>> 1e93ebd0 (Implement IMU data caching and retrieval)
                    }
                } else {
                    log::debug!(
                        "[FourSeasonsPlayer] Skipped malformed IMU line {}: invalid timestamp",
                        line_num
                    );
                }
            } else {
                log::debug!("[FourSeasonsPlayer] Skipped malformed IMU line {}: insufficient fields (expected 7+, got {})", line_num, parts.len());
            }
        }

        log::info!(
            "[FourSeasonsPlayer] Loaded {} IMU samples from {}",
            imu_data.len(),
            imu_file.display()
        );
        Ok(imu_data)
    }

    /// Create camera models from config using the datasets module helper function
    fn create_camera_models_from_config(
        config: &Config,
    ) -> Result<(
        crate::datasets::CameraModelType,
        crate::datasets::CameraModelType,
    )> {
        Ok(crate::datasets::create_camera_models_from_config(config))
    }

    fn initialize_estimator<'a>(_estimator: &mut Estimator<'a>, _image_data: &[ImageData]) {
        // TODO: Set initial pose if needed
        // For now, just a placeholder
        log::debug!("[4SeasonsPlayer] Estimator initialized");
    }

    fn process_single_frame<'a>(
        estimator: &mut Estimator<'a>,
        context: &mut FrameContext,
        image_data: &[ImageData],
        dataset_path: &str,
        imu_data: &[ImuData],
    ) -> Result<f64> {
        let frame_start = Instant::now();

        // Inform the estimator about the current frame index for visualization.
        estimator.set_viewer_frame(context.current_idx as i64);

        // Load stereo images
        let left_image =
            Self::load_image(dataset_path, &image_data[context.current_idx].filename, 0)?;
        let right_image =
            Self::load_image(dataset_path, &image_data[context.current_idx].filename, 1)?;

        if left_image.is_empty() {
            anyhow::bail!("Skipping frame {} due to empty image", context.current_idx);
        }

        // Get IMU data between previous and current frame
        let imu_between_frames = if !imu_data.is_empty() {
            Self::get_imu_data_between_frames(
                context.previous_frame_timestamp,
                image_data[context.current_idx].timestamp,
                imu_data,
            )
        } else {
            Vec::new()
        };

        // Process frame
        let imu_slice = if imu_between_frames.is_empty() {
            None
        } else {
            Some(imu_between_frames.as_slice())
        };
        estimator.process_frame(
            &left_image,
            &right_image,
            image_data[context.current_idx].timestamp,
            imu_slice,
        )?;

        // Update frame timestamp
        context.previous_frame_timestamp = image_data[context.current_idx].timestamp;

        let frame_duration = frame_start.elapsed();
        Ok(frame_duration.as_secs_f64() * 1000.0) // Return milliseconds
    }

    fn get_imu_data_between_frames(
        previous_timestamp: i64,
        current_timestamp: i64,
        all_imu_data: &[ImuData],
    ) -> Vec<ImuData> {
        // Use binary search for efficiency with large datasets
        // Find start index: first IMU sample > previous_timestamp
        let start_idx = all_imu_data.partition_point(|imu| imu.timestamp <= previous_timestamp);

        // Find end index: last IMU sample <= current_timestamp
        let end_idx = all_imu_data.partition_point(|imu| imu.timestamp <= current_timestamp);

        // Collect IMU samples in the range
        if start_idx < end_idx {
            all_imu_data[start_idx..end_idx].to_vec()
        } else {
            Vec::new()
        }
    }

    fn save_trajectories(_estimator: &Estimator, _context: &FrameContext, _dataset_path: &str) {
        // TODO: Implement trajectory saving
        log::debug!("[4SeasonsPlayer] Saving trajectories (placeholder)");
    }

    fn save_statistics(result: &PlayerResult, dataset_path: &str) {
        let stats_file = Path::new(dataset_path).join("statistics.txt");

        if let Ok(mut file) = std::fs::File::create(&stats_file) {
            use std::io::Write;
            writeln!(
                file,
                "════════════════════════════════════════════════════════════════════"
            )
            .ok();
            writeln!(
                file,
                "                          STATISTICS                                "
            )
            .ok();
            writeln!(
                file,
                "════════════════════════════════════════════════════════════════════"
            )
            .ok();
            writeln!(file).ok();

            // Timing statistics
            writeln!(
                file,
                "                          TIMING ANALYSIS                           "
            )
            .ok();
            writeln!(
                file,
                "════════════════════════════════════════════════════════════════════"
            )
            .ok();
            writeln!(file, " Total Frames Processed: {}", result.processed_frames).ok();
            writeln!(
                file,
                " Average Processing Time: {:.2}ms",
                result.average_processing_time_ms
            )
            .ok();
            let fps = 1000.0 / result.average_processing_time_ms;
            writeln!(file, " Average Frame Rate: {:.1}fps", fps).ok();
            writeln!(
                file,
                "════════════════════════════════════════════════════════════════════"
            )
            .ok();

            log::info!(
                "[4SeasonsPlayer] Saved statistics to: {}",
                stats_file.display()
            );
        }
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

        let imu_data = FourSeasonsPlayer::load_imu_data(dataset_path).expect("load imu data");
        assert!(imu_data.is_empty());
    }

    #[test]
    fn load_imu_data_skips_malformed_lines() {
        let dir = tempdir().expect("tempdir");
        let imu_path = dir.path().join("imu.txt");

        write_file(
            &imu_path,
            "timestamp w.x w.y w.z a.x a.y a.z\n\
not_a_timestamp 0 0 0 0 0 0\n\
1 0 0 0 0 0\n\
2,0,0,0,0,0,bad\n\
3,0.1,0.2,0.3,1.0,1.1,1.2\n\
4 0.1 0.2 0.3 1.0 1.1 1.2\n",
        );

        let dataset_path = dir.path().to_str().expect("path utf-8");
        let imu_data = FourSeasonsPlayer::load_imu_data(dataset_path).expect("load imu data");
        assert_eq!(timestamps(&imu_data), vec![3, 4]);
    }

    #[test]
    fn get_imu_data_between_frames_boundaries() {
        let imu_data = sample_imu_data();

        let between = FourSeasonsPlayer::get_imu_data_between_frames(2, 4, &imu_data);
        assert_eq!(timestamps(&between), vec![3, 4]);

        let between = FourSeasonsPlayer::get_imu_data_between_frames(0, 1, &imu_data);
        assert_eq!(timestamps(&between), vec![1]);

        let between = FourSeasonsPlayer::get_imu_data_between_frames(3, 3, &imu_data);
        assert!(between.is_empty());
=======
    fn initialize_estimator(&self, _estimator: &mut Estimator, _image_data: &[ImageData]) {
        // FourSeasons starts at identity pose - estimator already initialized with identity
        log::debug!("[FourSeasonsPlayer] Estimator initialized with identity pose");
>>>>>>> 99a3ba3a (Implement TODO comments: IMU loading, trajectory saving, and initial pose)
    }
}
