use crate::datasets::{
    config::Config, FrameContext, ImageData, ImuData, PlayerConfig, PlayerResult,
};
use crate::estimator::Estimator;
use crate::viewers::{create_viewer, Viewer};
use anyhow::{Context, Result};
use image::ImageReader;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;
use std::thread;
use std::time::{Duration, Instant};

pub struct TUMVIPlayer;

impl Default for TUMVIPlayer {
    fn default() -> Self {
        Self::new()
    }
}

impl TUMVIPlayer {
    pub const fn new() -> Self {
        TUMVIPlayer
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
                    log::info!("[TUMVIPlayer] Loaded {} IMU samples", data.len());
                }
                data
            },
            Err(e) => {
                log::warn!("[TUMVIPlayer] Failed to load IMU data: {}", e);
                Vec::new()
            },
        };

        let start_frame_idx = 0;
        let end_frame_idx = image_data.len();

        // Initialize viewer
        let mut viewer: Option<Box<dyn Viewer>> = match create_viewer() {
            Ok(v) => {
                log::info!("[TUMVIPlayer] Viewer initialized successfully");
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
                "[TUMVIPlayer] Average processing time: {:.2} ms ({:.1} fps)",
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

        log::info!("[TUMVIPlayer] Processing completed! Viewer remains open for inspection.");

        result
    }

    fn load_image_timestamps(dataset_path: &str) -> Result<Vec<ImageData>> {
        let data_file = Path::new(dataset_path).join("mav0/cam0/data.csv");
        let file = File::open(&data_file)
            .with_context(|| format!("Cannot open data.csv file: {}", data_file.display()))?;

        let reader = BufReader::new(file);
        let mut image_data = Vec::new();

        for (line_num, line) in reader.lines().enumerate() {
            let line = line?;

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

        if !full_path.exists() {
            anyhow::bail!("Cannot load image: {}", full_path.display());
        }

        // Load image using image crate
        let img = ImageReader::open(&full_path)
            .with_context(|| format!("Failed to open image: {}", full_path.display()))?
            .decode()
            .with_context(|| format!("Failed to decode image: {}", full_path.display()))?;

        // Convert to grayscale if needed (EuRoC images are typically grayscale)
        let gray_img = img.to_luma8();

        // Return raw pixel data as Vec<u8>
        let pixel_data = gray_img.as_raw().clone();

        Ok(pixel_data)
    }

    #[allow(dead_code)]
    fn load_imu_data(dataset_path: &str) -> Result<Vec<ImuData>> {
        let imu_file = Path::new(dataset_path).join("dso/imu.txt");

        if !imu_file.exists() {
            log::debug!(
                "[TUMVIPlayer] IMU file not found at {:?}, skipping",
                imu_file
            );
            return Ok(Vec::new());
        }

        let file = File::open(&imu_file)
            .with_context(|| format!("Cannot open IMU file: {}", imu_file.display()))?;

        let reader = BufReader::new(file);
        let mut imu_data = Vec::new();

        for (line_num, line) in reader.lines().enumerate() {
            let line = line?;

            // Skip header and empty lines
            if line_num == 0 || line.trim().is_empty() || line.trim_start().starts_with('#') {
                continue;
            }

            let parts: Vec<&str> = line.split_whitespace().collect();
            // Format: timestamp[ns] w.x w.y w.z a.x a.y a.z
            if parts.len() >= 7 {
                if let Ok(timestamp) = parts[0].parse::<i64>() {
                    if let (Ok(wx), Ok(wy), Ok(wz), Ok(ax), Ok(ay), Ok(az)) = (
                        parts[1].parse::<f64>(),
                        parts[2].parse::<f64>(),
                        parts[3].parse::<f64>(),
                        parts[4].parse::<f64>(),
                        parts[5].parse::<f64>(),
                        parts[6].parse::<f64>(),
                    ) {
                        imu_data.push(ImuData {
                            timestamp,
                            gyro: [wx, wy, wz],
                            accel: [ax, ay, az],
                        });
                    } else {
                        log::debug!(
                            "[TUMVIPlayer] Skipped malformed IMU line {}: invalid numeric values",
                            line_num
                        );
                    }
                } else {
                    log::debug!(
                        "[TUMVIPlayer] Skipped malformed IMU line {}: invalid timestamp",
                        line_num
                    );
                }
            } else {
                log::debug!("[TUMVIPlayer] Skipped malformed IMU line {}: insufficient fields (expected 7+, got {})", line_num, parts.len());
            }
        }

        log::info!(
            "[TUMVIPlayer] Loaded {} IMU samples from {}",
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
        log::debug!("[TUMVIPlayer] Estimator initialized");
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
        log::debug!("[TUMVIPlayer] Saving trajectories (placeholder)");
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
                "[TUMVIPlayer] Saved statistics to: {}",
                stats_file.display()
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datasets::test_utils::{sample_imu_data, timestamps, write_file};
    use tempfile::tempdir;

    #[test]
    fn load_imu_data_missing_file_returns_empty() {
        let dir = tempdir().expect("tempdir");
        let dataset_path = dir.path().to_str().expect("path utf-8");

        let imu_data = TUMVIPlayer::load_imu_data(dataset_path).expect("load imu data");
        assert!(imu_data.is_empty());
    }

    #[test]
    fn load_imu_data_skips_malformed_lines() {
        let dir = tempdir().expect("tempdir");
        let imu_path = dir.path().join("dso/imu.txt");

        write_file(
            &imu_path,
            "timestamp w.x w.y w.z a.x a.y a.z\n\
not_a_timestamp 0 0 0 0 0 0\n\
1 0 0 0 0 0\n\
2 0 0 0 0 0 bad\n\
3 0.1 0.2 0.3 1.0 1.1 1.2\n",
        );

        let dataset_path = dir.path().to_str().expect("path utf-8");
        let imu_data = TUMVIPlayer::load_imu_data(dataset_path).expect("load imu data");
        assert_eq!(imu_data.len(), 1);
        assert_eq!(imu_data[0].timestamp, 3);
    }

    #[test]
    fn get_imu_data_between_frames_boundaries() {
        let imu_data = sample_imu_data();

        let between = TUMVIPlayer::get_imu_data_between_frames(2, 4, &imu_data);
        assert_eq!(timestamps(&between), vec![3, 4]);

        let between = TUMVIPlayer::get_imu_data_between_frames(0, 1, &imu_data);
        assert_eq!(timestamps(&between), vec![1]);

        let between = TUMVIPlayer::get_imu_data_between_frames(3, 3, &imu_data);
        assert!(between.is_empty());
    }
}
