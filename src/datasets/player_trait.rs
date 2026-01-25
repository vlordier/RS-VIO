//! Trait-oriented abstraction for dataset players
//!
//! This module provides a common trait interface for different dataset players,
//! enabling polymorphic behavior and composition-based design patterns.

use crate::datasets::{FrameContext, ImageData, ImuData, PlayerConfig, PlayerResult};
use crate::estimator::Estimator;
use crate::viewers::Viewer;
use crate::{Result, VIOError};
use std::path::{Path, PathBuf};

/// Common interface for dataset players
///
/// This trait defines the contract that all dataset players must implement,
/// allowing for polymorphic behavior across different dataset formats (EuRoC, TUM-VI, 4Seasons).
///
/// # Design Pattern
/// This trait implements the **Strategy Pattern**, allowing different dataset loading
/// strategies to be used interchangeably. It also enables **Trait Objects** for runtime
/// polymorphism, where the specific player type doesn't need to be known at compile time.
pub trait DatasetPlayer: Send + Sync {
    /// Run the dataset player with the given configuration
    ///
    /// This is the main entry point that orchestrates the entire processing pipeline:
    /// - Loading image timestamps
    /// - Initializing the estimator
    /// - Processing frames iteratively
    /// - Collecting statistics
    fn run(&self, config: &PlayerConfig) -> crate::Result<PlayerResult>;

    /// Load image timestamps from the dataset
    ///
    /// Each dataset has a specific format for storing metadata about images.
    /// This method abstracts away the dataset-specific CSV/file format differences.
    ///
    /// # Arguments
    /// * `dataset_path` - Path to the dataset root directory
    ///
    /// # Returns
    /// A vector of `ImageData` containing timestamps and filenames
    fn load_image_timestamps(&self, dataset_path: &str) -> Result<Vec<ImageData>>;

    /// Load a single image from the dataset
    ///
    /// Different datasets store images in different directory structures.
    /// This method encapsulates dataset-specific path resolution.
    ///
    /// # Arguments
    /// * `dataset_path` - Path to the dataset root directory
    /// * `filename` - Relative path to the image file
    /// * `cam_id` - Camera ID (0 for left, 1 for right)
    ///
    /// # Returns
    /// Raw grayscale pixel data as a `Vec<u8>`
    fn load_image(&self, dataset_path: &str, filename: &str, cam_id: u32) -> Result<Vec<u8>>;

    /// Load IMU data for a frame interval
    ///
    /// IMU data synchronization is dataset-specific. This method provides
    /// a common interface for accessing synchronized IMU measurements.
    ///
    /// # Arguments
    /// * `dataset_path` - Path to the dataset root directory
    /// * `image_data` - Reference to all image metadata
    /// * `start_frame_idx` - Index of the first frame
    /// * `end_frame_idx` - Index of the last frame (exclusive)
    ///
    /// # Returns
    /// A `Result` indicating success or failure of IMU data loading
    fn load_imu_data(
        &self,
        dataset_path: &str,
        image_data: &[ImageData],
        start_frame_idx: usize,
        end_frame_idx: usize,
    ) -> Result<()>;

    /// Get IMU data between two frame timestamps
    ///
    /// This retrieves synchronized IMU measurements that occurred between
    /// two image frame timestamps.
    ///
    /// # Arguments
    /// * `previous_timestamp` - Timestamp of the previous frame (nanoseconds)
    /// * `current_timestamp` - Timestamp of the current frame (nanoseconds)
    ///
    /// # Returns
    /// A vector of `ImuData` entries between the two timestamps
    fn get_imu_data_between_frames(
        &self,
        previous_timestamp: i64,
        current_timestamp: i64,
    ) -> Vec<ImuData>;

    /// Process a single frame through the VIO pipeline
    ///
    /// This is where the actual visual processing happens. The frame is passed
    /// to the estimator for feature detection, tracking, and pose estimation.
    ///
    /// # Arguments
    /// * `estimator` - Mutable reference to the estimator
    /// * `context` - Mutable frame context tracking state
    /// * `image_data` - Reference to image metadata
    /// * `dataset_path` - Path to the dataset root directory
    ///
    /// # Returns
    /// Processing time in milliseconds
    fn process_single_frame(
        &self,
        estimator: &mut Estimator,
        context: &mut FrameContext,
        image_data: &[ImageData],
        dataset_path: &str,
    ) -> Result<f64>;

    /// Save trajectory and pose estimates to disk
    ///
    /// After processing, results need to be persisted. Dataset-specific
    /// format requirements are handled by the implementing type.
    ///
    /// # Arguments
    /// * `estimator` - Reference to the estimator with computed results
    /// * `context` - Frame context with processing statistics
    /// * `dataset_path` - Output directory for results
    fn save_trajectories(&self, estimator: &Estimator, context: &FrameContext, dataset_path: &str);

    /// Save processing statistics to disk
    ///
    /// Performance metrics and diagnostics are written to a statistics file
    /// in a dataset-specific format.
    ///
    /// # Arguments
    /// * `result` - Processing result with timing statistics
    /// * `dataset_path` - Output directory for statistics
    fn save_statistics(&self, result: &PlayerResult, stats_path: &Path);

    /// Create camera models from configuration
    ///
    /// Camera calibration models depend on the intrinsic/distortion models
    /// specified in the configuration. This method instantiates the appropriate
    /// camera model for this dataset.
    ///
    /// # Arguments
    /// * `config` - Configuration containing camera parameters
    ///
    /// # Returns
    /// A tuple of (left_camera, right_camera) as `CameraModelType` enums
    fn create_camera_models_from_config(
        &self,
        config: &crate::datasets::config::Config,
    ) -> Result<(
        crate::datasets::CameraModelType,
        crate::datasets::CameraModelType,
    )>;

    /// Initialize the estimator with dataset-specific defaults
    ///
    /// Some datasets require special initialization (e.g., setting initial pose,
    /// configuring feature detectors). This method allows customization per dataset.
    ///
    /// # Arguments
    /// * `estimator` - Mutable reference to the estimator
    /// * `image_data` - Reference to image metadata (for statistics)
    fn initialize_estimator(&self, estimator: &mut Estimator, image_data: &[ImageData]);
}

/// Execute the standard dataset processing loop
///
/// This is a generic implementation that can be reused by all dataset players.
/// It factors out the common control flow logic while allowing customization
/// through the trait methods.
/// This function encapsulates the repeated pattern of:
/// 1. Loading images and IMU data
/// 2. Creating the estimator
/// 3. Processing frames in a loop
/// 4. Handling auto-play vs step mode
/// 5. Collecting statistics
///
/// By extracting this pattern, we avoid code duplication across player implementations.
pub fn execute(
    player: &dyn DatasetPlayer,
    config: &PlayerConfig,
    dataset_name: &str,
) -> Result<PlayerResult> {
    let mut result = PlayerResult::default();

    // Load image timestamps
    let image_data = player.load_image_timestamps(&config.dataset_path)?;

    if image_data.is_empty() {
        return Err(VIOError::Config("No images found in dataset".to_string()));
    }

    let start_frame_idx = 0;
    let end_frame_idx = image_data.len();

    // Load IMU data for the full dataset (cached for efficient retrieval)
    if let Err(e) = player.load_imu_data(
        &config.dataset_path,
        &image_data,
        start_frame_idx,
        end_frame_idx,
    ) {
        log::warn!("[{}] Failed to load IMU data: {}", dataset_name, e);
    }

    // Load full YAML config
    let cfg = crate::datasets::config::Config::load(&config.config_path)?;

    // Clone visualization config to avoid moving cfg
    let visualization = cfg.visualization.clone();

    // Initialize viewer according to visualization config
    let viewer: Option<Box<dyn Viewer>> = if visualization.enable_viewer {
        match crate::viewers::create_viewer(&visualization) {
            Ok(v) => {
                log::info!("[{}] Viewer initialized successfully", dataset_name);
                Some(v)
            },
            Err(e) => {
                log::warn!("Failed to initialize viewer: {}", e);
                None
            },
        }
    } else {
        None
    };

    // Create camera models from config
    let (left_cam, right_cam) = player.create_camera_models_from_config(&cfg)?;

    // Create estimator with viewer and cameras
    let mut estimator = Estimator::new_with_cameras(cfg, viewer, Some(left_cam), Some(right_cam));
    player.initialize_estimator(&mut estimator, &image_data);

    // Process frames
    let mut context = FrameContext::new(config.step_mode);
    context.current_idx = start_frame_idx;

    while context.current_idx < end_frame_idx {
        let should_process_frame = if context.auto_play {
            true
        } else if context.advance_frame {
            context.advance_frame = false;
            true
        } else {
            std::thread::sleep(std::time::Duration::from_millis(30));
            continue;
        };

        if should_process_frame {
            let frame_start = std::time::Instant::now();
            let _processing_time = match player.process_single_frame(
                &mut estimator,
                &mut context,
                &image_data,
                &config.dataset_path,
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

            // Calculate sleep time based on frame intervals (only in auto mode)
            if context.auto_play && context.current_idx < end_frame_idx {
                let current_timestamp = image_data[context.current_idx - 1].timestamp;
                let next_timestamp = image_data[context.current_idx].timestamp;
                let frame_interval_ms = (next_timestamp - current_timestamp) as f64 / 1e6; // nanoseconds to milliseconds

                let sleep_time_ms = (frame_interval_ms - total_time_ms).max(0.0);
                if sleep_time_ms > 0.0 {
                    std::thread::sleep(std::time::Duration::from_millis(sleep_time_ms as u64));
                }
            }
        }
    }

    // Save results
    if config.enable_statistics {
        player.save_trajectories(&estimator, &context, &config.dataset_path);
        let stats_path: PathBuf = if let Some(path) = &config.stats_output_path {
            PathBuf::from(path)
        } else {
            Path::new(&config.dataset_path).join("statistics.txt")
        };
        player.save_statistics(&result, &stats_path);
    }

    // Calculate final statistics
    result.success = true;
    result.processed_frames = context.processed_frames;

    if !result.frame_processing_times.is_empty() {
        result.average_processing_time_ms = result.frame_processing_times.iter().sum::<f64>()
            / result.frame_processing_times.len() as f64;

        log::info!(
            "[{}] Average processing time: {:.2} ms ({:.1} fps)",
            dataset_name,
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

    log::info!(
        "[{}] Processing completed!{}",
        dataset_name,
        if visualization.enable_viewer {
            " Viewer remains open for inspection."
        } else {
            ""
        }
    );

    Ok(result)
}

/// Common implementation of process_single_frame
///
/// This function handles the shared logic for processing a single frame,
/// allowing dataset players to focus on dataset-specific loading.
pub fn process_single_frame_common(
    estimator: &mut Estimator,
    context: &mut FrameContext,
    image_data: &[ImageData],
    dataset_path: &str,
    load_left_image: impl Fn(&str, &str, u32) -> Result<Vec<u8>>,
    load_right_image: impl Fn(&str, &str, u32) -> Result<Vec<u8>>,
    get_imu_data: impl Fn(i64, i64) -> Vec<ImuData>,
) -> Result<f64> {
    let frame_start = std::time::Instant::now();

    // Inform the estimator about the current frame index for visualization.
    estimator.set_viewer_frame(context.current_idx as i64);

    // Load stereo images (measure decode time)
    let io_start = std::time::Instant::now();
    let left_image = load_left_image(dataset_path, &image_data[context.current_idx].filename, 0)?;
    let right_image = load_right_image(dataset_path, &image_data[context.current_idx].filename, 1)?;
    let io_time_ms = io_start.elapsed().as_secs_f64() * 1000.0;

    if left_image.is_empty() {
        return Err(VIOError::Image(format!(
            "Skipping frame {} due to empty image",
            context.current_idx
        )));
    }

    // Get IMU data for VIO mode (skip first frame since no previous timestamp)
    let imu_start = std::time::Instant::now();
    let imu_data: Option<Vec<ImuData>> = if context.processed_frames > 0 {
        Some(get_imu_data(
            context.previous_frame_timestamp,
            image_data[context.current_idx].timestamp,
        ))
    } else {
        None
    };
    let imu_time_ms = imu_start.elapsed().as_secs_f64() * 1000.0;

    // Process frame (measure estimator time)
    let est_start = std::time::Instant::now();
    let imu_slice = imu_data.as_deref();
    estimator.process_frame(
        &left_image,
        &right_image,
        image_data[context.current_idx].timestamp,
        imu_slice,
    )?;
    let est_time_ms = est_start.elapsed().as_secs_f64() * 1000.0;

    // Update frame timestamp
    context.previous_frame_timestamp = image_data[context.current_idx].timestamp;

    // Accumulate instrumentation
    context.io_decode_time_ms_sum += io_time_ms;
    context.imu_fetch_time_ms_sum += imu_time_ms;
    context.estimator_time_ms_sum += est_time_ms;
    context.frames_timed += 1;

    // Periodic summary (every 50 frames)
    if context.current_idx % 50 == 0 && context.frames_timed > 0 {
        let avg_io = context.io_decode_time_ms_sum / context.frames_timed as f64;
        let avg_imu = context.imu_fetch_time_ms_sum / context.frames_timed as f64;
        let avg_est = context.estimator_time_ms_sum / context.frames_timed as f64;
        let total_avg_ms = avg_io + avg_imu + avg_est;
        let fps = if total_avg_ms > 0.0 { 1000.0 / total_avg_ms } else { 0.0 };
        println!(
            "[Perf] avg_io={:.2}ms avg_imu={:.2}ms avg_est={:.2}ms total={:.2}ms ({:.1} fps) over {} frames",
            avg_io, avg_imu, avg_est, total_avg_ms, fps, context.frames_timed
        );
    }

    let frame_duration = frame_start.elapsed();
    Ok(frame_duration.as_secs_f64() * 1000.0) // Return milliseconds
}

/// Common implementation of save_statistics
///
/// This function handles the shared logic for saving statistics to a file.
pub fn save_statistics_common(result: &PlayerResult, stats_path: &Path) {
    let stats_file = stats_path;

    if let Ok(mut file) = std::fs::File::create(stats_file) {
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
        writeln!(
            file,
            "════════════════════════════════════════════════════════════════════"
        )
        .ok();
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
        writeln!(file).ok();
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
    }

    // Also emit a simple CSV for per-frame timing if available
    if !result.frame_processing_times.is_empty() {
        if let Some(stem) = stats_path.file_stem() {
            let csv_path = stats_path.with_file_name(stem);
            // Append suffix before extension (or just add .csv if none)
            let mut csv_os = csv_path.into_os_string();
            csv_os.push("_frames.csv");
            let csv_path = std::path::PathBuf::from(csv_os);

            if let Ok(mut csv_file) = std::fs::File::create(&csv_path) {
                use std::io::Write;
                writeln!(csv_file, "frame_idx,processing_time_ms").ok();
                for (idx, t_ms) in result.frame_processing_times.iter().enumerate() {
                    writeln!(csv_file, "{},{}", idx, t_ms).ok();
                }
            }
        }
    }
}
