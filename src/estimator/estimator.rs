use crate::calibration::online_intrinsics::{CalibrationObservation, OnlineIntrinsicsRefiner};
use crate::datasets::config::Config;
use crate::datasets::CameraModelType;
use crate::datasets::ImuData;
use crate::debug_log;
use crate::estimator::keyframe_culler::AggressiveCullingConfig;
use crate::estimator::point_quality::PointQualityConfig;
use crate::estimator::Frame;
use crate::estimator::SlidingWindow;
use crate::estimator::{FrameWorkspace, WorkspaceConfig};
use crate::feature_tracker::StereoPatchTracker;
use crate::fl;
use crate::imu::ExtrinsicCalibrator;
use crate::imu::ImuAidedKeyframeSelector;
use crate::imu::ImuBiasEstimator;
use crate::imu::ImuConfig;
use crate::imu::ImuMotionPredictor;
use crate::imu::ImuMotionPrior;
use crate::imu::ImuPreintegrator;
use crate::imu::PreintegratedImu;
use crate::imu::VelocityEstimator;
use crate::imu::{DenoiseConfig, ImuDenoiseFilter};
use crate::imu::{HigherOrderFilter, HigherOrderFilterConfig};
use crate::optimization::loop_closure::{
    KeyframeDescriptor, LoopClosureConfig, LoopClosureDetector,
};
use crate::types::{Float, Matrix4x4, Vector3};
use crate::viewers::Viewer;
use crate::vision::{StereoSuperResolutionConfig, StereoSuperResolver};
use crate::{Result, VIOError};
use image::GrayImage;
use nalgebra as na;
use std::io::Write;
use std::time::{Duration, Instant};

/// Placeholder estimator implementation.
/// Currently mimics the control flow and logging structure of the C++ Estimator::process_frame,
/// but uses dummy values for tracking, optimization, and mapping.
pub struct Estimator {
    frame_id_counter: u64,
    frames_since_last_keyframe: u64,
    /// When true, emit detailed per-frame logs (equivalent to Config::m_enable_debug_output).
    enable_debug_output: bool,
    /// Full configuration loaded from YAML (used to derive intrinsics, etc.).
    pub(crate) config: Config,
    /// Patch-based stereo tracker reused across all frames.
    stereo_patch_tracker: StereoPatchTracker<6>,
    /// Sliding window of keyframes for bundle adjustment optimization.
    sliding_window: SlidingWindow,
    /// Optional viewer used for visualization; owned by the estimator.
    viewer: Option<Box<dyn Viewer>>,
    /// Left camera model with intrinsics and distortion.
    left_cam: CameraModelType,
    /// Right camera model with intrinsics and distortion.
    right_cam: CameraModelType,
    // Transformation from body to left camera
    T_B_Cl: Matrix4x4,
    // Transformation from body to right camera
    T_B_Cr: Matrix4x4,
    // Full trajectory of keyframes with timestamps
    trajectory: Vec<(i64, Matrix4x4)>,
    // Maximum allowed time for frame processing (for real-time safety)
    max_frame_processing_time: Duration,
    // IMU preintegrator for between keyframes
    imu_preintegrator: ImuPreintegrator,
    // IMU motion predictor for feature tracking
    imu_motion_predictor: ImuMotionPredictor,
    // Velocity estimator from accelerometer
    velocity_estimator: VelocityEstimator,
    // Online extrinsic calibrator (IMU to camera)
    extrinsic_calibrator: ExtrinsicCalibrator,
    // IMU-aided keyframe selection
    keyframe_selector: ImuAidedKeyframeSelector,
    // Current preintegrated IMU measurements
    current_imu_preintegration: Option<PreintegratedImu>,
    // Timestamp of last frame for IMU integration
    last_imu_timestamp: Option<i64>,
    // Number of IMU measurements processed
    imu_measurement_count: usize,
    // Current body velocity estimate
    current_velocity: na::Vector3<Float>,
    // Whether velocity estimator has been initialized
    velocity_estimator_initialized: bool,
    // IMU bias estimator for initialization
    bias_estimator: ImuBiasEstimator,
    // Whether system is in initialization phase (collecting IMU for bias estimation)
    is_initializing: bool,
    // Loop-closure detector for global consistency
    loop_closure_detector: LoopClosureDetector,
    // ORB descriptor extractor for loop closure
    orb_extractor: Option<crate::optimization::loop_closure::orb::OrbExtractor>,
    // Frame workspace with preallocated buffers for per-frame processing
    frame_workspace: FrameWorkspace,
    // Frame counter for sampled logging (log every N frames to reduce overhead)
    frame_count: u64,
    // File writer for f0 fundamental frequency logging
    f0_log_writer: std::sync::Mutex<Option<std::fs::File>>,
    // File writer for spectral analysis (gyro power spectrum)
    spectrum_log_writer: std::sync::Mutex<Option<std::fs::File>>,
    // Real-time IMU denoising filter
    denoise_filter: ImuDenoiseFilter,
    // Higher-order filtering (jerk, snap) and f0 analysis
    higher_order_filter: HigherOrderFilter,
    // Stereo super-resolution refinement using IMU confidence signals
    stereo_super_resolver: StereoSuperResolver,
    // Online intrinsics refiner for self-calibration
    intrinsics_refiner: Option<OnlineIntrinsicsRefiner>,
}

impl Estimator {
    #![allow(non_snake_case)]

    /// Create a new estimator configured with camera intrinsics and distortion
    /// loaded from the YAML configuration.
    pub fn new(config: Config, viewer: Option<Box<dyn Viewer>>) -> Self {
        Self::new_with_cameras(config, viewer, None, None)
    }

    /// Create a new estimator with optional camera models.
    /// If camera models are provided, they will be used; otherwise, they will be created from config.
    pub fn new_with_cameras(
        config: Config,
        viewer: Option<Box<dyn Viewer>>,
        left_cam: Option<CameraModelType>,
        right_cam: Option<CameraModelType>,
    ) -> Self {
        // Use provided cameras or create from config
        let (left_cam, right_cam) = match (left_cam, right_cam) {
            (Some(l), Some(r)) => (l, r),
            _ => crate::datasets::create_camera_models_from_config(&config),
        };

        let feature_config = config.feature_detection.clone();

        // Compute the transformation from left to right (T_C1_C0) as in compute_stereo.
        // Cast f64 config values to Float for f32/f64 compatibility
        let T_B_Cl = na::Matrix4::from_row_slice(
            &config
                .camera
                .T_B_Cl
                .iter()
                .map(|&x| x as Float)
                .collect::<Vec<_>>(),
        );
        let T_B_Cr = na::Matrix4::from_row_slice(
            &config
                .camera
                .T_B_Cr
                .iter()
                .map(|&x| x as Float)
                .collect::<Vec<_>>(),
        );
        let keyframe_window_size = config.keyframe_management.keyframe_window_size as usize;
        let processing_timeout_ms = config.keyframe_management.processing_timeout_ms;

        // Initialize aggressive keyframe culling with tuned defaults.
        // These are algorithm parameters that are typically set once during tuning
        // rather than per-dataset configuration. See AggressiveCullingConfig for details.
        let culling_config = Some(AggressiveCullingConfig {
            min_parallax_rad: fl!(0.05),
            min_observations: 20,
            max_similar_poses: 3,
            pose_similarity_translation: 0.1,
            enable_parallax_culling: true,
            enable_observation_culling: true,
            enable_redundancy_culling: true,
            enable_age_culling: true,
            max_age: 10,
            reserve_fraction: 0.7,
        });

        // Initialize point quality scorer with tuned defaults.
        // These parameters control map point filtering and are platform-independent.
        // See PointQualityConfig for parameter descriptions.
        let quality_config = Some(PointQualityConfig {
            min_track_length: 3,
            min_parallax: 0.05,
            max_point_age: 30,
            min_baseline_diversity: 0.1,
            quality_threshold: 0.3,
            auto_cull: true,
            cull_fraction: 0.2,
            max_map_size: 2000,
            detect_dynamic: true,
            motion_inconsistency_threshold: 5.0,
            min_angle_variance: 0.1,
            max_reproj_error: 2.0,
        });

        // Initialize online intrinsics refiner
        let initial_intrinsics = crate::calibration::CameraIntrinsics {
            fx: config.camera.left_intrinsics[0] as f64,
            fy: config.camera.left_intrinsics[1] as f64,
            cx: config.camera.left_intrinsics[2] as f64,
            cy: config.camera.left_intrinsics[3] as f64,
            width: config.camera.image_width,
            height: config.camera.image_height,
        };
        let intrinsics_refiner = Some(OnlineIntrinsicsRefiner::new(initial_intrinsics));

        // Initialize IMU components
        let imu_config = ImuConfig::default();
        // Initialize loop-closure detector with default config
        let loop_config = LoopClosureConfig::default();
        Estimator {
            frame_id_counter: 0,
            frames_since_last_keyframe: 0,
            enable_debug_output: true,
            config: config.clone(),
            stereo_patch_tracker: StereoPatchTracker::<6>::from_config(&feature_config),
            sliding_window: SlidingWindow::with_marginalization_config(
                keyframe_window_size,
                crate::optimization::marginalization::MarginalizationConfig::default(),
                culling_config,
                quality_config,
            ),
            viewer,
            left_cam,
            right_cam,
            T_B_Cl,
            T_B_Cr,
            trajectory: Vec::new(),
            // Default: 100ms deadline for 10Hz operation (with margin)
            // For 30Hz target 33ms, use Duration::from_millis(30)
            max_frame_processing_time: Duration::from_millis(processing_timeout_ms),
            // IMU components
            imu_preintegrator: ImuPreintegrator::new(imu_config.clone()),
            imu_motion_predictor: ImuMotionPredictor::new(imu_config.clone()),
            velocity_estimator: VelocityEstimator::new(imu_config.clone()),
            extrinsic_calibrator: ExtrinsicCalibrator::new(T_B_Cl),
            keyframe_selector: ImuAidedKeyframeSelector::new(
                fl!(config.keyframe_management.translation_threshold),
                fl!(config.keyframe_management.rotation_threshold),
            ),
            current_imu_preintegration: None,
            last_imu_timestamp: None,
            imu_measurement_count: 0,
            current_velocity: na::Vector3::<Float>::zeros(),
            velocity_estimator_initialized: false,
            bias_estimator: ImuBiasEstimator::new(imu_config.clone()),
            is_initializing: true,
            loop_closure_detector: LoopClosureDetector::new(loop_config.clone()),
            orb_extractor: if loop_config.descriptor_type == "orb" {
                use crate::optimization::loop_closure::orb::{OrbConfig, OrbExtractor};
                Some(OrbExtractor::new(OrbConfig::default()))
            } else {
                None
            },
            frame_workspace: FrameWorkspace::new(WorkspaceConfig {
                max_image_width: config.camera.image_width,
                max_image_height: config.camera.image_height,
                // Approximate capacity using grid_cols * max_features_per_grid
                max_features_per_frame: (feature_config.grid_cols
                    * feature_config.max_features_per_grid)
                    as usize,
                max_imu_samples: 200,
                capacity_headroom: 1.2,
            }),
            frame_count: 0,
            f0_log_writer: std::sync::Mutex::new(
                std::fs::File::create("/tmp/f0_data.csv")
                    .ok()
                    .and_then(|mut f| {
                        let _ = writeln!(f, "frame,timestamp_ns,f0_hz");
                        Some(f)
                    })
            ),
            spectrum_log_writer: std::sync::Mutex::new(
                std::fs::File::create("/tmp/gyro_spectrum.csv")
                    .ok()
                    .and_then(|mut f| {
                        let _ = writeln!(f, "frame,timestamp_ns,gyro_x_rms,gyro_y_rms,gyro_z_rms,gyro_magnitude_rms");
                        Some(f)
                    })
            ),
            // Initialize IMU denoising filter with default configuration
            denoise_filter: ImuDenoiseFilter::new(DenoiseConfig::default()),
            // Initialize higher-order filter for jerk/snap and f0 analysis
            higher_order_filter: HigherOrderFilter::new(HigherOrderFilterConfig::default()),
            // Initialize stereo super-resolution refinement
            stereo_super_resolver: StereoSuperResolver::new(StereoSuperResolutionConfig::default()),
            // Initialize online intrinsics refiner
            intrinsics_refiner,
        }
    }

    /// Process a single stereo frame.
    pub fn process_frame(
        &mut self,
        left_image: &[u8],
        right_image: &[u8],
        timestamp_ns: i64,
        imu_data: Option<&[ImuData]>,
    ) -> Result<()> {
        let _total_start_time = Instant::now();
        let deadline = _total_start_time + self.max_frame_processing_time;
        self.frame_count += 1;
        let should_log = self.frame_count % 30 == 0; // Log every 30 frames (~1 Hz @ 30 FPS)

        // New frame: update counters
        self.frame_id_counter += 1;
        self.frames_since_last_keyframe += 1;

        if self.enable_debug_output && should_log {
            debug_log!(
                "============================== Frame {} ==============================",
                self.frame_id_counter
            );
        }

        // Check deadline after initial setup
        if Instant::now() > deadline {
            log::error!("[Estimator] Frame processing exceeded deadline during initialization");
            return Err(VIOError::Optimization(
                "Frame processing timeout".to_string(),
            ));
        }

        // Timing variables
        let mut _frame_creation_time_ms = 0.0f64;
        let mut _patch_tracking_time_ms = 0.0f64;
        let mut _motion_tracking_time_ms = 0.0f64;
        let mut _optimization_time_ms = 0.0f64;

        // Create workspace for frame processing (reusable buffers for RANSAC, descriptors, etc.)
        let mut workspace = crate::estimator::frame_workspace::FrameWorkspace::default(); // Frame creation
        let frame_creation_start = Instant::now();

        // Reset workspace buffers for new frame processing
        self.frame_workspace.reset();

        // Load images into preallocated workspace buffers (zero-copy)
        let img_w = self.config.camera.image_width;
        let img_h = self.config.camera.image_height;

        self.frame_workspace
            .load_left_image(left_image)
            .map_err(|e| {
                log::error!("[Estimator] Failed to load left image: {}", e);
                VIOError::Image(format!("Failed to load left image: {}", e))
            })?;
        self.frame_workspace
            .load_right_image(right_image)
            .map_err(|e| {
                log::error!("[Estimator] Failed to load right image: {}", e);
                VIOError::Image(format!("Failed to load right image: {}", e))
            })?;

        // Create GrayImage objects from workspace buffers without reallocation
        let left_buf = self.frame_workspace.take_left_image_buffer();
        let left_img = match GrayImage::from_raw(img_w, img_h, left_buf) {
            Some(img) => img,
            None => {
                log::error!("[Estimator] Failed to construct GrayImage for left camera");
                return Err(VIOError::Image(
                    "Failed to create left image: size mismatch".to_string(),
                ));
            },
        };
        let right_buf = self.frame_workspace.take_right_image_buffer();
        let right_img = match GrayImage::from_raw(img_w, img_h, right_buf) {
            Some(img) => img,
            None => {
                log::error!(
                    "[Estimator] Failed to construct GrayImage for right camera ({}x{}, len={})",
                    img_w,
                    img_h,
                    right_image.len()
                );
                return Err(VIOError::Image(
                    "Failed to create right image: size mismatch".to_string(),
                ));
            },
        };

        // Create frame (images are not stored, only features will be added)
        let mut current_frame = Frame::from_stereo_images(
            timestamp_ns,
            self.frame_id_counter as i32,
            self.left_cam.clone(),
            self.right_cam.clone(),
            self.T_B_Cl,
            self.T_B_Cr,
        );

        let mut processed_accel: Option<Vec<[f32; 3]>> = None;
        let mut processed_gyro: Option<Vec<[f32; 3]>> = None;

        // Process IMU measurements
        if let Some(imu) = imu_data {
            // Check if IMU is enabled in debug config
            if self.config.debug.use_imu {
                let imu_len = imu.len();
                let mut filtered_imu: Vec<ImuData> = Vec::with_capacity(imu_len);
                let mut accel_filtered: Vec<[f32; 3]> = Vec::with_capacity(imu_len);
                let mut gyro_filtered: Vec<[f32; 3]> = Vec::with_capacity(imu_len);

                let bias_initialized = self.bias_estimator.is_initialized;

                for imu_sample in imu.iter() {
                    if self.is_initializing {
                        self.bias_estimator.add_sample(imu_sample, true);
                    }

                    let (gyro_corrected, accel_corrected) = if bias_initialized {
                        (
                            self.bias_estimator.correct_gyro(imu_sample),
                            self.bias_estimator.correct_accel(imu_sample),
                        )
                    } else {
                        (
                            na::Vector3::new(
                                imu_sample.gyro[0],
                                imu_sample.gyro[1],
                                imu_sample.gyro[2],
                            ),
                            na::Vector3::new(
                                imu_sample.accel[0],
                                imu_sample.accel[1],
                                imu_sample.accel[2],
                            ),
                        )
                    };

                    let gyro_f32 = [
                        gyro_corrected[0] as f32,
                        gyro_corrected[1] as f32,
                        gyro_corrected[2] as f32,
                    ];
                    let accel_f32 = [
                        accel_corrected[0] as f32,
                        accel_corrected[1] as f32,
                        accel_corrected[2] as f32,
                    ];

                    let accel_denoised = self.denoise_filter.process_accel(&accel_f32);
                    let gyro_denoised = self.denoise_filter.process_gyro(&gyro_f32);

                    // Get updated weight after processing
                    let current_weight = self.denoise_filter.weight_scale;

                    // Process acceleration through higher-order filter for jerk/snap and f0 analysis
                    let higher_order_output =
                        self.higher_order_filter.process_accel(accel_denoised);

                    // Use f0 confidence to further weight the measurement
                    // Combine denoise weight with f0 confidence
                    let f0_weighted = if self.higher_order_filter.config.enable_f0_weighting {
                        current_weight * higher_order_output.f0_confidence
                    } else {
                        current_weight
                    };

                    let accel_scaled = [
                        accel_denoised[0] * f0_weighted,
                        accel_denoised[1] * f0_weighted,
                        accel_denoised[2] * f0_weighted,
                    ];
                    let gyro_scaled = [
                        gyro_denoised[0] * f0_weighted,
                        gyro_denoised[1] * f0_weighted,
                        gyro_denoised[2] * f0_weighted,
                    ];

                    accel_filtered.push(accel_scaled);
                    gyro_filtered.push(gyro_scaled);

                    // Direct conversion to f64 for preintegration
                    let accel_vec = na::Vector3::new(
                        accel_scaled[0] as f64,
                        accel_scaled[1] as f64,
                        accel_scaled[2] as f64,
                    );
                    let gyro_vec = na::Vector3::new(
                        gyro_scaled[0] as f64,
                        gyro_scaled[1] as f64,
                        gyro_scaled[2] as f64,
                    );

                    if let Some(last_ts) = self.last_imu_timestamp {
                        let dt = (imu_sample.timestamp - last_ts) as f64 / 1e9;
                        if dt > 0.0 {
                            self.imu_preintegrator
                                .propagate_corrected(gyro_vec, accel_vec, dt);
                        }
                    }
                    self.last_imu_timestamp = Some(imu_sample.timestamp);
                    self.imu_measurement_count += 1;

                    filtered_imu.push(ImuData {
                        timestamp: imu_sample.timestamp,
                        gyro: [gyro_vec[0], gyro_vec[1], gyro_vec[2]],
                        accel: [accel_vec[0], accel_vec[1], accel_vec[2]],
                    });

                    if self.is_initializing && self.bias_estimator.sample_count() >= 100 {
                        self.is_initializing = false;
                        log::info!(
                            "[Estimator] IMU initialization complete. Gyro bias: [{:.4}, {:.4}, {:.4}] rad/s, Accel bias: [{:.4}, {:.4}, {:.4}] m/s²",
                            self.bias_estimator.gyro_bias[0],
                            self.bias_estimator.gyro_bias[1],
                            self.bias_estimator.gyro_bias[2],
                            self.bias_estimator.accel_bias[0],
                            self.bias_estimator.accel_bias[1],
                            self.bias_estimator.accel_bias[2]
                        );
                    }
                }

                for sample in &filtered_imu {
                    self.frame_workspace.push_imu_sample(sample).map_err(|e| {
                        log::warn!("[Estimator] IMU buffer overflow: {}", e);
                        VIOError::Optimization(e)
                    })?;
                }

                // Attach filtered IMU measurements to frame (clone from workspace)
                current_frame.imu_from_last_frame = self.frame_workspace.imu_samples().to_vec();
                processed_accel = Some(accel_filtered);
                processed_gyro = Some(gyro_filtered);

                // Use motion predictor for feature tracking
                let focal_length = self.config.camera.left_intrinsics[0] as f64;
                for feature in &mut current_frame.left_features {
                    let (du, dv) = self.imu_motion_predictor.predict_feature_displacement(
                        &filtered_imu,
                        (feature.pixel_coord[0] as f64, feature.pixel_coord[1] as f64),
                        focal_length,
                    );
                    // Apply predicted displacement as initial guess for optical flow
                    feature.pixel_coord[0] = (feature.pixel_coord[0] as f64 + du) as f32;
                    feature.pixel_coord[1] = (feature.pixel_coord[1] as f64 + dv) as f32;
                }

                // Update velocity estimator
                let dt = if let Some(last_ts) = self.last_imu_timestamp {
                    (timestamp_ns - last_ts) as f64 / 1e9
                } else {
                    0.01
                };

                // Initialize velocity estimator on first IMU batch
                if !self.velocity_estimator_initialized && !filtered_imu.is_empty() {
                    let initial_orientation = na::UnitQuaternion::identity();
                    self.velocity_estimator
                        .initialize_from_imu(&filtered_imu, &initial_orientation);
                    self.velocity_estimator_initialized = true;
                    if should_log {
                        debug_log!(
                            "[Estimator] Velocity estimator initialized with {} IMU samples",
                            filtered_imu.len()
                        );
                    }
                }

                // Update velocity estimator
                self.velocity_estimator.update(&filtered_imu, dt);

                // Accumulate for extrinsic calibration
                if self.sliding_window.is_full() {
                    let keyframe_poses = self.sliding_window.get_keyframe_poses();
                    if let Some(T_W_B) = keyframe_poses.last() {
                        let T_W_B_copy = *T_W_B;
                        let rotmat = na::Rotation3::from_matrix_unchecked(
                            T_W_B_copy.fixed_view::<3, 3>(0, 0).into_owned(),
                        );
                        let R_W_B = na::UnitQuaternion::from_rotation_matrix(&rotmat);
                        self.extrinsic_calibrator
                            .add_measurement(&T_W_B_copy, R_W_B);

                        // Run calibration periodically
                        if self.imu_measurement_count % 100 == 0 {
                            // Temporarily disabled due to NaN issues
                            // let error = self.extrinsic_calibrator.calibrate_iteration();
                            // log::debug!(
                            //     "[Estimator] IMU extrinsic calibration error: {:.6} rad",
                            //     error
                            // );
                        }
                    }
                }
            } else if should_log {
                debug_log!("[Estimator] IMU disabled via debug config");
            }
        }

        _frame_creation_time_ms = frame_creation_start.elapsed().as_secs_f64() * 1000.0;

        // Visualize IMU data (before and after processing)
        if let Some(imu) = imu_data {
            eprintln!(
                "[IMU_VIZ] Frame {}: Received {} IMU samples, use_imu={}",
                self.frame_count,
                imu.len(),
                self.config.debug.use_imu
            );
            if self.config.debug.use_imu {
                if let (Some(accel), Some(gyro)) =
                    (processed_accel.as_ref(), processed_gyro.as_ref())
                {
                    eprintln!(
                        "[IMU_VIZ] Frame {}: Calling view_imu_results() with {} samples",
                        self.frame_count,
                        imu.len()
                    );
                    self.view_imu_results(imu, accel, gyro, timestamp_ns);
                    eprintln!(
                        "[IMU_VIZ] Frame {}: Finished IMU visualization",
                        self.frame_count
                    );
                } else {
                    eprintln!(
                        "[IMU_VIZ] Frame {}: Skipping visualization (no processed IMU data available)",
                        self.frame_count
                    );
                }
            }
        } else {
            eprintln!(
                "[IMU_VIZ] Frame {}: NO IMU DATA RECEIVED (imu_data is None)",
                self.frame_count
            );
        }

        // Patch tracking
        if let Some(imu) = imu_data {
            let fx = self
                .config
                .camera
                .left_intrinsics
                .get(0)
                .copied()
                .unwrap_or(500.0) as f32;
            let fy = self
                .config
                .camera
                .left_intrinsics
                .get(1)
                .copied()
                .unwrap_or(500.0) as f32;
            let cx = self
                .config
                .camera
                .left_intrinsics
                .get(2)
                .copied()
                .unwrap_or(320.0) as f32;
            let cy = self
                .config
                .camera
                .left_intrinsics
                .get(3)
                .copied()
                .unwrap_or(240.0) as f32;

            // Always set calibrated intrinsics for geometric gating
            self.stereo_patch_tracker
                .set_camera_intrinsics(fx, fy, cx, cy);

            if self.config.debug.use_imu {
                self.stereo_patch_tracker
                    .set_imu_rotation_hint(imu, (fx, fy, cx, cy));
            }
        }

        let tracking_start = Instant::now();
        self.stereo_patch_tracker
            .process_frame(&left_img, &right_img, &mut current_frame);
        _patch_tracking_time_ms = tracking_start.elapsed().as_secs_f64() * 1000.0;
        self.view_patch_tracking_results(&current_frame, &left_img, &right_img, img_w, img_h);

        // Stereo super-resolution refinement with IMU confidence weighting
        // This refines subpixel disparities using confidence from IMU noise/motion analysis
        if !current_frame.left_features.is_empty() && !current_frame.right_features.is_empty() {
            let _superres_start = Instant::now();

            // Compute IMU confidence metric from denoise and higher-order filters
            let denoise_weight = self.denoise_filter.weight_scale;
            let f0_confidence = self.higher_order_filter.f0_confidence;
            let imu_confidence =
                ((denoise_weight as Float) * (f0_confidence as Float)).clamp(0.0, 1.0);

            // Estimate motion state from acceleration magnitude
            let accel_magnitude = if let Some(accel) = processed_accel.as_ref() {
                if !accel.is_empty() {
                    let last_accel = accel[accel.len() - 1];
                    (last_accel[0] * last_accel[0]
                        + last_accel[1] * last_accel[1]
                        + last_accel[2] * last_accel[2])
                        .sqrt()
                } else {
                    0.0
                }
            } else {
                0.0
            };

            let motion_state = match accel_magnitude {
                x if x < 1.0 => "hover",
                x if x < 3.0 => "moving",
                _ => "accelerating",
            };

            // Get current acceleration for motion compensation
            let current_accel = if let Some(accel) = processed_accel.as_ref() {
                if !accel.is_empty() {
                    let last = accel[accel.len() - 1];
                    [last[0] as Float, last[1] as Float, last[2] as Float]
                } else {
                    [0.0 as Float, 0.0, 0.0]
                }
            } else {
                [0.0 as Float, 0.0, 0.0]
            };

            // Collect feature coordinates for refinement
            let left_coords: Vec<(Float, Float)> = current_frame
                .left_features
                .iter()
                .map(|f| (f.pixel_coord[0] as Float, f.pixel_coord[1] as Float))
                .collect();

            let right_coords: Vec<(Float, Float)> = current_frame
                .right_features
                .iter()
                .map(|f| (f.pixel_coord[0] as Float, f.pixel_coord[1] as Float))
                .collect();

            let feature_ids: Vec<usize> = current_frame
                .left_features
                .iter()
                .map(|f| f.feature_id)
                .collect();

            if left_coords.len() == right_coords.len() && !left_coords.is_empty() {
                let left_data = left_img.as_raw();
                let right_data = right_img.as_raw();

                let refined_features = self.stereo_super_resolver.refine_features(
                    left_data,
                    right_data,
                    img_w,
                    img_h,
                    &left_coords,
                    &right_coords,
                    &feature_ids,
                    imu_confidence,
                    motion_state,
                    &current_accel,
                );

                // Apply refined coordinates to features
                for (idx, (left_feat, right_feat)) in current_frame
                    .left_features
                    .iter_mut()
                    .zip(current_frame.right_features.iter_mut())
                    .enumerate()
                {
                    if idx < refined_features.len() && refined_features[idx].is_valid {
                        left_feat.pixel_coord[0] = refined_features[idx].left_x_refined as f32;
                        left_feat.pixel_coord[1] = refined_features[idx].left_y_refined as f32;
                        right_feat.pixel_coord[0] = refined_features[idx].right_x_refined as f32;
                        right_feat.pixel_coord[1] = refined_features[idx].right_y_refined as f32;
                    }
                }

                if should_log {
                    debug_log!(
                        "[Estimator] Stereo super-resolution: refined {} features (IMU confidence: {:.3}, motion: {})",
                        refined_features.len(),
                        imu_confidence,
                        motion_state
                    );
                }
            }
        }

        // Return image buffers to workspace for reuse
        let left_buf_back = left_img.into_raw();
        let right_buf_back = right_img.into_raw();
        self.frame_workspace.put_left_image_buffer(left_buf_back);
        self.frame_workspace.put_right_image_buffer(right_buf_back);

        // Check deadline after patch tracking
        if Instant::now() > deadline {
            log::warn!("[Estimator] Frame processing exceeded deadline after patch tracking");
            return Err(VIOError::Optimization(
                "Frame processing timeout after patch tracking".to_string(),
            ));
        }

        // Motion tracking - only if the sliding window is full (has initialized keyframes)
        if self.sliding_window.is_full() {
            // DEBUG
            let motion_tracking_start = Instant::now();
            let motion_tracking_result = self.sliding_window.track_motion(&current_frame);
            match motion_tracking_result {
                Ok(Some(T_W_B)) => {
                    // Apply the optimized pose to the current frame
                    current_frame.state.T_W_B = T_W_B;

                    // IMU-aided keyframe selection
                    let keyframe_poses = self.sliding_window.get_keyframe_poses();
                    let T_W_B_last_kf = match keyframe_poses.last() {
                        Some(pose) => pose,
                        None => {
                            log::error!("[Estimator] No keyframe poses available");
                            return Err(VIOError::Optimization(
                                "No keyframe poses available".to_string(),
                            ));
                        },
                    };

                    // Compute IMU-visual rotation deviation
                    let R_obs = na::Rotation3::from_matrix_unchecked(
                        T_W_B.fixed_view::<3, 3>(0, 0).into_owned(),
                    );
                    let R_last = na::Rotation3::from_matrix_unchecked(
                        T_W_B_last_kf.fixed_view::<3, 3>(0, 0).into_owned(),
                    );
                    let R_obs_quat = na::UnitQuaternion::from_rotation_matrix(&R_obs);
                    let R_last_quat = na::UnitQuaternion::from_rotation_matrix(&R_last);
                    let dq = R_last_quat.inverse() * R_obs_quat;
                    let imu_visual_deviation = dq.angle();

                    // Use IMU-aided keyframe selector
                    let (is_imu_keyframe, keyframe_reason) = self
                        .keyframe_selector
                        .should_be_keyframe(&T_W_B, timestamp_ns, imu_visual_deviation.abs());

                    // Fallback to visual-only check
                    let T_W_B_last_kf_inv = match T_W_B_last_kf.try_inverse() {
                        Some(inv) => inv,
                        None => {
                            log::error!("[Estimator] Matrix inversion failed for T_W_B_last_kf");
                            return Err(VIOError::Optimization(
                                "Matrix inversion failed".to_string(),
                            ));
                        },
                    };
                    let T_rel = T_W_B * T_W_B_last_kf_inv;
                    let t_rel = T_rel.fixed_view::<3, 1>(0, 3).into_owned();
                    let R_rel = T_rel.fixed_view::<3, 3>(0, 0).into_owned();
                    let rotmat = na::Rotation3::from_matrix_unchecked(R_rel);
                    let euler: (Float, Float, Float) = rotmat.euler_angles();
                    let rotation_norm = (euler.0.abs() + euler.1.abs() + euler.2.abs()).abs();

                    // Keyframe if either visual or IMU criteria met
                    let translation_threshold =
                        fl!(self.config.keyframe_management.translation_threshold);
                    let rotation_threshold =
                        fl!(self.config.keyframe_management.rotation_threshold);
                    let visual_keyframe =
                        t_rel.norm() > translation_threshold || rotation_norm > rotation_threshold;

                    let is_keyframe = is_imu_keyframe || visual_keyframe;

                    if is_keyframe {
                        let _reason = if is_imu_keyframe && !visual_keyframe {
                            format!("IMU-aided: {}", keyframe_reason)
                        } else if visual_keyframe {
                            let t_norm = t_rel.norm();
                            format!("Visual: trans={:.3}m, rot={:.3}rad", t_norm, rotation_norm)
                        } else {
                            keyframe_reason
                        };
                        debug_log!("[Estimator] Keyframe triggered: {}", _reason);

                        current_frame.is_keyframe = true;
                    } else {
                        current_frame.is_keyframe = false;
                    }
                    self.view_motion_tracking_results(&T_W_B);
                },
                Ok(None) => {
                    log::warn!(
                        "[Estimator] Motion tracking failed (optimization did not converge)"
                    );
                },
                Err(e) => {
                    log::error!("[Estimator] Motion tracking error: {:?}", e);
                },
            }
            _motion_tracking_time_ms = motion_tracking_start.elapsed().as_secs_f64() * 1000.0;
        } else {
            debug_log!("[Estimator] Sliding window is not full, skipping motion tracking");
        }

        // View map points and keyframe poses
        // Bundle adjustment
        if current_frame.is_keyframe {
            let optimization_start = Instant::now();
            // Save timestamp and pose before frame is moved
            let frame_timestamp = current_frame.timestamp_ns;
            let frame_pose = current_frame.state.T_W_B;
            let kf_id = self.frame_id_counter;

            // Detect loop closures for this keyframe
            let descriptor =
                self.create_keyframe_descriptor(kf_id, frame_timestamp, &current_frame, frame_pose);

            if let Ok(constraints) =
                self.loop_closure_detector
                    .detect_loop_closure(kf_id, descriptor, &mut workspace)
            {
                if !constraints.is_empty() {
                    log::info!(
                        "[Estimator] Detected {} loop closure(s) for keyframe {}",
                        constraints.len(),
                        kf_id
                    );
                    self.sliding_window
                        .add_loop_closure_constraints(constraints);
                }
            }

            self.sliding_window.add_frame(current_frame.clone());

            // Add observations to intrinsics refiner for self-calibration
            self.add_intrinsics_observations(&current_frame);

            // Provide IMU motion prior to optimizer when available
            let imu_prior = if self.config.optimization.imu_prior_enable {
                self.get_imu_motion_prior()
            } else {
                None
            };
            let imu_weights = if self.config.optimization.imu_prior_enable {
                Some((
                    self.config.optimization.imu_prior_weight_pos,
                    self.config.optimization.imu_prior_weight_rot,
                ))
            } else {
                None
            };
            let imu_huber_delta = if self.config.optimization.imu_prior_enable {
                Some(self.config.optimization.imu_prior_huber_delta)
            } else {
                None
            };

            // Store IMU preintegration for tight coupling BEFORE optimization
            // This ensures the most recent preintegration (between last two KFs) is available
            let preintegration_for_storage = if self.config.optimization.imu_prior_enable {
                self.imu_preintegrator
                    .create_tight_coupling_preintegration()
            } else {
                None
            };

            // Add IMU preintegration to sliding window BEFORE optimization
            // This enables tight-coupled IMU factors in the optimization
            if let Some(ref preint) = preintegration_for_storage {
                let prev_kf_idx = self.sliding_window.len().saturating_sub(2);
                if prev_kf_idx < self.sliding_window.len() {
                    self.sliding_window.add_imu_preintegration(
                        prev_kf_idx,
                        prev_kf_idx + 1,
                        preint.clone(),
                    );
                }
            }

            if let Err(e) =
                self.sliding_window
                    .optimize_with_imu(imu_prior, imu_weights, imu_huber_delta)
            {
                log::error!("[Estimator] Bundle adjustment optimization failed: {:?}", e);
                // Continue execution even if optimization fails
            }

            // Reset IMU preintegrator after optimization for next interval
            if self.config.optimization.imu_prior_enable {
                self.imu_preintegrator.reset();
            }

            // Update camera intrinsics from online refiner after optimization
            self.update_intrinsics_from_refiner();

            // Update extrinsics from calibrator periodically
            self.update_extrinsics_from_calibrator();

            _optimization_time_ms = optimization_start.elapsed().as_secs_f64() * 1000.0;
            self.view_optimization_results(frame_timestamp);
        }

        // Final timing summary
        let _total_duration_ms = _total_start_time.elapsed().as_secs_f64() * 1000.0;
        debug_log!(
            "[Timing] frame_creation={:.3} ms, patch_tracking={:.3} ms, motion_tracking={:.3} ms, optimization={:.3} ms, total={:.3} ms",
            _frame_creation_time_ms,
            _patch_tracking_time_ms,
            _motion_tracking_time_ms,
            _optimization_time_ms,
            _total_duration_ms
        );

        Ok(())
    }

    /// Helper: set the current frame index on the attached viewer, if any.
    pub fn set_viewer_frame(&mut self, frame_id: i64) {
        if let Some(v) = &mut self.viewer {
            v.set_frame(frame_id);
        }
    }

    /// Test hook: set maximum map points for bounding memory during tests.
    pub fn set_max_map_points(&mut self, max_map_points: usize) {
        self.sliding_window.set_max_map_points(max_map_points);
    }

    /// Test hook: inspect current map point count.
    pub fn map_points_len(&self) -> usize {
        self.sliding_window.map_points_len()
    }

    /// Test hook: adjust frame processing deadline for timing-sensitive tests.
    pub fn set_max_frame_processing_time(&mut self, duration: Duration) {
        self.max_frame_processing_time = duration;
    }

    /// Test hook: number of frames processed.
    pub fn frame_count(&self) -> u64 {
        self.frame_count
    }

    /// Create a keyframe descriptor for loop-closure detection
    fn create_keyframe_descriptor(
        &self,
        keyframe_id: u64,
        timestamp: i64,
        frame: &Frame,
        pose: Matrix4x4,
    ) -> KeyframeDescriptor {
        // Convert Matrix4x4 to Isometry3 (used in both branches)
        let t = pose.fixed_view::<3, 1>(0, 3);
        let translation = na::Translation3::from(t.clone_owned());
        let rotation =
            na::Rotation3::from_matrix_unchecked(pose.fixed_view::<3, 3>(0, 0).into_owned());
        let pose_isometry = na::Isometry3::from_parts(
            translation,
            na::UnitQuaternion::from_rotation_matrix(&rotation),
        );

        // Try ORB descriptor if extractor is available
        if self.orb_extractor.is_some() {
            // Use ORB features from left image
            // Note: In a real implementation, you would pass the actual grayscale image data
            // For now, we'll extract features from the first frame's features
            let num_features = frame.left_features.len().max(frame.right_features.len());

            // Create a simple descriptor from feature statistics combined with presence flag
            let mut descriptor = vec![fl!(0.0); 10];
            descriptor[0] = if self.orb_extractor.is_some() {
                fl!(1.0)
            } else {
                fl!(0.0)
            }; // ORB enabled flag
            descriptor[1] = num_features as Float / fl!(200.0); // Normalized feature count

            // Add left image feature statistics
            if !frame.left_features.is_empty() {
                let avg_x: f32 = frame
                    .left_features
                    .iter()
                    .map(|f| f.pixel_coord[0])
                    .sum::<f32>()
                    / frame.left_features.len() as f32;
                let avg_y: f32 = frame
                    .left_features
                    .iter()
                    .map(|f| f.pixel_coord[1])
                    .sum::<f32>()
                    / frame.left_features.len() as f32;
                descriptor[2] = avg_x as Float / self.config.camera.image_width as Float;
                descriptor[3] = avg_y as Float / self.config.camera.image_height as Float;
                descriptor[4] = frame.left_features.len() as Float / fl!(200.0);
            }

            // Add right image feature statistics
            if !frame.right_features.is_empty() {
                let avg_x: f32 = frame
                    .right_features
                    .iter()
                    .map(|f| f.pixel_coord[0])
                    .sum::<f32>()
                    / frame.right_features.len() as f32;
                let avg_y: f32 = frame
                    .right_features
                    .iter()
                    .map(|f| f.pixel_coord[1])
                    .sum::<f32>()
                    / frame.right_features.len() as f32;
                descriptor[5] = avg_x as Float / self.config.camera.image_width as Float;
                descriptor[6] = avg_y as Float / self.config.camera.image_height as Float;
                descriptor[7] = frame.right_features.len() as Float / fl!(200.0);
            }

            // Pose-derived features
            descriptor[8] = (t[0] / fl!(10.0)).tanh();
            descriptor[9] = (t[1] / fl!(10.0)).tanh();

            KeyframeDescriptor {
                keyframe_id,
                timestamp,
                descriptor,
                num_features,
                pose: pose_isometry,
            }
        } else {
            // Fallback to simple descriptor
            let num_features = frame.left_features.len().max(frame.right_features.len());

            // Create a simple descriptor from feature statistics (10-dim vector)
            let mut descriptor = vec![fl!(0.0); 10];

            if !frame.left_features.is_empty() {
                let avg_x: f32 = frame
                    .left_features
                    .iter()
                    .map(|f| f.pixel_coord[0])
                    .sum::<f32>()
                    / frame.left_features.len() as f32;
                let avg_y: f32 = frame
                    .left_features
                    .iter()
                    .map(|f| f.pixel_coord[1])
                    .sum::<f32>()
                    / frame.left_features.len() as f32;
                descriptor[0] = avg_x as Float / self.config.camera.image_width as Float;
                descriptor[1] = avg_y as Float / self.config.camera.image_height as Float;

                // Add feature distribution stats
                descriptor[2] = frame.left_features.len() as Float / fl!(200.0);
                // Normalized feature count
            }

            if !frame.right_features.is_empty() {
                let avg_x: f32 = frame
                    .right_features
                    .iter()
                    .map(|f| f.pixel_coord[0])
                    .sum::<f32>()
                    / frame.right_features.len() as f32;
                let avg_y: f32 = frame
                    .right_features
                    .iter()
                    .map(|f| f.pixel_coord[1])
                    .sum::<f32>()
                    / frame.right_features.len() as f32;
                descriptor[3] = avg_x as Float / self.config.camera.image_width as Float;
                descriptor[4] = avg_y as Float / self.config.camera.image_height as Float;

                descriptor[5] = frame.right_features.len() as Float / fl!(200.0);
            }

            // Fill remaining dimensions with pose-derived features
            descriptor[6] = (t[0] / fl!(10.0)).tanh(); // Position features (bounded)
            descriptor[7] = (t[1] / fl!(10.0)).tanh();
            descriptor[8] = (t[2] / fl!(10.0)).tanh();
            descriptor[9] = num_features as Float / fl!(200.0);

            KeyframeDescriptor {
                keyframe_id,
                timestamp,
                descriptor,
                num_features,
                pose: pose_isometry,
            }
        }
    }

    /// Visualize tracking results: stereo images with tracked features.
    fn view_patch_tracking_results(
        &mut self,
        frame: &Frame,
        left_img: &GrayImage,
        right_img: &GrayImage,
        width: u32,
        height: u32,
    ) {
        if let Some(v) = &mut self.viewer {
            // Collect pixel coordinates into temporary vectors so we can pass slices.
            let left_points: Vec<(usize, [f32; 2])> = frame
                .left_features()
                .iter()
                .map(|f| (f.feature_id, f.pixel_coord))
                .collect();
            let right_points: Vec<(usize, [f32; 2])> = frame
                .right_features()
                .iter()
                .map(|f| (f.feature_id, f.pixel_coord))
                .collect();

            v.log_image_with_features_colored(
                left_img.as_raw(),
                width,
                height,
                &left_points,
                "stereo/left",
            );
            v.log_image_with_features_colored(
                right_img.as_raw(),
                width,
                height,
                &right_points,
                "stereo/right",
            );
        }
    }

    fn view_motion_tracking_results(&mut self, T_W_B: &Matrix4x4) {
        if let Some(v) = &mut self.viewer {
            let pose_path = "pose_current".to_string();
            v.log_pose(*T_W_B, pose_path.as_str());

            let width = self.config.camera.image_width;
            let height = self.config.camera.image_height;

            // Log left camera frustum at the pose location (left camera is at the pose)
            let left_focal_length = self.config.camera.left_intrinsics[0] as f32;
            let left_cam_path = format!("{}_left", pose_path);
            let T_W_Cl = T_W_B * self.T_B_Cl;
            v.log_pose(T_W_Cl, left_cam_path.as_str());
            v.log_camera_frustum(
                left_focal_length,
                width,
                height,
                left_cam_path.as_str(),
                0.4,
            );
        }
    }

    /// Visualize optimization results: map points, keyframe poses, and camera frustums.
    fn view_optimization_results(&mut self, timestamp_ns: i64) {
        // History of keyframe poses with timestamps (update even without viewer for saving)
        let keyframe_poses = self.sliding_window.get_keyframe_poses();
        if let Some(&mat) = keyframe_poses.last() {
            self.trajectory.push((timestamp_ns, mat));
        }

        // Viewer-only visualization (skip if no viewer)
        if let Some(v) = &mut self.viewer {
            // Map points
            let colored_points: Vec<(usize, [f32; 3])> = self
                .sliding_window
                .map_points
                .iter()
                .map(|(&feature_id, &point)| (feature_id, point))
                .collect();
            v.log_points_colored(&colored_points, "map/points");

            // Keyframe poses with left and right camera frustrums
            let system_poses = self.sliding_window.get_keyframe_poses();
            for (pose_id, T_W_B) in system_poses.iter().enumerate() {
                // pose is T_W_B
                let pose_path = format!("pose_{}", pose_id);
                v.log_pose(*T_W_B, pose_path.as_str());

                let width = self.config.camera.image_width;
                let height = self.config.camera.image_height;

                // Log left camera frustum at the pose location (left camera is at the pose)
                let left_focal_length = self.config.camera.left_intrinsics[0] as f32;
                let left_cam_path = format!("{}_left", pose_path);
                let T_W_Cl = T_W_B * self.T_B_Cl;
                v.log_pose(T_W_Cl, left_cam_path.as_str());
                let size = 0.2; // if pose_id == system_poses.len() - 1 { 0.5 } else { 0.2 };
                v.log_camera_frustum(
                    left_focal_length,
                    width,
                    height,
                    left_cam_path.as_str(),
                    size,
                );

                // Log right camera pose and frustum for last pose
                // Removed for now as it made too much clutter
                /*
                if pose_id == system_poses.len() - 1 {
                    let right_cam_path = format!("{}_right", pose_path);
                    let right_focal_length = self.config.camera.right_intrinsics[0] as f32;
                    let T_W_Cr = T_W_B * self.T_B_Cr;
                    v.log_pose(T_W_Cr, right_cam_path.as_str());
                    v.log_camera_frustum(right_focal_length, width, height, right_cam_path.as_str(), size);
                }
                */
            }

            // Display trajectory as a continuous 3D path
            let trajectory_poses: Vec<Matrix4x4> =
                self.trajectory.iter().map(|(_, pose)| *pose).collect();
            v.log_trajectory(&trajectory_poses, "trajectory/path");
        }
    }

    /// Visualize IMU data: raw measurements, bias-corrected data, and harmonic decomposition
    fn view_imu_results(
        &mut self,
        imu_data: &[crate::datasets::ImuData],
        processed_accel: &[[f32; 3]],
        processed_gyro: &[[f32; 3]],
        timestamp_ns: i64,
    ) {
        if imu_data.is_empty() {
            return;
        }

        debug_log!(
            "[Estimator] Logging {} IMU samples to Rerun viewer",
            imu_data.len()
        );

        // Convert IMU data to format for visualization (don't hold mutable borrow of self)
        let mut raw_accel = Vec::with_capacity(imu_data.len());
        let mut raw_gyro = Vec::with_capacity(imu_data.len());

        for imu_sample in imu_data {
            raw_accel.push([
                imu_sample.accel[0] as f32,
                imu_sample.accel[1] as f32,
                imu_sample.accel[2] as f32,
            ]);
            raw_gyro.push([
                imu_sample.gyro[0] as f32,
                imu_sample.gyro[1] as f32,
                imu_sample.gyro[2] as f32,
            ]);
        }

        // Convert to f32 slices for visualization
        let raw_accel_f32: Vec<[f32; 3]> = raw_accel;
        let raw_gyro_f32: Vec<[f32; 3]> = raw_gyro;
        let processed_accel_f32: Vec<[f32; 3]> = processed_accel.to_vec();
        let processed_gyro_f32: Vec<[f32; 3]> = processed_gyro.to_vec();

        // Compute decomposition before getting viewer mutable borrow
        let decomp_result =
            self.compute_imu_decomposition(&processed_accel_f32, &processed_gyro_f32);
        let fundamental_freq = self.estimate_fundamental_frequency(&processed_gyro_f32);

        // Log denoising filter quality
        let _filter_quality = self.denoise_filter.quality();
        debug_log!(
            "[DENOISE] Frame {}: quality={:.2}, samples={}",
            self.frame_count,
            _filter_quality,
            imu_data.len()
        );

        // Log f0 to CSV file
        if let Ok(mut writer) = self.f0_log_writer.lock() {
            if let Some(ref mut f) = *writer {
                let _ = writeln!(
                    f,
                    "{},{},{:.2}",
                    self.frame_count, timestamp_ns, fundamental_freq
                );
                let _ = f.flush();
            }
        }

        // Log gyroscope spectrum (RMS values per axis) to CSV file
        let gyro_x_rms = (processed_gyro_f32.iter().map(|g| g[0] * g[0]).sum::<f32>()
            / processed_gyro_f32.len() as f32)
            .sqrt();
        let gyro_y_rms = (processed_gyro_f32.iter().map(|g| g[1] * g[1]).sum::<f32>()
            / processed_gyro_f32.len() as f32)
            .sqrt();
        let gyro_z_rms = (processed_gyro_f32.iter().map(|g| g[2] * g[2]).sum::<f32>()
            / processed_gyro_f32.len() as f32)
            .sqrt();
        let gyro_mag_rms = (processed_gyro_f32
            .iter()
            .map(|g| g[0] * g[0] + g[1] * g[1] + g[2] * g[2])
            .sum::<f32>()
            / processed_gyro_f32.len() as f32)
            .sqrt();

        if let Ok(mut writer) = self.spectrum_log_writer.lock() {
            if let Some(ref mut f) = *writer {
                let _ = writeln!(
                    f,
                    "{},{},{:.6},{:.6},{:.6},{:.6}",
                    self.frame_count,
                    timestamp_ns,
                    gyro_x_rms,
                    gyro_y_rms,
                    gyro_z_rms,
                    gyro_mag_rms
                );
                let _ = f.flush();
            }
        }

        // Now get mutable borrow for viewer
        if let Some(v) = &mut self.viewer {
            // Log raw IMU data
            v.log_imu_raw(timestamp_ns, &raw_accel_f32, &raw_gyro_f32, "imu/raw");

            // Log processed (bias-corrected) IMU data
            v.log_imu_processed(
                timestamp_ns,
                &processed_accel_f32,
                &processed_gyro_f32,
                "imu/processed",
            );

            // Log harmonic decomposition if available
            if let Some((gravity, vibration)) = decomp_result {
                // Log harmonic decomposition (gravity, bias, harmonics)
                let bias_accel = [
                    self.bias_estimator.accel_bias[0] as f32,
                    self.bias_estimator.accel_bias[1] as f32,
                    self.bias_estimator.accel_bias[2] as f32,
                ];
                let bias_gyro = [
                    self.bias_estimator.gyro_bias[0] as f32,
                    self.bias_estimator.gyro_bias[1] as f32,
                    self.bias_estimator.gyro_bias[2] as f32,
                ];

                v.log_imu_harmonics(
                    timestamp_ns,
                    gravity,
                    bias_accel,
                    bias_gyro,
                    &[vibration],
                    "imu/harmonics",
                );

                // Log signal quality metrics (including fundamental frequency f0)
                let snr_values = [1.0_f32, 1.0_f32, 1.0_f32]; // Placeholder
                let rms_values = [0.5_f32, 0.5_f32, 0.5_f32]; // Placeholder
                let peak_values = [1.0_f32, 1.0_f32, 1.0_f32]; // Placeholder
                v.log_imu_signal_quality(
                    timestamp_ns,
                    snr_values,
                    rms_values,
                    peak_values,
                    "running",
                    fundamental_freq,
                    "imu/quality",
                );
            }
        }
    }

    /// Estimate the fundamental frequency (f0) from accelerometer data
    fn estimate_fundamental_frequency(&self, gyro_data: &[[f32; 3]]) -> f32 {
        if gyro_data.len() < 2 {
            return 0.0;
        }

        // Sample rate from IMU (EuRoC is typically 200 Hz)
        let sample_rate = 200.0_f32; // Hz

        // Compute magnitude of gyro vector
        let magnitudes: Vec<f32> = gyro_data
            .iter()
            .map(|g| (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt())
            .collect();

        // Remove DC component (subtract mean)
        let mean = magnitudes.iter().sum::<f32>() / magnitudes.len() as f32;
        let centered: Vec<f32> = magnitudes.iter().map(|&m| m - mean).collect();

        // Simple peak detection: count local maxima (simplified zero-crossing on derivative)
        let mut peak_count = 0;
        for i in 1..centered.len().saturating_sub(1) {
            if centered[i] > centered[i - 1] && centered[i] > centered[i + 1] {
                peak_count += 1;
            }
        }

        // Frequency = peaks * (sample_rate / window_duration)
        // With 2 peaks per cycle: f0 = (peak_count / 2) * sample_rate / duration
        if peak_count > 0 {
            let duration_seconds = centered.len() as f32 / sample_rate;
            (peak_count as f32 / 2.0) / duration_seconds
        } else {
            // Fallback: estimate from RMS of signal as approximate frequency indicator
            let rms = (centered.iter().map(|x| x * x).sum::<f32>() / centered.len() as f32).sqrt();
            // Map RMS to rough frequency (typical sensor vibration 5-50 Hz)
            if rms > 0.01 {
                5.0 + rms * 100.0 // Simple linear mapping
            } else {
                0.0
            }
        }
    }

    /// Decompose processed IMU data into gravity and vibration components
    fn compute_imu_decomposition(
        &self,
        accel: &[[f32; 3]],
        _gyro: &[[f32; 3]],
    ) -> Option<([f32; 3], [f32; 3])> {
        if accel.is_empty() {
            return None;
        }

        // Estimate gravity as mean of acceleration (assuming motion is small)
        let mut gravity = [0.0_f32; 3];
        for acc in accel {
            gravity[0] += acc[0];
            gravity[1] += acc[1];
            gravity[2] += acc[2];
        }
        gravity[0] /= accel.len() as f32;
        gravity[1] /= accel.len() as f32;
        gravity[2] /= accel.len() as f32;

        // Normalize gravity to standard gravity (9.81 m/s²)
        let gravity_mag =
            (gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]).sqrt();
        if gravity_mag > 0.1 {
            gravity[0] = gravity[0] / gravity_mag * 9.81;
            gravity[1] = gravity[1] / gravity_mag * 9.81;
            gravity[2] = gravity[2] / gravity_mag * 9.81;
        } else {
            gravity = [0.0, 0.0, -9.81];
        }

        let vibration = [
            gravity[0] - accel.get(0).map(|a| a[0]).unwrap_or(0.0),
            gravity[1] - accel.get(0).map(|a| a[1]).unwrap_or(0.0),
            gravity[2] - accel.get(0).map(|a| a[2]).unwrap_or(0.0),
        ];

        Some((gravity, vibration))
    }

    /// Get the current trajectory (list of keyframe poses with timestamps)
    pub fn get_trajectory(&self) -> &Vec<(i64, Matrix4x4)> {
        &self.trajectory
    }

    /// Get preintegrated IMU measurements between last two keyframes
    pub fn get_imu_preintegration(&self) -> Option<&PreintegratedImu> {
        self.current_imu_preintegration.as_ref()
    }

    /// Get current velocity estimate
    pub fn get_velocity(&self) -> Option<Vector3> {
        if self.velocity_estimator.is_initialized() {
            let v = self.velocity_estimator.get_velocity();
            Some(Vector3::new(v.x as Float, v.y as Float, v.z as Float))
        } else {
            None
        }
    }

    /// Get current IMU-camera extrinsic calibration
    pub fn get_extrinsic_calibration(&self) -> Matrix4x4 {
        self.extrinsic_calibrator.get_extrinsics()
    }

    /// Get number of IMU measurements processed
    pub fn imu_measurement_count(&self) -> usize {
        self.imu_measurement_count
    }

    /// Get IMU motion prior for optimization
    ///
    /// Returns the preintegrated IMU measurements between the last two keyframes,
    /// useful for adding IMU constraints to bundle adjustment.
    pub fn get_imu_motion_prior(&self) -> Option<ImuMotionPrior> {
        // Check if we have valid preintegrated measurements
        if !self.imu_preintegrator.is_valid() {
            log::trace!("[Estimator] No valid IMU preintegration for prior");
            return None;
        }

        // Get last keyframe pose from sliding window
        let keyframe_poses = self.sliding_window.get_keyframe_poses();
        let last_keyframe_pose = keyframe_poses.last()?;

        // Get current velocity
        let velocity = if self.velocity_estimator_initialized {
            self.velocity_estimator.get_velocity()
        } else {
            self.current_velocity
        };

        // Use the preintegrator's method to create the prior with proper config
        self.imu_preintegrator
            .create_motion_prior(*last_keyframe_pose, velocity)
    }

    /// Reset IMU-aided keyframe selector (e.g., after loop closure)
    pub fn reset_keyframe_selector(&mut self) {
        self.keyframe_selector.reset();
    }

    /// Get IMU measurement rate (for monitoring)
    pub fn get_imu_rate(&self) -> f64 {
        if self.imu_measurement_count > 0 && self.frame_id_counter > 0 {
            self.imu_measurement_count as f64 / self.frame_id_counter as f64
        } else {
            0.0
        }
    }

    /// Update camera intrinsics from online refiner if available
    fn update_intrinsics_from_refiner(&mut self) {
        if let Some(refiner) = &self.intrinsics_refiner {
            if refiner.is_converged() {
                debug_log!(
                    "[Estimator] Intrinsics converged after {} observations",
                    refiner.total_observations()
                );
            }
        }
    }

    /// Update camera extrinsics from online extrinsic calibrator if available
    ///
    /// Called periodically to apply calibrated IMU-to-camera extrinsics.
    /// The calibrator accumulates pose measurements and refines T_BC.
    fn update_extrinsics_from_calibrator(&mut self) {
        let measurement_count = self.extrinsic_calibrator.measurement_count();

        // Only apply after collecting enough measurements
        if measurement_count < 100 {
            return;
        }

        // Run calibration periodically (every 100 measurements after initial collection)
        if measurement_count % 100 == 0 {
            let _error = self.extrinsic_calibrator.calibrate_iteration();
            debug_log!(
                "[Estimator] IMU extrinsic calibration iteration {}, error: {:.6} rad",
                self.extrinsic_calibrator.iterations(),
                _error
            );
        }

        // Apply calibrated extrinsics periodically
        if self.extrinsic_calibrator.iterations() > 0 && measurement_count % 500 == 0 {
            let calibrated_T_BC = self.extrinsic_calibrator.get_extrinsics();

            // Update the stored extrinsics
            self.T_B_Cl = calibrated_T_BC;

            // Also update in all frame states if window has frames
            for frame in self.sliding_window.keyframes_mut() {
                frame.state.T_B_Cl = calibrated_T_BC;
            }

            debug_log!(
                "[Estimator] Applied calibrated extrinsics: T_B_Cl updated (iterations: {})",
                self.extrinsic_calibrator.iterations()
            );
        }
    }

    /// Add observations to intrinsics refiner for self-calibration
    fn add_intrinsics_observations(&mut self, frame: &Frame) {
        if let Some(ref mut refiner) = self.intrinsics_refiner {
            let observations: Vec<CalibrationObservation> = frame
                .left_features
                .iter()
                .filter(|f| f.undistorted_coord[0] >= 0.0 && f.undistorted_coord[1] >= 0.0)
                .map(|f| CalibrationObservation {
                    observation_2d: na::Vector2::new(
                        f.undistorted_coord[0] as Float,
                        f.undistorted_coord[1] as Float,
                    ),
                    ..Default::default()
                })
                .collect();
            if !observations.is_empty() {
                refiner.update(&observations);
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use crate::datasets::config::Config;
    use serde_yaml;

    fn create_test_config() -> Config {
        // Create a minimal config for testing
        let yaml = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
    grid_size: 10
    max_features_per_grid: 50
    optical_flow_max_iterations: 30
    optical_flow_convergence_threshold: 0.01
optimization:
    bundle_adjustment_max_iterations: 10
    pnp_max_iterations: 5
"#;
        serde_yaml::from_str(yaml).unwrap()
    }

    #[test]
    fn test_estimator_creation() {
        let config = create_test_config();
        let estimator = Estimator::new(config, None);
        assert_eq!(estimator.frame_count(), 0);
    }

    #[test]
    fn test_camera_model_creation() {
        let config = create_test_config();
        let (_left_cam, _right_cam) = crate::datasets::create_camera_models_from_config(&config);
        // Cameras are created successfully if no panic
    }

    #[test]
    fn test_process_frame_basic() {
        let config = create_test_config();
        let mut estimator = Estimator::new(config, None);
        // Increase timeout to allow for slower test environments
        estimator.set_max_frame_processing_time(std::time::Duration::from_secs(10));

        // Create dummy image data
        let left_image = vec![128u8; 640 * 480];
        let right_image = vec![128u8; 640 * 480];
        let timestamp_ns = 1000000000; // 1 second

        // This should not error
        estimator
            .process_frame(&left_image, &right_image, timestamp_ns, None)
            .expect("process_frame should succeed with synthetic input");
    }

    #[test]
    fn test_frame_creation() {
        let config = create_test_config();
        let (left_cam, right_cam) = crate::datasets::create_camera_models_from_config(&config);
        let t_b_cl = na::Matrix4::identity();
        let t_b_cr = na::Matrix4::from_row_slice(&[
            1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ]);

        let frame = Frame::from_stereo_images(1000000000, 0, left_cam, right_cam, t_b_cl, t_b_cr);

        assert_eq!(frame.frame_id, 0);
        assert_eq!(frame.timestamp_ns, 1000000000);
    }
}
