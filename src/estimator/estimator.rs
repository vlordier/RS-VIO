use crate::datasets::config::Config;
use crate::datasets::CameraModelType;
use crate::datasets::ImuData;
use crate::estimator::constant_velocity_model::{ConstantVelocityConfig, ConstantVelocityModel};
use crate::estimator::sliding_window::SlidingWindow;
use crate::estimator::Frame;
use crate::feature_tracker::StereoPatchTracker;
use crate::imu::ExtrinsicCalibrator;
use crate::imu::ImuAidedKeyframeSelector;
use crate::imu::ImuBiasEstimator;
use crate::imu::ImuConfig;
use crate::imu::ImuMotionPredictor;
use crate::imu::ImuMotionPrior;
use crate::imu::ImuPreintegrator;
use crate::imu::PreintegratedImu;
use crate::imu::VelocityEstimator;
use crate::types::{Float, Matrix4x4, Vector3};
use crate::viewers::Viewer;
use crate::VIOError;
use anyhow::Result;
use image::GrayImage;
use nalgebra as na;
use std::time::{Duration, Instant};

/// Placeholder estimator implementation.
/// Currently mimics the control flow and logging structure of the C++ Estimator::process_frame,
/// but uses dummy values for tracking, optimization, and mapping.
pub struct Estimator<'a> {
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
    /// Optional viewer used for visualization; outlives the estimator.
    viewer: Option<&'a mut dyn Viewer>,
    /// Left camera model with intrinsics and distortion.
    left_cam: CameraModelType,
    /// Right camera model with intrinsics and distortion.
    right_cam: CameraModelType,
    // Transformation from body to left camera
    T_B_Cl: Matrix4x4,
    // Transformation from body to right camera
    T_B_Cr: Matrix4x4,
    // Full trajectory of keyframes
    trajectory: Vec<Matrix4x4>,
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
    current_velocity: na::Vector3<f64>,
    // Whether velocity estimator has been initialized
    velocity_estimator_initialized: bool,
    // IMU bias estimator for initialization
    bias_estimator: ImuBiasEstimator,
    // Whether system is in initialization phase (collecting IMU for bias estimation)
    is_initializing: bool,
    // Constant velocity motion model (fallback when IMU unavailable)
    #[allow(dead_code)]
    cv_motion_model: ConstantVelocityModel,
}

impl<'a> Estimator<'a> {
    #![allow(non_snake_case)]

    /// Create a new estimator configured with camera intrinsics and distortion
    /// loaded from the YAML configuration.
    ///
    /// The `viewer` reference must outlive the estimator.
    pub fn new(config: Config, viewer: Option<&'a mut dyn Viewer>) -> Self {
        Self::new_with_cameras(config, viewer, None, None)
    }

    /// Create a new estimator with optional camera models.
    /// If camera models are provided, they will be used; otherwise, they will be created from config.
    ///
    /// The `viewer` reference must outlive the estimator.
    pub fn new_with_cameras(
        config: Config,
        viewer: Option<&'a mut dyn Viewer>,
        left_cam: Option<CameraModelType>,
        right_cam: Option<CameraModelType>,
    ) -> Self {
        // Use provided cameras or create from config
        let (left_cam, right_cam) = match (left_cam, right_cam) {
            (Some(l), Some(r)) => (l, r),
            _ => crate::datasets::create_camera_models_from_config(&config),
        };

        // Compute the transformation from left to right (T_C1_C0) as in compute_stereo.
        let T_B_Cl = na::Matrix4::from_row_slice(&config.camera.T_B_Cl);
        let T_B_Cr = na::Matrix4::from_row_slice(&config.camera.T_B_Cr);
        let keyframe_window_size = config.keyframe_management.keyframe_window_size as usize;
        let grid_size = config.feature_detection.grid_cols;
        let optical_flow_max_iterations = config.feature_detection.optical_flow_max_iterations;
        let optical_flow_convergence_threshold =
            config.feature_detection.optical_flow_convergence_threshold;
        let processing_timeout_ms = config.keyframe_management.processing_timeout_ms;

        // Initialize IMU components
        let imu_config = ImuConfig::default();
        Estimator {
            frame_id_counter: 0,
            frames_since_last_keyframe: 0,
            enable_debug_output: true,
            config,
            stereo_patch_tracker: StereoPatchTracker::<6>::new(
                grid_size,
                optical_flow_max_iterations,
                optical_flow_convergence_threshold,
            ),
            sliding_window: SlidingWindow::new(keyframe_window_size),
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
                config.keyframe_management.translation_threshold as f64,
                config.keyframe_management.rotation_threshold as f64,
            ),
            current_imu_preintegration: None,
            last_imu_timestamp: None,
            imu_measurement_count: 0,
            current_velocity: na::Vector3::zeros(),
            velocity_estimator_initialized: false,
            bias_estimator: ImuBiasEstimator::new(imu_config.clone()),
            is_initializing: true,
            cv_motion_model: ConstantVelocityModel::new(ConstantVelocityConfig::default()),
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
        let total_start_time = Instant::now();

        // New frame: update counters
        self.frame_id_counter += 1;
        self.frames_since_last_keyframe += 1;

        if self.enable_debug_output {
            log::debug!(
                "============================== Frame {} ==============================",
                self.frame_id_counter
            );
        }

        // Timing placeholders (warning: assigned before read in log::debug below)
        #[allow(unused_assignments)]
        let mut frame_creation_time_ms = 0.0f64;
        #[allow(unused_assignments)]
        let mut patch_tracking_time_ms = 0.0f64;
        let mut motion_tracking_time_ms = 0.0f64;
        let mut optimization_time_ms = 0.0f64;

        // Frame creation
        let frame_creation_start = Instant::now();

        // Create GrayImage objects directly from input slices (no clone needed for tracking)
        let img_w = self.config.camera.image_width;
        let img_h = self.config.camera.image_height;

        let left_img = match GrayImage::from_raw(img_w, img_h, left_image.to_vec()) {
            Some(img) => img,
            None => {
                log::error!(
                    "[Estimator] Failed to construct GrayImage for left camera ({}x{}, len={})",
                    img_w,
                    img_h,
                    left_image.len()
                );
                return Ok(());
            },
        };
        let right_img = match GrayImage::from_raw(img_w, img_h, right_image.to_vec()) {
            Some(img) => img,
            None => {
                log::error!(
                    "[Estimator] Failed to construct GrayImage for right camera ({}x{}, len={})",
                    img_w,
                    img_h,
                    right_image.len()
                );
                return Ok(());
            },
        };

        /*
        // For debugging with TUM-VI: undistort the images with EUCM and save the result
        let eucm = match &self.left_cam {
            crate::datasets::CameraModelType::EUCM(cam) => cam,
            _ => panic!("left_cam is not an EUCM instance"),
        };
        let model1 = GenericModel::EUCM(*eucm);
        let p = model1.estimate_new_camera_matrix_for_undistort(0.0, Some((1024, 1024)));
        let (xmap, ymap) = model1.init_undistort_map(&p, (1024, 1024), None);
        let img_l8 = DynamicImage::ImageLuma8(right_img.clone());
        let remaped = camera_intrinsic_model::remap(&img_l8, &xmap, &ymap);
        remaped.save("remaped0.png").unwrap();
        */

        // Create frame (images are not stored, only features will be added)
        let mut current_frame = Frame::from_stereo_images(
            timestamp_ns,
            self.frame_id_counter as i32,
            self.left_cam.clone(),
            self.right_cam.clone(),
            self.T_B_Cl,
            self.T_B_Cr,
        );

        // Process IMU measurements
        if let Some(imu) = imu_data {
            // Attach IMU measurements to frame
            current_frame.imu_from_last_frame = imu.to_vec();

            // During initialization, collect IMU samples for bias estimation
            if self.is_initializing {
                for imu_sample in imu {
                    self.bias_estimator.add_sample(imu_sample, true);
                }

                // Check if bias estimation is complete (need enough samples)
                if self.bias_estimator.sample_count() >= 100 {
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

            // Propagate IMU preintegrator with bias-corrected measurements
            for imu_sample in imu {
                if let Some(last_ts) = self.last_imu_timestamp {
                    let dt = (imu_sample.timestamp - last_ts) as f64 / 1e9;
                    if dt > 0.0 {
                        // Apply bias correction if available
                        if self.bias_estimator.is_initialized {
                            let gyro_corrected = self.bias_estimator.correct_gyro(imu_sample);
                            let accel_corrected = self.bias_estimator.correct_accel(imu_sample);
                            self.imu_preintegrator.propagate_corrected(
                                gyro_corrected,
                                accel_corrected,
                                dt,
                            );
                        } else {
                            self.imu_preintegrator.propagate(imu_sample, dt);
                        }
                    }
                }
                self.last_imu_timestamp = Some(imu_sample.timestamp);
                self.imu_measurement_count += 1;
            }

            // Use motion predictor for feature tracking
            let focal_length = self.config.camera.left_intrinsics[0] as f64;
            for feature in &mut current_frame.left_features {
                let (du, dv) = self.imu_motion_predictor.predict_feature_displacement(
                    imu,
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
            if !self.velocity_estimator_initialized && !imu.is_empty() {
                let initial_orientation = na::UnitQuaternion::identity();
                self.velocity_estimator
                    .initialize_from_imu(imu, &initial_orientation);
                self.velocity_estimator_initialized = true;
                log::debug!(
                    "[Estimator] Velocity estimator initialized with {} IMU samples",
                    imu.len()
                );
            }

            // Update velocity estimator
            self.velocity_estimator.update(imu, dt);

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
                        let error = self.extrinsic_calibrator.calibrate_iteration();
                        log::debug!(
                            "[Estimator] IMU extrinsic calibration error: {:.6} rad",
                            error
                        );
                    }
                }
            }
        }

        frame_creation_time_ms = frame_creation_start.elapsed().as_secs_f64() * 1000.0;

        // Patch tracking
        let tracking_start = Instant::now();
        self.stereo_patch_tracker
            .process_frame(&left_img, &right_img, &mut current_frame);
        patch_tracking_time_ms = tracking_start.elapsed().as_secs_f64() * 1000.0;
        self.view_patch_tracking_results(&current_frame, &left_img, &right_img, img_w, img_h);

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
                    let euler: (f64, f64, f64) = rotmat.euler_angles();
                    let rotation_norm = (euler.0.abs() + euler.1.abs() + euler.2.abs()).abs();

                    // Keyframe if either visual or IMU criteria met
                    let translation_threshold =
                        self.config.keyframe_management.translation_threshold as f64;
                    let rotation_threshold =
                        self.config.keyframe_management.rotation_threshold as f64;
                    let visual_keyframe =
                        t_rel.norm() > translation_threshold || rotation_norm > rotation_threshold;

                    let is_keyframe = is_imu_keyframe || visual_keyframe;

                    if is_keyframe {
                        let reason = if is_imu_keyframe && !visual_keyframe {
                            format!("IMU-aided: {}", keyframe_reason)
                        } else if visual_keyframe {
                            let t_norm = t_rel.norm();
                            format!("Visual: trans={:.3}m, rot={:.3}rad", t_norm, rotation_norm)
                        } else {
                            keyframe_reason
                        };
                        log::debug!("[Estimator] Keyframe triggered: {}", reason);

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
            motion_tracking_time_ms = motion_tracking_start.elapsed().as_secs_f64() * 1000.0;
        } else {
            log::debug!("[Estimator] Sliding window is not full, skipping motion tracking");
        }

        // View map points and keyframe poses
        // Bundle adjustment
        if current_frame.is_keyframe {
            let optimization_start = Instant::now();
            self.sliding_window.add_frame(current_frame);
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
            if let Err(e) = self
                .sliding_window
                .optimize_with_imu(imu_prior, imu_weights, imu_huber_delta)
            {
                log::error!("[Estimator] Bundle adjustment optimization failed: {:?}", e);
                // Continue execution even if optimization fails
            }
            optimization_time_ms = optimization_start.elapsed().as_secs_f64() * 1000.0;
            self.view_optimization_results();
        }

        // Final timing summary
        let total_duration_ms = total_start_time.elapsed().as_secs_f64() * 1000.0;
        log::debug!(
            "[Timing] frame_creation={:.3} ms, patch_tracking={:.3} ms, motion_tracking={:.3} ms, optimization={:.3} ms, total={:.3} ms",
            frame_creation_time_ms,
            patch_tracking_time_ms,
            motion_tracking_time_ms,
            optimization_time_ms,
            total_duration_ms
        );

        Ok(())
    }

    /// Helper: set the current frame index on the attached viewer, if any.
    pub fn set_viewer_frame(&mut self, frame_id: i64) {
        if let Some(v) = &mut self.viewer {
            v.set_frame(frame_id);
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
    fn view_optimization_results(&mut self) {
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

            // History of keyframe poses
            #[allow(clippy::unwrap_used)] // Validated by sliding_window - keyframes exist
            let mat = *self.sliding_window.get_keyframe_poses().first().unwrap();
            self.trajectory.push(mat);

            // Display trajectory as a continuous 3D path
            v.log_trajectory(&self.trajectory, "trajectory/path");
            // log::info!("[Estimator] System position: {:?}, {:?}, {:?}", mat[0][3], mat[1][3], mat[2][3]);
        }
    }

    /// Get the current trajectory (list of keyframe poses)
    pub fn get_trajectory(&self) -> &Vec<Matrix4x4> {
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
        let preint = self.imu_preintegrator.get();

        // Get last keyframe pose from sliding window
        let keyframe_poses = self.sliding_window.get_keyframe_poses();
        let last_keyframe_pose = keyframe_poses.last()?;

        // Get current velocity
        let velocity = if self.velocity_estimator.is_initialized() {
            self.velocity_estimator.get_velocity()
        } else {
            self.current_velocity
        };

        let gravity = na::Vector3::new(0.0, 0.0, -9.81);

        Some(ImuMotionPrior::from_preintegration(
            preint,
            *last_keyframe_pose,
            velocity,
            gravity,
        ))
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
}

    /// Get IMU measurement rate (for monitoring)
    pub fn get_imu_rate(&self) -> f64 {
        if self.imu_measurement_count > 0 && self.frame_id_counter > 0 {
            self.imu_measurement_count as f64 / self.frame_id_counter as f64
        } else {
            0.0
        }
    }
}
