use super::state::Estimator;
use crate::datasets::ImuData;
use crate::debug_log;
use crate::estimator::Frame;
use crate::imu::ImuVector3; // Add this line for ImuVector3
use crate::imu_fl; // Add this line for imu_fl! macro
use crate::{Result, VIOError};
use image::GrayImage;
use nalgebra as na;
use std::time::Instant;

impl Estimator {
    /// Process a single stereo frame.
    pub fn process_frame(
        &mut self,
        left_image: &[u8],
        right_image: &[u8],
        timestamp_ns: i64,
        imu_data: Option<&[ImuData]>,
    ) -> Result<()> {
        // Fast-mode gating via environment variable for high-ROI speedups
        let fast_mode = match std::env::var("RS_VIO_FAST") {
            Ok(val) => val == "1" || val.eq_ignore_ascii_case("true"),
            Err(_) => false,
        };
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
        let mut _processed_gyro: Option<Vec<[f32; 3]>> = None;

        // Process IMU measurements
        if let Some(imu) = imu_data {
            // Check if IMU is enabled in debug config
            if self.config.debug.use_imu {
                let imu_len = imu.len();
                let mut filtered_imu: Vec<ImuData> = Vec::with_capacity(imu_len);
                let mut accel_filtered: Vec<[f32; 3]> = Vec::with_capacity(imu_len);
                let mut gyro_filtered: Vec<[f32; 3]> = Vec::with_capacity(imu_len);

                let bias_initialized = self.imu_processor.bias_estimator.is_initialized;

                for imu_sample in imu.iter() {
                    if self.imu_processor.is_initializing {
                        self.imu_processor
                            .bias_estimator
                            .add_sample(imu_sample, true);
                    }

                    let (gyro_corrected, accel_corrected) = if bias_initialized {
                        (
                            self.imu_processor.bias_estimator.correct_gyro(imu_sample),
                            self.imu_processor.bias_estimator.correct_accel(imu_sample),
                        )
                    } else {
                        (
                            ImuVector3::new(
                                imu_fl!(imu_sample.gyro[0]),
                                imu_fl!(imu_sample.gyro[1]),
                                imu_fl!(imu_sample.gyro[2]),
                            ),
                            ImuVector3::new(
                                imu_fl!(imu_sample.accel[0]),
                                imu_fl!(imu_sample.accel[1]),
                                imu_fl!(imu_sample.accel[2]),
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

                    let accel_denoised =
                        self.imu_processor.denoise_filter.process_accel(&accel_f32);
                    let gyro_denoised = self.imu_processor.denoise_filter.process_gyro(&gyro_f32);

                    // Get updated weight after processing
                    let current_weight = self.imu_processor.denoise_filter.weight_scale;

                    // Process acceleration through higher-order filter for jerk/snap and f0 analysis
                    let higher_order_output = self
                        .imu_processor
                        .higher_order_filter
                        .process_accel(accel_denoised);

                    // Use f0 confidence to further weight the measurement
                    // Combine denoise weight with f0 confidence
                    let f0_weighted = if self
                        .imu_processor
                        .higher_order_filter
                        .config
                        .enable_f0_weighting
                    {
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

                    // Direct conversion to ImuFloat for preintegration
                    let accel_vec = ImuVector3::new(
                        imu_fl!(accel_scaled[0]),
                        imu_fl!(accel_scaled[1]),
                        imu_fl!(accel_scaled[2]),
                    );
                    let gyro_vec = ImuVector3::new(
                        imu_fl!(gyro_scaled[0]),
                        imu_fl!(gyro_scaled[1]),
                        imu_fl!(gyro_scaled[2]),
                    );

                    if let Some(last_ts) = self.imu_processor.last_timestamp {
                        let dt = (imu_sample.timestamp - last_ts) as f64 / 1e9;
                        if dt > 0.0 {
                            self.imu_processor
                                .preintegrator
                                .propagate_corrected(gyro_vec, accel_vec, dt);
                        }
                    }
                    self.imu_processor.last_timestamp = Some(imu_sample.timestamp);
                    self.imu_processor.stats.total_measurements += 1;

                    filtered_imu.push(ImuData {
                        timestamp: imu_sample.timestamp,
                        gyro: [gyro_vec[0] as f64, gyro_vec[1] as f64, gyro_vec[2] as f64],
                        accel: [
                            accel_vec[0] as f64,
                            accel_vec[1] as f64,
                            accel_vec[2] as f64,
                        ],
                    });

                    if self.imu_processor.is_initializing
                        && self.imu_processor.bias_estimator.sample_count() >= 100
                    {
                        self.imu_processor.is_initializing = false;
                        log::info!(
                            "[Estimator] IMU initialization complete. Gyro bias: [{:.4}, {:.4}, {:.4}] rad/s, Accel bias: [{:.4}, {:.4}, {:.4}] m/s²",
                            self.imu_processor.bias_estimator.gyro_bias[0],
                            self.imu_processor.bias_estimator.gyro_bias[1],
                            self.imu_processor.bias_estimator.gyro_bias[2],
                            self.imu_processor.bias_estimator.accel_bias[0],
                            self.imu_processor.bias_estimator.accel_bias[1],
                            self.imu_processor.bias_estimator.accel_bias[2]
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
                _processed_gyro = Some(gyro_filtered);

                // Use motion predictor for feature tracking
                let focal_length = self.config.camera.left_intrinsics[0] as f64;
                for feature in &mut current_frame.left_features {
                    let (du, dv) = self
                        .imu_processor
                        .motion_predictor
                        .predict_feature_displacement(
                            &filtered_imu,
                            (feature.pixel_coord[0] as f64, feature.pixel_coord[1] as f64),
                            focal_length,
                        );
                    // Apply predicted displacement as initial guess for optical flow
                    feature.pixel_coord[0] = (feature.pixel_coord[0] as f64 + du) as f32;
                    feature.pixel_coord[1] = (feature.pixel_coord[1] as f64 + dv) as f32;
                }

                // Update velocity estimator
                let dt = if let Some(last_ts) = self.imu_processor.last_timestamp {
                    (timestamp_ns - last_ts) as f64 / 1e9
                } else {
                    0.01
                };

                // Initialize velocity estimator on first IMU batch
                if !self.imu_processor.velocity_estimator_initialized && !filtered_imu.is_empty() {
                    let initial_orientation = na::UnitQuaternion::identity();
                    self.imu_processor
                        .velocity_estimator
                        .initialize_from_imu(&filtered_imu, &initial_orientation);
                    self.imu_processor.velocity_estimator_initialized = true;
                    if should_log {
                        debug_log!(
                            "[Estimator] Velocity estimator initialized with {} IMU samples",
                            filtered_imu.len()
                        );
                    }
                }

                // Update velocity estimator
                self.imu_processor
                    .velocity_estimator
                    .update(&filtered_imu, dt);

                // Accumulate for extrinsic calibration
                if self.backend.sliding_window.is_full() {
                    let keyframe_poses = self.backend.sliding_window.get_keyframe_poses();
                    if let Some(T_W_B) = keyframe_poses.last() {
                        let T_W_B_copy = *T_W_B;
                        let rotmat = na::Rotation3::from_matrix_unchecked(
                            T_W_B_copy.fixed_view::<3, 3>(0, 0).into_owned(),
                        );
                        let R_W_B = na::UnitQuaternion::from_rotation_matrix(&rotmat);
                        self.imu_processor
                            .extrinsic_calibrator
                            .add_measurement(&T_W_B_copy, R_W_B);

                        // NOTE: Periodic calibration disabled due to NaN instabilities in edge cases
                        // Can be re-enabled once calibrator is stabilized for all motion profiles
                    }
                }
            } else if should_log {
                debug_log!("[Estimator] IMU disabled via debug config");
            }
        }

        _frame_creation_time_ms = frame_creation_start.elapsed().as_secs_f64() * 1000.0;

        // Note: IMU visualization disabled to reduce frame processing overhead
        // Re-enable by uncommenting the view_imu_results call if needed for debugging

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
            self.frontend
                .stereo_patch_tracker
                .set_camera_intrinsics(fx, fy, cx, cy);

            if self.config.debug.use_imu {
                self.frontend
                    .stereo_patch_tracker
                    .set_imu_rotation_hint(imu, (fx, fy, cx, cy));

                // Provide velocity estimate for improved feature tracking
                if let Some(velocity) = self.get_velocity() {
                    self.frontend.stereo_patch_tracker.set_velocity_hint([
                        velocity.x as f32,
                        velocity.y as f32,
                        velocity.z as f32,
                    ]);
                }
            }
        }

        let tracking_start = Instant::now();
        self.frontend
            .stereo_patch_tracker
            .process_frame(&left_img, &right_img, &mut current_frame);
        _patch_tracking_time_ms = tracking_start.elapsed().as_secs_f64() * 1000.0;
        self.view_patch_tracking_results(&current_frame, &left_img, &right_img, img_w, img_h);

        // Stereo super-resolution refinement with IMU confidence weighting
        // Skip in fast mode to reduce per-frame compute cost
        if !fast_mode
            && !current_frame.left_features.is_empty()
            && !current_frame.right_features.is_empty()
        {
            let _superres_start = Instant::now();

            // Compute IMU confidence metric from denoise and higher-order filters
            let denoise_weight = self.imu_processor.denoise_filter.weight_scale;
            let f0_confidence = self.imu_processor.higher_order_filter.f0_confidence;
            let imu_confidence = ((denoise_weight as crate::types::Float)
                * (f0_confidence as crate::types::Float))
                .clamp(0.0, 1.0);

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
                    [
                        last[0] as crate::types::Float,
                        last[1] as crate::types::Float,
                        last[2] as crate::types::Float,
                    ]
                } else {
                    [0.0 as crate::types::Float, 0.0, 0.0]
                }
            } else {
                [0.0 as crate::types::Float, 0.0, 0.0]
            };

            // Collect feature coordinates for refinement
            let left_coords: Vec<(crate::types::Float, crate::types::Float)> = current_frame
                .left_features
                .iter()
                .map(|f| {
                    (
                        f.pixel_coord[0] as crate::types::Float,
                        f.pixel_coord[1] as crate::types::Float,
                    )
                })
                .collect();

            let right_coords: Vec<(crate::types::Float, crate::types::Float)> = current_frame
                .right_features
                .iter()
                .map(|f| {
                    (
                        f.pixel_coord[0] as crate::types::Float,
                        f.pixel_coord[1] as crate::types::Float,
                    )
                })
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

        // Optionally capture the left image for fusion consumers; default is off to avoid extra copies.
        if self.config.debug.capture_left_image_for_fusion {
            let left_image_copy = left_img.as_raw().clone();
            current_frame.set_left_image_plane(left_image_copy, img_w, img_h);
        }

        // Optional: Apply multi-frame fusion to enhance feature confidence
        // Skip in fast mode to reduce extra copying and compute
        if !fast_mode {
            if let Some(fusion_strat) = self.fusion_strategy.as_mut() {
            // Add current frame to buffer (Arc-wrapped to avoid expensive clones)
            self.fusion_frame_buffer
                .push_back(std::sync::Arc::new(current_frame.clone()));

            // Keep buffer within capacity (default 5 frames)
            if self.fusion_frame_buffer.len() > 5 {
                self.fusion_frame_buffer.pop_front();
            }

            // Apply fusion when we have at least 2 frames
            if self.fusion_frame_buffer.len() >= 2 {
                let frames_vec: Vec<_> = self
                    .fusion_frame_buffer
                    .iter()
                    .map(|arc_frame| (**arc_frame).clone())
                    .collect();

                if let Ok(fused) = fusion_strat.fuse(&frames_vec) {
                    // Apply per-feature confidence from fusion to current frame
                    let feature_count = current_frame.left_features.len();
                    if fused.feature_confidence.len() >= feature_count {
                        for (feat, conf) in current_frame
                            .left_features
                            .iter_mut()
                            .zip(fused.feature_confidence.iter())
                        {
                            // Blend fusion confidence with existing tracking confidence
                            feat.quality.confidence =
                                ((feat.quality.confidence as crate::types::Float + conf) / 2.0)
                                    as f32;
                        }
                        if should_log {
                            debug_log!(
                                "[Estimator] Fusion: enhanced {} features, SNR improvement: {:?}dB",
                                feature_count,
                                fused.metrics.snr_improvement_db
                            );
                        }
                    }
                }
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

        // Motion tracking - only need a few keyframes to start
        let num_kfs = self.backend.sliding_window.len();
        if self.backend.sliding_window.has_enough_for_tracking() {
            log::debug!(
                "[Estimator] Running motion tracking with {} keyframes",
                num_kfs
            );
            let motion_tracking_start = Instant::now();
            let motion_tracking_result = self.backend.sliding_window.track_motion(&current_frame);
            match motion_tracking_result {
                Ok(Some(T_W_B)) => {
                    log::info!(
                        "[Estimator] Frame {}: Motion tracking SUCCESS, updating pose",
                        self.frame_count
                    );
                    // Apply the optimized pose to the current frame
                    current_frame.state.T_W_B = T_W_B;

                    // IMU-aided keyframe selection
                    let keyframe_poses = self.backend.sliding_window.get_keyframe_poses();
                    let T_W_B_last_kf: &na::Matrix4<f64> = match keyframe_poses.last() {
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
                        .imu_processor
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
                    let T_rel: na::Matrix4<f64> = T_W_B * T_W_B_last_kf_inv;
                    let t_rel = T_rel.fixed_view::<3, 1>(0, 3).into_owned();
                    let R_rel = T_rel.fixed_view::<3, 3>(0, 0).into_owned();
                    let rotmat = na::Rotation3::from_matrix_unchecked(R_rel);
                    let euler: (
                        crate::types::Float,
                        crate::types::Float,
                        crate::types::Float,
                    ) = rotmat.euler_angles();
                    let rotation_norm = (euler.0.abs() + euler.1.abs() + euler.2.abs()).abs();
                    let t_norm = t_rel.norm();

                    // Keyframe if either visual or IMU criteria met
                    let translation_threshold =
                        crate::fl!(self.config.keyframe_management.translation_threshold);
                    let rotation_threshold =
                        crate::fl!(self.config.keyframe_management.rotation_threshold);
                    let visual_keyframe =
                        t_norm > translation_threshold || rotation_norm > rotation_threshold;

                    // Bootstrap: force keyframes for initialization even with low motion
                    let num_kfs = self.backend.sliding_window.len();
                    let bootstrap_keyframe = num_kfs < 5; // Force first 5 keyframes

                    let is_keyframe = is_imu_keyframe || visual_keyframe || bootstrap_keyframe;

                    let reason = if bootstrap_keyframe && !is_imu_keyframe && !visual_keyframe {
                        format!("Bootstrap: {} keyframes", num_kfs)
                    } else if is_imu_keyframe && !visual_keyframe {
                        format!("IMU-aided: {}", keyframe_reason)
                    } else if visual_keyframe {
                        format!("Visual: trans={:.3}m, rot={:.3}rad", t_norm, rotation_norm)
                    } else {
                        keyframe_reason.clone()
                    };

                    log::info!(
                        "[Estimator] KF decision: is_kf={}, imu_kf={}, visual_kf={}, t_norm={:.3}m, rot_norm={:.3}rad, reason={}",
                        is_keyframe,
                        is_imu_keyframe,
                        visual_keyframe,
                        t_norm,
                        rotation_norm,
                        reason
                    );

                    if is_keyframe {
                        debug_log!("[Estimator] Keyframe triggered: {}", reason);
                        current_frame.is_keyframe = true;
                        log::debug!(
                            "[Estimator] Frame {} set as KEYFRAME: {}",
                            self.frame_count,
                            reason
                        );
                    } else {
                        current_frame.is_keyframe = false;
                        if self.frame_count % 50 == 0 {
                            log::debug!(
                                "[Estimator] Frame {} NOT keyframe: t={:.3}m, r={:.3}rad (need >{:.3}m or >{:.3}rad)",
                                self.frame_count,
                                t_norm,
                                rotation_norm,
                                translation_threshold,
                                rotation_threshold
                            );
                        }
                    }
                    self.view_motion_tracking_results(&T_W_B);
                },
                Ok(None) => {
                    log::warn!(
                        "[Estimator] Frame {}: Motion tracking failed (optimization did not converge), pose not updated",
                        self.frame_count
                    );
                },
                Err(e) => {
                    log::error!(
                        "[Estimator] Frame {}: Motion tracking error: {:?}",
                        self.frame_count,
                        e
                    );
                },
            }
            _motion_tracking_time_ms = motion_tracking_start.elapsed().as_secs_f64() * 1000.0;
        } else if self.frame_count % 50 == 0 {
            log::debug!(
                "[Estimator] Frame {}: Motion tracking skipped - only {} keyframes (need 3 for tracking)",
                self.frame_count,
                num_kfs
            );
        }

        // View map points and keyframe poses
        // Bundle adjustment
        if current_frame.is_keyframe {
            log::info!(
                "[Estimator] Adding keyframe {}: left_features={}, right_features={}",
                self.frame_count,
                current_frame.left_features.len(),
                current_frame.right_features.len()
            );
            let optimization_start = Instant::now();
            // Save timestamp and pose before frame is moved
            let frame_timestamp = current_frame.timestamp_ns;
            let frame_pose = current_frame.state.T_W_B;
            let kf_id = self.frame_id_counter;

            // Detect loop closures for this keyframe
            let descriptor =
                self.create_keyframe_descriptor(kf_id, frame_timestamp, &current_frame, frame_pose);

            match self.loop_closure_detector.detect_loop_closure(
                kf_id,
                descriptor,
                &mut self.frame_workspace,
            ) {
                Ok(constraints) if !constraints.is_empty() => {
                    log::info!(
                        "[Estimator] Detected {} loop closure(s) for keyframe {}",
                        constraints.len(),
                        kf_id
                    );
                    self.backend
                        .sliding_window
                        .add_loop_closure_constraints(constraints.clone());
                    // Also add to global pose graph for full SLAM optimization
                    self.global_pose_graph
                        .add_loop_closure_constraints(constraints);
                },
                Ok(_) => {},
                Err(e) => {
                    log::warn!("[Estimator] Loop closure detection failed: {:?}", e);
                },
            }

            // Add keyframe to global pose graph
            self.global_pose_graph.add_keyframe_pose(&current_frame);

            self.backend.sliding_window.add_frame(current_frame.clone());

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
                self.imu_processor
                    .preintegrator
                    .create_tight_coupling_preintegration()
            } else {
                None
            };

            // Add IMU preintegration to sliding window BEFORE optimization
            // This enables tight-coupled IMU factors in the optimization
            if let Some(ref preint) = preintegration_for_storage {
                let prev_kf_idx = self.backend.sliding_window.len().saturating_sub(2);
                if prev_kf_idx < self.backend.sliding_window.len() {
                    self.backend.sliding_window.add_imu_preintegration(
                        prev_kf_idx,
                        prev_kf_idx + 1,
                        preint.clone(),
                    );
                }
            }

            // Early map initialization: triangulate features once we have 3+ keyframes
            // This enables PnP factors for motion tracking before full bundle adjustment
            self.backend.sliding_window.triangulate_early_map();

            // In fast mode, reduce bundle adjustment frequency to every 3rd keyframe
            let should_optimize_ba = if fast_mode {
                (self.backend.sliding_window.len() % 3) == 0
            } else {
                true
            };
            if should_optimize_ba {
                if let Err(e) = self.backend.sliding_window.optimize_with_imu(
                    imu_prior,
                    imu_weights,
                    imu_huber_delta,
                ) {
                    log::error!("[Estimator] Bundle adjustment optimization failed: {:?}", e);
                    // Continue execution even if optimization fails
                } else {
                    // Export teacher frame data after successful optimization
                    #[cfg(feature = "export-teacher")]
                    if self.config.enable_export {
                        if let Err(e) = self.export_teacher_frame(
                            left_image,
                            right_image,
                            img_w,
                            img_h,
                            timestamp_ns,
                        ) {
                            log::warn!("[Estimator] Failed to export teacher frame: {}", e);
                        }
                    }
                }
            } else if self.frame_count % 50 == 0 {
                log::debug!("[Estimator] Fast mode: skipping BA on this keyframe");
            }

            // Report created map points after optimization
            let mp_len = self.backend.sliding_window.map_points_len();
            log::info!(
                "[Estimator] Post-optimization: map_points={}, keyframes={}",
                mp_len,
                self.backend.sliding_window.len()
            );

            // Reset IMU preintegrator after optimization for next interval
            if self.config.optimization.imu_prior_enable {
                self.imu_processor.preintegrator.reset();
            }

            // Update camera intrinsics from online refiner after optimization
            self.update_intrinsics_from_refiner();

            // Update extrinsics from calibrator periodically
            self.update_extrinsics_from_calibrator();

            // Check if global pose graph should run optimization
            let (should_opt, reason) = self.global_pose_graph.should_optimize();
            if should_opt {
                log::info!("[Estimator] Triggering global optimization: {}", reason);
                match self.global_pose_graph.optimize() {
                    Ok(result) => {
                        log::info!(
                            "[Estimator] Global optimization completed: {:.1}ms, {} iterations",
                            result.optimization_time_ms,
                            result.iterations
                        );
                    },
                    Err(e) => {
                        log::warn!("[Estimator] Global optimization failed: {}", e);
                        // Continue execution even if global optimization fails
                    },
                }
            }

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

    /// Export teacher frame data to HDF5 file
    /// Called after successful bundle adjustment optimization
    #[cfg(feature = "export-teacher")]
    fn export_teacher_frame(
        &mut self,
        left_image_raw: &[u8],
        right_image_raw: &[u8],
        img_width: u32,
        img_height: u32,
        timestamp_ns: i64,
    ) -> Result<()> {
        use crate::export::TeacherFrame;
        use ndarray::Array2;

        // Reconstruct GrayImage objects from raw buffers
        let left_image = match image::GrayImage::from_raw(img_width, img_height, left_image_raw.to_vec()) {
            Some(img) => img,
            None => return Err(VIOError::Optimization("Failed to reconstruct left image".to_string())),
        };
        let right_image = match image::GrayImage::from_raw(img_width, img_height, right_image_raw.to_vec()) {
            Some(img) => img,
            None => return Err(VIOError::Optimization("Failed to reconstruct right image".to_string())),
        };

        // Use current frame timestamp as double precision seconds
        let timestamp_sec = timestamp_ns as f64 / 1_000_000_000.0;

        // Get camera pose - use identity if not available
        let pose = na::Isometry3::identity();

        // Optical flow grid (8x6)
        let flow_grid = self.extract_optical_flow_grid()?;

        // Extract depth map
        let depth_map = self.extract_depth_map().unwrap_or_else(|| {
            Array2::<f32>::from_elem((
                self.config.camera.image_height as usize,
                self.config.camera.image_width as usize,
            ), -1.0)
        });

        // Extract depth confidence
        let depth_confidence = self.extract_depth_confidence().unwrap_or_else(|| {
            Array2::<f32>::from_elem((
                self.config.camera.image_height as usize,
                self.config.camera.image_width as usize,
            ), 0.0)
        });

        // Compute reprojection errors
        let reprojection_errors = self.compute_reprojection_errors().unwrap_or_default();

        // Downscale images to 256x256 for student network
        let left_downscaled = crate::export::downscale_image(&left_image, 256);
        let right_downscaled = crate::export::downscale_image(&right_image, 256);

        // Get IMU preintegration if available
        let imu_preint = if self.config.optimization.imu_prior_enable {
            // Create 15D vector from IMU preintegrator state
            let preint_state = self.imu_processor.preintegrator.get();
            let p = preint_state.delta_position;
            let v = preint_state.delta_velocity;
            let q = preint_state.delta_rotation;
            
            // Convert rotation to axis-angle
            let axis_angle = q.scaled_axis();
            
            na::SVector::<f64, 15>::from([
                p.x as f64, p.y as f64, p.z as f64,                    // delta_p (3)
                v.x as f64, v.y as f64, v.z as f64,                    // delta_v (3)
                axis_angle.x as f64, axis_angle.y as f64, axis_angle.z as f64,  // delta_w (3)
                0.0, 0.0, 0.0,                    // bias_w (3) - placeholder
                0.0, 0.0, 0.0,                    // bias_a (3) - placeholder
            ])
        } else {
            na::SVector::<f64, 15>::zeros()
        };

        // Create TeacherFrame struct
        let teacher_frame = TeacherFrame {
            frame_id: self.frame_id_counter as usize,
            timestamp: timestamp_sec,
            left_image: left_image.clone(),
            right_image: right_image.clone(),
            left_downscaled,
            right_downscaled,
            imu_preintegration: imu_preint,
            imu_covariance: na::SVector::<f64, 15>::zeros(), // TODO: extract from IMU uncertainty
            imu_sample_count: 0, // TODO: track from IMU data
            flow_grid,
            flow_quality: 0.5, // TODO: compute from flow tracking
            pose_world_cam: pose,
            pose_covariance: na::Matrix6::identity(), // TODO: extract from BA
            velocity: na::Vector3::zeros(),  // TODO: extract from state
            previous_pose: pose,  // TODO: use previous frame pose
            previous_velocity: na::Vector3::zeros(),  // TODO: extract from velocity estimate
            match_quality: vec![0.5; 32], // TODO: extract from feature matcher
            time_since_keyframe: 1.0,  // TODO: track keyframe count
            depth_map,
            depth_confidence,
            reprojection_errors,
            mean_reprojection_error: 0.0, // TODO: compute mean
            ba_iterations: self.config.optimization.bundle_adjustment_max_iterations as usize,
            ba_converged: true, // Assuming converged since we're exporting
        };

        // Export to HDF5
        if let Err(e) = self.export_manager.export_frame(teacher_frame) {
            log::warn!("[Estimator] Failed to write HDF5 frame: {}", e);
        }

        Ok(())
    }

    /// Extract optical flow grid (8x6) from feature tracker state
    #[cfg(feature = "export-teacher")]
    fn extract_optical_flow_grid(
        &self,
    ) -> Result<Vec<crate::export::OpticalFlowPoint>> {
        // Extract sparse optical flow points from the current frame's feature data
        // For simplicity, create a coarse grid representation
        // For now, return empty vector as we don't have direct flow access in processor
        // This could be populated from stereo matching residuals or feature tracking state
        
        Ok(Vec::new())
    }

    /// Extract semi-dense depth map from triangulation
    #[cfg(feature = "export-teacher")]
    fn extract_depth_map(&self) -> Option<ndarray::Array2<f32>> {
        use ndarray::Array2;
        
        let img_h = self.config.camera.image_height as usize;
        let img_w = self.config.camera.image_width as usize;
        
        // Initialize depth map with invalid values (-1.0)
        let mut depth_map = Array2::<f32>::from_elem((img_h, img_w), -1.0f32);
        
        // Get camera intrinsics from config
        if self.config.camera.left_intrinsics.len() < 4 {
            return Some(depth_map); // Return default if intrinsics missing
        }
        
        let fx = self.config.camera.left_intrinsics[0] as f32;
        let fy = self.config.camera.left_intrinsics[1] as f32;
        let cx = self.config.camera.left_intrinsics[2] as f32;
        let cy = self.config.camera.left_intrinsics[3] as f32;
        
        // Get last keyframe for pose information
        // Get last keyframe for pose information
        // Note: We can't call keyframes_mut() since this function takes &self
        // Use the last pose from trajectory
        if let Some((_, last_pose)) = self.trajectory.last() {
            let T_W_B = *last_pose;
            // Use identity for camera pose (simplified for now)
            let T_B_Cl = na::Matrix4::identity();
            
            // Compute T_W_Cl = T_W_B * T_B_Cl
            let T_W_Cl = T_W_B * T_B_Cl;
            let T_Cl_W = na::Matrix4::from(T_W_Cl).try_inverse()
                .unwrap_or_else(na::Matrix4::identity);
            
            // Project each map point
            for (_id, point_xyz) in self.backend.sliding_window.map_points.iter() {
                // Create point in world frame (f64)
                let p_W = na::Vector4::new(point_xyz[0] as f64, point_xyz[1] as f64, point_xyz[2] as f64, 1.0);
                let p_Cl = T_Cl_W * p_W;
                
                if p_Cl.z > 0.1 { // Only positive depth
                    let x_pix = (fx * p_Cl.x as f32 / p_Cl.z as f32 + cx) as i32;
                    let y_pix = (fy * p_Cl.y as f32 / p_Cl.z as f32 + cy) as i32;
                    
                    if x_pix >= 0 && x_pix < img_w as i32 && y_pix >= 0 && y_pix < img_h as i32 {
                        let depth_val = p_Cl.z as f32;
                        depth_map[[y_pix as usize, x_pix as usize]] = depth_val;
                    }
                }
            }
        }
        
        Some(depth_map)
    }

    /// Extract depth confidence map
    #[cfg(feature = "export-teacher")]
    fn extract_depth_confidence(&self) -> Option<ndarray::Array2<f32>> {
        use ndarray::Array2;
        
        let img_h = self.config.camera.image_height as usize;
        let img_w = self.config.camera.image_width as usize;
        
        // Initialize confidence map with zeros
        let confidence_map = Array2::<f32>::from_elem((img_h, img_w), 0.0f32);
        
        // For simplicity, set confidence based on depth map validity
        // In practice, this could be based on observation count or residuals
        if let Some(depth_map) = self.extract_depth_map() {
            let mut result = confidence_map;
            // Set confidence 0.9 where depth is positive
            for (idx, &depth_val) in depth_map.iter().enumerate() {
                if depth_val > 0.0 {
                    let y = idx / img_w;
                    let x = idx % img_w;
                    result[[y, x]] = 0.9f32;
                }
            }
            return Some(result);
        }
        
        Some(confidence_map)
    }

    /// Compute reprojection errors after optimization
    #[cfg(feature = "export-teacher")]
    fn compute_reprojection_errors(&self) -> Option<Vec<f32>> {
        // Compute residuals for each map point observation
        // For now, return empty vector as we'd need access to BA residual cache
        // This would be computed from the optimization result
        
        Some(Vec::new())
    }
}
