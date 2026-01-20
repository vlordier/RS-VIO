use super::state::Estimator;
use crate::datasets::ImuData;
use crate::debug_log;
use crate::estimator::Frame;
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
                ((denoise_weight as crate::types::Float) * (f0_confidence as crate::types::Float)).clamp(0.0, 1.0);

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
                    [last[0] as crate::types::Float, last[1] as crate::types::Float, last[2] as crate::types::Float]
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
                .map(|f| (f.pixel_coord[0] as crate::types::Float, f.pixel_coord[1] as crate::types::Float))
                .collect();

            let right_coords: Vec<(crate::types::Float, crate::types::Float)> = current_frame
                .right_features
                .iter()
                .map(|f| (f.pixel_coord[0] as crate::types::Float, f.pixel_coord[1] as crate::types::Float))
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
                    let euler: (crate::types::Float, crate::types::Float, crate::types::Float) = rotmat.euler_angles();
                    let rotation_norm = (euler.0.abs() + euler.1.abs() + euler.2.abs()).abs();

                    // Keyframe if either visual or IMU criteria met
                    let translation_threshold =
                        crate::fl!(self.config.keyframe_management.translation_threshold);
                    let rotation_threshold =
                        crate::fl!(self.config.keyframe_management.rotation_threshold);
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
}
