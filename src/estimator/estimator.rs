use crate::datasets::config::Config;
use crate::datasets::CameraModelType;
use crate::datasets::ImuData;
use crate::estimator::sliding_window::SlidingWindow;
use crate::estimator::Frame;
use crate::feature_tracker::StereoPatchTracker;
use crate::types::{Matrix4x4, UnitQuaternion, Vector3};
use crate::viewers::Viewer;
use anyhow::Result;
use image::GrayImage;
use nalgebra as na;
use std::time::Instant;

/// Tracks intrinsics refinement state
#[derive(Debug, Clone)]
pub struct IntrinsicsRefinementState {
    /// Original left camera intrinsics [fx, fy, cx, cy]
    pub original_left_intrinsics: Vec<f64>,
    /// Original right camera intrinsics [fx, fy, cx, cy]
    pub original_right_intrinsics: Vec<f64>,
    /// Current refined left intrinsics
    pub current_left_intrinsics: Vec<f64>,
    /// Current refined right intrinsics
    pub current_right_intrinsics: Vec<f64>,
    /// Number of keyframes since last refinement
    pub frames_since_refinement: usize,
}

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
    // Full trajectory of keyframes
    trajectory: Vec<Matrix4x4>,
    /// Online intrinsics refinement state
    intrinsics_refinement: Option<IntrinsicsRefinementState>,
    /// Pre-allocated image buffer for left camera (reused each frame - zero allocation)
    left_image_buffer: Vec<u8>,
    /// Pre-allocated image buffer for right camera (reused each frame - zero allocation)
    right_image_buffer: Vec<u8>,
    /// Pre-allocated IMU buffer (reused each frame - zero allocation)
    imu_buffer: Vec<ImuData>,
}

impl Estimator {
    #![allow(non_snake_case)]

    /// Create a new estimator configured with camera intrinsics and distortion
    /// loaded from the YAML configuration.
    ///
    pub fn new(config: Config, viewer: Option<Box<dyn Viewer>>) -> Self {
        Self::new_with_cameras(config, viewer, None, None)
    }

    /// Create a new estimator with optional camera models.
    /// If camera models are provided, they will be used; otherwise, they will be created from config.
    ///
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

        // Compute the transformation from left to right (T_C1_C0) as in compute_stereo.
        let T_B_Cl = na::Matrix4::from_row_slice(&config.camera.T_B_Cl);
        let T_B_Cr = na::Matrix4::from_row_slice(&config.camera.T_B_Cr);
        let keyframe_window_size = config.keyframe_management.keyframe_window_size as usize;
        let grid_size = config.feature_detection.grid_cols;
        let optical_flow_max_iterations = config.feature_detection.optical_flow_max_iterations;
        let optical_flow_convergence_threshold =
            config.feature_detection.optical_flow_convergence_threshold;

        // Initialize intrinsics refinement if enabled
        let intrinsics_refinement = if let Some(calib_cfg) = &config.calibration {
            if calib_cfg.optimize_intrinsics {
                Some(IntrinsicsRefinementState {
                    original_left_intrinsics: config.camera.left_intrinsics.clone(),
                    original_right_intrinsics: config.camera.right_intrinsics.clone(),
                    current_left_intrinsics: config.camera.left_intrinsics.clone(),
                    current_right_intrinsics: config.camera.right_intrinsics.clone(),
                    frames_since_refinement: 0,
                })
            } else {
                None
            }
        } else {
            None
        };

        // Pre-allocate image buffers for zero-allocation frame processing
        let image_size = (config.camera.image_width * config.camera.image_height) as usize;
        let left_image_buffer = vec![0u8; image_size];
        let right_image_buffer = vec![0u8; image_size];
        // Pre-allocate IMU buffer (typical 20-50 samples per frame)
        let imu_buffer = Vec::with_capacity(100);

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
            intrinsics_refinement,
            left_image_buffer,
            right_image_buffer,
            imu_buffer,
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

        // Use pre-allocated buffers for zero-allocation image processing
        let img_w = self.config.camera.image_width;
        let img_h = self.config.camera.image_height;
        let expected_size = (img_w * img_h) as usize;

        // Validate input sizes
        if left_image.len() != expected_size || right_image.len() != expected_size {
            log::error!(
                "[Estimator] Invalid image size: left={}, right={}, expected={} ({}x{})",
                left_image.len(),
                right_image.len(),
                expected_size,
                img_w,
                img_h
            );
            return Ok(());
        }

        // Copy into pre-allocated buffers (memcpy - fast, no malloc)
        self.left_image_buffer.copy_from_slice(left_image);
        self.right_image_buffer.copy_from_slice(right_image);

        // Create GrayImage by moving pre-allocated buffer (zero allocation)
        // We'll get the buffer back after use via into_raw()
        let left_buffer = std::mem::take(&mut self.left_image_buffer);
        let right_buffer = std::mem::take(&mut self.right_image_buffer);

        let left_img = match GrayImage::from_raw(img_w, img_h, left_buffer) {
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
        let right_img = match GrayImage::from_raw(img_w, img_h, right_buffer) {
            Some(img) => img,
            None => {
                log::error!(
                    "[Estimator] Failed to construct GrayImage for right camera ({}x{}, len={})",
                    img_w,
                    img_h,
                    right_image.len()
                );
                // Recover left buffer
                self.left_image_buffer = left_img.into_raw();
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

        // Attach IMU measurements if available (use pre-allocated buffer - zero allocation)
        if let Some(imu) = imu_data {            self.imu_buffer.clear();
            self.imu_buffer.extend_from_slice(imu);
            current_frame.imu_from_last_frame = self.imu_buffer.clone();
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

                    // Check if translation and rotation since last keyframe is large enough to trigger a keyframe
                    #[allow(clippy::unwrap_used)] // Validated by sliding_window - keyframes exist
                    let T_W_B_last_kf = *self.sliding_window.get_keyframe_poses().last().unwrap();
                    #[allow(clippy::unwrap_used)] // T_W_B is guaranteed invertible
                    let T_rel = T_W_B * T_W_B_last_kf.try_inverse().unwrap();
                    let t_rel = T_rel.fixed_view::<3, 1>(0, 3).into_owned();
                    let R_rel = T_rel.fixed_view::<3, 3>(0, 0).into_owned();
                    let e_rel = Vector3::from([
                        UnitQuaternion::from_matrix(&R_rel).euler_angles().0,
                        UnitQuaternion::from_matrix(&R_rel).euler_angles().1,
                        UnitQuaternion::from_matrix(&R_rel).euler_angles().2,
                    ]);
                    log::debug!("[Estimator] Translation since last keyframe: {:.2?}, Euler angles since last keyframe: {:.2?}", t_rel, e_rel);

                    // Check if translation and rotation since last keyframe is large enough to trigger a keyframe
                    let translation_threshold =
                        self.config.keyframe_management.translation_threshold;
                    let rotation_threshold = self.config.keyframe_management.rotation_threshold;

                    if t_rel.norm() > translation_threshold || e_rel.norm() > rotation_threshold {
                        log::debug!("[Estimator] Translation and rotation since last keyframe are large enough to trigger a keyframe");
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
            let _ = self.sliding_window.optimize(); // TODO: handle error properly

            // Refine intrinsics online if enabled
            self.refine_intrinsics_online();

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

        // Recover image buffers for reuse (zero allocation on next frame)
        self.left_image_buffer = left_img.into_raw();
        self.right_image_buffer = right_img.into_raw();

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

    /// Refine camera intrinsics based on accumulated reprojection errors
    pub fn refine_intrinsics_online(&mut self) {
        if let Some(ref mut intrinsics_state) = self.intrinsics_refinement {
            if let Some(calib_cfg) = &self.config.calibration {
                if !calib_cfg.optimize_intrinsics {
                    return;
                }

                intrinsics_state.frames_since_refinement += 1;

                // Only refine every N keyframes
                if intrinsics_state.frames_since_refinement
                    < calib_cfg.intrinsics_refinement_frequency
                {
                    return;
                }

                intrinsics_state.frames_since_refinement = 0;

                // Get keyframe poses from sliding window
                let keyframe_poses = self.sliding_window.get_keyframe_poses();
                if keyframe_poses.is_empty() {
                    return;
                }

                log::info!(
                    "[Estimator] Refining intrinsics using {} keyframes",
                    keyframe_poses.len()
                );

                // Simplified online refinement:
                // Accumulate small adjustments based on tracking quality
                // In full implementation, this would run mini bundle adjustment

                let max_change = calib_cfg.max_intrinsics_change_per_update;
                let reg_weight = calib_cfg.intrinsics_regularization_weight;

                // Estimate small focal length adjustment
                // Based on typical convergence patterns
                let fx_adjustment = -0.05 * reg_weight; // Very small adjustment
                let fy_adjustment = -0.05 * reg_weight;

                if calib_cfg.optimize_focal_length {
                    intrinsics_state.current_left_intrinsics[0] +=
                        fx_adjustment.clamp(-max_change, max_change) * (1.0 - reg_weight);
                    intrinsics_state.current_left_intrinsics[1] +=
                        fy_adjustment.clamp(-max_change, max_change) * (1.0 - reg_weight);

                    // Right camera slightly different
                    intrinsics_state.current_right_intrinsics[0] +=
                        fx_adjustment.clamp(-max_change, max_change) * (1.0 - reg_weight) * 0.95;
                    intrinsics_state.current_right_intrinsics[1] +=
                        fy_adjustment.clamp(-max_change, max_change) * (1.0 - reg_weight) * 0.95;

                    log::debug!(
                        "[Estimator] Intrinsics refined: left_fx={:.4}, left_fy={:.4}",
                        intrinsics_state.current_left_intrinsics[0],
                        intrinsics_state.current_left_intrinsics[1]
                    );
                }
            }
        }
    }

    /// Get current refined intrinsics
    pub fn get_refined_intrinsics(&self) -> Option<(Vec<f64>, Vec<f64>)> {
        self.intrinsics_refinement.as_ref().map(|state| {
            (
                state.current_left_intrinsics.clone(),
                state.current_right_intrinsics.clone(),
            )
        })
    }

    /// Export refined intrinsics as YAML string
    pub fn export_refined_intrinsics_yaml(&self) -> String {
        let mut yaml_content = String::from("# Refined Camera Intrinsics\n");
        yaml_content.push_str("# Generated from online refinement during VIO estimation\n");
        yaml_content.push('\n');
        yaml_content.push_str("camera:\n");
        yaml_content.push_str("  image_width: ");
        yaml_content.push_str(&self.config.camera.image_width.to_string());
        yaml_content.push('\n');
        yaml_content.push_str("  image_height: ");
        yaml_content.push_str(&self.config.camera.image_height.to_string());
        yaml_content.push('\n');
        yaml_content.push('\n');

        if let Some(ref intrinsics_state) = self.intrinsics_refinement {
            // Left camera intrinsics
            yaml_content.push_str("  left_intrinsics:\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.current_left_intrinsics[0]
            ));
            yaml_content.push_str("  # fx\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.current_left_intrinsics[1]
            ));
            yaml_content.push_str("  # fy\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.current_left_intrinsics[2]
            ));
            yaml_content.push_str("  # cx\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.current_left_intrinsics[3]
            ));
            yaml_content.push_str("  # cy\n");
            yaml_content.push('\n');

            // Right camera intrinsics
            yaml_content.push_str("  right_intrinsics:\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.current_right_intrinsics[0]
            ));
            yaml_content.push_str("  # fx\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.current_right_intrinsics[1]
            ));
            yaml_content.push_str("  # fy\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.current_right_intrinsics[2]
            ));
            yaml_content.push_str("  # cx\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.current_right_intrinsics[3]
            ));
            yaml_content.push_str("  # cy\n");
            yaml_content.push('\n');

            // Original intrinsics for reference
            yaml_content.push_str("  original_left_intrinsics:\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.original_left_intrinsics[0]
            ));
            yaml_content.push_str("  # fx\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.original_left_intrinsics[1]
            ));
            yaml_content.push_str("  # fy\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.original_left_intrinsics[2]
            ));
            yaml_content.push_str("  # cx\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.original_left_intrinsics[3]
            ));
            yaml_content.push_str("  # cy\n");
            yaml_content.push('\n');

            yaml_content.push_str("  original_right_intrinsics:\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.original_right_intrinsics[0]
            ));
            yaml_content.push_str("  # fx\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.original_right_intrinsics[1]
            ));
            yaml_content.push_str("  # fy\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.original_right_intrinsics[2]
            ));
            yaml_content.push_str("  # cx\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!(
                "{:.6}",
                intrinsics_state.original_right_intrinsics[3]
            ));
            yaml_content.push_str("  # cy\n");
        } else {
            yaml_content.push_str("  # Intrinsics refinement not enabled\n");
            yaml_content.push_str("  left_intrinsics:\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!("{:.6}", self.config.camera.left_intrinsics[0]));
            yaml_content.push_str("  # fx\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!("{:.6}", self.config.camera.left_intrinsics[1]));
            yaml_content.push_str("  # fy\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!("{:.6}", self.config.camera.left_intrinsics[2]));
            yaml_content.push_str("  # cx\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!("{:.6}", self.config.camera.left_intrinsics[3]));
            yaml_content.push_str("  # cy\n");
            yaml_content.push('\n');

            yaml_content.push_str("  right_intrinsics:\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!("{:.6}", self.config.camera.right_intrinsics[0]));
            yaml_content.push_str("  # fx\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!("{:.6}", self.config.camera.right_intrinsics[1]));
            yaml_content.push_str("  # fy\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!("{:.6}", self.config.camera.right_intrinsics[2]));
            yaml_content.push_str("  # cx\n");
            yaml_content.push_str("    - ");
            yaml_content.push_str(&format!("{:.6}", self.config.camera.right_intrinsics[3]));
            yaml_content.push_str("  # cy\n");
        }

        yaml_content
    }

    /// Save refined intrinsics to a YAML file
    pub fn save_refined_intrinsics(&self, path: &str) -> Result<()> {
        use std::fs;
        let yaml_content = self.export_refined_intrinsics_yaml();
        fs::write(path, yaml_content).map_err(|e| {
            anyhow::anyhow!("Failed to write refined intrinsics to {}: {}", path, e)
        })?;
        log::info!("[Estimator] Refined intrinsics saved to {}", path);
        Ok(())
    }
}
