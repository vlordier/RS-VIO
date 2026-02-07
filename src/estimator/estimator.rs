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
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use std::time::Instant;

/// Stereo visual odometry estimator.
/// Processes stereo frames through feature tracking, motion estimation,
/// keyframe management, and sliding-window bundle adjustment.
pub struct Estimator {
    frame_id_counter: u64,
    /// Full configuration loaded from YAML (used to derive intrinsics, etc.).
    pub(crate) config: Config,
    /// Patch-based stereo tracker reused across all frames.
    stereo_patch_tracker: StereoPatchTracker<6>,
    /// Sliding window of keyframes for bundle adjustment optimization.
    sliding_window: Arc<Mutex<SlidingWindow>>,
    /// Optimization in-flight flag for background bundle adjustment.
    optimization_in_flight: Arc<AtomicBool>,
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

        // Pre-allocate image buffers for zero-allocation frame processing
        let image_size = (config.camera.image_width * config.camera.image_height) as usize;
        let left_image_buffer = vec![0u8; image_size];
        let right_image_buffer = vec![0u8; image_size];
        // Pre-allocate IMU buffer (typical 20-50 samples per frame)
        let imu_buffer = Vec::with_capacity(100);

        Estimator {
            frame_id_counter: 0,
            config,
            stereo_patch_tracker: StereoPatchTracker::<6>::new(
                grid_size,
                optical_flow_max_iterations,
                optical_flow_convergence_threshold,
            ),
            sliding_window: Arc::new(Mutex::new(SlidingWindow::new(keyframe_window_size))),
            optimization_in_flight: Arc::new(AtomicBool::new(false)),
            viewer,
            left_cam,
            right_cam,
            T_B_Cl,
            T_B_Cr,
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

        log::debug!(
            "============================== Frame {} ==============================",
            self.frame_id_counter
        );

        // Timing placeholders (warning: assigned before read in log::debug below)
        #[allow(unused_assignments)]
        let mut frame_creation_time_ms = 0.0f64;
        #[allow(unused_assignments)]
        let mut patch_tracking_time_ms = 0.0f64;
        let motion_tracking_time_ms;
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
            anyhow::bail!(
                "Invalid image size: left={}, right={}, expected={} ({}x{})",
                left_image.len(),
                right_image.len(),
                expected_size,
                img_w,
                img_h
            );
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
                // Recover right buffer (not yet consumed); left buffer lost in from_raw
                self.right_image_buffer = right_buffer;
                self.left_image_buffer = vec![0u8; left_image.len()];
                log::error!(
                    "[Estimator] Failed to construct GrayImage for left camera ({}x{}, len={})",
                    img_w,
                    img_h,
                    left_image.len()
                );
                anyhow::bail!(
                    "Failed to construct GrayImage for left camera ({}x{}, len={})",
                    img_w,
                    img_h,
                    left_image.len()
                );
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
                anyhow::bail!(
                    "Failed to construct GrayImage for right camera ({}x{}, len={})",
                    img_w,
                    img_h,
                    right_image.len()
                );
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
            i32::try_from(self.frame_id_counter).unwrap_or(i32::MAX),
            self.left_cam.clone(),
            self.right_cam.clone(),
            self.T_B_Cl,
            self.T_B_Cr,
        );

        // Attach IMU measurements if available (use pre-allocated buffer - zero allocation)
        if let Some(imu) = imu_data {
            self.imu_buffer.clear();
            self.imu_buffer.extend_from_slice(imu);
            current_frame.imu_from_last_frame =
                std::mem::replace(&mut self.imu_buffer, Vec::with_capacity(100));
        }

        frame_creation_time_ms = frame_creation_start.elapsed().as_secs_f64() * 1000.0;

        // Patch tracking
        let tracking_start = Instant::now();
        self.stereo_patch_tracker
            .process_frame(&left_img, &right_img, &mut current_frame);
        patch_tracking_time_ms = tracking_start.elapsed().as_secs_f64() * 1000.0;
        self.view_patch_tracking_results(&current_frame, &left_img, &right_img, img_w, img_h);

        // Motion tracking - only if the sliding window is full (has initialized keyframes)
        let mut is_bootstrap = false;
        // last_kf_pose extracted while holding the try_lock guard to avoid a
        // second blocking lock() after the guard is dropped.
        let mut last_kf_pose: Option<Matrix4x4> = None;
        let motion_tracking_result = match self.sliding_window.try_lock() {
            Ok(mut sliding_window) => {
                if sliding_window.is_full() {
                    let motion_tracking_start = Instant::now();
                    let result = sliding_window.track_motion(&current_frame);
                    // Grab last keyframe pose while we still hold the lock
                    last_kf_pose = sliding_window.last_keyframe_pose();
                    drop(sliding_window);
                    let motion_tracking_elapsed =
                        motion_tracking_start.elapsed().as_secs_f64() * 1000.0;
                    (result, motion_tracking_elapsed)
                } else {
                    is_bootstrap = true;
                    drop(sliding_window);
                    (Ok(None), 0.0)
                }
            },
            Err(std::sync::TryLockError::WouldBlock) => {
                log::debug!("Sliding window busy, skipping motion tracking");
                (Ok(None), 0.0)
            },
            Err(std::sync::TryLockError::Poisoned(e)) => {
                return Err(anyhow::anyhow!("Sliding window mutex is poisoned: {}", e));
            },
        };

        match motion_tracking_result.0 {
            Ok(Some(T_W_B)) => {
                // Apply the optimized pose to the current frame
                current_frame.state.T_W_B = T_W_B;

                // Use pre-fetched last keyframe pose (grabbed while holding try_lock)
                let Some(T_W_B_last_kf) = last_kf_pose else {
                    log::warn!("[Estimator] No last keyframe pose available for keyframe decision");
                    current_frame.is_keyframe = true;
                    motion_tracking_time_ms = motion_tracking_result.1;
                    // Skip to keyframe insertion
                    let optimization_start = Instant::now();
                    {
                        let mut sliding_window = self.sliding_window.lock().unwrap();
                        sliding_window.add_frame(current_frame);
                    }
                    self.schedule_optimization();
                    optimization_time_ms = optimization_start.elapsed().as_secs_f64() * 1000.0;
                    let total_duration_ms = total_start_time.elapsed().as_secs_f64() * 1000.0;
                    log::debug!(
                        "[Timing] frame_creation={:.3} ms, patch_tracking={:.3} ms, motion_tracking={:.3} ms, optimization={:.3} ms, total={:.3} ms",
                        frame_creation_time_ms, patch_tracking_time_ms, motion_tracking_time_ms, optimization_time_ms, total_duration_ms
                    );
                    self.left_image_buffer = left_img.into_raw();
                    self.right_image_buffer = right_img.into_raw();
                    return Ok(());
                };
                #[allow(clippy::unwrap_used)] // T_W_B is guaranteed invertible
                let T_rel = T_W_B * T_W_B_last_kf.try_inverse().unwrap();
                let t_rel = T_rel.fixed_view::<3, 1>(0, 3).into_owned();
                let R_rel = T_rel.fixed_view::<3, 3>(0, 0).into_owned();
                let e_rel = {
                    let (r, p, y) = UnitQuaternion::from_matrix(&R_rel).euler_angles();
                    Vector3::from([r, p, y])
                };
                log::debug!("[Estimator] Translation since last keyframe: {:.2?}, Euler angles since last keyframe: {:.2?}", t_rel, e_rel);

                // Check if translation and rotation since last keyframe is large enough to trigger a keyframe
                let translation_threshold = self.config.keyframe_management.translation_threshold;
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
                if is_bootstrap {
                    log::info!("[Estimator] Bootstrap: adding keyframe (window not yet full)");
                    // is_keyframe stays true (default from from_stereo_images)
                } else {
                    log::warn!(
                        "[Estimator] Motion tracking failed (optimization did not converge)"
                    );
                    current_frame.is_keyframe = false;
                }
            },
            Err(e) => {
                log::error!("[Estimator] Motion tracking error: {:?}", e);
                current_frame.is_keyframe = false;
            },
        }
        motion_tracking_time_ms = motion_tracking_result.1;

        // View map points and keyframe poses
        // Bundle adjustment
        if current_frame.is_keyframe {
            let optimization_start = Instant::now();
            {
                let mut sliding_window = self.sliding_window.lock().unwrap();
                sliding_window.add_frame(current_frame);
            }
            self.schedule_optimization();

            optimization_time_ms = optimization_start.elapsed().as_secs_f64() * 1000.0;
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

    /// Schedule background bundle adjustment on keyframes (non-blocking)
    fn schedule_optimization(&self) {
        if !self.optimization_in_flight.swap(true, Ordering::AcqRel) {
            let sliding_window = Arc::clone(&self.sliding_window);
            let in_flight = Arc::clone(&self.optimization_in_flight);

            std::thread::spawn(move || match sliding_window.lock() {
                Ok(mut window) => {
                    let _ = window.optimize();
                    in_flight.store(false, Ordering::Release);
                },
                Err(e) => {
                    log::error!(
                        "Failed to acquire sliding window lock for optimization: {}",
                        e
                    );
                    in_flight.store(false, Ordering::Release);
                },
            });
        }
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
            v.log_pose(*T_W_B, "pose_current");

            let width = self.config.camera.image_width;
            let height = self.config.camera.image_height;

            // Log left camera frustum at the pose location (left camera is at the pose)
            let left_focal_length = self.config.camera.left_intrinsics[0] as f32;
            let T_W_Cl = T_W_B * self.T_B_Cl;
            v.log_pose(T_W_Cl, "pose_current_left");
            v.log_camera_frustum(left_focal_length, width, height, "pose_current_left", 0.4);
        }
    }
}
