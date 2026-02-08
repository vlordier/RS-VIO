//! Core VIO estimator with sliding-window optimization.
//!
//! Orchestrates feature tracking, IMU integration, keyframe selection,
//! and nonlinear optimization to produce real-time pose estimates.

use crate::datasets::config::Config;
use crate::datasets::CameraModelType;
use crate::datasets::ImuData;
use crate::estimator::sliding_window::SlidingWindow;
use crate::estimator::Frame;
use crate::feature_tracker::StereoPatchTracker;
use crate::types::{Matrix4x4, UnitQuaternion};
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
    /// Last two successfully tracked poses for constant-velocity prediction.
    /// (previous, current) — used to extrapolate initial guess for track_motion.
    last_two_poses: (Option<Matrix4x4>, Option<Matrix4x4>),
    /// Counter for consecutive motion tracking failures.
    /// When this exceeds a threshold, we force a keyframe to prevent map staleness.
    consecutive_tracking_failures: u32,
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
            last_two_poses: (None, None),
            consecutive_tracking_failures: 0,
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

        // Timing
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

        let frame_creation_time_ms = frame_creation_start.elapsed().as_secs_f64() * 1000.0;

        // Patch tracking
        let tracking_start = Instant::now();
        self.stereo_patch_tracker
            .process_frame(&left_img, &right_img, &mut current_frame);
        let patch_tracking_time_ms = tracking_start.elapsed().as_secs_f64() * 1000.0;
        self.view_patch_tracking_results(&current_frame, &left_img, &right_img, img_w, img_h);

        // Motion tracking - only if the sliding window is full (has initialized keyframes)
        let mut is_bootstrap = false;
        // last_kf_pose extracted while holding the try_lock guard to avoid a
        // second blocking lock() after the guard is dropped.
        let mut last_kf_pose: Option<Matrix4x4> = None;

        // Constant-velocity prediction: extrapolate from last two successful poses
        let predicted_pose = match self.last_two_poses {
            (Some(prev), Some(curr)) => {
                // delta = curr * prev^{-1}, predicted = delta * curr
                prev.try_inverse().map(|prev_inv| {
                    let delta = curr * prev_inv;
                    delta * curr
                })
            },
            (_, Some(curr)) => Some(curr), // Only one pose: use it directly
            _ => None,
        };

        let motion_tracking_result = match self.sliding_window.try_lock() {
            Ok(mut sliding_window) => {
                if sliding_window.is_full() {
                    let motion_tracking_start = Instant::now();
                    let result = sliding_window.track_motion(&current_frame, predicted_pose);
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
                // Tracking succeeded: update pose history and reset failure counter
                self.last_two_poses = (self.last_two_poses.1, Some(T_W_B));
                self.consecutive_tracking_failures = 0;

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
                        let mut sliding_window = self
                            .sliding_window
                            .lock()
                            .map_err(|e| anyhow::anyhow!("Sliding window lock poisoned: {}", e))?;
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
                let T_W_B_last_kf_inv = match T_W_B_last_kf.try_inverse() {
                    Some(inv) => inv,
                    None => {
                        log::warn!("[Estimator] Last keyframe pose is singular, skipping keyframe decision");
                        self.left_image_buffer = left_img.into_raw();
                        self.right_image_buffer = right_img.into_raw();
                        return Ok(());
                    },
                };
                let T_rel = T_W_B * T_W_B_last_kf_inv;
                let t_rel = T_rel.fixed_view::<3, 1>(0, 3).into_owned();
                let R_rel = T_rel.fixed_view::<3, 3>(0, 0).into_owned();
                let angle_rel = UnitQuaternion::from_matrix(&R_rel).angle();
                log::debug!("[Estimator] Translation since last keyframe: {:.2?}, Rotation angle since last keyframe: {:.4} rad", t_rel, angle_rel);

                // Check if translation and rotation since last keyframe is large enough to trigger a keyframe
                let translation_threshold = self.config.keyframe_management.translation_threshold;
                let rotation_threshold = self.config.keyframe_management.rotation_threshold;

                if t_rel.norm() > translation_threshold || angle_rel > rotation_threshold {
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
                    self.consecutive_tracking_failures += 1;
                    // Force keyframe after 5 consecutive failures to refresh the map
                    // and prevent death spiral from stale map_points
                    const MAX_CONSECUTIVE_FAILURES: u32 = 5;
                    if self.consecutive_tracking_failures >= MAX_CONSECUTIVE_FAILURES {
                        log::warn!(
                            "[Estimator] Motion tracking failed {} consecutive times, forcing keyframe to refresh map",
                            self.consecutive_tracking_failures
                        );
                        // Use the predicted pose (or last known pose) for the forced keyframe
                        if let Some(pose) = predicted_pose.or(self.last_two_poses.1) {
                            current_frame.state.T_W_B = pose;
                        }
                        current_frame.is_keyframe = true;
                        self.consecutive_tracking_failures = 0;
                    } else {
                        log::warn!(
                            "[Estimator] Motion tracking failed (optimization did not converge) [{}/{}]",
                            self.consecutive_tracking_failures,
                            MAX_CONSECUTIVE_FAILURES
                        );
                        current_frame.is_keyframe = false;
                    }
                }
            },
            Err(e) => {
                log::error!("[Estimator] Motion tracking error: {:?}", e);
                self.consecutive_tracking_failures += 1;
                current_frame.is_keyframe = false;
            },
        }
        motion_tracking_time_ms = motion_tracking_result.1;

        // View map points and keyframe poses
        // Bundle adjustment
        if current_frame.is_keyframe {
            let optimization_start = Instant::now();
            {
                let mut sliding_window = self
                    .sliding_window
                    .lock()
                    .map_err(|e| anyhow::anyhow!("Sliding window lock poisoned: {}", e))?;
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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::datasets::config::{
        CameraConfig, Config, FeatureDetectionConfig, KeyframeManagementConfig,
        OptimizationConfig,
    };

    fn create_test_config() -> Config {
        Config {
            camera: CameraConfig {
                image_width: 640,
                image_height: 480,
                left_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
                left_distortion: vec![0.0, 0.0, 0.0, 0.0, 0.0],
                right_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
                right_distortion: vec![0.0, 0.0, 0.0, 0.0, 0.0],
                left_model: None,
                right_model: None,
                T_B_Cl: vec![
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0,
                ],
                T_B_Cr: vec![
                    1.0, 0.0, 0.0, -0.12, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0,
                ],
            },
            keyframe_management: KeyframeManagementConfig {
                keyframe_window_size: 8,
                translation_threshold: 0.2,
                rotation_threshold: 0.1,
            },
            feature_detection: FeatureDetectionConfig {
                grid_cols: 16,
                max_features_per_grid: 80,
                optical_flow_max_iterations: 30,
                optical_flow_convergence_threshold: 0.01,
            },
            optimization: OptimizationConfig {
                bundle_adjustment_max_iterations: 5,
                pnp_max_iterations: 5,
            },
            calibration: None,
        }
    }

    #[test]
    fn test_estimator_new_default() {
        let config = create_test_config();
        let estimator = Estimator::new(config, None);
        assert_eq!(estimator.frame_id_counter, 0);
        assert_eq!(estimator.config.camera.image_width, 640);
        assert_eq!(estimator.config.camera.image_height, 480);
        assert!(estimator.viewer.is_none());
        assert_eq!(estimator.consecutive_tracking_failures, 0);
    }

    #[test]
    fn test_estimator_new_with_explicit_cameras() {
        let config = create_test_config();
        let (left_cam, right_cam) = crate::datasets::create_camera_models_from_config(&config);
        let estimator =
            Estimator::new_with_cameras(config, None, Some(left_cam), Some(right_cam));
        assert_eq!(estimator.frame_id_counter, 0);
        assert!(estimator.viewer.is_none());
        // Sliding window should be empty initially
        let sw = estimator.sliding_window.lock().unwrap();
        assert!(sw.is_empty());
    }

    #[test]
    fn test_process_frame_rejects_wrong_image_size() {
        let config = create_test_config();
        let mut estimator = Estimator::new(config, None);
        let small_img = vec![128u8; 100]; // Much smaller than 640x480
        let result = estimator.process_frame(&small_img, &small_img, 1_000_000_000, None);
        assert!(result.is_err());
        let err_msg = format!("{}", result.unwrap_err());
        assert!(err_msg.contains("Invalid image size"));
    }

    #[test]
    fn test_process_frame_bootstrap_adds_keyframe() {
        let config = create_test_config();
        let w = config.camera.image_width as usize;
        let h = config.camera.image_height as usize;
        let mut estimator = Estimator::new(config, None);
        let image = vec![128u8; w * h];
        let result = estimator.process_frame(&image, &image, 1_000_000_000, None);
        assert!(result.is_ok());
        assert_eq!(estimator.frame_id_counter, 1);
        // Bootstrap should add the frame as a keyframe to the sliding window
        let sw = estimator.sliding_window.lock().unwrap();
        assert_eq!(sw.len(), 1);
    }

    #[test]
    fn test_set_viewer_frame_without_viewer() {
        let config = create_test_config();
        let mut estimator = Estimator::new(config, None);
        // Should not panic when no viewer is attached
        estimator.set_viewer_frame(42);
    }
}
