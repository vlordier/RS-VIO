use crate::datasets::config::Config;
use crate::datasets::CameraModelType;
use crate::datasets::ImuData;
use crate::estimator::Frame;
use crate::estimator::SlidingWindow;
use crate::feature_tracker::StereoPatchTracker;
use crate::types::{Matrix4x4, UnitQuaternion, Vector3};
use crate::viewers::Viewer;
use crate::{Result, VIOError};
use image::GrayImage;
use nalgebra as na;
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
    // Full trajectory of keyframes
    trajectory: Vec<Matrix4x4>,
    // Maximum allowed time for frame processing (for real-time safety)
    max_frame_processing_time: Duration,
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
        let T_B_Cl = na::Matrix4::from_row_slice(&config.camera.T_B_Cl);
        let T_B_Cr = na::Matrix4::from_row_slice(&config.camera.T_B_Cr);
        let keyframe_window_size = config.keyframe_management.keyframe_window_size as usize;
        let processing_timeout_ms = config.keyframe_management.processing_timeout_ms;
        Estimator {
            frame_id_counter: 0,
            frames_since_last_keyframe: 0,
            enable_debug_output: true,
            config,
            stereo_patch_tracker: StereoPatchTracker::<6>::from_config(&feature_config),
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

        // New frame: update counters
        self.frame_id_counter += 1;
        self.frames_since_last_keyframe += 1;

        if self.enable_debug_output {
            log::debug!(
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

        // Frame creation
        let frame_creation_start = Instant::now();

        // Create GrayImage objects directly from input slices (no clone needed for tracking)
        let img_w = self.config.camera.image_width;
        let img_h = self.config.camera.image_height;

        let left_img = match GrayImage::from_raw(img_w, img_h, left_image.to_vec()) {
            Some(img) => img,
            None => {
                log::error!("[Estimator] Failed to construct GrayImage for left camera");
                return Err(VIOError::Image(
                    "Failed to create left image: size mismatch".to_string(),
                ));
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

        // Attach IMU measurements if available.
        if let Some(imu) = imu_data {
            current_frame.imu_from_last_frame = imu.to_vec();
        }

        _frame_creation_time_ms = frame_creation_start.elapsed().as_secs_f64() * 1000.0;

        // Patch tracking
        let tracking_start = Instant::now();
        self.stereo_patch_tracker
            .process_frame(&left_img, &right_img, &mut current_frame);
        _patch_tracking_time_ms = tracking_start.elapsed().as_secs_f64() * 1000.0;
        self.view_patch_tracking_results(&current_frame, &left_img, &right_img, img_w, img_h);

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

                    // Check if translation and rotation since last keyframe is large enough to trigger a keyframe
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
            _motion_tracking_time_ms = motion_tracking_start.elapsed().as_secs_f64() * 1000.0;
        } else {
            log::debug!("[Estimator] Sliding window is not full, skipping motion tracking");
        }

        // View map points and keyframe poses
        // Bundle adjustment
        if current_frame.is_keyframe {
            let optimization_start = Instant::now();
            self.sliding_window.add_frame(current_frame);
            if let Err(e) = self.sliding_window.optimize() {
                log::error!("[Estimator] Bundle adjustment optimization failed: {:?}", e);
                // Continue execution even if optimization fails
            }
            _optimization_time_ms = optimization_start.elapsed().as_secs_f64() * 1000.0;
            self.view_optimization_results();
        }

        // Final timing summary
        let total_duration_ms = _total_start_time.elapsed().as_secs_f64() * 1000.0;
        log::debug!(
            "[Timing] frame_creation={:.3} ms, patch_tracking={:.3} ms, motion_tracking={:.3} ms, optimization={:.3} ms, total={:.3} ms",
            _frame_creation_time_ms,
            _patch_tracking_time_ms,
            _motion_tracking_time_ms,
            _optimization_time_ms,
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
        self.frame_id_counter
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
            if let Some(&mat) = self.sliding_window.get_keyframe_poses().first() {
                self.trajectory.push(mat);

                // Display trajectory as a continuous 3D path
                v.log_trajectory(&self.trajectory, "trajectory/path");
                // log::info!("[Estimator] System position: {:?}, {:?}, {:?}", mat[0][3], mat[1][3], mat[2][3]);
            }
        }
    }

    /// Get a reference to the trajectory (for saving to file)
    pub fn get_trajectory(&self) -> &Vec<Matrix4x4> {
        &self.trajectory
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
        estimator.set_max_frame_processing_time(std::time::Duration::from_millis(1000));

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
