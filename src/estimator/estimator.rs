//! Stereo VIO estimator.
//!
//! The [`Estimator`] is the central component of the VIO pipeline. It receives stereo
//! image pairs, tracks features, manages keyframe insertion, and runs bundle adjustment
//! optimization through the [`SlidingWindow`].

use crate::datasets::config::Config;
use crate::datasets::CameraModelType;
use crate::datasets::ImuData;
use nalgebra034;
use crate::estimator::sliding_window::{inverse_se3, SlidingWindow};
use crate::estimator::relocalization::{Relocalizer, RelocalizationConfig};
use crate::estimator::Frame;
use crate::feature_tracker::StereoPatchTracker;
use crate::types::{Matrix4x4, UnitQuaternion, Vector3};
use crate::viewers::Viewer;
use anyhow::Result;
use image::GrayImage;
use nalgebra as na;
use std::time::Instant;

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
    pub sliding_window: SlidingWindow,
    /// Relocalization module for recovery from tracking failures.
    relocalizer: Relocalizer,
    /// Optional viewer used for visualization; outlives the estimator.
    viewer: Option<&'a mut dyn Viewer>,
    // Transformation from body to left camera
    T_B_Cl: Matrix4x4,
    // Transformation from body to right camera
    T_B_Cr: Matrix4x4,
    // Full trajectory of keyframes (post-BA corrected)
    trajectory: Vec<Matrix4x4>,
    // Pre-BA PnP trajectory (for visualising BA correction effect)
    trajectory_pre_ba: Vec<Matrix4x4>,
    /// Estimated poses with timestamps: (timestamp_ns, T_W_B).
    /// Populated every frame that produces a motion-tracking result.
    pub pose_log: Vec<(i64, Matrix4x4)>,
    // Pre-computed feature Vec capacity (grid cell count); constant after construction.
    feat_capacity: usize,
    // Pre-computed undistortion LUTs: lut[y * lut_width + x] = [bearing_x, bearing_y] at integer pixel (x, y).
    // Built once at construction; avoids per-feature iterative unproject_one calls at runtime.
    left_undist_lut: Vec<[f32; 2]>,
    right_undist_lut: Vec<[f32; 2]>,
    lut_width: usize,
    lut_height: usize,
    /// IMU initialization state.
    imu_initialized: bool,
    /// Collected IMU samples for initialization (first ~2 seconds).
    imu_init_samples: Vec<crate::datasets::ImuData>,
    /// IMU samples accumulated since the last keyframe.
    /// Appended each frame; given to the next KF frame so inter-KF preintegration
    /// covers the full KF-to-KF interval, not just the most recent camera frame.
    imu_since_last_kf: Vec<crate::datasets::ImuData>,
    /// Diagnostic statistics for anomaly detection
    frame_times_ms: Vec<f64>,
    motion_tracking_times_ms: Vec<f64>,
    optimization_times_ms: Vec<f64>,
}

/// Build a `w × h` undistortion lookup table.
/// `lut[y * w + x]` holds the normalised bearing `[u_x, u_y]` (z = 1)
/// for integer pixel `(x, y)`, cast to f32.
fn build_undist_lut(cam: &CameraModelType, w: u32, h: u32) -> Vec<[f32; 2]> {
    let mut lut = Vec::with_capacity((w * h) as usize);
    for y in 0..h {
        for x in 0..w {
            let u = cam.as_camera_model().unproject_one(
                &nalgebra034::Vector2::new(x as f64, y as f64),
            );
            lut.push([u[0] as f32, u[1] as f32]);
        }
    }
    lut
}

/// Bilinear interpolation into an undistortion LUT.
/// `px` / `py` are sub-pixel feature coordinates; `w` and `h` are the LUT dimensions.
#[inline(always)]
fn bilinear_lut(lut: &[[f32; 2]], w: usize, h: usize, px: f32, py: f32) -> [f32; 2] {
    let ix = (px as usize).min(w - 2);
    let iy = (py as usize).min(h - 2);
    let dx = px - ix as f32;
    let dy = py - iy as f32;
    // 1-mul bilinear weight derivation: w11=dx*dy, then derive the rest with adds/subs.
    let w11 = dx * dy;
    let w00 = 1.0 - dx - dy + w11;
    let w10 = dx - w11;
    let w01 = dy - w11;
    let base = iy * w + ix;
    // SAFETY: ix = min(px as usize, w-2) <= w-2, iy = min(py as usize, h-2) <= h-2.
    // base + w + 1 = iy*w + ix + w + 1 <= (h-2)*w + (w-2) + w + 1 = (h-1)*w - 1 < h*w = lut.len().
    let (v00, v10, v01, v11) = unsafe {(
        *lut.get_unchecked(base),
        *lut.get_unchecked(base + 1),
        *lut.get_unchecked(base + w),
        *lut.get_unchecked(base + w + 1),
    )};
    [
        w00 * v00[0] + w10 * v10[0] + w01 * v01[0] + w11 * v11[0],
        w00 * v00[1] + w10 * v10[1] + w01 * v01[1] + w11 * v11[1],
    ]
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

        let img_w = config.camera.image_width;
        let img_h = config.camera.image_height;
        let feat_capacity = ((img_w / grid_size + 1) * (img_h / grid_size + 1)) as usize;

        let left_undist_lut = build_undist_lut(&left_cam, img_w, img_h);
        let right_undist_lut = build_undist_lut(&right_cam, img_w, img_h);
        let cam_imu_td_ns = (config.optimization.cam_imu_td_s * 1e9) as i64;
        let cam_imu_td_online_refinement = config.optimization.cam_imu_td_online_refinement;

        Estimator {
            frame_id_counter: 0,
            frames_since_last_keyframe: 0,
            enable_debug_output: true,
            stereo_patch_tracker: StereoPatchTracker::<6>::new(
                grid_size,
                optical_flow_max_iterations,
                optical_flow_convergence_threshold,
            ),
            sliding_window: {
                let mut sw = SlidingWindow::new(keyframe_window_size);
                sw.cam_imu_td_ns = cam_imu_td_ns;
                sw.cam_imu_td_online_refinement = cam_imu_td_online_refinement;
                // Set IMU parameters from config (clone before move)
                sw.imu_gyro_noise_std = config.imu.gyro_noise_std;
                sw.imu_gyro_random_walk_std = config.imu.gyro_random_walk_std;
                sw.imu_accel_noise_std = config.imu.accel_noise_std;
                sw.imu_accel_random_walk_std = config.imu.accel_random_walk_std;
                sw.imu_gravity_magnitude = config.imu.gravity_magnitude;
                sw
            },
            relocalizer: Relocalizer::new(RelocalizationConfig::default()),
            viewer,
            T_B_Cl,
            T_B_Cr,
            trajectory: Vec::new(),
            trajectory_pre_ba: Vec::new(),
            pose_log: Vec::new(),
            feat_capacity,
            left_undist_lut,
            right_undist_lut,
            lut_width: img_w as usize,
            lut_height: img_h as usize,
            config: config.clone(),  // Clone here to keep a copy
            imu_initialized: false,
            imu_init_samples: Vec::new(),
            imu_since_last_kf: Vec::new(),
            frame_times_ms: Vec::new(),
            motion_tracking_times_ms: Vec::new(),
            optimization_times_ms: Vec::new(),
        }
    }

    /// Process a single stereo frame.
    #[allow(unused_assignments)] // timing vars initialized then overwritten
    #[allow(clippy::cognitive_complexity)]
    #[allow(clippy::too_many_lines)]
    pub fn process_frame(
        &mut self,
        left_image: Vec<u8>,
        right_image: Vec<u8>,
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

        // Timing placeholders
        let mut frame_creation_time_ms = 0.0f64;
        let mut patch_tracking_time_ms = 0.0f64;
        let mut motion_tracking_time_ms = 0.0f64;
        let mut optimization_time_ms = 0.0f64;

        // Frame creation
        let frame_creation_start = Instant::now();

        // Create GrayImage objects directly from input slices (no clone needed for tracking)
        let img_w = self.config.camera.image_width;
        let img_h = self.config.camera.image_height;

        let left_len = left_image.len();
        let right_len = right_image.len();
        let left_img = match GrayImage::from_raw(img_w, img_h, left_image) {
            Some(img) => img,
            None => {
                log::error!(
                    "[Estimator] Failed to construct GrayImage for left camera ({}x{}, len={})",
                    img_w, img_h, left_len
                );
                return Ok(());
            }
        };
        let right_img = match GrayImage::from_raw(img_w, img_h, right_image) {
            Some(img) => img,
            None => {
                log::error!(
                    "[Estimator] Failed to construct GrayImage for right camera ({}x{}, len={})",
                    img_w, img_h, right_len
                );
                return Ok(());
            }
        };

        // Create frame; pre-size feature Vecs to the expected grid cell count.
        let mut current_frame = Frame::from_stereo_images(
            timestamp_ns,
            self.frame_id_counter as i32,
            self.T_B_Cl,
            self.T_B_Cr,
            self.feat_capacity,
        );

        // Attach IMU measurements if available.
        // Accumulate into `imu_since_last_kf` so each KF frame gets the full
        // inter-KF IMU interval for preintegration (not just the last 50ms).
        if let Some(imu) = imu_data {
            self.imu_since_last_kf.extend_from_slice(imu);
            current_frame.imu_from_last_frame = self.imu_since_last_kf.clone();

            // Collect IMU samples for initialization (first 2 seconds or ~400 samples)
            if !self.imu_initialized && self.imu_init_samples.len() < 400 {
                self.imu_init_samples.extend_from_slice(imu);

                // Initialize IMU once we have enough samples
                if self.imu_init_samples.len() >= 200 {
                    // Initialize IMU biases from stationary assumption
                    self.sliding_window.initialize_imu_from_stationary(
                        &self.imu_init_samples,
                        &mut current_frame.state,
                    );
                    self.imu_initialized = true;
                    log::info!(
                        "[Estimator] IMU initialized with {} samples, gyro_bias=[{:.4},{:.4},{:.4}], accel_bias=[{:.4},{:.4},{:.4}]",
                        self.imu_init_samples.len(),
                        current_frame.state.gyro_bias[0],
                        current_frame.state.gyro_bias[1],
                        current_frame.state.gyro_bias[2],
                        current_frame.state.accel_bias[0],
                        current_frame.state.accel_bias[1],
                        current_frame.state.accel_bias[2],
                    );
                }
            }
        }

        frame_creation_time_ms = frame_creation_start.elapsed().as_secs_f64() * 1000.0;

        // Patch tracking
        let tracking_start = Instant::now();
        self.stereo_patch_tracker
            .process_frame(&left_img, &right_img, &mut current_frame);

        // Undistort features — bilinear lookup into pre-computed LUT avoids per-feature
        // iterative unproject_one calls (up to 15 Newton-Raphson iterations for OpenCV5).
        let lw = self.lut_width;
        let lh = self.lut_height;
        for feat in &mut current_frame.left_features {
            feat.undistorted_coord = bilinear_lut(
                &self.left_undist_lut,
                lw,
                lh,
                feat.pixel_coord[0],
                feat.pixel_coord[1],
            );
        }
        for feat in &mut current_frame.right_features {
            feat.undistorted_coord = bilinear_lut(
                &self.right_undist_lut,
                lw,
                lh,
                feat.pixel_coord[0],
                feat.pixel_coord[1],
            );
        }

        // Compute ORB descriptors for loop closure detection
        let orb_detector = crate::feature_tracker::orb::OrbDetector::with_defaults();
        if !current_frame.left_features.is_empty() {
            let pts: Vec<(f32, f32)> = current_frame.left_features.iter()
                .map(|f| (f.pixel_coord[0], f.pixel_coord[1]))
                .collect();
            current_frame.left_descriptors = orb_detector.compute_descriptors_for_points(&left_img, &pts);
        }
        if !current_frame.right_features.is_empty() {
            let pts: Vec<(f32, f32)> = current_frame.right_features.iter()
                .map(|f| (f.pixel_coord[0], f.pixel_coord[1]))
                .collect();
            current_frame.right_descriptors = orb_detector.compute_descriptors_for_points(&right_img, &pts);
        }

        patch_tracking_time_ms = tracking_start.elapsed().as_secs_f64() * 1000.0;
        self.view_patch_tracking_results(&current_frame, &left_img, &right_img, img_w, img_h);

        // Motion tracking - only if the sliding window is full (has initialized keyframes)
        if self.sliding_window.is_full() {
            // Log CV prediction before PnP so Rerun shows warm-start vs. final pose.
            let cv_pred = self.sliding_window.predict_current_pose();
            if let Some(v) = &mut self.viewer {
                v.log_pose(cv_pred, "prediction/cv");
            }

            // DEBUG
            let motion_tracking_start = Instant::now();
            if self.enable_debug_output {
                log::debug!("[Frame {}] Starting motion tracking (window size={})",
                    self.frame_id_counter, self.sliding_window.len());
            }
            let motion_tracking_result = self.sliding_window.track_motion(&current_frame);
            match motion_tracking_result {
                Ok(Some(T_W_B)) => {
                    // Apply the optimized pose to the current frame
                    current_frame.state.T_W_B = T_W_B;

                    // Check if translation and rotation since last keyframe is large enough to trigger a keyframe
                    // Use get_frame() to avoid allocating a Vec of all poses
                    let T_W_B_last_kf = self
                        .sliding_window
                        .get_frame(self.sliding_window.len() - 1)
                        .expect("sliding window should have at least one keyframe")
                        .state
                        .T_W_B;
                    let T_rel = T_W_B * inverse_se3(&T_W_B_last_kf);
                    let t_rel = T_rel.fixed_view::<3, 1>(0, 3).into_owned();
                    // Geodesic rotation angle via matrix trace: θ = acos((tr(R)-1)/2).
                    // Computing cos(θ) directly is cheaper than UnitQuaternion::from_matrix
                    // (Shepperd method: 4-case branch + sqrt + div) + angle() (acos):
                    // 3 adds + 1 mul vs ~20 ops + sqrt + acos.
                    // acos is deferred to the triggered-keyframe log (rare path).
                    let cos_r = ((T_rel[(0, 0)] + T_rel[(1, 1)] + T_rel[(2, 2)]) - 1.0) * 0.5;

                    if log::log_enabled!(log::Level::Debug) {
                        let R_rel: na::Matrix3<f64> = T_rel.fixed_view::<3, 3>(0, 0).into_owned();
                        let q_rel = UnitQuaternion::from_matrix(&R_rel);
                        let e_rel = q_rel.euler_angles();
                        let e_rel = Vector3::from([e_rel.0, e_rel.1, e_rel.2]);
                        log::debug!("[Estimator] Translation since last keyframe: {:.2?}, Euler angles since last keyframe: {:.2?}", t_rel, e_rel);
                    }

                    // Check if translation and rotation since last keyframe is large enough to trigger a keyframe
                    let translation_threshold =
                        self.config.keyframe_management.translation_threshold;
                    let rotation_threshold = self.config.keyframe_management.rotation_threshold;

                    // IMU-based keyframe criteria: high variance or large predicted displacement
                    let imu_triggers_keyframe = if let Some(imu) = imu_data {
                        // High gyro variance → fast rotation → visual tracking unreliable
                        // Use mutable accumulators instead of .fold() to avoid per-sample array allocs
                        let mut gx2 = 0.0f64;
                        let mut gy2 = 0.0f64;
                        let mut gz2 = 0.0f64;
                        let mut accel_sum = 0.0f64;
                        for s in imu.iter() {
                            gx2 += s.gyro[0] * s.gyro[0];
                            gy2 += s.gyro[1] * s.gyro[1];
                            gz2 += s.gyro[2] * s.gyro[2];
                            accel_sum += s.accel[0].abs() + s.accel[1].abs() + s.accel[2].abs();
                        }
                        let n = imu.len() as f64;
                        // Avoid sqrt and division: compare squared/scaled values directly.
                        // gyro_rms = sqrt(sum_sq)/n > thr  ⟺  sum_sq > (n*thr)²
                        // imu_disp = accel_sum/n > thr      ⟺  accel_sum > n*thr
                        let imu_gyro_thr = self.config.keyframe_management.imu_gyro_kf_threshold;
                        let imu_accel_thr = self.config.keyframe_management.imu_accel_kf_threshold;
                        let n_rot_thr = n * imu_gyro_thr;
                        (gx2 + gy2 + gz2) > n_rot_thr * n_rot_thr
                            || accel_sum > n * imu_accel_thr
                    } else {
                        false
                    };

                    // Track-based keyframe criteria: high ratio of new features → low overlap
                    // with previous keyframe. Triggers keyframes during rotation-in-place
                    // or low-texture scenes where visual odometry would otherwise drift.
                    let total_features = current_frame.left_features.len();
                    let new_features = current_frame
                        .left_features
                        .iter()
                        .filter(|f| f.age == 0)
                        .count();
                    let new_ratio = if total_features > 0 {
                        new_features as f64 / total_features as f64
                    } else {
                        0.0
                    };
                    let new_feature_threshold = 0.4; // 40% new features → create keyframe
                    let track_triggers_keyframe = new_ratio > new_feature_threshold;

                    // Squared translation: avoids sqrt on every non-keyframe frame.
                    // Cosine comparison: avoids acos on every non-keyframe frame.
                    // θ > rotation_threshold ↔ cos(θ) < cos(rotation_threshold) for θ ∈ [0, π].
                    let t_sq = t_rel.norm_squared();
                    let t_sq_thresh = translation_threshold * translation_threshold;
                    let cos_r_thresh = rotation_threshold.cos();
                    if t_sq > t_sq_thresh
                        || cos_r < cos_r_thresh
                        || imu_triggers_keyframe
                        || track_triggers_keyframe
                    {
                        let reason = if imu_triggers_keyframe {
                            "IMU"
                        } else if t_sq > t_sq_thresh {
                            "translation"
                        } else if cos_r < cos_r_thresh {
                            "rotation"
                        } else {
                            "low_overlap"
                        };
                        // sqrt and acos only on the triggered-keyframe path (rare).
                        log::debug!(
                            "[Estimator] Keyframe triggered by {} (t={:.3}, r={:.3}, new_ratio={:.2})",
                            reason,
                            t_sq.sqrt(),
                            cos_r.clamp(-1.0, 1.0).acos(),
                            new_ratio
                        );
                        current_frame.is_keyframe = true;
                    } else {
                        current_frame.is_keyframe = false;
                    }
                    self.pose_log.push((timestamp_ns, T_W_B));
                    self.view_motion_tracking_results(&T_W_B);

                    // Record PnP pose (pre-BA) for visualising BA correction.
                    self.trajectory_pre_ba.push(T_W_B);
                    if let Some(v) = &mut self.viewer {
                        // Red: pre-BA PnP poses (raw, before bundle adjustment correction)
                        v.log_trajectory(&self.trajectory_pre_ba, "trajectory/pre_ba", Some([220, 80, 80]));

                        // Scalar metrics — useful time-series panels in Rerun.
                        let total_feats = current_frame.left_features.len() as f64;
                        v.log_scalar("metrics/feature_count", total_feats);
                        v.log_scalar("metrics/new_feature_ratio", new_ratio);
                        v.log_scalar("metrics/is_keyframe", if current_frame.is_keyframe { 1.0 } else { 0.0 });
                        v.log_scalar("metrics/window_size", self.sliding_window.len() as f64);
                    }
                }
                Ok(None) => {
                    log::warn!(
                        "[Estimator] Motion tracking failed (optimization did not converge)"
                    );
                    // Attempt relocalization using actual map points from BA
                    let reloc_result = self.relocalizer.relocalize_with_map_points(
                        &current_frame,
                        &self.sliding_window.map_points
                    );
                    if reloc_result.success {
                        if let Some(T_W_B) = reloc_result.T_W_B {
                            log::info!(
                                "[Relocalizer] Successfully recovered pose with {} inliers",
                                reloc_result.n_inliers
                            );
                            current_frame.state.T_W_B = T_W_B;
                            self.pose_log.push((timestamp_ns, T_W_B));
                            self.view_motion_tracking_results(&T_W_B);
                        }
                    } else {
                        log::warn!(
                            "[Relocalizer] Relocalization failed: {:?}",
                            reloc_result.error_message
                        );
                    }
                }
                Err(e) => {
                    log::error!("[Estimator] Motion tracking error: {:?}", e);
                    // Attempt relocalization on error too (using actual map points)
                    let reloc_result = self.relocalizer.relocalize_with_map_points(
                        &current_frame,
                        &self.sliding_window.map_points
                    );
                    if reloc_result.success {
                        if let Some(T_W_B) = reloc_result.T_W_B {
                            log::info!(
                                "[Relocalizer] Recovered pose after error with {} inliers",
                                reloc_result.n_inliers
                            );
                            current_frame.state.T_W_B = T_W_B;
                            self.pose_log.push((timestamp_ns, T_W_B));
                            self.view_motion_tracking_results(&T_W_B);
                        }
                    }
                }
            }
            motion_tracking_time_ms = motion_tracking_start.elapsed().as_secs_f64() * 1000.0;
        } else {
            // Window still filling up: every frame becomes a keyframe to populate the window.
            // This is the only place is_keyframe is forced true; all other paths set it
            // explicitly via the motion threshold check (true) or tracking failure (stays false).
            current_frame.is_keyframe = true;
            log::debug!("[Estimator] Sliding window not full ({}/{}), forcing keyframe",
                self.sliding_window.len(), self.config.keyframe_management.keyframe_window_size);
        }

        // View map points and keyframe poses
        // Bundle adjustment
        if current_frame.is_keyframe {
            let optimization_start = Instant::now();
            
            // Add to relocalization database before modifying the frame
            self.relocalizer.add_keyframe(&current_frame);

            self.sliding_window.add_frame(current_frame);
            // Reset the IMU accumulation buffer — the next KF will start fresh.
            self.imu_since_last_kf.clear();
            if let Err(e) = self.sliding_window.optimize() {
                log::warn!("[Estimator] Optimization failed: {}", e);
            }
            optimization_time_ms = optimization_start.elapsed().as_secs_f64() * 1000.0;

            // After BA, update pose_log with the BA-corrected newest KF pose.
            // The pre-BA PnP pose was already logged; replace it with the post-BA value
            // so the trajectory is consistent with the map that subsequent frames use.
            if let Some(newest_kf) = self.sliding_window.get_frame(self.sliding_window.len() - 1) {
                let ba_pose = newest_kf.state.T_W_B;
                if let Some(last_entry) = self.pose_log.last_mut() {
                    if last_entry.0 == timestamp_ns {
                        last_entry.1 = ba_pose;
                    }
                }
            }

            self.view_optimization_results();
        }

        // Final timing summary
        let total_duration_ms = total_start_time.elapsed().as_secs_f64() * 1000.0;

        // Track statistics for anomaly detection
        self.frame_times_ms.push(total_duration_ms);
        if motion_tracking_time_ms > 0.0 {
            self.motion_tracking_times_ms.push(motion_tracking_time_ms);
        }
        if optimization_time_ms > 0.0 {
            self.optimization_times_ms.push(optimization_time_ms);
        }

        // Log per-frame timing
        log::debug!(
            "[Timing] frame_creation={:.3} ms, patch_tracking={:.3} ms, motion_tracking={:.3} ms, optimization={:.3} ms, total={:.3} ms",
            frame_creation_time_ms,
            patch_tracking_time_ms,
            motion_tracking_time_ms,
            optimization_time_ms,
            total_duration_ms
        );

        // Every 10 frames, emit diagnostic statistics
        if self.frame_id_counter % 10 == 0 {
            let n_frames = self.frame_times_ms.len();
            if n_frames >= 10 {
                let recent_times = &self.frame_times_ms[n_frames.saturating_sub(10)..];
                let avg_frame_ms = recent_times.iter().sum::<f64>() / recent_times.len() as f64;
                let max_frame_ms = recent_times.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                let min_frame_ms = recent_times.iter().cloned().fold(f64::INFINITY, f64::min);

                log::info!(
                    "[Diagnostics] Frame {}: last 10 frames avg={:.1}ms, min={:.1}ms, max={:.1}ms (window_size={})",
                    self.frame_id_counter, avg_frame_ms, min_frame_ms, max_frame_ms, self.sliding_window.len()
                );

                // Anomaly detection: flag frames much slower than average
                if max_frame_ms > avg_frame_ms * 2.0 {
                    log::warn!(
                        "[Diagnostics] Slow frame detected: frame {} took {:.1}ms (avg {:.1}ms, >2x slower)",
                        self.frame_id_counter, max_frame_ms, avg_frame_ms
                    );
                }
            }

            // Motion tracking diagnostics
            if !self.motion_tracking_times_ms.is_empty() {
                let n_mt = self.motion_tracking_times_ms.len();
                let recent_mt = &self.motion_tracking_times_ms[n_mt.saturating_sub(5)..];
                let avg_mt_ms = recent_mt.iter().sum::<f64>() / recent_mt.len() as f64;
                log::debug!("[Diagnostics] Motion tracking avg (last 5): {:.1}ms", avg_mt_ms);
            }

            // Optimization diagnostics
            if !self.optimization_times_ms.is_empty() {
                let n_opt = self.optimization_times_ms.len();
                let recent_opt = &self.optimization_times_ms[n_opt.saturating_sub(5)..];
                let avg_opt_ms = recent_opt.iter().sum::<f64>() / recent_opt.len() as f64;
                log::debug!("[Diagnostics] Optimization avg (last 5): {:.1}ms", avg_opt_ms);
            }
        }

        // Warn if total frame time exceeds 100ms (timeout will be ~600s with 6000+ frames)
        if total_duration_ms > 100.0 {
            log::warn!(
                "[Diagnostics] SLOW FRAME: frame {} took {:.1}ms (>100ms threshold)",
                self.frame_id_counter, total_duration_ms
            );
        }

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
            // Use string literals directly — avoids two heap allocations per frame
            // ("pose_current".to_string() + format!(..._left)) that the original code did.
            v.log_pose(*T_W_B, "pose_current");

            let width = self.config.camera.image_width;
            let height = self.config.camera.image_height;

            // Log left camera frustum at the pose location (left camera is at the pose)
            let left_focal_length = self.config.camera.left_intrinsics[0] as f32;
            let T_W_Cl = T_W_B * self.T_B_Cl;
            v.log_pose(T_W_Cl, "pose_current_left");
            v.log_camera_frustum(
                left_focal_length,
                width,
                height,
                "pose_current_left",
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

            // History of keyframe poses: record newest KF (last in window) as current position.
            let mat = *system_poses
                .last()
                .expect("sliding window should have at least one keyframe");
            self.trajectory.push(mat);

            // Green: post-BA corrected trajectory (ground truth for subsequent frames)
            v.log_trajectory(&self.trajectory, "trajectory/path", Some([80, 220, 80]));
        }
    }
}
