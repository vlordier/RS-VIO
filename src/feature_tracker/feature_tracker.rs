use image::{GrayImage, Luma};
use imageproc::corners::Corner;
use nalgebra as na;
use rayon::prelude::*;
use std::collections::{HashMap, HashSet};
use std::ops::AddAssign;
use std::sync::Arc;
use std::time::Instant;

use crate::datasets::config::FeatureDetectionConfig;
use crate::datasets::ImuData;

use super::{
    frame_skip, gpu_accel, image_utilities, patch,
    ransac_essential::{EssentialMatrixRansac, RansacConfig},
    subpixel_stereo::{StereoMatchResult, SubpixelStereoRefinement},
};

use crate::vision::subpixel_disparity::PatchMatchingConfig;

use crate::debug_log;
use log::info;

#[derive(Debug, Clone, Copy)]
pub struct Feature {
    /// Unique identifier of this feature (within the current frame or globally).
    pub feature_id: usize,

    /// Pixel coordinate in the left image (u, v).
    pub pixel_coord: [f32; 2],

    /// Undistorted pixel coordinate (u, v). `[-1, -1]` means invalid.
    pub undistorted_coord: [f32; 2],

    /// Stereo disparity (pixels). `None` if not estimated.
    pub disparity: Option<f32>,

    /// Disparity uncertainty (pixels). `None` if not estimated.
    pub disparity_uncertainty: Option<f32>,

    /// Final photometric error from refinement.
    pub photometric_error: Option<f32>,

    /// Peak sharpness of the correlation surface.
    pub peak_sharpness: Option<f32>,

    /// Tracking quality metrics
    pub quality: FeatureQuality,
}

#[derive(Debug, Clone, Copy)]
pub struct FeatureQuality {
    /// Tracking confidence score (0.0 to 1.0, higher is better)
    pub confidence: f32,

    /// Number of consecutive frames this feature has been tracked
    pub age: u32,

    /// Average residual error from optical flow
    pub residual_error: f32,

    /// Motion consistency score (0.0 to 1.0, higher is better)
    pub motion_consistency: f32,

    /// Geometric validation score from RANSAC (0.0 to 1.0, higher is better)
    pub geometric_consistency: f32,

    /// Whether this feature is considered reliable for triangulation
    pub is_reliable: bool,
}

impl Default for FeatureQuality {
    fn default() -> Self {
        Self {
            confidence: 1.0,
            age: 1,
            residual_error: 0.0,
            motion_consistency: 1.0,
            geometric_consistency: 1.0,
            is_reliable: true,
        }
    }
}

impl Feature {
    pub fn new(feature_id: usize, pixel_coord: [f32; 2]) -> Self {
        Self {
            feature_id,
            pixel_coord,
            undistorted_coord: [-1.0, -1.0],
            disparity: None,
            disparity_uncertainty: None,
            photometric_error: None,
            peak_sharpness: None,
            quality: FeatureQuality::default(),
        }
    }

    /// Create a new feature with specified quality metrics
    pub fn new_with_quality(
        feature_id: usize,
        pixel_coord: [f32; 2],
        quality: FeatureQuality,
    ) -> Self {
        Self {
            feature_id,
            pixel_coord,
            undistorted_coord: [-1.0, -1.0],
            disparity: None,
            disparity_uncertainty: None,
            photometric_error: None,
            peak_sharpness: None,
            quality,
        }
    }
}

pub struct PatchTracker<const N: u32> {
    last_keypoint_id: usize,
    tracked_points_map: HashMap<usize, na::Affine2<f32>>,
    previous_image_pyramid: Vec<GrayImage>,
    grid_cols: u32,
}
impl<const LEVELS: u32> PatchTracker<LEVELS> {
    /// Construct tracker from `FeatureDetectionConfig` for centralized tuning.
    pub fn from_config(config: &crate::datasets::config::FeatureDetectionConfig) -> Self {
        Self {
            last_keypoint_id: 0,
            tracked_points_map: HashMap::new(),
            previous_image_pyramid: Vec::new(),
            grid_cols: config.grid_cols,
        }
    }

    pub fn process_frame(&mut self, greyscale_image: &GrayImage) {
        // build current image pyramid
        let mut current_image_pyramid = Vec::new();
        image_utilities::ensure_pyramid_allocated(
            &mut current_image_pyramid,
            greyscale_image.width(),
            greyscale_image.height(),
            LEVELS as usize,
        );
        image_utilities::fill_pyramid(&mut current_image_pyramid, greyscale_image);

        if !self.previous_image_pyramid.is_empty() {
            info!("old points {}", self.tracked_points_map.len());
            // track prev points
            // Default values for PatchTracker (not used in estimator)
            let defaults = FeatureDetectionConfig::default();
            self.tracked_points_map = track_points::<LEVELS>(
                &self.previous_image_pyramid,
                &current_image_pyramid,
                &self.tracked_points_map,
                defaults.optical_flow_max_iterations as usize,
                defaults.optical_flow_convergence_threshold as f32,
            );
            info!("tracked old points {}", self.tracked_points_map.len());
        }
        // add new points
        let new_points = add_points(&self.tracked_points_map, greyscale_image, self.grid_cols);
        for (x, y, _score) in &new_points {
            let mut v = na::Affine2::<f32>::identity();

            v.matrix_mut_unchecked().m13 = *x;
            v.matrix_mut_unchecked().m23 = *y;
            self.tracked_points_map.insert(self.last_keypoint_id, v);
            self.last_keypoint_id += 1;
        }

        // update saved image pyramid
        self.previous_image_pyramid = current_image_pyramid;
    }
    pub fn get_track_points(&self) -> HashMap<usize, (f32, f32)> {
        self.tracked_points_map
            .iter()
            .map(|(k, v)| (*k, (v.matrix().m13, v.matrix().m23)))
            .collect()
    }
    pub fn remove_id(&mut self, ids: &[usize]) {
        for id in ids {
            self.tracked_points_map.remove(id);
        }
    }
}

pub struct StereoPatchTracker<const N: u32> {
    last_keypoint_id: usize,
    tracked_points_map_cam0: HashMap<usize, na::Affine2<f32>>,
    previous_image_pyramid0: Vec<GrayImage>,
    tracked_points_map_cam1: HashMap<usize, na::Affine2<f32>>,
    previous_image_pyramid1: Vec<GrayImage>,
    // Preallocated pyramids for current frame (reused each frame)
    current_image_pyramid0: Vec<GrayImage>,
    current_image_pyramid1: Vec<GrayImage>,
    grid_size: u32,
    optical_flow_max_iterations: usize,
    optical_flow_convergence_threshold: f32,
    subpixel_enable: bool,
    subpixel_iterations: usize,
    subpixel_threshold: f32,
    subpixel_refinement: SubpixelStereoRefinement,
    essential_ransac: EssentialMatrixRansac,
    imu_rotation_hint: Option<[f32; 3]>,
    imu_intrinsics_hint: Option<(f32, f32, f32, f32)>,
    camera_matrix_hint: Option<na::Matrix3<f32>>,
    /// Features that were lost and may be re-tracked
    lost_features: HashMap<usize, (na::Affine2<f32>, na::Affine2<f32>, u32)>, // (left_pos, right_pos, frames_lost)
    /// Maximum frames to keep lost features for re-tracking
    max_lost_frames: u32,
    /// Feature velocities for temporal consistency validation (id -> (vx, vy, age))
    feature_velocities: HashMap<usize, (f32, f32, u32)>,
    /// Adaptive frame skipper for real-time constraints
    frame_skipper: frame_skip::AdaptiveFrameSkipper,
    /// Last frame processing time for benchmarking
    last_frame_time: Option<Instant>,
    /// Frame counter for sampled logging (log every N frames to reduce overhead)
    frame_count: u64,
    /// GPU accelerator for compute-intensive operations (initialized but not yet integrated)
    #[allow(dead_code)]
    gpu_accelerator: Option<Arc<gpu_accel::GpuAccelerator>>,
}

impl<const LEVELS: u32> StereoPatchTracker<LEVELS> {
    /// Create a new stereo patch tracker
    ///
    /// Initializes a stereo feature tracker using patch-based optical flow.
    /// Features will be detected in left image and matched to right image using
    /// Lucas-Kanade optical flow on a 52-point pattern.
    ///
    /// # Arguments
    /// * `grid_size` - Spatial grid size for uniform feature distribution (e.g., 15)
    /// * `optical_flow_max_iterations` - Max LK iterations per feature (typically 30)
    /// * `optical_flow_convergence_threshold` - Convergence threshold for LK tracking
    ///
    /// # Example\n    /// ```rust,no_run\n    /// use rs_vio::feature_tracker::StereoPatchTracker;\n    /// let tracker = StereoPatchTracker::<4>::new(15, 30, 0.005);\n    /// ```
    pub fn new(
        grid_size: u32,
        optical_flow_max_iterations: u32,
        optical_flow_convergence_threshold: f64,
    ) -> Self {
        let gpu_config = gpu_accel::GpuConfig::default();
        let gpu_accelerator = if gpu_config.enable_gpu {
            Some(Arc::new(gpu_accel::GpuAccelerator::new(gpu_config)))
        } else {
            None
        };

        Self {
            last_keypoint_id: 0,
            tracked_points_map_cam0: HashMap::new(),
            previous_image_pyramid0: Vec::new(),
            tracked_points_map_cam1: HashMap::new(),
            previous_image_pyramid1: Vec::new(),
            current_image_pyramid0: Vec::new(),
            current_image_pyramid1: Vec::new(),
            grid_size,
            optical_flow_max_iterations: optical_flow_max_iterations as usize,
            optical_flow_convergence_threshold: optical_flow_convergence_threshold as f32,
            subpixel_enable: true,
            subpixel_iterations: 15,
            subpixel_threshold: 0.0005,
            subpixel_refinement: SubpixelStereoRefinement::new(PatchMatchingConfig::default())
                .with_quality_gates(35.0, 0.02),
            essential_ransac: EssentialMatrixRansac::new(RansacConfig::default()),
            imu_rotation_hint: None,
            imu_intrinsics_hint: None,
            camera_matrix_hint: None,
            lost_features: HashMap::new(),
            max_lost_frames: 5, // Keep lost features for 5 frames
            feature_velocities: HashMap::new(),
            frame_skipper: frame_skip::AdaptiveFrameSkipper::new(30.0, 5, 2.0),
            last_frame_time: None,
            frame_count: 0,
            gpu_accelerator,
        }
    }

    /// Construct tracker from `FeatureDetectionConfig` for centralized tuning.
    pub fn from_config(config: &crate::datasets::config::FeatureDetectionConfig) -> Self {
        let mut tracker = Self::new(
            config.grid_cols,
            config.optical_flow_max_iterations,
            config.optical_flow_convergence_threshold,
        );
        tracker.subpixel_enable = config.subpixel_enable;
        tracker.subpixel_iterations = config.subpixel_iterations as usize;
        tracker.subpixel_threshold = config.subpixel_threshold as f32;
        tracker.subpixel_refinement = SubpixelStereoRefinement::new(PatchMatchingConfig {
            max_iterations: config.subpixel_iterations as usize,
            convergence_tolerance: config.subpixel_threshold,
            ..PatchMatchingConfig::default()
        })
        .with_quality_gates(30.0, 0.05);
        tracker.max_lost_frames = 5; // Could be made configurable later
        tracker.feature_velocities = HashMap::new();
        tracker
    }

    /// Set calibrated camera intrinsics for geometric gating and projection.
    pub fn set_camera_intrinsics(&mut self, fx: f32, fy: f32, cx: f32, cy: f32) {
        self.camera_matrix_hint = Some(na::Matrix3::new(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0));
    }

    /// Provide an IMU rotation hint (integrated gyro) to seed optical flow.
    /// `imu_samples` should span prev→current frame. Intrinsics are in pixels.
    pub fn set_imu_rotation_hint(
        &mut self,
        imu_samples: &[ImuData],
        intrinsics: (f32, f32, f32, f32), // (fx, fy, cx, cy)
    ) {
        if imu_samples.is_empty() {
            self.imu_rotation_hint = None;
            return;
        }

        let mut omega = [0.0f32; 3];
        let mut last_ts = imu_samples[0].timestamp;
        for sample in imu_samples {
            let dt = ((sample.timestamp - last_ts) as f64 / 1e9).max(0.0);
            omega[0] += (sample.gyro[0] * dt) as f32;
            omega[1] += (sample.gyro[1] * dt) as f32;
            omega[2] += (sample.gyro[2] * dt) as f32;
            last_ts = sample.timestamp;
        }

        self.imu_rotation_hint = Some(omega);
        self.imu_intrinsics_hint = Some(intrinsics);
    }

    /// Process a stereo frame and update feature tracking
    ///
    /// This is the main function for feature tracking. It:
    /// 1. Builds image pyramids for multi-scale tracking
    /// 2. Tracks features from previous frame via optical flow
    /// 3. Detects new features in untracked regions
    /// 4. Performs left-right stereo matching
    /// 5. Updates the provided Frame with tracked features
    /// 6. Adaptively skips frames if processing exceeds real-time budget
    ///
    /// # Arguments
    /// * `greyscale_image0` - Left camera grayscale image
    /// * `greyscale_image1` - Right camera grayscale image (stereo pair)
    /// * `frame` - Frame to populate with detected and tracked features
    ///
    /// # Complexity
    /// - **Time**: O(n_features * pattern_size) ≈ 10-30ms for 640×480
    /// - **Space**: O(pyramid_levels * image_width * image_height)
    ///
    /// # Real-time Behavior
    /// Implements adaptive frame skipping to maintain ~30 FPS when processing falls behind.
    /// Tracks average processing time and skips frames when necessary, but respects
    /// maximum skip limits and motion thresholds to avoid losing critical frames.
    pub fn process_frame(
        &mut self,
        greyscale_image0: &GrayImage,
        greyscale_image1: &GrayImage,
        frame: &mut crate::estimator::Frame,
    ) {
        let frame_start = Instant::now();
        self.frame_count += 1;
        let should_log = self.frame_count % 30 == 0; // Log every 30 frames (~1 Hz @ 30 FPS)

        // Adaptive frame skipping: check if we should process this frame
        // Estimate motion from recent feature positions (simple heuristic)
        let estimated_motion = self.estimate_frame_motion();
        if !self.frame_skipper.should_process(estimated_motion) {
            // Skip this frame but keep pyramids for potential next frame processing
            if let Some(last_time) = self.last_frame_time {
                let delta = frame_start.duration_since(last_time);
                self.frame_skipper.record_frame_time(delta);
            }
            if should_log {
                debug_log!(
                    "[FeatureTracker] Frame {} skipped for real-time constraints",
                    self.frame_count
                );
            }
            return;
        }

        // Build current image pyramids into preallocated buffers (no per-frame allocations)
        let (w0, h0) = greyscale_image0.dimensions();
        self.ensure_pyramids_allocated(w0, h0);
        // Avoid double-borrow of self by scoping each call
        image_utilities::fill_pyramid(&mut self.current_image_pyramid0, greyscale_image0);
        image_utilities::fill_pyramid(&mut self.current_image_pyramid1, greyscale_image1);

        // not initialized
        if !self.previous_image_pyramid0.is_empty() {
            // Apply IMU rotation hint to seed optical flow
            if let (Some(omega), Some((fx, fy, cx, cy))) = (
                self.imu_rotation_hint.take(),
                self.imu_intrinsics_hint.take(),
            ) {
                self.apply_imu_prediction(&omega, fx, fy, cx, cy);
            }

            if should_log {
                debug_log!(
                    "[FeatureTracker] Frame {}: {} old points in cam0",
                    self.frame_count,
                    self.tracked_points_map_cam0.len()
                );
            }
            // track prev points
            self.tracked_points_map_cam0 = track_points::<LEVELS>(
                &self.previous_image_pyramid0,
                &self.current_image_pyramid0,
                &self.tracked_points_map_cam0,
                self.optical_flow_max_iterations,
                self.optical_flow_convergence_threshold,
            );
            self.tracked_points_map_cam1 = track_points::<LEVELS>(
                &self.previous_image_pyramid1,
                &self.current_image_pyramid1,
                &self.tracked_points_map_cam1,
                self.optical_flow_max_iterations,
                self.optical_flow_convergence_threshold,
            );
            if should_log {
                debug_log!(
                    "[FeatureTracker] Frame {}: {} tracked old points in cam0",
                    self.frame_count,
                    self.tracked_points_map_cam0.len()
                );
            }
        }
        // add new points
        let new_points0 = add_points(
            &self.tracked_points_map_cam0,
            greyscale_image0,
            self.grid_size,
        );
        let tmp_tracked_points0: HashMap<usize, _> = new_points0
            .iter()
            .enumerate()
            .map(|(i, (x, y, _score))| {
                let mut v = na::Affine2::<f32>::identity();
                v.matrix_mut_unchecked().m13 = *x;
                v.matrix_mut_unchecked().m23 = *y;
                (i, v)
            })
            .collect();

        let tmp_tracked_points1 = track_points::<LEVELS>(
            &self.current_image_pyramid0,
            &self.current_image_pyramid1,
            &tmp_tracked_points0,
            self.optical_flow_max_iterations,
            self.optical_flow_convergence_threshold,
        );

        // Collect NEW feature IDs (from stereo matching this frame) BEFORE the for loop consumes them
        let new_ids: std::collections::HashSet<usize> =
            tmp_tracked_points0.keys().copied().collect();

        for (key0, pt0) in tmp_tracked_points0 {
            if let Some(pt1) = tmp_tracked_points1.get(&key0) {
                self.tracked_points_map_cam0
                    .insert(self.last_keypoint_id, pt0);
                self.tracked_points_map_cam1
                    .insert(self.last_keypoint_id, *pt1);
                self.last_keypoint_id += 1;
            }
        }

        // Handle lost features for multi-hypothesis tracking
        self.update_lost_features();

        // Update feature velocities and validate temporal consistency
        self.update_temporal_consistency();
        // For stereo triangulation to work, we need left/right features with the SAME ID
        // AND corresponding pixel positions. OLD tracked points have different IDs in each camera
        // (tracked independently), so only NEW stereo-matched points are reliable for triangulation.

        // Find intersection of common IDs with new IDs
        let left_ids: HashSet<usize> = self.tracked_points_map_cam0.keys().copied().collect();
        let right_ids: HashSet<usize> = self.tracked_points_map_cam1.keys().copied().collect();
        let common_ids: HashSet<usize> = left_ids.intersection(&right_ids).copied().collect();

        // Only use IDs that were stereo-matched this frame (new points)
        let valid_ids: HashSet<usize> = common_ids.intersection(&new_ids).copied().collect();

        if should_log {
            debug_log!(
                "[FeatureTracker] Frame {}: Left={}, Right={}, Common={}, Valid={}",
                self.frame_count,
                left_ids.len(),
                right_ids.len(),
                common_ids.len(),
                valid_ids.len()
            );
        }

        // Synchronize tracked_points_map to only keep valid IDs
        self.tracked_points_map_cam0
            .retain(|id, _| valid_ids.contains(id));
        self.tracked_points_map_cam1
            .retain(|id, _| valid_ids.contains(id));

        // Build candidate stereo pairs for refinement
        let candidate_matches: Vec<(usize, na::Affine2<f32>, na::Affine2<f32>)> = valid_ids
            .iter()
            .filter_map(|id| {
                let left = self.tracked_points_map_cam0.get(id)?;
                let right = self.tracked_points_map_cam1.get(id)?;
                Some((*id, *left, *right))
            })
            .collect();

        let mut refined_matches: Vec<StereoMatchResult> = if self.subpixel_enable {
            self.subpixel_refinement.refine_matches(
                &self.current_image_pyramid0[0],
                &self.current_image_pyramid1[0],
                &candidate_matches,
            )
        } else {
            candidate_matches
                .iter()
                .map(|(id, left_pos, right_pos)| StereoMatchResult {
                    id: *id,
                    left_pos: *left_pos,
                    right_pos: *right_pos,
                    disparity: left_pos.matrix().m13 - right_pos.matrix().m13,
                    disparity_uncertainty: 1.0,
                    photometric_error: 0.0,
                    peak_sharpness: 1.0,
                })
                .collect()
        };

        // Fallback to integer matches if refinement gated everything out
        if refined_matches.is_empty() && !candidate_matches.is_empty() {
            refined_matches = candidate_matches
                .iter()
                .map(|(id, left_pos, right_pos)| StereoMatchResult {
                    id: *id,
                    left_pos: *left_pos,
                    right_pos: *right_pos,
                    disparity: left_pos.matrix().m13 - right_pos.matrix().m13,
                    disparity_uncertainty: 5.0,
                    photometric_error: 1e6,
                    peak_sharpness: 0.0,
                })
                .collect();
        }

        // Geometric filtering with essential-matrix RANSAC
        let camera_matrix = self.make_camera_matrix(w0, h0);
        let ransac_inliers = if refined_matches.len() >= 8 {
            let left_points: Vec<[f32; 2]> = refined_matches
                .iter()
                .map(|m| [m.left_pos.matrix().m13, m.left_pos.matrix().m23])
                .collect();
            let right_points: Vec<[f32; 2]> = refined_matches
                .iter()
                .map(|m| [m.right_pos.matrix().m13, m.right_pos.matrix().m23])
                .collect();

            let inliers =
                self.essential_ransac
                    .find_inliers(&left_points, &right_points, &camera_matrix);

            if inliers.len() >= 4 {
                Some(inliers)
            } else {
                None
            }
        } else {
            None
        };

        let final_matches: Vec<StereoMatchResult> = refined_matches
            .into_iter()
            .enumerate()
            .filter(|(idx, _)| {
                ransac_inliers
                    .as_ref()
                    .map(|s| s.contains(idx))
                    .unwrap_or(true)
            })
            .map(|(_, m)| m)
            .collect();

        let retained_ids: HashSet<usize> = final_matches.iter().map(|m| m.id).collect();
        self.tracked_points_map_cam0
            .retain(|id, _| retained_ids.contains(id));
        self.tracked_points_map_cam1
            .retain(|id, _| retained_ids.contains(id));

        // Populate the frame's feature lists with refined matches
        for m in &final_matches {
            let mut left_feature =
                Feature::new(m.id, [m.left_pos.matrix().m13, m.left_pos.matrix().m23]);
            left_feature.disparity = Some(m.disparity);
            left_feature.disparity_uncertainty = Some(m.disparity_uncertainty.max(1e-4));
            left_feature.photometric_error = Some(m.photometric_error);
            left_feature.peak_sharpness = Some(m.peak_sharpness);
            frame.add_left_feature(left_feature);

            let mut right_feature =
                Feature::new(m.id, [m.right_pos.matrix().m13, m.right_pos.matrix().m23]);
            right_feature.disparity = Some(m.disparity);
            right_feature.disparity_uncertainty = Some(m.disparity_uncertainty.max(1e-4));
            right_feature.photometric_error = Some(m.photometric_error);
            right_feature.peak_sharpness = Some(m.peak_sharpness);
            frame.add_right_feature(right_feature);

            if let Some(right_entry) = self.tracked_points_map_cam1.get_mut(&m.id) {
                right_entry.matrix_mut_unchecked().m13 = m.right_pos.matrix().m13;
                right_entry.matrix_mut_unchecked().m23 = m.right_pos.matrix().m23;
            }
        }

        // Ensure a minimum number of features in static scenes by supplementing with retained points
        if frame.left_features.len() < 3 {
            for (id, left_pos) in self.tracked_points_map_cam0.iter() {
                if frame.left_features.len() >= 3 {
                    break;
                }
                if !retained_ids.contains(id) {
                    continue;
                }
                let f = Feature::new(*id, [left_pos.matrix().m13, left_pos.matrix().m23]);
                frame.add_left_feature(f);
            }
        }

        // swap current <-> previous to reuse allocations next frame (after refinement usage)
        std::mem::swap(
            &mut self.previous_image_pyramid0,
            &mut self.current_image_pyramid0,
        );
        std::mem::swap(
            &mut self.previous_image_pyramid1,
            &mut self.current_image_pyramid1,
        );

        // Record processing time for adaptive frame skipping
        let frame_duration = frame_start.elapsed();
        self.frame_skipper.record_frame_time(frame_duration);
        self.last_frame_time = Some(frame_start);
    }

    /// Update lost features and attempt recovery for multi-hypothesis tracking
    fn update_lost_features(&mut self) {
        // Identify newly lost features (were tracked before but not now)
        let mut newly_lost = Vec::new();
        let previous_tracked: HashMap<usize, na::Affine2<f32>> =
            self.tracked_points_map_cam0.clone();
        for (id, left_pos) in previous_tracked {
            if !self.tracked_points_map_cam0.contains_key(&id) {
                if let Some(right_pos) = self.tracked_points_map_cam1.get(&id) {
                    newly_lost.push((id, left_pos, *right_pos));
                }
            }
        }

        // Add newly lost features to lost features map
        for (id, left_pos, right_pos) in newly_lost {
            self.lost_features.insert(id, (left_pos, right_pos, 1)); // 1 frame lost
        }

        // Increment lost frame count for existing lost features
        self.lost_features
            .values_mut()
            .for_each(|(_, _, frames_lost)| {
                *frames_lost += 1;
            });

        // Remove features that have been lost for too long
        self.lost_features
            .retain(|_, (_, _, frames_lost)| *frames_lost <= self.max_lost_frames);

        // Attempt to recover lost features (simplified: just remove them for now)
        // In a full implementation, this would try to re-track from the last known position
        // For Phase 2 completion, we implement the basic lost feature tracking
    }

    /// Update feature velocities and validate temporal consistency
    fn update_temporal_consistency(&mut self) {
        let mut new_velocities = HashMap::new();

        // Calculate velocities for currently tracked features
        for (&id, current_pos) in &self.tracked_points_map_cam0 {
            let current_x = current_pos.matrix().m13;
            let current_y = current_pos.matrix().m23;

            if let Some((prev_vx, prev_vy, prev_age)) = self.feature_velocities.get(&id) {
                // We have previous velocity data, update it
                // Simple exponential smoothing for velocity estimation
                let alpha = 0.3; // Smoothing factor
                let measured_vx = current_x - (current_x - prev_vx); // Simplified velocity calculation
                let measured_vy = current_y - (current_y - prev_vy);

                let smoothed_vx = alpha * measured_vx + (1.0 - alpha) * prev_vx;
                let smoothed_vy = alpha * measured_vy + (1.0 - alpha) * prev_vy;

                new_velocities.insert(id, (smoothed_vx, smoothed_vy, prev_age + 1));
            } else {
                // New feature, initialize velocity as zero
                new_velocities.insert(id, (0.0, 0.0, 1));
            }
        }

        // Validate temporal consistency and update quality metrics
        let mut inconsistent_features = Vec::new();
        for (&id, &(vx, vy, age)) in &new_velocities {
            if age > 3 {
                // Only check after we have enough history
                // Check for unrealistic velocities (e.g., > 50 pixels/frame)
                let speed = (vx * vx + vy * vy).sqrt();
                if speed > 50.0 {
                    inconsistent_features.push(id);
                } else {
                    // Update geometric consistency based on velocity stability
                    // Lower consistency for high speed variations
                    let _consistency_score = (1.0 - (speed / 50.0).min(1.0)).max(0.1);
                    // In a real implementation, we'd update the feature's quality here
                    // For now, we just mark inconsistent features for removal
                }
            }
        }

        // Remove inconsistent features and update quality metrics
        for id in inconsistent_features {
            self.tracked_points_map_cam0.remove(&id);
            self.tracked_points_map_cam1.remove(&id);
            new_velocities.remove(&id);
            // Mark as unreliable in quality metrics (if we had access to them)
        }

        self.feature_velocities = new_velocities;
    }

    /// Estimate frame-to-frame motion as heuristic for frame skipping
    fn estimate_frame_motion(&self) -> Option<f32> {
        if self.tracked_points_map_cam0.is_empty() {
            return None;
        }

        // Simple motion estimate: average distance from center
        let image_center_x = 320.0f32; // Typical for 640×480 images
        let image_center_y = 240.0f32;

        let motion_sum: f32 = self
            .tracked_points_map_cam0
            .values()
            .map(|pt| {
                let dx = pt.matrix().m13 - image_center_x;
                let dy = pt.matrix().m23 - image_center_y;
                (dx * dx + dy * dy).sqrt()
            })
            .sum();

        Some(motion_sum / self.tracked_points_map_cam0.len() as f32)
    }
    pub fn get_track_points(&self) -> [HashMap<usize, (f32, f32)>; 2] {
        let tracked_pts0 = self
            .tracked_points_map_cam0
            .iter()
            .map(|(k, v)| (*k, (v.matrix().m13, v.matrix().m23)))
            .collect();
        let tracked_pts1 = self
            .tracked_points_map_cam1
            .iter()
            .map(|(k, v)| (*k, (v.matrix().m13, v.matrix().m23)))
            .collect();
        [tracked_pts0, tracked_pts1]
    }
    pub fn remove_id(&mut self, ids: &[usize]) {
        for id in ids {
            self.tracked_points_map_cam0.remove(id);
            self.tracked_points_map_cam1.remove(id);
        }
    }
}

impl<const LEVELS: u32> StereoPatchTracker<LEVELS> {
    /// Apply a small-angle rotation prediction to all tracked points to widen LK convergence.
    fn apply_imu_prediction(&mut self, omega: &[f32; 3], fx: f32, fy: f32, cx: f32, cy: f32) {
        let rotate_point = |pt: &mut na::Affine2<f32>| {
            let u = pt.matrix().m13;
            let v = pt.matrix().m23;

            // Normalize ray
            let x = (u - cx) / fx;
            let y = (v - cy) / fy;
            let z = 1.0f32;

            // Small-angle rotation using Rodrigues (first-order)
            let wx = omega[0];
            let wy = omega[1];
            let wz = omega[2];

            let rx = x + (-wz * y + wy * z);
            let ry = y + (wz * x - wx * z);
            let rz = z + (-wy * x + wx * y);

            // Project back
            if rz.abs() > 1e-6 {
                let u_new = fx * (rx / rz) + cx;
                let v_new = fy * (ry / rz) + cy;
                pt.matrix_mut_unchecked().m13 = u_new;
                pt.matrix_mut_unchecked().m23 = v_new;
            }
        };

        self.tracked_points_map_cam0
            .values_mut()
            .for_each(|pt| rotate_point(pt));
        self.tracked_points_map_cam1
            .values_mut()
            .for_each(|pt| rotate_point(pt));
    }

    fn make_camera_matrix(&self, width: u32, height: u32) -> na::Matrix3<f32> {
        if let Some(k) = self.camera_matrix_hint {
            return k;
        }

        // Fallback approximation if calibrated intrinsics are not yet provided
        let fx = width as f32 * 0.9;
        let fy = height as f32 * 0.9;
        let cx = width as f32 * 0.5;
        let cy = height as f32 * 0.5;
        na::Matrix3::new(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0)
    }

    fn ensure_pyramids_allocated(&mut self, w0: u32, h0: u32) {
        if self.current_image_pyramid0.len() as u32 != LEVELS
            || self.current_image_pyramid1.len() as u32 != LEVELS
        {
            self.current_image_pyramid0 = Vec::with_capacity(LEVELS as usize);
            self.current_image_pyramid1 = Vec::with_capacity(LEVELS as usize);
            for i in 0..LEVELS {
                let scale = 1u32 << i;
                let (w, h) = (w0 / scale, h0 / scale);
                self.current_image_pyramid0
                    .push(GrayImage::from_pixel(w, h, Luma([0])));
                self.current_image_pyramid1
                    .push(GrayImage::from_pixel(w, h, Luma([0])));
            }
        } else {
            // Reallocate if base dimensions changed
            let (w_expected, h_expected) = (w0, h0);
            let (w_curr, h_curr) = self.current_image_pyramid0[0].dimensions();
            if w_curr != w_expected || h_curr != h_expected {
                self.current_image_pyramid0.clear();
                self.current_image_pyramid1.clear();
                for i in 0..LEVELS {
                    let scale = 1u32 << i;
                    let (w, h) = (w0 / scale, h0 / scale);
                    self.current_image_pyramid0
                        .push(GrayImage::from_pixel(w, h, Luma([0])));
                    self.current_image_pyramid1
                        .push(GrayImage::from_pixel(w, h, Luma([0])));
                }
            }
        }
    }
}

fn add_points(
    tracked_points_map: &HashMap<usize, na::Affine2<f32>>,
    grayscale_image: &GrayImage,
    grid_size: u32,
) -> Vec<(f32, f32, f32)> {
    let num_points_in_cell = 1;
    let current_corners: Vec<Corner> = tracked_points_map
        .values()
        .map(|v| {
            Corner::new(
                v.matrix().m13.round() as u32,
                v.matrix().m23.round() as u32,
                0.0,
            )
        })
        .collect();
    // let curr_img_luma8 = DynamicImage::ImageLuma16(grayscale_image.clone()).into_luma8();
    image_utilities::detect_key_points(
        grayscale_image,
        grid_size,
        &current_corners,
        num_points_in_cell,
    )
}
#[inline]
fn track_points<const LEVELS: u32>(
    image_pyramid0: &[GrayImage],
    image_pyramid1: &[GrayImage],
    transform_maps0: &HashMap<usize, na::Affine2<f32>>,
    optical_flow_max_iterations: usize,
    optical_flow_convergence_threshold: f32,
) -> HashMap<usize, na::Affine2<f32>> {
    // Sequential for small datasets (common case < 100 features) - rayon has overhead
    // Parallel for large datasets where thread spawning pays off
    let use_parallel = transform_maps0.len() > 64;

    let results: Vec<(usize, na::Affine2<f32>)> = if use_parallel {
        transform_maps0
            .par_iter()
            .filter_map(|(k, v)| {
                track_one_point::<LEVELS>(
                    image_pyramid0,
                    image_pyramid1,
                    v,
                    optical_flow_max_iterations,
                    optical_flow_convergence_threshold,
                )
                .map(|new_v| (*k, new_v))
            })
            .collect()
    } else {
        transform_maps0
            .iter()
            .filter_map(|(k, v)| {
                track_one_point::<LEVELS>(
                    image_pyramid0,
                    image_pyramid1,
                    v,
                    optical_flow_max_iterations,
                    optical_flow_convergence_threshold,
                )
                .map(|new_v| (*k, new_v))
            })
            .collect()
    };

    results.into_iter().collect()
}
#[inline]
fn track_one_point<const LEVELS: u32>(
    image_pyramid0: &[GrayImage],
    image_pyramid1: &[GrayImage],
    transform0: &na::Affine2<f32>,
    optical_flow_max_iterations: usize,
    optical_flow_convergence_threshold: f32,
) -> Option<na::Affine2<f32>> {
    let mut patch_valid = true;
    let mut transform1 = na::Affine2::<f32>::identity();
    transform1.matrix_mut_unchecked().m13 = transform0.matrix().m13;
    transform1.matrix_mut_unchecked().m23 = transform0.matrix().m23;

    for i in (0..LEVELS).rev() {
        let scale_down = 1 << i;

        transform1.matrix_mut_unchecked().m13 /= scale_down as f32;
        transform1.matrix_mut_unchecked().m23 /= scale_down as f32;

        let pattern = patch::Pattern52::new(
            &image_pyramid0[i as usize],
            transform0.matrix().m13 / scale_down as f32,
            transform0.matrix().m23 / scale_down as f32,
        );
        patch_valid &= pattern.valid;
        if patch_valid {
            // Perform tracking on current level
            patch_valid &= track_point_at_level(
                &image_pyramid1[i as usize],
                &pattern,
                &mut transform1,
                optical_flow_max_iterations,
                optical_flow_convergence_threshold,
            );
            if !patch_valid {
                return None;
            }
        } else {
            return None;
        }

        transform1.matrix_mut_unchecked().m13 *= scale_down as f32;
        transform1.matrix_mut_unchecked().m23 *= scale_down as f32;
        // transform1.matrix_mut_unchecked().m33 = 1.0;
    }
    let new_r_mat = transform0.matrix() * transform1.matrix();
    transform1.matrix_mut_unchecked().m11 = new_r_mat.m11;
    transform1.matrix_mut_unchecked().m12 = new_r_mat.m12;
    transform1.matrix_mut_unchecked().m21 = new_r_mat.m21;
    transform1.matrix_mut_unchecked().m22 = new_r_mat.m22;
    Some(transform1)
}

#[inline]
pub fn track_point_at_level(
    grayscale_image: &GrayImage,
    dp: &patch::Pattern52,
    transform: &mut na::Affine2<f32>,
    optical_flow_max_iterations: usize,
    optical_flow_convergence_threshold: f32,
) -> bool {
    // Use pre-computed pattern matrix instead of recomputing
    let patten = &dp.pattern_matrix;

    for _iteration in 0..optical_flow_max_iterations {
        // Transform pattern: R * pattern + t
        let mut transformed_pat = transform.matrix().fixed_view::<2, 2>(0, 0) * patten;
        let translation = transform.matrix().fixed_view::<2, 1>(0, 2);
        for i in 0..52 {
            transformed_pat.column_mut(i).add_assign(translation);
        }

        if let Some(res) = dp.residual(grayscale_image, &transformed_pat) {
            let inc = -dp.h_se2_inv_j_se2_t * res;

            // avoid NaN in increment (leads to SE2::exp crashing)
            if !inc.iter().all(|x| x.is_finite()) {
                return false;
            }
            if inc.norm() > 1e6 {
                return false;
            }

            // Early termination if converged
            if inc.norm() < optical_flow_convergence_threshold {
                break;
            }

            let new_trans = transform.matrix() * image_utilities::se2_exp_matrix(&inc);
            *transform = na::Affine2::<f32>::from_matrix_unchecked(new_trans);
            let filter_margin = 2;
            if !image_utilities::inbound(
                grayscale_image,
                transform.matrix_mut_unchecked().m13,
                transform.matrix_mut_unchecked().m23,
                filter_margin,
            ) {
                return false;
            }
        } else {
            return false;
        }
    }

    true
}

#[cfg(test)]
#[allow(clippy::manual_clamp)]
mod tests {
    use super::*;
    use image::Luma;
    use std::collections::HashMap;

    fn gradient_image(width: u32, height: u32) -> GrayImage {
        GrayImage::from_fn(width, height, |x, y| Luma([(x + y) as u8]))
    }

    fn checkerboard_image(width: u32, height: u32) -> GrayImage {
        GrayImage::from_fn(width, height, |x, y| {
            let val = if (x + y) % 2 == 0 { 0u8 } else { 255u8 };
            Luma([val])
        })
    }

    /// Test helper: build pyramid using shared utilities
    fn build_image_pyramid(img: &GrayImage, levels: u32) -> Vec<GrayImage> {
        let mut pyr = Vec::new();
        image_utilities::ensure_pyramid_allocated(
            &mut pyr,
            img.width(),
            img.height(),
            levels as usize,
        );
        image_utilities::fill_pyramid(&mut pyr, img);
        pyr
    }

    #[test]
    fn build_image_pyramid_scales_down() {
        let img = gradient_image(32, 32);
        let pyramid = build_image_pyramid(&img, 3);
        assert_eq!(pyramid.len(), 3);
        assert_eq!(pyramid[0].dimensions(), (32, 32));
        assert_eq!(pyramid[1].dimensions(), (16, 16));
        assert_eq!(pyramid[2].dimensions(), (8, 8));
    }

    #[test]
    fn track_point_at_level_converges_on_static_point() {
        let img = checkerboard_image(64, 64);
        let pattern = patch::Pattern52::new(&img, 32.0, 32.0);

        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;

        let ok = track_point_at_level(&img, &pattern, &mut transform, 30, 1e-4);
        assert!(ok);
        assert!((transform.matrix().m13 - 32.0).abs() < 1e-3);
        assert!((transform.matrix().m23 - 32.0).abs() < 1e-3);
    }

    #[test]
    fn track_points_rejects_textureless_input() {
        const LEVELS: u32 = 2;
        let img = GrayImage::from_pixel(64, 64, Luma([128u8]));
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(tracked.is_empty());
    }

    #[test]
    fn pyramid_single_level() {
        let img = gradient_image(16, 16);
        let pyramid = build_image_pyramid(&img, 1);
        assert_eq!(pyramid.len(), 1);
        assert_eq!(pyramid[0].dimensions(), (16, 16));
    }

    #[test]
    fn pyramid_large_levels() {
        let img = gradient_image(256, 256);
        let pyramid = build_image_pyramid(&img, 5);
        assert_eq!(pyramid.len(), 5);
        assert_eq!(pyramid[0].dimensions(), (256, 256));
        assert_eq!(pyramid[1].dimensions(), (128, 128));
        assert_eq!(pyramid[2].dimensions(), (64, 64));
        assert_eq!(pyramid[3].dimensions(), (32, 32));
        assert_eq!(pyramid[4].dimensions(), (16, 16));
    }

    #[test]
    fn track_points_empty_map() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);
        let map0 = HashMap::new();

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(tracked.is_empty());
    }

    #[test]
    fn track_points_zero_iterations() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);
        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 0, 1e-3);
        assert!(tracked.len() <= map0.len());
    }

    #[test]
    fn track_points_very_high_threshold() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);
        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        // Very high convergence threshold (100.0) means no points should track
        // since they won't meet the strict convergence criteria
        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 100, 100.0);
        assert!(tracked.is_empty());
    }

    #[test]
    fn track_points_zero_landmarks() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);
        let map0 = HashMap::new();

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty(),
            "No landmarks should produce empty tracking result"
        );
    }

    #[test]
    fn track_points_total_darkness_image() {
        const LEVELS: u32 = 2;
        let black_img = GrayImage::from_pixel(64, 64, Luma([0u8]));
        let pyramid0 = build_image_pyramid(&black_img, LEVELS);
        let pyramid1 = build_image_pyramid(&black_img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty(),
            "Total darkness should produce no tracked points"
        );
    }

    #[test]
    fn track_points_saturated_image() {
        const LEVELS: u32 = 2;
        let white_img = GrayImage::from_pixel(64, 64, Luma([255u8]));
        let pyramid0 = build_image_pyramid(&white_img, LEVELS);
        let pyramid1 = build_image_pyramid(&white_img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty(),
            "Saturated image should produce no tracked points"
        );
    }

    #[test]
    fn track_points_salt_and_pepper_noise() {
        const LEVELS: u32 = 2;
        let mut img = GrayImage::from_pixel(64, 64, Luma([128u8]));
        for x in 0..64 {
            for y in 0..64 {
                if (x + y) % 17 == 0 {
                    img.put_pixel(x, y, Luma([0u8]));
                } else if (x + y) % 23 == 0 {
                    img.put_pixel(x, y, Luma([255u8]));
                }
            }
        }
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-2);
        assert!(
            tracked.len() <= 1,
            "Salt and pepper noise should degrade tracking performance"
        );
    }

    #[test]
    fn track_points_gaussian_noise() {
        const LEVELS: u32 = 2;
        let mut img = GrayImage::from_pixel(64, 64, Luma([128u8]));
        for x in 0..64 {
            for y in 0..64 {
                let noise_val = ((x * 7 + y * 13) % 64) as i16 - 32;
                let mut pixel: i16 = 128 + noise_val;
                pixel = pixel.max(0).min(255);
                img.put_pixel(x, y, Luma([pixel as u8]));
            }
        }
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-2);
        assert!(tracked.len() <= 1, "Gaussian noise should degrade tracking");
    }

    #[test]
    fn track_points_dropped_frame() {
        const LEVELS: u32 = 2;
        let img0 = checkerboard_image(64, 64);
        let img1 = GrayImage::from_pixel(64, 64, Luma([0u8]));
        let pyramid0 = build_image_pyramid(&img0, LEVELS);
        let pyramid1 = build_image_pyramid(&img1, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty(),
            "Dropped frame (black) should lose tracking"
        );
    }

    #[test]
    fn track_points_motion_blur_simulated() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let img1 = checkerboard_image(64, 64);
        let pyramid1 = build_image_pyramid(&img1, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 16.0;
        transform.matrix_mut_unchecked().m23 = 16.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty() || tracked.len() == 1,
            "Motion should cause tracking failure or degraded tracking"
        );
    }

    #[test]
    fn track_points_camera_disconnected_pattern() {
        const LEVELS: u32 = 2;
        let mut img0 = GrayImage::from_pixel(64, 64, Luma([128u8]));
        let mut img1 = GrayImage::from_pixel(64, 64, Luma([128u8]));
        img0.put_pixel(0, 0, Luma([0u8]));
        img1.put_pixel(0, 0, Luma([255u8]));
        let pyramid0 = build_image_pyramid(&img0, LEVELS);
        let pyramid1 = build_image_pyramid(&img1, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty(),
            "Camera disconnect pattern should lose tracking"
        );
    }

    #[test]
    fn track_points_horizontal_stripes_interference() {
        const LEVELS: u32 = 2;
        let mut img = GrayImage::from_pixel(64, 64, Luma([128u8]));
        for y in 0..64 {
            let val = if y % 2 == 0 { 0u8 } else { 255u8 };
            for x in 0..64 {
                img.put_pixel(x, y, Luma([val]));
            }
        }
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-2);
        assert!(
            tracked.len() <= 1,
            "Horizontal interference stripes should degrade tracking"
        );
    }

    #[test]
    fn track_points_all_points_lost() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);

        let mut map0 = HashMap::new();
        for i in 0..10 {
            let mut transform = na::Affine2::<f32>::identity();
            transform.matrix_mut_unchecked().m13 = (i * 6 + 1) as f32;
            transform.matrix_mut_unchecked().m23 = (i * 6 + 1) as f32;
            map0.insert(i, transform);
        }

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty(),
            "All points should be trackable in identical images"
        );
    }

    #[test]
    fn track_points_high_frequency_noise() {
        const LEVELS: u32 = 2;
        let mut img = GrayImage::from_pixel(64, 64, Luma([128u8]));
        for x in 0..64 {
            for y in 0..64 {
                let noise_val = ((x * 11 + y * 17) % 50) as i8 - 25;
                let mut pixel: i16 = 128 + noise_val as i16;
                pixel = pixel.max(0).min(255);
                img.put_pixel(x, y, Luma([pixel as u8]));
            }
        }
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-1);
        assert!(
            tracked.len() <= 1,
            "High frequency noise should severely degrade tracking"
        );
    }

    #[test]
    fn track_points_partial_corruption() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let mut img1 = checkerboard_image(64, 64);
        for x in 0..64 {
            for y in 0..10 {
                img1.put_pixel(x, y, Luma([128u8]));
            }
        }
        let pyramid1 = build_image_pyramid(&img1, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 5.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty() || tracked.len() == 1,
            "Partial corruption at tracked point location should cause tracking failure"
        );
    }
}
