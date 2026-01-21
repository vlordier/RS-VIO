/// Stereo patch tracker implementation
use image::{GrayImage, Luma};
use nalgebra as na;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::time::Instant;

use crate::datasets::config::FeatureDetectionConfig;
use crate::datasets::ImuData;
use crate::debug_log;
use crate::feature_tracker::{
    frame_skip, gpu_accel, image_utilities,
    ransac_essential::{EssentialMatrixRansac, RansacConfig},
    subpixel_stereo::{StereoMatchResult, SubpixelStereoRefinement},
    FrameStabilizer, StabilizerConfig, TrackFirstDetector, TrackFirstConfig,
};
use crate::vision::subpixel_disparity::PatchMatchingConfig;

use super::tracking::{add_points, track_points};
use super::types::Feature;

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
    #[allow(dead_code)]
    essential_ransac: EssentialMatrixRansac,
    /// Pluggable stereo matching strategy (BasicRANSAC, IMUGuided, etc.)
    #[allow(dead_code)]
    matching_strategy: Box<dyn crate::feature_tracker::StereoMatchingStrategy>,
    /// Previous frame's match results for temporal consistency
    #[allow(dead_code)]
    previous_match_results: Vec<StereoMatchResult>,
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
    /// Frame stabilizer for multi-frame geometric super-resolution
    frame_stabilizer_cam0: Option<FrameStabilizer>,
    frame_stabilizer_cam1: Option<FrameStabilizer>,
    /// Track-first detector for maintaining long feature tracks
    track_first_detector: Option<TrackFirstDetector>,
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
    /// # Example
    /// ```rust,no_run
    /// use rs_vio::feature_tracker::StereoPatchTracker;
    /// let tracker = StereoPatchTracker::<4>::new(15, 30, 0.005);
    /// ```
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
            // Initialize with default strategy (BasicRANSAC)
            matching_strategy: Box::new(crate::feature_tracker::BasicRANSACStrategy::new(
                crate::feature_tracker::BasicRANSACConfig::default(),
            )),
            previous_match_results: Vec::new(),
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
            frame_stabilizer_cam0: None,
            frame_stabilizer_cam1: None,
            track_first_detector: None,
        }
    }

    /// Construct tracker from `FeatureDetectionConfig` for centralized tuning.
    pub fn from_config(config: &FeatureDetectionConfig) -> Self {
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
        tracker.max_lost_frames = 5;
        tracker.feature_velocities = HashMap::new();
        tracker
    }

    /// Set calibrated camera intrinsics for geometric gating and projection.
    pub fn set_camera_intrinsics(&mut self, fx: f32, fy: f32, cx: f32, cy: f32) {
        self.camera_matrix_hint = Some(na::Matrix3::new(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0));
    }

    /// Enable frame stabilization with specified configuration
    pub fn enable_frame_stabilization(&mut self, config: StabilizerConfig, fx: f32, fy: f32, cx: f32, cy: f32) {
        if config.enabled {
            self.frame_stabilizer_cam0 = Some(FrameStabilizer::new(config, fx, fy, cx, cy));
            self.frame_stabilizer_cam1 = Some(FrameStabilizer::new(config, fx, fy, cx, cy));
        } else {
            self.frame_stabilizer_cam0 = None;
            self.frame_stabilizer_cam1 = None;
        }
    }

    /// Enable track-first detection strategy
    pub fn enable_track_first_detection(&mut self, config: TrackFirstConfig, image_width: u32, image_height: u32) {
        self.track_first_detector = Some(TrackFirstDetector::new(config, image_width, image_height));
    }

    /// Disable frame stabilization
    pub fn disable_frame_stabilization(&mut self) {
        self.frame_stabilizer_cam0 = None;
        self.frame_stabilizer_cam1 = None;
    }

    /// Disable track-first detection
    pub fn disable_track_first_detection(&mut self) {
        self.track_first_detector = None;
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

    /// Set the stereo matching strategy at runtime
    ///
    /// Allows switching between different matching strategies without recompilation.
    /// Useful for A/B testing or platform-specific optimization.
    pub fn set_matching_strategy(
        &mut self,
        strategy: Box<dyn crate::feature_tracker::StereoMatchingStrategy>,
    ) {
        self.matching_strategy = strategy;
    }

    /// Load stereo matching strategy from configuration file
    ///
    /// # Arguments
    /// * `path` - Path to YAML configuration file with strategy selection
    ///
    /// # Example
    /// ```yaml
    /// strategy: IMUGuided
    /// params:
    ///   search_margin_px: 8.0
    ///   max_iterations: 500
    /// ```
    pub fn load_matching_strategy_config(&mut self, path: &str) -> Result<(), String> {
        let config = crate::feature_tracker::MatchingStrategyConfig::from_file(path)
            .map_err(|e| e.to_string())?;
        self.matching_strategy = config.create_strategy()?;
        Ok(())
    }

    /// Get the name of the currently active matching strategy
    pub fn matching_strategy_name(&self) -> &'static str {
        self.matching_strategy.name()
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
        let estimated_motion = self.estimate_frame_motion();
        if !self.frame_skipper.should_process(estimated_motion) {
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

        // Apply frame stabilization if enabled
        let processed_image0 = if let Some(ref mut stabilizer) = self.frame_stabilizer_cam0 {
            // Get rotation from IMU hint or identity
            let rotation = if let Some(omega) = self.imu_rotation_hint {
                let angle = (omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]).sqrt();
                if angle > 0.001 {
                    let axis = na::Vector3::new(omega[0] / angle, omega[1] / angle, omega[2] / angle);
                    na::Rotation3::from_axis_angle(&na::Unit::new_normalize(axis), angle)
                } else {
                    na::Rotation3::identity()
                }
            } else {
                na::Rotation3::identity()
            };
            
            stabilizer.process_frame(greyscale_image0, rotation, frame.timestamp_ns as f64)
        } else {
            greyscale_image0.clone()
        };

        let processed_image1 = if let Some(ref mut stabilizer) = self.frame_stabilizer_cam1 {
            let rotation = if let Some(omega) = self.imu_rotation_hint {
                let angle = (omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]).sqrt();
                if angle > 0.001 {
                    let axis = na::Vector3::new(omega[0] / angle, omega[1] / angle, omega[2] / angle);
                    na::Rotation3::from_axis_angle(&na::Unit::new_normalize(axis), angle)
                } else {
                    na::Rotation3::identity()
                }
            } else {
                na::Rotation3::identity()
            };
            
            stabilizer.process_frame(greyscale_image1, rotation, frame.timestamp_ns as f64)
        } else {
            greyscale_image1.clone()
        };

        // Build current image pyramids into preallocated buffers
        let (w0, h0) = processed_image0.dimensions();
        self.ensure_pyramids_allocated(w0, h0);
        image_utilities::fill_pyramid(&mut self.current_image_pyramid0, &processed_image0);
        image_utilities::fill_pyramid(&mut self.current_image_pyramid1, &processed_image1);

        // Track features if initialized
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

        // Add new points using track-first strategy if enabled, otherwise use default detection
        let new_points0 = if let Some(ref mut detector) = self.track_first_detector {
            // Convert tracked points to format expected by track-first detector
            let tracked_features: Vec<(usize, na::Vector2<f32>)> = self.tracked_points_map_cam0
                .iter()
                .map(|(id, affine)| {
                    let mat = affine.matrix();
                    (*id, na::Vector2::new(mat[(0, 2)], mat[(1, 2)]))
                })
                .collect();
            
            // Update with tracking residuals if available (for now, use None)
            detector.update_tracks(&processed_image0, &tracked_features, None);
            
            // Get newly detected features by checking the detector's internal state
            // For now, use default detection if track-first doesn't need new features
            if detector.needs_new_features() {
                add_points(
                    &self.tracked_points_map_cam0,
                    &processed_image0,
                    self.grid_size,
                )
            } else {
                Vec::new()
            }
        } else {
            add_points(
                &self.tracked_points_map_cam0,
                &processed_image0,
                self.grid_size,
            )
        };
        
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

        // Collect NEW feature IDs (from stereo matching this frame)
        let new_ids: HashSet<usize> = tmp_tracked_points0.keys().copied().collect();

        for (key0, pt0) in tmp_tracked_points0 {
            if let Some(pt1) = tmp_tracked_points1.get(&key0) {
                self.tracked_points_map_cam0
                    .insert(self.last_keypoint_id, pt0);
                self.tracked_points_map_cam1
                    .insert(self.last_keypoint_id, *pt1);
                self.last_keypoint_id += 1;
            }
        }

        // Handle lost features and temporal consistency
        self.update_lost_features();
        self.update_temporal_consistency();

        // Find valid stereo pairs
        let left_ids: HashSet<usize> = self.tracked_points_map_cam0.keys().copied().collect();
        let right_ids: HashSet<usize> = self.tracked_points_map_cam1.keys().copied().collect();
        let common_ids: HashSet<usize> = left_ids.intersection(&right_ids).copied().collect();
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

        // Synchronize tracked points
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

        // Use selected stereo matching strategy for outlier rejection
        let camera_matrix = self.make_camera_matrix(w0, h0);

        // Prepare IMU state for strategy (if available)
        let imu_state = if let Some(omega) = self.imu_rotation_hint {
            if let Some((_fx, _fy, _cx, _cy)) = self.imu_intrinsics_hint {
                // IMU state would come from fusion pipeline
                // For now, use zero velocity as placeholder
                // TODO: Connect to actual fusion/estimator velocity estimates
                Some(crate::feature_tracker::IMUState {
                    velocity: na::Vector3::zeros(),
                    angular_velocity: na::Vector3::new(omega[0], omega[1], omega[2]),
                    dt: 1.0 / 30.0, // Assume 30 FPS, should get from actual frame timing
                })
            } else {
                None
            }
        } else {
            None
        };

        // Build previous depth map for temporal consistency strategy
        let previous_depth: Vec<(usize, f32)> = self
            .previous_match_results
            .iter()
            .map(|m| (m.id, m.disparity))
            .collect();

        // Extract features in format expected by strategy
        let features: Vec<(usize, f32, f32)> = refined_matches
            .iter()
            .map(|m| (m.id, m.left_pos.matrix().m13, m.left_pos.matrix().m23))
            .collect();

        // Call stereo matching strategy
        let strategy_result = self.matching_strategy.match_stereo(
            greyscale_image0.as_raw(),
            greyscale_image1.as_raw(),
            w0 as usize,
            h0 as usize,
            &features,
            &camera_matrix,
            imu_state.as_ref(),
            if previous_depth.is_empty() {
                None
            } else {
                Some(&previous_depth)
            },
        );

        // Log strategy metrics
        if should_log {
            debug_log!(
                "[FeatureTracker] Frame {}: Strategy='{}' inliers={}/{} time={:.3}ms",
                self.frame_count,
                self.matching_strategy.name(),
                strategy_result.metrics.inliers_final,
                strategy_result.metrics.candidates_initial,
                strategy_result.metrics.time_total_ms
            );
        }

        // Store results for next frame (temporal consistency)
        self.previous_match_results = strategy_result.matches.clone();

        // Use strategy-filtered matches directly
        let final_matches = strategy_result.matches;

        let retained_ids: HashSet<usize> = final_matches.iter().map(|m| m.id).collect();
        self.tracked_points_map_cam0
            .retain(|id, _| retained_ids.contains(id));
        self.tracked_points_map_cam1
            .retain(|id, _| retained_ids.contains(id));

        // Populate the frame's feature lists
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

        // Ensure minimum features in static scenes
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

        // Swap pyramids to reuse allocations
        std::mem::swap(
            &mut self.previous_image_pyramid0,
            &mut self.current_image_pyramid0,
        );
        std::mem::swap(
            &mut self.previous_image_pyramid1,
            &mut self.current_image_pyramid1,
        );

        // Record processing time
        let frame_duration = frame_start.elapsed();
        self.frame_skipper.record_frame_time(frame_duration);
        self.last_frame_time = Some(frame_start);
    }

    /// Update lost features and attempt recovery
    fn update_lost_features(&mut self) {
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

        for (id, left_pos, right_pos) in newly_lost {
            self.lost_features.insert(id, (left_pos, right_pos, 1));
        }

        self.lost_features
            .values_mut()
            .for_each(|(_, _, frames_lost)| {
                *frames_lost += 1;
            });

        self.lost_features
            .retain(|_, (_, _, frames_lost)| *frames_lost <= self.max_lost_frames);
    }

    /// Update feature velocities and validate temporal consistency
    fn update_temporal_consistency(&mut self) {
        let mut new_velocities = HashMap::new();

        for (&id, current_pos) in &self.tracked_points_map_cam0 {
            let current_x = current_pos.matrix().m13;
            let current_y = current_pos.matrix().m23;

            if let Some((prev_vx, prev_vy, prev_age)) = self.feature_velocities.get(&id) {
                let alpha = 0.3;
                let measured_vx = current_x - (current_x - prev_vx);
                let measured_vy = current_y - (current_y - prev_vy);

                let smoothed_vx = alpha * measured_vx + (1.0 - alpha) * prev_vx;
                let smoothed_vy = alpha * measured_vy + (1.0 - alpha) * prev_vy;

                new_velocities.insert(id, (smoothed_vx, smoothed_vy, prev_age + 1));
            } else {
                new_velocities.insert(id, (0.0, 0.0, 1));
            }
        }

        // Validate temporal consistency
        let mut inconsistent_features = Vec::new();
        for (&id, &(vx, vy, age)) in &new_velocities {
            if age > 3 {
                let speed = (vx * vx + vy * vy).sqrt();
                if speed > 50.0 {
                    inconsistent_features.push(id);
                }
            }
        }

        for id in inconsistent_features {
            self.tracked_points_map_cam0.remove(&id);
            self.tracked_points_map_cam1.remove(&id);
            new_velocities.remove(&id);
        }

        self.feature_velocities = new_velocities;
    }

    /// Estimate frame-to-frame motion for frame skipping
    fn estimate_frame_motion(&self) -> Option<f32> {
        if self.tracked_points_map_cam0.is_empty() {
            return None;
        }

        let image_center_x = 320.0f32;
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

    /// Apply IMU rotation prediction to tracked points
    fn apply_imu_prediction(&mut self, omega: &[f32; 3], fx: f32, fy: f32, cx: f32, cy: f32) {
        let rotate_point = |pt: &mut na::Affine2<f32>| {
            let u = pt.matrix().m13;
            let v = pt.matrix().m23;

            let x = (u - cx) / fx;
            let y = (v - cy) / fy;
            let z = 1.0f32;

            let wx = omega[0];
            let wy = omega[1];
            let wz = omega[2];

            let rx = x + (-wz * y + wy * z);
            let ry = y + (wz * x - wx * z);
            let rz = z + (-wy * x + wx * y);

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
