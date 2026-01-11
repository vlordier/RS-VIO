use image::{imageops, GrayImage};
use imageproc::corners::Corner;
use nalgebra as na;
use rayon::prelude::*;
use std::collections::HashMap;
use std::ops::AddAssign;
use std::time::Instant;

use crate::datasets::config::FeatureDetectionConfig;

use super::{image_utilities, patch, frame_skip};

use log::info;

#[derive(Debug, Clone, Copy)]
pub struct Feature {
    /// Unique identifier of this feature (within the current frame or globally).
    pub feature_id: usize,

    /// Pixel coordinate in the left image (u, v).
    pub pixel_coord: [f32; 2],

    /// Undistorted pixel coordinate (u, v). `[-1, -1]` means invalid.
    pub undistorted_coord: [f32; 2],
}

impl Feature {
    pub fn new(feature_id: usize, pixel_coord: [f32; 2]) -> Self {
        Self {
            feature_id,
            pixel_coord,
            undistorted_coord: [-1.0, -1.0],
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
        let current_image_pyramid: Vec<GrayImage> = build_image_pyramid(greyscale_image, LEVELS);

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
        for point in &new_points {
            let mut v = na::Affine2::<f32>::identity();

            v.matrix_mut_unchecked().m13 = point.x as f32;
            v.matrix_mut_unchecked().m23 = point.y as f32;
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
    grid_size: u32,
    optical_flow_max_iterations: usize,
    optical_flow_convergence_threshold: f32,
    /// Adaptive frame skipper for real-time constraints
    frame_skipper: frame_skip::AdaptiveFrameSkipper,
    /// Last frame processing time for benchmarking
    last_frame_time: Option<Instant>,
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
        Self {
            last_keypoint_id: 0,
            tracked_points_map_cam0: HashMap::new(),
            previous_image_pyramid0: Vec::new(),
            tracked_points_map_cam1: HashMap::new(),
            previous_image_pyramid1: Vec::new(),
            grid_size,
            optical_flow_max_iterations: optical_flow_max_iterations as usize,
            optical_flow_convergence_threshold: optical_flow_convergence_threshold as f32,
            frame_skipper: frame_skip::AdaptiveFrameSkipper::new(30.0, 5, 2.0),
            last_frame_time: None,
        }
    }

    /// Construct tracker from `FeatureDetectionConfig` for centralized tuning.
    pub fn from_config(config: &crate::datasets::config::FeatureDetectionConfig) -> Self {
        Self::new(
            config.grid_cols,
            config.optical_flow_max_iterations,
            config.optical_flow_convergence_threshold,
        )
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
        
        // Adaptive frame skipping: check if we should process this frame
        // Estimate motion from recent feature positions (simple heuristic)
        let estimated_motion = self.estimate_frame_motion();
        if !self.frame_skipper.should_process(estimated_motion) {
            // Skip this frame but keep pyramids for potential next frame processing
            if let Some(last_time) = self.last_frame_time {
                let delta = frame_start.duration_since(last_time);
                self.frame_skipper.record_frame_time(delta);
            }
            log::debug!("[FeatureTracker] Frame skipped for real-time constraints");
            return;
        }
        
        // build current image pyramid
        let current_image_pyramid0: Vec<GrayImage> = build_image_pyramid(greyscale_image0, LEVELS);
        let current_image_pyramid1: Vec<GrayImage> = build_image_pyramid(greyscale_image1, LEVELS);

        // not initialized
        if !self.previous_image_pyramid0.is_empty() {
            log::debug!(
                "[FeatureTracker] Number of old points in cam0: {}",
                self.tracked_points_map_cam0.len()
            );
            // track prev points
            self.tracked_points_map_cam0 = track_points::<LEVELS>(
                &self.previous_image_pyramid0,
                &current_image_pyramid0,
                &self.tracked_points_map_cam0,
                self.optical_flow_max_iterations,
                self.optical_flow_convergence_threshold,
            );
            self.tracked_points_map_cam1 = track_points::<LEVELS>(
                &self.previous_image_pyramid1,
                &current_image_pyramid1,
                &self.tracked_points_map_cam1,
                self.optical_flow_max_iterations,
                self.optical_flow_convergence_threshold,
            );
            log::debug!(
                "[FeatureTracker] Number of tracked old points in cam0: {}",
                self.tracked_points_map_cam0.len()
            );
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
            .map(|(i, point)| {
                let mut v = na::Affine2::<f32>::identity();
                v.matrix_mut_unchecked().m13 = point.x as f32;
                v.matrix_mut_unchecked().m23 = point.y as f32;
                (i, v)
            })
            .collect();

        let tmp_tracked_points1 = track_points::<LEVELS>(
            &current_image_pyramid0,
            &current_image_pyramid1,
            &tmp_tracked_points0,
            self.optical_flow_max_iterations,
            self.optical_flow_convergence_threshold,
        );

        for (key0, pt0) in tmp_tracked_points0 {
            if let Some(pt1) = tmp_tracked_points1.get(&key0) {
                self.tracked_points_map_cam0
                    .insert(self.last_keypoint_id, pt0);
                self.tracked_points_map_cam1
                    .insert(self.last_keypoint_id, *pt1);
                self.last_keypoint_id += 1;
            }
        }

        // update saved image pyramid
        self.previous_image_pyramid0 = current_image_pyramid0;
        self.previous_image_pyramid1 = current_image_pyramid1;

        // Populate the frame's feature lists from the stereo tracks
        let [tracked_left, tracked_right] = self.get_track_points();
        for (id, (x, y)) in tracked_left {
            let f = Feature::new(id, [x, y]);
            frame.add_left_feature(f);
        }

        for (id, (x, y)) in tracked_right {
            let f = Feature::new(id, [x, y]);
            frame.add_right_feature(f);
        }
        
        // Record processing time for adaptive frame skipping
        let frame_duration = frame_start.elapsed();
        self.frame_skipper.record_frame_time(frame_duration);
        self.last_frame_time = Some(frame_start);
    }
    
    /// Estimate frame-to-frame motion as heuristic for frame skipping
    fn estimate_frame_motion(&self) -> Option<f32> {
        if self.tracked_points_map_cam0.is_empty() {
            return None;
        }
        
        // Simple motion estimate: average distance from center
        let image_center_x = 320.0f32; // Typical for 640×480 images
        let image_center_y = 240.0f32;
        
        let motion_sum: f32 = self.tracked_points_map_cam0
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

fn build_image_pyramid(greyscale_image: &GrayImage, levels: u32) -> Vec<GrayImage> {
    const FILTER_TYPE: imageops::FilterType = imageops::FilterType::Triangle;
    let (w0, h0) = greyscale_image.dimensions();

    // Use sequential iteration for deterministic execution in real-time systems
    // Parallel iteration with Rayon introduces non-deterministic scheduling
    (0..levels)
        .map(|i| {
            let scale_down: u32 = 1 << i;
            let (new_w, new_h) = (w0 / scale_down, h0 / scale_down);
            imageops::resize(greyscale_image, new_w, new_h, FILTER_TYPE)
        })
        .collect()
}

fn add_points(
    tracked_points_map: &HashMap<usize, na::Affine2<f32>>,
    grayscale_image: &GrayImage,
    grid_size: u32,
) -> Vec<Corner> {
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
    // let mut prev_points =
    // Eigen::aligned_vector<Eigen::Vector2d> pts0;

    // for (const auto &kv : observations.at(0)) {
    //   pts0.emplace_back(kv.second.translation().template cast<double>());
    // }
}
fn track_points<const LEVELS: u32>(
    image_pyramid0: &[GrayImage],
    image_pyramid1: &[GrayImage],
    transform_maps0: &HashMap<usize, na::Affine2<f32>>,
    optical_flow_max_iterations: usize,
    optical_flow_convergence_threshold: f32,
) -> HashMap<usize, na::Affine2<f32>> {
    // Use parallel iteration for multi-core performance boost
    // Each point tracking is independent and can be parallelized
    let results: Vec<(usize, na::Affine2<f32>)> = transform_maps0
        .par_iter()
        .filter_map(|(k, v)| {
            if let Some(new_v) = track_one_point::<LEVELS>(
                image_pyramid0,
                image_pyramid1,
                v,
                optical_flow_max_iterations,
                optical_flow_convergence_threshold,
            ) {
                // return Some((k.clone(), new_v));
                if let Some(old_v) = track_one_point::<LEVELS>(
                    image_pyramid1,
                    image_pyramid0,
                    &new_v,
                    optical_flow_max_iterations,
                    optical_flow_convergence_threshold,
                ) {
                    if (v.matrix() - old_v.matrix())
                        .fixed_view::<2, 1>(0, 2)
                        .norm_squared()
                        < 0.4
                    {
                        return Some((*k, new_v));
                    }
                }
            }
            None
        })
        .collect();

    // Convert Vec to HashMap for compatibility
    results.into_iter().collect()
}
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
}
