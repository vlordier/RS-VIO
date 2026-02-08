//! Stereo patch tracker: KLT optical flow with stereo matching.

use image::{imageops, GrayImage};
use imageproc::corners::Corner;
use nalgebra as na;
use rayon::prelude::*;
use std::collections::HashMap;

use super::{image_utilities, patch};

#[derive(Debug, Clone)]
pub struct Feature {
    /// Unique identifier of this feature (within the current frame or globally).
    pub feature_id: usize,

    /// Pixel coordinate in the left image (u, v).
    pub pixel_coord: [f32; 2],

    /// Undistorted pixel coordinate (u, v). `[-1, -1]` means invalid.
    pub undistorted_coord: [f32; 2],
}

impl Feature {
    pub const fn new(feature_id: usize, pixel_coord: [f32; 2]) -> Self {
        Self {
            feature_id,
            pixel_coord,
            undistorted_coord: [-1.0, -1.0],
        }
    }
}

pub struct StereoPatchTracker<const N: u32> {
    last_keypoint_id: usize,
    tracked_points_map_cam0: HashMap<usize, na::Affine2<f32>>,
    previous_image_pyramid0: Vec<GrayImage>,
    current_image_pyramid0: Vec<GrayImage>,
    tracked_points_map_cam1: HashMap<usize, na::Affine2<f32>>,
    previous_image_pyramid1: Vec<GrayImage>,
    current_image_pyramid1: Vec<GrayImage>,
    has_previous: bool,
    grid_size: u32,
    optical_flow_max_iterations: usize,
    optical_flow_convergence_threshold: f32,
}

impl<const LEVELS: u32> StereoPatchTracker<LEVELS> {
    pub fn new(
        grid_size: u32,
        optical_flow_max_iterations: u32,
        optical_flow_convergence_threshold: f64,
    ) -> Self {
        Self {
            last_keypoint_id: 0,
            tracked_points_map_cam0: HashMap::new(),
            previous_image_pyramid0: Vec::new(),
            current_image_pyramid0: Vec::new(),
            tracked_points_map_cam1: HashMap::new(),
            previous_image_pyramid1: Vec::new(),
            current_image_pyramid1: Vec::new(),
            has_previous: false,
            grid_size,
            optical_flow_max_iterations: optical_flow_max_iterations as usize,
            optical_flow_convergence_threshold: optical_flow_convergence_threshold as f32,
        }
    }

    pub fn process_frame(
        &mut self,
        greyscale_image0: &GrayImage,
        greyscale_image1: &GrayImage,
        frame: &mut crate::estimator::Frame,
    ) {
        // build current image pyramid (reuse buffers)
        ensure_pyramid_buffers(
            &mut self.current_image_pyramid0,
            greyscale_image0.width(),
            greyscale_image0.height(),
            LEVELS,
        );
        ensure_pyramid_buffers(
            &mut self.previous_image_pyramid0,
            greyscale_image0.width(),
            greyscale_image0.height(),
            LEVELS,
        );
        ensure_pyramid_buffers(
            &mut self.current_image_pyramid1,
            greyscale_image1.width(),
            greyscale_image1.height(),
            LEVELS,
        );
        ensure_pyramid_buffers(
            &mut self.previous_image_pyramid1,
            greyscale_image1.width(),
            greyscale_image1.height(),
            LEVELS,
        );

        // Parallelize pyramid construction
        let (current_pyr0, current_pyr1) = (
            &mut self.current_image_pyramid0,
            &mut self.current_image_pyramid1,
        );
        rayon::join(
            || build_pyramid_in_place(greyscale_image0, current_pyr0),
            || build_pyramid_in_place(greyscale_image1, current_pyr1),
        );

        // not initialized
        if self.has_previous {
            log::debug!(
                "[FeatureTracker] Number of old points in cam0: {}",
                self.tracked_points_map_cam0.len()
            );
            // track prev points
            // Parallelize temporal tracking (Left and Right cameras independently)
            let prev_pyr0 = &self.previous_image_pyramid0;
            let prev_pyr1 = &self.previous_image_pyramid1;
            let map0 = &self.tracked_points_map_cam0;
            let map1 = &self.tracked_points_map_cam1;
            let max_iters = self.optical_flow_max_iterations;
            let threshold = self.optical_flow_convergence_threshold;

            let (new_map0, new_map1) = rayon::join(
                || {
                    track_points::<LEVELS>(
                        prev_pyr0,
                        &self.current_image_pyramid0,
                        map0,
                        max_iters,
                        threshold,
                    )
                },
                || {
                    track_points::<LEVELS>(
                        prev_pyr1,
                        &self.current_image_pyramid1,
                        map1,
                        max_iters,
                        threshold,
                    )
                },
            );

            self.tracked_points_map_cam0 = new_map0;
            self.tracked_points_map_cam1 = new_map1;

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
            &self.current_image_pyramid0,
            &self.current_image_pyramid1,
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

        // update saved image pyramid (swap to reuse allocations)
        std::mem::swap(
            &mut self.previous_image_pyramid0,
            &mut self.current_image_pyramid0,
        );
        std::mem::swap(
            &mut self.previous_image_pyramid1,
            &mut self.current_image_pyramid1,
        );
        self.has_previous = true;

        // Populate the frame's feature lists directly from tracked_points_maps
        // (avoids allocating 2 temporary HashMaps per frame)
        for (&id, v) in &self.tracked_points_map_cam0 {
            let f = Feature::new(id, [v.matrix().m13, v.matrix().m23]);
            frame.add_left_feature(f);
        }
        for (&id, v) in &self.tracked_points_map_cam1 {
            let f = Feature::new(id, [v.matrix().m13, v.matrix().m23]);
            frame.add_right_feature(f);
        }
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
}

fn ensure_pyramid_buffers(
    pyramid: &mut Vec<GrayImage>,
    base_width: u32,
    base_height: u32,
    levels: u32,
) {
    let target_len = levels as usize;
    if pyramid.len() != target_len {
        pyramid.clear();
        pyramid.reserve(target_len);
        for level in 0..levels {
            let scale_down = 1 << level;
            let width = (base_width / scale_down).max(1);
            let height = (base_height / scale_down).max(1);
            pyramid.push(GrayImage::new(width, height));
        }
        return;
    }

    let mut resize_needed = false;
    for (level, img) in pyramid.iter().enumerate() {
        let scale_down = 1 << level;
        let width = (base_width / scale_down).max(1);
        let height = (base_height / scale_down).max(1);
        if img.width() != width || img.height() != height {
            resize_needed = true;
            break;
        }
    }

    if resize_needed {
        pyramid.clear();
        pyramid.reserve(target_len);
        for level in 0..levels {
            let scale_down = 1 << level;
            let width = (base_width / scale_down).max(1);
            let height = (base_height / scale_down).max(1);
            pyramid.push(GrayImage::new(width, height));
        }
    }
}

fn build_pyramid_in_place(base: &GrayImage, pyramid: &mut [GrayImage]) {
    if pyramid.is_empty() {
        return;
    }

    // Copy or resize base image to pyramid level 0
    if pyramid[0].width() == base.width() && pyramid[0].height() == base.height() {
        // Reuse existing buffer: memcpy instead of clone avoids allocation
        pyramid[0].copy_from_slice(base);
    } else {
        let resized = imageops::resize(
            base,
            pyramid[0].width(),
            pyramid[0].height(),
            imageops::FilterType::Triangle,
        );
        pyramid[0] = resized;
    }

    // Build remaining pyramid levels by downsampling with 2×2 box filter (in-place, zero alloc)
    for level in 1..pyramid.len() {
        let (left, right) = pyramid.split_at_mut(level);
        let src = &left[level - 1];
        let sw = src.width();
        let sh = src.height();
        let dw = (sw / 2).max(1);
        let dh = (sh / 2).max(1);
        let dst = &mut right[0];
        debug_assert!(dst.width() == dw && dst.height() == dh);
        let src_raw = src.as_raw();
        let dst_raw = dst.as_mut();
        let sw_usize = sw as usize;
        // Fast interior: all four 2×2 neighbors guaranteed in-bounds
        let safe_dh = if sh >= 2 { dh.saturating_sub(1) } else { 0 };
        let safe_dw = if sw >= 2 { dw.saturating_sub(1) } else { 0 };
        for dy in 0..safe_dh {
            let sy = (dy * 2) as usize;
            let row0 = sy * sw_usize;
            let row1 = row0 + sw_usize;
            let dst_row = (dy * dw) as usize;
            for dx in 0..safe_dw {
                let sx = (dx * 2) as usize;
                let v = (src_raw[row0 + sx] as u16
                    + src_raw[row0 + sx + 1] as u16
                    + src_raw[row1 + sx] as u16
                    + src_raw[row1 + sx + 1] as u16)
                    / 4;
                dst_raw[dst_row + dx as usize] = v as u8;
            }
        }
        // Border pixels: last row and/or last column need bounds-checked access
        for dy in 0..dh {
            let start_dx = if dy < safe_dh { safe_dw } else { 0 };
            for dx in start_dx..dw {
                let sx = (dx * 2) as usize;
                let sy = (dy * 2) as usize;
                let i = sy * sw_usize + sx;
                let v = (src_raw[i] as u16
                    + src_raw.get(i + 1).copied().unwrap_or(src_raw[i]) as u16
                    + src_raw.get(i + sw_usize).copied().unwrap_or(src_raw[i]) as u16
                    + src_raw.get(i + sw_usize + 1).copied().unwrap_or(src_raw[i]) as u16)
                    / 4;
                dst_raw[(dy * dw + dx) as usize] = v as u8;
            }
        }
    }
}

fn add_points(
    tracked_points_map: &HashMap<usize, na::Affine2<f32>>,
    grayscale_image: &GrayImage,
    grid_size: u32,
) -> Vec<Corner> {
    let num_points_in_cell = 1;
    // Pre-allocate capacity to avoid reallocation during collection
    let mut current_corners: Vec<Corner> = Vec::with_capacity(tracked_points_map.len());
    current_corners.extend(tracked_points_map.values().map(|v| {
        Corner::new(
            v.matrix().m13.round() as u32,
            v.matrix().m23.round() as u32,
            0.0,
        )
    }));
    image_utilities::detect_key_points(
        grayscale_image,
        grid_size,
        &current_corners,
        num_points_in_cell,
    )
}
fn track_points<const LEVELS: u32>(
    image_pyramid0: &[GrayImage],
    image_pyramid1: &[GrayImage],
    transform_maps0: &HashMap<usize, na::Affine2<f32>>,
    optical_flow_max_iterations: usize,
    optical_flow_convergence_threshold: f32,
) -> HashMap<usize, na::Affine2<f32>> {
    // Collect with parallel iterator, then insert into pre-allocated HashMap
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

    // Pre-allocate HashMap with capacity hint before inserting
    let mut transform_maps1: HashMap<usize, na::Affine2<f32>> =
        HashMap::with_capacity(results.len());
    transform_maps1.extend(results);

    transform_maps1
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
    for _iteration in 0..optical_flow_max_iterations {
        if let Some(res) = dp.residual(grayscale_image, transform) {
            let inc = -dp.h_se2_inv_j_se2_t * res;

            // avoid NaN in increment (leads to SE2::exp crashing)
            if !inc.iter().all(|x| x.is_finite()) {
                return false;
            }
            let inc_norm = inc.norm();
            if inc_norm > 1e6 {
                return false;
            }

            // Early termination if converged
            if inc_norm < optical_flow_convergence_threshold {
                break;
            }

            let new_trans = transform.matrix() * image_utilities::se2_exp_matrix(&inc);
            *transform = na::Affine2::<f32>::from_matrix_unchecked(new_trans);
            let filter_margin = 2;
            if !image_utilities::inbound(
                grayscale_image,
                transform.matrix().m13,
                transform.matrix().m23,
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
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::field_reassign_with_default)]
mod tests {
    use super::*;

    #[test]
    fn feature_new_pixel_coord() {
        let f = Feature::new(42, [10.5, 20.3]);
        assert_eq!(f.pixel_coord, [10.5, 20.3]);
    }

    #[test]
    fn feature_new_undistorted_coord_defaults() {
        let f = Feature::new(0, [1.0, 2.0]);
        assert_eq!(f.undistorted_coord, [-1.0, -1.0]);
    }

    #[test]
    fn feature_new_feature_id() {
        let f = Feature::new(99, [0.0, 0.0]);
        assert_eq!(f.feature_id, 99);
    }

    #[test]
    fn stereo_patch_tracker_new_has_previous_false() {
        let tracker = StereoPatchTracker::<3>::new(16, 30, 0.01);
        assert!(!tracker.has_previous);
    }

    #[test]
    fn stereo_patch_tracker_new_track_points_empty() {
        let tracker = StereoPatchTracker::<3>::new(16, 30, 0.01);
        let [cam0, cam1] = tracker.get_track_points();
        assert!(cam0.is_empty());
        assert!(cam1.is_empty());
    }

    #[test]
    fn stereo_patch_tracker_generic_n_compiles() {
        // Verify that different pyramid levels N work correctly:
        // N=2 (fewer levels) vs N=5 (more levels) should both initialize.
        let t2 = StereoPatchTracker::<2>::new(8, 50, 0.001);
        let t5 = StereoPatchTracker::<5>::new(8, 50, 0.001);
        assert!(!t2.has_previous);
        assert!(!t5.has_previous);
        assert!(t2.get_track_points()[0].is_empty());
        assert!(t5.get_track_points()[0].is_empty());
    }

    #[test]
    fn stereo_patch_tracker_process_frame_detects_features() {
        // Bright 20×20 squares on 40×40 dark grid — corners of each square
        // produce strong FAST9 responses (≥11 contiguous dark ring pixels).
        let img = image::GrayImage::from_fn(640, 480, |x, y| {
            let bx = x % 40;
            let by = y % 40;
            if (10..30).contains(&bx) && (10..30).contains(&by) {
                image::Luma([255u8])
            } else {
                image::Luma([0u8])
            }
        });

        let mut tracker = StereoPatchTracker::<3>::new(16, 30, 0.01);
        let mut frame = crate::estimator::Frame::new(1_000_000_000, 0);
        tracker.process_frame(&img, &img, &mut frame);

        assert!(tracker.has_previous);
        assert!(!frame.left_features.is_empty());
        assert!(!frame.right_features.is_empty());

        for f in &frame.left_features {
            assert!((0.0..640.0).contains(&f.pixel_coord[0]));
            assert!((0.0..480.0).contains(&f.pixel_coord[1]));
        }

        assert_eq!(tracker.get_track_points()[0].len(), frame.left_features.len());
    }

    #[test]
    fn stereo_patch_tracker_tracks_across_frames() {
        // Same bright-squares pattern as test 1
        let img1 = image::GrayImage::from_fn(640, 480, |x, y| {
            let bx = x % 40;
            let by = y % 40;
            if (10..30).contains(&bx) && (10..30).contains(&by) {
                image::Luma([255u8])
            } else {
                image::Luma([0u8])
            }
        });

        // Shifted by 2 pixels horizontally
        let img2 = image::GrayImage::from_fn(640, 480, |x, y| {
            let bx = (x + 2) % 40;
            let by = y % 40;
            if (10..30).contains(&bx) && (10..30).contains(&by) {
                image::Luma([255u8])
            } else {
                image::Luma([0u8])
            }
        });

        let mut tracker = StereoPatchTracker::<3>::new(16, 30, 0.01);

        let mut frame0 = crate::estimator::Frame::new(1_000_000_000, 0);
        tracker.process_frame(&img1, &img1, &mut frame0);

        let mut frame1 = crate::estimator::Frame::new(2_000_000_000, 1);
        tracker.process_frame(&img2, &img2, &mut frame1);

        assert!(!tracker.get_track_points()[0].is_empty());
        assert!(!frame1.left_features.is_empty());
    }
}
