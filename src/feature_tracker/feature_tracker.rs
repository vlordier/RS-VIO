use image::GrayImage;
use imageproc::corners::Corner;
use nalgebra as na;
use rayon::prelude::*;
use rustc_hash::FxHashMap;
use super::{image_utilities, patch};
use log::info;

#[derive(Debug, Clone)]
pub struct Feature {
    /// Unique identifier of this feature (within the current frame or globally).
    pub feature_id: usize,

    /// Pixel coordinate in the left image (u, v).
    pub pixel_coord: [f32; 2],

    /// Undistorted pixel coordinate (u, v). `[-1, -1]` means invalid.
    pub undistorted_coord: [f32; 2],

    /// Number of consecutive frames this feature has been tracked.
    /// Starts at 0 for newly detected features, increments each frame.
    pub age: u32,
}

impl Feature {
    pub fn new(feature_id: usize, pixel_coord: [f32; 2]) -> Self {
        Self {
            feature_id,
            pixel_coord,
            undistorted_coord: [-1.0, -1.0],
            age: 0,
        }
    }
}

#[derive(Default)]
pub struct PatchTracker<const N: u32> {
    last_keypoint_id: usize,
    tracked_points_map: FxHashMap<usize, na::Affine2<f32>>,
    previous_image_pyramid: Vec<GrayImage>,
}
impl<const LEVELS: u32> PatchTracker<LEVELS> {
    pub fn process_frame(&mut self, greyscale_image: &GrayImage) {
        // build current image pyramid
        let current_image_pyramid: Vec<GrayImage> = build_image_pyramid(greyscale_image, LEVELS);

        if !self.previous_image_pyramid.is_empty() {
            info!("old points {}", self.tracked_points_map.len());
            // track prev points
            // Default values for PatchTracker (not used in estimator)
            const DEFAULT_OPTICAL_FLOW_MAX_ITERATIONS: usize = 30;
            const DEFAULT_OPTICAL_FLOW_CONVERGENCE_THRESHOLD: f32 = 0.005;
            self.tracked_points_map = track_points::<LEVELS>(
                &self.previous_image_pyramid,
                &current_image_pyramid,
                &self.tracked_points_map,
                DEFAULT_OPTICAL_FLOW_MAX_ITERATIONS,
                DEFAULT_OPTICAL_FLOW_CONVERGENCE_THRESHOLD,
                [0.0, 0.0, 0.0],
            );
            info!("tracked old points {}", self.tracked_points_map.len());
        }
        // add new points
        // Default grid_size for PatchTracker (not used in estimator)
        const DEFAULT_GRID_SIZE: u32 = 30;
        let mut scratch_corners = Vec::new();
        let mut scratch_grid = Vec::new();
        let new_points = add_points(&self.tracked_points_map, greyscale_image, DEFAULT_GRID_SIZE, &mut scratch_corners, &mut scratch_grid);
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
    pub fn remove_id(&mut self, ids: &[usize]) {
        for id in ids {
            self.tracked_points_map.remove(id);
        }
    }
}

pub struct StereoPatchTracker<const N: u32> {
    last_keypoint_id: usize,
    tracked_points_map_cam0: FxHashMap<usize, na::Affine2<f32>>,
    previous_image_pyramid0: Vec<GrayImage>,
    tracked_points_map_cam1: FxHashMap<usize, na::Affine2<f32>>,
    previous_image_pyramid1: Vec<GrayImage>,
    /// Scratch pyramids for the current frame; swapped with previous at end of each frame
    /// to avoid heap allocation/deallocation of pyramid-level `Vec<u8>` buffers every frame.
    scratch_pyramid0: Vec<GrayImage>,
    scratch_pyramid1: Vec<GrayImage>,
    grid_size: u32,
    optical_flow_max_iterations: usize,
    optical_flow_convergence_threshold: f32,
    /// Persistent age counter per feature ID: number of frames tracked so far.
    age_map_cam0: FxHashMap<usize, u32>,
    /// Median pixel displacement + in-plane rotation from the last frame, used to
    /// warm-start LK tracking: `[dx, dy, dtheta_rad]`.
    /// Translation reduces iterations for translational motion; rotation hint
    /// pre-rotates the initial SE2 transform so LK only corrects the residual,
    /// improving convergence and tracking success rate during camera rotation.
    velocity_hint_cam0: [f32; 3],
    velocity_hint_cam1: [f32; 3],
    /// Reusable scratch Vecs for median velocity / disparity computation.
    /// Allocated once (first frame) then reused via .clear(), avoiding per-frame heap allocs.
    scratch_dx: Vec<f32>,
    scratch_dy: Vec<f32>,
    scratch_dtheta: Vec<f32>,
    /// Scratch HashMap for new-point stereo matching (indices 0..N → Affine2 transforms).
    /// Avoids a per-frame allocation for the temporary map used in track_points for new features.
    scratch_new_points: FxHashMap<usize, na::Affine2<f32>>,
    /// Scratch `Vec<Corner>` for the current-corner positions passed to detect_key_points.
    scratch_current_corners: Vec<imageproc::corners::Corner>,
    /// Scratch grid Vec for detect_key_points: avoids re-allocating the num_cells grid
    /// (typically ~1500 Corner entries = ~18 KB) on every call after the first frame.
    scratch_detect_grid: Vec<imageproc::corners::Corner>,
}

impl<const LEVELS: u32> StereoPatchTracker<LEVELS> {
    pub fn new(
        grid_size: u32,
        optical_flow_max_iterations: u32,
        optical_flow_convergence_threshold: f64,
    ) -> Self {
        Self {
            last_keypoint_id: 0,
            tracked_points_map_cam0: FxHashMap::default(),
            previous_image_pyramid0: Vec::new(),
            tracked_points_map_cam1: FxHashMap::default(),
            previous_image_pyramid1: Vec::new(),
            scratch_pyramid0: Vec::new(),
            scratch_pyramid1: Vec::new(),
            grid_size,
            optical_flow_max_iterations: optical_flow_max_iterations as usize,
            optical_flow_convergence_threshold: optical_flow_convergence_threshold as f32,
            age_map_cam0: FxHashMap::default(),
            velocity_hint_cam0: [0.0, 0.0, 0.0],
            velocity_hint_cam1: [0.0, 0.0, 0.0],
            scratch_dx: Vec::new(),
            scratch_dy: Vec::new(),
            scratch_dtheta: Vec::new(),
            scratch_new_points: FxHashMap::default(),
            scratch_current_corners: Vec::new(),
            scratch_detect_grid: Vec::new(),
        }
    }

    #[allow(clippy::too_many_lines)]
    pub fn process_frame(
        &mut self,
        greyscale_image0: &GrayImage,
        greyscale_image1: &GrayImage,
        frame: &mut crate::estimator::Frame,
    ) {
        // Fill scratch pyramids in parallel — cam0 and cam1 are independent.
        // After the first two frames the backing `Vec<u8>` buffers are reused (no malloc/free).
        {
            let (scratch0, scratch1) = (&mut self.scratch_pyramid0, &mut self.scratch_pyramid1);
            rayon::join(
                || fill_image_pyramid(greyscale_image0, LEVELS, scratch0),
                || fill_image_pyramid(greyscale_image1, LEVELS, scratch1),
            );
        }

        // not initialized
        if !self.previous_image_pyramid0.is_empty() {
            log::debug!(
                "[FeatureTracker] Number of old points in cam0: {}",
                self.tracked_points_map_cam0.len()
            );
            // Track cam0 and cam1 features in parallel — fully independent operations.
            // Extract field references before closures to avoid whole-self borrow.
            let iters = self.optical_flow_max_iterations;
            let thresh = self.optical_flow_convergence_threshold;
            let hint0 = self.velocity_hint_cam0;
            let hint1 = self.velocity_hint_cam1;
            let prev0 = &self.previous_image_pyramid0;
            let prev1 = &self.previous_image_pyramid1;
            let cur0 = &self.scratch_pyramid0;
            let cur1 = &self.scratch_pyramid1;
            let map0 = &self.tracked_points_map_cam0;
            let map1 = &self.tracked_points_map_cam1;
            let (new_map0, new_map1) = rayon::join(
                || track_points::<LEVELS>(prev0, cur0, map0, iters, thresh, hint0),
                || track_points::<LEVELS>(prev1, cur1, map1, iters, thresh, hint1),
            );
            // Update velocity hints from the tracked displacements before overwriting maps.
            // Pass scratch Vecs to avoid per-call heap allocation (reused across frames).
            self.velocity_hint_cam0 = compute_median_velocity(
                &self.tracked_points_map_cam0, &new_map0,
                &mut self.scratch_dx, &mut self.scratch_dy, &mut self.scratch_dtheta,
            );
            self.velocity_hint_cam1 = compute_median_velocity(
                &self.tracked_points_map_cam1, &new_map1,
                &mut self.scratch_dx, &mut self.scratch_dy, &mut self.scratch_dtheta,
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
            &mut self.scratch_current_corners,
            &mut self.scratch_detect_grid,
        );
        // Reuse scratch_new_points via clear()+extend() to avoid per-frame HashMap allocation.
        self.scratch_new_points.clear();
        self.scratch_new_points.extend(new_points0.iter().enumerate().map(|(i, point)| {
            let mut v = na::Affine2::<f32>::identity();
            v.matrix_mut_unchecked().m13 = point.x as f32;
            v.matrix_mut_unchecked().m23 = point.y as f32;
            (i, v)
        }));

        // Warm-start stereo matching with the median disparity from existing pairs.
        let stereo_hint = compute_stereo_disparity_hint(
            &self.tracked_points_map_cam0,
            &self.tracked_points_map_cam1,
            &mut self.scratch_dx,
            &mut self.scratch_dy,
        );
        let tmp_tracked_points1 = track_points::<LEVELS>(
            &self.scratch_pyramid0,
            &self.scratch_pyramid1,
            &self.scratch_new_points,
            self.optical_flow_max_iterations,
            self.optical_flow_convergence_threshold,
            [stereo_hint[0], stereo_hint[1], 0.0],
        );

        // Update age map BEFORE inserting new features:
        // drop IDs that were lost in track_points, increment survivors.
        // New features added below start at age=0 and must not be incremented this frame.
        // Single retain pass: drop dead entries and increment survivors in one traversal.
        {
            let tracked = &self.tracked_points_map_cam0;
            self.age_map_cam0.retain(|k, age| {
                if tracked.contains_key(k) {
                    *age += 1;
                    true
                } else {
                    false
                }
            });
        }

        for (key0, pt0) in self.scratch_new_points.drain() {
            if let Some(pt1) = tmp_tracked_points1.get(&key0) {
                self.tracked_points_map_cam0
                    .insert(self.last_keypoint_id, pt0);
                self.tracked_points_map_cam1
                    .insert(self.last_keypoint_id, *pt1);
                self.age_map_cam0.insert(self.last_keypoint_id, 0);
                self.last_keypoint_id += 1;
            }
        }

        // Stereo re-match cam0 features that lost their cam1 temporal track.
        // cam0 and cam1 track independently, so a feature may remain in cam0 but drop
        // from cam1.  Without a stereo match the landmark falls back to depth=2.0,
        // which degrades scale accuracy — especially for rotation-heavy sequences where
        // cam1 temporal tracking is less stable.
        // Re-use scratch_new_points (just drained above) as a temporary buffer.
        self.scratch_new_points.extend(
            self.tracked_points_map_cam0.iter()
                .filter(|(id, _)| !self.tracked_points_map_cam1.contains_key(id))
                .map(|(&id, &t)| (id, t))
        );
        if !self.scratch_new_points.is_empty() {
            let re_matched = track_points::<LEVELS>(
                &self.scratch_pyramid0,
                &self.scratch_pyramid1,
                &self.scratch_new_points,
                self.optical_flow_max_iterations,
                self.optical_flow_convergence_threshold,
                [stereo_hint[0], stereo_hint[1], 0.0],
            );
            self.tracked_points_map_cam1.extend(re_matched);
            self.scratch_new_points.clear();
        }

        // Swap scratch ↔ previous: O(1) pointer swap, no heap allocation or deallocation.
        // After the swap, scratch_pyramid holds the old previous (will be refilled next frame).
        std::mem::swap(&mut self.previous_image_pyramid0, &mut self.scratch_pyramid0);
        std::mem::swap(&mut self.previous_image_pyramid1, &mut self.scratch_pyramid1);

        // Populate the frame's feature lists — iterate the tracking maps directly
        // instead of calling get_track_points(), which allocates two temporary HashMaps.
        for (&id, transform) in &self.tracked_points_map_cam0 {
            let mut f = Feature::new(id, [transform.matrix().m13, transform.matrix().m23]);
            f.age = self.age_map_cam0.get(&id).copied().unwrap_or(0);
            frame.add_left_feature(f);
        }
        for (&id, transform) in &self.tracked_points_map_cam1 {
            // Right features mirror left ages (stereo pairs share the same track ID)
            let mut f = Feature::new(id, [transform.matrix().m13, transform.matrix().m23]);
            f.age = self.age_map_cam0.get(&id).copied().unwrap_or(0);
            frame.add_right_feature(f);
        }
    }
    pub fn remove_id(&mut self, ids: &[usize]) {
        for id in ids {
            self.tracked_points_map_cam0.remove(id);
            self.tracked_points_map_cam1.remove(id);
        }
    }
}

/// Downsample an image by exactly 2× using a 2×2 box average.
///
/// Each output pixel is the integer average of the 2×2 input block,
/// computed entirely with u16 arithmetic (no floats, no resampling overhead).
/// This is both faster than `imageops::resize(..., Triangle)` and gives
/// better anti-aliasing for exact 2× downsampling (box filter = ideal).
#[inline]
pub fn halve_image(src: &GrayImage) -> GrayImage {
    let (w, h) = src.dimensions();
    let (nw, nh) = ((w / 2).max(1), (h / 2).max(1));
    let raw = src.as_raw();
    let mut out = vec![0u8; (nw * nh) as usize];
    let stride = w as usize;
    let nw_usize = nw as usize;
    for y in 0..nh as usize {
        // Slice exactly 2×nw bytes per row — gives LLVM contiguous-chunk semantics
        // that enable better auto-vectorisation than the 2*x indexed loop.
        let row0 = &raw[2 * y * stride..2 * y * stride + 2 * nw_usize];
        let row1 = &raw[(2 * y + 1) * stride..(2 * y + 1) * stride + 2 * nw_usize];
        let out_row = &mut out[y * nw_usize..(y + 1) * nw_usize];
        for (dst, (r0, r1)) in out_row
            .iter_mut()
            .zip(row0.chunks_exact(2).zip(row1.chunks_exact(2)))
        {
            let sum = r0[0] as u16 + r0[1] as u16 + r1[0] as u16 + r1[1] as u16;
            *dst = ((sum + 2) >> 2) as u8;
        }
    }
    GrayImage::from_raw(nw, nh, out).expect("Image dimensions must be valid")
}

pub fn build_image_pyramid(greyscale_image: &GrayImage, levels: u32) -> Vec<GrayImage> {
    // Build pyramid hierarchically — each level halved from the previous.
    // Total pixel work: W×H × (1 + ¼ + 1/16 + …) ≈ 1.33×W×H,
    // vs 2×W×H if each level were downsampled from the full original.
    let mut pyramid = Vec::with_capacity(levels as usize);
    pyramid.push(greyscale_image.clone()); // Level 0 = original
    for _ in 1..levels {
        let prev = pyramid.last().expect("Pyramid should have at least one level");
        pyramid.push(halve_image(prev));
    }
    pyramid
}

/// Downsample `src` into `dst` by exactly 2×, writing into `dst`'s existing backing buffer.
/// `dst` must already have the correct (half) dimensions — guaranteed after the first frame.
#[inline]
fn halve_image_into(src: &GrayImage, dst: &mut GrayImage) {
    let (w, h) = src.dimensions();
    let nw = (w / 2).max(1) as usize;
    let nh = (h / 2).max(1) as usize;
    let raw_src = src.as_raw();
    let raw_dst = dst.as_mut();
    let stride = w as usize;
    for y in 0..nh {
        let row0 = &raw_src[2 * y * stride..2 * y * stride + 2 * nw];
        let row1 = &raw_src[(2 * y + 1) * stride..(2 * y + 1) * stride + 2 * nw];
        let out_row = &mut raw_dst[y * nw..(y + 1) * nw];
        for (dst_px, (r0, r1)) in out_row
            .iter_mut()
            .zip(row0.chunks_exact(2).zip(row1.chunks_exact(2)))
        {
            let sum = r0[0] as u16 + r0[1] as u16 + r1[0] as u16 + r1[1] as u16;
            *dst_px = ((sum + 2) >> 2) as u8;
        }
    }
}

/// Fill `pyramid` in-place from `src`, reusing existing ``Vec<u8>`` backing buffers.
/// Falls back to `build_image_pyramid` on the first call (when the pyramid is uninitialized).
fn fill_image_pyramid(src: &GrayImage, levels: u32, pyramid: &mut Vec<GrayImage>) {
    if pyramid.len() != levels as usize {
        // First call: allocate all levels.
        *pyramid = build_image_pyramid(src, levels);
        return;
    }
    // Refill level 0: overwrite existing buffer with the new source image.
    pyramid[0].as_mut().copy_from_slice(src.as_raw());
    // Refill each subsequent level from the previous, reusing backing buffers.
    for i in 1..levels as usize {
        let (left, right) = pyramid.split_at_mut(i);
        halve_image_into(&left[i - 1], &mut right[0]);
    }
}

/// `scratch` is a reusable Vec that is cleared and filled with current corner positions.
/// `grid_scratch` is reused for the internal grid in detect_key_points (~18 KB after first call).
fn add_points(
    tracked_points_map: &FxHashMap<usize, na::Affine2<f32>>,
    grayscale_image: &GrayImage,
    grid_size: u32,
    scratch: &mut Vec<Corner>,
    grid_scratch: &mut Vec<Corner>,
) -> Vec<Corner> {
    // Skip FAST9 detection when the grid is already ≥75% occupied.
    // `detect_key_points` can return at most (num_cells − N_tracked) new corners,
    // so running it when N_tracked ≥ 3/4 · num_cells yields fewer than num_cells/4
    // new features — not worth the ~2 ms FAST9 full-image scan.
    // Uses integer arithmetic (×4 / ×3) to avoid floating-point division.
    let (w, h) = (grayscale_image.width(), grayscale_image.height());
    let num_cells = ((w / grid_size) as usize + 1) * ((h / grid_size) as usize + 1);
    if tracked_points_map.len() * 4 >= num_cells * 3 {
        return Vec::new();
    }
    scratch.clear();
    scratch.extend(tracked_points_map.values().map(|v| {
        Corner::new(
            v.matrix().m13.round() as u32,
            v.matrix().m23.round() as u32,
            0.0,
        )
    }));
    image_utilities::detect_key_points(grayscale_image, grid_size, scratch, grid_scratch)
}
/// Compute the median (x, y) stereo disparity from currently tracked stereo pairs.
/// Used to warm-start LK when matching new features from cam0 to cam1.
/// Requires at least 4 pairs; returns `[0.0, 0.0]` otherwise.
///
/// `scratch_dx` / `scratch_dy` are reusable buffers (cleared on entry) to avoid
/// per-call heap allocation.
fn compute_stereo_disparity_hint(
    map_cam0: &FxHashMap<usize, na::Affine2<f32>>,
    map_cam1: &FxHashMap<usize, na::Affine2<f32>>,
    scratch_dx: &mut Vec<f32>,
    scratch_dy: &mut Vec<f32>,
) -> [f32; 2] {
    scratch_dx.clear();
    scratch_dy.clear();
    for (k, v0) in map_cam0 {
        if let Some(v1) = map_cam1.get(k) {
            scratch_dx.push(v1.matrix().m13 - v0.matrix().m13);
            scratch_dy.push(v1.matrix().m23 - v0.matrix().m23);
        }
    }
    if scratch_dx.len() < 4 {
        return [0.0, 0.0];
    }
    let mid = scratch_dx.len() / 2;
    scratch_dx.select_nth_unstable_by(mid, |a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    scratch_dy.select_nth_unstable_by(mid, |a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    [scratch_dx[mid], scratch_dy[mid]]
}

/// Compute the median `[dx, dy, dtheta]` between matching entries in `old_map` and `new_map`.
///
/// `dtheta` is the median in-plane rotation change (radians), computed from the SE2
/// rotation components via `delta_R = old_R^T * new_R`.  Used to warm-start the
/// rotation of `transform1` in `track_one_point`, reducing LK iterations and improving
/// tracking success during camera rotation.
///
/// Requires at least 4 matching features; returns `[0.0, 0.0, 0.0]` otherwise.
/// `scratch_dx` / `scratch_dy` / `scratch_dtheta` are reusable buffers (cleared on entry).
fn compute_median_velocity(
    old_map: &FxHashMap<usize, na::Affine2<f32>>,
    new_map: &FxHashMap<usize, na::Affine2<f32>>,
    scratch_dx: &mut Vec<f32>,
    scratch_dy: &mut Vec<f32>,
    scratch_dtheta: &mut Vec<f32>,
) -> [f32; 3] {
    scratch_dx.clear();
    scratch_dy.clear();
    scratch_dtheta.clear();
    for (k, new_v) in new_map {
        if let Some(old_v) = old_map.get(k) {
            scratch_dx.push(new_v.matrix().m13 - old_v.matrix().m13);
            scratch_dy.push(new_v.matrix().m23 - old_v.matrix().m23);
            // delta_R = old_R^T * new_R; extract angle from its sin and cos.
            // old_R^T * new_R: [0,0] = cos(new-old), [1,0] = sin(new-old)
            let (oc, os) = (old_v.matrix().m11, old_v.matrix().m21); // cos, sin of old
            let (nc, ns) = (new_v.matrix().m11, new_v.matrix().m21); // cos, sin of new
            let delta_cos = oc * nc + os * ns;
            let delta_sin = oc * ns - os * nc;
            scratch_dtheta.push(delta_sin.atan2(delta_cos));
        }
    }
    if scratch_dx.len() < 4 {
        return [0.0, 0.0, 0.0];
    }
    let mid = scratch_dx.len() / 2;
    // O(N) average-case median — faster than O(N log N) sort for this use.
    scratch_dx.select_nth_unstable_by(mid, |a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    scratch_dy.select_nth_unstable_by(mid, |a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    scratch_dtheta.select_nth_unstable_by(mid, |a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    [scratch_dx[mid], scratch_dy[mid], scratch_dtheta[mid]]
}

fn track_points<const LEVELS: u32>(
    image_pyramid0: &[GrayImage],
    image_pyramid1: &[GrayImage],
    transform_maps0: &FxHashMap<usize, na::Affine2<f32>>,
    optical_flow_max_iterations: usize,
    optical_flow_convergence_threshold: f32,
    // Motion hint `[dx, dy, dtheta]`: warm-starts LK translation and rotation.
    // dtheta (radians) pre-rotates transform1 so LK corrects only the residual rotation.
    velocity_hint: [f32; 3],
) -> FxHashMap<usize, na::Affine2<f32>> {
    // Parallel tracking with forward-backward consistency check.
    // The backward pass is a consistency check only — it does not need full
    // convergence, so 3 iterations is sufficient to detect bad tracks without
    // spending the full optical_flow_max_iterations budget a second time.
    let backward_iters = optical_flow_max_iterations.min(3);
    // Precompute squared threshold once — track_point_at_level uses norm_squared()
    // instead of norm() to avoid a sqrt per LK iteration.
    let convergence_threshold_sq = optical_flow_convergence_threshold * optical_flow_convergence_threshold;
    // Precompute rotation hint sin/cos once — same value for every feature in this batch.
    // sin(-θ) = -sh, cos(-θ) = ch, so the backward pass just negates sh.
    let rot_hint = velocity_hint[2];
    let (hint_sh, hint_ch) = if rot_hint != 0.0 { rot_hint.sin_cos() } else { (0.0, 1.0) };
    transform_maps0
        .par_iter()
        .filter_map(|(k, v)| {
            let config = LkTrackConfig {
                optical_flow_max_iterations,
                convergence_threshold_sq,
            };
            let hint = LkMotionHint {
                velocity: velocity_hint,
                sin_theta: hint_sh,
                cos_theta: hint_ch,
            };
            if let Some(new_v) = track_one_point::<LEVELS>(
                image_pyramid0,
                image_pyramid1,
                v,
                config,
                hint,
            ) {
                // Forward-backward consistency check (reduced iterations).
                // Backward hint starts the backward search near the original position.
                let backward_config = LkTrackConfig {
                    optical_flow_max_iterations: backward_iters,
                    convergence_threshold_sq,
                };
                let backward_hint = LkMotionHint {
                    velocity: [-velocity_hint[0], -velocity_hint[1], -rot_hint],
                    sin_theta: -hint_sh,
                    cos_theta: hint_ch,
                };
                if let Some(old_v) = track_one_point::<LEVELS>(
                    image_pyramid1,
                    image_pyramid0,
                    &new_v,
                    backward_config,
                    backward_hint,
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
        .collect()
}

/// Configuration for LK point tracking.
struct LkTrackConfig {
    optical_flow_max_iterations: usize,
    convergence_threshold_sq: f32,
}

/// Motion hint for LK tracking with precomputed sin/cos.
struct LkMotionHint {
    velocity: [f32; 3],
    sin_theta: f32,
    cos_theta: f32,
}

fn track_one_point<const LEVELS: u32>(
    image_pyramid0: &[GrayImage],
    image_pyramid1: &[GrayImage],
    transform0: &na::Affine2<f32>,
    config: LkTrackConfig,
    hint: LkMotionHint,
) -> Option<na::Affine2<f32>> {
    let mut patch_valid = true;
    let mut transform1 = na::Affine2::<f32>::identity();
    // Warm-start translation from velocity hint.
    transform1.matrix_mut_unchecked().m13 = transform0.matrix().m13 + hint.velocity[0];
    transform1.matrix_mut_unchecked().m23 = transform0.matrix().m23 + hint.velocity[1];
    // Warm-start rotation using precomputed sin/cos of the rotation hint.
    if hint.velocity[2] != 0.0 {
        transform1.matrix_mut_unchecked().m11 = hint.cos_theta;
        transform1.matrix_mut_unchecked().m12 = -hint.sin_theta;
        transform1.matrix_mut_unchecked().m21 = hint.sin_theta;
        transform1.matrix_mut_unchecked().m22 = hint.cos_theta;
    }

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
                config.optical_flow_max_iterations,
                config.convergence_threshold_sq,
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
    // Compose rotation blocks only (2×2 × 2×2 = 8 muls + 4 adds) instead of the full
    // 3×3 SE2 matrix multiply (27 muls + 18 adds).  Translation is already in transform1.
    let r0 = transform0.matrix().fixed_view::<2, 2>(0, 0);
    let r1 = transform1.matrix().fixed_view::<2, 2>(0, 0);
    let new_r = r0 * r1;
    transform1.matrix_mut_unchecked().m11 = new_r.m11;
    transform1.matrix_mut_unchecked().m12 = new_r.m12;
    transform1.matrix_mut_unchecked().m21 = new_r.m21;
    transform1.matrix_mut_unchecked().m22 = new_r.m22;
    Some(transform1)
}

#[inline]
pub fn track_point_at_level(
    grayscale_image: &GrayImage,
    dp: &patch::Pattern52,
    transform: &mut na::Affine2<f32>,
    optical_flow_max_iterations: usize,
    // Pre-squared threshold: avoids sqrt per iteration (inc.norm_sq < threshold^2).
    convergence_threshold_sq: f32,
) -> bool {
    // Build pattern matrix from compile-time constant — memcpy, called only for valid patches.
    let patten = na::SMatrix::<f32, 2, { patch::PATTERN52_SIZE }>::from_column_slice(
        &patch::Pattern52::PATTERN_MATRIX_COL_MAJOR,
    );

    const FILTER_MARGIN: f32 = 2.0;
    let x_max = (grayscale_image.width() - 2) as f32;
    let y_max = (grayscale_image.height() - 2) as f32;

    for _iteration in 0..optical_flow_max_iterations {
        // Recompute full R*pattern+t each iteration (rotation changes every step).
        let transformed_pat = {
            let r = transform.matrix().fixed_view::<2, 2>(0, 0);
            let tx = transform.matrix().m13;
            let ty = transform.matrix().m23;
            let mut p = r * patten;
            for chunk in p.as_mut_slice().chunks_exact_mut(2) {
                chunk[0] += tx;
                chunk[1] += ty;
            }
            p
        };
        let center = (transform.matrix().m13, transform.matrix().m23);
        if let Some(res) = dp.residual(grayscale_image, &transformed_pat, center) {
            // h_se2_inv_j_se2_t is SMatrix<f32, 52, 3> (column-major): three contiguous
            // 52-float columns in a single 156-float flat array.
            // Combined 3-way dot product: reads `res` once (vs 3×) and processes h0/h1/h2
            // simultaneously, reducing FMA count by ~33% vs three separate .dot() calls.
            // AVX2 vectorization: load 8 residuals once, multiply with h0/h1/h2 in the same
            // iteration — 4 instructions per 8 elements vs 6 for three separate loops.
            let h = dp.h_se2_inv_j_se2_t.as_slice();
            let r = res.as_slice();
            let (mut s0, mut s1, mut s2) = (0.0f32, 0.0f32, 0.0f32);
            for i in 0..patch::PATTERN52_SIZE {
                // SAFETY: i < 52, PATTERN52_SIZE = 52; h has 156 elements (3 columns of 52).
                let ri = unsafe { *r.get_unchecked(i) };
                s0 += ri * unsafe { *h.get_unchecked(i) };
                s1 += ri * unsafe { *h.get_unchecked(i + patch::PATTERN52_SIZE) };
                s2 += ri * unsafe { *h.get_unchecked(i + 2 * patch::PATTERN52_SIZE) };
            }
            let inc = na::SVector::<f32, 3>::new(-s0, -s1, -s2);

            // Compare squared norm to avoid sqrt per iteration.
            // Divergence: norm > 1e6  ⟺  norm_sq > 1e12.
            let inc_norm_sq = inc.norm_squared();
            if !inc_norm_sq.is_finite() || inc_norm_sq > 1e12 {
                return false;
            }

            // Early termination if converged (threshold already squared by caller).
            if inc_norm_sq < convergence_threshold_sq {
                break;
            }

            image_utilities::apply_se2_update(transform, &inc);

            let tx = transform.matrix().m13;
            let ty = transform.matrix().m23;
            if tx < FILTER_MARGIN || ty < FILTER_MARGIN || tx >= x_max || ty >= y_max {
                return false;
            }
        } else {
            return false;
        }
    }

    true
}
