use image::GrayImage;
use nalgebra as na;

use super::image_utilities;

pub const PATTERN52_SIZE: usize = 52;
pub struct Pattern52 {
    pub valid: bool,
    /// True when all 52 source-pattern points were in-bounds during construction.
    /// Used in `residual()` to skip per-point validity checks on the source side
    /// (the target-side `bbox_safe` check covers the other condition).
    pub all_valid: bool,
    pub mean: f32,
    pub pos: na::SVector<f32, 2>,
    pub data: [f32; PATTERN52_SIZE], // negative if the point is not valid
    /// `H⁻¹ J^T` stored as a `PATTERN52_SIZE × 3` matrix (transposed from the natural 3 × 52).
    ///
    /// Column-major layout with 52 rows gives three contiguous 52-float columns, one per SE2
    /// increment component.  The LK update `inc = -(H⁻¹ J^T) res` becomes three independent
    /// 52-element dot products of contiguous memory, enabling AVX2 8-wide SIMD — vs the original
    /// 3 × 52 layout whose rows had stride 3 (12-byte gaps), which cannot be vectorized efficiently.
    pub h_se2_inv_j_se2_t: na::SMatrix<f32, PATTERN52_SIZE, 3>,
}
impl Pattern52 {
    pub const PATTERN_RAW: [[f32; 2]; PATTERN52_SIZE] = [
        [-3.0, 7.0],
        [-1.0, 7.0],
        [1.0, 7.0],
        [3.0, 7.0],
        [-5.0, 5.0],
        [-3.0, 5.0],
        [-1.0, 5.0],
        [1.0, 5.0],
        [3.0, 5.0],
        [5.0, 5.0],
        [-7.0, 3.0],
        [-5.0, 3.0],
        [-3.0, 3.0],
        [-1.0, 3.0],
        [1.0, 3.0],
        [3.0, 3.0],
        [5.0, 3.0],
        [7.0, 3.0],
        [-7.0, 1.0],
        [-5.0, 1.0],
        [-3.0, 1.0],
        [-1.0, 1.0],
        [1.0, 1.0],
        [3.0, 1.0],
        [5.0, 1.0],
        [7.0, 1.0],
        [-7.0, -1.0],
        [-5.0, -1.0],
        [-3.0, -1.0],
        [-1.0, -1.0],
        [1.0, -1.0],
        [3.0, -1.0],
        [5.0, -1.0],
        [7.0, -1.0],
        [-7.0, -3.0],
        [-5.0, -3.0],
        [-3.0, -3.0],
        [-1.0, -3.0],
        [1.0, -3.0],
        [3.0, -3.0],
        [5.0, -3.0],
        [7.0, -3.0],
        [-5.0, -5.0],
        [-3.0, -5.0],
        [-1.0, -5.0],
        [1.0, -5.0],
        [3.0, -5.0],
        [5.0, -5.0],
        [-3.0, -7.0],
        [-1.0, -7.0],
        [1.0, -7.0],
        [3.0, -7.0],
    ];

    /// Pre-computed scaled pattern matrix at compile time (PATTERN_RAW / 2.0).
    /// Avoids 104 floating-point divisions per `Pattern52::new()`.
    pub const PATTERN_SCALED: [[f32; 2]; PATTERN52_SIZE] = {
        let mut out = [[0.0f32; 2]; PATTERN52_SIZE];
        let mut i = 0;
        while i < PATTERN52_SIZE {
            out[i][0] = Self::PATTERN_RAW[i][0] / 2.0;
            out[i][1] = Self::PATTERN_RAW[i][1] / 2.0;
            i += 1;
        }
        out
    };

    /// Column-major flat array for `SMatrix::<f32, 2, PATTERN52_SIZE>` construction.
    /// Nalgebra stores matrices column-major: column `j` = [x_j, y_j] = PATTERN_SCALED`[j]`.
    /// Using `from_column_slice` with a pre-built const avoids the per-element closure
    /// overhead of `from_fn` when constructing the pattern matrix in `new()`.
    pub const PATTERN_MATRIX_COL_MAJOR: [f32; PATTERN52_SIZE * 2] = {
        let mut out = [0.0f32; PATTERN52_SIZE * 2];
        let mut j = 0;
        while j < PATTERN52_SIZE {
            out[j * 2] = Self::PATTERN_SCALED[j][0];
            out[j * 2 + 1] = Self::PATTERN_SCALED[j][1];
            j += 1;
        }
        out
    };

    // verified
    #[allow(clippy::too_many_lines)]
    pub fn set_data_jac_se2(
        &mut self,
        greyscale_image: &GrayImage,
        j_se2: &mut na::SMatrix<f32, PATTERN52_SIZE, 3>,
    ) {
        let mut num_valid_points = 0;
        let mut sum: f32 = 0.0;
        let mut gs0: f32 = 0.0;
        let mut gs1: f32 = 0.0;
        let mut gs2: f32 = 0.0;

        // Single bbox check: if the feature centre is >= PATTERN_SAFE_MARGIN from all
        // borders, every pattern point is in-bounds and per-point inbound() calls are
        // skipped (saves 52 × 4 comparisons in the common case).
        let w = greyscale_image.width() as f32;
        let h = greyscale_image.height() as f32;
        let bbox_safe = self.pos.x >= Self::PATTERN_SAFE_MARGIN
            && self.pos.y >= Self::PATTERN_SAFE_MARGIN
            && self.pos.x < w - Self::PATTERN_SAFE_MARGIN
            && self.pos.y < h - Self::PATTERN_SAFE_MARGIN;

        // Use PATTERN_SCALED (= PATTERN_RAW / 2.0, precomputed at compile time)
        // to avoid 208 divisions per call (52 points × 4 divisions each).
        // Split by bbox_safe to eliminate the per-point inbound() check in the common case,
        // letting LLVM schedule the image_grad calls without a conditional branch.
        if bbox_safe {
            // Fast path: pre-load a 12×12 pixel block into a stack array before the 52-point loop.
            //
            // Each image_grad call reads 12 pixels in a 4×4 neighbourhood.  The 52 pattern
            // points span ±3.5 px from centre, and image_grad needs ±1 px beyond for finite
            // differences, giving a total read extent of [cx-5, cx+6] × [cy-5, cy+6] — exactly
            // 12×12 pixels.  Pre-loading trades 12 sequential row copies (prefetcher-friendly)
            // for 52 × 12 = 624 scattered reads from the full image, converting potential L2/L3
            // misses (level-0 image is 128 KB at 512×256) to guaranteed L1 hits on a 144-byte array.
            //
            // SAFETY: bbox_safe guarantees pos.x >= 5.5, so floor(pos.x) >= 5, ix_min = floor(pos.x)-5 >= 0.
            //   pos.x < w-5.5  ⟹  floor(pos.x) <= w-6  ⟹  ix_min+BLOCK-1 = floor(pos.x)+6 <= w-1.
            //   Same for y.  Therefore every row copy of BLOCK bytes is within raw_pixels.
            const BLOCK: usize = 12;
            let cx_int = self.pos.x as usize;
            let cy_int = self.pos.y as usize;
            let ix_min = cx_int - 5;
            let iy_min = cy_int - 5;
            let width_usize = w as usize;
            let raw_pixels = greyscale_image.as_raw();
            let mut local_px = [0u8; BLOCK * BLOCK];
            for row in 0..BLOCK {
                let src_start = (iy_min + row) * width_usize + ix_min;
                unsafe {
                    local_px[row * BLOCK..(row + 1) * BLOCK].copy_from_slice(
                        std::slice::from_raw_parts(raw_pixels.get_unchecked(src_start), BLOCK),
                    );
                }
            }
            // Inlined image_grad using local_px instead of the full image.
            // image_grad reads a 4×4 block: rows [iy-1..iy+2], cols [ix-1..ix+3].
            // Mapped to block: bx = ix-1-ix_min, by = iy-1-iy_min; base_off = by*BLOCK+bx.
            for (i, sp) in Self::PATTERN_SCALED.iter().enumerate() {
                let px = self.pos.x + sp[0];
                let py = self.pos.y + sp[1];
                let ix = px as usize;
                let iy = py as usize;
                let dx = px - ix as f32;
                let dy = py - iy as f32;
                let w11 = dx * dy;
                let w00 = 1.0 - dx - dy + w11;
                let w10 = dx - w11;
                let w01 = dy - w11;
                // base at (ix-1, iy-1) in local block
                let base_off = (iy - 1 - iy_min) * BLOCK + (ix - 1 - ix_min);
                // SAFETY: bbox_safe guarantees all accesses within [0, BLOCK*BLOCK).
                // Worst case: max ix = cx_int+3, ix-1 = cx_int+2, ix-1-ix_min = cx_int+2-(cx_int-5)=7.
                // base_off max = 7*12+7 = 91; max offset below = 3*12+3 = 39; 91+39=130 < 144. ✓
                let (p10, p20, p01, p11, p21, p31, p02, p12, p22, p32, p13, p23) = unsafe {(
                    *local_px.get_unchecked(base_off + 1)              as f32,
                    *local_px.get_unchecked(base_off + 2)              as f32,
                    *local_px.get_unchecked(base_off + BLOCK)          as f32,
                    *local_px.get_unchecked(base_off + BLOCK + 1)      as f32,
                    *local_px.get_unchecked(base_off + BLOCK + 2)      as f32,
                    *local_px.get_unchecked(base_off + BLOCK + 3)      as f32,
                    *local_px.get_unchecked(base_off + 2 * BLOCK)      as f32,
                    *local_px.get_unchecked(base_off + 2 * BLOCK + 1)  as f32,
                    *local_px.get_unchecked(base_off + 2 * BLOCK + 2)  as f32,
                    *local_px.get_unchecked(base_off + 2 * BLOCK + 3)  as f32,
                    *local_px.get_unchecked(base_off + 3 * BLOCK + 1)  as f32,
                    *local_px.get_unchecked(base_off + 3 * BLOCK + 2)  as f32,
                )};
                let val = w00 * p11 + w01 * p12 + w10 * p21 + w11 * p22;
                let gx  = 0.5 * (w00 * (p21 - p01) + w01 * (p22 - p02)
                               + w10 * (p31 - p11) + w11 * (p32 - p12));
                let gy  = 0.5 * (w00 * (p12 - p10) + w01 * (p13 - p11)
                               + w10 * (p22 - p20) + w11 * (p23 - p21));
                let r2 = gy * sp[0] - gx * sp[1];
                j_se2[(i, 0)] = gx;
                j_se2[(i, 1)] = gy;
                j_se2[(i, 2)] = r2;
                gs0 += gx;
                gs1 += gy;
                gs2 += r2;
                self.data[i] = val;
                sum += val;
            }
            num_valid_points = PATTERN52_SIZE;
        } else {
            // Slow path: per-point inbound check needed.
            for (i, sp) in Self::PATTERN_SCALED.iter().enumerate() {
                let p = self.pos + na::SVector::<f32, 2>::new(sp[0], sp[1]);
                if image_utilities::inbound(greyscale_image, p.x, p.y, 2) {
                    let val_grad = image_utilities::image_grad(greyscale_image, p.x, p.y);
                    let gx = val_grad[1];
                    let gy = val_grad[2];
                    let r2 = gy * sp[0] - gx * sp[1];
                    j_se2[(i, 0)] = gx;
                    j_se2[(i, 1)] = gy;
                    j_se2[(i, 2)] = r2;
                    gs0 += gx;
                    gs1 += gy;
                    gs2 += r2;
                    self.data[i] = val_grad[0];
                    sum += val_grad[0];
                    num_valid_points += 1;
                } else {
                    self.data[i] = -1.0;
                }
            }
        }
        self.mean = sum / num_valid_points as f32;
        let mean_inv = num_valid_points as f32 / sum;
        self.all_valid = num_valid_points == PATTERN52_SIZE;

        // Mean-normalisation correction fused with mean_inv scale.
        // j_se2 is SMatrix<f32, 52, 3> (column-major): column k occupies a contiguous
        // 52-float block at offset k*52.  Processing one column at a time lets LLVM
        // use SIMD on a stride-1 stream, unlike the previous row-by-row loop which
        // accessed three non-contiguous columns per iteration (stride 52 between them).
        let s_inv = 1.0 / sum;
        let gs = [gs0, gs1, gs2];
        if self.all_valid {
            // Fast path: all 52 data[] entries are non-negative — skip the d.max(0.0) clamp.
            // Pure FMA form `j_val = j_val * mean_inv - coeff * d` lets LLVM generate a
            // tight AVX2 FMA loop with no maxps instruction, saving one op per element
            // (52 elements × 3 columns = 156 maxps eliminated per Pattern52::new call).
            for (col, &gs_k) in gs.iter().enumerate() {
                let coeff = gs_k * s_inv * mean_inv;
                for (j_val, &d) in j_se2.column_mut(col).iter_mut().zip(self.data.iter()) {
                    *j_val = *j_val * mean_inv - coeff * d;
                }
            }
        } else {
            // Slow path: invalid points have j_se2[i] = 0.0 (never set in first loop)
            // and d = -1.0 (clamped to 0.0 via max), so both terms vanish.
            for (col, &gs_k) in gs.iter().enumerate() {
                let coeff = gs_k * s_inv * mean_inv;
                for (j_val, &d) in j_se2.column_mut(col).iter_mut().zip(self.data.iter()) {
                    *j_val = *j_val * mean_inv - coeff * d.max(0.0);
                }
            }
        }
        // Normalize data unconditionally: invalid entries hold -1.0, and
        // -1.0 * mean_inv = -(mean_inv) < 0, so the `>= 0.0` guard in residual() still works.
        for d in self.data.iter_mut() {
            *d *= mean_inv;
        }
    }
    pub fn new(greyscale_image: &GrayImage, px: f32, py: f32) -> Pattern52 {
        let mut j_se2 = na::SMatrix::<f32, PATTERN52_SIZE, 3>::zeros();

        let mut p = Pattern52 {
            valid: false,
            all_valid: false,
            mean: 1.0,
            pos: na::SVector::<f32, 2>::new(px, py),
            data: [0.0; PATTERN52_SIZE], // negative if the point is not valid
            h_se2_inv_j_se2_t: na::SMatrix::<f32, 52, 3>::zeros(),
        };
        p.set_data_jac_se2(greyscale_image, &mut j_se2);
        // Exploit symmetry of H = J^T·J: compute only 6 unique dot products (upper triangle)
        // instead of the full 9 from j_se2.transpose() * j_se2.
        // Saves 3 × PATTERN52_SIZE = 156 multiplications per construction.
        let h00 = j_se2.column(0).dot(&j_se2.column(0));
        let h01 = j_se2.column(0).dot(&j_se2.column(1));
        let h02 = j_se2.column(0).dot(&j_se2.column(2));
        let h11 = j_se2.column(1).dot(&j_se2.column(1));
        let h12 = j_se2.column(1).dot(&j_se2.column(2));
        let h22 = j_se2.column(2).dot(&j_se2.column(2));
        let h_se2 = na::SMatrix::<f32, 3, 3>::new(
            h00, h01, h02,
            h01, h11, h12,
            h02, h12, h22,
        );
        // Compute H^{-1} as a 3×3 matrix (Cholesky solve against I₃: 36 FMAs), then
        // multiply j_se2 × H^{-1} (52×3 × 3×3 = 780 ops) to get (H^{-1} J^T)^T directly.
        //
        // vs the prior approach: j_se2.transpose() (156-float copy) + solve against 52 RHS
        // (624 FMAs) + j_t.transpose() (156-float copy) = 936 ops + 2 transpositions.
        //
        // Since H is symmetric, (H^{-1} J^T)^T = J (H^{-1})^T = J × H^{-1} = j_se2 × h_inv.
        if let Some(x) = h_se2.cholesky() {
            let mut h_inv = na::Matrix3::<f32>::identity();
            x.solve_mut(&mut h_inv);           // h_inv = H^{-1} (3×3 solve against I₃)
            p.h_se2_inv_j_se2_t = j_se2 * h_inv; // (H^{-1} J^T)^T in 52×3 layout

            // Guard: if mean ≤ ε the patch is all-black (sum ≈ 0) and mean_inv = N/sum
            // would produce inf/NaN.  With u8 pixel inputs, gradients are always finite,
            // and if mean > ε the Cholesky solve and normalisation are guaranteed finite,
            // so the previous .all(|x| x.is_finite()) scans (208 comparisons per patch)
            // are redundant and can be removed.
            p.valid = p.mean > f32::EPSILON;
        }

        p
    }
    /// Max half-extent of the scaled pattern (PATTERN_RAW max coord 7 / scale_down 2)
    /// plus the 2-pixel bilinear-interpolation safety margin.
    const PATTERN_SAFE_MARGIN: f32 = 7.0 / 2.0 + 2.0; // = 5.5

    /// Compute residual between this (source) pattern and the image at `transformed_pattern`.
    ///
    /// `center` is the SE2 translation — the center of `transformed_pattern` in image space.
    /// Passing it avoids a full 52-point min/max scan for the bounding-box safety check.
    #[inline]
    #[allow(clippy::too_many_lines)]
    pub fn residual(
        &self,
        greyscale_image: &GrayImage,
        transformed_pattern: &na::SMatrix<f32, 2, PATTERN52_SIZE>,
        center: (f32, f32),
    ) -> Option<na::SVector<f32, PATTERN52_SIZE>> {
        let mut sum: f32 = 0.0;
        let mut num_valid_points = 0;
        let mut residual = na::SVector::<f32, PATTERN52_SIZE>::zeros();

        let width = greyscale_image.width();
        let height = greyscale_image.height();
        let raw_pixels = greyscale_image.as_raw();
        let width_usize = width as usize; // precompute cast used twice per inner loop iteration

        // 4-comparison bbox check using the known pattern half-extent from the SE2 center.
        // Replaces a 52-point min/max scan (208 comparisons) with 4 comparisons.
        let (cx, cy) = center;
        let w_bound = (width - 2) as f32;
        let h_bound = (height - 2) as f32;
        let bbox_safe = cx >= Self::PATTERN_SAFE_MARGIN
            && cx + Self::PATTERN_SAFE_MARGIN < w_bound
            && cy >= Self::PATTERN_SAFE_MARGIN
            && cy + Self::PATTERN_SAFE_MARGIN < h_bound;

        // SAFETY for get_unchecked below: ix in [2, width-3], iy in [2, height-3] in both paths.
        // For bbox_safe: center check + PATTERN_SAFE_MARGIN guarantees all points in bounds.
        // For slow path: per-point check x>=2 && x<w_bound && y>=2 && y<h_bound is equivalent.
        // Either implies idx00+width_usize+1 = iy*w+ix+w+1 <= (h-3)*w+(w-3)+w+1 = (h-2)*w-2 < len.
        if bbox_safe {
            // Fast path: guaranteed all 52 points are in-bounds; no branch per point.
            // Pre-load a 12×12 pixel block into a stack array.  The 52 bilinear lookups span
            // at most ±4 pixel rows/cols from the center, so a 12×12 block covers all reads.
            // This trades 12 sequential 12-byte row copies (cache-friendly, prefetcher-friendly)
            // for 52×4=208 scattered reads across ~9 image rows separated by width_usize bytes,
            // ensuring all subsequent lookups hit L1 cache instead of potentially L2/L3.
            //
            // SAFETY: bbox_safe requires cx >= 5.5 and cx + 5.5 < width-2, so
            //   ix_min = floor(cx) - 4 >= 1 (floor(cx) >= 5 since cx >= 5.5)
            //   ix_min + BLOCK - 1 = floor(cx) + 7 <= (width-8) + 7 = width-1 ✓
            //   Similarly iy_min and iy_min + BLOCK - 1 are in [1, height-1] ✓
            // SAFETY for block index: pattern x in [cx-3.5, cx+3.5], so floor(x) in
            //   [floor(cx)-4, floor(cx)+4], ix_off = floor(x)-ix_min in [0, 8].
            //   max local_idx + BLOCK + 1 = 8*12 + 8 + 12 + 1 = 117 < 144 = BLOCK*BLOCK ✓
            const BLOCK: usize = 12;
            let ix_min = cx as usize - 4;  // floor(cx) - 4; safe since floor(cx) >= 5
            let iy_min = cy as usize - 4;
            let mut local_px = [0u8; BLOCK * BLOCK];
            for row in 0..BLOCK {
                let src_start = (iy_min + row) * width_usize + ix_min;
                // SAFETY: src_start to src_start+BLOCK are within raw_pixels (proven above).
                unsafe {
                    local_px[row * BLOCK..(row + 1) * BLOCK].copy_from_slice(
                        std::slice::from_raw_parts(raw_pixels.get_unchecked(src_start), BLOCK),
                    );
                }
            }
            for i in 0..PATTERN52_SIZE {
                let x = transformed_pattern[(0, i)];
                let y = transformed_pattern[(1, i)];
                let ix = x as usize;
                let iy = y as usize;
                let dx = x - ix as f32;
                let dy = y - iy as f32;
                let w11 = dx * dy;
                let w00 = 1.0 - dx - dy + w11;
                let w10 = dx - w11;
                let w01 = dy - w11;
                // Offset within the pre-loaded block — avoids multiply by runtime `width`.
                // BLOCK=12 is a compile-time constant so iy_off*12 compiles to shift+add.
                let ix_off = ix - ix_min;
                let iy_off = iy - iy_min;
                let local_idx = iy_off * BLOCK + ix_off;
                let (px00, px10, px01, px11) = unsafe {(
                    *local_px.get_unchecked(local_idx)          as f32,
                    *local_px.get_unchecked(local_idx + 1)      as f32,
                    *local_px.get_unchecked(local_idx + BLOCK)  as f32,
                    *local_px.get_unchecked(local_idx + BLOCK + 1) as f32,
                )};
                residual[i] = w00 * px00 + w01 * px01 + w10 * px10 + w11 * px11;
                sum += residual[i];
            }
            num_valid_points = PATTERN52_SIZE;
        } else {
            // Slow path: check each point individually.
            for i in 0..PATTERN52_SIZE {
                let x = transformed_pattern[(0, i)];
                let y = transformed_pattern[(1, i)];
                if x >= 2.0 && y >= 2.0 && x < w_bound && y < h_bound {
                    let ix = x as u32;
                    let iy = y as u32;
                    let dx = x - ix as f32;
                    let dy = y - iy as f32;
                    let w11 = dx * dy;
                    let w00 = 1.0 - dx - dy + w11;
                    let w10 = dx - w11;
                    let w01 = dy - w11;
                    let idx00 = (iy * width + ix) as usize;
                    let (px00, px10, px01, px11) = unsafe {(
                        *raw_pixels.get_unchecked(idx00)                   as f32,
                        *raw_pixels.get_unchecked(idx00 + 1)               as f32,
                        *raw_pixels.get_unchecked(idx00 + width_usize)     as f32,
                        *raw_pixels.get_unchecked(idx00 + width_usize + 1) as f32,
                    )};
                    residual[i] = w00 * px00 + w01 * px01 + w10 * px10 + w11 * px11;
                    sum += residual[i];
                    num_valid_points += 1;
                } else {
                    residual[i] = -1.0;
                }
            }
        }

        // Early-exit: if too few current-frame points are valid, the second pass cannot
        // satisfy num_residuals > PATTERN52_SIZE/2 (since num_residuals ≤ num_valid_points).
        // Skips the entire normalization pass for badly-positioned patches.
        if num_valid_points <= PATTERN52_SIZE / 2 || sum < f32::EPSILON {
            return None;
        }

        // Precompute scale = N/sum once (1 division) instead of per-element (52 divisions).
        let scale = num_valid_points as f32 / sum;

        // Fast path: if ALL source points are valid AND all target points are valid (bbox_safe),
        // skip 104 per-element checks and the num_residuals counter entirely.
        if self.all_valid && bbox_safe {
            for i in 0..PATTERN52_SIZE {
                residual[i] = scale * residual[i] - self.data[i];
            }
            return Some(residual);
        }

        let mut num_residuals = 0;
        for i in 0..PATTERN52_SIZE {
            if residual[i] >= 0.0 && self.data[i] >= 0.0 {
                residual[i] = scale * residual[i] - self.data[i];
                num_residuals += 1;
            } else {
                residual[i] = 0.0;
            }
        }
        if num_residuals > PATTERN52_SIZE / 2 {
            Some(residual)
        } else {
            None
        }
    }
}
