use image::GrayImage;
use imageproc::corners::{corners_fast9, Corner};
use nalgebra as na;

/// Bilinear interpolation + Sobel gradient at sub-pixel position (x, y).
/// Reads a 4×4 neighborhood (16 pixels) and computes value + dx + dy in one pass.
///
/// # Safety
/// Caller must ensure `x >= 1`, `y >= 1`, `x < width - 2`, `y < height - 2`.
/// In debug mode, this is verified with assertions.
#[inline(always)]
/// Compute bilinear-interpolated intensity and image gradients at (x, y).
///
/// Returns `[value, grad_x, grad_y]`. Called 52 times per `Pattern52::new`
/// so `#[inline(always)]` is critical for keeping return values in registers
/// and enabling LLVM to CSE the bilinear weights across the call site.
pub fn image_grad(grayscale_image: &GrayImage, x: f32, y: f32) -> na::SVector<f32, 3> {
    // Truncation == floor for positive floats (saves a function call)
    let ix = x as u32;
    let iy = y as u32;

    // Memory safety: verify we have enough room for the 4×4 neighborhood
    debug_assert!(
        ix >= 1 && ix < grayscale_image.width() - 2,
        "image_grad: x={} out of bounds for width={}",
        ix,
        grayscale_image.width()
    );
    debug_assert!(
        iy >= 1 && iy < grayscale_image.height() - 2,
        "image_grad: y={} out of bounds for height={}",
        iy,
        grayscale_image.height()
    );
    let dx = x - ix as f32;
    let dy = y - iy as f32;
    // Derive all 4 bilinear weights from one multiplication.
    // w00=(1-dx)(1-dy), w10=dx(1-dy), w01=(1-dx)dy, w11=dx·dy
    // Identity: w00=1-dx-dy+w11, w10=dx-w11, w01=dy-w11.
    let w11 = dx * dy;
    let w00 = 1.0 - dx - dy + w11;
    let w10 = dx - w11;
    let w01 = dy - w11;

    let width = grayscale_image.width();
    let raw_pixels = grayscale_image.as_raw();

    // Read 12 pixels needed for center value + finite-difference gradients.
    let base = ((iy - 1) * width + ix - 1) as usize;
    let w = width as usize;
    // SAFETY: caller guarantees ix in [2, width-3] and iy in [2, height-3]
    // (via `bbox_safe` or `inbound(.., radius=2)` check before calling).
    // Therefore base = (iy-1)*width+(ix-1), iy-1 >= 1, ix-1 >= 1.
    // Maximum index: base + 3*w + 3 = (iy+2)*width+(ix+2)
    //   <= (height-1)*width+(width-1) = height*width-1 < raw_pixels.len().
    let (p10, p20, p01, p11, p21, p31, p02, p12, p22, p32, p13, p23) = unsafe {(
        *raw_pixels.get_unchecked(base + 1)         as f32,
        *raw_pixels.get_unchecked(base + 2)         as f32,
        *raw_pixels.get_unchecked(base + w)         as f32,
        *raw_pixels.get_unchecked(base + w + 1)     as f32,
        *raw_pixels.get_unchecked(base + w + 2)     as f32,
        *raw_pixels.get_unchecked(base + w + 3)     as f32,
        *raw_pixels.get_unchecked(base + 2 * w)     as f32,
        *raw_pixels.get_unchecked(base + 2 * w + 1) as f32,
        *raw_pixels.get_unchecked(base + 2 * w + 2) as f32,
        *raw_pixels.get_unchecked(base + 2 * w + 3) as f32,
        *raw_pixels.get_unchecked(base + 3 * w + 1) as f32,
        *raw_pixels.get_unchecked(base + 3 * w + 2) as f32,
    )};

    // Value at (x, y): bilinear interpolation of center 2×2 block.
    let val = w00 * p11 + w01 * p12 + w10 * p21 + w11 * p22;

    // Gradients via the difference form: interp(x±1,y) = Σ wᵢ·p_shifted, so
    // right-left = Σ wᵢ·(p_right_i − p_left_i).  Replaces 2×(4 muls+3 adds)
    // with 4 subs + 4 muls + 3 adds per gradient, saving 4 multiplications each.
    let grad_x = 0.5 * (w00 * (p21 - p01) + w01 * (p22 - p02)
                      + w10 * (p31 - p11) + w11 * (p32 - p12));
    let grad_y = 0.5 * (w00 * (p12 - p10) + w01 * (p13 - p11)
                      + w10 * (p22 - p20) + w11 * (p23 - p21));

    na::SVector::<f32, 3>::new(val, grad_x, grad_y)
}

#[inline(always)]
pub fn point_in_bound(keypoint: &Corner, height: u32, width: u32, radius: u32) -> bool {
    keypoint.x >= radius
        && keypoint.x <= width - radius
        && keypoint.y >= radius
        && keypoint.y <= height - radius
}

#[inline(always)]
pub fn inbound(image: &GrayImage, x: f32, y: f32, radius: u32) -> bool {
    let r = radius as f32;
    x >= r
        && y >= r
        && x < (image.width() - radius) as f32
        && y < (image.height() - radius) as f32
}

pub fn se2_exp_matrix(a: &na::SVector<f32, 3>) -> na::SMatrix<f32, 3, 3> {
    let theta = a[2];
    let (sin, cos) = theta.sin_cos();
    let (sin_theta_by_theta, one_minus_cos_theta_by_theta) = if theta.abs() < f32::EPSILON {
        let theta_sq = theta * theta;
        (1.0f32 - theta_sq / 6.0, 0.5 * theta - theta * theta_sq / 24.0)
    } else {
        (sin / theta, (1.0 - cos) / theta)
    };
    let mut se2_mat = na::SMatrix::<f32, 3, 3>::identity();
    se2_mat.m11 = cos;
    se2_mat.m12 = -sin;
    se2_mat.m21 = sin;
    se2_mat.m22 = cos;
    se2_mat.m13 = sin_theta_by_theta * a[0] - one_minus_cos_theta_by_theta * a[1];
    se2_mat.m23 = one_minus_cos_theta_by_theta * a[0] + sin_theta_by_theta * a[1];
    se2_mat
}

/// Apply SE2 increment `T ← T * Exp(inc)` in-place.
///
/// Avoids constructing an intermediate 3×3 SE2-exp matrix and a full 3×3×3×3
/// multiply (27 muls + 18 adds).  Since both `T` and `Exp(inc)` are SE2
/// matrices with last row [0 0 1], only the 2×3 upper block needs updating
/// (12 muls + 6 adds).  The sin_cos pair and translational part of the
/// exponential are folded in directly.
#[inline(always)]
pub fn apply_se2_update(transform: &mut na::Affine2<f32>, inc: &na::SVector<f32, 3>) {
    let theta = inc[2];
    // Fast path: for negligible rotation (|theta| < 1e-4 rad), skip sin_cos() entirely
    // and apply a pure translation update.  The position error on the outermost pattern
    // point (radius 3.5 px) is at most 1e-4 × 3.5 ≈ 0.00035 px — below interpolation
    // accuracy.  This path is hit for most late-stage LK iterations where convergence
    // has reduced the rotation increment well below the translation threshold.
    if theta.abs() < 1e-4 {
        let tm = transform.matrix_mut_unchecked();
        tm.m13 += inc[0];
        tm.m23 += inc[1];
        return;
    }
    // theta >= 1e-4 here (smaller values returned early above), so the Taylor-series
    // branch (theta < EPSILON ≈ 1.2e-7) is unreachable — use direct sin/cos division.
    let (sin, cos) = theta.sin_cos();
    let stbt = sin / theta;
    let omcbt = (1.0 - cos) / theta;
    // Translational part of Exp(inc)
    let ex = stbt * inc[0] - omcbt * inc[1];
    let ey = omcbt * inc[0] + stbt * inc[1];
    // T * Exp: upper-2×3 block only (last row [0,0,1] is invariant)
    // [a  b  p]   [cos  -sin  ex]   [a·cos+b·sin  -a·sin+b·cos  a·ex+b·ey+p]
    // [c  d  q] × [sin   cos  ey] = [c·cos+d·sin  -c·sin+d·cos  c·ex+d·ey+q]
    let m = transform.matrix();
    let (a, b, p) = (m.m11, m.m12, m.m13);
    let (c, d, q) = (m.m21, m.m22, m.m23);
    let tm = transform.matrix_mut_unchecked();
    tm.m11 =  a * cos + b * sin;
    tm.m12 = -a * sin + b * cos;
    tm.m13 =  a * ex  + b * ey + p;
    tm.m21 =  c * cos + d * sin;
    tm.m22 = -c * sin + d * cos;
    tm.m23 =  c * ex  + d * ey + q;
    // m31, m32, m33 stay [0, 0, 1]
}

/// `grid_scratch` is a reusable backing buffer for the per-cell grid.
/// On the first call it is sized to `num_cells`; subsequent calls reuse the allocation
/// (clear + resize without malloc), saving ~18 KB of heap traffic per frame.
pub fn detect_key_points(
    image: &GrayImage,
    grid_size: u32,
    current_corners: &[Corner],
    grid_scratch: &mut Vec<Corner>,
) -> Vec<Corner> {
    const EDGE_THRESHOLD: u32 = 19;
    let h = image.height();
    let w = image.width();

    let x_start = (w % grid_size) / 2;
    let x_stop = x_start + grid_size * (w / grid_size - 1) + 1;
    let y_start = (h % grid_size) / 2;
    let y_stop = y_start + grid_size * (h / grid_size - 1) + 1;

    // Flat row-major per-cell array: score encodes three states —
    //   NEG_INFINITY = empty (no existing feature, no FAST corner yet)
    //   INFINITY     = occupied by an existing tracked feature (FAST corners cannot win)
    //   finite > NEG_INFINITY = best new FAST score seen for this cell
    // This fuses the previous separate occupied: Vec<bool> into all_corners.score,
    // saving one heap allocation per detection call.
    let grid_cols = (w / grid_size) as usize + 1;
    let grid_rows = (h / grid_size) as usize + 1;
    let num_cells = grid_rows * grid_cols;
    // Reuse grid_scratch: clear + resize reuses the backing allocation after the first call
    // (~18 KB of malloc/free per frame avoided).
    grid_scratch.clear();
    grid_scratch.resize(num_cells, Corner::new(0, 0, f32::NEG_INFINITY));
    let all_corners = &mut *grid_scratch;

    // Same reciprocal used for both current_corners and fast_corners cell lookups.
    let grid_size_inv = 1.0_f32 / grid_size as f32;

    for corner in current_corners {
        if corner.x >= x_start
            && corner.y >= y_start
            && corner.x < x_stop + grid_size
            && corner.y < y_stop + grid_size
        {
            let gx = ((corner.x - x_start) as f32 * grid_size_inv) as usize;
            let gy = ((corner.y - y_start) as f32 * grid_size_inv) as usize;
            // Mark occupied: INFINITY > any finite FAST score, so new corners won't overwrite.
            all_corners[gy * grid_cols + gx].score = f32::INFINITY;
        }
    }

    // **Single full-image FAST pass** — replaces the previous approach of copying a sub-image
    // per cell and running FAST on each (N_empty_cells × to_image() + N × 7 FAST calls).
    // One O(W×H) FAST pass with no sub-image allocation is cheaper overall: the parallel
    // per-cell approach had per-cell GrayImage allocation overhead + Vec<Corner> per call.
    // The per-cell max_by(score) below preserves the strongest corner per empty cell.
    let fast_corners = corners_fast9(image, 20);

    // Merge the two independent filter conditions (edge threshold + grid bounds) into a single
    // 4-comparison filter, halving the number of comparisons per FAST corner from 8 to 4.
    let cx_lo = EDGE_THRESHOLD.max(x_start);
    let cx_hi = (w - EDGE_THRESHOLD).min(x_stop + grid_size - 1);
    let cy_lo = EDGE_THRESHOLD.max(y_start);
    let cy_hi = (h - EDGE_THRESHOLD).min(y_stop + grid_size - 1);

    for corner in fast_corners {
        if corner.x < cx_lo || corner.x > cx_hi || corner.y < cy_lo || corner.y > cy_hi {
            continue;
        }
        let gx = ((corner.x - x_start) as f32 * grid_size_inv) as usize;
        let gy = ((corner.y - y_start) as f32 * grid_size_inv) as usize;
        let cell_idx = gy * grid_cols + gx;
        // Occupied cells (score=INFINITY): FAST scores are finite, so this guard is implicit.
        // Empty cells (score=NEG_INFINITY): any FAST corner wins.
        if corner.score > all_corners[cell_idx].score {
            all_corners[cell_idx] = corner;
        }
    }

    // Collect cells with a new FAST corner: score is finite and > NEG_INFINITY.
    // Occupied cells (score=INFINITY) and empty cells (score=NEG_INFINITY) are dropped.
    // grid_scratch retains its backing buffer (capacity = num_cells); next call reuses it.
    grid_scratch.iter()
        .filter(|c| c.score > f32::NEG_INFINITY && c.score.is_finite())
        .copied()
        .collect()
}
