//! Fixed-size image patch for template-based optical flow tracking.

use image::GrayImage;
use nalgebra as na;
use std::ops::AddAssign;
use std::sync::LazyLock;

use super::image_utilities;

/// Pre-computed pattern matrix (2×52) — avoids recomputing on every Pattern52::new() call.
/// With ~200 tracked points × 3 pyramid levels = 600 constructions per frame, this
/// eliminates 600 × 104-element divisions per frame.
static PATTERN52_MATRIX: LazyLock<na::SMatrix<f32, 2, PATTERN52_SIZE>> = LazyLock::new(|| {
    na::SMatrix::<f32, 2, PATTERN52_SIZE>::from_fn(|i, j| Pattern52::PATTERN_RAW[j][i] / 2.0)
});

pub const PATTERN52_SIZE: usize = 52;
pub struct Pattern52 {
    pub valid: bool,
    pub mean: f32,
    pub pos: na::SVector<f32, 2>,
    pub data: [f32; PATTERN52_SIZE], // negative if the point is not valid
    pub h_se2_inv_j_se2_t: na::SMatrix<f32, 3, PATTERN52_SIZE>,
    pub pattern_scale_down: f32,
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

    // verified
    pub fn set_data_jac_se2(
        &mut self,
        greyscale_image: &GrayImage,
        j_se2: &mut na::SMatrix<f32, 3, PATTERN52_SIZE>,
    ) {
        let mut num_valid_points = 0;
        let mut sum: f32 = 0.0;
        let mut grad_sum_se2 = na::SVector::<f32, 3>::zeros();

        let mut jw_se2 = na::SMatrix::<f32, 2, 3>::identity();

        for i in 0..PATTERN52_SIZE {
            let px = PATTERN52_MATRIX[(0, i)];
            let py = PATTERN52_MATRIX[(1, i)];
            let p = self.pos + na::SVector::<f32, 2>::new(px, py);
            jw_se2[(0, 2)] = -py;
            jw_se2[(1, 2)] = px;

            if image_utilities::inbound(greyscale_image, p.x, p.y, 2) {
                let val_grad = image_utilities::image_grad(greyscale_image, p.x, p.y);

                self.data[i] = val_grad[0];
                sum += val_grad[0];
                let re = jw_se2.transpose() * val_grad.fixed_rows::<2>(1);
                j_se2.set_column(i, &re);
                grad_sum_se2.add_assign(re);
                num_valid_points += 1;
            } else {
                self.data[i] = -1.0;
            }
        }

        if num_valid_points == 0 {
            self.mean = 0.0;
            return;
        }
        self.mean = sum / num_valid_points as f32;

        let mean_inv = num_valid_points as f32 / sum;

        for i in 0..Self::PATTERN_RAW.len() {
            if self.data[i] >= 0.0 {
                let rhs = grad_sum_se2 * (self.data[i] / sum);
                j_se2.column_mut(i).add_assign(-rhs);
                self.data[i] *= mean_inv;
            } else {
                j_se2.column_mut(i).fill(0.0);
            }
        }
        *j_se2 *= mean_inv;
    }
    pub fn new(greyscale_image: &GrayImage, px: f32, py: f32) -> Pattern52 {
        let mut j_se2 = na::SMatrix::<f32, 3, PATTERN52_SIZE>::zeros();
        let pattern_scale_down = 2.0;

        let mut p = Pattern52 {
            valid: false,
            mean: 1.0,
            pos: na::SVector::<f32, 2>::new(px, py),
            data: [0.0; PATTERN52_SIZE], // negative if the point is not valid
            h_se2_inv_j_se2_t: na::SMatrix::<f32, 3, 52>::zeros(),
            pattern_scale_down,
        };
        p.set_data_jac_se2(greyscale_image, &mut j_se2);
        let h_se2 = j_se2 * j_se2.transpose();
        let mut h_se2_inv = na::SMatrix::<f32, 3, 3>::identity();

        if let Some(x) = h_se2.cholesky() {
            x.solve_mut(&mut h_se2_inv);
            p.h_se2_inv_j_se2_t = h_se2_inv * j_se2;

            // NOTE: while it's very unlikely we get a source patch with all black
            // pixels, since points are usually selected at corners, it doesn't cost
            // much to be safe here.

            // all-black patch cannot be normalized; will result in mean of "zero" and
            // H_se2_inv_J_se2_T will contain "NaN" and data will contain "inf"
            p.valid = p.mean > f32::EPSILON
                && p.h_se2_inv_j_se2_t.iter().all(|x| x.is_finite())
                && p.data.iter().all(|x| x.is_finite());
        }

        p
    }
    pub fn residual(
        &self,
        greyscale_image: &GrayImage,
        transform: &na::Affine2<f32>,
    ) -> Option<na::SVector<f32, PATTERN52_SIZE>> {
        let mut sum: f32 = 0.0;
        let mut num_valid_points = 0;
        let mut residual = na::SVector::<f32, PATTERN52_SIZE>::zeros();

        // Fast inline bilinear interpolation - avoids function call overhead
        let width = greyscale_image.width();
        let height = greyscale_image.height();
        let raw_pixels = greyscale_image.as_raw();

        // Extract transform components for fast manual multiplication
        let m = transform.matrix();
        let r11 = m.m11;
        let r12 = m.m12;
        let tx = m.m13;
        let r21 = m.m21;
        let r22 = m.m22;
        let ty = m.m23;

        for i in 0..PATTERN52_SIZE {
            let px_raw = PATTERN52_MATRIX[(0, i)];
            let py_raw = PATTERN52_MATRIX[(1, i)];

            let x = r11 * px_raw + r12 * py_raw + tx;
            let y = r21 * px_raw + r22 * py_raw + ty;

            // Fast bounds check
            if x >= 2.0 && y >= 2.0 && x < (width - 2) as f32 && y < (height - 2) as f32 {
                // Fast bilinear interpolation
                let ix = x.floor() as u32;
                let iy = y.floor() as u32;
                let dx = x - ix as f32;
                let dy = y - iy as f32;

                let ddx = 1.0 - dx;
                let ddy = 1.0 - dy;

                // Direct pixel access - much faster than get_pixel
                let idx00 = (iy * width + ix) as usize;
                let idx10 = (iy * width + ix + 1) as usize;
                let idx01 = ((iy + 1) * width + ix) as usize;
                let idx11 = ((iy + 1) * width + ix + 1) as usize;

                let px00 = raw_pixels[idx00] as f32;
                let px10 = raw_pixels[idx10] as f32;
                let px01 = raw_pixels[idx01] as f32;
                let px11 = raw_pixels[idx11] as f32;

                residual[i] = ddx * ddy * px00 + ddx * dy * px01 + dx * ddy * px10 + dx * dy * px11;
                sum += residual[i];
                num_valid_points += 1;
            } else {
                residual[i] = -1.0;
            }
        }

        // all-black patch cannot be normalized
        if sum < f32::EPSILON {
            return None;
        }

        let mut num_residuals = 0;

        for i in 0..PATTERN52_SIZE {
            if residual[i] >= 0.0 && self.data[i] >= 0.0 {
                let val = residual[i];
                residual[i] = num_valid_points as f32 * val / sum - self.data[i];
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
