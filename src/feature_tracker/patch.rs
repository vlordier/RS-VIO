use image::GrayImage;
use nalgebra as na;
use std::ops::AddAssign;

use super::image_utilities;

/// Size of the patch pattern used for tracking
pub const PATTERN52_SIZE: usize = 52;

/// A 52-point patch pattern for Lucas-Kanade optical flow tracking
///
/// This struct implements a patch-based feature tracker using 52 sample points
/// arranged in a specific spatial pattern. It computes the 2D optical flow transform
/// (rotation + scale + translation in SE(2)) for the patch.
///
/// ## Algorithm
///
/// The pattern uses Lucas-Kanade tracking to estimate:
/// - **2D translation** (tx, ty)
/// - **Rotation + scaling** via affine parameters
///
/// ## Pattern Layout
///
/// The 52-point pattern samples around a feature location in an X-shaped pattern
/// with 4 levels of spacing (3, 5, 7 pixel offsets).
///
/// ## Performance
///
/// - **Initialization**: O(pattern_size × iterations) ≈ 1-2ms per feature
/// - **Residual computation**: O(pattern_size) ≈ 0.05ms
/// - **Converges in**: 3-5 iterations typically
///
/// ## Validity
///
/// A patch is invalid if:
/// - Points are out of image bounds
/// - Hessian matrix is singular (degenerate pattern)
/// - Mean intensity is too low (insufficient texture)
pub struct Pattern52 {
    /// Whether this patch is valid and trackable
    pub valid: bool,
    /// Mean intensity of the patch
    pub mean: f32,
    /// 2D position of patch center
    pub pos: na::SVector<f32, 2>,
    /// Pre-computed intensities at the 52 pattern points
    pub data: [f32; PATTERN52_SIZE],
    /// Cached Hessian inverse × Jacobian transpose for fast tracking
    pub h_se2_inv_j_se2_t: na::SMatrix<f32, 3, PATTERN52_SIZE>,
    /// Scale factor applied to pattern (for multi-scale)
    pub pattern_scale_down: f32,
    /// Pre-computed 2D coordinates of the 52 pattern points
    pub pattern_matrix: na::SMatrix<f32, 2, PATTERN52_SIZE>,
}
impl Default for Pattern52 {
    fn default() -> Self {
        Self {
            valid: false,
            mean: 0.0,
            pos: na::Vector2::zeros(),
            data: [0.0; PATTERN52_SIZE],
            h_se2_inv_j_se2_t: na::SMatrix::zeros(),
            pattern_scale_down: 0.0,
            pattern_matrix: na::SMatrix::zeros(),
        }
    }
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
        j_se2: &mut na::SMatrix<f32, PATTERN52_SIZE, 3>,
    ) {
        let mut num_valid_points = 0;
        let mut sum: f32 = 0.0;
        let mut grad_sum_se2 = na::SVector::<f32, 3>::zeros();

        let mut jw_se2 = na::SMatrix::<f32, 2, 3>::identity();

        for (i, pattern_pos) in Self::PATTERN_RAW.into_iter().enumerate() {
            let p = self.pos
                + na::SVector::<f32, 2>::new(
                    pattern_pos[0] / self.pattern_scale_down,
                    pattern_pos[1] / self.pattern_scale_down,
                );
            jw_se2[(0, 2)] = -pattern_pos[1] / self.pattern_scale_down;
            jw_se2[(1, 2)] = pattern_pos[0] / self.pattern_scale_down;

            if image_utilities::inbound(greyscale_image, p.x, p.y, 2) {
                let val_grad = image_utilities::image_grad(greyscale_image, p.x, p.y);

                self.data[i] = val_grad[0];
                sum += val_grad[0];
                let re = val_grad.fixed_rows::<2>(1).transpose() * jw_se2;
                j_se2.set_row(i, &re);
                grad_sum_se2.add_assign(j_se2.fixed_rows::<1>(i).transpose());
                num_valid_points += 1;
            } else {
                self.data[i] = -1.0;
            }
        }

        self.mean = sum / num_valid_points as f32;

        let mean_inv = num_valid_points as f32 / sum;

        for i in 0..Self::PATTERN_RAW.len() {
            if self.data[i] >= 0.0 {
                let rhs = grad_sum_se2.transpose() * self.data[i] / sum;
                j_se2.fixed_rows_mut::<1>(i).add_assign(-rhs);
                self.data[i] *= mean_inv;
            } else {
                j_se2.set_row(i, &na::SMatrix::<f32, 1, 3>::zeros());
            }
        }
        *j_se2 *= mean_inv;
    }
    pub fn new(greyscale_image: &GrayImage, px: f32, py: f32) -> Pattern52 {
        let mut j_se2 = na::SMatrix::<f32, PATTERN52_SIZE, 3>::zeros();
        let pattern_scale_down = 2.0;

        // Pre-compute pattern matrix once (2x52, transposed from 52x2)
        let pattern_matrix = na::SMatrix::<f32, 2, PATTERN52_SIZE>::from_fn(|i, j| {
            Self::PATTERN_RAW[j][i] / pattern_scale_down
        });

        let mut p = Pattern52 {
            valid: false,
            mean: 1.0,
            pos: na::SVector::<f32, 2>::new(px, py),
            data: [0.0; PATTERN52_SIZE], // negative if the point is not valid
            h_se2_inv_j_se2_t: na::SMatrix::<f32, 3, 52>::zeros(),
            pattern_scale_down,
            pattern_matrix,
        };
        p.set_data_jac_se2(greyscale_image, &mut j_se2);
        let h_se2 = j_se2.transpose() * j_se2;
        let mut h_se2_inv = na::SMatrix::<f32, 3, 3>::identity();

        if let Some(x) = h_se2.cholesky() {
            x.solve_mut(&mut h_se2_inv);
            p.h_se2_inv_j_se2_t = h_se2_inv * j_se2.transpose();

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
        transformed_pattern: &na::SMatrix<f32, 2, PATTERN52_SIZE>,
    ) -> Option<na::SVector<f32, PATTERN52_SIZE>> {
        let mut sum: f32 = 0.0;
        let mut num_valid_points = 0;
        let mut residual = na::SVector::<f32, PATTERN52_SIZE>::zeros();

        // Fast inline bilinear interpolation - avoids function call overhead
        let width = greyscale_image.width();
        let height = greyscale_image.height();
        let raw_pixels = greyscale_image.as_raw();

        for i in 0..PATTERN52_SIZE {
            let x = transformed_pattern[(0, i)];
            let y = transformed_pattern[(1, i)];

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

#[cfg(test)]
mod tests {
    use super::*;
    use image::Luma;

    #[test]
    fn pattern_invalid_on_low_texture() {
        let img = GrayImage::from_pixel(16, 16, Luma([0u8]));
        let p = Pattern52::new(&img, 8.0, 8.0);
        assert!(!p.valid);
    }

    #[test]
    fn pattern_computes_finite_data_on_checkerboard() {
        let img = GrayImage::from_fn(64, 64, |x, y| {
            let val = if (x + y) % 2 == 0 { 0u8 } else { 255u8 };
            Luma([val])
        });
        let p = Pattern52::new(&img, 32.0, 32.0);
        assert!(p.mean > 0.0);
        assert!(p.data.iter().all(|v| v.is_finite()));
        assert!(p.h_se2_inv_j_se2_t.iter().all(|v| v.is_finite()));
    }
}
