use image::{GenericImage, GenericImageView, GrayImage, Luma};
use imageproc::corners::{corners_fast9, Corner};
use nalgebra as na;

pub fn image_grad(grayscale_image: &GrayImage, x: f32, y: f32) -> na::SVector<f32, 3> {
    // inbound
    let ix = x.floor() as u32;
    let iy = y.floor() as u32;

    let dx = x - ix as f32;
    let dy = y - iy as f32;

    let ddx = 1.0 - dx;
    let ddy = 1.0 - dy;

    // Use direct pixel access instead of get_pixel for better performance
    let width = grayscale_image.width();
    let raw_pixels = grayscale_image.as_raw();

    let idx00 = (iy * width + ix) as usize;
    let idx10 = (iy * width + ix + 1) as usize;
    let idx01 = ((iy + 1) * width + ix) as usize;
    let idx11 = ((iy + 1) * width + ix + 1) as usize;

    let px0y0 = raw_pixels[idx00] as f32;
    let px1y0 = raw_pixels[idx10] as f32;
    let px0y1 = raw_pixels[idx01] as f32;
    let px1y1 = raw_pixels[idx11] as f32;

    let res0 = ddx * ddy * px0y0 + ddx * dy * px0y1 + dx * ddy * px1y0 + dx * dy * px1y1;

    // Direct pixel access for gradient computation
    let idxm1y0 = (iy * width + ix - 1) as usize;
    let idxm1y1 = ((iy + 1) * width + ix - 1) as usize;
    let pxm1y0 = raw_pixels[idxm1y0] as f32;
    let pxm1y1 = raw_pixels[idxm1y1] as f32;

    let res_mx = ddx * ddy * pxm1y0 + ddx * dy * pxm1y1 + dx * ddy * px0y0 + dx * dy * px0y1;

    let idx2y0 = (iy * width + ix + 2) as usize;
    let idx2y1 = ((iy + 1) * width + ix + 2) as usize;
    let px2y0 = raw_pixels[idx2y0] as f32;
    let px2y1 = raw_pixels[idx2y1] as f32;

    let res_px = ddx * ddy * px1y0 + ddx * dy * px1y1 + dx * ddy * px2y0 + dx * dy * px2y1;

    let res1 = 0.5 * (res_px - res_mx);

    let idx0ym1 = ((iy - 1) * width + ix) as usize;
    let idx1ym1 = ((iy - 1) * width + ix + 1) as usize;
    let px0ym1 = raw_pixels[idx0ym1] as f32;
    let px1ym1 = raw_pixels[idx1ym1] as f32;

    let res_my = ddx * ddy * px0ym1 + ddx * dy * px0y0 + dx * ddy * px1ym1 + dx * dy * px1y0;

    let idx0y2 = ((iy + 2) * width + ix) as usize;
    let idx1y2 = ((iy + 2) * width + ix + 1) as usize;
    let px0y2 = raw_pixels[idx0y2] as f32;
    let px1y2 = raw_pixels[idx1y2] as f32;

    let res_py = ddx * ddy * px0y1 + ddx * dy * px0y2 + dx * ddy * px1y1 + dx * dy * px1y2;

    let res2 = 0.5 * (res_py - res_my);

    na::SVector::<f32, 3>::new(res0, res1, res2)
}

pub fn point_in_bound(keypoint: &Corner, height: u32, width: u32, radius: u32) -> bool {
    keypoint.x >= radius
        && keypoint.x <= width - radius
        && keypoint.y >= radius
        && keypoint.y <= height - radius
}

pub fn inbound(image: &GrayImage, x: f32, y: f32, radius: u32) -> bool {
    let x = x.round() as u32;
    let y = y.round() as u32;

    x >= radius && y >= radius && x < image.width() - radius && y < image.height() - radius
}

pub fn se2_exp_matrix(a: &na::SVector<f32, 3>) -> na::SMatrix<f32, 3, 3> {
    let theta = a[2];
    let mut so2 = na::Rotation2::new(theta);
    let sin_theta_by_theta;
    let one_minus_cos_theta_by_theta;

    if theta.abs() < f32::EPSILON {
        let theta_sq = theta * theta;
        sin_theta_by_theta = 1.0f32 - 1.0 / 6.0 * theta_sq;
        one_minus_cos_theta_by_theta = 0.5f32 * theta - 1. / 24. * theta * theta_sq;
    } else {
        let cos = so2.matrix_mut_unchecked().m22;
        let sin = so2.matrix_mut_unchecked().m21;
        sin_theta_by_theta = sin / theta;
        one_minus_cos_theta_by_theta = (1. - cos) / theta;
    }
    let mut se2_mat = na::SMatrix::<f32, 3, 3>::identity();
    se2_mat.m11 = so2.matrix_mut_unchecked().m11;
    se2_mat.m12 = so2.matrix_mut_unchecked().m12;
    se2_mat.m21 = so2.matrix_mut_unchecked().m21;
    se2_mat.m22 = so2.matrix_mut_unchecked().m22;
    se2_mat.m13 = sin_theta_by_theta * a[0] - one_minus_cos_theta_by_theta * a[1];
    se2_mat.m23 = one_minus_cos_theta_by_theta * a[0] + sin_theta_by_theta * a[1];
    se2_mat
}

/// Downsample an image by 2x using a 2x2 box filter (average).
/// Dst dimensions must be exactly half of src dimensions.
pub fn downsample_half_box(src: &GrayImage, dst: &mut GrayImage) {
    let (sw, sh) = src.dimensions();
    let (dw, dh) = dst.dimensions();
    debug_assert_eq!(dw, sw / 2);
    debug_assert_eq!(dh, sh / 2);
    let sdata = src.as_raw();
    let ddata = dst.as_mut();
    for y in 0..dh {
        let sy = y * 2;
        for x in 0..dw {
            let sx = x * 2;
            let idx0 = (sy * sw + sx) as usize;
            let idx1 = (sy * sw + sx + 1) as usize;
            let idx2 = ((sy + 1) * sw + sx) as usize;
            let idx3 = ((sy + 1) * sw + sx + 1) as usize;
            let sum = sdata[idx0] as u32
                + sdata[idx1] as u32
                + sdata[idx2] as u32
                + sdata[idx3] as u32;
            ddata[(y * dw + x) as usize] = (sum / 4) as u8;
        }
    }
}

/// Fill a preallocated pyramid in-place with downsampled levels.
/// Level 0 is copied from src; subsequent levels are 2x downsampled.
pub fn fill_pyramid(pyr: &mut [GrayImage], src: &GrayImage) {
    // Level 0: copy
    let (w0, h0) = src.dimensions();
    debug_assert_eq!(pyr[0].dimensions(), (w0, h0));
    pyr[0].copy_from(src, 0, 0).ok();
    // Subsequent levels: 2x2 box downsample
    for level in 1..pyr.len() {
        let (left, right) = pyr.split_at_mut(level);
        let prev = &left[level - 1];
        let dst = &mut right[0];
        let (pw, ph) = prev.dimensions();
        let (dw, dh) = dst.dimensions();
        debug_assert_eq!(dw, pw / 2);
        debug_assert_eq!(dh, ph / 2);
        downsample_half_box(prev, dst);
    }
}

/// Ensure a pyramid Vec is allocated with the correct number of levels and dimensions.
/// If already allocated, does nothing. Returns true if allocation was needed.
pub fn ensure_pyramid_allocated(pyr: &mut Vec<GrayImage>, w: u32, h: u32, levels: usize) -> bool {
    if pyr.len() == levels {
        // Already allocated, verify dimensions match
        let (pw, ph) = pyr[0].dimensions();
        if pw == w && ph == h {
            return false;
        }
    }
    // Allocate or reallocate
    pyr.clear();
    let mut cw = w;
    let mut ch = h;
    for _ in 0..levels {
        pyr.push(GrayImage::from_pixel(cw, ch, Luma([0u8])));
        cw /= 2;
        ch /= 2;
    }
    true
}

/// Refine corner position to sub-pixel accuracy using quadratic interpolation
/// of the corner response function around the detected corner location.
pub fn refine_corner_subpixel(image: &GrayImage, corner: &Corner, window_size: u32) -> (f32, f32) {
    let x = corner.x as f32;
    let y = corner.y as f32;
    let half_window = window_size as f32 / 2.0;

    // Check bounds
    if x - half_window < 0.0
        || y - half_window < 0.0
        || x + half_window >= image.width() as f32
        || y + half_window >= image.height() as f32
    {
        return (x, y);
    }

    // Simple centroid-based sub-pixel refinement
    // Compute intensity-weighted center of mass in a small window
    let window_size = 3; // 3x3 window for refinement
    let mut sum_intensity = 0.0;
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;

    for dy in -(window_size / 2)..=(window_size / 2) {
        for dx in -(window_size / 2)..=(window_size / 2) {
            let px = (x as i32 + dx) as u32;
            let py = (y as i32 + dy) as u32;

            if px < image.width() && py < image.height() {
                let intensity = image.get_pixel(px, py)[0] as f32;
                sum_intensity += intensity;
                sum_x += intensity * (x + dx as f32);
                sum_y += intensity * (y + dy as f32);
            }
        }
    }

    if sum_intensity > 0.0 {
        let refined_x = sum_x / sum_intensity;
        let refined_y = sum_y / sum_intensity;

        // Limit refinement to 0.5 pixel from original location
        let clamped_x = x + (refined_x - x).max(-0.5).min(0.5);
        let clamped_y = y + (refined_y - y).max(-0.5).min(0.5);

        (clamped_x, clamped_y)
    } else {
        (x, y)
    }
}

pub fn detect_key_points(
    image: &GrayImage,
    grid_size: u32,
    current_corners: &Vec<Corner>,
    num_points_in_cell: u32,
) -> Vec<(f32, f32, f32)> {
    const EDGE_THRESHOLD: u32 = 19;
    let h = image.height();
    let w = image.width();
    let mut all_corners = vec![];
    let mut grids =
        na::DMatrix::<i32>::zeros((h / grid_size + 1) as usize, (w / grid_size + 1) as usize);

    let x_start = (w % grid_size) / 2;
    let x_stop = x_start + grid_size * (w / grid_size - 1) + 1;

    let y_start = (h % grid_size) / 2;
    let y_stop = y_start + grid_size * (h / grid_size - 1) + 1;

    // add existing corners to grid
    for corner in current_corners {
        if corner.x >= x_start
            && corner.y >= y_start
            && corner.x < x_stop + grid_size
            && corner.y < y_stop + grid_size
        {
            let x = (corner.x - x_start) / grid_size;
            let y = (corner.y - y_start) / grid_size;

            grids[(y as usize, x as usize)] += 1;
        }
    }

    for x in (x_start..x_stop).step_by(grid_size as usize) {
        for y in (y_start..y_stop).step_by(grid_size as usize) {
            if grids[(
                ((y - y_start) / grid_size) as usize,
                ((x - x_start) / grid_size) as usize,
            )] > 0
            {
                continue;
            }

            let image_view = image.view(x, y, grid_size, grid_size).to_image();
            let mut points_added = 0;
            let mut threshold: u8 = 40;

            while points_added < num_points_in_cell && threshold >= 10 {
                let mut fast_corners = corners_fast9(&image_view, threshold);
                fast_corners.sort_by(|a, b| {
                    a.score
                        .total_cmp(&b.score)
                });

                for mut point in fast_corners {
                    if points_added >= num_points_in_cell {
                        break;
                    }
                    point.x += x;
                    point.y += y;
                    if point_in_bound(&point, h, w, EDGE_THRESHOLD) {
                        all_corners.push(point);
                        points_added += 1;
                    }
                }
                threshold -= 5;
            }
        }
    }

    // Apply sub-pixel refinement to detected corners
    let mut refined_corners = Vec::new();
    for corner in all_corners {
        let (refined_x, refined_y) = refine_corner_subpixel(image, &corner, 5);
        refined_corners.push((refined_x, refined_y, corner.score));
    }

    refined_corners
}

#[cfg(test)]
#[allow(clippy::all)]
mod tests {
    use super::*;
    use image::Luma;

    #[test]
    fn image_grad_flat_image_has_zero_gradient() {
        let img = GrayImage::from_pixel(6, 6, Luma([128u8]));

        let g = image_grad(&img, 2.5, 2.5);
        assert!((g[0] - 128.0).abs() < 1e-6);
        assert!(g[1].abs() < 1e-6);
        assert!(g[2].abs() < 1e-6);
    }

    #[test]
    fn inbound_checks_bounds() {
        let img = GrayImage::from_pixel(10, 10, Luma([0u8]));
        assert!(inbound(&img, 5.0, 5.0, 1));
        assert!(!inbound(&img, 0.0, 0.0, 1));
        assert!(!inbound(&img, 9.0, 9.0, 1));
    }

    #[test]
    fn se2_exp_matrix_zero_theta_translates() {
        let a = na::SVector::<f32, 3>::new(1.0, -2.0, 0.0);
        let mat = se2_exp_matrix(&a);
        assert!((mat.m11 - 1.0).abs() < 1e-6);
        assert!((mat.m22 - 1.0).abs() < 1e-6);
        assert!((mat.m13 - 1.0).abs() < 1e-6);
        assert!((mat.m23 + 2.0).abs() < 1e-6);
    }

    #[test]
    fn detect_key_points_finds_corner() {
        let mut img = GrayImage::from_pixel(64, 64, Luma([0u8]));
        // create a larger bright block to ensure FAST-9 finds a corner
        for x in 20..44 {
            for y in 20..44 {
                img.put_pixel(x, y, Luma([255u8]));
            }
        }

        let points = detect_key_points(&img, 8, &Vec::new(), 2);
        assert!(!points.is_empty());
        assert!(points
            .iter()
            .any(|p| p.0 >= 10.0 && p.0 <= 22.0 && p.1 >= 10.0 && p.1 <= 22.0));
    }

    #[test]
    fn inbound_exact_boundaries() {
        let img = GrayImage::from_pixel(10, 10, Luma([0u8]));
        // Just inside bounds
        assert!(inbound(&img, 1.0, 1.0, 1));
        // 0.4 rounds to 0, which fails the check (0 >= 1 is false)
        assert!(!inbound(&img, 0.4, 5.0, 1));
    }

    #[test]
    fn image_grad_steep_gradient() {
        let mut img = GrayImage::from_pixel(6, 6, Luma([0u8]));
        // Create vertical edge: left half dark, right half bright
        for x in 3..6 {
            for y in 0..6 {
                img.put_pixel(x, y, Luma([255u8]));
            }
        }
        let g = image_grad(&img, 2.5, 2.5);
        // Should have significant x-gradient
        assert!(g[1].abs() > 50.0);
    }

    #[test]
    fn image_grad_near_boundary() {
        let img = GrayImage::from_pixel(6, 6, Luma([128u8]));
        // Near edge - should still compute gradient
        let g = image_grad(&img, 1.0, 1.0);
        assert!((g[0] - 128.0).abs() < 1.0);
    }

    #[test]
    fn se2_exp_matrix_large_rotation() {
        let a = na::SVector::<f32, 3>::new(0.0, 0.0, std::f32::consts::PI / 2.0);
        let mat = se2_exp_matrix(&a);
        // Rotation by PI/2: cos(PI/2)=0, sin(PI/2)=1
        assert!(mat.m11.abs() < 0.1);
        assert!((mat.m12 + 1.0).abs() < 0.1); // Should be -1
        assert!((mat.m21 - 1.0).abs() < 0.1);
    }

    #[test]
    fn se2_exp_matrix_negative_rotation() {
        let a = na::SVector::<f32, 3>::new(0.0, 0.0, -std::f32::consts::PI / 4.0);
        let mat = se2_exp_matrix(&a);
        // Check determinant is 1 (rotation preserves orientation)
        let det = mat.m11 * mat.m22 - mat.m12 * mat.m21;
        assert!((det - 1.0).abs() < 1e-5);
    }

    #[test]
    fn se2_exp_matrix_translation_only() {
        let a = na::SVector::<f32, 3>::new(5.0, -3.0, 0.0);
        let mat = se2_exp_matrix(&a);
        assert!((mat.m13 - 5.0).abs() < 1e-5);
        assert!((mat.m23 + 3.0).abs() < 1e-5);
    }

    #[test]
    fn se2_exp_matrix_small_angle() {
        let a = na::SVector::<f32, 3>::new(1.0, 2.0, 1e-7);
        let mat = se2_exp_matrix(&a);
        // Small angle approximation: sin(θ) ≈ θ, cos(θ) ≈ 1
        assert!((mat.m11 - 1.0).abs() < 1e-6);
        assert!(mat.m12.abs() < 1e-6);
    }

    #[test]
    fn inbound_negative_coordinates() {
        let img = GrayImage::from_pixel(10, 10, Luma([0u8]));
        // Negative coordinates: -1.0 rounds to 0 (as u32)
        // With radius=0, x=0 passes (0 >= 0 && 0 < 10), so need radius > 0
        assert!(!inbound(&img, -1.0, 5.0, 1));
        assert!(!inbound(&img, 5.0, -1.0, 1));
    }

    #[test]
    fn inbound_large_coordinates() {
        let img = GrayImage::from_pixel(10, 10, Luma([0u8]));
        // Coordinates beyond image size
        assert!(!inbound(&img, 10.0, 5.0, 0));
        assert!(!inbound(&img, 5.0, 10.0, 0));
    }

    #[test]
    fn detect_key_points_no_features() {
        let img = GrayImage::from_pixel(64, 64, Luma([128u8]));
        let points = detect_key_points(&img, 8, &Vec::new(), 2);
        // Flat image should have no features
        assert!(points.is_empty());
    }

    #[test]
    fn detect_key_points_small_image() {
        let img = GrayImage::from_pixel(8, 8, Luma([0u8]));
        let points = detect_key_points(&img, 4, &Vec::new(), 2);
        // Very small image might have no features
        assert!(points.len() <= 1);
    }
}
