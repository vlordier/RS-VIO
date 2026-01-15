use image::{GenericImageView, GrayImage};
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

pub fn detect_key_points(
    image: &GrayImage,
    grid_size: u32,
    current_corners: &Vec<Corner>,
    num_points_in_cell: u32,
) -> Vec<Corner> {
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
                        .partial_cmp(&b.score)
                        .unwrap_or(std::cmp::Ordering::Equal)
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
    all_corners
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
            .any(|p| p.x >= 10 && p.x <= 22 && p.y >= 10 && p.y <= 22));
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
