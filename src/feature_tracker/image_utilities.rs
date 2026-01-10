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
                        a.score.partial_cmp(&b.score).unwrap_or(std::cmp::Ordering::Equal)
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
