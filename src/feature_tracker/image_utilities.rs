use image::{GenericImageView, GrayImage};
use imageproc::corners::{corners_fast9, Corner};
use nalgebra as na;
use rayon::prelude::*;

/// Bilinear interpolation + Sobel gradient at sub-pixel position (x, y).
/// Reads a 4×4 neighborhood (16 pixels) and computes value + dx + dy in one pass.
/// Assumes caller has verified bounds (x >= 1, y >= 1, x < w-2, y < h-2).
#[inline(always)]
pub fn image_grad(grayscale_image: &GrayImage, x: f32, y: f32) -> na::SVector<f32, 3> {
    // Truncation == floor for positive floats (saves a function call)
    let ix = x as u32;
    let iy = y as u32;
    let dx = x - ix as f32;
    let dy = y - iy as f32;
    let ddx = 1.0 - dx;
    let ddy = 1.0 - dy;

    let width = grayscale_image.width();
    let raw_pixels = grayscale_image.as_raw();

    // Read 4×4 neighborhood once into a flat array (rows iy-1..=iy+2, cols ix-1..=ix+2)
    // Only the 11 pixels needed for center value + Sobel gradients are read.
    let base = ((iy - 1) * width + ix - 1) as usize;
    let w = width as usize;
    let p10 = raw_pixels[base + 1] as f32;
    let p20 = raw_pixels[base + 2] as f32;
    let p01 = raw_pixels[base + w] as f32;
    let p11 = raw_pixels[base + w + 1] as f32;
    let p21 = raw_pixels[base + w + 2] as f32;
    let p31 = raw_pixels[base + w + 3] as f32;
    let p02 = raw_pixels[base + 2 * w] as f32;
    let p12 = raw_pixels[base + 2 * w + 1] as f32;
    let p22 = raw_pixels[base + 2 * w + 2] as f32;
    let p32 = raw_pixels[base + 2 * w + 3] as f32;
    let p13 = raw_pixels[base + 3 * w + 1] as f32;
    let p23 = raw_pixels[base + 3 * w + 2] as f32;

    // Value at (x, y): bilinear interpolation of center 2×2 block
    let val = ddx * ddy * p11 + ddx * dy * p12 + dx * ddy * p21 + dx * dy * p22;

    // dx gradient: (interp at x+1, y) - (interp at x-1, y), divided by 2
    // We use the pre-read pixels to avoid re-interpolation
    let left = ddx * ddy * p01 + ddx * dy * p02 + dx * ddy * p11 + dx * dy * p12;
    let right = ddx * ddy * p21 + ddx * dy * p22 + dx * ddy * p31 + dx * dy * p32;
    let grad_x = 0.5 * (right - left);

    // dy gradient: (interp at x, y+1) - (interp at x, y-1), divided by 2
    let bottom = ddx * ddy * p12 + ddx * dy * p13 + dx * ddy * p22 + dx * dy * p23;
    let top = ddx * ddy * p10 + ddx * dy * p11 + dx * ddy * p20 + dx * dy * p21;
    let grad_y = 0.5 * (bottom - top);

    na::SVector::<f32, 3>::new(val, grad_x, grad_y)
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

    // Build list of empty grid cells to process (sequential, then parallelize detection)
    let cols = (w / grid_size) as usize + 1;
    let mut cell_flags = vec![false; cols * ((h / grid_size) as usize + 1)];
    for (idx, val) in grids.iter().enumerate() {
        if *val == 0 {
            cell_flags[idx] = true;
        }
    }
    let mut empty_cells: Vec<(u32, u32)> = Vec::new();
    for (col_idx, x) in (x_start..x_stop).step_by(grid_size as usize).enumerate() {
        let gx = col_idx;
        for (row_idx, y) in (y_start..y_stop).step_by(grid_size as usize).enumerate() {
            if cell_flags[row_idx * cols + gx] {
                empty_cells.push((x, y));
            }
        }
    }

    // **Parallel FAST detection** — each cell is independent
    let cell_results: Vec<Vec<Corner>> = empty_cells
        .par_iter()
        .map(|&(x, y)| {
            let image_view = image.view(x, y, grid_size, grid_size).to_image();
            let mut points_added = 0;
            let mut threshold: u8 = 40;
            let mut cell_corners = Vec::new();

            while points_added < num_points_in_cell && threshold >= 10 {
                let mut fast_corners = corners_fast9(&image_view, threshold);
                fast_corners.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap());

                for mut point in fast_corners {
                    if points_added >= num_points_in_cell {
                        break;
                    }
                    point.x += x;
                    point.y += y;
                    if point_in_bound(&point, h, w, EDGE_THRESHOLD) {
                        cell_corners.push(point);
                        points_added += 1;
                    }
                }
                threshold -= 5;
            }
            cell_corners
        })
        .collect();

    // Merge results from all cells
    for mut cell_corners in cell_results {
        all_corners.append(&mut cell_corners);
    }
    all_corners
}
