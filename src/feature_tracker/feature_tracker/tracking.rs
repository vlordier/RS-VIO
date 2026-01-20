/// Core optical flow tracking algorithms

use image::GrayImage;
use imageproc::corners::Corner;
use nalgebra as na;
use rayon::prelude::*;
use std::collections::HashMap;
use std::ops::AddAssign;

use crate::feature_tracker::{image_utilities, patch};

pub fn add_points(
    tracked_points_map: &HashMap<usize, na::Affine2<f32>>,
    grayscale_image: &GrayImage,
    grid_size: u32,
) -> Vec<(f32, f32, f32)> {
    let num_points_in_cell = 1;
    let current_corners: Vec<Corner> = tracked_points_map
        .values()
        .map(|v| {
            Corner::new(
                v.matrix().m13.round() as u32,
                v.matrix().m23.round() as u32,
                0.0,
            )
        })
        .collect();
    image_utilities::detect_key_points(
        grayscale_image,
        grid_size,
        &current_corners,
        num_points_in_cell,
    )
}

#[inline]
pub fn track_points<const LEVELS: u32>(
    image_pyramid0: &[GrayImage],
    image_pyramid1: &[GrayImage],
    transform_maps0: &HashMap<usize, na::Affine2<f32>>,
    optical_flow_max_iterations: usize,
    optical_flow_convergence_threshold: f32,
) -> HashMap<usize, na::Affine2<f32>> {
    // Sequential for small datasets (common case < 100 features) - rayon has overhead
    // Parallel for large datasets where thread spawning pays off
    let use_parallel = transform_maps0.len() > 64;

    let results: Vec<(usize, na::Affine2<f32>)> = if use_parallel {
        transform_maps0
            .par_iter()
            .filter_map(|(k, v)| {
                track_one_point::<LEVELS>(
                    image_pyramid0,
                    image_pyramid1,
                    v,
                    optical_flow_max_iterations,
                    optical_flow_convergence_threshold,
                )
                .map(|new_v| (*k, new_v))
            })
            .collect()
    } else {
        transform_maps0
            .iter()
            .filter_map(|(k, v)| {
                track_one_point::<LEVELS>(
                    image_pyramid0,
                    image_pyramid1,
                    v,
                    optical_flow_max_iterations,
                    optical_flow_convergence_threshold,
                )
                .map(|new_v| (*k, new_v))
            })
            .collect()
    };

    results.into_iter().collect()
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
    let patten = &dp.pattern_matrix;

    for _iteration in 0..optical_flow_max_iterations {
        // Transform pattern: R * pattern + t
        let mut transformed_pat = transform.matrix().fixed_view::<2, 2>(0, 0) * patten;
        let translation = transform.matrix().fixed_view::<2, 1>(0, 2);
        for i in 0..52 {
            transformed_pat.column_mut(i).add_assign(translation);
        }

        if let Some(res) = dp.residual(grayscale_image, &transformed_pat) {
            let inc = -dp.h_se2_inv_j_se2_t * res;

            // avoid NaN in increment (leads to SE2::exp crashing)
            if !inc.iter().all(|x| x.is_finite()) {
                return false;
            }
            if inc.norm() > 1e6 {
                return false;
            }

            // Early termination if converged
            if inc.norm() < optical_flow_convergence_threshold {
                break;
            }

            let new_trans = transform.matrix() * image_utilities::se2_exp_matrix(&inc);
            *transform = na::Affine2::<f32>::from_matrix_unchecked(new_trans);
            let filter_margin = 2;
            if !image_utilities::inbound(
                grayscale_image,
                transform.matrix_mut_unchecked().m13,
                transform.matrix_mut_unchecked().m23,
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
