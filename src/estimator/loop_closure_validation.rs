//! Geometric validation for loop closure matches using 8-point algorithm and RANSAC.
//!
//! Validates loop closure candidates by:
//! 1. Estimating essential matrix from point correspondences
//! 2. Decomposing E into rotation and translation
//! 3. Triangulating matched points in 3D
//! 4. Estimating scale from point cloud

use crate::types::{Matrix3x3, Vector3};
use nalgebra as na;
use rand::Rng;

/// Result of loop closure geometric validation.
#[derive(Debug, Clone)]
pub struct LoopClosureValidation {
    /// Relative rotation matrix (R_newer_from_older)
    pub r_rel: Matrix3x3,
    /// Relative translation vector (t_newer_from_older), unit norm
    pub t_rel: Vector3,
    /// Estimated scale of the baseline
    pub scale: f64,
    /// Number of inlier triangulated points
    pub inlier_count: usize,
    /// Mean reprojection error of inliers
    pub reproj_error: f64,
}

/// Validate a loop closure match using 8-point RANSAC algorithm.
///
/// # Arguments
/// * `pts1` - Feature positions in first image (N×2, pixel coordinates)
/// * `pts2` - Feature positions in second image (N×2, pixel coordinates)
/// * `K` - Camera intrinsics (3×3)
/// * `max_iterations` - RANSAC iterations (default: 200)
/// * `reprojection_threshold` - Inlier threshold in pixels (default: 1.0)
///
/// # Returns
/// LoopClosureValidation if successful, None if too few points or degenerate case
pub fn validate_loop_closure(
    pts1: &[(f32, f32)],
    pts2: &[(f32, f32)],
    K: &Matrix3x3,
    max_iterations: usize,
    reprojection_threshold: f64,
) -> Option<LoopClosureValidation> {
    if pts1.len() < 8 || pts2.len() < 8 || pts1.len() != pts2.len() {
        return None;
    }

    // Convert to normalized coordinates
    let K_inv = K.try_inverse()?;
    let pts1_norm: Vec<[f64; 2]> = pts1
        .iter()
        .map(|&(x, y)| {
            let p = K_inv * na::Vector3::new(x as f64, y as f64, 1.0);
            [p[0] / p[2], p[1] / p[2]]
        })
        .collect();

    let pts2_norm: Vec<[f64; 2]> = pts2
        .iter()
        .map(|&(x, y)| {
            let p = K_inv * na::Vector3::new(x as f64, y as f64, 1.0);
            [p[0] / p[2], p[1] / p[2]]
        })
        .collect();

    // RANSAC loop
    let mut best_validation = None;
    let mut best_inlier_count = 0;
    let mut rng = rand::thread_rng();

    for _ in 0..max_iterations {
        // Random 8-point sample
        let sample_indices: Vec<usize> = (0..8)
            .map(|_| rng.gen_range(0..pts1_norm.len()))
            .collect();

        let sample1: Vec<[f64; 2]> = sample_indices.iter().map(|&i| pts1_norm[i]).collect();
        let sample2: Vec<[f64; 2]> = sample_indices.iter().map(|&i| pts2_norm[i]).collect();

        // Estimate essential matrix
        if let Some(E) = estimate_essential_matrix(&sample1, &sample2) {
            // Try all 4 decompositions
            let decompositions = decompose_essential_matrix(&E);

            for (R, t) in decompositions {
                // Count inliers using epipolar constraint
                let mut inlier_indices = Vec::new();

                for i in 0..pts1_norm.len() {
                    let err = epipolar_distance(pts1_norm[i], pts2_norm[i], &E);
                    if err < reprojection_threshold * 2.0 {
                        inlier_indices.push(i);
                    }
                }

                // Check chiral constraint: points should be in front of both cameras
                let valid_count = inlier_indices
                    .iter()
                    .filter(|&&i| check_chiral_constraint(pts1_norm[i], pts2_norm[i], &R, &t))
                    .count();

                if valid_count > best_inlier_count {
                    best_inlier_count = valid_count;

                    // Triangulate and estimate scale
                    if let Some(validation) = triangulate_and_estimate_scale(
                        &inlier_indices,
                        &pts1_norm,
                        &pts2_norm,
                        &R,
                        &t,
                        reprojection_threshold,
                    ) {
                        best_validation = Some(validation);
                    }
                }
            }
        }
    }

    best_validation
}

/// Estimate essential matrix using 8-point algorithm with SVD.
fn estimate_essential_matrix(pts1: &[[f64; 2]], pts2: &[[f64; 2]]) -> Option<Matrix3x3> {
    if pts1.len() < 8 {
        return None;
    }

    // Build constraint matrix A (N×9)
    let mut A = na::DMatrix::zeros(pts1.len(), 9);

    for i in 0..pts1.len() {
        let [x, y] = pts1[i];
        let [x_p, y_p] = pts2[i];

        A[(i, 0)] = x_p * x;
        A[(i, 1)] = x_p * y;
        A[(i, 2)] = x_p;
        A[(i, 3)] = y_p * x;
        A[(i, 4)] = y_p * y;
        A[(i, 5)] = y_p;
        A[(i, 6)] = x;
        A[(i, 7)] = y;
        A[(i, 8)] = 1.0;
    }

    // SVD: A = U S VT
    let svd = A.svd(true, true);

    // Essential matrix from last column of V (= last row of VT)
    let VT = svd.v_t?;
    let mut E = Matrix3x3::identity();
    for i in 0..3 {
        for j in 0..3 {
            E[(i, j)] = VT[(8, i * 3 + j)];
        }
    }

    // Enforce rank-2 constraint: keep largest 2 singular values
    let E_svd = E.svd(true, true);
    let U = E_svd.u?;
    let E_VT = E_svd.v_t?;

    let mut Sigma = na::Matrix3::zeros();
    Sigma[(0, 0)] = 1.0;
    Sigma[(1, 1)] = 1.0;
    Sigma[(2, 2)] = 0.0; // Force smallest singular value to zero

    Some(U * Sigma * E_VT)
}

/// Decompose essential matrix into 4 possible (R, t) solutions.
fn decompose_essential_matrix(E: &Matrix3x3) -> Vec<(Matrix3x3, Vector3)> {
    let svd = E.svd(true, true);

    let U = match svd.u {
        Some(u) => u,
        None => return Vec::new(),
    };

    let VT = match svd.v_t {
        Some(vt) => vt,
        None => return Vec::new(),
    };

    // W matrix for decomposition
    let W = na::Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);

    // Two possible rotations
    let R1 = U * W * VT;
    let R2 = U * W.transpose() * VT;

    // Ensure det(R) = +1
    let R1 = if R1.determinant() < 0.0 { -R1 } else { R1 };
    let R2 = if R2.determinant() < 0.0 { -R2 } else { R2 };

    // Two possible translations (from last column of U)
    let t_pos = U.column(2).into_owned();
    let t_neg = -t_pos;

    vec![(R1, t_pos.clone()), (R1, t_neg.clone()), (R2, t_pos), (R2, t_neg)]
}

/// Compute epipolar distance between point pair given essential matrix.
fn epipolar_distance(x1: [f64; 2], x2: [f64; 2], E: &Matrix3x3) -> f64 {
    let p1 = na::Vector3::new(x1[0], x1[1], 1.0);
    let p2 = na::Vector3::new(x2[0], x2[1], 1.0);

    let Ep1 = E * p1;
    let ETp2 = E.transpose() * p2;

    // Distance = |p2ᵀ E p1| / sqrt(l'₁² + l'₂² + l₁² + l₂²)
    let numerator = (p2.dot(&Ep1)).abs();
    let denominator = (Ep1[0] * Ep1[0] + Ep1[1] * Ep1[1] + ETp2[0] * ETp2[0] + ETp2[1] * ETp2[1])
        .sqrt();

    if denominator < 1e-8 {
        1e6
    } else {
        numerator / denominator
    }
}

/// Check if point is in front of both cameras (positive depth).
fn check_chiral_constraint(
    x1: [f64; 2],
    x2: [f64; 2],
    R: &Matrix3x3,
    t: &Vector3,
) -> bool {
    // Triangulate point
    if let Some(X) = triangulate_point(x1, x2, R, t) {
        // Check if positive depth in first camera
        if X[2] <= 0.0 {
            return false;
        }

        // Check if positive depth in second camera (X' = R X + t)
        let X_prime = R * X + t;
        if X_prime[2] <= 0.0 {
            return false;
        }

        true
    } else {
        false
    }
}

/// Triangulate a single point using linear least squares.
fn triangulate_point(
    x1: [f64; 2],
    x2: [f64; 2],
    R: &Matrix3x3,
    t: &Vector3,
) -> Option<Vector3> {
    // Simplified linear triangulation using simple method
    // For now, use a simple approach: find rays in each camera and find closest point

    // Ray from first camera: direction is (x1[0], x1[1], 1) normalized
    let ray1 = na::Vector3::new(x1[0], x1[1], 1.0).normalize();

    // Ray from second camera: direction is (x2[0], x2[1], 1) normalized
    let ray2 = na::Vector3::new(x2[0], x2[1], 1.0).normalize();

    // Transform ray2 to first camera frame
    let ray2_in_frame1 = R.transpose() * ray2;
    let t_in_frame1 = -R.transpose() * t;

    // Find closest point on two rays:
    // P1(s) = s * ray1
    // P2(u) = u * ray2_in_frame1 + t_in_frame1
    // Minimize |P1(s) - P2(u)|²

    let a = ray1.dot(&ray1);
    let b_coef = -ray1.dot(&ray2_in_frame1);
    let c = ray2_in_frame1.dot(&ray2_in_frame1);
    let d = ray1.dot(&t_in_frame1);
    let e = -ray2_in_frame1.dot(&t_in_frame1);

    let denom = a * c - b_coef * b_coef;
    if denom.abs() < 1e-8 {
        return None;
    }

    let s = (c * d + b_coef * e) / denom;
    if s < 0.1 {
        return None;
    }

    Some(s * ray1)
}

/// Compute skew-symmetric matrix for cross product.
#[allow(dead_code)]
fn skew_symmetric(v: Vector3) -> Matrix3x3 {
    na::Matrix3::new(
        0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0,
    )
}

/// Triangulate all inliers and estimate scale.
fn triangulate_and_estimate_scale(
    inlier_indices: &[usize],
    pts1: &[[f64; 2]],
    pts2: &[[f64; 2]],
    R: &Matrix3x3,
    t: &Vector3,
    _threshold: f64,
) -> Option<LoopClosureValidation> {
    if inlier_indices.len() < 10 {
        return None;
    }

    let mut valid_points = Vec::new();
    let mut total_error = 0.0;

    for &i in inlier_indices {
        if let Some(X) = triangulate_point(pts1[i], pts2[i], R, t) {
            // Check positive depth
            if X[2] > 0.1 && (R * X + t)[2] > 0.1 {
                // Estimate reprojection error (simple approximation)
                total_error += (X[0] - pts1[i][0]).abs() + (X[1] - pts1[i][1]).abs();
                valid_points.push(X);
            }
        }
    }

    if valid_points.len() < 10 {
        return None;
    }

    // Estimate scale from mean depth of triangulated points
    let mean_depth = valid_points.iter().map(|p| p[2]).sum::<f64>() / valid_points.len() as f64;

    // Scale is inverse of mean depth (E gives unit translation)
    let scale = 1.0 / mean_depth.max(0.1);
    let reproj_error = total_error / valid_points.len() as f64;

    // t_rel normalized (unit norm)
    let t_norm = t.norm();
    let t_rel = if t_norm > 1e-8 { t / t_norm } else { t.clone() };

    Some(LoopClosureValidation {
        r_rel: R.clone(),
        t_rel,
        scale,
        inlier_count: valid_points.len(),
        reproj_error,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_skew_symmetric() {
        let v = na::Vector3::new(1.0, 2.0, 3.0);
        let S = skew_symmetric(v);

        // S * v should be zero
        let result = S * v;
        assert!(result.norm() < 1e-10);
    }

}
