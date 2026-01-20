/// Depth and disparity estimation accuracy metrics

/// Depth estimation metrics
#[derive(Clone, Debug)]
pub struct DepthMetrics {
    /// Root Mean Square Error in meters
    pub rmse: f64,

    /// Mean Absolute Error in meters
    pub mae: f64,

    /// Percentage of pixels within threshold
    pub outlier_percentage: f64, // % of points beyond 2*std

    /// Depth accuracy statistics
    pub min_depth: f64,
    pub max_depth: f64,
    pub mean_depth: f64,

    /// Disparity accuracy (in pixels)
    pub disparity_rmse: f64,
    pub disparity_mae: f64,

    /// Number of estimated depth points
    pub point_count: usize,
}

/// Compute depth/disparity estimation accuracy
///
/// Compares estimated 3D points to ground truth based on stereo geometry
pub fn compute_depth_rmse(
    estimated_points: &[[f64; 3]],    // 3D points from estimation
    ground_truth_points: &[[f64; 3]], // 3D points from ground truth
    _depth_threshold: f64,            // Max acceptable depth error in meters
) -> Result<DepthMetrics, String> {
    if estimated_points.len() != ground_truth_points.len() {
        return Err("Point arrays must have equal length".to_string());
    }

    if estimated_points.is_empty() {
        return Err("No points to evaluate".to_string());
    }

    let mut depth_errors = Vec::new();
    let mut disparity_errors = Vec::new();
    let mut depths = Vec::new();

    for (est, gt) in estimated_points.iter().zip(ground_truth_points.iter()) {
        // Compute point-to-point distance
        let dx = est[0] - gt[0];
        let dy = est[1] - gt[1];
        let dz = est[2] - gt[2];
        let error = (dx * dx + dy * dy + dz * dz).sqrt();

        depth_errors.push(error);

        // Also track depth component (Z)
        let depth_error = (est[2] - gt[2]).abs();
        disparity_errors.push(depth_error);

        depths.push(gt[2]);
    }

    // Compute statistics
    let rmse =
        (depth_errors.iter().map(|&e| e * e).sum::<f64>() / depth_errors.len() as f64).sqrt();

    let mae = depth_errors.iter().sum::<f64>() / depth_errors.len() as f64;

    let mean_error = mae;
    let std_dev = (depth_errors
        .iter()
        .map(|&e| (e - mean_error).powi(2))
        .sum::<f64>()
        / depth_errors.len() as f64)
        .sqrt();

    let outliers = depth_errors.iter().filter(|&&e| e > 2.0 * std_dev).count();
    let outlier_pct = (outliers as f64 / depth_errors.len() as f64) * 100.0;

    let disparity_rmse = (disparity_errors.iter().map(|&e| e * e).sum::<f64>()
        / disparity_errors.len() as f64)
        .sqrt();

    let disparity_mae = disparity_errors.iter().sum::<f64>() / disparity_errors.len() as f64;

    let min_depth = depths.iter().copied().fold(f64::INFINITY, f64::min);
    let max_depth = depths.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let mean_depth = depths.iter().sum::<f64>() / depths.len() as f64;

    Ok(DepthMetrics {
        rmse,
        mae,
        outlier_percentage: outlier_pct,
        min_depth,
        max_depth,
        mean_depth,
        disparity_rmse,
        disparity_mae,
        point_count: estimated_points.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_depth_perfect() {
        let points = vec![[1.0, 2.0, 3.0]; 10];
        let metrics = compute_depth_rmse(&points, &points, 0.1).unwrap();

        assert!(metrics.rmse < 1e-10);
        assert!(metrics.mae < 1e-10);
    }

    #[test]
    fn test_depth_constant_offset() {
        let estimated = vec![[1.0, 2.0, 3.5]; 10];
        let ground_truth = vec![[1.0, 2.0, 3.0]; 10];

        let metrics = compute_depth_rmse(&estimated, &ground_truth, 0.1).unwrap();

        // Should have 0.5m constant error
        assert!((metrics.mae - 0.5).abs() < 1e-10);
    }
}
