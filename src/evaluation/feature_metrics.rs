/// Feature tracking quality metrics

/// Feature tracking metrics
#[derive(Clone, Debug)]
pub struct FeatureMetrics {
    /// Reprojection error in pixels
    pub reprojection_error_mean: f64,
    pub reprojection_error_std: f64,
    pub reprojection_error_max: f64,
    
    /// Feature tracking consistency
    pub match_inlier_ratio: f64,  // 0.0-1.0
    pub tracking_success_rate: f64,  // 0.0-1.0
    
    /// Feature position accuracy
    pub subpixel_accuracy: f64,  // in pixels
    
    /// Total features tracked
    pub feature_count: usize,
    pub inlier_count: usize,
    pub outlier_count: usize,
}

/// Compute reprojection error for feature matches
///
/// Measures how accurately features are located in images
pub fn compute_reprojection_error(
    projected_2d: &[(f64, f64)],  // 2D projections of 3D points
    actual_2d: &[(f64, f64)],      // Actual feature locations
    inlier_mask: Option<&[bool]>,  // Optional inlier mask
) -> Result<FeatureMetrics, String> {
    if projected_2d.len() != actual_2d.len() {
        return Err("Point arrays must have equal length".to_string());
    }
    
    if projected_2d.is_empty() {
        return Err("No features to evaluate".to_string());
    }
    
    let mut errors = Vec::new();
    let mut inlier_count = 0;
    
    for i in 0..projected_2d.len() {
        let dx = projected_2d[i].0 - actual_2d[i].0;
        let dy = projected_2d[i].1 - actual_2d[i].1;
        let error = (dx * dx + dy * dy).sqrt();
        
        // Consider point as inlier if below threshold (typically 1-2 pixels)
        if error < 2.0 {
            inlier_count += 1;
        }
        
        errors.push(error);
    }
    
    // Apply external inlier mask if provided
    if let Some(mask) = inlier_mask {
        inlier_count = mask.iter().filter(|&&m| m).count();
    }
    
    // Compute statistics
    let mean = errors.iter().sum::<f64>() / errors.len() as f64;
    let variance = errors.iter()
        .map(|&e| (e - mean).powi(2))
        .sum::<f64>() / errors.len() as f64;
    let std = variance.sqrt();
    let max = errors.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    
    let outlier_count = projected_2d.len() - inlier_count;
    let match_inlier_ratio = inlier_count as f64 / projected_2d.len() as f64;
    
    Ok(FeatureMetrics {
        reprojection_error_mean: mean,
        reprojection_error_std: std,
        reprojection_error_max: max,
        match_inlier_ratio,
        tracking_success_rate: match_inlier_ratio,
        subpixel_accuracy: std,
        feature_count: projected_2d.len(),
        inlier_count,
        outlier_count,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_perfect_reprojection() {
        let projected = vec![(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)];
        let actual = projected.clone();
        
        let metrics = compute_reprojection_error(&projected, &actual, None).unwrap();
        
        assert!(metrics.reprojection_error_mean < 1e-10);
        assert!(metrics.match_inlier_ratio > 0.99);
    }
    
    #[test]
    fn test_with_outliers() {
        let projected = vec![
            (1.0, 2.0),   // Good
            (3.0, 4.0),   // Good
            (10.0, 20.0), // Bad outlier
        ];
        let actual = vec![
            (1.0, 2.0),
            (3.0, 4.0),
            (5.0, 6.0),
        ];
        
        let metrics = compute_reprojection_error(&projected, &actual, None).unwrap();
        
        // Should have outlier
        assert!(metrics.reprojection_error_max > 10.0);
        assert!(metrics.match_inlier_ratio < 1.0);
    }
}
