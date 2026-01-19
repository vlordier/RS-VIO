/// Robustness and stability metrics

/// Robustness metrics
#[derive(Clone, Debug)]
pub struct RobustnessMetrics {
    /// Tracking failure rate (0.0 = never fails, 1.0 = always fails)
    pub failure_rate: f64,
    
    /// Number of tracking losses
    pub failure_count: usize,
    pub recovery_count: usize,
    
    /// Average failure duration (frames)
    pub avg_failure_duration: f64,
    pub max_failure_duration: usize,
    
    /// Pose jump magnitude (sudden large errors)
    pub max_pose_jump: f64,
    pub avg_pose_jump: f64,
    
    /// Velocity smoothness (derivative of pose)
    pub velocity_variance: f64,
    pub angular_velocity_variance: f64,
    
    /// Total frames processed
    pub frame_count: usize,
}

/// Track failure detection and recovery
/// Identifies frames where tracking quality drops significantly
pub fn track_failure_rate(
    pose_errors: &[f64],  // Pose estimation errors per frame
    confidence_scores: &[f64],  // Tracking confidence 0.0-1.0
    error_threshold: f64,  // Error threshold for failure
) -> Result<RobustnessMetrics, String> {
    if pose_errors.len() != confidence_scores.len() {
        return Err("Arrays must have equal length".to_string());
    }
    
    if pose_errors.is_empty() {
        return Err("No data to analyze".to_string());
    }
    
    // Detect failures: high error or low confidence
    let mut failures = vec![false; pose_errors.len()];
    for i in 0..pose_errors.len() {
        failures[i] = pose_errors[i] > error_threshold || confidence_scores[i] < 0.3;
    }
    
    // Count failure events and recoveries
    let mut failure_count = 0;
    let mut recovery_count = 0;
    let mut in_failure = false;
    let mut current_failure_duration = 0;
    let mut failure_durations = Vec::new();
    
    for is_failure in failures.iter() {
        if *is_failure && !in_failure {
            failure_count += 1;
            in_failure = true;
            current_failure_duration = 1;
        } else if *is_failure && in_failure {
            current_failure_duration += 1;
        } else if !*is_failure && in_failure {
            recovery_count += 1;
            in_failure = false;
            failure_durations.push(current_failure_duration);
            current_failure_duration = 0;
        }
    }
    
    if in_failure {
        failure_durations.push(current_failure_duration);
    }
    
    let avg_failure_duration = if !failure_durations.is_empty() {
        failure_durations.iter().sum::<usize>() as f64 / failure_durations.len() as f64
    } else {
        0.0
    };
    
    let max_failure_duration = failure_durations.iter().copied().max().unwrap_or(0);
    
    let failure_rate = failures.iter().filter(|&&f| f).count() as f64 / failures.len() as f64;
    
    // Compute pose jumps
    let mut pose_jumps = Vec::new();
    for i in 1..pose_errors.len() {
        let jump = (pose_errors[i] - pose_errors[i - 1]).abs();
        pose_jumps.push(jump);
    }
    
    let max_pose_jump = pose_jumps.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let avg_pose_jump = if !pose_jumps.is_empty() {
        pose_jumps.iter().sum::<f64>() / pose_jumps.len() as f64
    } else {
        0.0
    };
    
    // Compute velocity smoothness (variance of derivatives)
    let mut velocities = Vec::new();
    for i in 1..pose_errors.len() {
        velocities.push(pose_errors[i] - pose_errors[i - 1]);
    }
    
    let velocity_variance = if !velocities.is_empty() {
        let mean = velocities.iter().sum::<f64>() / velocities.len() as f64;
        velocities.iter()
            .map(|&v| (v - mean).powi(2))
            .sum::<f64>() / velocities.len() as f64
    } else {
        0.0
    };
    
    // For angular velocity, estimate from confidence scores
    let mut conf_diffs = Vec::new();
    for i in 1..confidence_scores.len() {
        conf_diffs.push(confidence_scores[i] - confidence_scores[i - 1]);
    }
    
    let angular_velocity_variance = if !conf_diffs.is_empty() {
        let mean = conf_diffs.iter().sum::<f64>() / conf_diffs.len() as f64;
        conf_diffs.iter()
            .map(|&d| (d - mean).powi(2))
            .sum::<f64>() / conf_diffs.len() as f64
    } else {
        0.0
    };
    
    Ok(RobustnessMetrics {
        failure_rate,
        failure_count,
        recovery_count,
        avg_failure_duration,
        max_failure_duration,
        max_pose_jump,
        avg_pose_jump,
        velocity_variance,
        angular_velocity_variance,
        frame_count: pose_errors.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_no_failures() {
        let errors = vec![0.01; 100];
        let confidence = vec![0.95; 100];
        
        let metrics = track_failure_rate(&errors, &confidence, 0.5).unwrap();
        
        assert!(metrics.failure_rate < 0.01);
        assert_eq!(metrics.failure_count, 0);
    }
    
    #[test]
    fn test_with_failures() {
        let mut errors = vec![0.01; 100];
        errors[20..30].fill(0.8);  // Failure interval
        
        let confidence = vec![0.95; 100];
        
        let metrics = track_failure_rate(&errors, &confidence, 0.5).unwrap();
        
        assert!(metrics.failure_rate > 0.05);
        assert!(metrics.failure_count > 0);
    }
}
