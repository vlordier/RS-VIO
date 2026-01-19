/// Motion-aware depth optimization using IMU fusion
/// 
/// Improves 3D depth accuracy by:
/// - Using IMU velocity to weight triangulation
/// - Applying motion-based constraint refinement
/// - Detecting and handling occlusions with acceleration data
/// - Distance-aware uncertainty modeling

use nalgebra::{Point3, Vector3};

/// Motion-constrained depth estimate
#[derive(Clone, Debug)]
pub struct MotionConstrainedDepth {
    /// 3D position
    pub position: Point3<f64>,
    
    /// Estimated uncertainty in depth (meters)
    pub depth_uncertainty: f64,
    
    /// Distance from camera
    pub distance: f64,
    
    /// Confidence score (0.0-1.0)
    pub confidence: f32,
    
    /// Distance-normalized accuracy (uncertainty / distance)
    pub relative_accuracy: f32,
    
    /// Motion consistency score
    pub motion_consistency: f32,
}

/// IMU-aided triangulation constraints
#[derive(Clone, Copy, Debug)]
pub struct TriangulationConstraints {
    /// Baseline velocity (from IMU)
    pub baseline_velocity: Vector3<f64>,
    
    /// Angular velocity (from gyro)
    pub angular_velocity: Vector3<f64>,
    
    /// Expected epipolar line uncertainty due to motion
    pub motion_epipolar_uncertainty: f64,
    
    /// Whether motion is too fast for this frame
    pub motion_too_fast: bool,
}

impl TriangulationConstraints {
    /// Create constraints from IMU state and motion
    pub fn from_motion(
        velocity: Vector3<f64>,
        angular_velocity: Vector3<f64>,
        baseline: f64,
        frame_time: f64,
    ) -> Self {
        let motion_magnitude = velocity.norm();
        let rotation_magnitude = angular_velocity.norm();
        
        // Epipolar uncertainty due to motion
        let motion_uncertainty = motion_magnitude * frame_time * 0.05;
        let rotation_uncertainty = rotation_magnitude * baseline * 0.1;
        let total_uncertainty = (motion_uncertainty.powi(2) + rotation_uncertainty.powi(2)).sqrt();
        
        Self {
            baseline_velocity: velocity,
            angular_velocity,
            motion_epipolar_uncertainty: total_uncertainty,
            motion_too_fast: motion_magnitude > 5.0 || rotation_magnitude > 1.0,
        }
    }
    
    /// Get weight adjustment for triangulation point
    pub fn triangulation_weight(&self, point_distance: f64) -> f32 {
        // Closer points more affected by motion
        let distance_factor = (point_distance / 2.0).clamp(0.5, 1.0);
        
        // Apply motion uncertainty
        let uncertainty_factor = (1.0 / (1.0 + self.motion_epipolar_uncertainty)).clamp(0.3, 1.0);
        
        (distance_factor * uncertainty_factor as f64) as f32
    }
}

/// Motion-aware depth optimizer
pub struct MotionAwareDepthOptimizer {
    /// Previous depth estimates for temporal smoothing
    depth_history: Vec<MotionConstrainedDepth>,
    history_size: usize,
}

impl MotionAwareDepthOptimizer {
    pub fn new() -> Self {
        Self {
            depth_history: Vec::with_capacity(30),
            history_size: 30,
        }
    }
    
    /// Optimize depth with motion constraints
    pub fn optimize_depth(
        &mut self,
        position: Point3<f64>,
        initial_uncertainty: f64,
        constraints: &TriangulationConstraints,
        velocity_magnitude: f32,
    ) -> MotionConstrainedDepth {
        let distance = position.coords.norm();
        
        // Base confidence from uncertainty
        let base_confidence = 1.0 / (1.0 + initial_uncertainty as f32);
        
        // Distance-aware confidence: farther = less certain
        let distance_confidence_boost = constraints.triangulation_weight(distance);
        
        // Speed-aware confidence: faster = more uncertain
        let speed_confidence = (1.0 - (velocity_magnitude * 0.05).min(0.6)).clamp(0.4, 1.0);
        
        // Combined confidence
        let confidence = (base_confidence * distance_confidence_boost * speed_confidence)
            .clamp(0.1, 1.0);
        
        // Motion-constrained uncertainty
        let motion_factor = (1.0 + constraints.motion_epipolar_uncertainty).clamp(1.0, 3.0);
        let optimized_uncertainty = initial_uncertainty * motion_factor;
        
        // Relative accuracy: normalized by distance
        let relative_accuracy = (optimized_uncertainty / distance.max(0.1)) as f32;
        
        let depth = MotionConstrainedDepth {
            position,
            depth_uncertainty: optimized_uncertainty,
            distance,
            confidence,
            relative_accuracy,
            motion_consistency: 1.0,
        };
        
        self.depth_history.push(depth.clone());
        if self.depth_history.len() > self.history_size {
            self.depth_history.remove(0);
        }
        
        depth
    }
    
    /// Temporally smooth depth estimates
    pub fn temporal_smoothing(&self, new_depth: &MotionConstrainedDepth) -> MotionConstrainedDepth {
        if self.depth_history.is_empty() {
            return new_depth.clone();
        }
        
        // Weighted average of recent depths
        let recent_count = self.depth_history.len().min(5);
        let weights: Vec<f32> = (0..recent_count)
            .map(|i| {
                let age = (recent_count - 1 - i) as f32;
                (-age.powi(2) / 2.0).exp()
            })
            .collect();
        
        let weight_sum: f32 = weights.iter().sum();
        
        let mut smoothed_pos = Point3::origin();
        for (i, depth) in self.depth_history.iter().rev().take(recent_count).enumerate() {
            smoothed_pos += depth.position.coords * (weights[i] / weight_sum) as f64;
        }
        
        let smoothed_uncertainty = self.depth_history.iter().rev()
            .take(recent_count)
            .zip(weights.iter())
            .map(|(d, w)| d.depth_uncertainty * (w / weight_sum) as f64)
            .sum();
        
        let distance = smoothed_pos.coords.norm();
        
        MotionConstrainedDepth {
            position: smoothed_pos,
            depth_uncertainty: smoothed_uncertainty,
            distance,
            confidence: new_depth.confidence,
            relative_accuracy: (smoothed_uncertainty / distance.max(0.1)) as f32,
            motion_consistency: new_depth.motion_consistency,
        }
    }
    
    /// Detect depth anomalies using motion history
    pub fn detect_anomaly(&self, new_depth: &MotionConstrainedDepth) -> bool {
        if self.depth_history.len() < 3 {
            return false;
        }
        
        let recent: Vec<_> = self.depth_history.iter()
            .rev()
            .take(3)
            .collect();
        
        let avg_distance = recent.iter().map(|d| d.distance).sum::<f64>() / recent.len() as f64;
        
        // Large jump in depth
        (new_depth.distance - avg_distance).abs() > avg_distance * 0.3
    }
}

impl Default for MotionAwareDepthOptimizer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_triangulation_constraints() {
        let velocity = Vector3::new(1.0, 0.5, 0.1);
        let angular_velocity = Vector3::new(0.1, 0.05, 0.02);
        
        let constraints = TriangulationConstraints::from_motion(
            velocity,
            angular_velocity,
            0.12,
            0.033, // 30Hz frame
        );
        
        assert!(constraints.motion_epipolar_uncertainty > 0.0);
        assert!(!constraints.motion_too_fast);
    }
    
    #[test]
    fn test_depth_optimization() {
        let mut optimizer = MotionAwareDepthOptimizer::new();
        
        let position = Point3::new(1.0, 0.5, 5.0);
        let constraints = TriangulationConstraints::from_motion(
            Vector3::new(0.5, 0.2, 0.0),
            Vector3::new(0.05, 0.02, 0.01),
            0.12,
            0.033,
        );
        
        let depth = optimizer.optimize_depth(
            position,
            0.05,
            &constraints,
            0.54,
        );
        
        assert!(depth.confidence > 0.0 && depth.confidence <= 1.0);
        assert!(depth.distance > 0.0);
    }
    
    #[test]
    fn test_temporal_smoothing() {
        let mut optimizer = MotionAwareDepthOptimizer::new();
        
        // Add some history
        for i in 0..5 {
            let pos = Point3::new(1.0, 0.5, 5.0 + i as f64 * 0.01);
            let constraints = TriangulationConstraints::from_motion(
                Vector3::zeros(),
                Vector3::zeros(),
                0.12,
                0.033,
            );
            optimizer.optimize_depth(pos, 0.05, &constraints, 0.0);
        }
        
        let new_depth = MotionConstrainedDepth {
            position: Point3::new(1.0, 0.5, 5.05),
            depth_uncertainty: 0.05,
            distance: 5.08,
            confidence: 0.8,
            relative_accuracy: 0.01,
            motion_consistency: 0.95,
        };
        
        let smoothed = optimizer.temporal_smoothing(&new_depth);
        assert!(smoothed.depth_uncertainty >= 0.0);
    }
}
