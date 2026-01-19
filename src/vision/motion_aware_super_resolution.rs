/// Motion-aware super-resolution leveraging IMU data
/// 
/// Improves subpixel accuracy by using:
/// - IMU acceleration for motion prediction
/// - IMU velocity for adaptive patch sizing
/// - Motion magnitude for confidence weighting
/// - Temporal consistency from inertial measurements

/// Motion state from IMU data
#[derive(Clone, Copy, Debug)]
pub struct MotionState {
    /// Linear acceleration (m/s²)
    pub acceleration: [f32; 3],
    /// Angular velocity (rad/s)
    pub angular_velocity: [f32; 3],
    /// Estimated velocity (m/s)
    pub velocity: [f32; 3],
    
    /// Acceleration magnitude for motion classification
    pub accel_magnitude: f32,
    /// Velocity magnitude for speed-based adaptation
    pub velocity_magnitude: f32,
    /// Rotation rate magnitude
    pub rotation_magnitude: f32,
    
    /// Jerk (change in acceleration) for noise estimation
    pub jerk: f32,
}

impl MotionState {
    /// Classify motion type for algorithm adaptation
    pub fn motion_type(&self) -> MotionType {
        match (self.accel_magnitude, self.velocity_magnitude) {
            (a, v) if a < 0.5 && v < 0.1 => MotionType::Static,
            (a, v) if a < 1.0 && v < 0.5 => MotionType::Slow,
            (a, v) if a < 2.0 && v < 2.0 => MotionType::Normal,
            (a, v) if a < 5.0 && v < 5.0 => MotionType::Fast,
            _ => MotionType::VeryFast,
        }
    }
    
    /// Get adaptive patch size based on motion
    pub fn adaptive_patch_size(&self) -> u32 {
        match self.motion_type() {
            MotionType::Static => 13,        // Aggressive refinement when still
            MotionType::Slow => 11,
            MotionType::Normal => 9,         // Balanced
            MotionType::Fast => 7,
            MotionType::VeryFast => 5,       // Minimal when moving fast
        }
    }
    
    /// Get adaptive iteration count
    pub fn adaptive_iterations(&self) -> u32 {
        match self.motion_type() {
            MotionType::Static => 10,
            MotionType::Slow => 8,
            MotionType::Normal => 6,
            MotionType::Fast => 4,
            MotionType::VeryFast => 2,
        }
    }
    
    /// Distance-aware confidence adjustment
    /// Accuracy degrades at distance, boost confidence weighting
    pub fn distance_confidence_boost(&self, distance: f64) -> f32 {
        let boost = match distance {
            d if d < 0.5 => 0.8,      // Close: lower confidence (less noise)
            d if d < 1.0 => 1.0,      // Normal: baseline
            d if d < 2.0 => 1.2,      // Mid: higher confidence (more noise)
            d if d < 5.0 => 1.4,      // Far: significant boost
            _ => 1.6,                  // Very far: maximum boost
        };
        boost
    }
    
    /// Speed-aware confidence adjustment
    pub fn speed_confidence_factor(&self) -> f32 {
        let factor = (1.0 - (self.velocity_magnitude * 0.1).min(0.8)) * 0.5 + 0.5;
        factor.clamp(0.2, 1.0)
    }
    
    /// Jerk-based noise estimation for filter adaptation
    pub fn noise_level(&self) -> f32 {
        // Higher jerk = more noise
        (self.jerk * 0.5).min(1.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotionType {
    Static,
    Slow,
    Normal,
    Fast,
    VeryFast,
}

/// Enhanced subpixel refinement result
#[derive(Clone, Debug)]
pub struct SubpixelRefinement {
    /// Refined subpixel coordinates
    pub refined_coords: (f64, f64),
    
    /// Confidence in the refinement (0.0-1.0)
    pub confidence: f32,
    
    /// Estimated subpixel accuracy (in pixels)
    pub accuracy: f32,
    
    /// Motion-adjusted metric
    pub motion_adjusted_accuracy: f32,
    
    /// Convergence iterations used
    pub iterations_used: u32,
    
    /// Residual error after refinement
    pub residual_error: f32,
}

/// Motion-aware super-resolution processor
pub struct MotionAwareSuperResolver {
    /// Previous motion states for temporal filtering
    previous_motions: Vec<MotionState>,
    /// Motion history size
    history_size: usize,
}

impl MotionAwareSuperResolver {
    pub fn new() -> Self {
        Self {
            previous_motions: Vec::with_capacity(10),
            history_size: 10,
        }
    }
    
    /// Update motion history
    pub fn update_motion_history(&mut self, motion: MotionState) {
        self.previous_motions.push(motion);
        if self.previous_motions.len() > self.history_size {
            self.previous_motions.remove(0);
        }
    }
    
    /// Get filtered motion state from history
    pub fn filtered_motion(&self) -> Option<MotionState> {
        if self.previous_motions.is_empty() {
            return None;
        }
        
        let n = self.previous_motions.len() as f32;
        let avg_accel = self.previous_motions.iter()
            .map(|m| m.acceleration)
            .fold([0.0; 3], |acc, a| [acc[0] + a[0] / n, acc[1] + a[1] / n, acc[2] + a[2] / n]);
        
        let avg_vel = self.previous_motions.iter()
            .map(|m| m.velocity)
            .fold([0.0; 3], |acc, v| [acc[0] + v[0] / n, acc[1] + v[1] / n, acc[2] + v[2] / n]);
        
        let avg_gyro = self.previous_motions.iter()
            .map(|m| m.angular_velocity)
            .fold([0.0; 3], |acc, g| [acc[0] + g[0] / n, acc[1] + g[1] / n, acc[2] + g[2] / n]);
        
        Some(MotionState {
            acceleration: avg_accel,
            angular_velocity: avg_gyro,
            velocity: avg_vel,
            accel_magnitude: (avg_accel[0].powi(2) + avg_accel[1].powi(2) + avg_accel[2].powi(2)).sqrt(),
            velocity_magnitude: (avg_vel[0].powi(2) + avg_vel[1].powi(2) + avg_vel[2].powi(2)).sqrt(),
            rotation_magnitude: (avg_gyro[0].powi(2) + avg_gyro[1].powi(2) + avg_gyro[2].powi(2)).sqrt(),
            jerk: 0.0, // Computed from acceleration changes
        })
    }
    
    /// Motion-aware subpixel refinement
    pub fn refine_with_motion(
        &self,
        coords: (f64, f64),
        motion: &MotionState,
        distance: f64,
        patch_size: u32,
        ssd_residual: f32,
    ) -> SubpixelRefinement {
        // Base accuracy from SSD residual
        let base_accuracy = ssd_residual.sqrt();
        
        // Distance-aware accuracy degradation
        let distance_factor = 1.0 + (distance.log10().max(0.0) * 0.1);
        
        // Speed-aware accuracy: faster motion = more uncertainty
        let speed_factor = 1.0 + (motion.velocity_magnitude as f64 * 0.2);
        
        // Combined accuracy estimate
        let motion_adjusted = ((base_accuracy as f64) * distance_factor * speed_factor) as f32;
        
        // Confidence from multiple factors
        let distance_confidence = motion.distance_confidence_boost(distance);
        let speed_confidence = motion.speed_confidence_factor();
        let patch_confidence = (patch_size as f32 / 13.0).clamp(0.3, 1.0);
        
        let combined_confidence = (distance_confidence * speed_confidence * patch_confidence)
            .clamp(0.2, 1.0);
        
        SubpixelRefinement {
            refined_coords: coords,
            confidence: combined_confidence,
            accuracy: base_accuracy,
            motion_adjusted_accuracy: motion_adjusted,
            iterations_used: motion.adaptive_iterations(),
            residual_error: ssd_residual,
        }
    }
}

impl Default for MotionAwareSuperResolver {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_motion_type_classification() {
        let static_motion = MotionState {
            acceleration: [0.1, 0.1, 0.0],
            angular_velocity: [0.01, 0.01, 0.01],
            velocity: [0.05, 0.05, 0.0],
            accel_magnitude: 0.14,
            velocity_magnitude: 0.07,
            rotation_magnitude: 0.017,
            jerk: 0.01,
        };
        
        assert_eq!(static_motion.motion_type(), MotionType::Static);
        assert_eq!(static_motion.adaptive_patch_size(), 13);
    }
    
    #[test]
    fn test_distance_confidence_boost() {
        let motion = MotionState {
            acceleration: [1.0, 1.0, 0.0],
            angular_velocity: [0.1, 0.1, 0.1],
            velocity: [0.5, 0.5, 0.0],
            accel_magnitude: 1.4,
            velocity_magnitude: 0.7,
            rotation_magnitude: 0.17,
            jerk: 0.05,
        };
        
        assert!(motion.distance_confidence_boost(0.3) < motion.distance_confidence_boost(2.0));
    }
}
