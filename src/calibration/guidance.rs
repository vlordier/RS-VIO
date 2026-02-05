//! Adaptive calibration guidance system
//!
//! This module provides intelligent guidance for optimal camera movements during
//! stereo calibration, ensuring maximum accuracy with minimal data collection.

use nalgebra as na;

/// Calibration coverage analysis for different parameter types
#[derive(Debug, Clone)]
pub struct CalibrationCoverage {
    /// Translation coverage in X, Y, Z directions (0-1, higher is better)
    pub translation_coverage: na::Vector3<f64>,
    /// Rotation coverage around X, Y, Z axes (0-1, higher is better)
    pub rotation_coverage: na::Vector3<f64>,
    /// Overall calibration coverage score (0-1)
    pub overall_coverage: f64,
    /// Baseline distance confidence (meters)
    pub baseline_confidence: f64,
    /// Focal length confidence (pixels)
    pub focal_length_confidence: f64,
    /// Principal point confidence (pixels)
    pub principal_point_confidence: f64,
}

/// Suggested camera movement for optimal calibration
#[derive(Debug, Clone)]
pub struct CalibrationSuggestion {
    /// Type of movement recommended
    pub movement_type: MovementType,
    /// Movement magnitude (meters or radians)
    pub magnitude: f64,
    /// Movement direction (unit vector)
    pub direction: na::Vector3<f64>,
    /// Expected improvement in calibration quality (0-1)
    pub expected_improvement: f64,
    /// Human-readable description
    pub description: String,
}

/// Types of calibration movements
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MovementType {
    /// Pure translation
    Translation,
    /// Pure rotation
    Rotation,
    /// Combined translation and rotation
    Combined,
    /// Circular motion for better baseline estimation
    Circular,
    /// Figure-8 pattern for comprehensive coverage
    FigureEight,
}

/// Real-time calibration status and guidance
#[derive(Debug, Clone)]
pub struct CalibrationGuidance {
    /// Current calibration coverage
    pub coverage: CalibrationCoverage,
    /// Next recommended movement
    pub next_suggestion: Option<CalibrationSuggestion>,
    /// Current calibration quality score (0-1)
    pub quality_score: f64,
    /// Estimated time to completion (seconds)
    pub estimated_time_remaining: Option<f64>,
    /// Whether calibration is ready
    pub calibration_ready: bool,
    /// Progress messages for user
    pub status_messages: Vec<String>,
}

impl Default for CalibrationCoverage {
    fn default() -> Self {
        Self {
            translation_coverage: na::Vector3::new(0.0, 0.0, 0.0),
            rotation_coverage: na::Vector3::new(0.0, 0.0, 0.0),
            overall_coverage: 0.0,
            baseline_confidence: 0.0,
            focal_length_confidence: 0.0,
            principal_point_confidence: 0.0,
        }
    }
}

impl Default for CalibrationGuidance {
    fn default() -> Self {
        Self::new()
    }
}

impl CalibrationGuidance {
    /// Create new calibration guidance system
    pub fn new() -> Self {
        Self {
            coverage: CalibrationCoverage::default(),
            next_suggestion: None,
            quality_score: 0.0,
            estimated_time_remaining: None,
            calibration_ready: false,
            status_messages: Vec::new(),
        }
    }

    /// Analyze current stereo pairs and update guidance
    pub fn update_guidance(&mut self, stereo_pairs: &[super::stereo_calibrator::StereoPair]) {
        self.analyze_coverage(stereo_pairs);
        self.generate_suggestion();
        self.update_quality_score();
        self.check_completion();
        self.update_status_messages();
    }

    /// Analyze how well different parameters are constrained
    fn analyze_coverage(&mut self, stereo_pairs: &[super::stereo_calibrator::StereoPair]) {
        if stereo_pairs.is_empty() {
            return;
        }

        // Analyze translation coverage by looking at camera motion patterns
        let mut translations = Vec::new();
        let mut rotations = Vec::new();

        // Assume first pair is reference pose
        let reference_timestamp = stereo_pairs[0].timestamp;

        for pair in stereo_pairs {
            let dt = pair.timestamp - reference_timestamp;

            // Extract motion from angular/linear velocity if available
            if let (Some(ang_vel), Some(lin_vel)) = (&pair.angular_velocity, &pair.linear_velocity)
            {
                let translation = lin_vel * dt;
                let rotation = ang_vel * dt;

                translations.push(translation);
                rotations.push(rotation);
            }
        }

        // Calculate coverage based on motion diversity
        if !translations.is_empty() {
            // Translation coverage based on range of motion in each direction
            let mut min_trans = na::Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut max_trans =
                na::Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

            for trans in &translations {
                min_trans = min_trans.inf(trans);
                max_trans = max_trans.sup(trans);
            }

            let trans_range = max_trans - min_trans;
            // Normalize by typical calibration motion (assume 0.5m range is good)
            self.coverage.translation_coverage =
                (trans_range / 0.5).inf(&na::Vector3::new(1.0, 1.0, 1.0));
        }

        if !rotations.is_empty() {
            // Rotation coverage based on angular range
            let mut min_rot = na::Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut max_rot =
                na::Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

            for rot in &rotations {
                min_rot = min_rot.inf(rot);
                max_rot = max_rot.sup(rot);
            }

            let rot_range = max_rot - min_rot;
            // Normalize by typical calibration rotation (assume 1.0 rad ~ 57 deg is good)
            self.coverage.rotation_coverage =
                (rot_range / 1.0).inf(&na::Vector3::new(1.0, 1.0, 1.0));
        }

        // Overall coverage is weighted average
        let trans_avg = self.coverage.translation_coverage.mean();
        let rot_avg = self.coverage.rotation_coverage.mean();
        self.coverage.overall_coverage = 0.6 * trans_avg + 0.4 * rot_avg;

        // Estimate parameter confidences based on coverage and data amount
        let data_factor = (stereo_pairs.len() as f64 / 20.0).min(1.0); // Normalize to 20 pairs
        self.coverage.baseline_confidence = self.coverage.translation_coverage.z * data_factor;
        self.coverage.focal_length_confidence = self.coverage.overall_coverage * data_factor;
        self.coverage.principal_point_confidence =
            self.coverage.overall_coverage * data_factor * 0.8;
    }

    /// Generate next movement suggestion based on current coverage
    fn generate_suggestion(&mut self) {
        // Find the weakest coverage area
        let trans_min = self.coverage.translation_coverage.min();
        let rot_min = self.coverage.rotation_coverage.min();

        if trans_min < 0.5 && trans_min <= rot_min {
            // Need more translation coverage
            let weak_axis = self.coverage.translation_coverage.imin();
            let direction = match weak_axis {
                0 => na::Vector3::new(1.0, 0.0, 0.0), // X-axis
                1 => na::Vector3::new(0.0, 1.0, 0.0), // Y-axis
                _ => na::Vector3::new(0.0, 0.0, 1.0), // Z-axis
            };

            let magnitude = 0.3; // 30cm translation
            self.next_suggestion = Some(CalibrationSuggestion {
                movement_type: MovementType::Translation,
                magnitude,
                direction,
                expected_improvement: (1.0 - trans_min) * 0.3,
                description: format!(
                    "Move camera {:.0}cm {} to improve translation coverage",
                    magnitude * 100.0,
                    match weak_axis {
                        0 => "left/right",
                        1 => "up/down",
                        _ => "forward/backward",
                    }
                ),
            });
        } else if rot_min < 0.5 {
            // Need more rotation coverage
            let weak_axis = self.coverage.rotation_coverage.imin();
            let direction = match weak_axis {
                0 => na::Vector3::new(1.0, 0.0, 0.0), // Rotate around X
                1 => na::Vector3::new(0.0, 1.0, 0.0), // Rotate around Y
                _ => na::Vector3::new(0.0, 0.0, 1.0), // Rotate around Z
            };

            let magnitude = 0.5; // ~30 degrees
            self.next_suggestion = Some(CalibrationSuggestion {
                movement_type: MovementType::Rotation,
                magnitude,
                direction,
                expected_improvement: (1.0 - rot_min) * 0.3,
                description: format!(
                    "Rotate camera {:.0}° around {} axis",
                    magnitude.to_degrees(),
                    match weak_axis {
                        0 => "pitch",
                        1 => "yaw",
                        _ => "roll",
                    }
                ),
            });
        } else if self.coverage.overall_coverage < 0.7 {
            // Need more comprehensive motion
            self.next_suggestion = Some(CalibrationSuggestion {
                movement_type: MovementType::FigureEight,
                magnitude: 0.4,                             // 40cm figure-8
                direction: na::Vector3::new(0.0, 0.0, 1.0), // Forward motion
                expected_improvement: (1.0 - self.coverage.overall_coverage) * 0.4,
                description: "Perform a figure-8 motion pattern for comprehensive calibration"
                    .to_string(),
            });
        } else {
            // Coverage is good, suggest circular motion for baseline refinement
            self.next_suggestion = Some(CalibrationSuggestion {
                movement_type: MovementType::Circular,
                magnitude: 0.3, // 30cm radius circle
                direction: na::Vector3::new(0.0, 0.0, 1.0),
                expected_improvement: 0.1,
                description: "Move in a circular pattern to refine stereo baseline estimation"
                    .to_string(),
            });
        }
    }

    /// Update overall quality score
    fn update_quality_score(&mut self) {
        // Quality based on coverage and data amount
        let coverage_score = self.coverage.overall_coverage;
        let baseline_score = self.coverage.baseline_confidence;
        let intrinsics_score = (self.coverage.focal_length_confidence
            + self.coverage.principal_point_confidence)
            / 2.0;

        self.quality_score =
            (0.4 * coverage_score + 0.3 * baseline_score + 0.3 * intrinsics_score).min(1.0);
    }

    /// Check if calibration is complete
    fn check_completion(&mut self) {
        self.calibration_ready = self.quality_score >= 0.8
            && self.coverage.baseline_confidence >= 0.7
            && self.coverage.focal_length_confidence >= 0.7;

        if self.calibration_ready {
            self.estimated_time_remaining = Some(0.0);
        } else {
            // Estimate time based on current progress
            let remaining_progress = 1.0 - self.quality_score;
            self.estimated_time_remaining = Some(remaining_progress * 60.0); // Assume 1 minute per 0.1 quality
        }
    }

    /// Update status messages for user feedback
    fn update_status_messages(&mut self) {
        self.status_messages.clear();

        // Overall status
        if self.calibration_ready {
            self.status_messages
                .push("🎉 Calibration ready! Quality is excellent.".to_string());
        } else {
            self.status_messages.push(format!(
                "📊 Calibration quality: {:.1}%",
                self.quality_score * 100.0
            ));
        }

        // Coverage breakdown
        self.status_messages.push(format!(
            "🎯 Coverage: Translation {:.0}%, Rotation {:.0}%",
            self.coverage.translation_coverage.mean() * 100.0,
            self.coverage.rotation_coverage.mean() * 100.0
        ));

        // Parameter confidences
        self.status_messages.push(format!(
            "📏 Baseline: {:.1}%, Focal length: {:.1}%, Principal point: {:.1}%",
            self.coverage.baseline_confidence * 100.0,
            self.coverage.focal_length_confidence * 100.0,
            self.coverage.principal_point_confidence * 100.0
        ));

        // Next suggestion
        if let Some(suggestion) = &self.next_suggestion {
            self.status_messages
                .push(format!("💡 Next: {}", suggestion.description));
            if suggestion.expected_improvement > 0.0 {
                self.status_messages.push(format!(
                    "📈 Expected improvement: +{:.1}% quality",
                    suggestion.expected_improvement * 100.0
                ));
            }
        }

        // Time estimate
        if let Some(time_remaining) = self.estimated_time_remaining {
            if time_remaining > 0.0 {
                self.status_messages.push(format!(
                    "⏱️  Estimated time remaining: {:.0}s",
                    time_remaining
                ));
            }
        }
    }

    /// Get current status for display
    pub fn get_status_display(&self) -> String {
        let mut display = String::new();

        for message in &self.status_messages {
            display.push_str(message);
            display.push('\n');
        }

        display
    }
}
