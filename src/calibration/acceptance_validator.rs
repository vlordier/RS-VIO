/// Calibration acceptance framework
///
/// Validates calibration quality against production thresholds.
/// Implements automated pass/fail decision for camera + IMU calibration bundles.
use crate::calibration::types::AcceptanceThresholds;
use std::collections::HashMap;

/// Acceptance decision and detailed quality report
#[derive(Debug, Clone)]
pub struct CalibrationAcceptanceReport {
    /// Overall acceptance: pass or fail
    pub accepted: bool,
    /// Overall quality score (0-100, higher is better)
    pub overall_score: f64,
    /// Individual metric scores
    pub metric_scores: HashMap<String, MetricScore>,
    /// Reasons for rejection (empty if accepted)
    pub rejection_reasons: Vec<String>,
    /// Recommended actions
    pub recommendations: Vec<String>,
}

/// Score for a single calibration metric
#[derive(Debug, Clone)]
pub struct MetricScore {
    pub name: String,
    pub measured_value: f64,
    pub threshold_good: f64,
    pub threshold_acceptable: f64,
    pub score: f64, // 0-100
    pub status: AcceptanceStatus,
    pub unit: String,
}

/// Quality status for a metric
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AcceptanceStatus {
    Excellent,  // Well below threshold
    Good,       // Below threshold
    Acceptable, // At/near threshold
    Poor,       // Above threshold
    Failed,     // Far above threshold
}

impl AcceptanceStatus {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Excellent => "Excellent",
            Self::Good => "Good",
            Self::Acceptable => "Acceptable",
            Self::Poor => "Poor",
            Self::Failed => "Failed",
        }
    }
}

impl std::fmt::Display for AcceptanceStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.as_str())
    }
}

/// Comprehensive calibration validator
pub struct CalibrationAcceptanceValidator {
    /// Acceptance thresholds
    thresholds: AcceptanceThresholds,
    /// Collected metrics
    metrics: HashMap<String, f64>,
}

impl CalibrationAcceptanceValidator {
    /// Create validator with standard thresholds
    pub fn new() -> Self {
        Self {
            thresholds: AcceptanceThresholds::standard(),
            metrics: HashMap::new(),
        }
    }

    /// Create with custom thresholds
    pub fn with_thresholds(thresholds: AcceptanceThresholds) -> Self {
        Self {
            thresholds,
            metrics: HashMap::new(),
        }
    }

    /// Record a measured metric
    pub fn record_metric(&mut self, name: &str, value: f64) {
        self.metrics.insert(name.to_string(), value);
    }

    /// Validate all collected metrics
    pub fn validate(&self) -> CalibrationAcceptanceReport {
        let mut metric_scores = HashMap::new();
        let mut rejection_reasons = Vec::new();
        let mut recommendations = Vec::new();
        let mut total_score = 0.0;
        let mut metric_count = 0;

        // Validate reprojection RMS (camera intrinsics)
        if let Some(&value) = self.metrics.get("reprojection_rms") {
            let (score, status) = self.score_reprojection(value);
            metric_scores.insert(
                "reprojection_rms".to_string(),
                MetricScore {
                    name: "Camera Reprojection RMS".to_string(),
                    measured_value: value,
                    threshold_good: self.thresholds.reprojection_rms_good,
                    threshold_acceptable: self.thresholds.reprojection_rms_max,
                    score,
                    status,
                    unit: "pixels".to_string(),
                },
            );

            total_score += score;
            metric_count += 1;

            if status == AcceptanceStatus::Failed {
                rejection_reasons.push(format!("Camera reprojection RMS too high: {:.3}px", value));
                recommendations.push(
                    "Recalibrate camera intrinsics with better calibration target".to_string(),
                );
            } else if status == AcceptanceStatus::Poor {
                recommendations
                    .push("Camera calibration is marginal; consider recalibrating".to_string());
            }
        }

        // Validate vertical disparity (stereo extrinsics)
        if let Some(&value) = self.metrics.get("vertical_disparity_rms") {
            let (score, status) = self.score_vertical_disparity(value);
            metric_scores.insert(
                "vertical_disparity_rms".to_string(),
                MetricScore {
                    name: "Vertical Disparity RMS".to_string(),
                    measured_value: value,
                    threshold_good: 0.1,
                    threshold_acceptable: self.thresholds.vertical_disparity_rms_max,
                    score,
                    status,
                    unit: "pixels".to_string(),
                },
            );

            total_score += score;
            metric_count += 1;

            if status == AcceptanceStatus::Failed {
                rejection_reasons.push(format!("Stereo rectification too poor: {:.3}px", value));
                recommendations
                    .push("Check stereo baseline alignment and re-estimate extrinsics".to_string());
            }
        }

        // Validate epipolar residual
        if let Some(&value) = self.metrics.get("epipolar_residual") {
            let (score, status) = self.score_epipolar(value);
            metric_scores.insert(
                "epipolar_residual".to_string(),
                MetricScore {
                    name: "Epipolar Residual".to_string(),
                    measured_value: value,
                    threshold_good: 0.3,
                    threshold_acceptable: self.thresholds.epipolar_residual_max,
                    score,
                    status,
                    unit: "pixels".to_string(),
                },
            );

            total_score += score;
            metric_count += 1;

            if status == AcceptanceStatus::Failed {
                rejection_reasons.push("Epipolar constraint violated too often".to_string());
                recommendations
                    .push("Improve feature matching quality or re-estimate geometry".to_string());
            }
        }

        // Validate timing observability
        if let Some(&value) = self.metrics.get("timing_observability") {
            let (score, status) = self.score_timing_observability(value);
            metric_scores.insert(
                "timing_observability".to_string(),
                MetricScore {
                    name: "Camera-IMU Timing Observability".to_string(),
                    measured_value: value,
                    threshold_good: 0.7,
                    threshold_acceptable: self.thresholds.timing_observability_min,
                    score,
                    status,
                    unit: "score".to_string(),
                },
            );

            total_score += score;
            metric_count += 1;

            if status == AcceptanceStatus::Failed {
                rejection_reasons.push("Camera-IMU timing not observable".to_string());
                recommendations.push(
                    "Ensure adequate motion diversity; time offset may not be observable"
                        .to_string(),
                );
            }
        }

        // Validate time offset jitter
        if let Some(&value) = self.metrics.get("time_offset_jitter") {
            let (score, status) = self.score_time_jitter(value);
            metric_scores.insert(
                "time_offset_jitter".to_string(),
                MetricScore {
                    name: "Time Offset Jitter".to_string(),
                    measured_value: value,
                    threshold_good: 0.5e-3, // 0.5ms
                    threshold_acceptable: self.thresholds.time_jitter_max,
                    score,
                    status,
                    unit: "seconds".to_string(),
                },
            );

            total_score += score;
            metric_count += 1;

            if status == AcceptanceStatus::Failed {
                rejection_reasons.push("Time offset estimate unstable".to_string());
                recommendations
                    .push("Improve IMU data quality or camera exposure stability".to_string());
            }
        }

        let overall_score = if metric_count > 0 {
            total_score / metric_count as f64
        } else {
            0.0
        };

        let (accepted, _acceptance_status) = if rejection_reasons.is_empty() {
            let status = if overall_score >= 90.0 {
                AcceptanceStatus::Excellent
            } else if overall_score >= 80.0 {
                AcceptanceStatus::Good
            } else {
                AcceptanceStatus::Acceptable
            };
            (true, status)
        } else {
            (false, AcceptanceStatus::Failed)
        };

        CalibrationAcceptanceReport {
            accepted,
            overall_score,
            metric_scores,
            rejection_reasons,
            recommendations,
        }
    }

    fn score_reprojection(&self, value: f64) -> (f64, AcceptanceStatus) {
        let good = self.thresholds.reprojection_rms_good;
        let max = self.thresholds.reprojection_rms_max;
        self.score_lower_is_better(value, good, max)
    }

    fn score_vertical_disparity(&self, value: f64) -> (f64, AcceptanceStatus) {
        let max = self.thresholds.vertical_disparity_rms_max;
        let good = max * 0.5;
        self.score_lower_is_better(value, good, max)
    }

    fn score_epipolar(&self, value: f64) -> (f64, AcceptanceStatus) {
        let max = self.thresholds.epipolar_residual_max;
        let good = max * 0.5;
        self.score_lower_is_better(value, good, max)
    }

    fn score_timing_observability(&self, value: f64) -> (f64, AcceptanceStatus) {
        let min = self.thresholds.timing_observability_min;
        let good = min + 0.2;
        self.score_higher_is_better(value, min, good)
    }

    fn score_time_jitter(&self, value: f64) -> (f64, AcceptanceStatus) {
        let max = self.thresholds.time_jitter_max;
        let good = max * 0.5;
        self.score_lower_is_better(value, good, max)
    }

    /// Score metric where lower is better
    fn score_lower_is_better(&self, value: f64, good: f64, max: f64) -> (f64, AcceptanceStatus) {
        if value <= good {
            (100.0, AcceptanceStatus::Excellent)
        } else if value <= max {
            let ratio = (value - good) / (max - good);
            let score = 100.0 - (ratio * 40.0); // 100 to 60
            (score, AcceptanceStatus::Good)
        } else if value <= max * 1.5 {
            (40.0, AcceptanceStatus::Acceptable)
        } else if value <= max * 2.0 {
            (20.0, AcceptanceStatus::Poor)
        } else {
            (0.0, AcceptanceStatus::Failed)
        }
    }

    /// Score metric where higher is better
    fn score_higher_is_better(&self, value: f64, min: f64, good: f64) -> (f64, AcceptanceStatus) {
        if value >= good {
            (100.0, AcceptanceStatus::Excellent)
        } else if value >= min {
            let ratio = (good - value) / (good - min);
            let score = 100.0 - (ratio * 40.0); // 100 to 60
            (score, AcceptanceStatus::Good)
        } else if value >= min * 0.75 {
            (40.0, AcceptanceStatus::Acceptable)
        } else if value >= min * 0.5 {
            (20.0, AcceptanceStatus::Poor)
        } else {
            (0.0, AcceptanceStatus::Failed)
        }
    }
}

impl CalibrationAcceptanceReport {
    /// Print a human-readable report
    pub fn print_report(&self) {
        println!("\n=== Calibration Acceptance Report ===");
        println!("Overall Quality Score: {:.1}/100.0", self.overall_score);
        println!(
            "Acceptance Status: {}",
            if self.accepted { "PASSED" } else { "FAILED" }
        );

        println!("\nMetric Details:");
        for (_name, metric) in &self.metric_scores {
            println!(
                "  {}: {:.3} {} (score: {:.1}, status: {})",
                metric.name, metric.measured_value, metric.unit, metric.score, metric.status
            );
        }

        if !self.rejection_reasons.is_empty() {
            println!("\nRejection Reasons:");
            for reason in &self.rejection_reasons {
                println!("  - {}", reason);
            }
        }

        if !self.recommendations.is_empty() {
            println!("\nRecommendations:");
            for rec in &self.recommendations {
                println!("  - {}", rec);
            }
        }
    }

    /// Export to JSON format
    pub fn to_json(&self) -> String {
        let mut json = format!(
            r#"{{
  "accepted": {},
  "overall_score": {},
  "metrics": {{
"#,
            self.accepted, self.overall_score
        );

        let metric_strs: Vec<String> = self
            .metric_scores
            .iter()
            .map(|(name, m)| {
                format!(
                    r#"    "{}": {{"value": {}, "unit": "{}", "status": "{}", "score": {}}}"#,
                    name, m.measured_value, m.unit, m.status, m.score
                )
            })
            .collect();
        json.push_str(&metric_strs.join(",\n"));

        json.push_str(&format!(
            r#"
  }},
  "rejection_reasons": [{}],
  "recommendations": [{}]
}}"#,
            self.rejection_reasons
                .iter()
                .map(|r| format!(r#""{}""#, r))
                .collect::<Vec<_>>()
                .join(", "),
            self.recommendations
                .iter()
                .map(|r| format!(r#""{}""#, r))
                .collect::<Vec<_>>()
                .join(", ")
        ));

        json
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_excellent_calibration() {
        let mut validator = CalibrationAcceptanceValidator::new();
        validator.record_metric("reprojection_rms", 0.2);
        validator.record_metric("vertical_disparity_rms", 0.1);
        validator.record_metric("epipolar_residual", 0.2);
        validator.record_metric("timing_observability", 0.8);
        validator.record_metric("time_offset_jitter", 0.3e-3);

        let report = validator.validate();
        assert!(report.accepted);
        assert!(report.overall_score >= 85.0);
        assert!(report.rejection_reasons.is_empty());
    }

    #[test]
    fn test_marginal_calibration() {
        let mut validator = CalibrationAcceptanceValidator::new();
        validator.record_metric("reprojection_rms", 0.6);
        validator.record_metric("vertical_disparity_rms", 0.25);
        validator.record_metric("epipolar_residual", 0.8);
        validator.record_metric("timing_observability", 0.55);
        validator.record_metric("time_offset_jitter", 0.9e-3);

        let report = validator.validate();
        assert!(report.accepted);
        assert!(report.overall_score < 80.0);
    }

    #[test]
    fn test_failed_calibration() {
        let mut validator = CalibrationAcceptanceValidator::new();
        validator.record_metric("reprojection_rms", 2.0);
        validator.record_metric("vertical_disparity_rms", 1.0);
        validator.record_metric("timing_observability", 0.2);

        let report = validator.validate();
        assert!(!report.accepted);
        assert!(!report.rejection_reasons.is_empty());
    }

    #[test]
    fn test_custom_thresholds() {
        let mut thresholds = AcceptanceThresholds::standard();
        thresholds.reprojection_rms_max = 0.5;

        let mut validator = CalibrationAcceptanceValidator::with_thresholds(thresholds);
        validator.record_metric("reprojection_rms", 0.4);

        let report = validator.validate();
        assert!(report.accepted);
    }
}
