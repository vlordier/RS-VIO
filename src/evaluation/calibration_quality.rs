/// Calibration quality assessment for confidence weighting
///
/// Converts CalibrationQualityReport into numerical confidence factors
/// that can be used to weight visual and IMU residuals in fusion algorithms.
use crate::calibration::types::CalibrationQualityReport;

/// Numerical confidence factors derived from calibration quality
#[derive(Clone, Debug)]
pub struct CalibrationConfidenceFactors {
    /// Visual residual confidence (0.0-1.0)
    /// Based on reprojection error and stereo rectification quality
    pub visual_confidence: f32,

    /// IMU residual confidence (0.0-1.0)
    /// Based on IMU integration consistency
    pub imu_confidence: f32,

    /// Timing synchronization confidence (0.0-1.0)
    /// Based on timing curve sharpness and observability
    pub timing_confidence: f32,

    /// Rolling shutter correction confidence (0.0-1.0)
    /// Based on RS significance and readout time estimation quality
    pub rolling_shutter_confidence: f32,

    /// Overall system confidence (0.0-1.0)
    /// Derived from calibration overall_score
    pub overall_confidence: f32,
}

impl CalibrationConfidenceFactors {
    /// Create confidence factors from calibration quality report
    pub fn from_quality_report(report: &CalibrationQualityReport) -> Self {
        // Extract per-metric scores from the report
        let visual_confidence = Self::extract_metric_score(report, "reprojection");
        let imu_confidence = Self::extract_metric_score(report, "imu");
        let timing_confidence = Self::extract_metric_score(report, "timing");
        let rolling_shutter_confidence = Self::extract_metric_score(report, "rolling_shutter");

        // Overall confidence from report score
        let overall_confidence = report.overall_score as f32;

        Self {
            visual_confidence,
            imu_confidence,
            timing_confidence,
            rolling_shutter_confidence,
            overall_confidence,
        }
    }

    /// Extract confidence score for a specific metric from the report
    fn extract_metric_score(report: &CalibrationQualityReport, metric_prefix: &str) -> f32 {
        // Find all metrics matching the prefix
        let matching_metrics: Vec<bool> = report
            .metrics
            .iter()
            .filter(|(name, _, _)| name.to_lowercase().contains(metric_prefix))
            .map(|(_, passed, _)| *passed)
            .collect();

        if matching_metrics.is_empty() {
            // No metric found - use overall score as fallback
            return report.overall_score as f32;
        }

        // Confidence based on pass rate
        let passed_count = matching_metrics.iter().filter(|&&p| p).count();
        passed_count as f32 / matching_metrics.len() as f32
    }

    /// Get confidence for weighting visual reprojection residuals
    ///
    /// Combines visual quality with timing (both affect reprojection)
    pub fn visual_residual_weight(&self) -> f32 {
        (self.visual_confidence * self.timing_confidence).sqrt()
    }

    /// Get confidence for weighting IMU preintegration residuals
    ///
    /// Combines IMU quality with timing
    pub fn imu_residual_weight(&self) -> f32 {
        (self.imu_confidence * self.timing_confidence).sqrt()
    }

    /// Get confidence for rolling shutter correction
    ///
    /// Only use RS correction if confidence is reasonable
    pub fn should_use_rolling_shutter(&self) -> bool {
        self.rolling_shutter_confidence > 0.5
    }

    /// Get adaptive robust loss threshold based on calibration quality
    ///
    /// Poor calibration → larger outlier threshold (more permissive)
    /// Good calibration → tighter threshold (reject more aggressively)
    pub fn robust_threshold_multiplier(&self) -> f32 {
        // Inverse relationship: poor calibration needs larger gates
        1.0 + (1.0 - self.overall_confidence) * 2.0
    }
}

/// Default: assume uncalibrated system (conservative confidence)
impl Default for CalibrationConfidenceFactors {
    fn default() -> Self {
        Self {
            visual_confidence: 0.5,
            imu_confidence: 0.5,
            timing_confidence: 0.3, // Low default: timing usually problematic
            rolling_shutter_confidence: 0.5,
            overall_confidence: 0.43, // (~0.5^0.75)
        }
    }
}

/// Statistics collector for calibration-aware performance tracking
#[derive(Clone, Debug)]
pub struct CalibrationQualityStats {
    /// Reprojection error with calibration weighting
    pub weighted_reprojection_error: Vec<f32>,

    /// Reprojection error without calibration weighting (baseline)
    pub unweighted_reprojection_error: Vec<f32>,

    /// IMU integration error with calibration weighting
    pub weighted_imu_error: Vec<f32>,

    /// IMU integration error without calibration weighting
    pub unweighted_imu_error: Vec<f32>,

    /// Number of visual residuals rejected (outliers)
    pub visual_outliers_rejected: usize,

    /// Number of IMU residuals rejected
    pub imu_outliers_rejected: usize,

    /// Confidence factor used
    pub confidence_factors: CalibrationConfidenceFactors,
}

impl CalibrationQualityStats {
    pub fn new(confidence_factors: CalibrationConfidenceFactors) -> Self {
        Self {
            weighted_reprojection_error: Vec::new(),
            unweighted_reprojection_error: Vec::new(),
            weighted_imu_error: Vec::new(),
            unweighted_imu_error: Vec::new(),
            visual_outliers_rejected: 0,
            imu_outliers_rejected: 0,
            confidence_factors,
        }
    }

    /// Add visual residual sample
    pub fn add_visual_residual(&mut self, error: f32, is_outlier: bool) {
        self.unweighted_reprojection_error.push(error);

        let weighted_error = error * self.confidence_factors.visual_residual_weight();
        self.weighted_reprojection_error.push(weighted_error);

        if is_outlier {
            self.visual_outliers_rejected += 1;
        }
    }

    /// Add IMU residual sample
    pub fn add_imu_residual(&mut self, error: f32, is_outlier: bool) {
        self.unweighted_imu_error.push(error);

        let weighted_error = error * self.confidence_factors.imu_residual_weight();
        self.weighted_imu_error.push(weighted_error);

        if is_outlier {
            self.imu_outliers_rejected += 1;
        }
    }

    /// Compute RMS improvement from calibration weighting
    pub fn compute_improvement(&self) -> CalibrationImprovementMetrics {
        let unweighted_visual_rms = rms(&self.unweighted_reprojection_error);
        let weighted_visual_rms = rms(&self.weighted_reprojection_error);

        let unweighted_imu_rms = rms(&self.unweighted_imu_error);
        let weighted_imu_rms = rms(&self.weighted_imu_error);

        CalibrationImprovementMetrics {
            visual_rms_improvement: (unweighted_visual_rms - weighted_visual_rms)
                / unweighted_visual_rms,
            imu_rms_improvement: (unweighted_imu_rms - weighted_imu_rms) / unweighted_imu_rms,
            visual_outlier_rate: self.visual_outliers_rejected as f32
                / self.unweighted_reprojection_error.len().max(1) as f32,
            imu_outlier_rate: self.imu_outliers_rejected as f32
                / self.unweighted_imu_error.len().max(1) as f32,
        }
    }
}

/// Metrics showing improvement from calibration quality awareness
#[derive(Clone, Debug)]
pub struct CalibrationImprovementMetrics {
    /// Visual RMS improvement (0.0 = no improvement, 1.0 = 100% improvement)
    pub visual_rms_improvement: f32,

    /// IMU RMS improvement
    pub imu_rms_improvement: f32,

    /// Visual outlier rejection rate
    pub visual_outlier_rate: f32,

    /// IMU outlier rejection rate
    pub imu_outlier_rate: f32,
}

/// Compute RMS of samples
fn rms(samples: &[f32]) -> f32 {
    if samples.is_empty() {
        return 0.0;
    }
    (samples.iter().map(|x| x * x).sum::<f32>() / samples.len() as f32).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calibration::types::CalibrationQualityReport;

    #[test]
    fn test_perfect_calibration_confidence() {
        let report = CalibrationQualityReport {
            passed: true,
            metrics: vec![
                (
                    "reprojection_rms".to_string(),
                    true,
                    "Excellent".to_string(),
                ),
                ("vertical_disparity".to_string(), true, "Good".to_string()),
                (
                    "timing_sharpness".to_string(),
                    true,
                    "Observable".to_string(),
                ),
                (
                    "imu_integration".to_string(),
                    true,
                    "Consistent".to_string(),
                ),
                (
                    "rolling_shutter_significance".to_string(),
                    true,
                    "Handled".to_string(),
                ),
            ],
            overall_score: 0.98,
        };

        let factors = CalibrationConfidenceFactors::from_quality_report(&report);

        assert!(factors.visual_confidence > 0.9);
        assert!(factors.imu_confidence > 0.9);
        assert!(factors.timing_confidence > 0.9);
        assert!(factors.rolling_shutter_confidence > 0.9);
        assert!(factors.overall_confidence > 0.9);
    }

    #[test]
    fn test_poor_calibration_confidence() {
        let report = CalibrationQualityReport {
            passed: false,
            metrics: vec![
                ("reprojection_rms".to_string(), false, "Poor".to_string()),
                ("vertical_disparity".to_string(), false, "Bad".to_string()),
                (
                    "timing_sharpness".to_string(),
                    false,
                    "Unobservable".to_string(),
                ),
                (
                    "imu_integration".to_string(),
                    false,
                    "Inconsistent".to_string(),
                ),
                (
                    "rolling_shutter_significance".to_string(),
                    false,
                    "Problematic".to_string(),
                ),
            ],
            overall_score: 0.25,
        };

        let factors = CalibrationConfidenceFactors::from_quality_report(&report);

        assert!(factors.visual_confidence < 0.5);
        assert!(factors.imu_confidence < 0.5);
        assert!(factors.timing_confidence < 0.5);
        assert!(factors.overall_confidence < 0.5);
    }

    #[test]
    fn test_marginal_calibration() {
        let report = CalibrationQualityReport {
            passed: true,
            metrics: vec![
                (
                    "reprojection_rms".to_string(),
                    true,
                    "Acceptable".to_string(),
                ),
                (
                    "vertical_disparity".to_string(),
                    false,
                    "Marginal".to_string(),
                ),
                ("timing_sharpness".to_string(), true, "Fair".to_string()),
                ("imu_integration".to_string(), true, "OK".to_string()),
            ],
            overall_score: 0.65,
        };

        let factors = CalibrationConfidenceFactors::from_quality_report(&report);

        // With mixed pass/fail, visual confidence should be moderate (50% of reprojection metrics passed)
        assert!(factors.visual_confidence >= 0.3);
        assert!(factors.overall_confidence >= 0.6 && factors.overall_confidence <= 0.7);
    }

    #[test]
    fn test_rolling_shutter_gating() {
        let report_good_rs = CalibrationQualityReport {
            passed: true,
            metrics: vec![
                (
                    "rolling_shutter_significance".to_string(),
                    true,
                    "Good".to_string(),
                ),
                (
                    "rolling_shutter_readout".to_string(),
                    true,
                    "Accurate".to_string(),
                ),
            ],
            overall_score: 0.95,
        };

        let factors_good = CalibrationConfidenceFactors::from_quality_report(&report_good_rs);
        assert!(factors_good.should_use_rolling_shutter());

        let report_bad_rs = CalibrationQualityReport {
            passed: false,
            metrics: vec![(
                "rolling_shutter_significance".to_string(),
                false,
                "Poor".to_string(),
            )],
            overall_score: 0.4,
        };

        let factors_bad = CalibrationConfidenceFactors::from_quality_report(&report_bad_rs);
        assert!(!factors_bad.should_use_rolling_shutter());
    }

    #[test]
    fn test_robust_threshold_adaptation() {
        let perfect_report = CalibrationQualityReport {
            passed: true,
            metrics: vec![],
            overall_score: 1.0,
        };

        let poor_report = CalibrationQualityReport {
            passed: false,
            metrics: vec![],
            overall_score: 0.2,
        };

        let factors_perfect = CalibrationConfidenceFactors::from_quality_report(&perfect_report);
        let factors_poor = CalibrationConfidenceFactors::from_quality_report(&poor_report);

        // Poor calibration should have larger robust threshold (more permissive)
        assert!(
            factors_poor.robust_threshold_multiplier()
                > factors_perfect.robust_threshold_multiplier()
        );
        assert!((factors_perfect.robust_threshold_multiplier() - 1.0).abs() < 0.01);
        // ~1.0 for perfect
    }

    #[test]
    fn test_calibration_quality_stats() {
        let factors = CalibrationConfidenceFactors {
            visual_confidence: 0.8,
            imu_confidence: 0.9,
            timing_confidence: 0.85,
            rolling_shutter_confidence: 0.75,
            overall_confidence: 0.82,
        };

        let mut stats = CalibrationQualityStats::new(factors);

        // Add some visual residuals
        stats.add_visual_residual(2.0, false);
        stats.add_visual_residual(3.0, false);
        stats.add_visual_residual(10.0, true); // Outlier

        // Add some IMU residuals
        stats.add_imu_residual(0.5, false);
        stats.add_imu_residual(0.8, false);

        assert_eq!(stats.visual_outliers_rejected, 1);
        assert_eq!(stats.unweighted_reprojection_error.len(), 3);

        let improvement = stats.compute_improvement();
        assert!(improvement.visual_outlier_rate > 0.0);
    }
}
