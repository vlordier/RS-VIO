//! Calibration quality assessment and logging

use std::collections::HashMap;

/// Comprehensive calibration quality metrics
#[derive(Debug, Clone)]
pub struct CalibrationQualityMetrics {
    /// Mean reprojection error (pixels)
    pub mean_reprojection_error: f64,
    /// Median reprojection error (pixels)
    pub median_reprojection_error: f64,
    /// Maximum reprojection error (pixels)
    pub max_reprojection_error: f64,
    /// Standard deviation of reprojection errors
    pub reprojection_error_std: f64,
    /// Percentage of points with error < 1 pixel
    pub accuracy_percentage_1px: f64,
    /// Percentage of points with error < 2 pixels
    pub accuracy_percentage_2px: f64,

    /// Epipolar geometry consistency score (0-1, higher is better)
    pub epipolar_consistency_score: f64,
    /// Stereo triangulation quality score (0-1, higher is better)
    pub triangulation_quality_score: f64,

    /// Rolling shutter compensation quality (if applicable)
    pub rolling_shutter_correction_score: Option<f64>,

    /// Feature quality statistics
    pub mean_feature_quality: f64,
    pub feature_quality_std: f64,

    /// Calibration stability over time (for temporal calibration)
    pub temporal_stability_score: Option<f64>,

    /// Detailed per-point errors for analysis
    pub per_point_errors: Vec<f64>,
    /// Outlier statistics
    pub outlier_count: usize,
    pub total_points: usize,
}

impl CalibrationQualityMetrics {
    /// Create metrics from reprojection errors
    pub fn from_reprojection_errors(errors: Vec<f64>) -> Self {
        if errors.is_empty() {
            return Self {
                mean_reprojection_error: 0.0,
                median_reprojection_error: 0.0,
                max_reprojection_error: 0.0,
                reprojection_error_std: 0.0,
                accuracy_percentage_1px: 0.0,
                accuracy_percentage_2px: 0.0,
                epipolar_consistency_score: 0.0,
                triangulation_quality_score: 0.0,
                rolling_shutter_correction_score: None,
                mean_feature_quality: 0.0,
                feature_quality_std: 0.0,
                temporal_stability_score: None,
                per_point_errors: Vec::new(),
                outlier_count: 0,
                total_points: 0,
            };
        }
        let total_points = errors.len();
        let mean_error = errors.iter().sum::<f64>() / total_points as f64;

        let mut sorted_errors = errors.clone();
        sorted_errors.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median_error = sorted_errors[total_points / 2];

        let max_error = *sorted_errors.last().unwrap_or(&0.0);
        let variance =
            errors.iter().map(|e| (e - mean_error).powi(2)).sum::<f64>() / total_points as f64;
        let std_error = variance.sqrt();

        let accuracy_1px =
            errors.iter().filter(|&&e| e < 1.0).count() as f64 / total_points as f64 * 100.0;
        let accuracy_2px =
            errors.iter().filter(|&&e| e < 2.0).count() as f64 / total_points as f64 * 100.0;

        // Simple outlier detection (points with error > 3*std + mean)
        let outlier_threshold = mean_error + 3.0 * std_error;
        let outlier_count = errors.iter().filter(|&&e| e > outlier_threshold).count();

        Self {
            mean_reprojection_error: mean_error,
            median_reprojection_error: median_error,
            max_reprojection_error: max_error,
            reprojection_error_std: std_error,
            accuracy_percentage_1px: accuracy_1px,
            accuracy_percentage_2px: accuracy_2px,
            epipolar_consistency_score: 0.0,  // To be computed
            triangulation_quality_score: 0.0, // To be computed
            rolling_shutter_correction_score: None,
            mean_feature_quality: 0.0,
            feature_quality_std: 0.0,
            temporal_stability_score: None,
            per_point_errors: errors,
            outlier_count,
            total_points,
        }
    }

    /// Check if calibration meets quality thresholds
    pub fn is_calibration_acceptable(&self, thresholds: &QualityThresholds) -> bool {
        self.mean_reprojection_error < thresholds.max_mean_reprojection_error
            && self.accuracy_percentage_1px > thresholds.min_accuracy_percentage_1px
            && self.outlier_count < thresholds.max_outlier_count
    }

    /// Generate human-readable quality report
    pub fn generate_report(&self) -> String {
        format!(
            r#"Calibration Quality Report
============================

Reprojection Errors:
  Mean: {:.2} pixels
  Median: {:.2} pixels
  Max: {:.2} pixels
  Std Dev: {:.2} pixels

Accuracy:
  < 1px: {:.1}%
  < 2px: {:.1}%

Statistics:
  Total points: {}
  Outliers: {} ({:.1}%)

Quality Scores:
  Epipolar consistency: {:.3}
  Triangulation quality: {:.3}
  Rolling shutter correction: {:.3}
  Feature quality: {:.3} ± {:.3}
  Temporal stability: {:.3}

Overall Assessment: {}
"#,
            self.mean_reprojection_error,
            self.median_reprojection_error,
            self.max_reprojection_error,
            self.reprojection_error_std,
            self.accuracy_percentage_1px,
            self.accuracy_percentage_2px,
            self.total_points,
            self.outlier_count,
            if self.total_points > 0 {
                self.outlier_count as f64 / self.total_points as f64 * 100.0
            } else {
                0.0
            },
            self.epipolar_consistency_score,
            self.triangulation_quality_score,
            self.rolling_shutter_correction_score.unwrap_or(0.0),
            self.mean_feature_quality,
            self.feature_quality_std,
            self.temporal_stability_score.unwrap_or(0.0),
            if self.is_calibration_acceptable(&QualityThresholds::default()) {
                "ACCEPTABLE"
            } else {
                "NEEDS IMPROVEMENT"
            }
        )
    }
}

/// Quality thresholds for calibration acceptance
#[derive(Debug, Clone)]
pub struct QualityThresholds {
    pub max_mean_reprojection_error: f64,
    pub min_accuracy_percentage_1px: f64,
    pub max_outlier_count: usize,
}

impl Default for QualityThresholds {
    fn default() -> Self {
        Self {
            max_mean_reprojection_error: 1.0,  // pixels
            min_accuracy_percentage_1px: 70.0, // percentage
            max_outlier_count: 10,             // absolute count
        }
    }
}

/// Calibration logger for tracking quality over time
pub struct CalibrationLogger {
    metrics_history: Vec<(f64, CalibrationQualityMetrics)>, // (timestamp, metrics)
    quality_thresholds: QualityThresholds,
}

impl CalibrationLogger {
    pub const fn new(thresholds: QualityThresholds) -> Self {
        Self {
            metrics_history: Vec::new(),
            quality_thresholds: thresholds,
        }
    }

    /// Log calibration quality at given timestamp
    pub fn log_metrics(&mut self, timestamp: f64, metrics: CalibrationQualityMetrics) {
        self.metrics_history.push((timestamp, metrics));
    }

    /// Get quality trend analysis
    pub fn analyze_trends(&self) -> HashMap<String, f64> {
        if self.metrics_history.is_empty() {
            return HashMap::new();
        }

        let recent_metrics: Vec<_> = self.metrics_history.iter()
            .rev()
            .take(10) // Last 10 measurements
            .collect();

        let mean_errors: Vec<f64> = recent_metrics
            .iter()
            .map(|(_, m)| m.mean_reprojection_error)
            .collect();

        let improvement_rate = if mean_errors.len() >= 2 {
            (mean_errors[mean_errors.len() - 1] - mean_errors[0]) / (mean_errors.len() - 1) as f64
        } else {
            0.0
        };

        let stability = if mean_errors.len() >= 2 {
            let mean = mean_errors.iter().sum::<f64>() / mean_errors.len() as f64;
            let variance = mean_errors.iter().map(|e| (e - mean).powi(2)).sum::<f64>()
                / mean_errors.len() as f64;
            1.0 / (1.0 + variance.sqrt()) // Higher stability = lower variance
        } else {
            1.0
        };

        let mut trends = HashMap::new();
        trends.insert("improvement_rate".to_string(), improvement_rate);
        trends.insert("stability_score".to_string(), stability);
        trends.insert(
            "current_accuracy".to_string(),
            recent_metrics[0].1.accuracy_percentage_1px,
        );

        trends
    }

    /// Generate comprehensive logging report
    pub fn generate_full_report(&self) -> String {
        let trends = self.analyze_trends();

        let mut report = String::from("Calibration Quality History Report\n");
        report.push_str("===================================\n\n");

        report.push_str(&format!(
            "Total measurements: {}\n",
            self.metrics_history.len()
        ));
        report.push_str(&format!(
            "Quality thresholds: {:?}\n\n",
            self.quality_thresholds
        ));

        if let Some((_, latest)) = self.metrics_history.last() {
            report.push_str("Latest Quality Metrics:\n");
            report.push_str(&latest.generate_report());
            report.push('\n');
        }

        report.push_str("Trend Analysis:\n");
        for (key, value) in &trends {
            report.push_str(&format!("  {}: {:.3}\n", key, value));
        }

        report
    }
}
