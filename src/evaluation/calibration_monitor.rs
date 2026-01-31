/// Closed-loop calibration monitoring and auto-triggering
///
/// Automatically detects degraded calibration quality and recommends recalibration
use crate::evaluation::CalibrationAwareMetrics;

/// Monitors calibration health over time and recommends actions
#[derive(Clone, Debug)]
pub struct CalibrationMonitor {
    /// Thresholds for various metrics
    pub thresholds: CalibrationHealthThresholds,

    /// Historical metrics for trend analysis
    pub history: CalibrationHealthHistory,

    /// Current health assessment
    pub current_health: CalibrationHealth,
}

/// Configurable health thresholds
#[derive(Clone, Debug)]
pub struct CalibrationHealthThresholds {
    /// Minimum acceptable improvement % (default: 15%)
    pub min_improvement_pct: f32,

    /// Minimum median track length in frames (default: 4)
    pub min_median_track_frames: f32,

    /// Minimum survival rate > 5 frames (default: 40%)
    pub min_survival_rate_5: f32,

    /// Maximum acceptable outlier rate (default: 10%)
    pub max_outlier_rate: f32,

    /// Minimum RMS improvement (px) - if improvement < this, consider recalibration
    pub min_rms_improvement_px: f32,

    /// Number of samples to collect before next assessment
    pub assessment_interval: usize,
}

impl Default for CalibrationHealthThresholds {
    fn default() -> Self {
        Self {
            min_improvement_pct: 15.0,
            min_median_track_frames: 4.0,
            min_survival_rate_5: 40.0,
            max_outlier_rate: 10.0,
            min_rms_improvement_px: 0.05,
            assessment_interval: 10000, // 10k measurements
        }
    }
}

/// Current health status
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CalibrationHealth {
    /// All metrics excellent
    Excellent,

    /// Most metrics good, minor issues
    Good,

    /// Some metrics below threshold
    Marginal,

    /// Multiple failures - recommend recalibration
    Poor,
}

impl CalibrationHealth {
    pub fn needs_recalibration(&self) -> bool {
        matches!(self, CalibrationHealth::Poor)
    }

    pub fn should_monitor_closely(&self) -> bool {
        matches!(self, CalibrationHealth::Marginal | CalibrationHealth::Poor)
    }
}

/// Detailed assessment reasons
#[derive(Clone, Debug)]
pub struct CalibrationHealthAssessment {
    pub health: CalibrationHealth,
    pub issues: Vec<HealthIssue>,
    pub recommendations: Vec<String>,
    pub metrics_summary: MetricsSummary,
}

#[derive(Clone, Debug)]
pub struct HealthIssue {
    pub category: String,
    pub severity: IssueSeverity,
    pub message: String,
    pub value: f32,
    pub threshold: f32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum IssueSeverity {
    Info,
    Warning,
    Critical,
}

#[derive(Clone, Debug)]
pub struct MetricsSummary {
    pub overall_improvement_pct: f32,
    pub median_track_length: f32,
    pub survival_rate_5: f32,
    pub outlier_rate: f32,
    pub visual_rms_px: f64,
    pub imu_rms: f64,
}

/// Historical tracking for trend analysis
#[derive(Clone, Debug)]
pub struct CalibrationHealthHistory {
    /// Recent assessments (max 100)
    assessments: Vec<CalibrationHealthSnapshot>,

    /// Max history size
    max_size: usize,
}

#[derive(Clone, Debug)]
pub struct CalibrationHealthSnapshot {
    pub timestamp: std::time::SystemTime,
    pub health: CalibrationHealth,
    pub improvement_pct: f32,
    pub track_length: f32,
}

impl CalibrationHealthHistory {
    pub fn new() -> Self {
        Self {
            assessments: Vec::new(),
            max_size: 100,
        }
    }

    pub fn add(&mut self, health: CalibrationHealth, improvement: f32, track_length: f32) {
        self.assessments.push(CalibrationHealthSnapshot {
            timestamp: std::time::SystemTime::now(),
            health,
            improvement_pct: improvement,
            track_length,
        });

        if self.assessments.len() > self.max_size {
            self.assessments.remove(0);
        }
    }

    pub fn get_trend(&self) -> CalibrationTrend {
        if self.assessments.len() < 2 {
            return CalibrationTrend::Insufficient;
        }

        let recent = &self.assessments[self.assessments.len() - 10..];
        let poor_count = recent
            .iter()
            .filter(|s| s.health == CalibrationHealth::Poor)
            .count();
        let marginal_count = recent
            .iter()
            .filter(|s| s.health == CalibrationHealth::Marginal)
            .count();

        let avg_improvement: f32 =
            recent.iter().map(|s| s.improvement_pct).sum::<f32>() / recent.len() as f32;

        if poor_count >= 3 {
            CalibrationTrend::Degrading
        } else if marginal_count >= 5 {
            CalibrationTrend::Declining
        } else if avg_improvement < 10.0 {
            CalibrationTrend::Stagnant
        } else {
            CalibrationTrend::Stable
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CalibrationTrend {
    Insufficient,
    Stable,
    Stagnant,
    Declining,
    Degrading,
}

impl CalibrationMonitor {
    pub fn new(thresholds: CalibrationHealthThresholds) -> Self {
        Self {
            thresholds,
            history: CalibrationHealthHistory::new(),
            current_health: CalibrationHealth::Good,
        }
    }

    /// Assess calibration quality from metrics
    pub fn assess(&mut self, metrics: &CalibrationAwareMetrics) -> CalibrationHealthAssessment {
        let mut issues = Vec::new();
        let mut score = 100.0_f32;

        // Extract summary metrics
        let improvement = metrics
            .weighted_residuals
            .improvement_vs(&metrics.unweighted_residuals);
        let track_length = metrics.track_survival.median_track_length;
        let survival_5 = metrics.track_survival.survival_rate_5;
        let outlier_rate = metrics.weighted_residuals.outlier_rate;
        let visual_rms = metrics.weighted_residuals.visual_rms;
        let imu_rms = metrics.weighted_residuals.imu_rms;

        // Check improvement
        if improvement < self.thresholds.min_improvement_pct {
            issues.push(HealthIssue {
                category: "Improvement".to_string(),
                severity: IssueSeverity::Critical,
                message: format!(
                    "Calibration weighting not improving residuals ({:.1}% vs {:.1}% target)",
                    improvement, self.thresholds.min_improvement_pct
                ),
                value: improvement,
                threshold: self.thresholds.min_improvement_pct,
            });
            score -= 30.0;
        }

        // Check track length
        if track_length < self.thresholds.min_median_track_frames {
            issues.push(HealthIssue {
                category: "Tracking".to_string(),
                severity: IssueSeverity::Critical,
                message: format!(
                    "Feature tracks too short ({:.1}f vs {:.1}f frames minimum)",
                    track_length, self.thresholds.min_median_track_frames
                ),
                value: track_length,
                threshold: self.thresholds.min_median_track_frames,
            });
            score -= 25.0;
        }

        // Check survival rate
        if survival_5 < self.thresholds.min_survival_rate_5 {
            issues.push(HealthIssue {
                category: "Tracking".to_string(),
                severity: IssueSeverity::Warning,
                message: format!(
                    "Low feature survival rate ({:.1}% vs {:.1}% target)",
                    survival_5, self.thresholds.min_survival_rate_5
                ),
                value: survival_5,
                threshold: self.thresholds.min_survival_rate_5,
            });
            score -= 15.0;
        }

        // Check outlier rate
        if outlier_rate > self.thresholds.max_outlier_rate {
            issues.push(HealthIssue {
                category: "Outliers".to_string(),
                severity: IssueSeverity::Warning,
                message: format!(
                    "High outlier rate ({:.1}% vs {:.1}% maximum)",
                    outlier_rate, self.thresholds.max_outlier_rate
                ),
                value: outlier_rate,
                threshold: self.thresholds.max_outlier_rate,
            });
            score -= 10.0;
        }

        // Check RMS improvement absolute value
        let rms_improvement_px = (metrics.unweighted_residuals.visual_rms
            - metrics.weighted_residuals.visual_rms) as f32;
        if rms_improvement_px < self.thresholds.min_rms_improvement_px && improvement > 5.0 {
            issues.push(HealthIssue {
                category: "RMS".to_string(),
                severity: IssueSeverity::Info,
                message: format!(
                    "RMS improvement below {:.3} px ({:.3} px actual)",
                    self.thresholds.min_rms_improvement_px, rms_improvement_px
                ),
                value: rms_improvement_px,
                threshold: self.thresholds.min_rms_improvement_px,
            });
            score -= 5.0;
        }

        // Determine health status
        let health = if score >= 80.0 {
            CalibrationHealth::Excellent
        } else if score >= 60.0 {
            CalibrationHealth::Good
        } else if score >= 40.0 {
            CalibrationHealth::Marginal
        } else {
            CalibrationHealth::Poor
        };

        self.current_health = health.clone();
        self.history.add(health.clone(), improvement, track_length);

        // Generate recommendations
        let mut recommendations = Vec::new();

        if health.needs_recalibration() {
            recommendations
                .push("⚠️  RECOMMEND RECALIBRATION: Multiple metrics below thresholds".to_string());
            recommendations
                .push("   1. Recapture calibration dataset (slow + fast rotation)".to_string());
            recommendations.push("   2. Ensure sharp lighting and good texture".to_string());
            recommendations.push("   3. Check time synchronization (camera ↔ IMU)".to_string());
        } else if issues.iter().any(|i| i.severity == IssueSeverity::Critical) {
            recommendations
                .push("⚠️  MARGINAL CALIBRATION: Some critical metrics failing".to_string());
            if improvement < self.thresholds.min_improvement_pct {
                recommendations
                    .push("   → Weighting not helping; check T_IC and Δt estimates".to_string());
            }
            if track_length < self.thresholds.min_median_track_frames {
                recommendations.push(
                    "   → Feature tracking unstable; IMU-aided init may be failing".to_string(),
                );
            }
        } else if !issues.is_empty() {
            recommendations.push("ℹ️  GOOD CALIBRATION: Minor issues detected".to_string());
            recommendations.push("   → Monitor metrics but continue operation".to_string());
        } else {
            recommendations
                .push("✓  EXCELLENT CALIBRATION: All metrics within thresholds".to_string());
        }

        // Add trend-based recommendations
        match self.history.get_trend() {
            CalibrationTrend::Degrading => {
                recommendations
                    .push("⚠️  TREND: Calibration degrading over recent samples".to_string());
                recommendations.push("   → Schedule recalibration soon".to_string());
            },
            CalibrationTrend::Declining => {
                recommendations.push("⚠️  TREND: Calibration slowly declining".to_string());
                recommendations
                    .push("   → Monitor next 100 samples for further decline".to_string());
            },
            CalibrationTrend::Stagnant => {
                recommendations.push("ℹ️  TREND: Calibration improvements stagnant".to_string());
                recommendations.push("   → Current calibration may be near limits".to_string());
            },
            _ => {},
        }

        CalibrationHealthAssessment {
            health,
            issues,
            recommendations,
            metrics_summary: MetricsSummary {
                overall_improvement_pct: improvement,
                median_track_length: track_length,
                survival_rate_5: survival_5,
                outlier_rate,
                visual_rms_px: visual_rms,
                imu_rms,
            },
        }
    }

    /// Generate a printable health report
    pub fn health_report(&self, assessment: &CalibrationHealthAssessment) -> String {
        let mut report = String::new();

        report.push_str("\n═════════════════════════════════════════════════════════\n");
        report.push_str("  CALIBRATION HEALTH ASSESSMENT\n");
        report.push_str("═════════════════════════════════════════════════════════\n\n");

        // Health status
        match &assessment.health {
            CalibrationHealth::Excellent => {
                report.push_str("🟢 STATUS: EXCELLENT\n");
            },
            CalibrationHealth::Good => {
                report.push_str("🟡 STATUS: GOOD\n");
            },
            CalibrationHealth::Marginal => {
                report.push_str("🟠 STATUS: MARGINAL\n");
            },
            CalibrationHealth::Poor => {
                report.push_str("🔴 STATUS: POOR\n");
            },
        }

        // Metrics summary
        report.push_str("\nKEY METRICS:\n");
        report.push_str(&format!(
            "  Improvement:        {:.1}% (target: 15%+)\n",
            assessment.metrics_summary.overall_improvement_pct
        ));
        report.push_str(&format!(
            "  Track length:       {:.1} frames (target: 4+ frames)\n",
            assessment.metrics_summary.median_track_length
        ));
        report.push_str(&format!(
            "  Survival >5f:       {:.1}% (target: 40%+)\n",
            assessment.metrics_summary.survival_rate_5
        ));
        report.push_str(&format!(
            "  Outlier rate:       {:.1}% (target: <10%)\n",
            assessment.metrics_summary.outlier_rate
        ));
        report.push_str(&format!(
            "  Visual RMS:         {:.4} px\n",
            assessment.metrics_summary.visual_rms_px
        ));
        report.push_str(&format!(
            "  IMU RMS:            {:.6} rad/s\n",
            assessment.metrics_summary.imu_rms
        ));

        // Issues
        if !assessment.issues.is_empty() {
            report.push_str("\nISSUES DETECTED:\n");
            for issue in &assessment.issues {
                let icon = match issue.severity {
                    IssueSeverity::Info => "ℹ️ ",
                    IssueSeverity::Warning => "⚠️ ",
                    IssueSeverity::Critical => "❌",
                };
                report.push_str(&format!("  {}{}\n", icon, issue.message));
            }
        }

        // Recommendations
        if !assessment.recommendations.is_empty() {
            report.push_str("\nRECOMMENDATIONS:\n");
            for rec in &assessment.recommendations {
                report.push_str(&format!("  {}\n", rec));
            }
        }

        report.push_str("\n═════════════════════════════════════════════════════════\n\n");
        report
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::evaluation::{TrackSurvivalStats, WeightedResidualStats};

    fn create_test_metrics(improvement_pct: f32, track_length: f32) -> CalibrationAwareMetrics {
        use crate::evaluation::{
            CalibrationAwareMetrics, DistanceBinnedImprovement, SpeedBinnedImprovement,
        };

        CalibrationAwareMetrics {
            weighted_residuals: WeightedResidualStats {
                visual_rms: 0.3,
                imu_rms: 0.001,
                total_rms: 0.25,
                sample_count: 100,
                outlier_rate: 5.0,
                mean_residual: 0.25,
                median_residual: 0.2,
                std_residual: 0.1,
            },
            unweighted_residuals: WeightedResidualStats {
                visual_rms: (0.3 / (1.0 - improvement_pct / 100.0)) as f64,
                imu_rms: 0.001,
                total_rms: (0.25 / (1.0 - improvement_pct / 100.0)) as f64,
                sample_count: 100,
                outlier_rate: 7.0,
                mean_residual: 0.3,
                median_residual: 0.25,
                std_residual: 0.15,
            },
            track_survival: TrackSurvivalStats {
                median_track_length: track_length,
                mean_track_length: track_length + 1.0,
                max_track_length: (track_length * 3.0) as usize,
                survival_rate_5: 50.0,
                survival_rate_10: 25.0,
                total_features: 100,
            },
            distance_bin_improvements: DistanceBinnedImprovement::new(),
            speed_bin_improvements: SpeedBinnedImprovement::new(),
        }
    }

    #[test]
    fn test_excellent_calibration() {
        let mut monitor = CalibrationMonitor::new(CalibrationHealthThresholds::default());
        let metrics = create_test_metrics(50.0, 10.0); // 50% improvement, 10 frame tracks

        let assessment = monitor.assess(&metrics);
        assert_eq!(assessment.health, CalibrationHealth::Excellent);
        assert!(assessment.issues.is_empty());
    }

    #[test]
    fn test_poor_calibration() {
        let mut monitor = CalibrationMonitor::new(CalibrationHealthThresholds::default());
        let metrics = create_test_metrics(2.0, 1.0); // 2% improvement, 1 frame tracks

        let assessment = monitor.assess(&metrics);
        assert!(
            assessment.health == CalibrationHealth::Marginal
                || assessment.health == CalibrationHealth::Poor
        );
        assert!(!assessment.issues.is_empty());
    }

    #[test]
    fn test_health_report_generation() {
        let mut monitor = CalibrationMonitor::new(CalibrationHealthThresholds::default());
        let metrics = create_test_metrics(30.0, 6.0);

        let assessment = monitor.assess(&metrics);
        let report = monitor.health_report(&assessment);

        assert!(report.contains("CALIBRATION HEALTH ASSESSMENT"));
        assert!(report.contains("KEY METRICS"));
    }
}
