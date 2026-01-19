/// Evaluation results aggregation and comparison

use crate::evaluation::{
    TrajectoryMetrics, DepthMetrics, FeatureMetrics, RobustnessMetrics,
};

/// Configuration identifier
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Configuration {
    Baseline,           // No IMU filtering, no super-resolution
    ImuOnly,           // IMU filtering only
    SuperResOnly,      // Super-resolution only
    FullFusion,        // Both IMU filtering and super-resolution
}

impl Configuration {
    pub fn name(&self) -> &'static str {
        match self {
            Configuration::Baseline => "Baseline (no filters)",
            Configuration::ImuOnly => "IMU Filtering Only",
            Configuration::SuperResOnly => "Super-Resolution Only",
            Configuration::FullFusion => "Full Fusion (IMU + Super-Res)",
        }
    }
}

/// Complete evaluation results for a single configuration
#[derive(Clone, Debug)]
pub struct ConfigurationResults {
    pub config: Configuration,
    pub trajectory: Option<TrajectoryMetrics>,
    pub depth: Option<DepthMetrics>,
    pub features: Option<FeatureMetrics>,
    pub robustness: Option<RobustnessMetrics>,
    pub computation_time_ms: f64,  // Per-frame average
}

impl ConfigurationResults {
    pub fn new(config: Configuration) -> Self {
        Self {
            config,
            trajectory: None,
            depth: None,
            features: None,
            robustness: None,
            computation_time_ms: 0.0,
        }
    }
    
    /// Format results as human-readable string
    pub fn to_string(&self) -> String {
        let mut output = String::new();
        output.push_str(&format!("\n{}\n", self.config.name()));
        output.push_str(&"=".repeat(60));
        output.push('\n');
        
        if let Some(ref traj) = self.trajectory {
            output.push_str(&format!("Trajectory Accuracy:\n"));
            output.push_str(&format!("  ATE (Absolute Trajectory Error):\n"));
            output.push_str(&format!("    Mean:  {:.6} m\n", traj.ate_mean));
            output.push_str(&format!("    Std:   {:.6} m\n", traj.ate_std));
            output.push_str(&format!("    RMSE:  {:.6} m\n", traj.ate_rmse));
            output.push_str(&format!("  RPE (Relative Pose Error):\n"));
            output.push_str(&format!("    Translation Mean: {:.6} m\n", traj.rpe_translation_mean));
            output.push_str(&format!("    Translation Std:  {:.6} m\n", traj.rpe_translation_std));
            output.push_str(&format!("    Rotation Mean:    {:.4}°\n", traj.rpe_rotation_mean));
            output.push_str(&format!("    Rotation Std:     {:.4}°\n", traj.rpe_rotation_std));
        }
        
        if let Some(ref depth) = self.depth {
            output.push_str(&format!("\nDepth Accuracy:\n"));
            output.push_str(&format!("  RMSE:                {:.6} m\n", depth.rmse));
            output.push_str(&format!("  MAE:                 {:.6} m\n", depth.mae));
            output.push_str(&format!("  Outlier Percentage:  {:.2}%\n", depth.outlier_percentage));
            output.push_str(&format!("  Disparity RMSE:      {:.4} px\n", depth.disparity_rmse));
            output.push_str(&format!("  Points Evaluated:    {}\n", depth.point_count));
        }
        
        if let Some(ref features) = self.features {
            output.push_str(&format!("\nFeature Quality:\n"));
            output.push_str(&format!("  Reprojection Error:\n"));
            output.push_str(&format!("    Mean: {:.4} px\n", features.reprojection_error_mean));
            output.push_str(&format!("    Std:  {:.4} px\n", features.reprojection_error_std));
            output.push_str(&format!("    Max:  {:.4} px\n", features.reprojection_error_max));
            output.push_str(&format!("  Inlier Ratio:        {:.2}%\n", features.match_inlier_ratio * 100.0));
            output.push_str(&format!("  Features Tracked:    {}\n", features.feature_count));
        }
        
        if let Some(ref robust) = self.robustness {
            output.push_str(&format!("\nRobustness:\n"));
            output.push_str(&format!("  Failure Rate:        {:.2}%\n", robust.failure_rate * 100.0));
            output.push_str(&format!("  Failure Count:       {}\n", robust.failure_count));
            output.push_str(&format!("  Recovery Count:      {}\n", robust.recovery_count));
            output.push_str(&format!("  Avg Failure Duration: {:.1} frames\n", robust.avg_failure_duration));
            output.push_str(&format!("  Max Failure Duration: {} frames\n", robust.max_failure_duration));
            output.push_str(&format!("  Max Pose Jump:       {:.6} m\n", robust.max_pose_jump));
        }
        
        output.push_str(&format!("\nPerformance:\n"));
        output.push_str(&format!("  Per-Frame Time:      {:.3} ms\n", self.computation_time_ms));
        
        output
    }
}

/// Comparison results between configurations
pub struct ComparisonResults {
    pub configurations: Vec<ConfigurationResults>,
}

impl ComparisonResults {
    pub fn new(configs: Vec<ConfigurationResults>) -> Self {
        Self {
            configurations: configs,
        }
    }
    
    /// Generate comparison summary
    pub fn summary(&self) -> String {
        let mut output = String::new();
        output.push_str("\n");
        output.push_str(&"=".repeat(80));
        output.push_str("\nFUSION CONFIGURATION EVALUATION SUMMARY\n");
        output.push_str(&"=".repeat(80));
        output.push('\n');
        
        // Find baseline for comparison
        let baseline = self.configurations.iter()
            .find(|c| c.config == Configuration::Baseline);
        
        // Trajectory comparison
        if let Some(baseline) = baseline {
            if let Some(ref baseline_traj) = baseline.trajectory {
                output.push_str("\n## TRAJECTORY ACCURACY COMPARISON ##\n\n");
                output.push_str("Configuration                 | ATE RMSE | Improvement | RPE Trans |\n");
                output.push_str(&"-".repeat(70));
                output.push('\n');
                
                for config in &self.configurations {
                    if let Some(ref traj) = config.trajectory {
                        let improvement = (1.0 - traj.ate_rmse / baseline_traj.ate_rmse) * 100.0;
                        let status = if improvement > 0.0 { "✓" } else { "✗" };
                        output.push_str(&format!(
                            "{:<28} | {:.4} m | {}{:+.1}% | {:.4} m\n",
                            config.config.name(),
                            traj.ate_rmse,
                            status,
                            improvement,
                            traj.rpe_translation_mean
                        ));
                    }
                }
            }
        }
        
        // Depth comparison
        if let Some(baseline) = baseline {
            if let Some(ref baseline_depth) = baseline.depth {
                output.push_str("\n## DEPTH ACCURACY COMPARISON ##\n\n");
                output.push_str("Configuration                 | Depth RMSE | Improvement |\n");
                output.push_str(&"-".repeat(55));
                output.push('\n');
                
                for config in &self.configurations {
                    if let Some(ref depth) = config.depth {
                        let improvement = (1.0 - depth.rmse / baseline_depth.rmse) * 100.0;
                        let status = if improvement > 0.0 { "✓" } else { "✗" };
                        output.push_str(&format!(
                            "{:<28} | {:.4} m | {}{:+.1}%\n",
                            config.config.name(),
                            depth.rmse,
                            status,
                            improvement
                        ));
                    }
                }
            }
        }
        
        // Performance comparison
        output.push_str("\n## PERFORMANCE COMPARISON ##\n\n");
        output.push_str("Configuration                 | Time (ms) | Overhead |\n");
        output.push_str(&"-".repeat(50));
        output.push('\n');
        
        if let Some(baseline) = baseline {
            let baseline_time = baseline.computation_time_ms;
            for config in &self.configurations {
                let overhead = ((config.computation_time_ms / baseline_time - 1.0) * 100.0).max(0.0);
                output.push_str(&format!(
                    "{:<28} | {:.3} ms | {:.1}%\n",
                    config.config.name(),
                    config.computation_time_ms,
                    overhead
                ));
            }
        }
        
        // Recommendations
        output.push_str("\n## RECOMMENDATIONS ##\n\n");
        
        let mut best_accuracy = ("", f64::INFINITY);
        let mut best_speed = ("", f64::INFINITY);
        
        for config in &self.configurations {
            if let Some(ref traj) = config.trajectory {
                if traj.ate_rmse < best_accuracy.1 {
                    best_accuracy = (config.config.name(), traj.ate_rmse);
                }
            }
            if config.computation_time_ms < best_speed.1 {
                best_speed = (config.config.name(), config.computation_time_ms);
            }
        }
        
        output.push_str(&format!("Best Accuracy:  {}\n", best_accuracy.0));
        output.push_str(&format!("Best Speed:     {}\n", best_speed.0));
        output.push_str(&format!("\n→ Recommended: Check accuracy/speed trade-off for your use case\n"));
        
        output
    }
}

pub fn compare_configurations(results: Vec<ConfigurationResults>) -> ComparisonResults {
    ComparisonResults::new(results)
}
