/// Runtime configuration for selecting and configuring stereo matching strategies
///
/// This module enables runtime selection of matching strategies without recompilation,
/// while still supporting compile-time selection via feature flags for lean binaries.
use crate::feature_tracker::*;
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Configuration for stereo matching strategy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MatchingStrategyConfig {
    /// Strategy to use
    pub strategy: String,
    /// Strategy-specific parameters
    #[serde(default)]
    pub params: StrategyParams,
}

/// Strategy-specific parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StrategyParams {
    /// For BasicRANSAC: max iterations
    #[serde(default = "default_max_iterations")]
    pub max_iterations: usize,
    /// For BasicRANSAC: inlier threshold (pixels)
    #[serde(default = "default_inlier_threshold")]
    pub inlier_threshold: f32,
    /// For IMUGuided: search window margin
    #[serde(default = "default_search_margin")]
    pub search_margin_px: f32,
    /// For TemporalConsistency: depth change threshold
    #[serde(default = "default_depth_threshold")]
    pub depth_change_threshold: f32,
    /// For HybridOpticalFlow: optical flow threshold
    #[serde(default = "default_flow_threshold")]
    pub flow_magnitude_threshold: f32,
}

fn default_max_iterations() -> usize {
    1000
}
fn default_inlier_threshold() -> f32 {
    1.0
}
fn default_search_margin() -> f32 {
    8.0
}
fn default_depth_threshold() -> f32 {
    0.2
}
fn default_flow_threshold() -> f32 {
    2.0
}

impl Default for StrategyParams {
    fn default() -> Self {
        Self {
            max_iterations: default_max_iterations(),
            inlier_threshold: default_inlier_threshold(),
            search_margin_px: default_search_margin(),
            depth_change_threshold: default_depth_threshold(),
            flow_magnitude_threshold: default_flow_threshold(),
        }
    }
}

impl Default for MatchingStrategyConfig {
    fn default() -> Self {
        Self {
            strategy: "BasicRANSAC".to_string(),
            params: StrategyParams::default(),
        }
    }
}

impl MatchingStrategyConfig {
    /// Load configuration from YAML file
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, Box<dyn std::error::Error>> {
        let contents = std::fs::read_to_string(path)?;
        let config: MatchingStrategyConfig = serde_yaml::from_str(&contents)?;
        Ok(config)
    }

    /// Create strategy instance from configuration
    pub fn create_strategy(&self) -> Result<Box<dyn StereoMatchingStrategy>, String> {
        match self.strategy.as_str() {
            #[cfg(feature = "matching-basic-ransac")]
            "BasicRANSAC" => {
                let cfg = BasicRANSACConfig {
                    max_iterations: self.params.max_iterations,
                    inlier_threshold: self.params.inlier_threshold,
                    min_inliers: 20,
                    confidence: 0.99,
                };
                use crate::feature_tracker::BasicRANSACStrategy;
                Ok(Box::new(BasicRANSACStrategy::new(cfg)))
            },
            #[cfg(feature = "matching-imu-guided")]
            "IMUGuided" => {
                use crate::feature_tracker::IMUGuidedStrategy;
                let cfg = BasicRANSACConfig {
                    max_iterations: self.params.max_iterations,
                    inlier_threshold: self.params.inlier_threshold,
                    min_inliers: 20,
                    confidence: 0.99,
                };
                Ok(Box::new(IMUGuidedStrategy::new(
                    self.params.search_margin_px,
                    cfg,
                )))
            },
            #[cfg(feature = "matching-temporal")]
            "TemporalConsistency" => {
                use crate::feature_tracker::TemporalConsistencyStrategy;
                Ok(Box::new(TemporalConsistencyStrategy {
                    depth_change_threshold: self.params.depth_change_threshold,
                    temporal_weight: 0.9,
                }))
            },
            #[cfg(feature = "matching-hybrid-of")]
            "HybridOpticalFlow" => {
                use crate::feature_tracker::HybridOpticalFlowStrategy;
                let cfg = BasicRANSACConfig {
                    max_iterations: 500,
                    inlier_threshold: self.params.inlier_threshold,
                    min_inliers: 20,
                    confidence: 0.99,
                };
                let mut strategy = HybridOpticalFlowStrategy::default();
                strategy.flow_magnitude_threshold = self.params.flow_magnitude_threshold;
                strategy.ransac_config = cfg;
                Ok(Box::new(strategy))
            },
            _ => Err(format!(
                "Unknown strategy: '{}'. Available strategies: {}",
                self.strategy,
                Self::available_strategies().join(", ")
            )),
        }
    }

    /// Get list of available strategies (based on compiled features)
    pub fn available_strategies() -> Vec<&'static str> {
        let mut strategies = Vec::new();
        #[cfg(feature = "matching-basic-ransac")]
        strategies.push("BasicRANSAC");
        #[cfg(feature = "matching-imu-guided")]
        strategies.push("IMUGuided");
        #[cfg(feature = "matching-temporal")]
        strategies.push("TemporalConsistency");
        #[cfg(feature = "matching-hybrid-of")]
        strategies.push("HybridOpticalFlow");
        strategies
    }

    /// Print available strategies and their descriptions
    pub fn print_available() {
        println!("\nAvailable Stereo Matching Strategies:");
        println!("=====================================\n");

        #[cfg(feature = "matching-basic-ransac")]
        {
            println!("1. BasicRANSAC");
            println!("   Description: Standard 8-point RANSAC with configurable iterations");
            println!("   Use case: Baseline approach, good for general-purpose SLAM");
            println!("   Speed: ~0.1-0.2ms per 100 matches");
            println!("   Config example:");
            println!("     strategy: BasicRANSAC");
            println!("     params:");
            println!("       max_iterations: 1000");
            println!("       inlier_threshold: 1.0");
            println!();
        }

        #[cfg(feature = "matching-imu-guided")]
        {
            println!("2. IMUGuided");
            println!("   Description: Uses IMU velocity to predict and restrict search window");
            println!("   Use case: Drones and vehicles with predictable motion");
            println!("   Speed: ~8x faster stereo matching");
            println!("   Config example:");
            println!("     strategy: IMUGuided");
            println!("     params:");
            println!("       search_margin_px: 8.0");
            println!("       max_iterations: 500");
            println!();
        }

        #[cfg(feature = "matching-temporal")]
        {
            println!("3. TemporalConsistency");
            println!("   Description: Frame-to-frame depth coherence for outlier rejection");
            println!("   Use case: Real-time applications with strict latency constraints");
            println!("   Speed: ~100x faster than RANSAC");
            println!("   Config example:");
            println!("     strategy: TemporalConsistency");
            println!("     params:");
            println!("       depth_change_threshold: 0.2");
            println!();
        }

        #[cfg(feature = "matching-hybrid-of")]
        {
            println!("4. HybridOpticalFlow");
            println!("   Description: Coarse optical flow + selective stereo in active regions");
            println!("   Use case: Low-power systems or very high frame rates");
            println!("   Speed: ~30% faster than BasicRANSAC");
            println!("   Config example:");
            println!("     strategy: HybridOpticalFlow");
            println!("     params:");
            println!("       flow_magnitude_threshold: 2.0");
            println!();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_defaults() {
        let cfg = MatchingStrategyConfig::default();
        assert_eq!(cfg.strategy, "BasicRANSAC");
        // Defaults from StrategyParams::default()
        let params = StrategyParams::default();
        assert_eq!(params.max_iterations, 1000);
        assert_eq!(params.inlier_threshold, 1.0);
    }

    #[test]
    fn test_available_strategies() {
        let strategies = MatchingStrategyConfig::available_strategies();
        // At least BasicRANSAC should be available (default feature)
        assert!(!strategies.is_empty());
    }
}
