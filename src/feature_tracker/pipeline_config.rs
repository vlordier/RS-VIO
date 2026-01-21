//! Comprehensive VIO pipeline configuration system
//!
//! Provides flexible, platform-aware configuration for the entire feature tracking pipeline.
//! Supports CPU-only, GPU-accelerated, and hard-realtime scenarios with tunable parameters.

use serde::{Deserialize, Serialize};

use super::{StabilizerConfig, TrackFirstConfig};

/// Target platform for VIO pipeline
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TargetPlatform {
    /// CPU-only (e.g., Raspberry Pi 5, small ARM)
    CpuOnly,
    /// GPU-enabled (e.g., Jetson Nano/Xavier/Orin)
    GpuEnabled,
    /// Hard realtime constraints (minimal latency)
    HardRealtime,
    /// Custom configuration
    Custom,
}

/// Feature detection backend
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DetectionBackend {
    /// Shi-Tomasi (GFTT) - fast, CPU-friendly
    ShiTomasi,
    /// FAST corner detector
    Fast,
    /// AGAST (adaptive FAST)
    Agast,
    /// SuperPoint (learned, requires GPU)
    SuperPoint,
    /// DISK/R2D2 (learned, requires GPU)
    Disk,
}

/// Feature descriptor type for matching
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DescriptorType {
    /// No descriptors (tracking-only)
    None,
    /// ORB descriptors (fast, CPU-friendly)
    Orb,
    /// BRIEF descriptors
    Brief,
    /// AKAZE binary descriptors
    Akaze,
    /// SuperPoint learned descriptors (GPU)
    SuperPoint,
    /// LightGlue learned matching (GPU)
    LightGlue,
}

/// Multi-frame fusion strategy
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FusionStrategy {
    /// No fusion (single frame)
    None,
    /// Rotation-only shift-and-add
    RotationOnly,
    /// SE(3) warp with planar assumption
    PlanarSe3,
    /// Depth-aware fusion (two-pass)
    DepthAware,
    /// Patch-level fusion around tracked points
    PatchLevel,
    /// Adaptive (choose based on scene)
    Adaptive,
}

/// Tracking strategy
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TrackingStrategy {
    /// Detect every frame (naive)
    DetectEveryFrame,
    /// Track-first, detect-to-fill (recommended)
    TrackFirst,
    /// Semi-direct (SVO-style)
    SemiDirect,
}

/// Performance budget configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceBudget {
    /// Target FPS (frames per second)
    pub target_fps: f32,
    /// Maximum latency per frame (ms)
    pub max_latency_ms: f32,
    /// Maximum CPU usage (0.0-1.0)
    pub max_cpu_usage: f32,
    /// Maximum memory usage (MB)
    pub max_memory_mb: usize,
    /// Enable GPU if available
    pub enable_gpu: bool,
}

impl Default for PerformanceBudget {
    fn default() -> Self {
        Self {
            target_fps: 30.0,
            max_latency_ms: 33.0, // ~30 FPS
            max_cpu_usage: 0.8,
            max_memory_mb: 512,
            enable_gpu: false,
        }
    }
}

/// Multi-frame fusion configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FusionConfig {
    /// Fusion strategy to use
    pub strategy: FusionStrategy,
    /// Number of frames to fuse (3-7 typical)
    pub num_frames: usize,
    /// Frame stabilizer configuration (if using rotation-only)
    pub stabilizer: Option<StabilizerConfig>,
    /// Depth-aware fusion parameters
    pub depth_aware_params: Option<DepthAwareFusionParams>,
}

impl Default for FusionConfig {
    fn default() -> Self {
        Self {
            strategy: FusionStrategy::RotationOnly,
            num_frames: 5,
            stabilizer: Some(StabilizerConfig::default()),
            depth_aware_params: None,
        }
    }
}

/// Parameters for depth-aware fusion
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DepthAwareFusionParams {
    /// Run coarse stereo for depth estimates
    pub enable_coarse_stereo: bool,
    /// Minimum depth (meters)
    pub min_depth: f32,
    /// Maximum depth (meters)
    pub max_depth: f32,
    /// Occlusion handling threshold
    pub occlusion_threshold: f32,
}

impl Default for DepthAwareFusionParams {
    fn default() -> Self {
        Self {
            enable_coarse_stereo: true,
            min_depth: 0.3,
            max_depth: 100.0,
            occlusion_threshold: 0.1,
        }
    }
}

/// Feature detection configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionConfig {
    /// Detection backend
    pub backend: DetectionBackend,
    /// Tracking strategy
    pub tracking_strategy: TrackingStrategy,
    /// Track-first detector config (if using track-first)
    pub track_first: Option<TrackFirstConfig>,
    /// Minimum feature quality (0.0-1.0)
    pub min_quality: f32,
    /// Use pyramidal detection
    pub use_pyramid: bool,
    /// Number of pyramid levels
    pub pyramid_levels: usize,
    /// Enforce spatial distribution (ANMS)
    pub enforce_spatial_distribution: bool,
    /// Grid cell size for spatial distribution
    pub grid_cell_size: u32,
}

impl Default for DetectionConfig {
    fn default() -> Self {
        Self {
            backend: DetectionBackend::ShiTomasi,
            tracking_strategy: TrackingStrategy::TrackFirst,
            track_first: Some(TrackFirstConfig::default()),
            min_quality: 0.01,
            use_pyramid: true,
            pyramid_levels: 3,
            enforce_spatial_distribution: true,
            grid_cell_size: 32,
        }
    }
}

/// Feature matching configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MatchingConfig {
    /// Descriptor type
    pub descriptor_type: DescriptorType,
    /// Run matching on keyframes only
    pub keyframes_only: bool,
    /// Maximum descriptor distance for match
    pub max_descriptor_distance: f32,
    /// Use ratio test (Lowe's ratio)
    pub use_ratio_test: bool,
    /// Ratio test threshold
    pub ratio_test_threshold: f32,
}

impl Default for MatchingConfig {
    fn default() -> Self {
        Self {
            descriptor_type: DescriptorType::None,
            keyframes_only: true,
            max_descriptor_distance: 50.0,
            use_ratio_test: true,
            ratio_test_threshold: 0.8,
        }
    }
}

/// Stereo configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StereoConfig {
    /// Baseline between cameras (meters)
    pub baseline: f32,
    /// Maximum disparity search range
    pub max_disparity: i32,
    /// Use subpixel refinement
    pub subpixel_refinement: bool,
    /// Cost aggregation window size
    pub cost_window_size: usize,
    /// Uniqueness ratio (0.0-1.0)
    pub uniqueness_ratio: f32,
    /// Texture threshold
    pub texture_threshold: f32,
}

impl Default for StereoConfig {
    fn default() -> Self {
        Self {
            baseline: 0.11, // Typical stereo rig
            max_disparity: 128,
            subpixel_refinement: true,
            cost_window_size: 5,
            uniqueness_ratio: 0.15,
            texture_threshold: 10.0,
        }
    }
}

/// Comprehensive VIO pipeline configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VIOPipelineConfig {
    /// Target platform
    pub platform: TargetPlatform,
    /// Performance budget
    pub performance: PerformanceBudget,
    /// Multi-frame fusion configuration
    pub fusion: FusionConfig,
    /// Feature detection configuration
    pub detection: DetectionConfig,
    /// Feature matching configuration
    pub matching: MatchingConfig,
    /// Stereo configuration
    pub stereo: StereoConfig,
    /// Enable IMU integration
    pub enable_imu: bool,
    /// Enable loop closure
    pub enable_loop_closure: bool,
    /// Enable bundle adjustment
    pub enable_bundle_adjustment: bool,
}

impl Default for VIOPipelineConfig {
    fn default() -> Self {
        Self::cpu_only()
    }
}

impl VIOPipelineConfig {
    /// CPU-only configuration (Raspberry Pi 5 / small ARM)
    pub fn cpu_only() -> Self {
        Self {
            platform: TargetPlatform::CpuOnly,
            performance: PerformanceBudget {
                target_fps: 20.0,
                max_latency_ms: 50.0,
                max_cpu_usage: 0.8,
                max_memory_mb: 256,
                enable_gpu: false,
            },
            fusion: FusionConfig {
                strategy: FusionStrategy::RotationOnly,
                num_frames: 3, // Fewer frames for memory
                stabilizer: Some(StabilizerConfig {
                    enabled: true,
                    buffer_size: 3,
                    accumulation_weight: 0.7,
                    min_rotation_threshold: 0.01,
                }),
                depth_aware_params: None,
            },
            detection: DetectionConfig {
                backend: DetectionBackend::ShiTomasi,
                tracking_strategy: TrackingStrategy::TrackFirst,
                track_first: Some(TrackFirstConfig {
                    min_features: 100,
                    max_features: 200,
                    grid_cell_size: 32,
                    min_features_per_cell: 2,
                    corner_quality_threshold: 0.01,
                    min_feature_distance: 10.0,
                }),
                min_quality: 0.01,
                use_pyramid: true,
                pyramid_levels: 2, // Fewer levels for speed
                enforce_spatial_distribution: true,
                grid_cell_size: 32,
            },
            matching: MatchingConfig {
                descriptor_type: DescriptorType::Orb,
                keyframes_only: true,
                max_descriptor_distance: 50.0,
                use_ratio_test: true,
                ratio_test_threshold: 0.8,
            },
            stereo: StereoConfig::default(),
            enable_imu: true,
            enable_loop_closure: true,
            enable_bundle_adjustment: true,
        }
    }

    /// GPU-enabled configuration (Jetson Nano/Xavier/Orin)
    pub fn gpu_enabled() -> Self {
        Self {
            platform: TargetPlatform::GpuEnabled,
            performance: PerformanceBudget {
                target_fps: 30.0,
                max_latency_ms: 33.0,
                max_cpu_usage: 0.6,
                max_memory_mb: 1024,
                enable_gpu: true,
            },
            fusion: FusionConfig {
                strategy: FusionStrategy::Adaptive, // Can afford more
                num_frames: 5,
                stabilizer: Some(StabilizerConfig {
                    enabled: true,
                    buffer_size: 5,
                    accumulation_weight: 0.7,
                    min_rotation_threshold: 0.01,
                }),
                depth_aware_params: Some(DepthAwareFusionParams::default()),
            },
            detection: DetectionConfig {
                backend: DetectionBackend::ShiTomasi, // Keep CPU for tracking
                tracking_strategy: TrackingStrategy::TrackFirst,
                track_first: Some(TrackFirstConfig {
                    min_features: 150,
                    max_features: 300,
                    grid_cell_size: 32,
                    min_features_per_cell: 2,
                    corner_quality_threshold: 0.01,
                    min_feature_distance: 10.0,
                }),
                min_quality: 0.005, // Can afford better quality
                use_pyramid: true,
                pyramid_levels: 3,
                enforce_spatial_distribution: true,
                grid_cell_size: 32,
            },
            matching: MatchingConfig {
                descriptor_type: DescriptorType::SuperPoint,
                keyframes_only: true,
                max_descriptor_distance: 0.7, // Normalized for learned
                use_ratio_test: true,
                ratio_test_threshold: 0.9,
            },
            stereo: StereoConfig::default(),
            enable_imu: true,
            enable_loop_closure: true,
            enable_bundle_adjustment: true,
        }
    }

    /// Hard realtime configuration (minimal latency)
    pub fn hard_realtime() -> Self {
        Self {
            platform: TargetPlatform::HardRealtime,
            performance: PerformanceBudget {
                target_fps: 60.0,
                max_latency_ms: 16.0,
                max_cpu_usage: 0.7,
                max_memory_mb: 128,
                enable_gpu: false,
            },
            fusion: FusionConfig {
                strategy: FusionStrategy::None, // Too expensive
                num_frames: 1,
                stabilizer: None,
                depth_aware_params: None,
            },
            detection: DetectionConfig {
                backend: DetectionBackend::Fast, // Fastest detector
                tracking_strategy: TrackingStrategy::TrackFirst,
                track_first: Some(TrackFirstConfig {
                    min_features: 80,
                    max_features: 150,
                    grid_cell_size: 48, // Larger cells = fewer features
                    min_features_per_cell: 1,
                    corner_quality_threshold: 0.02, // Lower quality ok
                    min_feature_distance: 15.0,
                }),
                min_quality: 0.02,
                use_pyramid: true,
                pyramid_levels: 2,
                enforce_spatial_distribution: true,
                grid_cell_size: 48,
            },
            matching: MatchingConfig {
                descriptor_type: DescriptorType::None, // No matching
                keyframes_only: true,
                max_descriptor_distance: 50.0,
                use_ratio_test: false,
                ratio_test_threshold: 0.8,
            },
            stereo: StereoConfig {
                max_disparity: 64, // Smaller search range
                subpixel_refinement: false,
                cost_window_size: 3,
                ..Default::default()
            },
            enable_imu: true,
            enable_loop_closure: false,      // Too expensive
            enable_bundle_adjustment: false, // Too expensive
        }
    }

    /// Balanced configuration for typical drone use
    pub fn balanced() -> Self {
        Self {
            platform: TargetPlatform::Custom,
            performance: PerformanceBudget {
                target_fps: 30.0,
                max_latency_ms: 33.0,
                max_cpu_usage: 0.75,
                max_memory_mb: 512,
                enable_gpu: false,
            },
            fusion: FusionConfig {
                strategy: FusionStrategy::RotationOnly,
                num_frames: 5,
                stabilizer: Some(StabilizerConfig::default()),
                depth_aware_params: None,
            },
            detection: DetectionConfig::default(),
            matching: MatchingConfig {
                descriptor_type: DescriptorType::Orb,
                keyframes_only: true,
                max_descriptor_distance: 50.0,
                use_ratio_test: true,
                ratio_test_threshold: 0.8,
            },
            stereo: StereoConfig::default(),
            enable_imu: true,
            enable_loop_closure: true,
            enable_bundle_adjustment: true,
        }
    }

    /// Validate configuration for runtime consistency
    pub fn validate(&self) -> Result<(), String> {
        // Check GPU requirements
        if !self.performance.enable_gpu {
            match self.detection.backend {
                DetectionBackend::SuperPoint | DetectionBackend::Disk => {
                    return Err("SuperPoint/DISK require GPU enabled".to_string());
                },
                _ => {},
            }
            match self.matching.descriptor_type {
                DescriptorType::SuperPoint | DescriptorType::LightGlue => {
                    return Err("SuperPoint/LightGlue descriptors require GPU".to_string());
                },
                _ => {},
            }
        }

        // Check fusion consistency
        if self.fusion.strategy == FusionStrategy::RotationOnly && self.fusion.stabilizer.is_none()
        {
            return Err("Rotation-only fusion requires stabilizer config".to_string());
        }

        if self.fusion.strategy == FusionStrategy::DepthAware
            && self.fusion.depth_aware_params.is_none()
        {
            return Err("Depth-aware fusion requires depth params".to_string());
        }

        // Check tracking consistency
        if self.detection.tracking_strategy == TrackingStrategy::TrackFirst
            && self.detection.track_first.is_none()
        {
            return Err("Track-first strategy requires track_first config".to_string());
        }

        // Check feature count sanity
        if let Some(ref tf_config) = self.detection.track_first {
            if tf_config.min_features > tf_config.max_features {
                return Err("min_features must be <= max_features".to_string());
            }
        }

        // Check performance budget sanity
        if self.performance.target_fps <= 0.0 {
            return Err("target_fps must be positive".to_string());
        }

        if self.performance.max_latency_ms <= 0.0 {
            return Err("max_latency_ms must be positive".to_string());
        }

        Ok(())
    }

    /// Get human-readable configuration summary
    pub fn summary(&self) -> String {
        format!(
            "VIO Pipeline Configuration:\n\
             Platform: {:?}\n\
             Target FPS: {:.1}\n\
             Fusion: {:?} ({} frames)\n\
             Detection: {:?} ({:?})\n\
             Tracking: {:?}\n\
             Descriptors: {:?}\n\
             IMU: {}, Loop: {}, BA: {}",
            self.platform,
            self.performance.target_fps,
            self.fusion.strategy,
            self.fusion.num_frames,
            self.detection.backend,
            self.detection.tracking_strategy,
            self.detection.tracking_strategy,
            self.matching.descriptor_type,
            self.enable_imu,
            self.enable_loop_closure,
            self.enable_bundle_adjustment
        )
    }

    /// Save configuration to TOML file
    pub fn save_toml(&self, path: &std::path::Path) -> std::io::Result<()> {
        let toml_string = toml::to_string_pretty(self).map_err(std::io::Error::other)?;
        std::fs::write(path, toml_string)
    }

    /// Load configuration from TOML file
    pub fn load_toml(path: &std::path::Path) -> std::io::Result<Self> {
        let contents = std::fs::read_to_string(path)?;
        toml::from_str(&contents)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cpu_only_config() {
        let config = VIOPipelineConfig::cpu_only();
        assert!(config.validate().is_ok());
        assert_eq!(config.platform, TargetPlatform::CpuOnly);
        assert!(!config.performance.enable_gpu);
        assert_eq!(config.detection.backend, DetectionBackend::ShiTomasi);
    }

    #[test]
    fn test_gpu_enabled_config() {
        let config = VIOPipelineConfig::gpu_enabled();
        assert!(config.validate().is_ok());
        assert_eq!(config.platform, TargetPlatform::GpuEnabled);
        assert!(config.performance.enable_gpu);
    }

    #[test]
    fn test_hard_realtime_config() {
        let config = VIOPipelineConfig::hard_realtime();
        assert!(config.validate().is_ok());
        assert_eq!(config.platform, TargetPlatform::HardRealtime);
        assert_eq!(config.performance.target_fps, 60.0);
        assert_eq!(config.fusion.strategy, FusionStrategy::None);
    }

    #[test]
    fn test_config_validation_gpu_mismatch() {
        let mut config = VIOPipelineConfig::cpu_only();
        config.detection.backend = DetectionBackend::SuperPoint;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_config_validation_missing_stabilizer() {
        let mut config = VIOPipelineConfig::cpu_only();
        config.fusion.strategy = FusionStrategy::RotationOnly;
        config.fusion.stabilizer = None;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_config_summary() {
        let config = VIOPipelineConfig::balanced();
        let summary = config.summary();
        assert!(summary.contains("VIO Pipeline"));
        assert!(summary.contains("30.0"));
    }
}
