pub mod adaptive_fusion_algorithm;
pub mod imu_aided_tracking;
pub mod motion_aware_depth_optimization;
pub mod motion_aware_super_resolution;
pub mod rolling_shutter_correction;
/// Vision processing module for stereo super-resolution and feature refinement.
///
/// This module implements state-of-the-art real-time stereo vision processing techniques
/// integrated with IMU confidence signals for adaptive subpixel refinement.
pub mod stereo_super_resolution;
pub mod subpixel_disparity;
pub mod temporal_super_resolution;
pub mod visualization;

// Public exports
pub use adaptive_fusion_algorithm::{AdaptiveFusionConfig, OptimizedFusionAlgorithm};
pub use imu_aided_tracking::{IMUAidedTracker, IMUMeasurement, TrackedFeature, TrackingStatistics};
pub use motion_aware_depth_optimization::{
    MotionAwareDepthOptimizer, MotionConstrainedDepth, TriangulationConstraints,
};
pub use motion_aware_super_resolution::{
    MotionAwareSuperResolver, MotionState, MotionType, SubpixelRefinement as MotionAwareRefinement,
};
pub use rolling_shutter_correction::{RollingShutterBundleAdjustment, RollingShutterFramePose};
pub use stereo_super_resolution::{
    AccumulationStats, StereoSuperResolutionConfig, StereoSuperResolver, SubpixelRefinement,
};
pub use subpixel_disparity::{
    ImagePyramid, PatchMatchingConfig, SubPixelDisparityRefiner, SubPixelDisparityResult,
};
pub use temporal_super_resolution::{
    AccumulationQuality, TemporalSRStats, TemporalSuperResolution, TemporalSuperResolutionConfig,
};
pub use visualization::{
    generate_plot_script, DisparityComparison, DisparityStats, RollingShutterComparison,
    RollingShutterStats, TrackingComparison, TrackingStats,
};
