/// Vision processing module for stereo super-resolution and feature refinement.
///
/// This module implements state-of-the-art real-time stereo vision processing techniques
/// integrated with IMU confidence signals for adaptive subpixel refinement.

pub mod stereo_super_resolution;
pub mod motion_aware_super_resolution;
pub mod motion_aware_depth_optimization;
pub mod adaptive_fusion_algorithm;

// Public exports
pub use stereo_super_resolution::{
    StereoSuperResolver, StereoSuperResolutionConfig, SubpixelRefinement, AccumulationStats,
};
pub use motion_aware_super_resolution::{
    MotionAwareSuperResolver, MotionState, MotionType, SubpixelRefinement as MotionAwareRefinement,
};
pub use motion_aware_depth_optimization::{
    MotionAwareDepthOptimizer, MotionConstrainedDepth, TriangulationConstraints,
};
pub use adaptive_fusion_algorithm::{
    OptimizedFusionAlgorithm, AdaptiveFusionConfig,
};
