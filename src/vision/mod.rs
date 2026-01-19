/// Vision processing module for stereo super-resolution and feature refinement.
///
/// This module implements state-of-the-art real-time stereo vision processing techniques
/// integrated with IMU confidence signals for adaptive subpixel refinement.

pub mod stereo_super_resolution;

// Public exports
pub use stereo_super_resolution::{
    StereoSuperResolver, StereoSuperResolutionConfig, SubpixelRefinement, AccumulationStats,
};
