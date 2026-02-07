//! Visual feature detection, tracking, and stereo matching.
//!
//! Provides patch-based KLT tracking, async detection pipelines,
//! and stereo correspondence for VIO front-end processing.

pub mod async_detector;
#[allow(clippy::module_inception)] // Re-exported as crate::feature_tracker::*
pub mod enhanced_detector;
#[allow(clippy::module_inception)] // Re-exported as crate::feature_tracker::*
pub mod feature_tracker;
pub mod image_utilities;
pub mod patch;
pub mod stereo_matcher;

pub use async_detector::{AsyncDetectorConfig, AsyncFeatureDetector, DetectedFeature};
pub use enhanced_detector::{
    hamming_distance, EnhancedDetectorConfig, EnhancedFeature, EnhancedFeatureDetector,
};
pub use feature_tracker::*;
pub use patch::Pattern52;
pub use stereo_matcher::{StereoMatch, StereoMatcher, StereoMatcherConfig};
