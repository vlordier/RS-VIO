//! Feature tracking module: detection, matching, and optical flow.

pub mod async_detector;
#[allow(clippy::module_inception)] // Re-exported as crate::feature_tracker::*
pub mod orb_detector;
#[allow(clippy::module_inception)] // Re-exported as crate::feature_tracker::*
pub mod feature_tracker;
pub mod image_utilities;
pub mod patch;
pub mod stereo_matcher;

pub use async_detector::{AsyncDetectorConfig, AsyncFeatureDetector, DetectedFeature};
pub use orb_detector::{
    hamming_distance, OrbDetectorConfig, OrbFeature, OrbFeatureDetector,
};
pub use feature_tracker::*;
pub use patch::Pattern52;
pub use stereo_matcher::{StereoMatch, StereoMatcher, StereoMatcherConfig};
