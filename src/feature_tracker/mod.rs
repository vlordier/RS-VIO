//! Feature detection, tracking, and patch-based stereo matching.
//!
//! The feature tracker detects FAST corners, extracts them in a grid pattern,
//! and tracks them across frames using normalized cross-correlation (NCC)
//! with sub-pixel refinement via inverse compositional Lucas–Kanade.
//!
//! ## Components
//!
//! - [`StereoPatchTracker`](feature_tracker::StereoPatchTracker) — main tracker, detects and tracks features
//! - [`Pattern52`] — 5×5 pattern used for patch matching
//! - Grid-based feature detection to ensure uniform coverage

pub mod feature_tracker;
pub mod image_utilities;
pub mod patch;

pub use feature_tracker::*;
pub use patch::Pattern52;
