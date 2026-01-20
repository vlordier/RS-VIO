/// Feature tracker module - split from monolithic file for better organization

mod mono_tracker;
mod stereo_tracker;
mod tracking;
mod types;

#[cfg(test)]
mod tests;

// Re-export public API
pub use mono_tracker::PatchTracker;
pub use stereo_tracker::StereoPatchTracker;
pub use tracking::{add_points, track_point_at_level, track_points};
pub use types::{Feature, FeatureQuality};
