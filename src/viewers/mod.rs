//! # Visualization Module
//!
//! Real-time 3D visualization of VIO results.
//!
//! ## Overview
//!
//! Provides visualization tools for monitoring VIO pipeline output:
//! - Camera trajectories
//! - 3D map points
//! - Camera frustums
//! - Feature tracking (in-progress)
//!
//! ## Components
//!
//! - [`Viewer`] - Visualization trait
//! - [`RerunViewer`] - Rerun.io backend
//! - Color mapping for consistent feature visualization
//!
//! ## Visualization Backend
//!
//! Currently uses [Rerun.io](https://www.rerun.io/):
//! - Real-time 3D visualization
//! - Low-latency streaming
//! - Rich annotation support
//! - Cloud-native architecture
//!
//! ## Usage
//!
//! ```rust,ignore
//! use rs_vio::viewers::{create_viewer, VisualizationConfig};
//!
//! let config = VisualizationConfig::default();
//! let mut viewer = create_viewer("rs-vio", config)?;
//!
//! // Log trajectories, points, etc.
//! viewer.log_pose(&[0.0; 7], "world/camera", 0)?;
//! viewer.log_points(&[[1.0, 2.0, 3.0]], "world/points");
//! ```
//!
//! ## Feature Color Mapping
//!
//! Features are assigned deterministic colors based on ID:
//! - Consistent across frames for tracking visualization
//! - Hash-based assignment for visual distinctiveness
//! - Thread-safe via mutex protection
//!
//! ## Performance
//!
//! - Minimal overhead (~1-2ms per frame)
//! - Asynchronous transmission to Rerun
//! - Can be disabled for production use
//!
//! ## See Also
//! - [`crate::estimator::Estimator`] - Main VIO pipeline
//! - [`crate::feature_tracker::StereoPatchTracker`] - Feature tracking

pub mod rerun;
pub mod viewer;
pub mod screenshot_helper;

pub use rerun::{create_viewer, RerunViewer};
pub use viewer::Viewer;
pub use screenshot_helper::{ScreenshotId, ScreenshotMetadata, ScreenshotResult, ViewScreenshotRequest, ScreenshotBatch};

use std::collections::HashMap;
use std::sync::Mutex;

/// Global color map for feature IDs
///
/// Uses a deterministic hash-based color assignment to ensure consistent colors
/// across frames and visualization sessions.
static FEATURE_COLOR_MAP: Mutex<Option<HashMap<usize, [u8; 3]>>> = Mutex::new(None);

/// Get or assign a color for a given feature ID
///
/// Colors are deterministically assigned based on the feature ID using a hash function,
/// ensuring the same feature ID always produces the same color across visualizations.
/// This is essential for tracking visualization where color consistency helps identify
/// features across time.
///
/// # Arguments
/// * `feature_id` - Unique identifier for the feature
///
/// # Returns
/// RGB color as `[u8; 3]` with minimum brightness of 50 for visibility
pub fn get_feature_color(feature_id: usize) -> [u8; 3] {
    let mut map = match FEATURE_COLOR_MAP.lock() {
        Ok(m) => m,
        Err(poisoned) => {
            log::warn!("[Viewer] Mutex was poisoned, recovering");
            poisoned.into_inner()
        },
    };

    if map.is_none() {
        *map = Some(HashMap::new());
    }

    let map = match map.as_mut() {
        Some(m) => m,
        None => {
            // This should never happen due to the check above, but handle it safely
            log::error!("[Viewer] Color map is unexpectedly None");
            return [128, 128, 128]; // Return gray as fallback
        },
    };

    // Check if color already assigned
    if let Some(&color) = map.get(&feature_id) {
        return color;
    }

    // Generate a deterministic color based on feature ID using a hash
    // This ensures the same feature ID always gets the same color
    let hash = feature_id as u64;

    // Use a simple hash function to generate RGB values
    // This creates visually distinct colors
    let r = ((hash * 2654435761) % 256) as u8;
    let g = ((hash * 2246822507) % 256) as u8;
    let b = ((hash * 3266489917) % 256) as u8;

    // Ensure minimum brightness for visibility
    let color = [r.max(50), g.max(50), b.max(50)];

    map.insert(feature_id, color);
    color
}
