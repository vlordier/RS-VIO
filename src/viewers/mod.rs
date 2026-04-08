//! Visualization viewers for VIO data.
//!
//! Provides the [`Viewer`] trait for logging poses, images, and map points
//! to external visualization tools (e.g., Rerun).

pub mod rerun;
pub mod viewer;

pub use rerun::{create_viewer, RerunViewer};
pub use viewer::Viewer;

use std::collections::HashMap;
use std::sync::Mutex;

/// Global color map for feature IDs.
/// Uses a deterministic hash-based color assignment to ensure consistent colors.
static FEATURE_COLOR_MAP: Mutex<Option<HashMap<usize, [u8; 3]>>> = Mutex::new(None);

/// Get or assign a color for a given feature ID.
///
/// Colors are deterministically assigned based on the feature ID using a hash,
/// ensuring the same feature always gets the same color across frames.
pub fn get_feature_color(feature_id: usize) -> [u8; 3] {
    let mut map = FEATURE_COLOR_MAP.lock().unwrap_or_else(|e| e.into_inner());

    if map.is_none() {
        *map = Some(HashMap::new());
    }

    let map = map.as_mut().expect("color map should be initialized");

    // Check if color already assigned
    if let Some(&color) = map.get(&feature_id) {
        return color;
    }

    // Generate a deterministic color based on feature ID
    let hash = feature_id as u64;
    let r = ((hash.wrapping_mul(2654435761)) % 256) as u8;
    let g = ((hash.wrapping_mul(2246822507)) % 256) as u8;
    let b = ((hash.wrapping_mul(3266489917)) % 256) as u8;

    // Ensure minimum brightness for visibility
    let color = [r.max(50), g.max(50), b.max(50)];

    map.insert(feature_id, color);
    color
}
