pub mod rerun;
pub mod viewer;

pub use rerun::{create_viewer, RerunViewer};
pub use viewer::Viewer;

use std::collections::HashMap;
use std::sync::Mutex;

/// Global color map for feature IDs
/// Uses a deterministic hash-based color assignment to ensure consistent colors
static FEATURE_COLOR_MAP: Mutex<Option<HashMap<usize, [u8; 3]>>> = Mutex::new(None);

/// Get or assign a color for a given feature ID
/// Colors are deterministically assigned based on the feature ID using a hash function
pub fn get_feature_color(feature_id: usize) -> [u8; 3] {
    #[allow(clippy::unwrap_used)] // Mutex only poisoned on panic
    let mut map = FEATURE_COLOR_MAP.lock().unwrap();

    if map.is_none() {
        *map = Some(HashMap::new());
    }

    #[allow(clippy::unwrap_used)] // Just checked is_none() above
    let map = map.as_mut().unwrap();

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
