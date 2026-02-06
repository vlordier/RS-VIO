#[cfg(feature = "rerun-viewer")]
pub mod rerun;
pub mod viewer;

#[cfg(feature = "rerun-viewer")]
pub use self::rerun::RerunViewer;
pub use viewer::Viewer;

use anyhow::Result;

/// Create a viewer instance. Returns the Rerun viewer when the `rerun-viewer`
/// feature is enabled, otherwise returns `Ok(None)`.
pub fn create_viewer() -> Result<Option<Box<dyn Viewer>>> {
    #[cfg(feature = "rerun-viewer")]
    {
        self::rerun::create_viewer().map(Some)
    }
    #[cfg(not(feature = "rerun-viewer"))]
    {
        Ok(None)
    }
}

use std::collections::HashMap;
use std::sync::{LazyLock, Mutex};

/// Global color map for feature IDs
/// Uses a deterministic hash-based color assignment to ensure consistent colors
static FEATURE_COLOR_MAP: LazyLock<Mutex<HashMap<usize, [u8; 3]>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

/// Get or assign a color for a given feature ID
/// Colors are deterministically assigned based on the feature ID using a hash function
pub fn get_feature_color(feature_id: usize) -> [u8; 3] {
    let mut map = match FEATURE_COLOR_MAP.lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    };

    // Check if color already assigned
    if let Some(&color) = map.get(&feature_id) {
        return color;
    }

    // Generate a deterministic color based on feature ID using a hash
    let hash = feature_id as u64;
    let r = ((hash * 2654435761) % 256) as u8;
    let g = ((hash * 2246822507) % 256) as u8;
    let b = ((hash * 3266489917) % 256) as u8;

    // Ensure minimum brightness for visibility
    let color = [r.max(50), g.max(50), b.max(50)];

    map.insert(feature_id, color);
    color
}
