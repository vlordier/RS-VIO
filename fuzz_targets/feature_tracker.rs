#![no_main]
#![allow(unused_crate_dependencies)]

use libfuzzer_sys::fuzz_target;
use rs_vio::feature_tracker::{FeatureTracker, FeatureTrackerConfig};
use rs_vio::types::{Point2D, Point3D};

fuzz_target!(|data: (Vec<Point2D>, Vec<Point3D>, Vec<f64>)| {
    let (keypoints, points_3d, timestamps) = data;

    if keypoints.is_empty() || timestamps.is_empty() {
        return;
    }

    let config = FeatureTrackerConfig {
        max_features: 1000,
        quality_level: 0.01,
        min_distance: 10.0,
        block_size: 3,
        use_harris: false,
        harris_k: 0.04,
    };

    let mut tracker = FeatureTracker::new(config);

    // Fuzz track frame
    let _ = tracker.track_frame(&keypoints, timestamps[0]);

    // Fuzz triangulation
    if points_3d.len() >= 2 && keypoints.len() >= 2 {
        let _ = rs_vio::geometry::triangulate(
            &keypoints[0..2].to_vec(),
            &points_3d[0..2].to_vec(),
            None,
        );
    }
});
