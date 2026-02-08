//! Rolling shutter detection for stereo calibration.
//!
//! Provides functions to detect rolling shutter effects from stereo pair data
//! by analyzing position-dependent distortions, temporal consistency, and
//! geometric constraint violations.

use crate::calibration::stereo_calibrator::{RollingShutterDetectionInfo, StereoPair};

/// Get detailed rolling shutter detection information.
///
/// Returns `None` if fewer than 3 stereo pairs are available.
pub(crate) fn rolling_shutter_detection_info(
    stereo_pairs: &[StereoPair],
    image_height: u32,
) -> Option<RollingShutterDetectionInfo> {
    if stereo_pairs.len() < 3 {
        return None;
    }

    let position_score = analyze_position_distortion(stereo_pairs, image_height);
    let temporal_score = analyze_temporal_consistency(stereo_pairs);
    let geometric_score = analyze_geometric_distortions(stereo_pairs, image_height);

    let combined_score = 0.4 * position_score + 0.4 * temporal_score + 0.2 * geometric_score;
    let detected = combined_score > 0.6;

    Some(RollingShutterDetectionInfo {
        position_distortion_score: position_score,
        temporal_consistency_score: temporal_score,
        geometric_distortion_score: geometric_score,
        combined_score,
        rolling_shutter_detected: detected,
        confidence: (combined_score * 100.0).round() as u8,
    })
}

/// Automatically detect if rolling shutter compensation is needed.
///
/// Uses the provided `log_fn` callback (if any) for diagnostic output.
pub(crate) fn detect_rolling_shutter(
    stereo_pairs: &[StereoPair],
    image_height: u32,
    log_fn: Option<&dyn Fn(&str)>,
) -> bool {
    if stereo_pairs.len() < 3 {
        // Need multiple frames for reliable detection
        return false;
    }

    // Method 1: Analyze feature position correlation with distortion
    let position_distortion_score = analyze_position_distortion(stereo_pairs, image_height);

    // Method 2: Check temporal consistency across frames
    let temporal_consistency_score = analyze_temporal_consistency(stereo_pairs);

    // Method 3: Geometric constraint violations that suggest rolling shutter
    let geometric_distortion_score = analyze_geometric_distortions(stereo_pairs, image_height);

    // Combine scores with weights
    let combined_score = 0.4 * position_distortion_score
        + 0.4 * temporal_consistency_score
        + 0.2 * geometric_distortion_score;

    // Threshold for rolling shutter detection
    let rolling_shutter_threshold = 0.6;

    if let Some(log) = log_fn {
        log(&format!(
            "🔍 Rolling shutter detection: {:.2} (threshold: {:.2})",
            combined_score, rolling_shutter_threshold
        ));
    }

    combined_score > rolling_shutter_threshold
}

/// Analyze correlation between vertical position and distortion patterns.
fn analyze_position_distortion(stereo_pairs: &[StereoPair], image_height: u32) -> f64 {
    let mut position_errors = Vec::new();

    for stereo_pair in stereo_pairs {
        for &(left_idx, right_idx) in &stereo_pair.correspondences {
            let left_pt = stereo_pair.left_features[left_idx];
            let right_pt = stereo_pair.right_features[right_idx];

            // Calculate vertical position (normalized 0-1)
            let v_pos = left_pt.y / image_height as f64;

            // Simple distortion metric: horizontal disparity variation
            let disparity = (left_pt.x - right_pt.x).abs();

            // Expected disparity for pinhole model (rough approximation)
            let expected_disparity = 50.0; // pixels, typical for stereo
            let distortion = (disparity - expected_disparity).abs() / expected_disparity;

            position_errors.push((v_pos, distortion));
        }
    }

    if position_errors.is_empty() {
        return 0.0;
    }

    // Calculate correlation between vertical position and distortion
    // Rolling shutter causes systematic distortion that correlates with position
    let mean_v =
        position_errors.iter().map(|(v, _)| v).sum::<f64>() / position_errors.len() as f64;
    let mean_d =
        position_errors.iter().map(|(_, d)| d).sum::<f64>() / position_errors.len() as f64;

    let covariance = position_errors
        .iter()
        .map(|(v, d)| (v - mean_v) * (d - mean_d))
        .sum::<f64>()
        / position_errors.len() as f64;

    let var_v = position_errors
        .iter()
        .map(|(v, _)| (v - mean_v).powi(2))
        .sum::<f64>()
        / position_errors.len() as f64;

    let var_d = position_errors
        .iter()
        .map(|(_, d)| (d - mean_d).powi(2))
        .sum::<f64>()
        / position_errors.len() as f64;

    let correlation = if var_v > 1e-10 && var_d > 1e-10 {
        covariance / (var_v.sqrt() * var_d.sqrt())
    } else {
        0.0
    };

    // Strong correlation suggests rolling shutter
    correlation.abs().min(1.0)
}

/// Analyze temporal consistency across multiple frames.
fn analyze_temporal_consistency(stereo_pairs: &[StereoPair]) -> f64 {
    if stereo_pairs.len() < 2 {
        return 0.0;
    }

    let mut consistency_scores = Vec::new();

    // Compare consecutive pairs for temporal consistency
    for i in 0..stereo_pairs.len() - 1 {
        let pair1 = &stereo_pairs[i];
        let pair2 = &stereo_pairs[i + 1];

        // Calculate average feature movement between frames
        let mut movements = Vec::new();

        // Simple temporal analysis: check if features move consistently
        // In rolling shutter, features at different heights move differently
        for &(left_idx, _) in &pair1.correspondences {
            if left_idx < pair1.left_features.len() && left_idx < pair2.left_features.len() {
                let pt1 = pair1.left_features[left_idx];
                let pt2 = pair2.left_features[left_idx];
                let movement = (pt1 - pt2).norm();
                movements.push(movement);
            }
        }

        if !movements.is_empty() {
            let mean_movement = movements.iter().sum::<f64>() / movements.len() as f64;
            let variance = movements
                .iter()
                .map(|m| (m - mean_movement).powi(2))
                .sum::<f64>()
                / movements.len() as f64;
            let consistency = 1.0 / (1.0 + variance.sqrt()); // Higher consistency = lower variance
            consistency_scores.push(consistency);
        }
    }

    if consistency_scores.is_empty() {
        0.0
    } else {
        consistency_scores.iter().sum::<f64>() / consistency_scores.len() as f64
    }
}

/// Analyze geometric distortions that suggest rolling shutter.
fn analyze_geometric_distortions(stereo_pairs: &[StereoPair], image_height: u32) -> f64 {
    let mut distortion_indicators = Vec::new();

    for stereo_pair in stereo_pairs {
        // Check for systematic distortions in epipolar geometry
        // Rolling shutter often violates simple epipolar constraints

        let mut epipolar_errors = Vec::new();

        for &(left_idx, right_idx) in &stereo_pair.correspondences {
            let left_pt = stereo_pair.left_features[left_idx];
            let right_pt = stereo_pair.right_features[right_idx];

            // Simple epipolar check: points should be at similar vertical positions
            // Rolling shutter can cause vertical misalignment
            let vertical_diff = (left_pt.y - right_pt.y).abs();

            // Normalize by image height
            let normalized_error = vertical_diff / image_height as f64;
            epipolar_errors.push(normalized_error);
        }

        if !epipolar_errors.is_empty() {
            let mean_error = epipolar_errors.iter().sum::<f64>() / epipolar_errors.len() as f64;
            // Higher mean error suggests rolling shutter effects
            distortion_indicators.push(mean_error.min(1.0));
        }
    }

    if distortion_indicators.is_empty() {
        0.0
    } else {
        distortion_indicators.iter().sum::<f64>() / distortion_indicators.len() as f64
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use nalgebra as na;

    /// Helper: build a stereo pair with uniform disparity of 50px and no vertical misalignment.
    fn make_uniform_pair(n_features: usize, timestamp: f64) -> StereoPair {
        let mut left = Vec::new();
        let mut right = Vec::new();
        let mut correspondences = Vec::new();
        for i in 0..n_features {
            let y = (i as f64 / (n_features - 1).max(1) as f64) * 480.0;
            let lx = 200.0 + (i as f64 * 3.7); // spread x a bit
            left.push(na::Vector2::new(lx, y));
            right.push(na::Vector2::new(lx - 50.0, y)); // constant 50px disparity, same y
            correspondences.push((i, i));
        }
        StereoPair {
            left_features: left,
            right_features: right,
            correspondences,
            timestamp,
            angular_velocity: None,
            linear_velocity: None,
            feature_qualities: vec![1.0; n_features],
        }
    }

    /// Helper: build a stereo pair where disparity varies linearly with y.
    /// At y=0 disparity=50, at y=480 disparity=90 (all above the expected 50px baseline).
    /// This ensures `(disparity - 50).abs() / 50` increases monotonically with y.
    fn make_y_correlated_pair(n_features: usize, timestamp: f64) -> StereoPair {
        let mut left = Vec::new();
        let mut right = Vec::new();
        let mut correspondences = Vec::new();
        for i in 0..n_features {
            let y = (i as f64 / (n_features - 1).max(1) as f64) * 480.0;
            let disparity = 50.0 + (y / 480.0) * 40.0; // 50 at top, 90 at bottom
            left.push(na::Vector2::new(200.0, y));
            right.push(na::Vector2::new(200.0 - disparity, y)); // same y (no geometric distortion)
            correspondences.push((i, i));
        }
        StereoPair {
            left_features: left,
            right_features: right,
            correspondences,
            timestamp,
            angular_velocity: None,
            linear_velocity: None,
            feature_qualities: vec![1.0; n_features],
        }
    }

    /// Helper: build a stereo pair with a fixed vertical misalignment in right features.
    fn make_vertical_misaligned_pair(
        n_features: usize,
        y_offset: f64,
        timestamp: f64,
    ) -> StereoPair {
        let mut left = Vec::new();
        let mut right = Vec::new();
        let mut correspondences = Vec::new();
        for i in 0..n_features {
            let y = (i as f64 / (n_features - 1).max(1) as f64) * 480.0;
            let lx = 200.0 + (i as f64 * 3.7);
            left.push(na::Vector2::new(lx, y));
            right.push(na::Vector2::new(lx - 50.0, y + y_offset));
            correspondences.push((i, i));
        }
        StereoPair {
            left_features: left,
            right_features: right,
            correspondences,
            timestamp,
            angular_velocity: None,
            linear_velocity: None,
            feature_qualities: vec![1.0; n_features],
        }
    }

    /// Helper: build a stereo pair with BOTH y-correlated disparity AND vertical misalignment.
    fn make_rs_evidence_pair(
        n_features: usize,
        y_offset: f64,
        timestamp: f64,
    ) -> StereoPair {
        let mut left = Vec::new();
        let mut right = Vec::new();
        let mut correspondences = Vec::new();
        for i in 0..n_features {
            let y = (i as f64 / (n_features - 1).max(1) as f64) * 480.0;
            let disparity = 50.0 + (y / 480.0) * 40.0;
            left.push(na::Vector2::new(200.0, y));
            right.push(na::Vector2::new(200.0 - disparity, y + y_offset));
            correspondences.push((i, i));
        }
        StereoPair {
            left_features: left,
            right_features: right,
            correspondences,
            timestamp,
            angular_velocity: None,
            linear_velocity: None,
            feature_qualities: vec![1.0; n_features],
        }
    }

    // ── Boundary tests ──────────────────────────────────────────────────

    #[test]
    fn test_detection_info_returns_none_with_fewer_than_3_pairs() {
        let pairs: Vec<StereoPair> = vec![make_uniform_pair(5, 0.0), make_uniform_pair(5, 1.0)];
        assert!(rolling_shutter_detection_info(&pairs, 480).is_none());
    }

    #[test]
    fn test_detect_rolling_shutter_returns_false_with_fewer_than_3_pairs() {
        let pairs: Vec<StereoPair> = vec![make_uniform_pair(5, 0.0)];
        assert!(!detect_rolling_shutter(&pairs, 480, None));
    }

    // ── Meaningful detection logic tests ─────────────────────────────────

    #[test]
    fn test_uniform_disparity_no_rs_detection() {
        // Constant 50px disparity, same y on both sides → no RS evidence.
        let pairs: Vec<StereoPair> = (0..4)
            .map(|i| make_uniform_pair(20, i as f64))
            .collect();

        let pos_score = analyze_position_distortion(&pairs, 480);
        assert!(
            pos_score < 0.1,
            "position distortion should be ~0 for uniform disparity, got {pos_score}"
        );

        let geo_score = analyze_geometric_distortions(&pairs, 480);
        assert!(
            geo_score < 1e-9,
            "geometric distortion should be 0 for aligned y, got {geo_score}"
        );

        assert!(
            !detect_rolling_shutter(&pairs, 480, None),
            "should NOT detect rolling shutter with uniform disparity"
        );
    }

    #[test]
    fn test_y_correlated_disparity_triggers_position_score() {
        // Disparity linearly varies with y → strong Pearson correlation.
        let pairs: Vec<StereoPair> = (0..4)
            .map(|i| make_y_correlated_pair(20, i as f64))
            .collect();

        let pos_score = analyze_position_distortion(&pairs, 480);
        assert!(
            pos_score > 0.7,
            "position distortion should be > 0.7 for linearly y-correlated disparity, got {pos_score}"
        );
    }

    #[test]
    fn test_vertical_misalignment_triggers_geometric_score() {
        // Right features offset by 10px vertically → geometric distortion.
        let pairs: Vec<StereoPair> = (0..4)
            .map(|i| make_vertical_misaligned_pair(20, 10.0, i as f64))
            .collect();

        let geo_score = analyze_geometric_distortions(&pairs, 480);
        // 10 / 480 ≈ 0.0208
        assert!(
            geo_score > 0.01,
            "geometric distortion should be > 0.01 for 10px vertical offset, got {geo_score}"
        );
    }

    #[test]
    fn test_combined_rs_evidence_triggers_detection() {
        // Both y-correlated disparity AND vertical misalignment → detection.
        let pairs: Vec<StereoPair> = (0..4)
            .map(|i| make_rs_evidence_pair(20, 10.0, i as f64))
            .collect();

        assert!(
            detect_rolling_shutter(&pairs, 480, None),
            "should detect rolling shutter with combined y-correlated disparity and vertical misalignment"
        );
    }
}
