//! Rolling shutter detection for stereo calibration.
//!
//! Provides functions to detect rolling shutter effects from stereo pair data
//! by analyzing position-dependent distortions, temporal consistency, and
//! geometric constraint violations.

use crate::calibration::stereo_calibrator::{RollingShutterDetectionInfo, StereoPair};

/// Get detailed rolling shutter detection information.
///
/// Returns `None` if fewer than 3 stereo pairs are available.
pub fn rolling_shutter_detection_info(
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
pub fn detect_rolling_shutter(
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
pub fn analyze_position_distortion(stereo_pairs: &[StereoPair], image_height: u32) -> f64 {
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
pub fn analyze_temporal_consistency(stereo_pairs: &[StereoPair]) -> f64 {
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
pub fn analyze_geometric_distortions(stereo_pairs: &[StereoPair], image_height: u32) -> f64 {
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

    fn make_stereo_pair(n_features: usize) -> StereoPair {
        let mut left = Vec::new();
        let mut right = Vec::new();
        let mut correspondences = Vec::new();
        for i in 0..n_features {
            left.push(na::Vector2::new(100.0 + i as f64 * 10.0, 50.0 + i as f64 * 5.0));
            right.push(na::Vector2::new(80.0 + i as f64 * 10.0, 50.0 + i as f64 * 5.0));
            correspondences.push((i, i));
        }
        StereoPair {
            left_features: left,
            right_features: right,
            correspondences,
            timestamp: 0.0,
            angular_velocity: None,
            linear_velocity: None,
            feature_qualities: vec![1.0; n_features],
        }
    }

    #[test]
    fn test_detection_info_returns_none_with_fewer_than_3_pairs() {
        let pairs: Vec<StereoPair> = vec![make_stereo_pair(5), make_stereo_pair(5)];
        assert!(rolling_shutter_detection_info(&pairs, 480).is_none());
    }

    #[test]
    fn test_detection_info_returns_some_with_3_pairs() {
        let pairs: Vec<StereoPair> = (0..3).map(|_| make_stereo_pair(10)).collect();
        let info = rolling_shutter_detection_info(&pairs, 480);
        assert!(info.is_some());
    }

    #[test]
    fn test_detect_rolling_shutter_returns_false_with_fewer_than_3_pairs() {
        let pairs: Vec<StereoPair> = vec![make_stereo_pair(5)];
        assert!(!detect_rolling_shutter(&pairs, 480, None));
    }

    #[test]
    fn test_analyze_position_distortion_empty_pairs() {
        let pairs: Vec<StereoPair> = Vec::new();
        assert_eq!(analyze_position_distortion(&pairs, 480), 0.0);
    }

    #[test]
    fn test_analyze_position_distortion_no_correspondences() {
        let pair = StereoPair {
            left_features: vec![na::Vector2::new(100.0, 200.0)],
            right_features: vec![na::Vector2::new(80.0, 200.0)],
            correspondences: vec![],
            timestamp: 0.0,
            angular_velocity: None,
            linear_velocity: None,
            feature_qualities: vec![],
        };
        assert_eq!(analyze_position_distortion(&[pair], 480), 0.0);
    }

    #[test]
    fn test_analyze_temporal_consistency_single_pair() {
        let pairs = vec![make_stereo_pair(5)];
        assert_eq!(analyze_temporal_consistency(&pairs), 0.0);
    }

    #[test]
    fn test_analyze_geometric_distortions_empty_pairs() {
        let pairs: Vec<StereoPair> = Vec::new();
        assert_eq!(analyze_geometric_distortions(&pairs, 480), 0.0);
    }
}
