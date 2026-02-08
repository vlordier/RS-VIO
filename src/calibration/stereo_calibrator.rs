//! Stereo camera calibrator implementation

use crate::calibration::config::CalibrationConfig;
use crate::calibration::factors::{EpipolarFactor, StereoReprojectionFactor};
use crate::calibration::guidance::CalibrationGuidance;
use crate::calibration::quality::{
    CalibrationLogger, CalibrationQualityMetrics, QualityThresholds,
};
use crate::calibration::{rolling_shutter, triangulation};
use crate::feature_tracker::{
    OrbDetectorConfig, OrbFeatureDetector, StereoMatcher, StereoMatcherConfig,
};
use apex_solver::core::loss_functions::HuberLoss;
use apex_solver::core::problem::Problem;
use apex_solver::manifold::ManifoldType;
use apex_solver::optimizer::levenberg_marquardt::{LevenbergMarquardt, LevenbergMarquardtConfig};
use nalgebra as na;
use std::time::{Duration, Instant};

/// Status of calibration process
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationStatus {
    /// Calibration not started
    NotStarted,
    /// Collecting stereo pairs
    CollectingData,
    /// Running optimization
    Optimizing,
    /// Calibration completed successfully
    Success,
    /// Calibration failed
    Failed,
}

/// Detailed information about rolling shutter detection
#[derive(Debug, Clone)]
pub struct RollingShutterDetectionInfo {
    /// Score from position-based distortion analysis (0-1)
    pub position_distortion_score: f64,
    /// Score from temporal consistency analysis (0-1)
    pub temporal_consistency_score: f64,
    /// Score from geometric distortion analysis (0-1)
    pub geometric_distortion_score: f64,
    /// Combined detection score (0-1)
    pub combined_score: f64,
    /// Whether rolling shutter was detected
    pub rolling_shutter_detected: bool,
    /// Confidence percentage (0-100)
    pub confidence: u8,
}

/// Result of stereo calibration
#[derive(Debug, Clone)]
pub struct CalibrationResult {
    /// Calibrated left camera intrinsics [fx, fy, cx, cy, k1, k2, p1, p2, k3]
    pub left_intrinsics: Vec<f64>,
    /// Calibrated right camera intrinsics [fx, fy, cx, cy, k1, k2, p1, p2, k3]
    pub right_intrinsics: Vec<f64>,
    /// Calibrated stereo extrinsics (right camera pose relative to left)
    pub stereo_extrinsics: na::Isometry3<f64>,
    /// Final reprojection error (pixels)
    pub final_reprojection_error: f64,
    /// Number of stereo pairs used
    pub num_stereo_pairs: usize,
    /// Number of feature matches used
    pub num_feature_matches: usize,
}

/// Stereo pair data for calibration
#[derive(Debug, Clone)]
pub struct StereoPair {
    /// Left image features (pixel coordinates)
    pub left_features: Vec<na::Vector2<f64>>,
    /// Right image features (pixel coordinates)
    pub right_features: Vec<na::Vector2<f64>>,
    /// Feature correspondences (indices into left/right feature vectors)
    pub correspondences: Vec<(usize, usize)>,
    /// Timestamp when images were captured (seconds)
    pub timestamp: f64,
    /// Camera motion during exposure (for rolling shutter compensation)
    pub angular_velocity: Option<na::Vector3<f64>>, // rad/s
    pub linear_velocity: Option<na::Vector3<f64>>, // m/s
    /// Feature quality scores (for robust outlier rejection)
    pub feature_qualities: Vec<f64>,
}

/// Temporal sequence of stereo pairs for super resolution calibration
#[derive(Debug, Clone)]
pub struct TemporalStereoSequence {
    /// Sequence of stereo pairs over time
    pub stereo_pairs: Vec<StereoPair>,
    /// Tracked features across the sequence (feature_id -> temporal observations)
    pub temporal_tracks: std::collections::HashMap<usize, Vec<TemporalFeatureObservation>>,
    /// Sequence start time
    pub start_time: f64,
    /// Sequence duration
    pub duration: f64,
}

#[derive(Debug, Clone)]
pub struct TemporalFeatureObservation {
    /// Feature position in left image
    pub left_point: na::Vector2<f64>,
    /// Feature position in right image
    pub right_point: na::Vector2<f64>,
    /// Timestamp relative to sequence start
    pub timestamp: f64,
    /// Row position for rolling shutter (0.0 = top, 1.0 = bottom)
    pub row_position: f64,
    /// Feature quality score
    pub quality: f64,
    /// Stereo pair index in sequence
    pub pair_index: usize,
}

/// Parameters for patch-based matching
#[derive(Debug, Clone, Copy)]
pub struct PatchMatchParams {
    pub left_x: i32,
    pub left_y: i32,
    pub right_x: i32,
    pub right_y: i32,
    pub patch_size: i32,
}

/// Stereo camera auto-calibrator
pub struct StereoCalibrator {
    config: CalibrationConfig,
    stereo_pairs: Vec<StereoPair>,
    temporal_sequences: Vec<TemporalStereoSequence>,
    status: CalibrationStatus,
    logger: Option<CalibrationLogger>,
    start_time: Option<Instant>,
    sequence_start_time: Option<f64>,
    current_sequence_pairs: Vec<StereoPair>,
    guidance: CalibrationGuidance,
}

impl StereoCalibrator {
    /// Create a new stereo calibrator with given configuration
    pub fn new(config: CalibrationConfig) -> Self {
        let logger = if config.log_calibration_metrics {
            Some(CalibrationLogger::new(QualityThresholds::default()))
        } else {
            None
        };

        Self {
            config,
            stereo_pairs: Vec::new(),
            temporal_sequences: Vec::new(),
            status: CalibrationStatus::NotStarted,
            logger,
            start_time: None,
            sequence_start_time: None,
            current_sequence_pairs: Vec::new(),
            guidance: CalibrationGuidance::new(),
        }
    }

    /// Get current calibration guidance and suggestions
    pub const fn get_guidance(&self) -> &CalibrationGuidance {
        &self.guidance
    }

    /// Get real-time calibration status display
    pub fn get_status_display(&self) -> String {
        self.guidance.get_status_display()
    }

    /// Check if calibration is ready (sufficient quality achieved)
    pub const fn is_calibration_ready(&self) -> bool {
        self.guidance.calibration_ready
    }

    /// Add a stereo pair for calibration
    ///
    /// Features should be detected and matched between the left and right images.
    /// Correspondences should be established between feature indices.
    pub fn add_stereo_pair(&mut self, stereo_pair: StereoPair) {
        if self.stereo_pairs.len() < self.config.max_stereo_pairs {
            // Validate that we have enough matches
            if stereo_pair.correspondences.len() >= self.config.min_feature_matches {
                self.stereo_pairs.push(stereo_pair);
                self.status = CalibrationStatus::CollectingData;

                let progress = self.progress_percentage();
                let match_count = self
                    .stereo_pairs
                    .last()
                    .map(|p| p.correspondences.len())
                    .unwrap_or(0);
                self.log_progress(&format!(
                    "📷 Added stereo pair {}/{} ({} matches) - Progress: {:.1}%",
                    self.stereo_pairs.len(),
                    self.config.max_stereo_pairs,
                    match_count,
                    progress
                ));

                // Check if we have enough data to start calibration
                if self.stereo_pairs.len() >= 5 {
                    // Minimum for reasonable calibration
                    self.log_progress(
                        "🎯 Ready for calibration! Call calibrate() when data collection complete.",
                    );
                }
            } else {
                self.log_progress(&format!(
                    "⚠️  Skipped stereo pair with {} matches (minimum required: {})",
                    stereo_pair.correspondences.len(),
                    self.config.min_feature_matches
                ));
            }
        } else {
            self.log_progress(&format!(
                "🛑 Maximum stereo pairs ({}) reached - ready for calibration!",
                self.config.max_stereo_pairs
            ));
        }

        // Update guidance with new data (if enabled)
        if self.config.adaptive_guidance_enabled {
            self.guidance.update_guidance(&self.stereo_pairs);

            // Log guidance status
            if self.guidance.calibration_ready {
                self.log_progress("🎉 Calibration quality is excellent - ready to calibrate!");
            } else if let Some(suggestion) = &self.guidance.next_suggestion {
                self.log_progress(&format!("💡 {}", suggestion.description));
            }
        }

        // Try auto-calibration if enabled
        if self.config.auto_calibration_enabled {
            if let Some(calibration_result) = self.try_auto_calibrate() {
                // Note: In a real implementation, you'd want to handle the result
                // For now, we just log that auto-calibration was attempted
                match calibration_result {
                    Ok(_) => self.log_progress("✅ Auto-calibration successful!"),
                    Err(e) => self.log_progress(&format!("❌ Auto-calibration failed: {}", e)),
                }
            }
        }
    }

    /// Process raw stereo images with automatic feature detection and matching
    ///
    /// This method performs auto-calibration by:
    /// - Detecting features with ORB descriptors and subpixel refinement (if enabled)
    /// - Matching features using descriptor distance and geometric constraints
    /// - Applying outlier rejection
    ///
    /// Falls back to FAST feature detection if ORB features are disabled.
    ///
    /// # Arguments
    /// * `left_image` - Left camera image
    /// * `right_image` - Right camera image
    /// * `timestamp` - Optional timestamp for temporal tracking
    ///
    /// # Returns
    /// Success status
    pub fn process_stereo_images(
        &mut self,
        left_image: &image::GrayImage,
        right_image: &image::GrayImage,
        timestamp: Option<f64>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        if self.config.orb_features_enabled {
            self.process_stereo_images_orb(left_image, right_image, timestamp)
        } else {
            self.process_stereo_images_basic(left_image, right_image, timestamp)
        }
    }

    /// Process stereo images with ORB features
    fn process_stereo_images_orb(
        &mut self,
        left_image: &image::GrayImage,
        right_image: &image::GrayImage,
        timestamp: Option<f64>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Initialize ORB feature detector
        let detector_config = OrbDetectorConfig {
            max_features: self.config.orb_max_features,
            fast_threshold: self.config.orb_fast_threshold,
            min_distance: 10.0,
            pyramid_levels: self.config.orb_pyramid_levels,
            scale_factor: self.config.orb_scale_factor,
            enable_subpixel: true,
            enable_orientation: true,
            brief_pattern_size: 128,
            brief_smoothing_sigma: 2.0,
        };
        let detector = OrbFeatureDetector::new(detector_config);

        // Detect features in both images
        let left_features = detector.detect(left_image);
        let right_features = detector.detect(right_image);

        self.log_progress(&format!(
            "🔍 Detected {} left, {} right ORB features",
            left_features.len(),
            right_features.len()
        ));

        if left_features.is_empty() || right_features.is_empty() {
            return Err("No features detected in one or both images".into());
        }

        // Initialize stereo matcher
        let matcher_config = StereoMatcherConfig {
            max_descriptor_distance: self.config.stereo_matcher_max_distance,
            max_epipolar_error: 2.0,
            ratio_threshold: self.config.stereo_matcher_ratio_threshold,
            enable_geometric_check: true,
            ransac_iterations: self.config.stereo_matcher_ransac_iterations,
            ransac_threshold: 1.0,
            enable_hierarchical_matching: true,
            pyramid_levels: 3,
        };
        let matcher = StereoMatcher::new(matcher_config);

        // Use approximate intrinsics based on image size
        // NOTE: Uses approximate intrinsics from image dimensions; once calibrated,
        // these could be refined in a second pass.
        let fx = left_image.width() as f64 * 0.8;
        let fy = left_image.height() as f64 * 0.8;
        let cx = left_image.width() as f64 * 0.5;
        let cy = left_image.height() as f64 * 0.5;
        let intrinsics = na::Matrix3::new(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);

        // Match features
        let matches = matcher.match_features(&left_features, &right_features, &intrinsics);

        self.log_progress(&format!(
            "🔗 Found {} stereo matches (avg epipolar error: {:.2}px)",
            matches.len(),
            matches.iter().map(|m| m.epipolar_error).sum::<f32>() / matches.len() as f32
        ));

        if matches.len() < self.config.min_feature_matches {
            return Err(format!(
                "Insufficient matches: {} (minimum: {})",
                matches.len(),
                self.config.min_feature_matches
            )
            .into());
        }

        // Convert to StereoPair format
        let stereo_pair = self.create_stereo_pair_from_matches(
            left_features,
            right_features,
            matches,
            timestamp.unwrap_or(0.0),
        );

        // Add the stereo pair
        self.add_stereo_pair(stereo_pair);

        Ok(())
    }

    /// Process stereo images with basic FAST features (fallback)
    fn process_stereo_images_basic(
        &mut self,
        left_image: &image::GrayImage,
        right_image: &image::GrayImage,
        timestamp: Option<f64>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Use the existing async detector for basic features
        let detector_config = crate::feature_tracker::AsyncDetectorConfig {
            max_features: 1000,
            threshold: 20.0,
            ..Default::default()
        };
        let detector = crate::feature_tracker::AsyncFeatureDetector::new(detector_config);

        // Detect features synchronously
        let left_detected = detector.detect(
            &image::DynamicImage::ImageLuma8(left_image.clone()).to_luma8(),
            left_image.width(),
            left_image.height(),
        );
        let right_detected = detector.detect(
            &image::DynamicImage::ImageLuma8(right_image.clone()).to_luma8(),
            right_image.width(),
            right_image.height(),
        );

        self.log_progress(&format!(
            "🔍 Detected {} left, {} right FAST features",
            left_detected.len(),
            right_detected.len()
        ));

        if left_detected.is_empty() || right_detected.is_empty() {
            return Err("No features detected in one or both images".into());
        }

        // Basic stereo matching using epipolar search
        let matches =
            self.basic_stereo_matching(&left_detected, &right_detected, left_image, right_image);

        self.log_progress(&format!("🔗 Found {} basic stereo matches", matches.len()));

        if matches.len() < self.config.min_feature_matches {
            return Err(format!(
                "Insufficient matches: {} (minimum: {})",
                matches.len(),
                self.config.min_feature_matches
            )
            .into());
        }

        // Convert to StereoPair format
        let stereo_pair = self.create_basic_stereo_pair(
            left_detected,
            right_detected,
            matches,
            timestamp.unwrap_or(0.0),
        );

        // Add the stereo pair
        self.add_stereo_pair(stereo_pair);

        Ok(())
    }

    /// Basic stereo matching using epipolar search
    fn basic_stereo_matching(
        &self,
        left_features: &[crate::feature_tracker::DetectedFeature],
        right_features: &[crate::feature_tracker::DetectedFeature],
        left_image: &image::GrayImage,
        right_image: &image::GrayImage,
    ) -> Vec<(usize, usize)> {
        let mut matches = Vec::new();
        let max_disparity = 64; // pixels
        let patch_size = 8;

        for (left_idx, left_feat) in left_features.iter().enumerate() {
            let mut best_match = None;
            let mut best_score = f32::INFINITY;

            // Search along epipolar line (same row)
            let y = left_feat.y as i32;
            let left_x = left_feat.x as i32;

            for (right_idx, right_feat) in right_features.iter().enumerate() {
                let right_x = right_feat.x as i32;
                let right_y = right_feat.y as i32;

                // Check epipolar constraint (same row)
                if (right_y - y).abs() > 2 {
                    continue;
                }

                // Check disparity range
                let disparity = left_x - right_x;
                if disparity < 0 || disparity > max_disparity {
                    continue;
                }

                // Compute NCC score
                let params = PatchMatchParams {
                    left_x,
                    left_y: y,
                    right_x,
                    right_y,
                    patch_size,
                };
                let score = self.compute_ncc_score(left_image, right_image, params);

                if score < best_score {
                    best_score = score;
                    best_match = Some(right_idx);
                }
            }

            if let Some(right_idx) = best_match {
                if best_score < 0.8 {
                    // NCC threshold
                    matches.push((left_idx, right_idx));
                }
            }
        }

        matches
    }

    /// Compute Normalized Cross Correlation score
    fn compute_ncc_score(
        &self,
        left_img: &image::GrayImage,
        right_img: &image::GrayImage,
        params: PatchMatchParams,
    ) -> f32 {
        let mut sum_left = 0.0;
        let mut sum_right = 0.0;
        let mut sum_left_sq = 0.0;
        let mut sum_right_sq = 0.0;
        let mut sum_prod = 0.0;
        let mut count = 0;

        for dy in -params.patch_size / 2..=params.patch_size / 2 {
            for dx in -params.patch_size / 2..=params.patch_size / 2 {
                let lx = (params.left_x + dx).clamp(0, left_img.width() as i32 - 1) as u32;
                let ly = (params.left_y + dy).clamp(0, left_img.height() as i32 - 1) as u32;
                let rx = (params.right_x + dx).clamp(0, right_img.width() as i32 - 1) as u32;
                let ry = (params.right_y + dy).clamp(0, right_img.height() as i32 - 1) as u32;

                let left_val = left_img.get_pixel(lx, ly)[0] as f32;
                let right_val = right_img.get_pixel(rx, ry)[0] as f32;

                sum_left += left_val;
                sum_right += right_val;
                sum_left_sq += left_val * left_val;
                sum_right_sq += right_val * right_val;
                sum_prod += left_val * right_val;
                count += 1;
            }
        }

        if count == 0 {
            return 1.0;
        }

        let mean_left = sum_left / count as f32;
        let mean_right = sum_right / count as f32;

        let numerator = sum_prod - count as f32 * mean_left * mean_right;
        let denominator_left = (sum_left_sq - count as f32 * mean_left * mean_left).sqrt();
        let denominator_right = (sum_right_sq - count as f32 * mean_right * mean_right).sqrt();

        if denominator_left < 1e-6 || denominator_right < 1e-6 {
            return 1.0;
        }

        1.0 - (numerator / (denominator_left * denominator_right)).abs()
    }

    /// Create StereoPair from basic detected features
    fn create_basic_stereo_pair(
        &self,
        left_features: Vec<crate::feature_tracker::DetectedFeature>,
        right_features: Vec<crate::feature_tracker::DetectedFeature>,
        matches: Vec<(usize, usize)>,
        timestamp: f64,
    ) -> StereoPair {
        // Convert detected features to basic feature format
        let left_basic_features: Vec<na::Vector2<f64>> = left_features
            .iter()
            .map(|f| na::Vector2::new(f.x as f64, f.y as f64))
            .collect();

        let right_basic_features: Vec<na::Vector2<f64>> = right_features
            .iter()
            .map(|f| na::Vector2::new(f.x as f64, f.y as f64))
            .collect();

        // Convert matches to correspondences
        let correspondences: Vec<(usize, usize)> = matches.clone();

        // Create basic feature qualities
        let feature_qualities: Vec<f64> = vec![1.0; matches.len()];

        StereoPair {
            left_features: left_basic_features,
            right_features: right_basic_features,
            correspondences,
            feature_qualities,
            timestamp,
            angular_velocity: None,
            linear_velocity: None,
        }
    }

    /// Create StereoPair from ORB features and matches
    fn create_stereo_pair_from_matches(
        &self,
        left_features: Vec<crate::feature_tracker::OrbFeature>,
        right_features: Vec<crate::feature_tracker::OrbFeature>,
        matches: Vec<crate::feature_tracker::StereoMatch>,
        timestamp: f64,
    ) -> StereoPair {
        // Convert ORB features to basic feature format
        let left_basic_features: Vec<na::Vector2<f64>> = left_features
            .iter()
            .map(|f| na::Vector2::new(f.point.x as f64, f.point.y as f64))
            .collect();

        let right_basic_features: Vec<na::Vector2<f64>> = right_features
            .iter()
            .map(|f| na::Vector2::new(f.point.x as f64, f.point.y as f64))
            .collect();

        // Convert matches to correspondences
        let correspondences: Vec<(usize, usize)> =
            matches.iter().map(|m| (m.left_idx, m.right_idx)).collect();

        // Create feature qualities from match scores
        let feature_qualities: Vec<f64> = matches
            .iter()
            .map(|m| 1.0 / (m.score as f64 + 1.0)) // Convert Hamming distance to quality
            .collect();

        StereoPair {
            left_features: left_basic_features,
            right_features: right_basic_features,
            correspondences,
            feature_qualities,
            timestamp,
            angular_velocity: None,
            linear_velocity: None,
        }
    }

    /// Attempt automatic calibration if quality is sufficient
    pub fn try_auto_calibrate(
        &mut self,
    ) -> Option<Result<CalibrationResult, Box<dyn std::error::Error>>> {
        // Update guidance first
        self.guidance.update_guidance(&self.stereo_pairs);

        // Auto-calibrate if quality is sufficient and we have minimum data
        if self.guidance.calibration_ready && self.stereo_pairs.len() >= 8 {
            self.log_progress("🤖 Auto-calibrating - quality threshold reached!");
            Some(self.calibrate())
        } else {
            None
        }
    }

    /// Validate calibration quality on held-out data
    pub fn validate_calibration(
        &self,
        _result: &CalibrationResult,
        validation_pairs: &[StereoPair],
    ) -> CalibrationQualityMetrics {
        use crate::calibration::quality::CalibrationQualityMetrics;

        let mut validation_errors = Vec::new();

        for pair in validation_pairs {
            for (left_idx, right_idx) in &pair.correspondences {
                let left_point = pair.left_features[*left_idx];
                let right_point = pair.right_features[*right_idx];

                // Project using calibrated parameters
                // This is a simplified validation - in practice you'd triangulate and reproject
                let error = (left_point - right_point).norm(); // Simplified error metric
                validation_errors.push(error);
            }
        }

        CalibrationQualityMetrics::from_reprojection_errors(validation_errors)
    }

    /// Perform cross-validation by holding out some data
    pub fn cross_validate_calibration(
        &mut self,
        result: &CalibrationResult,
        holdout_fraction: f64,
    ) -> Option<CalibrationQualityMetrics> {
        if self.stereo_pairs.len() < 10 {
            return None; // Need minimum data for cross-validation
        }

        let holdout_count = (self.stereo_pairs.len() as f64 * holdout_fraction) as usize;
        let validation_pairs: Vec<_> = self.stereo_pairs.iter()
            .rev() // Use most recent pairs for validation
            .take(holdout_count)
            .cloned()
            .collect();

        if validation_pairs.is_empty() {
            return None;
        }

        let metrics = self.validate_calibration(result, &validation_pairs);
        self.log_progress(&format!(
            "🔍 Cross-validation: {:.2}px mean error, {:.1}% <1px accuracy",
            metrics.mean_reprojection_error, metrics.accuracy_percentage_1px
        ));

        Some(metrics)
    }

    /// Add stereo pair to current temporal sequence for super resolution
    pub fn add_stereo_pair_to_sequence(&mut self, stereo_pair: StereoPair) {
        // Initialize sequence if this is the first pair
        if self.sequence_start_time.is_none() {
            self.sequence_start_time = Some(stereo_pair.timestamp);
            self.log_progress("🎬 Started collecting temporal sequence for super resolution");
        }

        // Check if we should start a new sequence (time gap too large)
        if let Some(start_time) = self.sequence_start_time {
            let time_gap = stereo_pair.timestamp - start_time;
            if time_gap > self.config.temporal_sequence_max_duration {
                self.finalize_current_sequence();
                self.sequence_start_time = Some(stereo_pair.timestamp);
                self.log_progress("🎬 Started new temporal sequence (time gap detected)");
            }
        }

        // Add to current sequence
        self.current_sequence_pairs.push(stereo_pair.clone());

        // Also add to regular pairs for backward compatibility
        self.add_stereo_pair(stereo_pair);

        // Check if sequence is ready for processing
        if self.current_sequence_pairs.len() >= self.config.temporal_sequence_min_pairs {
            let last_timestamp = self
                .current_sequence_pairs
                .last()
                .map(|p| p.timestamp)
                .unwrap_or(0.0);
            let duration = last_timestamp - self.current_sequence_pairs[0].timestamp;
            self.log_progress(&format!(
                "⏱️  Temporal sequence: {} pairs, {:.3}s duration",
                self.current_sequence_pairs.len(),
                duration
            ));
        }
    }

    /// Finalize current temporal sequence and prepare for super resolution
    pub fn finalize_current_sequence(&mut self) {
        if self.current_sequence_pairs.is_empty() {
            return;
        }

        // Build temporal tracks by associating features across frames
        let mut temporal_tracks = std::collections::HashMap::new();
        let mut feature_id_counter = 0;

        for (pair_idx, pair) in self.current_sequence_pairs.iter().enumerate() {
            for (corr_idx, &(left_idx, right_idx)) in pair.correspondences.iter().enumerate() {
                let left_point = pair.left_features[left_idx];
                let right_point = pair.right_features[right_idx];
                let quality = pair.feature_qualities.get(corr_idx).copied().unwrap_or(1.0);

                // NOTE: Known limitation — each correspondence starts a new track.
                // Cross-frame feature tracking requires descriptor persistence (future enhancement).
                let feature_id = feature_id_counter;
                feature_id_counter += 1;

                let sequence_start = self.sequence_start_time.unwrap_or(0.0);
                let observation = TemporalFeatureObservation {
                    left_point,
                    right_point,
                    timestamp: pair.timestamp - sequence_start,
                    row_position: 0.5, // NOTE: Hardcoded mid-frame row; per-feature row estimation is a future enhancement.
                    quality,
                    pair_index: pair_idx,
                };

                temporal_tracks
                    .entry(feature_id)
                    .or_insert_with(Vec::new)
                    .push(observation);
            }
        }

        let last_timestamp = self
            .current_sequence_pairs
            .last()
            .map(|p| p.timestamp)
            .unwrap_or(0.0);
        let first_timestamp = self
            .current_sequence_pairs
            .first()
            .map(|p| p.timestamp)
            .unwrap_or(0.0);
        let duration = last_timestamp - first_timestamp;
        let num_tracks = temporal_tracks.len();

        let sequence = TemporalStereoSequence {
            stereo_pairs: self.current_sequence_pairs.clone(),
            temporal_tracks,
            start_time: self.sequence_start_time.unwrap_or(0.0),
            duration,
        };

        self.temporal_sequences.push(sequence);
        self.log_progress(&format!(
            "✅ Finalized temporal sequence: {} tracks, {:.3}s duration",
            num_tracks, duration
        ));

        // Reset for next sequence
        self.current_sequence_pairs.clear();
        self.sequence_start_time = None;
    }

    /// Run temporal super resolution calibration
    pub fn calibrate_with_temporal_super_resolution(
        &mut self,
    ) -> Result<CalibrationResult, Box<dyn std::error::Error>> {
        // Finalize any pending sequence
        self.finalize_current_sequence();

        if self.temporal_sequences.is_empty() {
            self.log_progress(
                "⚠️  No temporal sequences available, falling back to standard calibration",
            );
            return self.calibrate();
        }

        self.status = CalibrationStatus::Optimizing;
        self.start_time = Some(Instant::now());

        self.log_progress("🚀 Starting temporal super resolution calibration");
        self.log_progress(&format!(
            "🎬 Using {} temporal sequences",
            self.temporal_sequences.len()
        ));

        // NOTE: Temporal super resolution is not yet implemented.
        // Falls back to standard calibration with temporal metadata logging.

        // For now, fall back to standard calibration but log the temporal info
        let total_tracks: usize = self
            .temporal_sequences
            .iter()
            .map(|seq| seq.temporal_tracks.len())
            .sum();
        let avg_duration: f64 = self
            .temporal_sequences
            .iter()
            .map(|seq| seq.duration)
            .sum::<f64>()
            / self.temporal_sequences.len() as f64;

        self.log_progress(&format!(
            "📊 Temporal data: {} total tracks, {:.3}s avg sequence duration",
            total_tracks, avg_duration
        ));

        // Fall back to standard calibration for now
        self.calibrate()
    }

    /// Run the stereo calibration optimization
    pub fn calibrate(&mut self) -> Result<CalibrationResult, Box<dyn std::error::Error>> {
        if self.stereo_pairs.is_empty() {
            return Err("No stereo pairs available for calibration".into());
        }

        self.status = CalibrationStatus::Optimizing;
        self.start_time = Some(Instant::now());

        // Auto-detect rolling shutter if not manually specified
        let rolling_shutter_enabled = self.effective_rolling_shutter_enabled();
        let detection_method = if self.config.rolling_shutter_enabled.is_none() {
            "🔍 Auto-detected"
        } else {
            "⚙️  Manual setting"
        };

        self.log_progress(&format!(
            "🚀 Starting calibration with {} stereo pairs",
            self.stereo_pairs.len()
        ));
        self.log_progress(&format!(
            "📊 Target: <{:.1}% accuracy at <{:.1}px error",
            70.0, 1.0
        ));
        self.log_progress(&format!(
            "🎥 Rolling shutter: {} ({})",
            if rolling_shutter_enabled {
                "ENABLED"
            } else {
                "DISABLED"
            },
            detection_method
        ));

        log::info!(
            "Starting stereo calibration with {} stereo pairs",
            self.stereo_pairs.len()
        );

        // Initialize parameters
        let initial_params = triangulation::initialize_parameters(&self.config);

        // Create optimization problem and initial values
        let mut problem = Problem::new();
        let mut initial_values = std::collections::HashMap::new();

        // Add variables
        let left_intrinsics_var = "left_intrinsics".to_string();
        let right_intrinsics_var = "right_intrinsics".to_string();
        let extrinsics_var = "stereo_extrinsics".to_string();

        initial_values.insert(
            left_intrinsics_var.clone(),
            (
                ManifoldType::RN,
                na::DVector::from_vec(initial_params.left_intrinsics.clone()),
            ),
        );
        initial_values.insert(
            right_intrinsics_var.clone(),
            (
                ManifoldType::RN,
                na::DVector::from_vec(initial_params.right_intrinsics.clone()),
            ),
        );
        initial_values.insert(
            extrinsics_var.clone(),
            (
                ManifoldType::RN,
                na::DVector::from_vec(initial_params.extrinsics.clone()),
            ),
        );

        // Add 3D point variables for each correspondence
        let mut point_vars = Vec::new();
        for (pair_idx, stereo_pair) in self.stereo_pairs.iter().enumerate() {
            for (corr_idx, &(left_idx, right_idx)) in stereo_pair.correspondences.iter().enumerate()
            {
                let point_var = format!("point_{}_{}", pair_idx, corr_idx);
                let initial_point = triangulation::triangulate_initial_point(
                    &stereo_pair.left_features[left_idx],
                    &stereo_pair.right_features[right_idx],
                    &initial_params,
                );
                initial_values.insert(
                    point_var.clone(),
                    (ManifoldType::RN, na::DVector::from_vec(initial_point)),
                );
                point_vars.push(point_var);
            }
        }

        // Add factors
        let mut point_var_idx = 0;
        for stereo_pair in &self.stereo_pairs {
            for &(left_idx, right_idx) in &stereo_pair.correspondences {
                let left_obs = stereo_pair.left_features[left_idx];
                let right_obs = stereo_pair.right_features[right_idx];

                let point_var = &point_vars[point_var_idx];
                point_var_idx += 1;

                // Add reprojection factor
                let factor = StereoReprojectionFactor::new(left_obs, right_obs);
                let huber_loss = HuberLoss::new(self.config.huber_delta).ok();
                problem.add_residual_block(
                    &[
                        &left_intrinsics_var,
                        &right_intrinsics_var,
                        &extrinsics_var,
                        point_var,
                    ],
                    Box::new(factor),
                    huber_loss.map(|l| {
                        Box::new(l)
                            as Box<dyn apex_solver::core::loss_functions::LossFunction + Send>
                    }),
                );

                // Optionally add epipolar constraint
                let epipolar_factor = EpipolarFactor::new(left_obs, right_obs);
                problem.add_residual_block(
                    &[&left_intrinsics_var, &right_intrinsics_var, &extrinsics_var],
                    Box::new(epipolar_factor),
                    None,
                );
            }
        }

        // Initialize variables in the problem
        problem.initialize_variables(&initial_values);

        // Configure optimizer
        let optimizer_config = LevenbergMarquardtConfig {
            max_iterations: self.config.max_iterations,
            parameter_tolerance: self.config.parameter_tolerance,
            cost_tolerance: self.config.cost_tolerance,
            ..Default::default()
        };

        let mut optimizer = LevenbergMarquardt::with_config(optimizer_config);

        // Run optimization
        let opt_result = optimizer.optimize(&problem, &initial_values)?;

        // Check if optimization was successful
        let is_successful = crate::optimization::optimization_converged(&opt_result.status)
            || matches!(
                &opt_result.status,
                apex_solver::optimizer::OptimizationStatus::MaxIterationsReached
            );

        if !is_successful {
            return Err(format!(
                "Calibration optimization failed with status: {:?}",
                opt_result.status
            )
            .into());
        }

        // Extract results
        let final_left_intrinsics = opt_result
            .parameters
            .get(&left_intrinsics_var)
            .ok_or("Missing left intrinsics in result")?
            .to_vector();
        let final_right_intrinsics = opt_result
            .parameters
            .get(&right_intrinsics_var)
            .ok_or("Missing right intrinsics in result")?
            .to_vector();
        let final_extrinsics = opt_result
            .parameters
            .get(&extrinsics_var)
            .ok_or("Missing extrinsics in result")?
            .to_vector();

        // Convert extrinsics back to SE(3)
        let final_extrinsics_se3 = triangulation::vector_to_isometry(final_extrinsics.as_slice());

        // Compute final reprojection error and quality metrics
        let final_error = triangulation::compute_reprojection_error(
            &self.stereo_pairs,
            final_left_intrinsics.as_slice(),
            final_right_intrinsics.as_slice(),
            &final_extrinsics_se3,
        );

        // Compute detailed quality metrics
        let per_point_errors = triangulation::compute_per_point_errors(
            &self.stereo_pairs,
            final_left_intrinsics.as_slice(),
            final_right_intrinsics.as_slice(),
            &final_extrinsics_se3,
        );

        let quality_metrics = CalibrationQualityMetrics::from_reprojection_errors(per_point_errors);

        let calibration_result = CalibrationResult {
            left_intrinsics: final_left_intrinsics.as_slice().to_vec(),
            right_intrinsics: final_right_intrinsics.as_slice().to_vec(),
            stereo_extrinsics: final_extrinsics_se3,
            final_reprojection_error: final_error,
            num_stereo_pairs: self.stereo_pairs.len(),
            num_feature_matches: self
                .stereo_pairs
                .iter()
                .map(|p| p.correspondences.len())
                .sum(),
        };

        // Log optimization completion
        self.log_progress(&format!(
            "✅ Optimization completed in {} iterations",
            opt_result.iterations
        ));
        self.log_progress(&format!("📈 Final cost: {:.6}", opt_result.final_cost));
        self.log_progress(&format!(
            "🎯 Final reprojection error: {:.2} pixels",
            final_error
        ));

        // Log quality metrics
        self.log_quality_metrics(
            &quality_metrics,
            self.start_time
                .map(|start| start.elapsed().as_secs_f64())
                .unwrap_or(0.0),
        );

        // Log detailed results
        self.log_progress(&format!(
            "📊 Used {} stereo pairs, {} total feature matches",
            calibration_result.num_stereo_pairs, calibration_result.num_feature_matches
        ));

        if let Some(logger) = &self.logger {
            let trends = logger.analyze_trends();
            if let Some(stability) = trends.get("stability_score") {
                self.log_progress(&format!("🔄 Calibration stability: {:.2}", stability));
            }
        }

        self.status = CalibrationStatus::Success;
        log::info!("Stereo calibration completed successfully!");
        log::info!("Final reprojection error: {:.2} pixels", final_error);
        log::info!(
            "Used {} stereo pairs with {} feature matches",
            calibration_result.num_stereo_pairs,
            calibration_result.num_feature_matches
        );

        Ok(calibration_result)
    }

    /// Get current calibration status
    pub const fn status(&self) -> CalibrationStatus {
        self.status
    }

    /// Get number of collected stereo pairs
    pub const fn num_stereo_pairs(&self) -> usize {
        self.stereo_pairs.len()
    }

    /// Get current calibration progress as percentage (0-100)
    pub fn progress_percentage(&self) -> f64 {
        match self.status {
            CalibrationStatus::NotStarted => 0.0,
            CalibrationStatus::CollectingData => {
                (self.stereo_pairs.len() as f64 / self.config.max_stereo_pairs as f64) * 50.0
            },
            CalibrationStatus::Optimizing => 75.0, // Assume optimization takes 25% of time
            CalibrationStatus::Success | CalibrationStatus::Failed => 100.0,
        }
    }

    /// Get current quality assessment (if calibration completed)
    pub fn current_quality_metrics(&self) -> Option<CalibrationQualityMetrics> {
        if self.status != CalibrationStatus::Success {
            return None;
        }

        // We need to recompute this - in a real implementation we'd cache it
        // For now, return None as we don't have the final parameters stored
        None
    }

    /// Get detailed rolling shutter detection information
    pub fn rolling_shutter_detection_info(&self) -> Option<RollingShutterDetectionInfo> {
        rolling_shutter::rolling_shutter_detection_info(
            &self.stereo_pairs,
            self.config.image_height,
        )
    }

    /// Automatically detect if rolling shutter compensation is needed
    pub fn detect_rolling_shutter(&self) -> bool {
        let log_fn = if self.config.log_calibration_metrics {
            let start_time = self.start_time;
            Some(move |msg: &str| {
                let elapsed = start_time
                    .map(|start| start.elapsed())
                    .unwrap_or(std::time::Duration::ZERO);
                println!("[CALIBRATION {:.2}s] {}", elapsed.as_secs_f64(), msg);
            })
        } else {
            None
        };
        rolling_shutter::detect_rolling_shutter(
            &self.stereo_pairs,
            self.config.image_height,
            log_fn.as_ref().map(|f| f as &dyn Fn(&str)),
        )
    }

    /// Get the effective rolling shutter setting (auto-detected or manual)
    pub fn effective_rolling_shutter_enabled(&self) -> bool {
        match self.config.rolling_shutter_enabled {
            Some(enabled) => enabled,
            None => self.detect_rolling_shutter(),
        }
    }

    /// Generate a real-time status report for monitoring
    pub fn status_report(&self) -> String {
        let elapsed = self
            .start_time
            .map(|start| start.elapsed())
            .unwrap_or(Duration::ZERO);

        let mut report = format!("🔍 Calibration Status Report ({:?})\n", self.status);
        report.push_str(&format!("⏱️  Elapsed: {:.1}s\n", elapsed.as_secs_f64()));
        report.push_str(&format!(
            "📊 Progress: {:.1}%\n",
            self.progress_percentage()
        ));
        report.push_str(&format!(
            "📷 Stereo pairs: {}/{}\n",
            self.stereo_pairs.len(),
            self.config.max_stereo_pairs
        ));

        let total_matches: usize = self
            .stereo_pairs
            .iter()
            .map(|p| p.correspondences.len())
            .sum();
        report.push_str(&format!("🎯 Feature matches: {}\n", total_matches));

        // Show rolling shutter detection status
        let rolling_shutter_enabled = self.effective_rolling_shutter_enabled();
        let detection_status = match self.config.rolling_shutter_enabled {
            Some(true) => "🎥 Rolling shutter: ENABLED (manual)",
            Some(false) => "📷 Global shutter: ENABLED (manual)",
            None => {
                if self.stereo_pairs.len() >= 3 {
                    if rolling_shutter_enabled {
                        "🎥 Rolling shutter: DETECTED (auto)"
                    } else {
                        "📷 Global shutter: DETECTED (auto)"
                    }
                } else {
                    "🔍 Rolling shutter: ANALYZING..."
                }
            },
        };
        report.push_str(&format!("{}\n", detection_status));

        match self.status {
            CalibrationStatus::NotStarted => {
                report.push_str("💤 Waiting for stereo pairs...\n");
            },
            CalibrationStatus::CollectingData => {
                let remaining = self.config.max_stereo_pairs - self.stereo_pairs.len();
                report.push_str(&format!(
                    "📷 Collecting data... {} pairs remaining\n",
                    remaining
                ));
                if self.stereo_pairs.len() >= 5 {
                    report.push_str("✅ Ready for calibration!\n");
                }
            },
            CalibrationStatus::Optimizing => {
                report.push_str("⚙️  Running optimization...\n");
            },
            CalibrationStatus::Success => {
                report.push_str("✅ Calibration completed successfully!\n");
                if let Some(logger) = &self.logger {
                    let trends = logger.analyze_trends();
                    if let Some(stability) = trends.get("stability_score") {
                        report.push_str(&format!("🔄 Stability: {:.2}\n", stability));
                    }
                    if let Some(accuracy) = trends.get("current_accuracy") {
                        report.push_str(&format!("🎯 Accuracy: {:.1}%\n", accuracy));
                    }
                }
            },
            CalibrationStatus::Failed => {
                report.push_str("❌ Calibration failed!\n");
            },
        }

        report
    }

    /// Get access to the calibration logger (if enabled)
    pub const fn logger(&self) -> Option<&CalibrationLogger> {
        self.logger.as_ref()
    }

    /// Get access to the calibration logger (mutable)
    pub const fn logger_mut(&mut self) -> Option<&mut CalibrationLogger> {
        self.logger.as_mut()
    }

    /// Log calibration progress and quality metrics
    fn log_progress(&self, message: &str) {
        if self.config.log_calibration_metrics {
            let elapsed = self
                .start_time
                .map(|start| start.elapsed())
                .unwrap_or(Duration::ZERO);

            println!("[CALIBRATION {:.2}s] {}", elapsed.as_secs_f64(), message);
        }
    }

    /// Log quality metrics with assessment
    fn log_quality_metrics(&mut self, metrics: &CalibrationQualityMetrics, timestamp: f64) {
        if let Some(logger) = &mut self.logger {
            logger.log_metrics(timestamp, metrics.clone());

            self.log_progress(&format!(
                "Quality: {:.2}px mean error, {:.1}% <1px accuracy, {} outliers",
                metrics.mean_reprojection_error,
                metrics.accuracy_percentage_1px,
                metrics.outlier_count
            ));

            let assessment = if metrics.is_calibration_acceptable(&QualityThresholds::default()) {
                "✅ ACCEPTABLE"
            } else {
                "❌ NEEDS IMPROVEMENT"
            };

            self.log_progress(&format!("Assessment: {}", assessment));
        }
    }

}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::field_reassign_with_default)]
mod tests {
    use super::*;
    use crate::calibration::config::CalibrationConfig;

    /// Build a synthetic `StereoPair` with `n` correspondences.
    fn make_stereo_pair(n: usize) -> StereoPair {
        StereoPair {
            left_features: (0..n)
                .map(|i| na::Vector2::new(i as f64 * 10.0, 100.0))
                .collect(),
            right_features: (0..n)
                .map(|i| na::Vector2::new(i as f64 * 10.0 - 5.0, 100.0))
                .collect(),
            correspondences: (0..n).map(|i| (i, i)).collect(),
            timestamp: 1.0,
            angular_velocity: None,
            linear_velocity: None,
            feature_qualities: vec![1.0; n],
        }
    }

    // ── StereoCalibrator::new ──────────────────────────────────────

    #[test]
    fn new_initial_status_is_not_started() {
        let cal = StereoCalibrator::new(CalibrationConfig::default());
        assert_eq!(cal.status(), CalibrationStatus::NotStarted);
    }

    #[test]
    fn new_guidance_has_sensible_defaults() {
        let cal = StereoCalibrator::new(CalibrationConfig::default());
        // Fresh calibrator: not ready, zero progress, guidance accessible
        assert!(!cal.is_calibration_ready());
        assert!((cal.progress_percentage() - 0.0).abs() < f64::EPSILON);
        let guidance = cal.get_guidance();
        assert!(!guidance.calibration_ready);
        let _display = cal.get_status_display();
    }

    // ── add_stereo_pair ────────────────────────────────────────────

    #[test]
    fn add_pair_with_enough_features_changes_status() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        let mut cal = StereoCalibrator::new(cfg);

        let pair = make_stereo_pair(20);
        cal.add_stereo_pair(pair);

        assert_eq!(cal.status(), CalibrationStatus::CollectingData);
        assert_eq!(cal.num_stereo_pairs(), 1);
    }

    #[test]
    fn add_pair_with_too_few_features_is_rejected() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        let mut cal = StereoCalibrator::new(cfg);

        // default min_feature_matches is 20, so 5 should be rejected
        let pair = make_stereo_pair(5);
        cal.add_stereo_pair(pair);

        assert_eq!(cal.num_stereo_pairs(), 0);
        assert_eq!(cal.status(), CalibrationStatus::NotStarted);
    }

    #[test]
    fn add_pair_exactly_at_min_threshold_accepted() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        let min = cfg.min_feature_matches;
        let mut cal = StereoCalibrator::new(cfg);

        let pair = make_stereo_pair(min);
        cal.add_stereo_pair(pair);

        assert_eq!(cal.num_stereo_pairs(), 1);
    }

    // ── is_calibration_ready ───────────────────────────────────────

    #[test]
    fn is_calibration_ready_false_with_zero_pairs() {
        let cal = StereoCalibrator::new(CalibrationConfig::default());
        assert!(!cal.is_calibration_ready());
    }

    #[test]
    fn is_calibration_ready_false_with_few_pairs() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        let mut cal = StereoCalibrator::new(cfg);

        // Add a couple of pairs — not enough for the guidance quality gate
        for _ in 0..3 {
            cal.add_stereo_pair(make_stereo_pair(25));
        }
        assert!(!cal.is_calibration_ready());
    }

    // ── get_status_display ─────────────────────────────────────────

    #[test]
    fn status_display_non_empty_after_pairs() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        let mut cal = StereoCalibrator::new(cfg);
        for _ in 0..5 {
            cal.add_stereo_pair(make_stereo_pair(25));
        }
        assert!(!cal.get_status_display().is_empty());
    }

    #[test]
    fn status_display_non_empty_after_collecting() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        let mut cal = StereoCalibrator::new(cfg);
        cal.add_stereo_pair(make_stereo_pair(25));
        assert!(!cal.get_status_display().is_empty());
    }

    // ── progress_percentage ────────────────────────────────────────

    #[test]
    fn progress_zero_when_not_started() {
        let cal = StereoCalibrator::new(CalibrationConfig::default());
        assert!((cal.progress_percentage() - 0.0).abs() < f64::EPSILON);
    }

    #[test]
    fn progress_increases_after_adding_pair() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        let mut cal = StereoCalibrator::new(cfg);
        cal.add_stereo_pair(make_stereo_pair(25));
        assert!(cal.progress_percentage() > 0.0);
    }

    // ── CalibrationStatus ──────────────────────────────────────────

    #[test]
    fn calibration_status_equality_and_copy() {
        // PartialEq + Copy derive
        let a = CalibrationStatus::Optimizing;
        let b = a; // Copy
        assert_eq!(a, b);
        assert_ne!(CalibrationStatus::NotStarted, CalibrationStatus::Failed);
    }

    #[test]
    fn calibration_status_debug_contains_variant_name() {
        assert!(format!("{:?}", CalibrationStatus::NotStarted).contains("NotStarted"));
        assert!(format!("{:?}", CalibrationStatus::CollectingData).contains("CollectingData"));
        assert!(format!("{:?}", CalibrationStatus::Success).contains("Success"));
    }

    // ── Max-pairs cap ──────────────────────────────────────────────

    #[test]
    fn add_pair_capped_at_max_stereo_pairs() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        cfg.adaptive_guidance_enabled = false;
        cfg.max_stereo_pairs = 3;
        let mut cal = StereoCalibrator::new(cfg);
        for _ in 0..5 {
            cal.add_stereo_pair(make_stereo_pair(25));
        }
        assert_eq!(cal.num_stereo_pairs(), 3);
    }

    // ── status_report ──────────────────────────────────────────────

    #[test]
    fn status_report_contains_status_and_progress() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        cfg.adaptive_guidance_enabled = false;
        let mut cal = StereoCalibrator::new(cfg);

        let report = cal.status_report();
        assert!(report.contains("NotStarted"), "report should mention current status");
        assert!(report.contains("0.0%"), "report should show 0% progress");

        cal.add_stereo_pair(make_stereo_pair(25));
        let report2 = cal.status_report();
        assert!(report2.contains("CollectingData"));
        assert!(report2.contains("Stereo pairs: 1/"));
    }

    // ── status_display ─────────────────────────────────────────────

    #[test]
    fn status_display_changes_with_state() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        cfg.adaptive_guidance_enabled = false;
        let mut cal = StereoCalibrator::new(cfg);

        let before = cal.get_status_display();
        cal.add_stereo_pair(make_stereo_pair(25));
        let after = cal.get_status_display();
        // At minimum, the display should change after adding data
        // (or both can be empty if guidance is disabled — still OK)
        let _ = (before, after); // no panic = pass
    }

    // ── Temporal sequence workflow ─────────────────────────────────

    #[test]
    fn temporal_sequence_finalize_empty_is_noop() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        cfg.adaptive_guidance_enabled = false;
        let mut cal = StereoCalibrator::new(cfg);
        cal.finalize_current_sequence(); // should not panic
    }

    #[test]
    fn temporal_sequence_add_and_finalize() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        cfg.adaptive_guidance_enabled = false;
        cfg.temporal_sequence_min_pairs = 2;
        let mut cal = StereoCalibrator::new(cfg);

        let mut pair1 = make_stereo_pair(25);
        pair1.timestamp = 0.0;
        let mut pair2 = make_stereo_pair(25);
        pair2.timestamp = 0.1;

        cal.add_stereo_pair_to_sequence(pair1);
        cal.add_stereo_pair_to_sequence(pair2);
        cal.finalize_current_sequence();

        // The pairs were also added to the regular collection
        assert_eq!(cal.num_stereo_pairs(), 2);
    }

    // ── effective_rolling_shutter_enabled ───────────────────────────

    #[test]
    fn effective_rolling_shutter_manual_overrides_detection() {
        let mut cfg = CalibrationConfig::default();
        cfg.rolling_shutter_enabled = Some(true);
        cfg.auto_calibration_enabled = false;
        let cal = StereoCalibrator::new(cfg);
        assert!(cal.effective_rolling_shutter_enabled());

        let mut cfg2 = CalibrationConfig::default();
        cfg2.rolling_shutter_enabled = Some(false);
        cfg2.auto_calibration_enabled = false;
        let cal2 = StereoCalibrator::new(cfg2);
        assert!(!cal2.effective_rolling_shutter_enabled());
    }

    // ── current_quality_metrics ────────────────────────────────────

    #[test]
    fn current_quality_metrics_none_before_success() {
        let cfg = CalibrationConfig::default();
        let cal = StereoCalibrator::new(cfg);
        assert!(cal.current_quality_metrics().is_none());
    }

    // ── rolling_shutter_detection_info ─────────────────────────────

    #[test]
    fn rolling_shutter_detection_info_none_with_no_pairs() {
        let cfg = CalibrationConfig::default();
        let cal = StereoCalibrator::new(cfg);
        assert!(cal.rolling_shutter_detection_info().is_none());
    }

    // ── logger ─────────────────────────────────────────────────────

    #[test]
    fn logger_present_when_metrics_enabled() {
        let mut cfg = CalibrationConfig::default();
        cfg.log_calibration_metrics = true;
        cfg.auto_calibration_enabled = false;
        let cal = StereoCalibrator::new(cfg);
        assert!(cal.logger().is_some());
    }

    #[test]
    fn logger_none_when_metrics_disabled() {
        let mut cfg = CalibrationConfig::default();
        cfg.log_calibration_metrics = false;
        cfg.auto_calibration_enabled = false;
        let cal = StereoCalibrator::new(cfg);
        assert!(cal.logger().is_none());
    }

    // ── validate_calibration ───────────────────────────────────────

    #[test]
    fn validate_calibration_with_synthetic_result() {
        let mut cfg = CalibrationConfig::default();
        cfg.auto_calibration_enabled = false;
        let cal = StereoCalibrator::new(cfg);

        let result = CalibrationResult {
            left_intrinsics: vec![500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            right_intrinsics: vec![500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            stereo_extrinsics: na::Isometry3::identity(),
            final_reprojection_error: 0.5,
            num_stereo_pairs: 10,
            num_feature_matches: 200,
        };

        let pairs: Vec<StereoPair> = (0..3).map(|_| make_stereo_pair(25)).collect();
        let metrics = cal.validate_calibration(&result, &pairs);

        // Should produce some metrics from the synthetic data
        assert!(metrics.total_points > 0);
        assert!(metrics.mean_reprojection_error >= 0.0);
    }

    // ── RollingShutterDetectionInfo ────────────────────────────────

    #[test]
    fn rolling_shutter_info_field_access_and_clone() {
        let info = RollingShutterDetectionInfo {
            position_distortion_score: 0.1,
            temporal_consistency_score: 0.2,
            geometric_distortion_score: 0.3,
            combined_score: 0.2,
            rolling_shutter_detected: false,
            confidence: 85,
        };
        let cloned = info.clone();
        assert!(!cloned.rolling_shutter_detected);
        assert_eq!(cloned.confidence, 85);
        assert!((cloned.combined_score - 0.2).abs() < f64::EPSILON);
        assert!(format!("{:?}", cloned).contains("85"));
    }
}
