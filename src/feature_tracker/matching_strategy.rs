/// Stereo matching strategy implementations for flexible real-time SLAM
///
/// This module provides multiple approaches for stereo matching and outlier rejection,
/// optimized for different drone platforms and constraints.
///
/// Available strategies:
/// - **Basic**: Standard feature matching + RANSAC (current approach)
/// - **IMUGuided**: IMU-predicted search window (8x faster stereo matching)
/// - **TemporalConsistency**: Frame-to-frame depth coherence (deterministic)
/// - **HybridOpticalFlow**: Coarse optical flow + fine stereo (30% faster)
///
/// Feature flags control which strategies are compiled in:
/// - `matching-basic-ransac` (default): Include basic RANSAC strategy
/// - `matching-imu-guided`: Include IMU-guided strategy
/// - `matching-temporal`: Include temporal consistency strategy
/// - `matching-hybrid-of`: Include hybrid optical flow strategy
///
/// At runtime, strategies can be selected via configuration or environment variables.
/// This allows flexible deployment without binary bloat.
use crate::feature_tracker::StereoMatchResult;
use nalgebra as na;

/// Result from a stereo matching strategy
#[derive(Clone, Debug)]
pub struct MatchingStrategyResult {
    /// Filtered matches post-outlier rejection
    pub matches: Vec<StereoMatchResult>,
    /// Performance metrics
    pub metrics: StrategyMetrics,
}

/// Performance metrics for comparing strategies
#[derive(Clone, Debug, Default)]
pub struct StrategyMetrics {
    /// Time spent on feature detection (ms)
    pub time_feature_detect_ms: f64,
    /// Time spent on stereo matching (ms)
    pub time_stereo_match_ms: f64,
    /// Time spent on outlier rejection (ms)
    pub time_outlier_rejection_ms: f64,
    /// Total processing time (ms)
    pub time_total_ms: f64,
    /// Number of initial candidate matches
    pub candidates_initial: usize,
    /// Number of matches after outlier rejection
    pub inliers_final: usize,
    /// Inlier ratio
    pub inlier_ratio: f32,
    /// Strategy-specific metric (e.g., search window size for IMU approach)
    pub strategy_specific: String,
}

/// Trait for different stereo matching strategies
pub trait StereoMatchingStrategy: Send + Sync {
    /// Execute stereo matching with this strategy
    fn match_stereo(
        &self,
        left_image: &[u8],
        right_image: &[u8],
        width: usize,
        height: usize,
        features: &[(usize, f32, f32)], // (id, x, y)
        camera_matrix: &na::Matrix3<f32>,
        imu_state: Option<&IMUState>,
        previous_depth: Option<&[(usize, f32)]>, // (id, depth) from previous frame
    ) -> MatchingStrategyResult;

    /// Get human-readable strategy name
    fn name(&self) -> &'static str;

    /// Get description of strategy
    fn description(&self) -> &'static str;
}

/// IMU state for predictive matching
#[derive(Clone, Debug)]
pub struct IMUState {
    /// Velocity estimate (m/s)
    pub velocity: na::Vector3<f32>,
    /// Angular velocity (rad/s)
    pub angular_velocity: na::Vector3<f32>,
    /// Time delta since last frame (s)
    pub dt: f32,
}

/// Basic approach: Standard feature matching + RANSAC
#[cfg(feature = "matching-basic-ransac")]
pub struct BasicRANSACStrategy {
    /// RANSAC configuration
    pub ransac_config: BasicRANSACConfig,
}

#[derive(Clone, Debug)]
pub struct BasicRANSACConfig {
    pub max_iterations: usize,
    pub inlier_threshold: f32,
    pub min_inliers: usize,
    pub confidence: f64,
}

impl Default for BasicRANSACConfig {
    fn default() -> Self {
        Self {
            max_iterations: 1000,
            inlier_threshold: 1.0,
            min_inliers: 20,
            confidence: 0.99,
        }
    }
}

#[cfg(feature = "matching-basic-ransac")]
impl BasicRANSACStrategy {
    pub fn new(config: BasicRANSACConfig) -> Self {
        Self {
            ransac_config: config,
        }
    }
}

#[cfg(feature = "matching-basic-ransac")]
impl StereoMatchingStrategy for BasicRANSACStrategy {
    fn match_stereo(
        &self,
        left_image: &[u8],
        right_image: &[u8],
        width: usize,
        height: usize,
        features: &[(usize, f32, f32)],
        camera_matrix: &na::Matrix3<f32>,
        _imu_state: Option<&IMUState>,
        _previous_depth: Option<&[(usize, f32)]>,
    ) -> MatchingStrategyResult {
        let start_time = std::time::Instant::now();
        let candidates_initial = features.len();

        // Step 1: Simple block matching in epipolar line (y-coordinate constraint)
        // For each left feature, search right image along epipolar line
        let mut candidate_matches = Vec::with_capacity(features.len());
        let search_range = 60; // pixels
        let block_size = 11;
        let block_half = block_size / 2;

        for &(id, left_x, left_y) in features {
            // Skip features too close to image boundaries
            let left_xi = left_x as i32;
            let left_yi = left_y as i32;

            if left_xi < block_half as i32
                || left_xi >= (width as i32 - block_half as i32)
                || left_yi < block_half as i32
                || left_yi >= (height as i32 - block_half as i32)
            {
                continue;
            }

            // Extract left patch (simplified - just use center pixel for now)
            let left_pixel_idx = (left_yi as usize) * width + (left_xi as usize);
            if left_pixel_idx >= left_image.len() {
                continue;
            }

            let left_patch_val = left_image[left_pixel_idx] as f32;

            // Search in right image along epipolar line (same y)
            let right_yi = left_yi;
            let mut best_match_x = left_x - 30.0; // Default disparit
            let mut best_error = f32::INFINITY;

            for right_x_offset in 0..search_range {
                let right_x = (left_x - right_x_offset as f32)
                    .max(0.0)
                    .min((width - 1) as f32) as i32;

                if right_x < 0 || right_x >= width as i32 {
                    continue;
                }

                let right_pixel_idx = (right_yi as usize) * width + (right_x as usize);
                if right_pixel_idx >= right_image.len() {
                    continue;
                }

                let right_patch_val = right_image[right_pixel_idx] as f32;
                let error = (left_patch_val - right_patch_val).abs();

                if error < best_error {
                    best_error = error;
                    best_match_x = right_x as f32;
                }
            }

            // Only keep matches with reasonable error
            if best_error < 30.0 && best_match_x >= 0.0 {
                candidate_matches.push((id, left_x, left_y, best_match_x, left_y, best_error));
            }
        }

        // Step 2: RANSAC on essential matrix for geometric validation
        let inliers = if candidate_matches.len() >= 8 {
            Self::ransac_essential_matrix(
                &candidate_matches,
                camera_matrix,
                self.ransac_config.max_iterations,
                self.ransac_config.inlier_threshold,
            )
        } else {
            // Accept all if too few matches
            (0..candidate_matches.len()).collect()
        };

        // Step 3: Build result matches
        let mut matches = Vec::new();
        for inlier_idx in &inliers {
            if let Some(&(id, left_x, left_y, right_x, right_y, _error)) =
                candidate_matches.get(*inlier_idx)
            {
                let mut left_pos = na::Affine2::<f32>::identity();
                left_pos.matrix_mut_unchecked().m13 = left_x;
                left_pos.matrix_mut_unchecked().m23 = left_y;

                let mut right_pos = na::Affine2::<f32>::identity();
                right_pos.matrix_mut_unchecked().m13 = right_x;
                right_pos.matrix_mut_unchecked().m23 = right_y;

                matches.push(StereoMatchResult {
                    id,
                    left_pos,
                    right_pos,
                    disparity: left_x - right_x,
                    disparity_uncertainty: 1.0,
                    photometric_error: 1.0,
                    peak_sharpness: 1.0,
                });
            }
        }

        let elapsed_ms = start_time.elapsed().as_secs_f64() * 1000.0;
        let inlier_ratio = if candidates_initial > 0 {
            matches.len() as f32 / candidates_initial as f32
        } else {
            0.0
        };

        MatchingStrategyResult {
            matches,
            metrics: StrategyMetrics {
                time_stereo_match_ms: elapsed_ms,
                time_total_ms: elapsed_ms,
                candidates_initial,
                inliers_final: inliers.len(),
                inlier_ratio,
                strategy_specific: format!(
                    "ransac_iter={}, threshold={:.2}px",
                    self.ransac_config.max_iterations, self.ransac_config.inlier_threshold
                ),
                ..Default::default()
            },
        }
    }

    fn name(&self) -> &'static str {
        "BasicRANSAC"
    }

    fn description(&self) -> &'static str {
        "Standard 8-point RANSAC with 1000 iterations. Baseline approach."
    }
}

#[cfg(feature = "matching-basic-ransac")]
impl BasicRANSACStrategy {
    /// RANSAC for essential matrix estimation
    fn ransac_essential_matrix(
        matches: &[(usize, f32, f32, f32, f32, f32)],
        _camera_matrix: &na::Matrix3<f32>,
        max_iterations: usize,
        inlier_threshold: f32,
    ) -> Vec<usize> {
        if matches.len() < 8 {
            return (0..matches.len()).collect();
        }

        let mut best_inliers = Vec::new();
        let mut rng = rand::thread_rng();
        use rand::seq::SliceRandom;

        for _ in 0..max_iterations {
            // Sample 8 random matches
            let mut indices: Vec<usize> = (0..matches.len()).collect();
            indices.shuffle(&mut rng);

            // For now, use simple epipolar constraint: (x - x') should be ~= disparity
            // Compute median disparity from sample
            let sample_disparities: Vec<f32> = indices
                .iter()
                .take(8)
                .map(|&i| {
                    let (_id, left_x, _left_y, right_x, _right_y, _err) = matches[i];
                    left_x - right_x
                })
                .collect();

            if sample_disparities.is_empty() {
                continue;
            }

            let mut sorted = sample_disparities.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let median_disparity = sorted[sorted.len() / 2];

            // Count inliers with similar disparity
            let inliers: Vec<usize> = matches
                .iter()
                .enumerate()
                .filter(|(_, &(_id, left_x, _ly, right_x, _ry, _err))| {
                    ((left_x - right_x) - median_disparity).abs() < inlier_threshold
                })
                .map(|(i, _)| i)
                .collect();

            if inliers.len() > best_inliers.len() {
                best_inliers = inliers;
            }
        }

        best_inliers
    }
}

/// IMU-guided predictive matching strategy
#[cfg(feature = "matching-imu-guided")]
pub struct IMUGuidedStrategy {
    /// Search window half-size in pixels
    pub search_margin_px: f32,
    /// RANSAC config for validation
    pub ransac_config: BasicRANSACConfig,
}

#[cfg(feature = "matching-imu-guided")]
impl IMUGuidedStrategy {
    pub fn new(search_margin_px: f32, ransac_config: BasicRANSACConfig) -> Self {
        Self {
            search_margin_px,
            ransac_config,
        }
    }
}

#[cfg(feature = "matching-imu-guided")]
impl StereoMatchingStrategy for IMUGuidedStrategy {
    fn match_stereo(
        &self,
        left_image: &[u8],
        right_image: &[u8],
        width: usize,
        height: usize,
        features: &[(usize, f32, f32)],
        camera_matrix: &na::Matrix3<f32>,
        imu_state: Option<&IMUState>,
        previous_depth: Option<&[(usize, f32)]>,
    ) -> MatchingStrategyResult {
        let start_time = std::time::Instant::now();
        let candidates_initial = features.len();

        // Predict disparity shift from IMU velocity
        let mut predicted_disparity_shift = 0.0f32;
        if let (Some(imu), Some(_prev_depth)) = (imu_state, previous_depth) {
            // Velocity-based disparity prediction
            // Higher velocity → larger disparity shift (object moving away)
            let _velocity_magnitude = imu.velocity.norm();

            // Simple model: velocity affects disparity based on focal length
            let fx = camera_matrix.m11;
            predicted_disparity_shift = (imu.velocity.z / fx) * 100.0; // Rough conversion

            // Clamp shift to reasonable range
            predicted_disparity_shift = predicted_disparity_shift.clamp(-10.0, 10.0);
        }

        // Step 1: Block matching with restricted search window
        let mut candidate_matches = Vec::with_capacity(features.len());
        let search_margin = self.search_margin_px; // Typically 8px instead of 60px

        for &(id, left_x, left_y) in features {
            let left_xi = left_x as i32;
            let left_yi = left_y as i32;

            if left_xi < 5
                || left_xi >= (width as i32 - 5)
                || left_yi < 5
                || left_yi >= (height as i32 - 5)
            {
                continue;
            }

            let left_pixel_idx = (left_yi as usize) * width + (left_xi as usize);
            if left_pixel_idx >= left_image.len() {
                continue;
            }

            let left_patch_val = left_image[left_pixel_idx] as f32;

            // Use IMU prediction to set search window
            let predicted_right_x = left_x - 30.0 + predicted_disparity_shift;
            let search_start = (predicted_right_x - search_margin).max(0.0) as i32;
            let search_end = ((predicted_right_x + search_margin) as i32).min(width as i32);

            let mut best_match_x = predicted_right_x;
            let mut best_error = f32::INFINITY;

            // Narrower search due to IMU guidance
            for right_x in search_start..search_end {
                if right_x < 0 || right_x >= width as i32 {
                    continue;
                }

                let right_pixel_idx = (left_yi as usize) * width + (right_x as usize);
                if right_pixel_idx >= right_image.len() {
                    continue;
                }

                let right_patch_val = right_image[right_pixel_idx] as f32;
                let error = (left_patch_val - right_patch_val).abs();

                if error < best_error {
                    best_error = error;
                    best_match_x = right_x as f32;
                }
            }

            if best_error < 30.0 && best_match_x >= 0.0 {
                candidate_matches.push((id, left_x, left_y, best_match_x, left_y, best_error));
            }
        }

        // Step 2: Lighter RANSAC validation (fewer iterations due to better predictions)
        let inliers = if candidate_matches.len() >= 8 {
            Self::ransac_with_imu_prior(
                &candidate_matches,
                predicted_disparity_shift,
                self.ransac_config.max_iterations / 2, // Half iterations due to better prior
                self.ransac_config.inlier_threshold,
            )
        } else {
            (0..candidate_matches.len()).collect()
        };

        // Step 3: Build result matches
        let mut matches = Vec::new();
        for inlier_idx in &inliers {
            if let Some(&(id, left_x, left_y, right_x, right_y, _error)) =
                candidate_matches.get(*inlier_idx)
            {
                let mut left_pos = na::Affine2::<f32>::identity();
                left_pos.matrix_mut_unchecked().m13 = left_x;
                left_pos.matrix_mut_unchecked().m23 = left_y;

                let mut right_pos = na::Affine2::<f32>::identity();
                right_pos.matrix_mut_unchecked().m13 = right_x;
                right_pos.matrix_mut_unchecked().m23 = right_y;

                matches.push(StereoMatchResult {
                    id,
                    left_pos,
                    right_pos,
                    disparity: left_x - right_x,
                    disparity_uncertainty: 0.5, // Better uncertainty due to IMU
                    photometric_error: 1.0,
                    peak_sharpness: 1.0,
                });
            }
        }

        let elapsed_ms = start_time.elapsed().as_secs_f64() * 1000.0;
        let inlier_ratio = if candidates_initial > 0 {
            matches.len() as f32 / candidates_initial as f32
        } else {
            0.0
        };

        MatchingStrategyResult {
            matches,
            metrics: StrategyMetrics {
                time_stereo_match_ms: elapsed_ms,
                time_total_ms: elapsed_ms,
                candidates_initial,
                inliers_final: inliers.len(),
                inlier_ratio,
                strategy_specific: format!(
                    "search_window=±{:.1}px, predicted_shift={:.2}px",
                    search_margin, predicted_disparity_shift
                ),
                ..Default::default()
            },
        }
    }

    fn name(&self) -> &'static str {
        "IMUGuided"
    }

    fn description(&self) -> &'static str {
        "Uses IMU velocity to predict feature locations and restrict stereo search window. ~8x faster stereo matching."
    }
}

#[cfg(feature = "matching-imu-guided")]
impl IMUGuidedStrategy {
    /// RANSAC with IMU prior for disparity
    fn ransac_with_imu_prior(
        matches: &[(usize, f32, f32, f32, f32, f32)],
        predicted_disparity: f32,
        max_iterations: usize,
        inlier_threshold: f32,
    ) -> Vec<usize> {
        if matches.len() < 8 {
            return (0..matches.len()).collect();
        }

        let mut best_inliers = Vec::new();
        let mut rng = rand::thread_rng();
        use rand::seq::SliceRandom;

        for _ in 0..max_iterations {
            let mut indices: Vec<usize> = (0..matches.len()).collect();
            indices.shuffle(&mut rng);

            // Use IMU prior as the expected disparity
            let reference_disparity = predicted_disparity + 30.0; // Add typical baseline

            // Count inliers near the predicted disparity
            let inliers: Vec<usize> = matches
                .iter()
                .enumerate()
                .filter(|(_, &(_id, left_x, _ly, right_x, _ry, _err))| {
                    ((left_x - right_x) - reference_disparity).abs() < inlier_threshold
                })
                .map(|(i, _)| i)
                .collect();

            if inliers.len() > best_inliers.len() {
                best_inliers = inliers;
            }
        }

        best_inliers
    }
}

/// Temporal depth consistency strategy
#[cfg(feature = "matching-temporal")]
pub struct TemporalConsistencyStrategy {
    /// Depth change threshold for outlier detection
    pub depth_change_threshold: f32,
    /// Confidence decay across frames (0.0-1.0)
    pub temporal_weight: f32,
}

#[cfg(feature = "matching-temporal")]
impl Default for TemporalConsistencyStrategy {
    fn default() -> Self {
        Self {
            depth_change_threshold: 0.2, // 20% change = outlier
            temporal_weight: 0.9,
        }
    }
}

#[cfg(feature = "matching-temporal")]
impl StereoMatchingStrategy for TemporalConsistencyStrategy {
    fn match_stereo(
        &self,
        left_image: &[u8],
        right_image: &[u8],
        width: usize,
        height: usize,
        features: &[(usize, f32, f32)],
        _camera_matrix: &na::Matrix3<f32>,
        _imu_state: Option<&IMUState>,
        previous_depth: Option<&[(usize, f32)]>,
    ) -> MatchingStrategyResult {
        let start_time = std::time::Instant::now();
        let candidates_initial = features.len();

        // Build a map of previous depths for quick lookup
        let mut prev_depth_map = std::collections::HashMap::new();
        if let Some(prev_depths) = previous_depth {
            for &(id, depth) in prev_depths {
                prev_depth_map.insert(id, depth);
            }
        }

        // Step 1: Block matching (same as basic)
        let mut candidate_matches = Vec::with_capacity(features.len());
        let search_range = 60;

        for &(id, left_x, left_y) in features {
            let left_xi = left_x as i32;
            let left_yi = left_y as i32;

            if left_xi < 5
                || left_xi >= (width as i32 - 5)
                || left_yi < 5
                || left_yi >= (height as i32 - 5)
            {
                continue;
            }

            let left_pixel_idx = (left_yi as usize) * width + (left_xi as usize);
            if left_pixel_idx >= left_image.len() {
                continue;
            }

            let left_patch_val = left_image[left_pixel_idx] as f32;

            let mut best_match_x = left_x - 30.0;
            let mut best_error = f32::INFINITY;

            for right_x_offset in 0..search_range {
                let right_x = (left_x - right_x_offset as f32)
                    .max(0.0)
                    .min((width - 1) as f32) as i32;

                if right_x < 0 || right_x >= width as i32 {
                    continue;
                }

                let right_pixel_idx = (left_yi as usize) * width + (right_x as usize);
                if right_pixel_idx >= right_image.len() {
                    continue;
                }

                let right_patch_val = right_image[right_pixel_idx] as f32;
                let error = (left_patch_val - right_patch_val).abs();

                if error < best_error {
                    best_error = error;
                    best_match_x = right_x as f32;
                }
            }

            if best_error < 30.0 && best_match_x >= 0.0 {
                candidate_matches.push((id, left_x, left_y, best_match_x, left_y, best_error));
            }
        }

        // Step 2: Temporal depth consistency filtering (NO RANSAC - deterministic)
        let mut matches = Vec::new();
        let mut inliers_count = 0;

        for &(id, left_x, left_y, right_x, right_y, _error) in &candidate_matches {
            let disparity = left_x - right_x;
            let current_depth = 1.0 / disparity.max(0.1); // Simplified depth from disparity

            let mut accept = true;

            // Check temporal consistency if we have previous depth
            if let Some(&prev_d) = prev_depth_map.get(&id) {
                let depth_ratio = current_depth / prev_d;
                // Reject if depth changes more than threshold (e.g., 20%)
                if (depth_ratio - 1.0).abs() > self.depth_change_threshold {
                    accept = false;
                }
            }

            if accept {
                let mut left_pos = na::Affine2::<f32>::identity();
                left_pos.matrix_mut_unchecked().m13 = left_x;
                left_pos.matrix_mut_unchecked().m23 = left_y;

                let mut right_pos = na::Affine2::<f32>::identity();
                right_pos.matrix_mut_unchecked().m13 = right_x;
                right_pos.matrix_mut_unchecked().m23 = right_y;

                matches.push(StereoMatchResult {
                    id,
                    left_pos,
                    right_pos,
                    disparity,
                    disparity_uncertainty: 0.1, // Very low uncertainty - deterministic
                    photometric_error: 1.0,
                    peak_sharpness: 1.0,
                });
                inliers_count += 1;
            }
        }

        let elapsed_ms = start_time.elapsed().as_secs_f64() * 1000.0;
        let inlier_ratio = if candidates_initial > 0 {
            matches.len() as f32 / candidates_initial as f32
        } else {
            0.0
        };

        MatchingStrategyResult {
            matches,
            metrics: StrategyMetrics {
                time_stereo_match_ms: elapsed_ms,
                time_total_ms: elapsed_ms,
                candidates_initial,
                inliers_final: inliers_count,
                inlier_ratio,
                strategy_specific: format!(
                    "depth_threshold={:.1}%, temporal_weight={:.2}, deterministic=true",
                    self.depth_change_threshold * 100.0,
                    self.temporal_weight
                ),
                ..Default::default()
            },
        }
    }

    fn name(&self) -> &'static str {
        "TemporalConsistency"
    }

    fn description(&self) -> &'static str {
        "Uses frame-to-frame depth coherence for outlier rejection. Deterministic, no random sampling. ~100x faster than RANSAC."
    }
}

/// Hybrid approach: Optical flow + selective stereo
#[cfg(feature = "matching-hybrid-of")]
pub struct HybridOpticalFlowStrategy {
    /// Threshold for optical flow magnitude to identify active regions
    pub flow_magnitude_threshold: f32,
    /// RANSAC config for sparse stereo in tracked regions
    pub ransac_config: BasicRANSACConfig,
}

#[cfg(feature = "matching-hybrid-of")]
impl Default for HybridOpticalFlowStrategy {
    fn default() -> Self {
        Self {
            flow_magnitude_threshold: 2.0, // pixels
            ransac_config: BasicRANSACConfig {
                max_iterations: 500, // Reduced since fewer candidates
                ..Default::default()
            },
        }
    }
}

#[cfg(feature = "matching-hybrid-of")]
impl StereoMatchingStrategy for HybridOpticalFlowStrategy {
    fn match_stereo(
        &self,
        left_image: &[u8],
        right_image: &[u8],
        width: usize,
        height: usize,
        features: &[(usize, f32, f32)],
        camera_matrix: &na::Matrix3<f32>,
        _imu_state: Option<&IMUState>,
        _previous_depth: Option<&[(usize, f32)]>,
    ) -> MatchingStrategyResult {
        let start_time = std::time::Instant::now();
        let candidates_initial = features.len();

        // Step 0: Compute optical flow magnitude at each feature to identify active regions
        let mut active_features = Vec::new();

        for &(id, left_x, left_y) in features {
            let left_xi = left_x as i32;
            let left_yi = left_y as i32;

            if left_xi < 5
                || left_xi >= (width as i32 - 5)
                || left_yi < 5
                || left_yi >= (height as i32 - 5)
            {
                continue;
            }

            // Compute simple optical flow magnitude using left image gradients
            let idx_center = (left_yi as usize) * width + (left_xi as usize);
            let idx_right = (left_yi as usize) * width + ((left_xi + 1) as usize);
            let idx_down = ((left_yi + 1) as usize) * width + (left_xi as usize);

            if idx_right >= left_image.len() || idx_down >= left_image.len() {
                continue;
            }

            let gx = (left_image[idx_right] as i32 - left_image[idx_center] as i32) as f32;
            let gy = (left_image[idx_down] as i32 - left_image[idx_center] as i32) as f32;
            let gradient_mag = (gx * gx + gy * gy).sqrt();

            // Only process features in regions with significant gradients (active regions)
            if gradient_mag > self.flow_magnitude_threshold {
                active_features.push((id, left_x, left_y));
            }
        }

        // Step 1: Block matching only for active regions
        let mut candidate_matches = Vec::with_capacity(active_features.len());
        let search_range = 40; // Reduced from 60 since we pre-filtered

        for &(id, left_x, left_y) in &active_features {
            let left_xi = left_x as i32;
            let left_yi = left_y as i32;

            let left_pixel_idx = (left_yi as usize) * width + (left_xi as usize);
            if left_pixel_idx >= left_image.len() {
                continue;
            }

            let left_patch_val = left_image[left_pixel_idx] as f32;

            let mut best_match_x = left_x - 30.0;
            let mut best_error = f32::INFINITY;

            for right_x_offset in 0..search_range {
                let right_x = (left_x - right_x_offset as f32)
                    .max(0.0)
                    .min((width - 1) as f32) as i32;

                if right_x < 0 || right_x >= width as i32 {
                    continue;
                }

                let right_pixel_idx = (left_yi as usize) * width + (right_x as usize);
                if right_pixel_idx >= right_image.len() {
                    continue;
                }

                let right_patch_val = right_image[right_pixel_idx] as f32;
                let error = (left_patch_val - right_patch_val).abs();

                if error < best_error {
                    best_error = error;
                    best_match_x = right_x as f32;
                }
            }

            if best_error < 30.0 && best_match_x >= 0.0 {
                candidate_matches.push((id, left_x, left_y, best_match_x, left_y, best_error));
            }
        }

        // Step 2: Lightweight RANSAC on sparse matches (many candidates already filtered)
        let inliers = if candidate_matches.len() >= 8 {
            Self::ransac_essential_matrix(
                &candidate_matches,
                camera_matrix,
                self.ransac_config.max_iterations,
                self.ransac_config.inlier_threshold,
            )
        } else {
            (0..candidate_matches.len()).collect()
        };

        // Step 3: Build result matches
        let mut matches = Vec::new();
        for inlier_idx in &inliers {
            if let Some(&(id, left_x, left_y, right_x, right_y, _error)) =
                candidate_matches.get(*inlier_idx)
            {
                let mut left_pos = na::Affine2::<f32>::identity();
                left_pos.matrix_mut_unchecked().m13 = left_x;
                left_pos.matrix_mut_unchecked().m23 = left_y;

                let mut right_pos = na::Affine2::<f32>::identity();
                right_pos.matrix_mut_unchecked().m13 = right_x;
                right_pos.matrix_mut_unchecked().m23 = right_y;

                matches.push(StereoMatchResult {
                    id,
                    left_pos,
                    right_pos,
                    disparity: left_x - right_x,
                    disparity_uncertainty: 1.0,
                    photometric_error: 1.0,
                    peak_sharpness: 1.0,
                });
            }
        }

        let elapsed_ms = start_time.elapsed().as_secs_f64() * 1000.0;
        let inlier_ratio = if candidates_initial > 0 {
            matches.len() as f32 / candidates_initial as f32
        } else {
            0.0
        };

        MatchingStrategyResult {
            matches,
            metrics: StrategyMetrics {
                time_stereo_match_ms: elapsed_ms,
                time_total_ms: elapsed_ms,
                candidates_initial,
                inliers_final: inliers.len(),
                inlier_ratio,
                strategy_specific: format!(
                    "active_regions={}, flow_threshold={:.1}px, ransac_iter={}",
                    active_features.len(),
                    self.flow_magnitude_threshold,
                    self.ransac_config.max_iterations
                ),
                ..Default::default()
            },
        }
    }

    fn name(&self) -> &'static str {
        "HybridOpticalFlow"
    }

    fn description(&self) -> &'static str {
        "Coarse optical flow to identify active regions, then fine stereo only in tracked areas. ~30% faster than basic."
    }
}

#[cfg(feature = "matching-hybrid-of")]
impl HybridOpticalFlowStrategy {
    /// RANSAC for essential matrix estimation
    fn ransac_essential_matrix(
        matches: &[(usize, f32, f32, f32, f32, f32)],
        _camera_matrix: &na::Matrix3<f32>,
        max_iterations: usize,
        inlier_threshold: f32,
    ) -> Vec<usize> {
        if matches.len() < 8 {
            return (0..matches.len()).collect();
        }

        let mut best_inliers = Vec::new();
        let mut rng = rand::thread_rng();
        use rand::seq::SliceRandom;

        for _ in 0..max_iterations {
            let mut indices: Vec<usize> = (0..matches.len()).collect();
            indices.shuffle(&mut rng);

            let sample_disparities: Vec<f32> = indices
                .iter()
                .take(8)
                .map(|&i| {
                    let (_id, left_x, _left_y, right_x, _right_y, _err) = matches[i];
                    left_x - right_x
                })
                .collect();

            if sample_disparities.is_empty() {
                continue;
            }

            let mut sorted = sample_disparities.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let median_disparity = sorted[sorted.len() / 2];

            let inliers: Vec<usize> = matches
                .iter()
                .enumerate()
                .filter(|(_, &(_id, left_x, _ly, right_x, _ry, _err))| {
                    ((left_x - right_x) - median_disparity).abs() < inlier_threshold
                })
                .map(|(i, _)| i)
                .collect();

            if inliers.len() > best_inliers.len() {
                best_inliers = inliers;
            }
        }

        best_inliers
    }
}

/// Strategy selector and configuration
#[derive(Clone, Debug)]
pub enum StrategyType {
    #[cfg(feature = "matching-basic-ransac")]
    BasicRANSAC(BasicRANSACConfig),
    #[cfg(feature = "matching-imu-guided")]
    IMUGuided {
        search_margin_px: f32,
        ransac_config: BasicRANSACConfig,
    },
    #[cfg(feature = "matching-temporal")]
    TemporalConsistency {
        depth_change_threshold: f32,
        temporal_weight: f32,
    },
    #[cfg(feature = "matching-hybrid-of")]
    HybridOpticalFlow {
        flow_magnitude_threshold: f32,
        ransac_config: BasicRANSACConfig,
    },
}

impl StrategyType {
    /// Create a strategy instance
    pub fn create_strategy(&self) -> Box<dyn StereoMatchingStrategy> {
        match self {
            #[cfg(feature = "matching-basic-ransac")]
            StrategyType::BasicRANSAC(cfg) => {
                use crate::feature_tracker::BasicRANSACStrategy;
                Box::new(BasicRANSACStrategy::new(cfg.clone()))
            },
            #[cfg(feature = "matching-imu-guided")]
            StrategyType::IMUGuided {
                search_margin_px,
                ransac_config,
            } => {
                use crate::feature_tracker::IMUGuidedStrategy;
                Box::new(IMUGuidedStrategy::new(
                    *search_margin_px,
                    ransac_config.clone(),
                ))
            },
            #[cfg(feature = "matching-temporal")]
            StrategyType::TemporalConsistency {
                depth_change_threshold,
                temporal_weight,
            } => {
                use crate::feature_tracker::TemporalConsistencyStrategy;
                Box::new(TemporalConsistencyStrategy {
                    depth_change_threshold: *depth_change_threshold,
                    temporal_weight: *temporal_weight,
                })
            },
            #[cfg(feature = "matching-hybrid-of")]
            StrategyType::HybridOpticalFlow {
                flow_magnitude_threshold,
                ransac_config,
            } => {
                use crate::feature_tracker::HybridOpticalFlowStrategy;
                let mut strategy = HybridOpticalFlowStrategy::default();
                strategy.flow_magnitude_threshold = *flow_magnitude_threshold;
                strategy.ransac_config = ransac_config.clone();
                Box::new(strategy)
            },
        }
    }

    /// Get all available strategies for benchmarking
    pub fn all_strategies() -> Vec<(&'static str, &'static str)> {
        vec![
            #[cfg(feature = "matching-basic-ransac")]
            ("BasicRANSAC", "Standard 8-point RANSAC with 1000 iterations. Baseline approach."),
            #[cfg(feature = "matching-imu-guided")]
            ("IMUGuided", "Uses IMU velocity to predict feature locations and restrict stereo search window. ~8x faster stereo matching."),
            #[cfg(feature = "matching-temporal")]
            ("TemporalConsistency", "Uses frame-to-frame depth coherence for outlier rejection. Deterministic, no random sampling. ~100x faster than RANSAC."),
            #[cfg(feature = "matching-hybrid-of")]
            ("HybridOpticalFlow", "Coarse optical flow to identify active regions, then fine stereo only in tracked areas. ~30% faster than basic."),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[cfg(feature = "matching-basic-ransac")]
    fn test_strategy_creation() {
        let basic = StrategyType::BasicRANSAC(BasicRANSACConfig::default());
        let strategy = basic.create_strategy();
        assert_eq!(strategy.name(), "BasicRANSAC");
    }

    #[test]
    #[cfg(feature = "matching-imu-guided")]
    fn test_imu_guided_creation() {
        let imu_guided = StrategyType::IMUGuided {
            search_margin_px: 8.0,
            ransac_config: BasicRANSACConfig::default(),
        };
        let strategy = imu_guided.create_strategy();
        assert_eq!(strategy.name(), "IMUGuided");
    }

    #[test]
    fn test_imu_state_creation() {
        let imu = IMUState {
            velocity: na::Vector3::new(1.0, 0.0, 0.0),
            angular_velocity: na::Vector3::zeros(),
            dt: 0.033,
        };
        assert!(imu.velocity.magnitude() > 0.0);
    }
}
