use crate::calibration::types::CalibrationResult;
use crate::evaluation::calibration_quality::CalibrationConfidenceFactors;
/// Adaptive fusion algorithm leveraging IMU + stereo super-resolution
///
/// This enhanced algorithm provides distance and speed-aware optimization
/// for maximal 3D resolution and accuracy across all operating regimes.
///
/// Now with calibration-aware confidence weighting for optimal performance.
use nalgebra::{Isometry3, Point3, Vector3};

/// Adaptive fusion configuration based on motion and distance
#[derive(Clone, Debug)]
pub struct AdaptiveFusionConfig {
    /// Enable IMU-guided super-resolution
    pub enable_imu_super_resolution: bool,

    /// Enable motion-aware depth optimization
    pub enable_motion_depth_optimization: bool,

    /// Distance threshold for near-field optimization (meters)
    pub near_field_threshold: f64,

    /// Distance threshold for far-field compensation (meters)
    pub far_field_threshold: f64,

    /// Speed threshold for adaptive patch sizing (m/s)
    pub speed_threshold: f32,
}

impl Default for AdaptiveFusionConfig {
    fn default() -> Self {
        Self {
            enable_imu_super_resolution: true,
            enable_motion_depth_optimization: true,
            near_field_threshold: 1.0,
            far_field_threshold: 5.0,
            speed_threshold: 1.0,
        }
    }
}

/// Optimized fusion algorithm with calibration-aware confidence weighting
pub struct OptimizedFusionAlgorithm {
    config: AdaptiveFusionConfig,
    /// Distance-aware confidence boosting
    distance_boost_factor: f32,
    /// Speed-aware confidence adjustment
    speed_adjustment: f32,
    /// Calibration result (optional, for quality-aware weighting)
    calibration: Option<CalibrationResult>,
    /// Calibration confidence factors
    calibration_confidence: CalibrationConfidenceFactors,
}

impl OptimizedFusionAlgorithm {
    pub fn new(config: AdaptiveFusionConfig) -> Self {
        Self {
            config,
            distance_boost_factor: 1.0,
            speed_adjustment: 1.0,
            calibration: None,
            calibration_confidence: CalibrationConfidenceFactors::default(),
        }
    }

    /// Create fusion algorithm with calibration result
    pub fn with_calibration(config: AdaptiveFusionConfig, calibration: CalibrationResult) -> Self {
        use crate::calibration::types::AcceptanceThresholds;

        let thresholds = AcceptanceThresholds::standard();
        let quality_report = calibration.quality_report(&thresholds);
        let calibration_confidence =
            CalibrationConfidenceFactors::from_quality_report(&quality_report);

        Self {
            config,
            distance_boost_factor: 1.0,
            speed_adjustment: 1.0,
            calibration: Some(calibration),
            calibration_confidence,
        }
    }

    /// Update calibration (e.g., after recalibration)
    pub fn update_calibration(&mut self, calibration: CalibrationResult) {
        use crate::calibration::types::AcceptanceThresholds;

        let thresholds = AcceptanceThresholds::standard();
        let quality_report = calibration.quality_report(&thresholds);
        self.calibration_confidence =
            CalibrationConfidenceFactors::from_quality_report(&quality_report);
        self.calibration = Some(calibration);
    }

    /// Adaptive confidence weighting: combines all signals INCLUDING calibration quality
    pub fn adaptive_confidence(
        &mut self,
        denoise_confidence: f32,
        super_res_confidence: f32,
        distance: f64,
        speed: f32,
    ) -> f32 {
        // Distance-aware: far field needs higher confidence
        let distance_factor = if distance < self.config.near_field_threshold {
            1.0 // Near field: standard confidence
        } else if distance < self.config.far_field_threshold {
            1.2 // Mid field: boost by 20%
        } else {
            1.5 // Far field: boost by 50%
        };

        // Speed-aware: faster motion needs careful confidence
        let speed_factor = if speed < self.config.speed_threshold {
            1.0 // Slow motion: standard
        } else {
            1.0 - ((speed - self.config.speed_threshold) * 0.1).min(0.4) // Reduce confidence for fast motion
        };

        // Calibration-aware: degrade confidence if calibration is poor
        let calibration_factor = self.calibration_confidence.overall_confidence;

        // Combined confidence: IMU-guided super-resolution boost × calibration quality
        // denoise_weight × f0_confidence × calibration = adaptive refinement
        let combined = (denoise_confidence
            * super_res_confidence
            * distance_factor
            * speed_factor
            * calibration_factor)
            .clamp(0.1, 1.0);

        self.distance_boost_factor = distance_factor;
        self.speed_adjustment = speed_factor;

        combined
    }

    /// Get calibration-aware weight for visual reprojection residuals
    pub fn visual_residual_weight(&self) -> f32 {
        self.calibration_confidence.visual_residual_weight()
    }

    /// Get calibration-aware weight for IMU preintegration residuals
    pub fn imu_residual_weight(&self) -> f32 {
        self.calibration_confidence.imu_residual_weight()
    }

    /// Check if rolling shutter correction should be used
    pub fn should_use_rolling_shutter(&self) -> bool {
        self.calibration_confidence.should_use_rolling_shutter()
    }

    /// Get robust outlier threshold multiplier based on calibration quality
    pub fn robust_threshold_multiplier(&self) -> f32 {
        self.calibration_confidence.robust_threshold_multiplier()
    }

    /// Subpixel refinement with motion adaptation
    pub fn adaptive_subpixel_refinement(
        &self,
        coords: (f64, f64),
        initial_error: f32,
        velocity_magnitude: f32,
        distance: f64,
    ) -> (f64, f64, f32) {
        // Base refinement quality
        let base_accuracy = 1.0 - (initial_error as f64).sqrt();

        // Distance-based accuracy adjustment
        let distance_adjusted = if distance < self.config.near_field_threshold {
            base_accuracy * 0.95 // Near field: slightly worse subpixel
        } else {
            base_accuracy * (1.0 + (distance / 5.0).powi(2) * 0.1)
        };

        // Speed-based accuracy adjustment
        let speed_factor = (1.0 - (velocity_magnitude as f64 * 0.05).min(0.5)).max(0.5);
        let final_accuracy = distance_adjusted * speed_factor;

        // Refined coordinates with adaptive refinement
        let refinement_strength = final_accuracy as f32;

        (coords.0, coords.1, refinement_strength)
    }

    /// Distance-aware depth optimization
    pub fn optimize_depth_for_distance(
        &self,
        position: Point3<f64>,
        triangulation_error: f64,
    ) -> f64 {
        let distance = position.coords.norm();

        // Far-field depth compensation: add weighted uncertainty
        if distance > self.config.far_field_threshold {
            let far_field_factor = 1.0 + ((distance - self.config.far_field_threshold) / 5.0) * 0.3;
            triangulation_error * far_field_factor
        } else {
            triangulation_error
        }
    }

    /// Speed-adaptive triangulation
    pub fn adaptive_triangulation_weight(&self, speed: f32, distance: f64) -> f32 {
        let base_weight = 1.0;

        // Slower motion: higher confidence in triangulation
        let speed_weight = 1.0 / (1.0 + speed * 0.2);

        // Distance consideration
        let distance_weight = if distance < self.config.near_field_threshold {
            1.0
        } else if distance < self.config.far_field_threshold {
            0.95
        } else {
            0.9
        };

        (base_weight * speed_weight * distance_weight).clamp(0.5, 1.5)
    }

    /// Predict pixel location using IMU-to-camera transform and IMU rotation
    ///
    /// This is THE critical integration point for IMU-aided tracking:
    /// 1. IMU predicts rotation ΔR in IMU frame
    /// 2. Transform to camera frame using T_IC
    /// 3. Project to get predicted pixel location
    ///
    /// Returns None if calibration not available
    pub fn predict_pixel_with_imu(
        &self,
        prev_pixel: (f64, f64),
        imu_delta_rotation: &Isometry3<f64>,
        camera_intrinsics_fx: f64,
        camera_intrinsics_fy: f64,
        camera_intrinsics_cx: f64,
        camera_intrinsics_cy: f64,
        camera_id: &str, // Which camera to use
    ) -> Option<(f64, f64)> {
        let calibration = self.calibration.as_ref()?;
        let camera_extrinsics = calibration.camera_imu_extrinsics.get(camera_id)?;
        let t_ic = &camera_extrinsics.extrinsics.t_imu_to_camera;

        // Back-project pixel to normalized ray in camera frame
        let u_prev = prev_pixel.0;
        let v_prev = prev_pixel.1;
        let x_norm = (u_prev - camera_intrinsics_cx) / camera_intrinsics_fx;
        let y_norm = (v_prev - camera_intrinsics_cy) / camera_intrinsics_fy;
        let ray_camera = Vector3::new(x_norm, y_norm, 1.0).normalize();

        // Transform ray to IMU frame
        let ray_imu = t_ic.inverse_transform_vector(&ray_camera);

        // Apply IMU rotation
        let ray_imu_rotated = imu_delta_rotation.rotation * ray_imu;

        // Transform rotated ray back to camera frame
        let ray_camera_rotated = t_ic.transform_vector(&ray_imu_rotated);

        // Project to pixel (homogeneous coordinates)
        let u_pred = camera_intrinsics_fx * (ray_camera_rotated.x / ray_camera_rotated.z)
            + camera_intrinsics_cx;
        let v_pred = camera_intrinsics_fy * (ray_camera_rotated.y / ray_camera_rotated.z)
            + camera_intrinsics_cy;

        Some((u_pred, v_pred))
    }

    /// Compute per-row rolling shutter capture time
    ///
    /// Returns capture time offset for a given image row
    pub fn rolling_shutter_time_offset(
        &self,
        row: usize,
        image_height: usize,
        camera_id: &str,
    ) -> Option<f64> {
        let calibration = self.calibration.as_ref()?;
        let rs_result = calibration.rolling_shutter.get(camera_id)?;

        // Only apply if RS correction is confident AND RS is significant
        if !self.should_use_rolling_shutter() || !rs_result.is_significant {
            return Some(0.0); // Treat as global shutter
        }

        let t_readout = rs_result.readout_time;

        // Per-row capture time: t = t_frame_start + (row/H) * t_readout
        let time_offset = (row as f64 / image_height as f64) * t_readout;
        Some(time_offset)
    }

    /// Combined 3D resolution improvement
    pub fn estimate_3d_resolution(
        &self,
        baseline_uncertainty: f64,
        denoise_factor: f32,
        super_res_factor: f32,
        distance: f64,
        speed: f32,
    ) -> f64 {
        let base_improvement =
            baseline_uncertainty * denoise_factor as f64 * super_res_factor as f64;

        // Apply distance and speed adjustments
        let adaptive_improvement = base_improvement
            * (1.0 - (distance / 10.0).min(0.5)) // Distance penalty
            * (1.0 - (speed as f64) * 0.05).max(0.7); // Speed penalty

        adaptive_improvement
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calibration::types::{
        CalibrationResult, CameraCalibrationResult, CameraIMUCalibrationResult,
        CameraIMUExtrinsics, CameraIntrinsics, IMUCalibrationResult, IMUIntrinsics,
        RollingShutterDetectionResult, TimingQuality,
    };
    use nalgebra::{Matrix3, Vector3};
    use std::collections::HashMap;

    fn create_test_calibration(overall_score: f64, rs_significant: bool) -> CalibrationResult {
        let mut cameras = HashMap::new();
        cameras.insert(
            "cam0".to_string(),
            CameraCalibrationResult {
                camera_id: "cam0".to_string(),
                intrinsics: CameraIntrinsics {
                    fx: 500.0,
                    fy: 500.0,
                    cx: 320.0,
                    cy: 240.0,
                    width: 640,
                    height: 480,
                },
                distortion: crate::calibration::types::DistortionModel::zero(),
                reprojection_rms: if overall_score > 0.8 { 0.25 } else { 0.75 },
                residual_by_region: Vec::new(),
                stability_across_runs: 0.05,
            },
        );

        let mut camera_imu_extrinsics = HashMap::new();
        camera_imu_extrinsics.insert(
            "cam0".to_string(),
            CameraIMUCalibrationResult {
                extrinsics: CameraIMUExtrinsics {
                    t_imu_to_camera: Isometry3::identity(),
                    time_offset_s: 0.001,
                    time_offset_uncertainty_s: 0.0001,
                    time_drift: None,
                    rolling_shutter_readout: if rs_significant { Some(0.033) } else { None },
                    rolling_shutter_uncertainty: if rs_significant { Some(0.001) } else { None },
                },
                timing_cost_curve: vec![(0.0, 1.0), (0.001, 0.5), (0.002, 1.0)],
                timing_observability: if overall_score > 0.8 { 0.9 } else { 0.3 },
                flow_prediction_error: (1.5, 0.5, 3.0),
            },
        );

        let mut rolling_shutter = HashMap::new();
        rolling_shutter.insert(
            "cam0".to_string(),
            RollingShutterDetectionResult {
                is_significant: rs_significant,
                readout_time: if rs_significant { 0.033 } else { 0.0 },
                readout_uncertainty: 0.001,
                significance_score: if rs_significant { 0.85 } else { 0.05 },
                straightness_vs_angular_velocity: Vec::new(),
            },
        );

        CalibrationResult {
            cameras,
            stereo: None,
            imu: IMUCalibrationResult {
                intrinsics: IMUIntrinsics {
                    accel_scale: Matrix3::identity(),
                    gyro_scale: Matrix3::identity(),
                    accel_bias: Vector3::zeros(),
                    gyro_bias: Vector3::zeros(),
                    accel_noise_density: 1e-3,
                    gyro_noise_density: 1e-4,
                    accel_bias_random_walk: 1e-5,
                    gyro_bias_random_walk: 1e-6,
                },
                gravity_magnitude_error: 0.01,
                bias_stability: vec![1e-6, 1e-6, 1e-6, 1e-5, 1e-5, 1e-5],
            },
            camera_imu_extrinsics,
            rolling_shutter,
            timing_quality: TimingQuality::Stable {
                offset_s: 0.001,
                jitter_s: 1e-5,
            },
            calibration_timestamp: 0.0,
        }
    }

    #[test]
    fn test_adaptive_confidence() {
        let config = AdaptiveFusionConfig::default();
        let mut algo = OptimizedFusionAlgorithm::new(config);

        // Near field, slow motion: high confidence
        let conf_near_slow = algo.adaptive_confidence(0.9, 0.9, 0.5, 0.2);

        // Far field, fast motion: lower confidence
        let conf_far_fast = algo.adaptive_confidence(0.9, 0.9, 10.0, 5.0);

        assert!(conf_near_slow > conf_far_fast);
    }

    #[test]
    fn test_adaptive_confidence_with_calibration() {
        let config = AdaptiveFusionConfig::default();

        // Excellent calibration
        let excellent_calibration = create_test_calibration(0.95, false);
        let mut algo_excellent =
            OptimizedFusionAlgorithm::with_calibration(config.clone(), excellent_calibration);

        // Poor calibration
        let poor_calibration = create_test_calibration(0.30, false);
        let mut algo_poor = OptimizedFusionAlgorithm::with_calibration(config, poor_calibration);

        // Same scene, different calibration quality
        let conf_excellent = algo_excellent.adaptive_confidence(0.9, 0.9, 1.0, 0.5);
        let conf_poor = algo_poor.adaptive_confidence(0.9, 0.9, 1.0, 0.5);

        // Excellent calibration should give higher confidence
        assert!(conf_excellent > conf_poor);
    }

    #[test]
    fn test_distance_depth_optimization() {
        let config = AdaptiveFusionConfig::default();
        let algo = OptimizedFusionAlgorithm::new(config);

        let near = Point3::new(0.0, 0.0, 0.5);
        let far = Point3::new(0.0, 0.0, 10.0);

        let near_opt = algo.optimize_depth_for_distance(near, 0.05);
        let far_opt = algo.optimize_depth_for_distance(far, 0.05);

        // Far field should have higher error
        assert!(far_opt > near_opt);
    }

    #[test]
    fn test_rolling_shutter_time_offset() {
        let config = AdaptiveFusionConfig::default();
        let calibration = create_test_calibration(0.9, true); // Good calibration, RS significant

        let algo = OptimizedFusionAlgorithm::with_calibration(config, calibration);

        let image_height = 1080;

        // Top row: minimal offset
        let t_top = algo
            .rolling_shutter_time_offset(0, image_height, "cam0")
            .unwrap();
        assert!((t_top - 0.0).abs() < 1e-6);

        // Middle row: half readout
        let t_mid = algo
            .rolling_shutter_time_offset(image_height / 2, image_height, "cam0")
            .unwrap();
        assert!((t_mid - 0.0165).abs() < 1e-3); // ~16.5ms

        // Bottom row: full readout
        let t_bot = algo
            .rolling_shutter_time_offset(image_height - 1, image_height, "cam0")
            .unwrap();
        assert!((t_bot - 0.033).abs() < 1e-3); // ~33ms
    }

    #[test]
    fn test_calibration_residual_weights() {
        let config = AdaptiveFusionConfig::default();
        let calibration = create_test_calibration(0.95, false);

        let algo = OptimizedFusionAlgorithm::with_calibration(config, calibration);

        // Excellent calibration: high weights
        assert!(algo.visual_residual_weight() > 0.7);
        assert!(algo.imu_residual_weight() > 0.7);
        assert!(algo.robust_threshold_multiplier() < 1.3); // Reasonable threshold
    }
}
