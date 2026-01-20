/// Core types for camera + IMU calibration pipeline.
///
/// This module provides data structures for:
/// - Camera intrinsics and distortion models
/// - Stereo extrinsics and rectification
/// - IMU intrinsic parameters and noise models
/// - Camera-IMU extrinsics (spatial) and time offset (temporal)
/// - Quality metrics and acceptance thresholds
use nalgebra::{Isometry3, Matrix3, Vector3};
use std::collections::HashMap;

/// Camera intrinsic parameters (K matrix: focal length, principal point).
#[derive(Clone, Debug)]
pub struct CameraIntrinsics {
    /// Focal length in x direction (pixels)
    pub fx: f64,
    /// Focal length in y direction (pixels)
    pub fy: f64,
    /// Principal point x coordinate (pixels)
    pub cx: f64,
    /// Principal point y coordinate (pixels)
    pub cy: f64,
    /// Image width (pixels)
    pub width: u32,
    /// Image height (pixels)
    pub height: u32,
}

impl CameraIntrinsics {
    /// Build intrinsic matrix K (3×3)
    pub fn k_matrix(&self) -> Matrix3<f64> {
        Matrix3::new(self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0)
    }

    /// Inverse intrinsic matrix K^{-1}
    pub fn k_matrix_inv(&self) -> Matrix3<f64> {
        Matrix3::new(
            1.0 / self.fx,
            0.0,
            -self.cx / self.fx,
            0.0,
            1.0 / self.fy,
            -self.cy / self.fy,
            0.0,
            0.0,
            1.0,
        )
    }

    /// Project normalized ray to image plane
    /// ray: unit direction [x, y, z]
    pub fn project(&self, ray: &Vector3<f64>) -> Vector3<f64> {
        let uv = Vector3::new(
            self.fx * ray.x / ray.z + self.cx,
            self.fy * ray.y / ray.z + self.cy,
            1.0,
        );
        uv
    }

    /// Unproject pixel to normalized ray
    /// pixel: [u, v]
    pub fn unproject(&self, pixel: &[f64; 2]) -> Vector3<f64> {
        let x = (pixel[0] - self.cx) / self.fx;
        let y = (pixel[1] - self.cy) / self.fy;
        let ray = Vector3::new(x, y, 1.0);
        ray / ray.norm()
    }
}

/// Distortion model parameters (radial + tangential).
/// Follows OpenCV convention: k1, k2, p1, p2 (+ optional k3, k4, k5, k6)
#[derive(Clone, Debug)]
pub struct DistortionModel {
    /// Radial distortion coefficient 1
    pub k1: f64,
    /// Radial distortion coefficient 2
    pub k2: f64,
    /// Tangential distortion coefficient 1
    pub p1: f64,
    /// Tangential distortion coefficient 2
    pub p2: f64,
    /// Optional: radial distortion coefficient 3
    pub k3: Option<f64>,
    /// Optional: radial distortion coefficient 4 (rational model)
    pub k4: Option<f64>,
    /// Optional: radial distortion coefficient 5
    pub k5: Option<f64>,
    /// Optional: radial distortion coefficient 6
    pub k6: Option<f64>,
}

impl DistortionModel {
    /// Create zero distortion model
    pub fn zero() -> Self {
        Self {
            k1: 0.0,
            k2: 0.0,
            p1: 0.0,
            p2: 0.0,
            k3: None,
            k4: None,
            k5: None,
            k6: None,
        }
    }

    /// Apply distortion to normalized coordinates [x, y]
    pub fn distort(&self, uv_norm: &[f64; 2]) -> [f64; 2] {
        let x = uv_norm[0];
        let y = uv_norm[1];
        let r2 = x * x + y * y;
        let r4 = r2 * r2;
        let r6 = r4 * r2;

        // Radial distortion
        let k3 = self.k3.unwrap_or(0.0);
        let radial = 1.0 + self.k1 * r2 + self.k2 * r4 + k3 * r6;

        // Tangential distortion
        let dx = 2.0 * self.p1 * x * y + self.p2 * (r2 + 2.0 * x * x);
        let dy = self.p1 * (r2 + 2.0 * y * y) + 2.0 * self.p2 * x * y;

        [x * radial + dx, y * radial + dy]
    }

    /// Remove distortion from normalized coordinates [x, y]
    /// Uses iterative Newton-Raphson (typically converges in 2-3 iterations)
    pub fn undistort(&self, uv_distorted: &[f64; 2]) -> [f64; 2] {
        let mut uv = *uv_distorted;
        for _ in 0..5 {
            let distorted = self.distort(&uv);
            let dx = uv_distorted[0] - distorted[0];
            let dy = uv_distorted[1] - distorted[1];
            if dx * dx + dy * dy < 1e-10 {
                break;
            }
            uv[0] += dx;
            uv[1] += dy;
        }
        uv
    }
}

/// Per-camera calibration result (intrinsics + distortion + quality metrics)
#[derive(Clone, Debug)]
pub struct CameraCalibrationResult {
    /// Camera ID or label
    pub camera_id: String,
    /// Intrinsic parameters
    pub intrinsics: CameraIntrinsics,
    /// Distortion model
    pub distortion: DistortionModel,
    /// RMS reprojection error (pixels)
    pub reprojection_rms: f64,
    /// Per-region residual statistics: (region_id, mean_error, std_error)
    pub residual_by_region: Vec<(String, f64, f64)>,
    /// Stability metric (lower is better) across multiple calibration runs
    pub stability_across_runs: f64,
}

/// Stereo extrinsics (left ↔ right camera)
#[derive(Clone, Debug)]
pub struct StereoExtrinsics {
    /// Rotation matrix (3×3) from left to right
    pub rotation: Matrix3<f64>,
    /// Translation vector from left to right (meters)
    pub translation: Vector3<f64>,
    /// Baseline (magnitude of translation)
    pub baseline: f64,
    /// Rectification transform for left camera
    pub rectification_left: Matrix3<f64>,
    /// Rectification transform for right camera
    pub rectification_right: Matrix3<f64>,
}

impl StereoExtrinsics {
    /// Compute as Isometry3 (rotation + translation)
    pub fn as_isometry(&self) -> Isometry3<f64> {
        Isometry3::from_parts(
            nalgebra::Translation3::from(self.translation),
            nalgebra::UnitQuaternion::from_matrix(&self.rotation),
        )
    }
}

/// Stereo calibration result (extrinsics + quality metrics)
#[derive(Clone, Debug)]
pub struct StereoCalibrationResult {
    /// Stereo extrinsics
    pub extrinsics: StereoExtrinsics,
    /// Vertical disparity RMS after rectification (pixels)
    pub vertical_disparity_rms: f64,
    /// Left-right consistency error (pixels)
    pub left_right_consistency_error: f64,
    /// Epipolar residual distribution: (mean, std, max)
    pub epipolar_residuals: (f64, f64, f64),
}

/// IMU intrinsic calibration (scale factors, biases, noise)
#[derive(Clone, Debug)]
pub struct IMUIntrinsics {
    /// Gyroscope scale factors (3×3 diagonal matrix, ideally ~1.0)
    pub gyro_scale: Matrix3<f64>,
    /// Accelerometer scale factors (3×3 diagonal matrix, ideally ~1.0)
    pub accel_scale: Matrix3<f64>,
    /// Gyroscope bias (rad/s)
    pub gyro_bias: Vector3<f64>,
    /// Accelerometer bias (m/s²)
    pub accel_bias: Vector3<f64>,
    /// Gyroscope white noise density (rad/s/√Hz)
    pub gyro_noise_density: f64,
    /// Gyroscope bias random walk (rad/s²/√Hz)
    pub gyro_bias_random_walk: f64,
    /// Accelerometer white noise density (m/s²/√Hz)
    pub accel_noise_density: f64,
    /// Accelerometer bias random walk (m/s³/√Hz)
    pub accel_bias_random_walk: f64,
}

impl IMUIntrinsics {
    /// Identity IMU calibration (no scale/bias correction)
    pub fn identity() -> Self {
        Self {
            gyro_scale: Matrix3::identity(),
            accel_scale: Matrix3::identity(),
            gyro_bias: Vector3::zeros(),
            accel_bias: Vector3::zeros(),
            gyro_noise_density: 0.01,
            gyro_bias_random_walk: 0.001,
            accel_noise_density: 0.05,
            accel_bias_random_walk: 0.005,
        }
    }

    /// Correct raw gyro measurement
    pub fn correct_gyro(&self, raw_gyro: &Vector3<f64>) -> Vector3<f64> {
        self.gyro_scale * (raw_gyro - self.gyro_bias)
    }

    /// Correct raw accel measurement
    pub fn correct_accel(&self, raw_accel: &Vector3<f64>) -> Vector3<f64> {
        self.accel_scale * (raw_accel - self.accel_bias)
    }
}

/// IMU calibration result
#[derive(Clone, Debug)]
pub struct IMUCalibrationResult {
    /// IMU intrinsic parameters
    pub intrinsics: IMUIntrinsics,
    /// Gravity magnitude error after calibration (m/s²)
    pub gravity_magnitude_error: f64,
    /// Bias stability over time (per-axis): [gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z]
    pub bias_stability: Vec<f64>,
}

/// Camera-IMU extrinsics (spatial transform and temporal offset)
#[derive(Clone, Debug)]
pub struct CameraIMUExtrinsics {
    /// Isometry3 transform from IMU to camera frame
    pub t_imu_to_camera: Isometry3<f64>,
    /// Time offset: camera_time = imu_time + dt (seconds)
    /// Positive if camera timestamps are ahead of IMU
    pub time_offset_s: f64,
    /// Time offset uncertainty (1-sigma, seconds)
    pub time_offset_uncertainty_s: f64,
    /// Optional: time drift α (camera_time = imu_time + dt0 + α*t)
    pub time_drift: Option<f64>,
    /// Rolling shutter readout time (seconds), if applicable
    pub rolling_shutter_readout: Option<f64>,
    /// Rolling shutter readout uncertainty
    pub rolling_shutter_uncertainty: Option<f64>,
}

impl CameraIMUExtrinsics {
    /// Predicted pixel location given feature at 3D point, IMU pose, and camera intrinsics
    /// point_world: 3D point in world frame
    /// t_world_to_imu: IMU pose (world → IMU)
    /// intrinsics: camera intrinsics
    /// Returns: [u, v] pixel coordinates
    pub fn predict_pixel(
        &self,
        point_world: &Vector3<f64>,
        t_world_to_imu: &Isometry3<f64>,
        intrinsics: &CameraIntrinsics,
    ) -> [f64; 2] {
        // Point in IMU frame
        let point_imu = t_world_to_imu * point_world;
        // Point in camera frame
        let point_camera = self.t_imu_to_camera * point_imu;
        // Project to image
        let uv = intrinsics.project(&point_camera);
        [uv.x / uv.z, uv.y / uv.z]
    }
}

/// Camera-IMU calibration result (extrinsics + quality metrics)
#[derive(Clone, Debug)]
pub struct CameraIMUCalibrationResult {
    /// Camera-IMU extrinsics
    pub extrinsics: CameraIMUExtrinsics,
    /// Timing cost curve (sweep of Δt values and corresponding reprojection RMS)
    pub timing_cost_curve: Vec<(f64, f64)>, // (Δt, reprojection_rms)
    /// Sharpness of timing minimum (higher = better observability)
    pub timing_observability: f64,
    /// Optical flow vs IMU prediction error distribution (degrees)
    pub flow_prediction_error: (f64, f64, f64), // (mean, std, max)
}

/// Rolling shutter detection result
#[derive(Clone, Debug)]
pub struct RollingShutterDetectionResult {
    /// Is rolling shutter significant?
    pub is_significant: bool,
    /// Estimated readout time (seconds)
    pub readout_time: f64,
    /// Readout time uncertainty (seconds)
    pub readout_uncertainty: f64,
    /// RS significance score: ratio of error with vs without RS model
    pub significance_score: f64,
    /// Per-frame line straightness errors: (angular_velocity, deviation)
    pub straightness_vs_angular_velocity: Vec<(f64, f64)>,
}

/// Timing quality assessment
#[derive(Clone, Debug)]
pub enum TimingQuality {
    /// Perfect sync, stable offset
    Stable { offset_s: f64, jitter_s: f64 },
    /// Offset drifts linearly: t_camera = t_imu + dt0 + α*t
    Drifting {
        offset_s: f64,
        drift_rate: f64,
        jitter_s: f64,
    },
    /// High jitter, timestamps unreliable
    Jittery { jitter_s: f64 },
}

impl TimingQuality {
    /// Operating mode recommendation based on quality
    pub fn recommended_mode(&self) -> &'static str {
        match self {
            Self::Stable { jitter_s, .. } if *jitter_s < 1e-4 => "tight_rs_model",
            Self::Stable { .. } => "tight_coupling",
            Self::Drifting { .. } => "online_drift_tracking",
            Self::Jittery { .. } => "simplified_imu_aiding",
        }
    }
}

/// Complete calibration result
#[derive(Clone, Debug)]
pub struct CalibrationResult {
    /// Per-camera intrinsics
    pub cameras: HashMap<String, CameraCalibrationResult>,
    /// Stereo calibration results (if applicable)
    pub stereo: Option<StereoCalibrationResult>,
    /// IMU intrinsic calibration
    pub imu: IMUCalibrationResult,
    /// Camera-IMU extrinsics (per camera)
    pub camera_imu_extrinsics: HashMap<String, CameraIMUCalibrationResult>,
    /// Rolling shutter detection (per camera)
    pub rolling_shutter: HashMap<String, RollingShutterDetectionResult>,
    /// Timing quality assessment
    pub timing_quality: TimingQuality,
    /// Timestamp of calibration
    pub calibration_timestamp: f64,
}

impl CalibrationResult {
    /// Generate a quality report from calibration results
    pub fn quality_report(&self, thresholds: &AcceptanceThresholds) -> CalibrationQualityReport {
        let mut metrics = Vec::new();
        let mut scores = Vec::new();

        // Check camera reprojection quality
        for (camera_id, camera) in &self.cameras {
            let passed = camera.reprojection_rms < thresholds.reprojection_rms_max;
            let score = 1.0 - (camera.reprojection_rms / thresholds.reprojection_rms_max).min(1.0);
            scores.push(score);
            metrics.push((
                format!("reprojection_rms_{}", camera_id),
                passed,
                format!("{:.3} px", camera.reprojection_rms),
            ));
        }

        // Check stereo quality
        if let Some(stereo) = &self.stereo {
            let passed = stereo.vertical_disparity_rms < thresholds.vertical_disparity_rms_max;
            let score = 1.0
                - (stereo.vertical_disparity_rms / thresholds.vertical_disparity_rms_max).min(1.0);
            scores.push(score);
            metrics.push((
                "vertical_disparity".to_string(),
                passed,
                format!("{:.3} px", stereo.vertical_disparity_rms),
            ));
        }

        // Check timing quality
        for (camera_id, extrinsics) in &self.camera_imu_extrinsics {
            let passed = extrinsics.timing_observability > thresholds.timing_observability_min;
            let score = extrinsics.timing_observability;
            scores.push(score);
            metrics.push((
                format!("timing_sharpness_{}", camera_id),
                passed,
                format!("observability: {:.2}", extrinsics.timing_observability),
            ));
        }

        // Check rolling shutter handling
        for (camera_id, rs) in &self.rolling_shutter {
            let passed = !rs.is_significant || rs.significance_score > 0.8;
            let score = if !rs.is_significant {
                1.0
            } else {
                rs.significance_score
            };
            scores.push(score);
            metrics.push((
                format!("rolling_shutter_{}", camera_id),
                passed,
                if !rs.is_significant {
                    "Not significant (global shutter)".to_string()
                } else {
                    format!(
                        "Readout: {:.1} ms, significance: {:.2}",
                        rs.readout_time * 1000.0,
                        rs.significance_score
                    )
                },
            ));
        }

        // Overall score: average of all individual scores
        let overall_score = if scores.is_empty() {
            0.5
        } else {
            scores.iter().sum::<f64>() / scores.len() as f64
        };

        // Pass if overall score is good and no critical failures
        let passed = overall_score > 0.7 && metrics.iter().all(|(_, p, _)| *p);

        CalibrationQualityReport {
            passed,
            metrics,
            overall_score,
        }
    }
}

/// Acceptance thresholds for calibration quality
#[derive(Clone, Debug)]
pub struct AcceptanceThresholds {
    /// Good reprojection RMS (pixels)
    pub reprojection_rms_good: f64,
    /// Maximum acceptable reprojection RMS (pixels)
    pub reprojection_rms_max: f64,
    /// Maximum vertical disparity after rectification (pixels)
    pub vertical_disparity_rms_max: f64,
    /// Maximum epipolar residual (pixels)
    pub epipolar_residual_max: f64,
    /// Minimum timing observability score
    pub timing_observability_min: f64,
    /// Maximum time offset jitter (seconds)
    pub time_jitter_max: f64,
    /// Minimum timing cost curve sharpness
    pub timing_curve_sharpness_min: f64,
}

impl AcceptanceThresholds {
    /// Standard thresholds for typical camera+IMU systems
    pub fn standard() -> Self {
        Self {
            reprojection_rms_good: 0.3,
            reprojection_rms_max: 0.8,
            vertical_disparity_rms_max: 0.3,
            epipolar_residual_max: 1.0,
            timing_observability_min: 0.5,
            time_jitter_max: 1e-3,
            timing_curve_sharpness_min: 0.3,
        }
    }

    /// Strict thresholds for high-accuracy applications
    pub fn strict() -> Self {
        Self {
            reprojection_rms_good: 0.15,
            reprojection_rms_max: 0.4,
            vertical_disparity_rms_max: 0.15,
            epipolar_residual_max: 0.5,
            timing_observability_min: 0.7,
            time_jitter_max: 1e-4,
            timing_curve_sharpness_min: 0.5,
        }
    }
}

/// Calibration quality report (acceptance decision + rationale)
#[derive(Clone, Debug)]
pub struct CalibrationQualityReport {
    /// Pass/fail decision
    pub passed: bool,
    /// Detailed pass/fail reasons per metric
    pub metrics: Vec<(String, bool, String)>, // (metric_name, passed, reason)
    /// Overall quality score (0.0 - 1.0)
    pub overall_score: f64,
}

impl CalibrationQualityReport {
    /// Create a summary string for logging/display
    pub fn summary(&self) -> String {
        let status = if self.passed { "PASS" } else { "FAIL" };
        let mut summary = format!("{}: {:.1}%\n", status, self.overall_score * 100.0);
        for (metric, passed, reason) in &self.metrics {
            let check = if *passed { "✓" } else { "✗" };
            summary.push_str(&format!("  {} {}: {}\n", check, metric, reason));
        }
        summary
    }
}
