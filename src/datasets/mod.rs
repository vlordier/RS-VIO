//! Dataset players and configuration.
//!
//! Provides dataset-specific players that read image timestamps and frames
//! from disk, feed them into the [`Estimator`](crate::estimator::Estimator),
//! and collect timing statistics.
//!
//! ## Players
//!
//! - [`euroc_player::EurocPlayer`] — EUROC MAV dataset
//! - [`tum_vi_player::TUMVIPlayer`] — TUM VI dataset
//! - [`fourseasons_player::FourSeasonsPlayer`] — 4Seasons dataset
//!
//! ## Data types
//!
//! - [`ImageData`] — timestamp and filename path for a single image
//! - [`ImuData`] — IMU gyroscope and accelerometer measurement
//! - [`CameraModelType`] — unified enum for all supported camera models

pub mod config;
pub mod euroc_player;
pub mod fourseasons_player;
pub mod tum_vi_player;

use crate::datasets::config::Config;
use camera_intrinsic_model::generic_model::CameraModel;
use camera_intrinsic_model::models::opencv5::OpenCVModel5;
use camera_intrinsic_model::models::EUCM;
use nalgebra034; // Required by camera-intrinsic-model for EUCM support

/// Equidistant (fisheye) distortion model for pinhole camera.
/// 
/// This is the model used by TUM-VI dataset (camera_model: pinhole, distortion_model: equidistant).
/// 
/// Projection (3D point to pixel):
///   r_u = sqrt(x² + y²) / z  (undistorted radius)
///   theta = atan(r_u)         (angle from optical axis)
///   r_d = theta * (1 + k1*theta² + k2*theta⁴ + k3*theta⁶ + k4*theta⁸)  (distorted radius)
///   u = fx * (r_d / r_u) * x + cx
///   v = fy * (r_d / r_u) * y + cy
/// 
/// Unprojection (pixel to bearing vector):
///   r_d = sqrt((u-cx)² + (v-cy)²) / f
///   theta = r_d / (1 + k1*r_d² + k2*r_d⁴ + ...)  (approximate inverse)
///   Bearing: [tan(theta)*x/r_d, tan(theta)*y/r_d, 1]
#[derive(Clone, Debug)]
pub struct EquidistantModel {
    /// Focal length x
    pub fx: f64,
    /// Focal length y
    pub fy: f64,
    /// Principal point x
    pub cx: f64,
    /// Principal point y
    pub cy: f64,
    /// Equidistant distortion coefficients [k1, k2, k3, k4]
    /// Used as: r_d = theta * (1 + k1*theta² + k2*theta⁴ + k3*theta⁶ + k4*theta⁸)
    pub k: [f64; 4],
    /// Image width
    pub width: u32,
    /// Image height
    pub height: u32,
}

impl EquidistantModel {
    /// Create a new equidistant camera model.
    pub fn new(
        fx: f64,
        fy: f64,
        cx: f64,
        cy: f64,
        k: [f64; 4],
        width: u32,
        height: u32,
    ) -> Self {
        Self { fx, fy, cx, cy, k, width, height }
    }
    
    /// Apply equidistant distortion: r_d = theta * (1 + k1*theta² + k2*theta⁴ + ...)
    #[inline]
    fn apply_distortion(&self, theta: f64) -> f64 {
        let theta2 = theta * theta;
        let theta4 = theta2 * theta2;
        let theta6 = theta4 * theta2;
        let theta8 = theta4 * theta4;
        
        theta * (1.0 + self.k[0] * theta2 + self.k[1] * theta4 + self.k[2] * theta6 + self.k[3] * theta8)
    }
    
    /// Approximate inverse distortion: theta ≈ r_d / (1 + k1*r_d² + ...)
    #[inline]
    fn remove_distortion(&self, r_d: f64) -> f64 {
        // For small distortion (TUM-VI has k < 0.01), use first-order approximation
        let r_d2 = r_d * r_d;
        let r_d4 = r_d2 * r_d2;
        
        // theta ≈ r_d * (1 - k1*r_d² - k2*r_d⁴ + ...)
        let inv = 1.0 - self.k[0] * r_d2 - self.k[1] * r_d4;
        r_d * inv
    }

    /// Unproject a pixel coordinate to a normalized bearing vector.
    pub fn unproject_one(&self, pixel: &nalgebra034::Vector2<f64>) -> nalgebra034::Vector2<f64> {
        // Normalize to focal-length coordinates
        let x = (pixel.x - self.cx) / self.fx;
        let y = (pixel.y - self.cy) / self.fy;

        // Compute distorted radius
        let r_d = (x * x + y * y).sqrt();
        if r_d < 1e-10 {
            return nalgebra034::Vector2::new(x, y);
        }
        
        // Remove distortion to get theta
        let theta = self.remove_distortion(r_d);
        
        // Compute undistorted radius: r_u = tan(theta)
        let r_u = if theta.abs() < 1.5 {
            theta.tan()
        } else {
            theta.signum() * 10.0
        };
        
        // Scale to get undistorted coordinates
        let scale = if r_d > 1e-10 { r_u / r_d } else { 1.0 };
        nalgebra034::Vector2::new(x * scale, y * scale)
    }
    
    /// Project a 3D point to pixel coordinates.
    pub fn project_one(&self, point: &nalgebra034::Vector3<f64>) -> nalgebra034::Vector2<f64> {
        // Handle points behind camera or too close
        if point.z < 0.1 {
            return nalgebra034::Vector2::new(self.cx, self.cy);
        }
        
        let x = point.x / point.z;
        let y = point.y / point.z;
        
        // Compute undistorted radius and angle
        let r_u = (x * x + y * y).sqrt();
        if r_u < 1e-10 {
            return nalgebra034::Vector2::new(self.cx, self.cy);
        }
        
        // True angle: theta = atan(r_u)
        let theta = r_u.atan();
        
        // Apply equidistant distortion
        let r_d = self.apply_distortion(theta);
        
        // Clamp to avoid extreme values
        let r_d = r_d.clamp(-3.0, 3.0);
        
        // Project to pixel coordinates
        let scale = if r_u > 1e-10 { r_d / r_u } else { 1.0 };
        let scale = scale.clamp(-5.0, 5.0);
        
        let u = x * scale * self.fx + self.cx;
        let v = y * scale * self.fy + self.cy;
        
        // Clamp to reasonable image coordinates
        let margin = 100.0;
        let u = u.clamp(-margin, self.width as f64 + margin);
        let v = v.clamp(-margin, self.height as f64 + margin);
        
        // Check for NaN/Inf
        if !u.is_finite() || !v.is_finite() {
            return nalgebra034::Vector2::new(self.cx, self.cy);
        }
        
        nalgebra034::Vector2::new(u, v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_equidistant_model_creation() {
        let model = EquidistantModel::new(
            190.0, 190.0, 256.0, 256.0,
            [0.0, 0.0, 0.0, 0.0],
            512, 512,
        );
        
        assert!((model.fx - 190.0).abs() < 1e-5);
        assert!((model.fy - 190.0).abs() < 1e-5);
        assert!((model.cx - 256.0).abs() < 1e-5);
        assert!((model.cy - 256.0).abs() < 1e-5);
        assert_eq!(model.width, 512);
        assert_eq!(model.height, 512);
    }

    #[test]
    fn test_equidistant_apply_distortion() {
        let model = EquidistantModel::new(
            190.0, 190.0, 256.0, 256.0,
            [0.0, 0.0, 0.0, 0.0],
            512, 512,
        );
        
        // Zero distortion should return theta unchanged
        let theta = 0.5;
        let r_d = model.apply_distortion(theta);
        assert!((r_d - theta).abs() < 1e-5);
        
        // With distortion
        let model = EquidistantModel::new(
            190.0, 190.0, 256.0, 256.0,
            [0.003, 0.0007, -0.002, 0.0002],
            512, 512,
        );
        let r_d = model.apply_distortion(theta);
        assert!(r_d > 0.0);
    }

    #[test]
    fn test_equidistant_remove_distortion() {
        let model = EquidistantModel::new(
            190.0, 190.0, 256.0, 256.0,
            [0.0, 0.0, 0.0, 0.0],
            512, 512,
        );
        
        // Zero distortion should return r_d unchanged
        let r_d = 0.5;
        let theta = model.remove_distortion(r_d);
        assert!((theta - r_d).abs() < 1e-5);
    }

    #[test]
    fn test_equidistant_unproject_center() {
        let model = EquidistantModel::new(
            190.0, 190.0, 256.0, 256.0,
            [0.0, 0.0, 0.0, 0.0],
            512, 512,
        );
        
        // Center pixel should give zero bearing
        let pixel = nalgebra034::Vector2::new(256.0, 256.0);
        let bearing = model.unproject_one(&pixel);
        assert!(bearing.x.abs() < 1e-5);
        assert!(bearing.y.abs() < 1e-5);
    }

    #[test]
    fn test_equidistant_unproject_edge_cases() {
        let model = EquidistantModel::new(
            190.0, 190.0, 256.0, 256.0,
            [0.0, 0.0, 0.0, 0.0],
            512, 512,
        );
        
        // Zero radius should handle gracefully
        let pixel = nalgebra034::Vector2::new(256.0, 256.0);
        let bearing = model.unproject_one(&pixel);
        assert!(bearing.x.is_finite());
        assert!(bearing.y.is_finite());
    }

    #[test]
    fn test_equidistant_project_forward_back() {
        let model = EquidistantModel::new(
            190.0, 190.0, 256.0, 256.0,
            [0.0, 0.0, 0.0, 0.0],
            512, 512,
        );
        
        // Project a point in front of camera
        let point = nalgebra034::Vector3::new(0.0, 0.0, 1.0);
        let pixel = model.project_one(&point);
        
        // Should project to center
        assert!((pixel.x - 256.0).abs() < 1.0);
        assert!((pixel.y - 256.0).abs() < 1.0);
    }

    #[test]
    fn test_equidistant_project_behind_camera() {
        let model = EquidistantModel::new(
            190.0, 190.0, 256.0, 256.0,
            [0.0, 0.0, 0.0, 0.0],
            512, 512,
        );
        
        // Point behind camera should project to center
        let point = nalgebra034::Vector3::new(0.0, 0.0, -1.0);
        let pixel = model.project_one(&point);
        
        assert!((pixel.x - 256.0).abs() < 1.0);
        assert!((pixel.y - 256.0).abs() < 1.0);
    }

    #[test]
    fn test_equidistant_project_edge_cases() {
        let model = EquidistantModel::new(
            190.0, 190.0, 256.0, 256.0,
            [0.0, 0.0, 0.0, 0.0],
            512, 512,
        );
        
        // Zero radius should handle gracefully
        let point = nalgebra034::Vector3::new(0.0, 0.0, 1.0);
        let pixel = model.project_one(&point);
        
        assert!(pixel.x.is_finite());
        assert!(pixel.y.is_finite());
    }

    #[test]
    fn test_equidistant_with_tum_vi_coeffs() {
        // Test with actual TUM-VI distortion coefficients
        let model = EquidistantModel::new(
            190.97847715128717, 190.9733070521226,
            254.93170605935475, 256.8974428996504,
            [0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182],
            512, 512,
        );
        
        let pixel = nalgebra034::Vector2::new(300.0, 300.0);
        let bearing = model.unproject_one(&pixel);
        
        assert!(bearing.x.is_finite());
        assert!(bearing.y.is_finite());
    }
}

impl CameraModel<f64> for EquidistantModel {
    fn unproject_one(
        &self,
        pixel: &nalgebra034::Vector2<f64>,
    ) -> nalgebra034::Vector3<f64> {
        let result = self.unproject_one(pixel);
        nalgebra034::Vector3::new(result.x, result.y, 1.0)
    }

    fn project_one(&self, point: &nalgebra034::Vector3<f64>) -> nalgebra034::Vector2<f64> {
        self.project_one(point)
    }

    fn width(&self) -> f64 {
        self.width as f64
    }

    fn height(&self) -> f64 {
        self.height as f64
    }
    
    fn set_params(&mut self, _params: &nalgebra034::DVector<f64>) {}
    
    fn params(&self) -> nalgebra034::DVector<f64> {
        nalgebra034::DVector::from_vec(vec![
            self.fx, self.fy, self.cx, self.cy,
            self.k[0], self.k[1], self.k[2], self.k[3],
        ])
    }
    
    fn camera_params(&self) -> nalgebra034::DVector<f64> {
        nalgebra034::DVector::from_vec(vec![self.fx, self.fy, self.cx, self.cy])
    }
    
    fn distortion_params(&self) -> nalgebra034::DVector<f64> {
        nalgebra034::DVector::from_vec(vec![self.k[0], self.k[1], self.k[2], self.k[3]])
    }
    
    fn set_w_h(&mut self, w: u32, h: u32) {
        self.width = w;
        self.height = h;
    }
    
    fn distortion_params_bound(&self) -> Vec<(usize, (f64, f64))> {
        Vec::new()
    }
}

// Image data structure
/// A single image entry from a dataset with its timestamp and file path.
#[derive(Debug, Clone)]
pub struct ImageData {
    /// Timestamp in nanoseconds.
    pub timestamp: i64,
    /// Relative or absolute filename path.
    pub filename: String,
}

/// IMU gyroscope and accelerometer measurement.
#[derive(Debug, Clone)]
pub struct ImuData {
    /// Timestamp in nanoseconds.
    pub timestamp: i64,
    /// Angular velocity in rad/s [x, y, z].
    pub gyro: [f64; 3],
    /// Linear acceleration in m/s² [x, y, z].
    pub accel: [f64; 3],
}

/// Tracks the processing state of a dataset player.
#[derive(Debug)]
pub struct FrameContext {
    pub current_idx: usize,
    pub processed_frames: usize,
    pub previous_frame_timestamp: i64,
    pub step_mode: bool,
    pub auto_play: bool,
    pub advance_frame: bool,
}

impl FrameContext {
    pub fn new(step_mode: bool) -> Self {
        FrameContext {
            current_idx: 0,
            processed_frames: 0,
            previous_frame_timestamp: 0,
            step_mode,
            auto_play: !step_mode,
            advance_frame: false,
        }
    }
}

/// Statistics and result from running a dataset player.
#[derive(Debug, Default)]
pub struct PlayerResult {
    pub success: bool,
    pub error_message: String,
    pub processed_frames: usize,
    pub frame_processing_times: Vec<f64>,
    pub average_processing_time_ms: f64,
}

/// Configuration for running a dataset player.
#[derive(Debug)]
pub struct PlayerConfig {
    pub config_path: String,
    pub dataset_path: String,
    pub enable_statistics: bool,
    pub enable_console_statistics: bool,
    pub step_mode: bool,
    /// Whether to use IMU data for motion prediction (VIO) or skip it (VO).
    pub use_imu: bool,
}

/// Enum to represent different camera model types
#[derive(Clone, Debug)]
pub enum CameraModelType {
    OpenCV5(OpenCVModel5<f64>),
    EUCM(EUCM<f64>),
    /// Equidistant fisheye model for TUM-VI dataset
    /// (camera_model: pinhole, distortion_model: equidistant)
    Equidistant(EquidistantModel),
}

impl CameraModelType {
    /// Get a reference to the underlying camera model as a trait object
    pub fn as_camera_model(&self) -> &dyn CameraModel<f64> {
        match self {
            CameraModelType::OpenCV5(cam) => cam,
            CameraModelType::EUCM(cam) => cam,
            CameraModelType::Equidistant(cam) => cam,
        }
    }
}

/// Create camera models from config
/// This helper function creates camera models from the configuration
/// for both left and right cameras. Supports OpenCVModel5, EUCM, and Equidistant models.
pub fn create_camera_models_from_config(config: &Config) -> (CameraModelType, CameraModelType) {
    let cam = &config.camera;

    // Determine left camera model type
    let left_model_str = cam.left_model.as_deref().unwrap_or("pinhole-radtan");
    let left_cam = if left_model_str == "EUCM" || left_model_str == "eucm" {
        // EUCM model: [fx, fy, cx, cy, alpha, beta]
        let eucm_params_vec: Vec<f64> = vec![
            cam.left_intrinsics.first().copied().unwrap_or(500.0), // fx
            cam.left_intrinsics.get(1).copied().unwrap_or(500.0),  // fy
            cam.left_intrinsics.get(2).copied().unwrap_or(320.0),  // cx
            cam.left_intrinsics.get(3).copied().unwrap_or(240.0),  // cy
            cam.left_distortion.first().copied().unwrap_or(0.5),   // alpha
            cam.left_distortion.get(1).copied().unwrap_or(1.0),    // beta
        ];
        let eucm_params = nalgebra034::DVector::from_vec(eucm_params_vec);
        CameraModelType::EUCM(EUCM::new(&eucm_params, cam.image_width, cam.image_height))
    } else if left_model_str == "equidistant" || left_model_str == "fisheye" {
        // TUM-VI equidistant model: [fx, fy, cx, cy], distortion: [k1, k2, k3, k4]
        let fx = cam.left_intrinsics.first().copied().unwrap_or(190.0);
        let fy = cam.left_intrinsics.get(1).copied().unwrap_or(190.0);
        let cx = cam.left_intrinsics.get(2).copied().unwrap_or(256.0);
        let cy = cam.left_intrinsics.get(3).copied().unwrap_or(256.0);
        
        let k = [
            cam.left_distortion.first().copied().unwrap_or(0.0),
            cam.left_distortion.get(1).copied().unwrap_or(0.0),
            cam.left_distortion.get(2).copied().unwrap_or(0.0),
            cam.left_distortion.get(3).copied().unwrap_or(0.0),
        ];
        
        CameraModelType::Equidistant(EquidistantModel::new(
            fx, fy, cx, cy, k,
            cam.image_width, cam.image_height,
        ))
    } else {
        // OpenCVModel5 (pinhole-radtan): [fx, fy, cx, cy, k1, k2, p1, p2, k3]
        let left_params_vec: Vec<f64> = vec![
            cam.left_intrinsics.first().copied().unwrap_or(500.0),
            cam.left_intrinsics.get(1).copied().unwrap_or(500.0),
            cam.left_intrinsics.get(2).copied().unwrap_or(320.0),
            cam.left_intrinsics.get(3).copied().unwrap_or(240.0),
            cam.left_distortion.first().copied().unwrap_or(0.0), // k1
            cam.left_distortion.get(1).copied().unwrap_or(0.0),  // k2
            cam.left_distortion.get(2).copied().unwrap_or(0.0),  // p1
            cam.left_distortion.get(3).copied().unwrap_or(0.0),  // p2
            cam.left_distortion.get(4).copied().unwrap_or(0.0),  // k3
        ];
        let left_params = nalgebra034::DVector::from_vec(left_params_vec);
        CameraModelType::OpenCV5(OpenCVModel5::new(
            &left_params,
            cam.image_width,
            cam.image_height,
        ))
    };

    // Determine right camera model type
    let right_model_str = cam.right_model.as_deref().unwrap_or("pinhole-radtan");
    let right_cam = if right_model_str == "EUCM" || right_model_str == "eucm" {
        // EUCM model: [fx, fy, cx, cy, alpha, beta]
        let eucm_params_vec: Vec<f64> = vec![
            cam.right_intrinsics.first().copied().unwrap_or(500.0), // fx
            cam.right_intrinsics.get(1).copied().unwrap_or(500.0),  // fy
            cam.right_intrinsics.get(2).copied().unwrap_or(320.0),  // cx
            cam.right_intrinsics.get(3).copied().unwrap_or(240.0),  // cy
            cam.right_distortion.first().copied().unwrap_or(0.5),   // alpha
            cam.right_distortion.get(1).copied().unwrap_or(1.0),    // beta
        ];
        let eucm_params = nalgebra034::DVector::from_vec(eucm_params_vec);
        CameraModelType::EUCM(EUCM::new(&eucm_params, cam.image_width, cam.image_height))
    } else if right_model_str == "equidistant" || right_model_str == "fisheye" {
        // TUM-VI equidistant model: [fx, fy, cx, cy], distortion: [k1, k2, k3, k4]
        let fx = cam.right_intrinsics.first().copied().unwrap_or(190.0);
        let fy = cam.right_intrinsics.get(1).copied().unwrap_or(190.0);
        let cx = cam.right_intrinsics.get(2).copied().unwrap_or(256.0);
        let cy = cam.right_intrinsics.get(3).copied().unwrap_or(256.0);
        
        let k = [
            cam.right_distortion.first().copied().unwrap_or(0.0),
            cam.right_distortion.get(1).copied().unwrap_or(0.0),
            cam.right_distortion.get(2).copied().unwrap_or(0.0),
            cam.right_distortion.get(3).copied().unwrap_or(0.0),
        ];
        
        CameraModelType::Equidistant(EquidistantModel::new(
            fx, fy, cx, cy, k,
            cam.image_width, cam.image_height,
        ))
    } else {
        // OpenCVModel5: [fx, fy, cx, cy, k1, k2, p1, p2, k3]
        let right_params_vec: Vec<f64> = vec![
            cam.right_intrinsics.first().copied().unwrap_or(500.0),
            cam.right_intrinsics.get(1).copied().unwrap_or(500.0),
            cam.right_intrinsics.get(2).copied().unwrap_or(320.0),
            cam.right_intrinsics.get(3).copied().unwrap_or(240.0),
            cam.right_distortion.first().copied().unwrap_or(0.0), // k1
            cam.right_distortion.get(1).copied().unwrap_or(0.0),  // k2
            cam.right_distortion.get(2).copied().unwrap_or(0.0),  // p1
            cam.right_distortion.get(3).copied().unwrap_or(0.0),  // p2
            cam.right_distortion.get(4).copied().unwrap_or(0.0),  // k3
        ];
        let right_params = nalgebra034::DVector::from_vec(right_params_vec);
        CameraModelType::OpenCV5(OpenCVModel5::new(
            &right_params,
            cam.image_width,
            cam.image_height,
        ))
    };

    (left_cam, right_cam)
}
