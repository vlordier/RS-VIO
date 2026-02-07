//! Camera model traits for heterogeneous multi-camera systems
//!
//! This module defines traits and implementations for different camera models,
//! enabling calibration of heterogeneous camera rigs with varying intrinsics.

use nalgebra as na;

/// Trait for camera models used in calibration
///
/// This trait defines the interface that all camera models must implement,
/// allowing heterogeneous camera rigs with different intrinsic parameters.
pub trait CameraModel: std::fmt::Debug + Send + Sync {
    /// Project a 3D point into the camera's image plane
    ///
    /// # Arguments
    /// * `point_3d` - 3D point in camera coordinates
    /// * `intrinsics` - Camera intrinsic parameters
    ///
    /// # Returns
    /// 2D point in image coordinates (pixels)
    fn project(&self, point_3d: &na::Vector3<f64>, intrinsics: &[f64]) -> na::Vector2<f64>;

    /// Unproject a 2D image point to a 3D ray
    ///
    /// # Arguments
    /// * `point_2d` - 2D point in image coordinates
    /// * `intrinsics` - Camera intrinsic parameters
    ///
    /// # Returns
    /// 3D ray direction from camera center through the point
    fn unproject(&self, point_2d: &na::Vector2<f64>, intrinsics: &[f64]) -> na::Vector3<f64>;

    /// Get the number of intrinsic parameters for this model
    fn num_intrinsics(&self) -> usize;

    /// Get default intrinsic parameters for initialization
    fn default_intrinsics(&self, image_width: u32, image_height: u32) -> Vec<f64>;

    /// Get parameter names for debugging/logging
    fn parameter_names(&self) -> Vec<String>;

    /// Check if intrinsics are valid
    fn validate_intrinsics(&self, intrinsics: &[f64]) -> Result<(), String>;

    /// Get the camera model name
    fn name(&self) -> &str;

    /// Clone into a Box (required for trait objects)
    fn clone_box(&self) -> Box<dyn CameraModel>;
}

/// Pinhole camera model with optional distortion
#[derive(Debug, Clone)]
pub struct PinholeCamera {
    /// Whether to include radial distortion
    pub has_distortion: bool,
    /// Distortion model type
    pub distortion_model: DistortionModel,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DistortionModel {
    /// No distortion
    None,
    /// OpenCV radial-tangential model (k1, k2, p1, p2, k3)
    OpenCV,
    /// Simple radial-only (k1, k2)
    RadialOnly,
}

impl Default for PinholeCamera {
    fn default() -> Self {
        Self {
            has_distortion: true,
            distortion_model: DistortionModel::OpenCV,
        }
    }
}

impl CameraModel for PinholeCamera {
    fn project(&self, point_3d: &na::Vector3<f64>, intrinsics: &[f64]) -> na::Vector2<f64> {
        // Guard against division by zero
        if point_3d.z.abs() < 1e-12 {
            return na::Vector2::new(f64::NAN, f64::NAN);
        }

        // Extract parameters
        let fx = intrinsics[0];
        let fy = intrinsics[1];
        let cx = intrinsics[2];
        let cy = intrinsics[3];

        // Normalize coordinates
        let x = point_3d.x / point_3d.z;
        let y = point_3d.y / point_3d.z;

        // Apply distortion if enabled
        let (xd, yd) = if self.has_distortion && intrinsics.len() > 4 {
            self.apply_distortion(x, y, &intrinsics[4..])
        } else {
            (x, y)
        };

        // Apply focal length and principal point
        na::Vector2::new(fx * xd + cx, fy * yd + cy)
    }

    fn unproject(&self, point_2d: &na::Vector2<f64>, intrinsics: &[f64]) -> na::Vector3<f64> {
        let fx = intrinsics[0];
        let fy = intrinsics[1];
        let cx = intrinsics[2];
        let cy = intrinsics[3];

        // Remove principal point and focal length
        let x = (point_2d.x - cx) / fx;
        let y = (point_2d.y - cy) / fy;

        // TODO: Implement undistortion for unprojection
        // For now, assume no distortion in unprojection
        na::Vector3::new(x, y, 1.0).normalize()
    }

    fn num_intrinsics(&self) -> usize {
        4 + if self.has_distortion {
            match self.distortion_model {
                DistortionModel::None => 0,
                DistortionModel::RadialOnly => 2,
                DistortionModel::OpenCV => 5,
            }
        } else {
            0
        }
    }

    fn default_intrinsics(&self, image_width: u32, image_height: u32) -> Vec<f64> {
        let mut params = vec![
            500.0,                     // fx
            500.0,                     // fy
            image_width as f64 / 2.0,  // cx
            image_height as f64 / 2.0, // cy
        ];

        if self.has_distortion {
            match self.distortion_model {
                DistortionModel::None => {},
                DistortionModel::RadialOnly => {
                    params.extend_from_slice(&[0.0, 0.0]); // k1, k2
                },
                DistortionModel::OpenCV => {
                    params.extend_from_slice(&[0.0, 0.0, 0.0, 0.0, 0.0]); // k1, k2, p1, p2, k3
                },
            }
        }

        params
    }

    fn parameter_names(&self) -> Vec<String> {
        let mut names = vec![
            "fx".to_string(),
            "fy".to_string(),
            "cx".to_string(),
            "cy".to_string(),
        ];

        if self.has_distortion {
            match self.distortion_model {
                DistortionModel::None => {},
                DistortionModel::RadialOnly => {
                    names.extend_from_slice(&["k1".to_string(), "k2".to_string()]);
                },
                DistortionModel::OpenCV => {
                    names.extend_from_slice(&[
                        "k1".to_string(),
                        "k2".to_string(),
                        "p1".to_string(),
                        "p2".to_string(),
                        "k3".to_string(),
                    ]);
                },
            }
        }

        names
    }

    fn validate_intrinsics(&self, intrinsics: &[f64]) -> Result<(), String> {
        if intrinsics.len() != self.num_intrinsics() {
            return Err(format!(
                "Expected {} intrinsics, got {}",
                self.num_intrinsics(),
                intrinsics.len()
            ));
        }

        let fx = intrinsics[0];
        let fy = intrinsics[1];

        if fx <= 0.0 || fy <= 0.0 {
            return Err("Focal lengths must be positive".to_string());
        }

        Ok(())
    }

    fn name(&self) -> &str {
        match self.distortion_model {
            DistortionModel::None => "Pinhole (no distortion)",
            DistortionModel::RadialOnly => "Pinhole (radial)",
            DistortionModel::OpenCV => "Pinhole (OpenCV)",
        }
    }

    fn clone_box(&self) -> Box<dyn CameraModel> {
        Box::new(self.clone())
    }
}

impl PinholeCamera {
    /// Apply distortion to normalized coordinates
    fn apply_distortion(&self, x: f64, y: f64, distortion_coeffs: &[f64]) -> (f64, f64) {
        let r2 = x * x + y * y;

        match self.distortion_model {
            DistortionModel::None => (x, y),
            DistortionModel::RadialOnly => {
                if distortion_coeffs.len() >= 2 {
                    let k1 = distortion_coeffs[0];
                    let k2 = distortion_coeffs[1];

                    let radial = 1.0 + k1 * r2 + k2 * r2 * r2;
                    (x * radial, y * radial)
                } else {
                    (x, y)
                }
            },
            DistortionModel::OpenCV => {
                if distortion_coeffs.len() >= 5 {
                    let k1 = distortion_coeffs[0];
                    let k2 = distortion_coeffs[1];
                    let p1 = distortion_coeffs[2];
                    let p2 = distortion_coeffs[3];
                    let k3 = distortion_coeffs[4];

                    let r2 = x * x + y * y;
                    let r4 = r2 * r2;
                    let r6 = r4 * r2;

                    let radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
                    let tangential_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
                    let tangential_y = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

                    (x * radial + tangential_x, y * radial + tangential_y)
                } else {
                    (x, y)
                }
            },
        }
    }
}

/// Fisheye camera model for wide-angle lenses
#[derive(Debug, Clone)]
pub struct FisheyeCamera {
    /// Fisheye projection model
    pub model: FisheyeModel,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FisheyeModel {
    /// Equidistant projection
    Equidistant,
    /// Equisolid angle projection
    Equisolid,
    /// Stereographic projection
    Stereographic,
}

impl Default for FisheyeCamera {
    fn default() -> Self {
        Self {
            model: FisheyeModel::Equidistant,
        }
    }
}

impl CameraModel for FisheyeCamera {
    fn project(&self, point_3d: &na::Vector3<f64>, intrinsics: &[f64]) -> na::Vector2<f64> {
        let fx = intrinsics[0];
        let fy = intrinsics[1];
        let cx = intrinsics[2];
        let cy = intrinsics[3];

        let x = point_3d.x;
        let y = point_3d.y;
        let z = point_3d.z;

        // Incidence angle: angle between the ray and the optical axis
        let r_xy = (x * x + y * y).sqrt();
        let theta = r_xy.atan2(z);

        // Azimuth angle in the image plane
        let phi = y.atan2(x);

        // Apply fisheye projection model to get projected radius
        let r = match self.model {
            FisheyeModel::Equidistant => theta,                       // r = θ
            FisheyeModel::Equisolid => 2.0 * (theta / 2.0).sin(),     // r = 2·sin(θ/2)
            FisheyeModel::Stereographic => 2.0 * (theta / 2.0).tan(), // r = 2·tan(θ/2)
        };

        // Convert to image coordinates
        na::Vector2::new(fx * r * phi.cos() + cx, fy * r * phi.sin() + cy)
    }

    fn unproject(&self, point_2d: &na::Vector2<f64>, intrinsics: &[f64]) -> na::Vector3<f64> {
        let fx = intrinsics[0];
        let fy = intrinsics[1];
        let cx = intrinsics[2];
        let cy = intrinsics[3];

        // Normalized image coordinates
        let mx = (point_2d.x - cx) / fx;
        let my = (point_2d.y - cy) / fy;
        let r = (mx * mx + my * my).sqrt();
        let phi = my.atan2(mx);

        // Invert the projection model to recover incidence angle θ
        let theta = match self.model {
            FisheyeModel::Equidistant => r, // r = θ  ⇒  θ = r
            FisheyeModel::Equisolid => 2.0 * (r / 2.0).clamp(-1.0, 1.0).asin(), // r = 2·sin(θ/2) ⇒ θ = 2·asin(r/2)
            FisheyeModel::Stereographic => 2.0 * (r / 2.0).atan(), // r = 2·tan(θ/2) ⇒ θ = 2·atan(r/2)
        };

        // Reconstruct the 3D bearing vector
        let sin_theta = theta.sin();
        na::Vector3::new(sin_theta * phi.cos(), sin_theta * phi.sin(), theta.cos())
    }

    fn num_intrinsics(&self) -> usize {
        4 // fx, fy, cx, cy
    }

    fn default_intrinsics(&self, image_width: u32, image_height: u32) -> Vec<f64> {
        vec![
            300.0,                     // fx (fisheye typically has lower focal length)
            300.0,                     // fy
            image_width as f64 / 2.0,  // cx
            image_height as f64 / 2.0, // cy
        ]
    }

    fn parameter_names(&self) -> Vec<String> {
        vec![
            "fx".to_string(),
            "fy".to_string(),
            "cx".to_string(),
            "cy".to_string(),
        ]
    }

    fn validate_intrinsics(&self, intrinsics: &[f64]) -> Result<(), String> {
        if intrinsics.len() != self.num_intrinsics() {
            return Err(format!(
                "Expected {} intrinsics, got {}",
                self.num_intrinsics(),
                intrinsics.len()
            ));
        }

        let fx = intrinsics[0];
        let fy = intrinsics[1];

        if fx <= 0.0 || fy <= 0.0 {
            return Err("Focal lengths must be positive".to_string());
        }

        Ok(())
    }

    fn name(&self) -> &str {
        match self.model {
            FisheyeModel::Equidistant => "Fisheye (equidistant)",
            FisheyeModel::Equisolid => "Fisheye (equisolid)",
            FisheyeModel::Stereographic => "Fisheye (stereographic)",
        }
    }

    fn clone_box(&self) -> Box<dyn CameraModel> {
        Box::new(self.clone())
    }
}

/// Enum for different camera model types
#[derive(Debug, Clone)]
pub enum CameraModelEnum {
    /// Pinhole camera model
    Pinhole(PinholeCamera),
    /// Fisheye camera model
    Fisheye(FisheyeCamera),
}

/// Dispatch a method call through `CameraModelEnum` to the inner model.
macro_rules! dispatch_camera {
    ($self:expr, $method:ident ( $($arg:expr),* $(,)? )) => {
        match $self {
            CameraModelEnum::Pinhole(m) => m.$method($($arg),*),
            CameraModelEnum::Fisheye(m) => m.$method($($arg),*),
        }
    };
}

impl CameraModel for CameraModelEnum {
    fn project(&self, point_3d: &na::Vector3<f64>, intrinsics: &[f64]) -> na::Vector2<f64> {
        dispatch_camera!(self, project(point_3d, intrinsics))
    }

    fn unproject(&self, point_2d: &na::Vector2<f64>, intrinsics: &[f64]) -> na::Vector3<f64> {
        dispatch_camera!(self, unproject(point_2d, intrinsics))
    }

    fn num_intrinsics(&self) -> usize {
        dispatch_camera!(self, num_intrinsics())
    }

    fn default_intrinsics(&self, image_width: u32, image_height: u32) -> Vec<f64> {
        dispatch_camera!(self, default_intrinsics(image_width, image_height))
    }

    fn parameter_names(&self) -> Vec<String> {
        dispatch_camera!(self, parameter_names())
    }

    fn validate_intrinsics(&self, intrinsics: &[f64]) -> Result<(), String> {
        dispatch_camera!(self, validate_intrinsics(intrinsics))
    }

    fn name(&self) -> &str {
        dispatch_camera!(self, name())
    }

    fn clone_box(&self) -> Box<dyn CameraModel> {
        Box::new(self.clone())
    }
}

/// Camera configuration for multi-camera systems
#[derive(Debug, Clone)]
pub struct CameraConfig {
    /// Unique camera ID
    pub id: String,
    /// Camera model
    pub model: CameraModelEnum,
    /// Image dimensions
    pub image_width: u32,
    pub image_height: u32,
    /// Initial intrinsic parameters (if known)
    pub initial_intrinsics: Option<Vec<f64>>,
}

impl CameraConfig {
    pub const fn new(
        id: String,
        model: CameraModelEnum,
        image_width: u32,
        image_height: u32,
    ) -> Self {
        Self {
            id,
            model,
            image_width,
            image_height,
            initial_intrinsics: None,
        }
    }

    pub fn with_initial_intrinsics(mut self, intrinsics: Vec<f64>) -> Self {
        self.initial_intrinsics = Some(intrinsics);
        self
    }
}
