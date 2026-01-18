//! Float precision configuration for SLAM system
//!
//! This module provides a configurable float type that can be switched between
//! f32 and f64 using Cargo features.
//!
//!
//! # Compile-time selection
//!
//! - Default: Uses `f64` (double precision)
//! - With `use_f32` feature: Uses `f32` (single precision)
//!
//! Build with f32:
//! ```bash
//! cargo build --features use_f32
//! ```

#[cfg(feature = "use_f32")]
/// Float type for SLAM computations (f32 precision)
pub type Float = f32;

#[cfg(not(feature = "use_f32"))]
/// Float type for SLAM computations (f64 precision, default)
pub type Float = f64;

/// Float constants that work with both f32 and f64
pub mod float_const {
    use super::Float;

    pub const ZERO: Float = 0.0;
    pub const ONE: Float = 1.0;
    pub const TWO: Float = 2.0;
    pub const HALF: Float = 0.5;
    pub const PI: Float = std::f64::consts::PI as Float;
    pub const EPSILON: Float = 1e-10 as Float;
}

/// Macro to cast literals to Float type (f32 or f64)
#[macro_export]
macro_rules! fl {
    ($val:expr) => {
        $val as $crate::types::Float
    };
}

// Re-export nalgebra types with the configured float precision
use crate::traits::Convert;
use nalgebra as na;
pub type Matrix4x4 = na::Matrix4<Float>;
pub type Matrix3x3 = na::Matrix3<Float>;
pub type Matrix2x2 = na::Matrix2<Float>;
pub type Point3 = na::Point3<Float>;
pub type Vector3 = na::Vector3<Float>;
pub type Vector2 = na::Vector2<Float>;
pub type UnitQuaternion = na::UnitQuaternion<Float>;
pub type Isometry3 = na::Isometry3<Float>;
pub type Matrix6 = na::Matrix6<Float>;
pub type DVector = na::DVector<Float>;
pub type DMatrix = na::DMatrix<Float>;
pub type SMatrix<const R: usize, const C: usize> = na::SMatrix<Float, R, C>;
pub type SVector<const N: usize> = na::SVector<Float, N>;
pub type Array4x4 = [[Float; 4]; 4];
pub type Array3x3 = [[Float; 3]; 3];
pub type Array3 = [Float; 3];
pub type Array2 = [Float; 2];

/// Newtype wrapper for Array4x4 to enable Display trait
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Array4x4Display(pub Array4x4);

/// Newtype wrapper for Array3x3 to enable Display trait
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Array3x3Display(pub Array3x3);

// ============================================================================
// Display implementations and formatting functions
// ============================================================================

use std::fmt;

/// Display implementation for Array4x4Display
impl fmt::Display for Array4x4Display {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Array4x4:")?;
        for row in self.0.iter() {
            writeln!(
                f,
                "  [{:8.4}, {:8.4}, {:8.4}, {:8.4}]",
                row[0], row[1], row[2], row[3]
            )?;
        }
        Ok(())
    }
}

/// Display implementation for Array3x3Display
impl fmt::Display for Array3x3Display {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Array3x3:")?;
        for row in self.0.iter() {
            writeln!(f, "  [{:8.4}, {:8.4}, {:8.4}]", row[0], row[1], row[2])?;
        }
        Ok(())
    }
}

/// Format Array4x4 as a string
pub fn format_array4x4(arr: &Array4x4) -> String {
    format!("{}", Array4x4Display(*arr))
}

/// Format Array3x3 as a string
pub fn format_array3x3(arr: &Array3x3) -> String {
    format!("{}", Array3x3Display(*arr))
}

/// Format Matrix4x4 as a string (since we can't implement Display for type aliases)
pub fn format_matrix4x4(mat: &Matrix4x4) -> String {
    let mut s = String::from("Matrix4x4:\n");
    for i in 0..4 {
        s.push_str(&format!(
            "  [{:8.4}, {:8.4}, {:8.4}, {:8.4}]\n",
            mat[(i, 0)],
            mat[(i, 1)],
            mat[(i, 2)],
            mat[(i, 3)]
        ));
    }
    s
}

/// Format Matrix3x3 as a string (since we can't implement Display for type aliases)
pub fn format_matrix3x3(mat: &Matrix3x3) -> String {
    let mut s = String::from("Matrix3x3:\n");
    for i in 0..3 {
        s.push_str(&format!(
            "  [{:8.4}, {:8.4}, {:8.4}]\n",
            mat[(i, 0)],
            mat[(i, 1)],
            mat[(i, 2)]
        ));
    }
    s
}

// ============================================================================
// Convert<T> implementations (unified conversion trait)
// Replaces fragmented ToMatrix/ToVector/ToArray/ToArrayVec traits
// ============================================================================

impl Convert<Matrix4x4> for Array4x4 {
    fn convert(&self) -> Matrix4x4 {
        na::Matrix4::from_row_slice(&[
            self[0][0], self[0][1], self[0][2], self[0][3], self[1][0], self[1][1], self[1][2],
            self[1][3], self[2][0], self[2][1], self[2][2], self[2][3], self[3][0], self[3][1],
            self[3][2], self[3][3],
        ])
    }
}

impl Convert<Matrix3x3> for Array3x3 {
    fn convert(&self) -> Matrix3x3 {
        na::Matrix3::from_row_slice(&[
            self[0][0], self[0][1], self[0][2], self[1][0], self[1][1], self[1][2], self[2][0],
            self[2][1], self[2][2],
        ])
    }
}

impl Convert<Vector3> for Array3 {
    fn convert(&self) -> Vector3 {
        na::Vector3::new(self[0], self[1], self[2])
    }
}

impl Convert<Vector2> for Array2 {
    fn convert(&self) -> Vector2 {
        na::Vector2::new(self[0], self[1])
    }
}

impl Convert<Array4x4> for Matrix4x4 {
    fn convert(&self) -> Array4x4 {
        [
            [self[(0, 0)], self[(0, 1)], self[(0, 2)], self[(0, 3)]],
            [self[(1, 0)], self[(1, 1)], self[(1, 2)], self[(1, 3)]],
            [self[(2, 0)], self[(2, 1)], self[(2, 2)], self[(2, 3)]],
            [self[(3, 0)], self[(3, 1)], self[(3, 2)], self[(3, 3)]],
        ]
    }
}

impl Convert<Array3x3> for Matrix3x3 {
    fn convert(&self) -> Array3x3 {
        [
            [self[(0, 0)], self[(0, 1)], self[(0, 2)]],
            [self[(1, 0)], self[(1, 1)], self[(1, 2)]],
            [self[(2, 0)], self[(2, 1)], self[(2, 2)]],
        ]
    }
}

impl Convert<Array3> for Vector3 {
    fn convert(&self) -> Array3 {
        [self[0], self[1], self[2]]
    }
}

impl Convert<Array2> for Vector2 {
    fn convert(&self) -> Array2 {
        [self[0], self[1]]
    }
}

// ============================================================================
// Camera Factory: Centralized camera model creation
// ============================================================================

/// Factory for creating camera models from configuration
///
/// This factory centralizes camera creation logic, eliminating duplication
/// across dataset players and other camera initialization sites.
pub struct CameraFactory;

impl CameraFactory {
    /// Create an OpenCV5 camera model from intrinsic parameters
    ///
    /// # Arguments
    /// * `fx`, `fy` - Focal lengths
    /// * `cx`, `cy` - Principal point coordinates
    /// * `k1`, `k2`, `k3` - Radial distortion coefficients
    /// * `p1`, `p2` - Tangential distortion coefficients
    /// * `width` - Image width in pixels
    /// * `height` - Image height in pixels
    ///
    /// # Returns
    /// A CameraModelType::OpenCV5 camera model ready for use
    #[allow(clippy::too_many_arguments)]
    pub fn opencv5(
        fx: Float,
        fy: Float,
        cx: Float,
        cy: Float,
        k1: Float,
        k2: Float,
        k3: Float,
        p1: Float,
        p2: Float,
        width: u32,
        height: u32,
    ) -> crate::datasets::CameraModelType {
        use camera_intrinsic_model::models::opencv5::OpenCVModel5;
        use nalgebra034::DVector;

        // camera-intrinsic-model requires f64, so cast if using f32
        let params = DVector::from_vec(vec![
            fx as f64, fy as f64, cx as f64, cy as f64, k1 as f64, k2 as f64, k3 as f64, p1 as f64,
            p2 as f64,
        ]);

        crate::datasets::CameraModelType::OpenCV5(OpenCVModel5::new(&params, width, height))
    }
}
