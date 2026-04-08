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

// Re-export nalgebra types with the configured float precision
use nalgebra as na;
pub type Matrix4x4 = na::Matrix4<Float>;
pub type Matrix3x3 = na::Matrix3<Float>;
pub type Matrix2x2 = na::Matrix2<Float>;
pub type Vector3 = na::Vector3<Float>;
pub type Vector2 = na::Vector2<Float>;
pub type UnitQuaternion = na::UnitQuaternion<Float>;
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
// Conversion traits: Array -> Matrix
// ============================================================================

/// Trait for converting array types to nalgebra matrix types
pub trait ToMatrix {
    type Output;
    fn to_matrix(&self) -> Self::Output;
}

impl ToMatrix for Array4x4 {
    type Output = Matrix4x4;
    fn to_matrix(&self) -> Self::Output {
        na::Matrix4::from_row_slice(&[
            self[0][0], self[0][1], self[0][2], self[0][3], self[1][0], self[1][1], self[1][2],
            self[1][3], self[2][0], self[2][1], self[2][2], self[2][3], self[3][0], self[3][1],
            self[3][2], self[3][3],
        ])
    }
}

impl ToMatrix for Array3x3 {
    type Output = Matrix3x3;
    fn to_matrix(&self) -> Self::Output {
        na::Matrix3::from_row_slice(&[
            self[0][0], self[0][1], self[0][2], self[1][0], self[1][1], self[1][2], self[2][0],
            self[2][1], self[2][2],
        ])
    }
}

/// Trait for converting array types to nalgebra vector types
pub trait ToVector {
    type Output;
    fn to_vector(&self) -> Self::Output;
}

impl ToVector for Array3 {
    type Output = Vector3;
    fn to_vector(&self) -> Self::Output {
        na::Vector3::new(self[0], self[1], self[2])
    }
}

impl ToVector for Array2 {
    type Output = Vector2;
    fn to_vector(&self) -> Self::Output {
        na::Vector2::new(self[0], self[1])
    }
}

// ============================================================================
// Conversion traits: Matrix -> Array
// ============================================================================

/// Trait for converting nalgebra matrix types to array types
pub trait ToArray {
    type Output;
    fn to_array(&self) -> Self::Output;
}

impl ToArray for Matrix4x4 {
    type Output = Array4x4;
    fn to_array(&self) -> Self::Output {
        [
            [self[(0, 0)], self[(0, 1)], self[(0, 2)], self[(0, 3)]],
            [self[(1, 0)], self[(1, 1)], self[(1, 2)], self[(1, 3)]],
            [self[(2, 0)], self[(2, 1)], self[(2, 2)], self[(2, 3)]],
            [self[(3, 0)], self[(3, 1)], self[(3, 2)], self[(3, 3)]],
        ]
    }
}

impl ToArray for Matrix3x3 {
    type Output = Array3x3;
    fn to_array(&self) -> Self::Output {
        [
            [self[(0, 0)], self[(0, 1)], self[(0, 2)]],
            [self[(1, 0)], self[(1, 1)], self[(1, 2)]],
            [self[(2, 0)], self[(2, 1)], self[(2, 2)]],
        ]
    }
}

/// Trait for converting nalgebra vector types to array types
pub trait ToArrayVec {
    type Output;
    fn to_array(&self) -> Self::Output;
}

impl ToArrayVec for Vector3 {
    type Output = Array3;
    fn to_array(&self) -> Self::Output {
        [self[0], self[1], self[2]]
    }
}

impl ToArrayVec for Vector2 {
    type Output = Array2;
    fn to_array(&self) -> Self::Output {
        [self[0], self[1]]
    }
}
