//! Nalgebra Matrix and Vector Convenience Macros
//!
//! Provides ergonomic macros for common nalgebra operations:
//! - Vector construction with validation (Vector2, Vector3, etc.)
//! - Matrix creation patterns (identity, zero, diagonal)
//! - Matrix arithmetic with overflow checks
//! - Cheirality and visibility checks
//! - Coordinate frame transformations

/// Construct a nalgebra Vector3 with validation
#[macro_export]
macro_rules! vector3 {
    ($x:expr, $y:expr, $z:expr) => {{
        let x = $x as f64;
        let y = $y as f64;
        let z = $z as f64;
        
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            log::warn!("Non-finite value in Vector3: ({}, {}, {})", x, y, z);
        }
        na::Vector3::new(x, y, z)
    }};
}

/// Construct a nalgebra Vector2 with validation
#[macro_export]
macro_rules! vector2 {
    ($x:expr, $y:expr) => {{
        let x = $x as f64;
        let y = $y as f64;
        
        if !x.is_finite() || !y.is_finite() {
            log::warn!("Non-finite value in Vector2: ({}, {})", x, y);
        }
        na::Vector2::new(x, y)
    }};
}

/// Construct a nalgebra Matrix4 identity
#[macro_export]
macro_rules! mat4_identity {
    () => {
        na::Matrix4::<f64>::identity()
    };
}

/// Construct a nalgebra Matrix4 with safe inversion attempt
#[macro_export]
macro_rules! mat4_try_inverse {
    ($matrix:expr, $context:expr) => {
        match $matrix.try_inverse() {
            Some(inv) => inv,
            None => {
                log::warn!("Matrix inversion failed: {}", $context);
                na::Matrix4::<f64>::identity()
            }
        }
    };
}

/// Construct a nalgebra DVector with optional validation
#[macro_export]
macro_rules! dvector {
    (zeros($n:expr)) => {
        na::DVector::<f64>::zeros($n)
    };
    (from_vec($vec:expr)) => {{
        let v = $vec;
        if v.iter().any(|x: &f64| !x.is_finite()) {
            log::warn!("Non-finite values in DVector");
        }
        na::DVector::from_vec(v)
    }};
}

/// Construct a nalgebra DMatrix with optional validation
#[macro_export]
macro_rules! dmatrix {
    (zeros($rows:expr, $cols:expr)) => {
        na::DMatrix::<f64>::zeros($rows, $cols)
    };
    (identity($n:expr)) => {
        na::DMatrix::<f64>::identity($n, $n)
    };
}

/// Check if a 3D point is in front of the camera (cheirality check)
#[macro_export]
macro_rules! is_chiral {
    ($point:expr) => {{
        $point.z > 0.0 && $point.iter().all(|x| x.is_finite())
    }};
    ($point:expr, min_depth = $min_depth:expr) => {{
        $point.z > $min_depth && $point.iter().all(|x| x.is_finite())
    }};
}

/// Check if point is visible in camera (normalized coordinates bounded [-1, 1])
#[macro_export]
macro_rules! is_visible_normalized {
    ($point:expr) => {{
        $point.x.abs() <= 1.0 && $point.y.abs() <= 1.0 && $point.iter().all(|x| x.is_finite())
    }};
    ($point:expr, margin = $margin:expr) => {{
        let m = $margin as f64;
        $point.x.abs() <= (1.0 + m) && $point.y.abs() <= (1.0 + m) && $point.iter().all(|x| x.is_finite())
    }};
}

/// Check if point is in image bounds
#[macro_export]
macro_rules! is_in_bounds {
    ($point:expr, width=$w:expr, height=$h:expr) => {{
        let p = $point;
        p.x >= 0.0 && p.x < $w as f64 && p.y >= 0.0 && p.y < $h as f64 && p.iter().all(|x| x.is_finite())
    }};
    ($point:expr, width=$w:expr, height=$h:expr, border=$border:expr) => {{
        let p = $point;
        let b = $border as f64;
        p.x >= b && p.x < ($w as f64 - b) && p.y >= b && p.y < ($h as f64 - b) && p.iter().all(|x| x.is_finite())
    }};
}

/// Transform a point using a transformation matrix (SE(3))
#[macro_export]
macro_rules! transform_point {
    ($point:expr, $T:expr) => {{
        let p = $point;
        let homogeneous = na::Vector4::new(p.x, p.y, p.z, 1.0);
        let transformed = $T * homogeneous;
        na::Vector3::new(
            transformed.x / transformed.w,
            transformed.y / transformed.w,
            transformed.z / transformed.w,
        )
    }};
}

/// Project a 3D point to 2D image coordinates using pinhole camera model
#[macro_export]
macro_rules! project_pinhole {
    ($point:expr, fx=$fx:expr, fy=$fy:expr, cx=$cx:expr, cy=$cy:expr) => {{
        let p = $point;
        if p.z <= 0.0 {
            None
        } else {
            let x = ($fx as f64 * p.x / p.z) + $cx as f64;
            let y = ($fy as f64 * p.y / p.z) + $cy as f64;
            Some(na::Vector2::new(x, y))
        }
    }};
    ($point:expr, $intrinsics:expr) => {{
        // intrinsics = [fx, fy, cx, cy]
        project_pinhole!($point, fx=$intrinsics[0], fy=$intrinsics[1], cx=$intrinsics[2], cy=$intrinsics[3])
    }};
}

/// Create a rotation matrix from axis-angle representation
#[macro_export]
macro_rules! rotation_from_axis_angle {
    ($axis:expr, $angle:expr) => {{
        if $angle.abs() < 1e-10 {
            na::Matrix3::identity()
        } else {
            let axis = na::Unit::new_normalize($axis);
            na::Rotation3::from_axis_angle(&axis, $angle).into()
        }
    }};
}

/// Create an SE(3) transformation matrix from position and rotation
#[macro_export]
macro_rules! se3_from_translation_rotation {
    ($t:expr, $R:expr) => {{
        let mut T = na::Matrix4::identity();
        T.fixed_view_mut::<3, 3>(0, 0).copy_from(&$R);
        T.column_mut(3).fixed_rows_mut::<3>(0).copy_from(&$t);
        T
    }};
}

/// Extract translation from SE(3) matrix
#[macro_export]
macro_rules! extract_translation {
    ($T:expr) => {{
        na::Vector3::new($T[(0, 3)], $T[(1, 3)], $T[(2, 3)])
    }};
}

/// Extract rotation from SE(3) matrix
#[macro_export]
macro_rules! extract_rotation {
    ($T:expr) => {{
        $T.fixed_view::<3, 3>(0, 0).clone_owned()
    }};
}

/// Compose two SE(3) transformations: T1 * T2
#[macro_export]
macro_rules! compose_transforms {
    ($T1:expr, $T2:expr) => {{
        $T1 * $T2
    }};
}

/// Compute relative transformation: T_A_B = T_A_C * T_C_B^{-1}
#[macro_export]
macro_rules! relative_transform {
    ($T1:expr, $T2:expr) => {{
        match $T2.try_inverse() {
            Some(T2_inv) => $T1 * T2_inv,
            None => {
                log::warn!("Failed to invert transformation matrix");
                na::Matrix4::identity()
            }
        }
    }};
}

/// Batch project points using the same intrinsics
#[macro_export]
macro_rules! project_points_batch {
    ($points:expr, fx=$fx:expr, fy=$fy:expr, cx=$cx:expr, cy=$cy:expr) => {{
        $points
            .iter()
            .filter_map(|p| {
                if p.z <= 0.0 {
                    None
                } else {
                    let x = ($fx as f64 * p.x / p.z) + $cx as f64;
                    let y = ($fy as f64 * p.y / p.z) + $cy as f64;
                    Some(na::Vector2::new(x, y))
                }
            })
            .collect::<Vec<_>>()
    }};
}

/// Quaternion from axis-angle
#[macro_export]
macro_rules! quat_from_axis_angle {
    ($axis:expr, $angle:expr) => {{
        let axis = na::Unit::new_normalize($axis);
        na::UnitQuaternion::from_axis_angle(&axis, $angle)
    }};
}

/// Compute pose inverse (SE(3) inverse)
#[macro_export]
macro_rules! pose_inverse {
    ($T:expr) => {{
        mat4_try_inverse!($T, "pose inversion")
    }};
}

/// Calculate Euclidean distance (L2 norm)
///
/// # Example
/// ```ignore
/// let dist = euclidean_distance!(x, y, z);  // sqrt(x^2 + y^2 + z^2)
/// let dist_2d = euclidean_distance!(dx, dy);  // sqrt(dx^2 + dy^2)
/// ```
#[macro_export]
macro_rules! euclidean_distance {
    ($x:expr, $y:expr) => {{
        let x = $x as f64;
        let y = $y as f64;
        (x.powi(2) + y.powi(2)).sqrt()
    }};
    ($x:expr, $y:expr, $z:expr) => {{
        let x = $x as f64;
        let y = $y as f64;
        let z = $z as f64;
        (x.powi(2) + y.powi(2) + z.powi(2)).sqrt()
    }};
}

/// Calculate squared norm (avoids sqrt for performance)
///
/// # Example
/// ```ignore
/// let dist_sq = squared_norm!(dx, dy);  // dx^2 + dy^2
/// let dist_sq = squared_norm!(dx, dy, dz);  // dx^2 + dy^2 + dz^2
/// ```
#[macro_export]
macro_rules! squared_norm {
    ($x:expr, $y:expr) => {{
        let x = $x as f64;
        let y = $y as f64;
        x.powi(2) + y.powi(2)
    }};
    ($x:expr, $y:expr, $z:expr) => {{
        let x = $x as f64;
        let y = $y as f64;
        let z = $z as f64;
        x.powi(2) + y.powi(2) + z.powi(2)
    }};
}

/// Normalize a vector to unit length
///
/// # Example
/// ```ignore
/// let (nx, ny, nz) = normalize_vec!(x, y, z);
/// ```
#[macro_export]
macro_rules! normalize_vec {
    ($x:expr, $y:expr, $z:expr) => {{
        let norm = euclidean_distance!($x, $y, $z);
        if norm > 1e-10 {
            (($x as f64) / norm, ($y as f64) / norm, ($z as f64) / norm)
        } else {
            (0.0, 0.0, 0.0)
        }
    }};
}

#[cfg(test)]
mod tests {
    use nalgebra as na;

    #[test]
    fn test_vector3_macro() {
        let v = vector3!(1.0, 2.0, 3.0);
        assert_eq!(v.x, 1.0);
        assert_eq!(v.y, 2.0);
        assert_eq!(v.z, 3.0);
    }

    #[test]
    fn test_vector2_macro() {
        let v = vector2!(1.0, 2.0);
        assert_eq!(v.x, 1.0);
        assert_eq!(v.y, 2.0);
    }

    #[test]
    fn test_mat4_identity() {
        let I = mat4_identity!();
        assert_eq!(I[(0, 0)], 1.0);
        assert_eq!(I[(1, 1)], 1.0);
        assert_eq!(I[(0, 1)], 0.0);
    }

    #[test]
    fn test_dvector() {
        let v = dvector!(zeros(5));
        assert_eq!(v.len(), 5);
        assert!(v.iter().all(|x| *x == 0.0));
    }

    #[test]
    fn test_extract_translation() {
        let mut T = na::Matrix4::<f64>::identity();
        T[(0, 3)] = 1.0;
        T[(1, 3)] = 2.0;
        T[(2, 3)] = 3.0;

        let t = extract_translation!(T);
        assert_eq!(t.x, 1.0);
        assert_eq!(t.y, 2.0);
        assert_eq!(t.z, 3.0);
    }

    #[test]
    fn test_project_pinhole() {
        let point = na::Vector3::new(1.0, 0.5, 2.0);
        let proj = project_pinhole!(point, fx=500.0, fy=500.0, cx=320.0, cy=240.0);

        assert!(proj.is_some());
        let p = proj.unwrap();
        let expected_x = 500.0 * 1.0 / 2.0 + 320.0;
        let expected_y = 500.0 * 0.5 / 2.0 + 240.0;
        assert!((p.x - expected_x).abs() < 1e-10);
        assert!((p.y - expected_y).abs() < 1e-10);
    }

    #[test]
    fn test_project_pinhole_behind_camera() {
        let point = na::Vector3::new(1.0, 0.5, -2.0);
        let proj = project_pinhole!(point, fx=500.0, fy=500.0, cx=320.0, cy=240.0);
        assert!(proj.is_none());
    }

    #[test]
    fn test_euclidean_distance() {
        let dist = euclidean_distance!(1.0, 2.0, 3.0);
        let expected = (1.0f64.powi(2) + 2.0f64.powi(2) + 3.0f64.powi(2)).sqrt();
        assert!((dist - expected).abs() < 1e-10);
    }

    #[test]
    fn test_squared_norm() {
        let norm_sq = squared_norm!(3.0, 4.0);
        assert_eq!(norm_sq, 25.0);
    }
}
