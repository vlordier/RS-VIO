#![no_main]
#![allow(unused_crate_dependencies)]

use libfuzzer_sys::fuzz_target;
use rs_vio::math_utils;

fuzz_target!(|data: (Vec<f64>, Vec<f64>, Vec<f64>, f64)| {
    let (a, b, c, threshold) = data;

    // Test matrix operations with limited sizes
    if a.len() > 100 || b.len() > 100 {
        return;
    }

    // Test vector operations
    let vec1: Vec<f64> = a.iter().take(10).cloned().collect();
    let vec2: Vec<f64> = b.iter().take(10).cloned().collect();

    if !vec1.is_empty() && !vec2.is_empty() {
        let _ = math_utils::dot_product(&vec1, &vec2);
        let _ = math_utils::vector_norm(&vec1);
        let _ = math_utils::normalize_vector(&vec1);
    }

    // Test rotation conversions
    if a.len() >= 4 {
        let qx = a[0] % 1.0;
        let qy = a[1] % 1.0;
        let qz = a[2] % 1.0;
        let qw = a[3] % 1.0;
        let _ = math_utils::quaternion_to_rotation_matrix(qx, qy, qz, qw);
        let _ = math_utils::quaternion_normalize(qx, qy, qz, qw);
    }

    // Test angle conversions
    if !c.is_empty() {
        let angle = c[0];
        let _ = math_utils::degrees_to_radians(angle);
        let _ = math_utils::radians_to_degrees(angle);
    }

    // Test projection
    if a.len() >= 3 {
        let point = [a[0], a[1], a[2]];
        let _ = math_utils::project_point_to_plane(&point, &[0.0, 0.0, 1.0], 0.0);
    }

    // Test interpolation
    if a.len() >= 2 && b.len() >= 2 && threshold >= 0.0 && threshold <= 1.0 {
        let _ = math_utils::slerp_quaternions(
            &[a[0], a[1], a[2], 1.0],
            &[b[0], b[1], b[2], 1.0],
            threshold,
        );
    }
});
