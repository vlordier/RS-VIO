//! BRIEF descriptor computation with rotation

use crate::optimization::loop_closure::descriptor_pool::OrbBinaryPool;
use std::sync::Arc;

/// Extract BRIEF descriptor (simplified)
/// In practice, use pre-defined BRIEF pattern; here we use pseudo-random tests
pub fn extract_brief(
    image: &[u8],
    x: usize,
    y: usize,
    width: usize,
    orientation: f64,
    pool: Option<&Arc<OrbBinaryPool>>,
) -> Vec<u8> {
    let mut descriptor = if let Some(p) = pool {
        p.acquire_binary().unwrap_or_else(|| vec![0u8; 32])
    } else {
        vec![0u8; 32]
    };
    let height = image.len() / width;
    let cos_angle = orientation.cos();
    let sin_angle = orientation.sin();

    // 256 bit tests (32 bytes)
    for i in 0..256 {
        // Pseudo-random test points (in practice, use ORB's fixed pattern)
        let p1_x = ((i as f64).sin() * 10.0) as i32;
        let p1_y = ((i as f64).cos() * 10.0) as i32;
        let p2_x = ((i as f64).sin() * 15.0) as i32;
        let p2_y = ((i as f64).cos() * 15.0) as i32;

        // Rotate points by orientation
        let r1_x = ((p1_x as f64 * cos_angle - p1_y as f64 * sin_angle) as i32).clamp(-16, 16);
        let r1_y = ((p1_x as f64 * sin_angle + p1_y as f64 * cos_angle) as i32).clamp(-16, 16);
        let r2_x = ((p2_x as f64 * cos_angle - p2_y as f64 * sin_angle) as i32).clamp(-16, 16);
        let r2_y = ((p2_x as f64 * sin_angle + p2_y as f64 * cos_angle) as i32).clamp(-16, 16);

        // Apply test
        let x1 = ((x as i32 + r1_x) as usize).min(width - 1);
        let y1 = ((y as i32 + r1_y) as usize).min(height - 1);
        let x2 = ((x as i32 + r2_x) as usize).min(width - 1);
        let y2 = ((y as i32 + r2_y) as usize).min(height - 1);

        let intensity1 = image.get(y1 * width + x1).copied().unwrap_or(128) as i32;
        let intensity2 = image.get(y2 * width + x2).copied().unwrap_or(128) as i32;

        let bit = if intensity1 > intensity2 { 1u8 } else { 0u8 };
        let byte_idx = i / 8;
        let bit_idx = i % 8;
        descriptor[byte_idx] |= bit << bit_idx;
    }

    descriptor
}
