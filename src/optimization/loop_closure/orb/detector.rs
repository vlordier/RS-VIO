//! Feature detection using FAST corners

use super::types::OrbConfig;

/// Compute FAST9 corner strength (simplified)
pub fn fast_corner_score(
    image: &[u8],
    x: usize,
    y: usize,
    width: usize,
    _config: &OrbConfig,
) -> Option<f64> {
    if x < 3 || y < 3 || x >= width - 3 {
        return None;
    }

    let height = image.len() / width;
    if y >= height - 3 {
        return None;
    }

    let center = image[y * width + x] as f64;
    let threshold = 50.0;

    // Sample 8 neighbors in circle
    let offsets = [
        (-1, -3),
        (-2, -2),
        (-3, -1),
        (-3, 0),
        (-3, 1),
        (-2, 2),
        (-1, 3),
        (0, 3),
    ];

    let mut high_count = 0;
    let mut low_count = 0;

    for (dx, dy) in offsets {
        let nx = (x as i32 + dx) as usize;
        let ny = (y as i32 + dy) as usize;
        let neighbor = image[ny * width + nx] as f64;

        if neighbor > center + threshold {
            high_count += 1;
        } else if neighbor < center - threshold {
            low_count += 1;
        }
    }

    if high_count >= 3 || low_count >= 3 {
        Some((high_count + low_count) as f64)
    } else {
        None
    }
}

/// Compute corner orientation using intensity centroid
pub fn compute_orientation(
    image: &[u8],
    x: usize,
    y: usize,
    width: usize,
    config: &OrbConfig,
) -> f64 {
    use std::f64::consts::PI;

    let patch_size = config.patch_size;
    let half_patch = patch_size / 2;
    let height = image.len() / width;

    let mut m10 = 0.0f64;
    let mut m01 = 0.0f64;

    for py in 0..patch_size {
        for px in 0..patch_size {
            let py_signed = py as i32 - half_patch as i32;
            let px_signed = px as i32 - half_patch as i32;

            let img_y = (y as i32 + py_signed) as usize;
            let img_x = (x as i32 + px_signed) as usize;

            if img_x < width && img_y < height {
                let intensity = image[img_y * width + img_x] as f64;
                m10 += px_signed as f64 * intensity;
                m01 += py_signed as f64 * intensity;
            }
        }
    }

    let angle = m01.atan2(m10);
    if angle < 0.0 {
        angle + 2.0 * PI
    } else {
        angle
    }
}
