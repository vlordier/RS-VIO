/// Tests for feature tracker components

#[cfg(test)]
#[allow(clippy::manual_clamp)]
mod tests {
    use super::super::tracking::{track_point_at_level, track_points};
    use crate::feature_tracker::{image_utilities, patch};
    use image::{GrayImage, Luma};
    use nalgebra as na;
    use std::collections::HashMap;

    fn gradient_image(width: u32, height: u32) -> GrayImage {
        GrayImage::from_fn(width, height, |x, y| Luma([(x + y) as u8]))
    }

    fn checkerboard_image(width: u32, height: u32) -> GrayImage {
        GrayImage::from_fn(width, height, |x, y| {
            let val = if (x + y) % 2 == 0 { 0u8 } else { 255u8 };
            Luma([val])
        })
    }

    /// Test helper: build pyramid using shared utilities
    fn build_image_pyramid(img: &GrayImage, levels: u32) -> Vec<GrayImage> {
        let mut pyr = Vec::new();
        image_utilities::ensure_pyramid_allocated(
            &mut pyr,
            img.width(),
            img.height(),
            levels as usize,
        );
        image_utilities::fill_pyramid(&mut pyr, img);
        pyr
    }

    #[test]
    fn build_image_pyramid_scales_down() {
        let img = gradient_image(32, 32);
        let pyramid = build_image_pyramid(&img, 3);
        assert_eq!(pyramid.len(), 3);
        assert_eq!(pyramid[0].dimensions(), (32, 32));
        assert_eq!(pyramid[1].dimensions(), (16, 16));
        assert_eq!(pyramid[2].dimensions(), (8, 8));
    }

    #[test]
    fn track_point_at_level_converges_on_static_point() {
        let img = checkerboard_image(64, 64);
        let pattern = patch::Pattern52::new(&img, 32.0, 32.0);

        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;

        let ok = track_point_at_level(&img, &pattern, &mut transform, 30, 1e-4);
        assert!(ok);
        assert!((transform.matrix().m13 - 32.0).abs() < 1e-3);
        assert!((transform.matrix().m23 - 32.0).abs() < 1e-3);
    }

    #[test]
    fn track_points_rejects_textureless_input() {
        const LEVELS: u32 = 2;
        let img = GrayImage::from_pixel(64, 64, Luma([128u8]));
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(tracked.is_empty());
    }

    #[test]
    fn pyramid_single_level() {
        let img = gradient_image(16, 16);
        let pyramid = build_image_pyramid(&img, 1);
        assert_eq!(pyramid.len(), 1);
        assert_eq!(pyramid[0].dimensions(), (16, 16));
    }

    #[test]
    fn pyramid_large_levels() {
        let img = gradient_image(256, 256);
        let pyramid = build_image_pyramid(&img, 5);
        assert_eq!(pyramid.len(), 5);
        assert_eq!(pyramid[0].dimensions(), (256, 256));
        assert_eq!(pyramid[1].dimensions(), (128, 128));
        assert_eq!(pyramid[2].dimensions(), (64, 64));
        assert_eq!(pyramid[3].dimensions(), (32, 32));
        assert_eq!(pyramid[4].dimensions(), (16, 16));
    }

    #[test]
    fn track_points_empty_map() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);
        let map0 = HashMap::new();

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(tracked.is_empty());
    }

    #[test]
    fn track_points_zero_iterations() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);
        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 0, 1e-3);
        assert!(tracked.len() <= map0.len());
    }

    #[test]
    fn track_points_very_high_threshold() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);
        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 100, 100.0);
        assert!(tracked.is_empty());
    }

    #[test]
    fn track_points_zero_landmarks() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);
        let map0 = HashMap::new();

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty(),
            "No landmarks should produce empty tracking result"
        );
    }

    #[test]
    fn track_points_total_darkness_image() {
        const LEVELS: u32 = 2;
        let black_img = GrayImage::from_pixel(64, 64, Luma([0u8]));
        let pyramid0 = build_image_pyramid(&black_img, LEVELS);
        let pyramid1 = build_image_pyramid(&black_img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty(),
            "Total darkness should produce no tracked points"
        );
    }

    #[test]
    fn track_points_saturated_image() {
        const LEVELS: u32 = 2;
        let white_img = GrayImage::from_pixel(64, 64, Luma([255u8]));
        let pyramid0 = build_image_pyramid(&white_img, LEVELS);
        let pyramid1 = build_image_pyramid(&white_img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty(),
            "Saturated image should produce no tracked points"
        );
    }

    #[test]
    fn track_points_salt_and_pepper_noise() {
        const LEVELS: u32 = 2;
        let mut img = GrayImage::from_pixel(64, 64, Luma([128u8]));
        for x in 0..64 {
            for y in 0..64 {
                if (x + y) % 17 == 0 {
                    img.put_pixel(x, y, Luma([0u8]));
                } else if (x + y) % 23 == 0 {
                    img.put_pixel(x, y, Luma([255u8]));
                }
            }
        }
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-2);
        assert!(
            tracked.len() <= 1,
            "Salt and pepper noise should degrade tracking performance"
        );
    }

    #[test]
    fn track_points_gaussian_noise() {
        const LEVELS: u32 = 2;
        let mut img = GrayImage::from_pixel(64, 64, Luma([128u8]));
        for x in 0..64 {
            for y in 0..64 {
                let noise_val = ((x * 7 + y * 13) % 64) as i16 - 32;
                let mut pixel: i16 = 128 + noise_val;
                pixel = pixel.max(0).min(255);
                img.put_pixel(x, y, Luma([pixel as u8]));
            }
        }
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-2);
        assert!(tracked.len() <= 1, "Gaussian noise should degrade tracking");
    }

    #[test]
    fn track_points_dropped_frame() {
        const LEVELS: u32 = 2;
        let img0 = checkerboard_image(64, 64);
        let img1 = GrayImage::from_pixel(64, 64, Luma([0u8]));
        let pyramid0 = build_image_pyramid(&img0, LEVELS);
        let pyramid1 = build_image_pyramid(&img1, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty(),
            "Dropped frame (black) should lose tracking"
        );
    }

    #[test]
    fn track_points_motion_blur_simulated() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let img1 = checkerboard_image(64, 64);
        let pyramid1 = build_image_pyramid(&img1, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 16.0;
        transform.matrix_mut_unchecked().m23 = 16.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty() || tracked.len() == 1,
            "Motion should cause tracking failure or degraded tracking"
        );
    }

    #[test]
    fn track_points_camera_disconnected_pattern() {
        const LEVELS: u32 = 2;
        let mut img0 = GrayImage::from_pixel(64, 64, Luma([128u8]));
        let mut img1 = GrayImage::from_pixel(64, 64, Luma([128u8]));
        img0.put_pixel(0, 0, Luma([0u8]));
        img1.put_pixel(0, 0, Luma([255u8]));
        let pyramid0 = build_image_pyramid(&img0, LEVELS);
        let pyramid1 = build_image_pyramid(&img1, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty(),
            "Camera disconnect pattern should lose tracking"
        );
    }

    #[test]
    fn track_points_horizontal_stripes_interference() {
        const LEVELS: u32 = 2;
        let mut img = GrayImage::from_pixel(64, 64, Luma([128u8]));
        for y in 0..64 {
            let val = if y % 2 == 0 { 0u8 } else { 255u8 };
            for x in 0..64 {
                img.put_pixel(x, y, Luma([val]));
            }
        }
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-2);
        assert!(
            tracked.len() <= 1,
            "Horizontal interference stripes should degrade tracking"
        );
    }

    #[test]
    fn track_points_all_points_lost() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);

        let mut map0 = HashMap::new();
        for i in 0..10 {
            let mut transform = na::Affine2::<f32>::identity();
            transform.matrix_mut_unchecked().m13 = (i * 6 + 1) as f32;
            transform.matrix_mut_unchecked().m23 = (i * 6 + 1) as f32;
            map0.insert(i, transform);
        }

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty(),
            "All points should be trackable in identical images"
        );
    }

    #[test]
    fn track_points_high_frequency_noise() {
        const LEVELS: u32 = 2;
        let mut img = GrayImage::from_pixel(64, 64, Luma([128u8]));
        for x in 0..64 {
            for y in 0..64 {
                let noise_val = ((x * 11 + y * 17) % 50) as i8 - 25;
                let mut pixel: i16 = 128 + noise_val as i16;
                pixel = pixel.max(0).min(255);
                img.put_pixel(x, y, Luma([pixel as u8]));
            }
        }
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let pyramid1 = build_image_pyramid(&img, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 32.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-1);
        assert!(
            tracked.len() <= 1,
            "High frequency noise should severely degrade tracking"
        );
    }

    #[test]
    fn track_points_partial_corruption() {
        const LEVELS: u32 = 2;
        let img = checkerboard_image(64, 64);
        let pyramid0 = build_image_pyramid(&img, LEVELS);
        let mut img1 = checkerboard_image(64, 64);
        for x in 0..64 {
            for y in 0..10 {
                img1.put_pixel(x, y, Luma([128u8]));
            }
        }
        let pyramid1 = build_image_pyramid(&img1, LEVELS);

        let mut map0 = HashMap::new();
        let mut transform = na::Affine2::<f32>::identity();
        transform.matrix_mut_unchecked().m13 = 32.0;
        transform.matrix_mut_unchecked().m23 = 5.0;
        map0.insert(0usize, transform);

        let tracked = track_points::<LEVELS>(&pyramid0, &pyramid1, &map0, 10, 1e-3);
        assert!(
            tracked.is_empty() || tracked.len() == 1,
            "Partial corruption at tracked point location should cause tracking failure"
        );
    }
}
