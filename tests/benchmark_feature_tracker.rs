use image::GrayImage;
use nalgebra as na;
use rs_vio::feature_tracker::feature_tracker::track_point_at_level;
use rs_vio::feature_tracker::patch::Pattern52;
use std::time::Instant;

#[test]
fn benchmark_track_point_at_level_loop() {
    let width = 640;
    let height = 480;
    // Create a synthetic image with some gradient to allow tracking to converge
    let mut image = GrayImage::new(width, height);
    for y in 0..height {
        for x in 0..width {
            let val = ((x as f32 / width as f32) * 255.0) as u8;
            image.put_pixel(x, y, image::Luma([val]));
        }
    }

    let cx = 320.0;
    let cy = 240.0;
    let _patch = Pattern52::new(&image, cx, cy);

    let mut start_transform = na::Affine2::identity();
    start_transform.matrix_mut_unchecked()[(0, 2)] = cx + 0.5;
    start_transform.matrix_mut_unchecked()[(1, 2)] = cy + 0.5;

    let iterations = 10_000;
    let start = Instant::now();

    let mut success_count = 0;
    for _ in 0..iterations {
        // Benchmark Pattern52 creation too
        let patch = Pattern52::new(&image, cx, cy);

        // Reset transform slightly off to force iterations
        let mut transform = start_transform;

        // We track against the same patch/image, it should converge quickly
        let converged = track_point_at_level(
            &image,
            &patch,
            &mut transform,
            10, // max iterations
            0.01,
        );
        if converged {
            success_count += 1;
        }
    }

    let duration = start.elapsed();
    println!(
        "Track point 10k times: {:?}, avg: {:?}",
        duration,
        duration / iterations as u32
    );
    println!("Success count: {}", success_count);

    // Correctness: tracking same patch against same image should mostly converge
    assert!(
        success_count > iterations / 2,
        "Tracker should converge >50% of the time on identical patch, got {}/{}",
        success_count,
        iterations
    );
}
