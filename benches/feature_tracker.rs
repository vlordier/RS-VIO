//! Benchmarks for the feature tracking hot paths.
//!
//! Run with: `cargo bench --bench feature_tracker`
//! Save baseline on VIO branch: `cargo bench --bench feature_tracker -- --save-baseline vio`
//! Compare on main: `cargo bench --bench feature_tracker -- --baseline vio`

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use image::GrayImage;
use imageproc::corners::Corner;
use nalgebra as na;
use rand::Rng;
use rs_vio::estimator::Frame;
use rs_vio::feature_tracker::{
    build_image_pyramid, image_utilities,
    patch::{Pattern52, PATTERN52_SIZE},
    StereoPatchTracker,
};

// ---------------------------------------------------------------------------
// Synthetic image helpers
// ---------------------------------------------------------------------------

/// 640×480 grayscale image with a smooth sinusoidal texture — strong gradients
/// for reliable LK tracking, no aliasing artifacts from hard edges.
fn smooth_image(width: u32, height: u32) -> GrayImage {
    GrayImage::from_fn(width, height, |x, y| {
        let v = 128.0
            + 60.0 * ((x as f32 / 23.0).sin() * (y as f32 / 17.0).cos())
            + 40.0 * ((x as f32 / 7.0 + y as f32 / 13.0).sin());
        image::Luma([(v.round() as i16).clamp(0, 255) as u8])
    })
}

/// Checkerboard image: fast to generate, lots of FAST corners.
fn checkerboard_image(width: u32, height: u32) -> GrayImage {
    let mut rng = rand::thread_rng();
    GrayImage::from_fn(width, height, |x, y| {
        let cell = ((x / 8) + (y / 8)) % 2;
        let base: u8 = if cell == 0 { 180 } else { 60 };
        let noise: i16 = rng.gen_range(-15..15);
        image::Luma([(base as i16 + noise).clamp(0, 255) as u8])
    })
}

// ---------------------------------------------------------------------------
// Pattern52::new
// ---------------------------------------------------------------------------

fn bench_pattern52_new(c: &mut Criterion) {
    let img = smooth_image(640, 480);
    c.bench_function("Pattern52::new", |b| {
        b.iter(|| Pattern52::new(&img, 320.0, 240.0))
    });
}

// ---------------------------------------------------------------------------
// Pattern52::residual
// ---------------------------------------------------------------------------

fn bench_pattern52_residual(c: &mut Criterion) {
    let img = smooth_image(640, 480);
    let pattern = Pattern52::new(&img, 320.0, 240.0);
    assert!(pattern.valid, "Smooth image must produce a valid patch");

    let patten = na::SMatrix::<f32, 2, PATTERN52_SIZE>::from_column_slice(
        &Pattern52::PATTERN_MATRIX_COL_MAJOR,
    );
    let transform = {
        let mut t = na::Affine2::<f32>::identity();
        t.matrix_mut_unchecked().m13 = 322.0;
        t.matrix_mut_unchecked().m23 = 241.0;
        t
    };
    let mut transformed_pat = transform.matrix().fixed_view::<2, 2>(0, 0) * &patten;
    let translation = transform.matrix().fixed_view::<2, 1>(0, 2);
    for mut col in transformed_pat.column_iter_mut() {
        col += translation;
    }
    let center = (transform.matrix().m13, transform.matrix().m23);

    c.bench_function("Pattern52::residual", |b| {
        b.iter(|| pattern.residual(&img, &transformed_pat, center))
    });
}

// ---------------------------------------------------------------------------
// track_point_at_level  (single-level LK inner loop)
// ---------------------------------------------------------------------------

fn bench_track_point_at_level(c: &mut Criterion) {
    let img = smooth_image(640, 480);
    let pattern = Pattern52::new(&img, 320.0, 240.0);
    assert!(pattern.valid);

    // Offset starting guess by 3 pixels so LK has real work to do
    let mut transform = na::Affine2::<f32>::identity();
    transform.matrix_mut_unchecked().m13 = 323.0;
    transform.matrix_mut_unchecked().m23 = 243.0;

    c.bench_function("track_point_at_level", |b| {
        b.iter(|| {
            let mut t = transform;
            rs_vio::feature_tracker::track_point_at_level(&img, &pattern, &mut t, 30, 0.005)
        })
    });
}

// ---------------------------------------------------------------------------
// build_image_pyramid
// ---------------------------------------------------------------------------

fn bench_build_image_pyramid(c: &mut Criterion) {
    let img = smooth_image(640, 480);
    let mut group = c.benchmark_group("build_image_pyramid");
    for &levels in &[3u32, 4u32] {
        group.bench_with_input(
            BenchmarkId::from_parameter(format!("L{}", levels)),
            &levels,
            |b, &l| b.iter(|| build_image_pyramid(&img, l)),
        );
    }
    group.finish();
}

// ---------------------------------------------------------------------------
// detect_key_points  (FAST + grid)
// ---------------------------------------------------------------------------

fn bench_detect_key_points(c: &mut Criterion) {
    let mut group = c.benchmark_group("detect_key_points");
    for &grid_size in &[20u32, 30u32, 40u32] {
        let img = checkerboard_image(640, 480);
        let empty: Vec<Corner> = Vec::new();
        let mut grid_scratch: Vec<Corner> = Vec::new();
        group.bench_with_input(
            BenchmarkId::from_parameter(format!("grid{}", grid_size)),
            &grid_size,
            |b, &gs| b.iter(|| image_utilities::detect_key_points(&img, gs, &empty, &mut grid_scratch)),
        );
    }
    group.finish();
}

// ---------------------------------------------------------------------------
// Full StereoPatchTracker::process_frame
// ---------------------------------------------------------------------------

fn bench_stereo_tracker_process_frame(c: &mut Criterion) {
    let img0 = checkerboard_image(640, 480);
    let img1 = checkerboard_image(640, 480);

    let mut group = c.benchmark_group("stereo_process_frame");

    group.bench_function("cold_start", |b| {
        b.iter(|| {
            let mut tracker = StereoPatchTracker::<4>::new(30, 20, 0.005);
            let mut frame = Frame::new(0, 0);
            tracker.process_frame(&img0, &img1, &mut frame);
        })
    });

    group.bench_function("warm_tracking", |b| {
        let mut tracker = StereoPatchTracker::<4>::new(30, 20, 0.005);
        let mut frame = Frame::new(0, 0);
        tracker.process_frame(&img0, &img1, &mut frame);
        b.iter(|| {
            let mut frame2 = Frame::new(1, 1);
            tracker.process_frame(&img0, &img1, &mut frame2);
        })
    });

    group.finish();
}

// ---------------------------------------------------------------------------
// Criterion harness
// ---------------------------------------------------------------------------

criterion_group!(
    benches,
    bench_pattern52_new,
    bench_pattern52_residual,
    bench_track_point_at_level,
    bench_build_image_pyramid,
    bench_detect_key_points,
    bench_stereo_tracker_process_frame,
);
criterion_main!(benches);
