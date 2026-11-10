//! Comprehensive tracker report: memory, accuracy, and quality metrics.
//!
//! Without dataset:
//!   cargo run --release --bin tracker_report
//!
//! With TUM-VI dataset (real-data accuracy section):
//!   cargo run --release --bin tracker_report -- --dataset /path/to/dataset-corridor1_512_16

use clap::Parser;
use image::{GrayImage, ImageReader};
use nalgebra as na;
use rayon;
use std::mem;
use std::path::Path;
use std::time::Instant;

use rs_vio::estimator::Frame;
use rs_vio::feature_tracker::{
    build_image_pyramid, halve_image, image_utilities,
    patch::{Pattern52, PATTERN52_SIZE},
    track_point_at_level, StereoPatchTracker,
};

#[derive(Parser)]
#[command(about = "RS-VIO tracker report — memory, speed, quality, accuracy")]
struct Args {
    /// Path to a TUM-VI dataset directory (e.g. dataset-corridor1_512_16).
    /// When provided, runs the real-data tracking section.
    #[arg(long)]
    dataset: Option<String>,

    /// Maximum number of frames to process from the dataset (default: 300).
    #[arg(long, default_value_t = 300)]
    max_frames: usize,
}

// ---------------------------------------------------------------------------
// Image generators
// ---------------------------------------------------------------------------

/// Smooth sinusoidal texture — strong gradients, no aliasing. Good for LK.
fn smooth_image(width: u32, height: u32) -> GrayImage {
    GrayImage::from_fn(width, height, |x, y| {
        let v = 128.0
            + 60.0 * ((x as f32 / 23.0).sin() * (y as f32 / 17.0).cos())
            + 40.0 * ((x as f32 / 7.0 + y as f32 / 13.0).sin());
        image::Luma([(v.round() as i16).clamp(0, 255) as u8])
    })
}

/// Deterministic checkerboard with noise — produces many FAST corners.
fn checkerboard_image(width: u32, height: u32, seed: u64) -> GrayImage {
    use std::hash::Hash;
    GrayImage::from_fn(width, height, |x, y| {
        let mut h = std::collections::hash_map::DefaultHasher::new();
        (x, y, seed).hash(&mut h);
        let noise = (std::hash::Hasher::finish(&h) % 31) as i16 - 15;
        let cell = ((x / 8) + (y / 8)) % 2;
        let base: u8 = if cell == 0 { 180 } else { 60 };
        image::Luma([(base as i16 + noise).clamp(0, 255) as u8])
    })
}

/// Shift `src` by (dx, dy) pixels using bilinear interpolation.
fn shift_image(src: &GrayImage, dx: f32, dy: f32) -> GrayImage {
    let (w, h) = src.dimensions();
    GrayImage::from_fn(w, h, |x, y| {
        let sx = x as f32 - dx;
        let sy = y as f32 - dy;
        if sx < 1.0 || sy < 1.0 || sx >= (w - 2) as f32 || sy >= (h - 2) as f32 {
            return image::Luma([0u8]);
        }
        let ix = sx as u32;
        let iy = sy as u32;
        let fx = sx - ix as f32;
        let fy = sy - iy as f32;
        let p00 = src.get_pixel(ix, iy)[0] as f32;
        let p10 = src.get_pixel(ix + 1, iy)[0] as f32;
        let p01 = src.get_pixel(ix, iy + 1)[0] as f32;
        let p11 = src.get_pixel(ix + 1, iy + 1)[0] as f32;
        let v = (1.0 - fx) * (1.0 - fy) * p00
            + fx * (1.0 - fy) * p10
            + (1.0 - fx) * fy * p01
            + fx * fy * p11;
        image::Luma([v.round() as u8])
    })
}

fn print_section(title: &str) {
    println!();
    println!("=== {} ===", title);
}

/// Run coarse-to-fine LK tracking from (cx,cy) in pyr0 → pyr1, return position.
fn track_coarse_to_fine(
    cx: f32,
    cy: f32,
    pyr0: &[GrayImage],
    pyr1: &[GrayImage],
) -> Option<(f32, f32)> {
    let mut t = na::Affine2::<f32>::identity();
    t.matrix_mut_unchecked().m13 = cx;
    t.matrix_mut_unchecked().m23 = cy;
    for level in (0..pyr0.len()).rev() {
        let scale = (1u32 << level) as f32;
        t.matrix_mut_unchecked().m13 /= scale;
        t.matrix_mut_unchecked().m23 /= scale;
        let pat = Pattern52::new(&pyr0[level], cx / scale, cy / scale);
        if !pat.valid { return None; }
        if !track_point_at_level(&pyr1[level], &pat, &mut t, 30, 0.005) { return None; }
        t.matrix_mut_unchecked().m13 *= scale;
        t.matrix_mut_unchecked().m23 *= scale;
    }
    Some((t.matrix().m13, t.matrix().m23))
}

// ---------------------------------------------------------------------------
// 1. Memory metrics
// ---------------------------------------------------------------------------

fn report_memory() {
    print_section("MEMORY — struct sizes");

    let p52 = mem::size_of::<Pattern52>();
    let aff = mem::size_of::<na::Affine2<f32>>();
    println!("  Pattern52                          {:>6} bytes", p52);
    println!("  na::Affine2<f32> (tracked point)   {:>6} bytes", aff);
    println!("  Estimated per-tracked-feature      {:>6} bytes (key + Affine2 + map overhead)",
             mem::size_of::<usize>() + aff + 32);
    println!();
    println!("  Pattern52 field layout:");
    println!("    valid: bool                           1 byte");
    println!("    mean:  f32                            4 bytes");
    println!("    pos:   SVector<f32,2>                 8 bytes");
    println!("    data:  [f32; {}]               {} bytes", PATTERN52_SIZE, PATTERN52_SIZE * 4);
    println!("    h_se2_inv_j_se2_t: SMatrix<f32,{},3>  {} bytes (52×3 for SIMD dot products)", PATTERN52_SIZE, 3 * PATTERN52_SIZE * 4);
    println!("    (removed in VIO: pattern_scale_down f32=4, pattern_matrix SMatrix<f32,2,52>=416)");

    let pyramid_total = ((640 * 480) + (320 * 240) + (160 * 120) + (80 * 60)) * 2;
    let map_bytes = 400 * (aff + mem::size_of::<usize>() + 32) * 2;
    println!();
    println!("  Estimated per-frame heap (640×480, ~400 features, 4 pyramid levels):");
    println!("    2× image pyramids:       {:>8} bytes  ({:.1} KB)", pyramid_total, pyramid_total as f32 / 1024.0);
    println!("    tracking maps (cam0+1):  {:>8} bytes  ({:.1} KB)", map_bytes, map_bytes as f32 / 1024.0);
}

// ---------------------------------------------------------------------------
// 2. Tracking accuracy on synthetic known motion
// ---------------------------------------------------------------------------

fn report_accuracy() {
    print_section("ACCURACY — tracking error on synthetic translations");

    let img = smooth_image(640, 480);
    let pyr0 = build_image_pyramid(&img, 4);

    // 5×5 grid of test centres well inside the safe border
    let centres: Vec<(f32, f32)> = (4..=8)
        .flat_map(|gy| (4..=8usize).map(move |gx| (gx as f32 * 70.0, gy as f32 * 55.0)))
        .collect();

    let translations: &[(f32, f32)] = &[
        (1.0, 0.0), (0.0, 1.5), (2.7, 1.3), (-3.1, 2.4), (5.0, -4.0), (0.3, 0.3),
    ];

    println!("  {:>13}  {:>8}  {:>8}  {:>13}  {:>8}", "shift(dx,dy)", "mean_err", "max_err", "tracked/total", "pass<0.5");
    for &(dx, dy) in translations {
        let shifted = shift_image(&img, dx, dy);
        let pyr1 = build_image_pyramid(&shifted, 4);
        let mut errors: Vec<f32> = Vec::new();
        for &(cx, cy) in &centres {
            match track_coarse_to_fine(cx, cy, &pyr0, &pyr1) {
                Some((rx, ry)) => {
                    let ex = rx - (cx + dx);
                    let ey = ry - (cy + dy);
                    errors.push((ex * ex + ey * ey).sqrt());
                }
                None => {}
            }
        }
        let n = centres.len();
        let mean = if errors.is_empty() { f32::NAN } else { errors.iter().sum::<f32>() / errors.len() as f32 };
        let max = errors.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        let pass = errors.iter().filter(|&&e| e < 0.5).count();
        println!("  ({:+5.1},{:+5.1})   {:>8.4}  {:>8.4}  {:>5}/{:<7}  {:>5}/{}", dx, dy, mean, max, errors.len(), n, pass, errors.len());
    }
}

// ---------------------------------------------------------------------------
// 3. Feature detection quality
// ---------------------------------------------------------------------------

fn report_detection_quality() {
    print_section("QUALITY — feature detection");

    let img = checkerboard_image(640, 480, 42);
    let empty: Vec<imageproc::corners::Corner> = Vec::new();

    println!("  {:>10}  {:>10}  {:>10}  {:>10}  {:>10}", "grid_size", "cells", "detected", "fill_rate", "time_µs");
    for gs in [20u32, 30, 40] {
        let n_cells = ((img.width() / gs) * (img.height() / gs)) as usize;
        const REPS: u32 = 100;
        let t0 = Instant::now();
        let mut last = Vec::new();
        let mut grid_scratch = Vec::new();
        for _ in 0..REPS {
            last = image_utilities::detect_key_points(&img, gs, &empty, &mut grid_scratch);
        }
        let us = t0.elapsed().as_micros() as f64 / REPS as f64;
        println!("  {:>10}  {:>10}  {:>10}  {:>9.1}%  {:>10.1}", gs, n_cells, last.len(), 100.0 * last.len() as f64 / n_cells as f64, us);
    }
}

// ---------------------------------------------------------------------------
// 4. Tracking convergence stats (success rate + residual)
// ---------------------------------------------------------------------------

fn report_convergence() {
    print_section("QUALITY — tracking convergence (smooth image, 2.3+1.7 px shift)");

    let img0 = smooth_image(640, 480);
    let img1 = shift_image(&img0, 2.3, 1.7);
    let pyr0 = build_image_pyramid(&img0, 4);
    let pyr1 = build_image_pyramid(&img1, 4);

    let pts: Vec<(f32, f32)> = (2..=9)
        .flat_map(|gy| (2..=9usize).map(move |gx| (gx as f32 * 70.0, gy as f32 * 55.0)))
        .collect();

    let (mut converged, mut failed) = (0usize, 0usize);
    let mut pos_errors: Vec<f32> = Vec::new();
    let mut residuals: Vec<f32> = Vec::new();

    for &(cx, cy) in &pts {
        match track_coarse_to_fine(cx, cy, &pyr0, &pyr1) {
            Some((rx, ry)) => {
                converged += 1;
                let ex = rx - (cx + 2.3);
                let ey = ry - (cy + 1.7);
                pos_errors.push((ex * ex + ey * ey).sqrt());

                // Compute final photometric residual
                let pat = Pattern52::new(&pyr0[0], cx, cy);
                if pat.valid {
                    let patten = na::SMatrix::<f32, 2, PATTERN52_SIZE>::from_column_slice(
                        &Pattern52::PATTERN_MATRIX_COL_MAJOR,
                    );
                    let mut t = na::Affine2::<f32>::identity();
                    t.matrix_mut_unchecked().m13 = rx;
                    t.matrix_mut_unchecked().m23 = ry;
                    let mut tpat = t.matrix().fixed_view::<2, 2>(0, 0) * &patten;
                    let tr = t.matrix().fixed_view::<2, 1>(0, 2);
                    for mut col in tpat.column_iter_mut() { col += tr; }
                    let center = (rx, ry);
                    if let Some(res) = pat.residual(&pyr1[0], &tpat, center) {
                        residuals.push(res.norm());
                    }
                }
            }
            None => { failed += 1; }
        }
    }

    let total = converged + failed;
    let mean_e = if pos_errors.is_empty() { f32::NAN } else { pos_errors.iter().sum::<f32>() / pos_errors.len() as f32 };
    let max_e = pos_errors.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
    let mean_r = if residuals.is_empty() { f32::NAN } else { residuals.iter().sum::<f32>() / residuals.len() as f32 };

    println!("  Total test points:    {}", total);
    println!("  Converged:            {}  ({:.1}%)", converged, 100.0 * converged as f64 / total as f64);
    println!("  Failed (track_point): {}", failed);
    println!("  Mean position error:  {:.5} px", mean_e);
    println!("  Max  position error:  {:.5} px", max_e);
    println!("  Mean final residual:  {:.4}", mean_r);
    println!("  Pass < 0.5 px:        {}/{}", pos_errors.iter().filter(|&&e| e < 0.5).count(), converged);
}

// ---------------------------------------------------------------------------
// 5. End-to-end throughput
// ---------------------------------------------------------------------------

fn report_throughput() {
    print_section("SPEED — end-to-end frame throughput");

    let img0 = checkerboard_image(640, 480, 1);
    let img1 = checkerboard_image(640, 480, 2);

    // Warm-up
    {
        let mut tracker = StereoPatchTracker::<4>::new(30, 20, 0.005);
        let mut frame = Frame::new(0, 0);
        tracker.process_frame(&img0, &img1, &mut frame);
    }

    // Warm tracking: stable feature set
    let mut tracker = StereoPatchTracker::<4>::new(30, 20, 0.005);
    let mut frame = Frame::new(0, 0);
    tracker.process_frame(&img0, &img1, &mut frame);

    const FRAMES: u32 = 50;
    let t0 = Instant::now();
    for i in 1..=FRAMES {
        let mut f = Frame::new(i as i64, i as i32);
        tracker.process_frame(&img0, &img1, &mut f);
    }
    let ms = t0.elapsed().as_secs_f64() * 1000.0 / FRAMES as f64;
    println!("  {} frames × process_frame  →  {:.2} ms/frame  →  {:.1} fps", FRAMES, ms, 1000.0 / ms);

    const REPS: u32 = 200;
    let t1 = Instant::now();
    for _ in 0..REPS { let _ = build_image_pyramid(&img0, 4); }
    let pyramid_us = t1.elapsed().as_micros() as f64 / REPS as f64;
    println!("  build_image_pyramid (L=4, 640×480)  →  {:.1} µs", pyramid_us);

    // Measure individual steps within pyramid building
    {
        let t_clone = Instant::now();
        for _ in 0..REPS { let _ = img0.clone(); }
        let clone_us = t_clone.elapsed().as_nanos() as f64 / REPS as f64 / 1000.0;

        let pyr = build_image_pyramid(&img0, 4);
        let t_l1 = Instant::now();
        for _ in 0..REPS { let _ = halve_image(&pyr[0]); }
        let l1_us = t_l1.elapsed().as_nanos() as f64 / REPS as f64 / 1000.0;
        let t_l2 = Instant::now();
        for _ in 0..REPS { let _ = halve_image(&pyr[1]); }
        let l2_us = t_l2.elapsed().as_nanos() as f64 / REPS as f64 / 1000.0;
        let t_l3 = Instant::now();
        for _ in 0..REPS { let _ = halve_image(&pyr[2]); }
        let l3_us = t_l3.elapsed().as_nanos() as f64 / REPS as f64 / 1000.0;
        println!("    clone L0 (307 KB)   →  {:.1} µs", clone_us);
        println!("    halve L0→L1         →  {:.1} µs", l1_us);
        println!("    halve L1→L2         →  {:.1} µs", l2_us);
        println!("    halve L2→L3         →  {:.1} µs", l3_us);
        println!("    sum                 →  {:.1} µs  (vs {:.1} total)", clone_us + l1_us + l2_us + l3_us, pyramid_us);
    }

    let smooth = smooth_image(640, 480);
    let t2 = Instant::now();
    for _ in 0..REPS { let _ = Pattern52::new(&smooth, 320.0, 240.0); }
    let p52_ns = t2.elapsed().as_nanos() as f64 / REPS as f64;
    println!("  Pattern52::new                      →  {:.1} ns", p52_ns);

    // Model how much of process_frame is Pattern52 construction.
    // After cold start the tracker has ~N features. Report predicted cost.
    {
        let mut tracker2 = StereoPatchTracker::<4>::new(30, 20, 0.005);
        let mut frame2 = Frame::new(0, 0);
        tracker2.process_frame(&img0, &img1, &mut frame2);
        let n = frame2.left_features().len();
        // Forward pass: 4 levels × N features. Backward: same.
        // Parallelized over 8 cores, two cameras via rayon::join.
        let n_cores = rayon::current_num_threads();
        let p52_per_frame_us = (n as f64 * 4.0 * 2.0 * 2.0 * p52_ns) / (n_cores as f64 * 1000.0);
        println!("  N features after cold start         →  {} (both cams)", n);
        println!("  predicted Pattern52::new cost       →  {:.0} µs  ({} cores, F+B, cam0+cam1)", p52_per_frame_us, n_cores);
        println!("  accounted (pyramid×2 + P52 model)   →  {:.0} µs  of {:.0} µs total", pyramid_us + p52_per_frame_us, ms * 1000.0);
    }
}

// ---------------------------------------------------------------------------
// 6. Real-data accuracy — TUM-VI dataset
// ---------------------------------------------------------------------------

/// Load a grayscale image from `<dataset>/mav0/cam{id}/data/<filename>`.
fn load_tum_image(dataset: &str, cam_id: u32, filename: &str) -> Option<GrayImage> {
    let cam = if cam_id == 0 { "cam0" } else { "cam1" };
    let path = Path::new(dataset).join("mav0").join(cam).join("data").join(filename);
    ImageReader::open(&path).ok()?.decode().ok().map(|img| img.to_luma8())
}

fn report_real_data(dataset: &str, max_frames: usize) {
    print_section("REAL DATA — TUM-VI tracking metrics");

    // Load sorted image timestamps from cam0
    let cam0_dir = Path::new(dataset).join("mav0/cam0/data");
    if !cam0_dir.exists() {
        println!("  ERROR: {} not found", cam0_dir.display());
        return;
    }

    let mut timestamps: Vec<(i64, String)> = std::fs::read_dir(&cam0_dir)
        .expect("read cam0/data")
        .filter_map(|e| e.ok())
        .filter(|e| e.path().extension().map_or(false, |x| x == "png"))
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().to_string();
            let stem = name.strip_suffix(".png")?;
            let ts: i64 = stem.parse().ok()?;
            Some((ts, name))
        })
        .collect();
    timestamps.sort_by_key(|(ts, _)| *ts);
    timestamps.truncate(max_frames);

    let n_total = timestamps.len();
    println!("  Dataset:    {}", dataset);
    println!("  Frames:     {} (of {} available, capped at {})", n_total, n_total, max_frames);

    let mut tracker = StereoPatchTracker::<4>::new(30, 10, 0.01);

    // Per-frame stats
    let mut features_left: Vec<usize> = Vec::with_capacity(n_total);
    let mut features_right: Vec<usize> = Vec::with_capacity(n_total);
    let mut tracked_counts: Vec<usize> = Vec::with_capacity(n_total);   // age > 0
    let mut new_counts: Vec<usize> = Vec::with_capacity(n_total);       // age == 0
    let mut frame_times_ms: Vec<f64> = Vec::with_capacity(n_total);
    let mut load_failures = 0usize;

    for (i, (ts, filename)) in timestamps.iter().enumerate() {
        let img0 = match load_tum_image(dataset, 0, filename) {
            Some(img) => img,
            None => { load_failures += 1; continue; }
        };
        let img1 = match load_tum_image(dataset, 1, filename) {
            Some(img) => img,
            None => { load_failures += 1; continue; }
        };

        let t0 = Instant::now();
        let mut frame = Frame::new(*ts, i as i32);
        tracker.process_frame(&img0, &img1, &mut frame);
        frame_times_ms.push(t0.elapsed().as_secs_f64() * 1000.0);

        let left = frame.left_features();
        let right = frame.right_features();
        let n_tracked = left.iter().filter(|f| f.age > 0).count();
        let n_new = left.iter().filter(|f| f.age == 0).count();

        features_left.push(left.len());
        features_right.push(right.len());
        tracked_counts.push(n_tracked);
        new_counts.push(n_new);
    }

    let n = frame_times_ms.len();
    if n == 0 {
        println!("  ERROR: no frames processed ({} load failures)", load_failures);
        return;
    }

    let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
    let mean_u = |v: &[usize]| v.iter().sum::<usize>() as f64 / v.len() as f64;
    let min_u = |v: &[usize]| *v.iter().min().unwrap_or(&0);
    let max_u = |v: &[usize]| *v.iter().max().unwrap_or(&0);

    let mean_time = mean(&frame_times_ms);
    let fps = 1000.0 / mean_time;

    // Tracking rate: for frames after the first, what fraction of left features were tracked (age > 0)?
    let tracking_rate = if tracked_counts.len() > 1 {
        let total_features: usize = features_left[1..].iter().sum();
        let total_tracked: usize = tracked_counts[1..].iter().sum();
        if total_features > 0 { 100.0 * total_tracked as f64 / total_features as f64 } else { 0.0 }
    } else { 0.0 };

    println!();
    println!("  {:>35}  {:>8}  {:>6}  {:>6}", "metric", "mean", "min", "max");
    println!("  {:>35}  {:>8.1}  {:>6}  {:>6}", "left features/frame",  mean_u(&features_left),  min_u(&features_left),  max_u(&features_left));
    println!("  {:>35}  {:>8.1}  {:>6}  {:>6}", "right features/frame", mean_u(&features_right), min_u(&features_right), max_u(&features_right));
    println!("  {:>35}  {:>8.1}  {:>6}  {:>6}", "tracked (age>0)/frame", mean_u(&tracked_counts), min_u(&tracked_counts), max_u(&tracked_counts));
    println!("  {:>35}  {:>8.1}  {:>6}  {:>6}", "new detections/frame",  mean_u(&new_counts),     min_u(&new_counts),     max_u(&new_counts));
    println!("  {:>35}  {:>8.2}  {:>6}  {:>6}", "process_frame (ms)",
             mean_time,
             frame_times_ms.iter().cloned().fold(f64::INFINITY, f64::min).round() as usize,
             frame_times_ms.iter().cloned().fold(f64::NEG_INFINITY, f64::max).round() as usize);
    println!();
    println!("  Throughput:    {:.1} fps", fps);
    println!("  Tracking rate: {:.1}%  (tracked / total left features, frames 2+)", tracking_rate);
    if load_failures > 0 {
        println!("  Load failures: {}", load_failures);
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

fn main() {
    let args = Args::parse();

    println!("RS-VIO tracker report");
    println!("  Pattern52 size: {}", PATTERN52_SIZE);

    report_memory();
    report_detection_quality();
    report_throughput();
    report_accuracy();
    report_convergence();

    if let Some(dataset) = &args.dataset {
        report_real_data(dataset, args.max_frames);
    } else {
        println!();
        println!("  (no --dataset provided; skipping real-data section)");
        println!("  Run with --dataset <path/to/dataset-corridor1_512_16> for TUM-VI metrics.");
    }

    println!();
    println!("Done.");
}
