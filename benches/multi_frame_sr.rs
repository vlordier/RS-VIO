//! Benchmark for frame stabilization and track-first detection
//!
//! Compares performance with/without multi-frame SR techniques

use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use image::{GrayImage, Luma};
use nalgebra as na;
use rs_vio::feature_tracker::{FrameStabilizer, StabilizerConfig, TrackFirstDetector, TrackFirstConfig};

fn create_test_image(width: u32, height: u32) -> GrayImage {
    let mut img = GrayImage::new(width, height);
    for y in 0..height {
        for x in 0..width {
            let value = ((x + y) % 256) as u8;
            img.put_pixel(x, y, Luma([value]));
        }
    }
    img
}

fn bench_frame_stabilizer(c: &mut Criterion) {
    let mut group = c.benchmark_group("frame_stabilizer");
    
    // Test different image sizes
    for (width, height) in &[(320, 240), (640, 480), (752, 480)] {
        let image = create_test_image(*width, *height);
        let rotation = na::Rotation3::identity();
        let timestamp = 0.0;
        
        // Test different buffer sizes
        for buffer_size in &[3, 5, 7] {
            let config = StabilizerConfig {
                enabled: true,
                buffer_size: *buffer_size,
                accumulation_weight: 0.7,
                min_rotation_threshold: 0.001,
            };
            
            let mut stabilizer = FrameStabilizer::new(
                config,
                458.0, // fx
                457.0, // fy
                *width as f32 / 2.0, // cx
                *height as f32 / 2.0, // cy
            );
            
            group.bench_with_input(
                BenchmarkId::new(
                    format!("{}x{}_buf{}", width, height, buffer_size),
                    buffer_size
                ),
                buffer_size,
                |b, _| {
                    b.iter(|| {
                        black_box(stabilizer.process_frame(&image, rotation, timestamp))
                    });
                },
            );
        }
    }
    
    group.finish();
}

fn bench_stabilizer_disabled_vs_enabled(c: &mut Criterion) {
    let mut group = c.benchmark_group("stabilizer_comparison");
    
    let width = 640;
    let height = 480;
    let image = create_test_image(width, height);
    let rotation = na::Rotation3::identity();
    let timestamp = 0.0;
    
    // Disabled
    {
        let config = StabilizerConfig {
            enabled: false,
            buffer_size: 5,
            accumulation_weight: 0.7,
            min_rotation_threshold: 0.001,
        };
        
        let mut stabilizer = FrameStabilizer::new(
            config,
            458.0,
            457.0,
            width as f32 / 2.0,
            height as f32 / 2.0,
        );
        
        group.bench_function("disabled", |b| {
            b.iter(|| {
                black_box(stabilizer.process_frame(&image, rotation, timestamp))
            });
        });
    }
    
    // Enabled
    {
        let config = StabilizerConfig {
            enabled: true,
            buffer_size: 5,
            accumulation_weight: 0.7,
            min_rotation_threshold: 0.001,
        };
        
        let mut stabilizer = FrameStabilizer::new(
            config,
            458.0,
            457.0,
            width as f32 / 2.0,
            height as f32 / 2.0,
        );
        
        group.bench_function("enabled", |b| {
            b.iter(|| {
                black_box(stabilizer.process_frame(&image, rotation, timestamp))
            });
        });
    }
    
    group.finish();
}

fn bench_track_first_detector(c: &mut Criterion) {
    let mut group = c.benchmark_group("track_first_detector");
    
    let width = 640;
    let height = 480;
    let image = create_test_image(width, height);
    
    // Generate some tracked points
    let mut tracked_points = Vec::new();
    for i in 0..150 {
        let x = ((i * 17) % width) as f32;
        let y = ((i * 13) % height) as f32;
        tracked_points.push((i, na::Vector2::new(x, y)));
    }
    
    // Test different feature count configurations
    for max_features in &[150, 250, 350] {
        let config = TrackFirstConfig {
            min_features: 100,
            max_features: *max_features,
            grid_cell_size: 32,
            min_features_per_cell: 2,
            corner_quality_threshold: 0.01,
            min_feature_distance: 10.0,
        };
        
        let mut detector = TrackFirstDetector::new(config, width, height);
        
        group.bench_with_input(
            BenchmarkId::new("max_features", max_features),
            max_features,
            |b, _| {
                b.iter(|| {
                    black_box(detector.update_tracks(&image, &tracked_points, None))
                });
            },
        );
    }
    
    group.finish();
}

fn bench_track_first_needs_detection(c: &mut Criterion) {
    let mut group = c.benchmark_group("track_first_needs_detection");
    
    let width = 640;
    let height = 480;
    let image = create_test_image(width, height);
    
    let config = TrackFirstConfig {
        min_features: 150,
        max_features: 300,
        grid_cell_size: 32,
        min_features_per_cell: 2,
        corner_quality_threshold: 0.01,
        min_feature_distance: 10.0,
    };
    
    let mut detector = TrackFirstDetector::new(config, width, height);
    
    // Test with different numbers of existing features
    for num_features in &[50, 100, 150, 200, 250] {
        let mut tracked_points = Vec::new();
        for i in 0..*num_features {
            let x = ((i * 17) % width) as f32;
            let y = ((i * 13) % height) as f32;
            tracked_points.push((i, na::Vector2::new(x, y)));
        }
        
        detector.update_tracks(&image, &tracked_points, None);
        
        group.bench_with_input(
            BenchmarkId::new("existing_features", num_features),
            num_features,
            |b, _| {
                b.iter(|| {
                    black_box(detector.needs_new_features())
                });
            },
        );
    }
    
    group.finish();
}

criterion_group!(
    benches,
    bench_frame_stabilizer,
    bench_stabilizer_disabled_vs_enabled,
    bench_track_first_detector,
    bench_track_first_needs_detection,
);
criterion_main!(benches);
