use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use rs_vio::feature_tracker::{PatchTracker, StereoPatchTracker};
use rs_vio::datasets::config::FeatureDetectionConfig;
use rs_vio::estimator::Frame;
use image::{GrayImage, Luma};

/// Create a realistic test image with features
fn create_test_image(width: u32, height: u32) -> GrayImage {
    GrayImage::from_fn(width, height, |x, y| {
        // Create a pattern with multiple scales of features
        let value = ((x as f32 / 10.0).sin() * 50.0 + (y as f32 / 10.0).cos() * 50.0 + 128.0) as u8;
        Luma([value])
    })
}

fn bench_mono_feature_tracking(c: &mut Criterion) {
    let config = FeatureDetectionConfig::default();
    let mut tracker = PatchTracker::<3>::from_config(&config);
    
    let img1 = create_test_image(640, 480);
    let img2 = create_test_image(640, 480);
    
    // Warmup - process first frame to initialize
    tracker.process_frame(&img1);
    
    c.bench_function("mono_feature_tracking_640x480", |b| {
        b.iter(|| {
            tracker.process_frame(black_box(&img2));
        })
    });
}

fn bench_stereo_feature_tracking(c: &mut Criterion) {
    let config = FeatureDetectionConfig::default();
    let mut tracker = StereoPatchTracker::<3>::from_config(&config);
    
    let img_left = create_test_image(640, 480);
    let img_right = create_test_image(640, 480);
    
    // Create a frame for processing
    let mut frame = Frame::new(0, 0);
    
    // Warmup - process first frame to initialize
    tracker.process_frame(&img_left, &img_right, &mut frame);
    
    c.bench_function("stereo_feature_tracking_640x480", |b| {
        b.iter(|| {
            let mut frame = Frame::new(0, 0);
            tracker.process_frame(black_box(&img_left), black_box(&img_right), black_box(&mut frame));
        })
    });
}

fn bench_different_resolutions(c: &mut Criterion) {
    let mut group = c.benchmark_group("feature_tracking_resolution");
    
    for &(width, height) in &[(320, 240), (640, 480), (1280, 720)] {
        let config = FeatureDetectionConfig::default();
        let mut tracker = PatchTracker::<3>::from_config(&config);
        
        let img1 = create_test_image(width, height);
        let img2 = create_test_image(width, height);
        
        tracker.process_frame(&img1);
        
        group.bench_with_input(
            BenchmarkId::from_parameter(format!("{}x{}", width, height)),
            &(width, height),
            |b, _| {
                b.iter(|| {
                    tracker.process_frame(black_box(&img2));
                })
            },
        );
    }
    
    group.finish();
}

criterion_group!(
    benches,
    bench_mono_feature_tracking,
    bench_stereo_feature_tracking,
    bench_different_resolutions
);
criterion_main!(benches);
