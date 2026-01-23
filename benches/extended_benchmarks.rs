use criterion::{black_box, criterion_group, criterion_main, Criterion};
use rs_vio::estimator::{PipelineMetrics};
use rs_vio::datasets::config::FeatureDetectionConfig;

// ============================================================================
// METRICS PERFORMANCE: Measuring observability cost
// ============================================================================

fn bench_metrics_collection(c: &mut Criterion) {
    let metrics = PipelineMetrics::new();
    
    c.bench_function("metrics_record_detection", |b| {
        b.iter(|| {
            metrics.record_detection(black_box(150), black_box(false));
        });
    });
    
    c.bench_function("metrics_record_optimization", |b| {
        b.iter(|| {
            metrics.record_optimization(black_box(500), black_box(false));
        });
    });
    
    c.bench_function("metrics_set_queue_depth", |b| {
        b.iter(|| {
            metrics.set_queue_depth(black_box(3));
        });
    });
    
    c.bench_function("metrics_record_recovered_error", |b| {
        b.iter(|| {
            metrics.record_recovered_error();
        });
    });
    
    c.bench_function("metrics_summary_generation", |b| {
        b.iter(|| {
            let _ = metrics.summary();
        });
    });
}

// ============================================================================
// QUEUE MONITORING: Tracking backpressure
// ============================================================================

fn bench_queue_tracking(c: &mut Criterion) {
    let metrics = PipelineMetrics::new();
    
    c.bench_function("queue_depth_read", |b| {
        b.iter(|| {
            let depth = metrics.queue_depth();
            black_box(depth);
        });
    });
    
    c.bench_function("max_queue_depth_tracking", |b| {
        let mut depth = 0;
        b.iter(|| {
            depth = (depth + 1) % 20;
            metrics.set_queue_depth(depth);
            black_box(metrics.max_queue_depth());
        });
    });
}

// ============================================================================
// ERROR RECOVERY OVERHEAD: Measuring resilience cost
// ============================================================================

fn bench_error_tracking(c: &mut Criterion) {
    let metrics = PipelineMetrics::new();
    
    c.bench_function("detection_error_tracking", |b| {
        b.iter(|| {
            metrics.record_detection(black_box(75), black_box(true));  // error=true
        });
    });
    
    c.bench_function("optimization_error_tracking", |b| {
        b.iter(|| {
            metrics.record_optimization(black_box(250), black_box(true));  // error=true
        });
    });
    
    c.bench_function("error_recovery_recording", |b| {
        b.iter(|| {
            metrics.record_recovered_error();
        });
    });
    
    c.bench_function("error_rate_calculation", |b| {
        b.iter(|| {
            let rate = metrics.error_rate();
            black_box(rate);
        });
    });
}

// ============================================================================
// THROUGHPUT METRICS: Frame processing rate
// ============================================================================

fn bench_throughput_calculation(c: &mut Criterion) {
    let metrics = PipelineMetrics::new();
    let start = std::time::Instant::now();
    
    // Simulate 100 frame processing
    for i in 0..100 {
        metrics.record_detection(15 + (i % 20) as u64, false);
    }
    
    c.bench_function("throughput_fps_calculation", |b| {
        b.iter(|| {
            let fps = metrics.throughput_fps(start.elapsed());
            black_box(fps);
        });
    });
}

// ============================================================================
// STAGE METRICS: Per-stage performance data
// ============================================================================

fn bench_stage_metrics(c: &mut Criterion) {
    let metrics = PipelineMetrics::new();
    
    // Record 50 detection and optimization operations
    for i in 0..50 {
        metrics.record_detection(20 + (i % 10) as u64, i % 10 == 0);
        metrics.record_optimization(100 + (i % 50) as u64, i % 15 == 0);
    }
    
    c.bench_function("detection_stage_metrics_snapshot", |b| {
        b.iter(|| {
            let _ = black_box(metrics.detection_metrics());
        });
    });
    
    c.bench_function("optimization_stage_metrics_snapshot", |b| {
        b.iter(|| {
            let _ = black_box(metrics.optimization_metrics());
        });
    });
}

// ============================================================================
// FEATURE DETECTION CONFIG: Baseline performance reference
// ============================================================================

fn bench_detector_config(c: &mut Criterion) {
    c.bench_function("feature_detection_config_creation", |b| {
        b.iter(|| {
            let _config = FeatureDetectionConfig::default();
        });
    });
    
    c.bench_function("feature_detection_config_clone", |b| {
        let config = FeatureDetectionConfig::default();
        b.iter(|| {
            let _ = black_box(config.clone());
        });
    });
}

// ============================================================================
// METRICS SCALABILITY: Large history buffer
// ============================================================================

fn bench_metrics_scalability(c: &mut Criterion) {
    c.bench_function("metrics_1000frame_history", |b| {
        b.iter_with_setup(
            || {
                let metrics = PipelineMetrics::new();
                for i in 0..1000 {
                    metrics.record_detection(50 + (i % 30) as u64, i % 50 == 0);
                }
                metrics
            },
            |metrics| {
                black_box(metrics.frame_history());
            },
        );
    });
    
    c.bench_function("metrics_reset_operation", |b| {
        let metrics = PipelineMetrics::new();
        for i in 0..100 {
            metrics.record_detection(50 + (i % 30) as u64, i % 50 == 0);
        }
        b.iter(|| {
            metrics.reset();
        });
    });
}

criterion_group!(
    benches,
    bench_metrics_collection,
    bench_queue_tracking,
    bench_error_tracking,
    bench_throughput_calculation,
    bench_stage_metrics,
    bench_detector_config,
    bench_metrics_scalability,
);

criterion_main!(benches);
