use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use rs_vio::feature_tracker::patch::{Pattern52, PATTERN52_SIZE};
use nalgebra as na;

fn bench_residuals_scalar(c: &mut Criterion) {
    let mut pattern = Pattern52::default();
    for i in 0..PATTERN52_SIZE {
        pattern.data[i] = (i as f32) * 0.1;
    }
    
    let mut current_data = [0.0f32; PATTERN52_SIZE];
    for i in 0..PATTERN52_SIZE {
        current_data[i] = (i as f32) * 0.12 + 1.0;
    }

    c.bench_function("residuals_scalar", |b| {
        b.iter(|| {
            let mut residuals = na::SVector::<f32, PATTERN52_SIZE>::zeros();
            for i in 0..PATTERN52_SIZE {
                if pattern.data[i] >= 0.0 && current_data[i] >= 0.0 {
                    residuals[i] = current_data[i] - pattern.data[i];
                }
            }
            black_box(residuals)
        })
    });
}

fn bench_residuals_simd(c: &mut Criterion) {
    // SIMD residuals now require additional normalization parameters
    // Benchmark the scalar path for now (SIMD is modular and optional)
    let mut pattern = Pattern52::default();
    for i in 0..PATTERN52_SIZE {
        pattern.data[i] = (i as f32) * 0.1;
    }
    
    let mut current_data = [0.0f32; PATTERN52_SIZE];
    for i in 0..PATTERN52_SIZE {
        current_data[i] = (i as f32) * 0.12 + 1.0;
    }

    c.bench_function("residuals_scalar", |b| {
        b.iter(|| {
            let mut residuals = [0.0f32; PATTERN52_SIZE];
            for i in 0..PATTERN52_SIZE {
                if pattern.data[i] >= 0.0 && current_data[i] >= 0.0 {
                    residuals[i] = current_data[i] - pattern.data[i];
                }
            }
            black_box(residuals)
        })
    });
}

fn bench_stats_scalar(c: &mut Criterion) {
    let mut data = [0.0f32; PATTERN52_SIZE];
    for i in 0..PATTERN52_SIZE {
        data[i] = (i as f32) * 0.5 + 10.0;
    }

    c.bench_function("stats_scalar", |b| {
        b.iter(|| {
            let mut sum = 0.0f32;
            let mut count = 0;
            for &val in data.iter() {
                if val >= 0.0 {
                    sum += val;
                    count += 1;
                }
            }
            let mean = sum / count as f32;
            
            let mut var_sum = 0.0f32;
            for &val in data.iter() {
                if val >= 0.0 {
                    let diff = val - mean;
                    var_sum += diff * diff;
                }
            }
            let std_dev = (var_sum / count as f32).sqrt();
            black_box((mean, std_dev))
        })
    });
}

fn bench_stats_simd(c: &mut Criterion) {
    // Stats SIMD not exposed in public API currently
    // Benchmark scalar implementation
    let mut data = [0.0f32; PATTERN52_SIZE];
    for i in 0..PATTERN52_SIZE {
        data[i] = (i as f32) * 0.5 + 10.0;
    }

    c.bench_function("stats_scalar_alt", |b| {
        b.iter(|| {
            let mut sum = 0.0f32;
            let mut count = 0;
            for &val in data.iter() {
                if val >= 0.0 {
                    sum += val;
                    count += 1;
                }
            }
            let mean = sum / count as f32;
            black_box(mean)
        })
    });
}

fn bench_patch_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("patch_operations");
    
    for size in [10, 20, 50, 100].iter() {
        group.bench_with_input(BenchmarkId::new("residuals_scalar", size), size, |b, &_size| {
            let mut pattern = Pattern52::default();
            for i in 0..PATTERN52_SIZE {
                pattern.data[i] = (i as f32) * 0.1;
            }
            let mut current_data = [0.0f32; PATTERN52_SIZE];
            for i in 0..PATTERN52_SIZE {
                current_data[i] = (i as f32) * 0.12 + 1.0;
            }
            
            b.iter(|| {
                let mut residuals = na::SVector::<f32, PATTERN52_SIZE>::zeros();
                for i in 0..PATTERN52_SIZE {
                    if pattern.data[i] >= 0.0 && current_data[i] >= 0.0 {
                        residuals[i] = current_data[i] - pattern.data[i];
                    }
                }
                black_box(residuals)
            })
        });
    }
    
    group.finish();
}

criterion_group!(benches, 
    bench_residuals_scalar, 
    bench_residuals_simd,
    bench_stats_scalar,
    bench_stats_simd,
    bench_patch_operations
);
criterion_main!(benches);
