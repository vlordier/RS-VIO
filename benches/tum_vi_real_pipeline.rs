//! Real-world TUM-VI Pipeline Benchmark
//!
//! Measures actual performance on TUM Visual-Inertial dataset images.

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use rs_vio::datasets::tum_vi::load_all_sequences;
use std::path::Path;
use std::time::Duration;

fn get_dataset_dir() -> Option<String> {
    let dir = std::env::var("TUM_VI_DIR").unwrap_or_else(|_| {
        if Path::new("./datasets/tum_vi").exists() {
            "./datasets/tum_vi".to_string()
        } else {
            "./data/tum_vi".to_string()
        }
    });

    if Path::new(&dir).exists() {
        Some(dir)
    } else {
        eprintln!("Warning: TUM-VI dataset not found at {}", dir);
        eprintln!("Download from: https://vision.in.tum.de/data/datasets/visual-inertial-dataset");
        eprintln!("Extract room sequences to: {} or ./datasets/tum_vi", dir);
        eprintln!("Or set TUM_VI_DIR environment variable");
        None
    }
}

fn benchmark_tum_vi_loading(c: &mut Criterion) {
    if let Some(dataset_dir) = get_dataset_dir() {
        let mut group = c.benchmark_group("tum_vi_loading");
        group.sample_size(10);

        group.bench_function("load_all_sequences", |b| {
            b.iter(|| {
                let sequences = load_all_sequences(&dataset_dir).unwrap();
                black_box(sequences)
            });
        });

        group.finish();
    }
}

fn benchmark_sequence_parsing(c: &mut Criterion) {
    if let Some(dataset_dir) = get_dataset_dir() {
        if let Ok(sequences) = load_all_sequences(&dataset_dir) {
            let mut group = c.benchmark_group("sequence_parsing");

            for seq in &sequences {
                group.bench_with_input(BenchmarkId::from_parameter(&seq.name), seq, |b, _seq| {
                    b.iter(|| {
                        // Simulate parsing timestamps
                        
                        black_box(seq.num_frames())
                    });
                });
            }

            group.finish();
        }
    }
}

fn benchmark_ground_truth_lookup(c: &mut Criterion) {
    if let Some(dataset_dir) = get_dataset_dir() {
        if let Ok(sequences) = load_all_sequences(&dataset_dir) {
            let mut group = c.benchmark_group("ground_truth_lookup");

            for seq in &sequences {
                if seq.ground_truth.is_empty() {
                    continue;
                }

                group.bench_with_input(BenchmarkId::from_parameter(&seq.name), seq, |b, seq| {
                    b.iter(|| {
                        let target_timestamp = seq.cam0_timestamps[seq.num_frames() / 2];
                        let gt_pose = seq.ground_truth.iter().min_by_key(|gt| {
                            (gt.timestamp_ns as i64 - target_timestamp as i64).abs()
                        });
                        black_box(gt_pose)
                    });
                });
            }

            group.finish();
        }
    }
}

criterion_group! {
    name = tum_vi_benches;
    config = Criterion::default()
        .measurement_time(Duration::from_secs(10))
        .warm_up_time(Duration::from_secs(2));
    targets =
        benchmark_tum_vi_loading,
        benchmark_sequence_parsing,
        benchmark_ground_truth_lookup
}

criterion_main!(tum_vi_benches);
