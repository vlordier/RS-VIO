//! Real-world TUM-VI Pipeline Benchmark
//! 
//! Measures actual performance on TUM Visual-Inertial dataset images.
//! This replaces synthetic benchmarks with real-world validation.

use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use rs_vio::datasets::tum_vi::{TumViSequence, load_all_sequences};
use std::path::Path;
use std::time::Duration;

fn benchmark_tum_vi_loading(c: &mut Criterion) {
    let dataset_dir = std::env::var("TUM_VI_DIR")
        .unwrap_or_else(|_| "./data/tum_vi".to_string());
    
    if !Path::new(&dataset_dir).exists() {
        eprintln!("Warning: TUM-VI dataset not found at {}", dataset_dir);
        eprintln!("Download from: https://vision.in.tum.de/data/datasets/visual-inertial-dataset");
        eprintln!("Extract room sequences to: {}", dataset_dir);
        eprintln!("Or set TUM_VI_DIR environment variable");
        return;
    }
    
    let mut group = c.benchmark_group("tum_vi_loading");
    group.sample_size(10); // Reduce samples for large dataset loading
    
    group.bench_function("load_all_sequences", |b| {
        b.iter(|| {
            let sequences = load_all_sequences(&dataset_dir).unwrap();
            black_box(sequences)
        });
    });
    
    group.finish();
}

fn benchmark_sequence_parsing(c: &mut Criterion) {
    let dataset_dir = std::env::var("TUM_VI_DIR")
        .unwrap_or_else(|_| "./data/tum_vi".to_string());
    
    if !Path::new(&dataset_dir).exists() {
        return;
    }
    
    let sequences = match load_all_sequences(&dataset_dir) {
        Ok(seqs) if !seqs.is_empty() => seqs,
        _ => return,
    };
    
    let mut group = c.benchmark_group("sequence_parsing");
    
    for seq in &sequences {
        group.bench_with_input(
            BenchmarkId::from_parameter(&seq.name),
            seq,
            |b, seq| {
                b.iter(|| {
                    let num_frames = black_box(seq.num_frames());
                    let frame_rate = black_box(seq.frame_rate());
                    let imu_count = black_box(seq.imu_data.len());
                    (num_frames, frame_rate, imu_count)
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_ground_truth_lookup(c: &mut Criterion) {
    let dataset_dir = std::env::var("TUM_VI_DIR")
        .unwrap_or_else(|_| "./data/tum_vi".to_string());
    
    if !Path::new(&dataset_dir).exists() {
        return;
    }
    
    let sequences = match load_all_sequences(&dataset_dir) {
        Ok(seqs) if !seqs.is_empty() => seqs,
        _ => return,
    };
    
    let mut group = c.benchmark_group("ground_truth_lookup");
    
    for seq in &sequences {
        if seq.ground_truth.is_empty() {
            continue;
        }
        
        group.bench_with_input(
            BenchmarkId::from_parameter(&seq.name),
            seq,
            |b, seq| {
                b.iter(|| {
                    // Binary search for nearest ground truth pose
                    let target_timestamp = seq.cam0_timestamps[seq.num_frames() / 2];
                    let gt_pose = seq.ground_truth.iter()
                        .min_by_key(|gt| {
                            (gt.timestamp_ns as i64 - target_timestamp as i64).abs()
                        });
                    black_box(gt_pose)
                });
            },
        );
    }
    
    group.finish();
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
