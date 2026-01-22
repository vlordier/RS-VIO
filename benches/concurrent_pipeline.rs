use criterion::{black_box, criterion_group, criterion_main, Criterion};
use rs_vio::estimator::{ConcurrentConfig, ConcurrentFrameProcessor, Frame};
use std::sync::Arc;

async fn run_concurrent(n: usize, mut config: ConcurrentConfig) {
    // Force small simulated work so the executor can exercise concurrency.
    config.simulated_work_ms = Some(config.simulated_work_ms.unwrap_or(2));
    let (mut processor, mut handle) = ConcurrentFrameProcessor::new(config.clone());
    handle.start();

    for i in 0..n {
        let frame = Arc::new(Frame::new(i as i64, i as i32));
        processor
            .process_frame(frame, None, i as i64)
            .await
            .expect("process_frame should succeed");
    }

    for _ in 0..n {
        let result = processor
            .recv_result()
            .await
            .expect("result expected from concurrent pipeline");
        black_box(result.frame_id);
    }

    handle.shutdown().await;
}

async fn run_sequential(n: usize, config: ConcurrentConfig) {
    let simulated = config.simulated_work_ms.unwrap_or(2);
    for i in 0..n {
        let frame = Frame::new(i as i64, i as i32);
        black_box(frame);
        if simulated > 0 {
            tokio::time::sleep(std::time::Duration::from_millis(simulated)).await;
        }
    }
}

fn bench_concurrent_pipeline(c: &mut Criterion) {
    let mut group = c.benchmark_group("concurrent_pipeline");
    
    group.sample_size(10);
    group.measurement_time(std::time::Duration::from_secs(10));

    let base_config = ConcurrentConfig {
        pipeline_depth: 8,
        feature_workers: 2,
        optimization_workers: 2,
        simulated_work_ms: Some(2),
        simulated_jitter_ms: Some(1),
        ..Default::default()
    };

    group.bench_function("concurrent_32_frames", |b| {
        b.iter(|| {
            let config = base_config.clone();
            let rt = tokio::runtime::Runtime::new().expect("tokio runtime");
            rt.block_on(async { run_concurrent(32, config).await })
        });
    });

    group.bench_function("sequential_32_frames", |b| {
        b.iter(|| {
            let config = base_config.clone();
            let rt = tokio::runtime::Runtime::new().expect("tokio runtime");
            rt.block_on(async { run_sequential(32, config).await })
        });
    });

    group.finish();
}

criterion_group!(benches, bench_concurrent_pipeline);
criterion_main!(benches);
