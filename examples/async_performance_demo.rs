use rs_vio::estimator::{ConcurrentConfig, ConcurrentFrameProcessor, Frame};
use std::sync::Arc;
use std::time::Instant;

#[tokio::main]
async fn main() {
    println!("🚀 Async Pipeline Performance Demo\n");

    let num_frames = 100;
    let work_ms = 5; // Simulated processing time per frame

    // Sequential baseline
    println!("📊 Sequential Processing (baseline):");
    let start = Instant::now();
    for i in 0..num_frames {
        let _frame = Frame::new(i as i64, i as i32);
        tokio::time::sleep(std::time::Duration::from_millis(work_ms)).await;
    }
    let seq_duration = start.elapsed();
    let seq_fps = num_frames as f64 / seq_duration.as_secs_f64();
    println!("   Time: {:.2}s", seq_duration.as_secs_f64());
    println!("   FPS: {:.1}", seq_fps);
    println!("   Throughput: {} frames", num_frames);

    // Concurrent pipeline
    println!("\n⚡ Concurrent Pipeline (4 stages):");
    let config = ConcurrentConfig {
        pipeline_depth: 4,
        feature_workers: 2,
        optimization_workers: 2,
        simulated_work_ms: Some(work_ms),
        ..Default::default()
    };

    let start = Instant::now();
    let (mut processor, mut handle) = ConcurrentFrameProcessor::new(config);
    handle.start();

    // Send all frames
    for i in 0..num_frames {
        let frame = Arc::new(Frame::new(i as i64, i as i32));
        processor
            .process_frame(frame, None, i as i64)
            .await
            .expect("process_frame failed");
    }

    // Receive all results
    for _ in 0..num_frames {
        let _result = processor.recv_result().await.expect("recv_result failed");
    }

    handle.shutdown().await;
    let conc_duration = start.elapsed();
    let conc_fps = num_frames as f64 / conc_duration.as_secs_f64();

    println!("   Time: {:.2}s", conc_duration.as_secs_f64());
    println!("   FPS: {:.1}", conc_fps);
    println!("   Throughput: {} frames", num_frames);

    // Performance comparison
    let speedup = seq_duration.as_secs_f64() / conc_duration.as_secs_f64();
    println!("\n📈 Performance Improvement:");
    println!("   Speedup: {:.2}x faster", speedup);
    println!(
        "   Time saved: {:.2}s ({:.1}%)",
        seq_duration.as_secs_f64() - conc_duration.as_secs_f64(),
        (1.0 - conc_duration.as_secs_f64() / seq_duration.as_secs_f64()) * 100.0
    );

    if speedup >= 2.0 {
        println!("\n✅ SUCCESS: Pipeline achieves >2x speedup through concurrent execution!");
    } else {
        println!(
            "\n⚠️  Speedup less than expected (target: 2x, actual: {:.2}x)",
            speedup
        );
    }
}
