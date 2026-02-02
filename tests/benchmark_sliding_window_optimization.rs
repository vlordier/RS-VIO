use rs_vio::estimator::{SlidingWindow, Frame};
use rs_vio::feature_tracker::Feature;
use std::time::Instant;

#[test]
fn bench_optimization_build() {
    let mut sw = SlidingWindow::new(8);
    
    // Add 8 frames
    for i in 0..8 {
        let mut frame = Frame::new((i * 100) as i64, i as i32);
        frame.is_keyframe = true;
        
        // Add 2000 features per frame to stress test
        for j in 0..2000 {
            // Feature ID: j, present in all frames
            frame.left_features.push(Feature::new(j, [10.0, 10.0]));
            frame.right_features.push(Feature::new(j, [20.0, 10.0]));
        }
        sw.add_frame(frame);
    }

    // Warmup
    let _ = sw.build_optimization_problem();

    let start = Instant::now();
    let (problem, initials) = sw.build_optimization_problem();
    let duration = start.elapsed();
    println!("BENCHMARK: SlidingWindow::build_optimization_problem took: {:?}", duration);
    println!("  Variables: {}, Residuals: {}", initials.len(), problem.num_residual_blocks());
}
