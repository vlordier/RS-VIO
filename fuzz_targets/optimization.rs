#![no_main]
#![allow(unused_crate_dependencies)]

use libfuzzer_sys::fuzz_target;
use rs_vio::optimization::{Optimizer, OptimizerConfig};
use rs_vio::types::Pose;

fuzz_target!(|data: (Vec<f64>, Vec<f64>, Vec<i64>)| {
    let (poses, residuals, iterations) = data;

    // Limit size to prevent OOM
    if poses.len() > 100 || residuals.len() > 1000 || iterations.is_empty() {
        return;
    }

    let config = OptimizerConfig {
        max_iterations: iterations[0].clamp(1, 100) as usize,
        tolerance: 1e-10,
        verbose: false,
    };

    let mut optimizer = Optimizer::new(config);

    // Create some test poses
    let test_poses: Vec<Pose> = poses
        .chunks(7)
        .take(10)
        .map(|chunk| Pose {
            x: chunk.get(0).copied().unwrap_or(0.0),
            y: chunk.get(1).copied().unwrap_or(0.0),
            z: chunk.get(2).copied().unwrap_or(0.0),
            qx: chunk.get(3).copied().unwrap_or(0.0),
            qy: chunk.get(4).copied().unwrap_or(0.0),
            qz: chunk.get(5).copied().unwrap_or(0.0),
            qw: chunk.get(6).copied().unwrap_or(1.0),
        })
        .collect();

    // Fuzz optimization
    if !test_poses.is_empty() && !residuals.is_empty() {
        let _ = optimizer.optimize(&test_poses, &residuals);
    }
});
