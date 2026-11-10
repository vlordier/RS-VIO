//! Benchmarks for motion prior overhead.
//!
//! Measures the actual cost of:
//! - process_noise_scale()
//! - gate_measurement()
//! - score_prediction()
//! - Adaptive Huber threshold computation

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use nalgebra as na;
use na::{UnitQuaternion, Vector3};
use rs_vio::estimator::{MotionPrior, QuadrotorPrior, FixedWingPrior, VtolPrior, VtolMode, UgvPrior, State, PriorContext};

fn bench_process_noise_scale(c: &mut Criterion) {
    let mut group = c.benchmark_group("process_noise_scale");
    group.throughput(Throughput::Elements(1));
    
    let state = State::identity();
    let ctx = PriorContext {
        feature_count: 100,
        visual_health: 0.9,
    };
    
    // Quadrotor
    let quad = QuadrotorPrior::default();
    group.bench_function("quadrotor", |b| {
        b.iter(|| quad.process_noise_scale(black_box(&state), 0.005, black_box(&ctx)))
    });
    
    // Fixed-wing
    let fw = FixedWingPrior::default();
    group.bench_function("fixed_wing", |b| {
        b.iter(|| fw.process_noise_scale(black_box(&state), 0.005, black_box(&ctx)))
    });
    
    // UGV
    let ugv = UgvPrior::default();
    group.bench_function("ugv", |b| {
        b.iter(|| ugv.process_noise_scale(black_box(&state), 0.005, black_box(&ctx)))
    });
    
    // VTOL hover
    let vtol_hover = VtolPrior::default();
    group.bench_function("vtol_hover", |b| {
        b.iter(|| vtol_hover.process_noise_scale(black_box(&state), 0.005, black_box(&ctx)))
    });
    
    // VTOL transition
    let mut vtol_trans = VtolPrior::default();
    vtol_trans.mode = VtolMode::Transition;
    group.bench_function("vtol_transition", |b| {
        b.iter(|| vtol_trans.process_noise_scale(black_box(&state), 0.005, black_box(&ctx)))
    });
    
    group.finish();
}

fn bench_gate_measurement(c: &mut Criterion) {
    let mut group = c.benchmark_group("gate_measurement");
    group.throughput(Throughput::Elements(1));
    
    let state = State::identity();
    let vision_pos = Vector3::new(0.1, 0.0, 0.0);
    let vision_rot = UnitQuaternion::identity();
    let _ctx = PriorContext {
        feature_count: 100,
        visual_health: 0.9,
    };
    
    // Quadrotor
    let quad = QuadrotorPrior::default();
    group.bench_function("quadrotor", |b| {
        b.iter(|| quad.gate_measurement(
            black_box(&state),
            black_box(&vision_pos),
            black_box(&vision_rot),
            0.005,
            50,
            0.5,
            1.0,
        ))
    });
    
    // Fixed-wing
    let fw = FixedWingPrior::default();
    group.bench_function("fixed_wing", |b| {
        b.iter(|| fw.gate_measurement(
            black_box(&state),
            black_box(&vision_pos),
            black_box(&vision_rot),
            0.005,
            50,
            0.5,
            1.0,
        ))
    });
    
    // UGV
    let ugv = UgvPrior::default();
    group.bench_function("ugv", |b| {
        b.iter(|| ugv.gate_measurement(
            black_box(&state),
            black_box(&vision_pos),
            black_box(&vision_rot),
            0.005,
            50,
            0.5,
            1.0,
        ))
    });
    
    group.finish();
}

fn bench_score_prediction(c: &mut Criterion) {
    let mut group = c.benchmark_group("score_prediction");
    group.throughput(Throughput::Elements(1));
    
    let prev_state = State::identity();
    let predicted_state = State::identity();
    let ctx = PriorContext {
        feature_count: 100,
        visual_health: 0.9,
    };
    
    // Quadrotor
    let quad = QuadrotorPrior::default();
    group.bench_function("quadrotor", |b| {
        b.iter(|| quad.score_prediction(
            black_box(&prev_state),
            black_box(&predicted_state),
            0.005,
            black_box(&ctx),
        ))
    });
    
    // Fixed-wing
    let fw = FixedWingPrior::default();
    group.bench_function("fixed_wing", |b| {
        b.iter(|| fw.score_prediction(
            black_box(&prev_state),
            black_box(&predicted_state),
            0.005,
            black_box(&ctx),
        ))
    });
    
    // UGV
    let ugv = UgvPrior::default();
    group.bench_function("ugv", |b| {
        b.iter(|| ugv.score_prediction(
            black_box(&prev_state),
            black_box(&predicted_state),
            0.005,
            black_box(&ctx),
        ))
    });
    
    group.finish();
}

fn bench_adaptive_huber(c: &mut Criterion) {
    let mut group = c.benchmark_group("adaptive_huber");
    group.throughput(Throughput::Elements(1));
    
    let prev_state = State::identity();
    let predicted_state = State::identity();
    let ctx = PriorContext {
        feature_count: 100,
        visual_health: 0.9,
    };
    
    // Measure full adaptive Huber computation
    let quad = QuadrotorPrior::default();
    group.bench_function("quadrotor_full", |b| {
        b.iter(|| {
            let prior_score = quad.score_prediction(
                black_box(&prev_state),
                black_box(&predicted_state),
                0.005,
                black_box(&ctx),
            );
            let adaptive_huber: f64 = 2.0 * (2.0 - prior_score.score as f64).max(1.0);
            adaptive_huber
        })
    });
    
    group.finish();
}

fn bench_total_prior_overhead(c: &mut Criterion) {
    let mut group = c.benchmark_group("total_prior_overhead");
    group.throughput(Throughput::Elements(1));
    
    let state = State::identity();
    let prev_state = State::identity();
    let vision_pos = Vector3::new(0.1, 0.0, 0.0);
    let vision_rot = UnitQuaternion::identity();
    let ctx = PriorContext {
        feature_count: 100,
        visual_health: 0.9,
    };
    
    // Measure all prior operations together (simulates one BA iteration)
    let quad = QuadrotorPrior::default();
    group.bench_function("quadrotor_all_ops", |b| {
        b.iter(|| {
            // Process noise scaling (once per IMU propagation)
            let _scale = quad.process_noise_scale(black_box(&state), 0.005, black_box(&ctx));
            
            // Score prediction (once per BA iteration)
            let score = quad.score_prediction(
                black_box(&prev_state),
                black_box(&state),
                0.005,
                black_box(&ctx),
            );
            
            // Adaptive Huber (once per BA iteration)
            let _huber: f64 = 2.0 * (2.0 - score.score as f64).max(1.0);
            
            // Gate measurement (once per PnP/track_motion)
            let _gate = quad.gate_measurement(
                black_box(&state),
                black_box(&vision_pos),
                black_box(&vision_rot),
                0.005,
                50,
                0.5,
                score.score,
            );
        })
    });
    
    group.finish();
}

criterion_group!(
    benches,
    bench_process_noise_scale,
    bench_gate_measurement,
    bench_score_prediction,
    bench_adaptive_huber,
    bench_total_prior_overhead,
);
criterion_main!(benches);
