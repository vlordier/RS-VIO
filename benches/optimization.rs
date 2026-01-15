//! # Optimization Benchmarks
//!
//! Performance benchmarks for the state optimization and sliding window subsystem.
//!
//! ## Benchmarks
//!
//! - **State Management** - Core data structure operations
//! - **Sliding Window Operations** - Keyframe management (3, 5, 10 frame windows)
//! - **State Serialization** - Copying and cloning (50-500 map points)
//! - **Observation Management** - Feature observation recording
//! - **Pose Recovery** - Fast pose lookup
//!
//! ## Expected Results
//!
//! add_frame_pose < 0.1ms | add_map_point < 0.1ms | get_poses < 0.1ms | serialize < 1.0ms
//!
//! See [BENCHMARKING.md](../BENCHMARKING.md#optimization-benchmarks) for detailed analysis.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::missing_const_for_fn
)]

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use nalgebra as na;
use rs_vio::estimator::sliding_window::SlidingWindow;
use rs_vio::estimator::state::State;
use rs_vio::types::*;
use std::collections::HashMap;

/// Create a synthetic state with camera poses and map points
///
/// Generates a realistic state with specified number of camera poses arranged
/// along a trajectory and map points distributed in 3D space.
fn create_test_state(num_frames: usize, num_points: usize) -> State {
    let mut poses = Vec::new();

    // Create camera poses along a trajectory
    for i in 0..num_frames {
        let angle = (i as f64) * 0.1;
        let rotation = na::Rotation3::from_euler_angles(angle * 0.5, angle, angle * 0.3);
        let translation = na::Vector3::new(i as f64 * 0.1, angle.sin() * 0.05, angle.cos() * 0.05);
        let pose = na::Isometry3::from_parts(na::Translation3::from(translation), rotation);
        poses.push(pose);
    }

    let mut state = State::new();

    // Add frame poses
    for (i, pose) in poses.iter().enumerate() {
        state.add_frame_pose(i as u64, *pose);
    }

    // Add map points
    for i in 0..num_points {
        let position = na::Point3::new(
            (i as f64) * 0.1 - (num_points as f64) * 0.05,
            ((i * 7) % 100) as f64 * 0.01 - 0.5,
            2.0 + ((i * 13) % 50) as f64 * 0.01,
        );
        state.add_map_point(i as u64, position);
    }

    state
}

fn bench_state_management(c: &mut Criterion) {
    let mut group = c.benchmark_group("state_management");
    group.sample_size(20);

    // Benchmark adding frames
    group.bench_function("add_frame_pose", |b| {
        let mut state = State::new();
        let angle = 0.5;
        let rotation = na::Rotation3::from_euler_angles(angle * 0.5, angle, angle * 0.3);
        let translation = na::Vector3::new(0.1, angle.sin() * 0.05, angle.cos() * 0.05);
        let pose = na::Isometry3::from_parts(na::Translation3::from(translation), rotation);

        b.iter(|| {
            state.add_frame_pose(black_box(0), black_box(pose));
        });
    });

    // Benchmark adding map points
    group.bench_function("add_map_point", |b| {
        let mut state = State::new();
        let position = black_box(na::Point3::new(0.5, 0.3, 2.0));

        b.iter(|| {
            state.add_map_point(black_box(0), position);
        });
    });

    group.finish();
}

fn bench_sliding_window_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("sliding_window");
    group.sample_size(10);

    for window_size in &[3, 5, 10] {
        group.bench_with_input(
            BenchmarkId::new("window_size", window_size),
            window_size,
            |b, &window_size| {
                let mut window = SlidingWindow::new(window_size);
                let state = create_test_state(window_size, window_size * 10);

                b.iter(|| {
                    let _window_state = window.get_current_state();
                    let _poses = window.get_keyframe_poses();
                });
            },
        );
    }

    group.finish();
}

fn bench_state_serialization(c: &mut Criterion) {
    let mut group = c.benchmark_group("state_serialization");
    group.sample_size(10);

    for num_points in &[50, 100, 200, 500] {
        group.bench_with_input(
            BenchmarkId::new("num_points", num_points),
            num_points,
            |b, &num_points| {
                let state = black_box(create_test_state(5, num_points));

                b.iter(|| {
                    // Simulate state copying (common operation)
                    let _state_copy = state.clone();
                });
            },
        );
    }

    group.finish();
}

fn bench_observation_management(c: &mut Criterion) {
    let mut group = c.benchmark_group("observation_management");
    group.sample_size(20);

    group.bench_function("record_observation", |b| {
        let mut state = State::new();
        state.add_frame_pose(0, na::Isometry3::identity());
        state.add_map_point(0, na::Point3::new(0.5, 0.3, 2.0));

        let observation = Observation {
            frame_id: 0,
            map_point_id: 0,
            pixel_coordinates: PixelCoordinates { x: 320.0, y: 240.0 },
            descriptor: vec![0u8; 32],
        };

        b.iter(|| {
            state.record_observation(
                black_box(observation.frame_id),
                black_box(observation.map_point_id),
                black_box(observation.pixel_coordinates),
            );
        });
    });

    group.finish();
}

fn bench_pose_recovery_time(c: &mut Criterion) {
    let mut group = c.benchmark_group("pose_recovery");
    group.sample_size(20);

    for num_poses in &[1, 3, 5, 10] {
        group.bench_with_input(
            BenchmarkId::new("num_poses", num_poses),
            num_poses,
            |b, &num_poses| {
                let mut state = create_test_state(*num_poses, *num_poses * 5);

                b.iter(|| {
                    for i in 0..*num_poses {
                        let _pose = state.get_frame_pose(i as u64);
                    }
                });
            },
        );
    }

    group.finish();
}

criterion_group!(
    benches,
    bench_state_management,
    bench_sliding_window_operations,
    bench_state_serialization,
    bench_observation_management,
    bench_pose_recovery_time
);
criterion_main!(benches);
