use criterion::{black_box, criterion_group, criterion_main, Criterion};
use rs_vio::optimization::tests::{create_test_problem, solve_optimization_problem};

fn bench_bundle_adjustment(c: &mut Criterion) {
    let (params, factors, config) = create_test_problem();

    c.bench_function("bundle_adjustment_full", |b| {
        b.iter(|| {
            let mut params_copy = params.clone();
            solve_optimization_problem(
                black_box(&mut params_copy),
                black_box(&factors),
                black_box(&config),
            );
        });
    });
}

fn bench_factor_computation(c: &mut Criterion) {
    use nalgebra as na;
    use rs_vio::optimization::factors::*;

    let observation = na::Vector2::new(100.0, 200.0);
    let t_c_w = na::Matrix4::identity();
    let factor = ReprojectionFactor::new(observation, t_c_w);

    let point_3d = na::Vector3::new(1.0, 2.0, 5.0);
    let params = na::Vector3::new(0.0, 0.0, 0.0); // dummy params

    c.bench_function("reprojection_factor_residual", |b| {
        b.iter(|| {
            black_box(factor.residual(black_box(&point_3d), black_box(&params)));
        });
    });
}

criterion_group!(benches, bench_bundle_adjustment, bench_factor_computation);
criterion_main!(benches);
