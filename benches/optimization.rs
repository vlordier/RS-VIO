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

fn bench_factor_computation(_c: &mut Criterion) {
    // Placeholder for future factor benchmarking
}

criterion_group!(benches, bench_bundle_adjustment, bench_factor_computation);
criterion_main!(benches);
