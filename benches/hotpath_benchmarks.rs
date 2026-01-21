use criterion::{black_box, criterion_group, criterion_main, Criterion};
use rs_vio::optimization::loop_closure::{
    orb_matcher::OrbMatcher, DescriptorMatcher, KeyframeDescriptor,
};
use rs_vio::types::{Float, Isometry3, Vector3};
use std::collections::BTreeMap;

fn create_test_keyframe_descriptors() -> (KeyframeDescriptor, KeyframeDescriptor) {
    let desc1 = KeyframeDescriptor {
        keyframe_id: 1,
        timestamp: 1000000,
        descriptor: (0..256)
            .map(|x| (Float::from(x) * 0.01).sin())
            .collect(),
        num_features: 100,
        pose: Isometry3::identity(),
    };

    let desc2 = KeyframeDescriptor {
        keyframe_id: 2,
        timestamp: 2000000,
        descriptor: (0..256)
            .map(|x| (Float::from(x) * 0.015).cos())
            .collect(),
        num_features: 95,
        pose: Isometry3::new(Vector3::new(1.0, 0.5, 0.0), Vector3::zeros()),
    };

    (desc1, desc2)
}

// Estimator benchmarks removed due to complex dependencies
// TODO: Add back when estimator API is simplified

fn bench_orb_feature_extraction(c: &mut Criterion) {
    use rs_vio::optimization::loop_closure::orb::OrbConfig;
    use rs_vio::optimization::loop_closure::orb::OrbExtractor;

    let config = OrbConfig::default();
    let extractor = OrbExtractor::new(config);
    let image = vec![128u8; 640 * 480]; // Gray image

    c.bench_function("orb_feature_extraction", |b| {
        b.iter(|| {
            let features = extractor.extract(black_box(&image), 640, 480);
            black_box(features);
        });
    });
}

fn bench_orb_descriptor_matching(c: &mut Criterion) {
    let matcher = OrbMatcher::new();
    let (desc1, desc2) = create_test_keyframe_descriptors();

    c.bench_function("orb_descriptor_matching", |b| {
        b.iter(|| {
            let metrics = matcher.match_keyframes(black_box(&desc1), black_box(&desc2));
            black_box(metrics);
        });
    });
}

fn bench_memory_allocation_patterns(c: &mut Criterion) {
    c.bench_function("vector_allocation_10k", |b| {
        b.iter(|| {
            let vec: Vec<f64> = (0..10000)
                .map(|x| f64::from(x) * 0.1)
                .collect();
            black_box(vec);
        });
    });

    c.bench_function("matrix_allocation_100x100", |b| {
        b.iter(|| {
            let matrix = nalgebra::DMatrix::<f64>::zeros(100, 100);
            black_box(matrix);
        });
    });

    c.bench_function("hashmap_operations", |b| {
        b.iter(|| {
            let mut map = BTreeMap::new();
            for i in 0..1000 {
                map.insert(i, f64::from(i) * 0.1);
            }
            black_box(map);
        });
    });
}

fn bench_numerical_operations(c: &mut Criterion) {
    let matrix = nalgebra::DMatrix::<f64>::from_fn(50, 50, |i, j| {
        let idx = u32::try_from(i + j).unwrap_or(u32::MAX);
        f64::from(idx)
    });
    let vector = nalgebra::DVector::<f64>::from_fn(50, |_, _| 0.0);

    c.bench_function("matrix_vector_multiplication", |b| {
        b.iter(|| {
            let result = black_box(&matrix) * black_box(&vector);
            black_box(result);
        });
    });

    c.bench_function("matrix_matrix_multiplication", |b| {
        b.iter(|| {
            let result = black_box(&matrix) * black_box(&matrix);
            black_box(result);
        });
    });

    c.bench_function("matrix_inverse_20x20", |b| {
        let matrix =
            nalgebra::DMatrix::<f64>::from_fn(20, 20, |i, j| if i == j { 1.0 } else { 0.01 });

        b.iter(|| {
            let result = black_box(&matrix).clone().try_inverse();
            black_box(result);
        });
    });

    c.bench_function("trigonometric_operations", |b| {
        b.iter(|| {
            let mut result = 0.0;
            for i in 0..1000 {
                let x = f64::from(i) * 0.001;
                result += x.sin() + x.cos() + x.tan();
            }
            black_box(result);
        });
    });
}

// Serialization benchmarks removed due to missing serde_json dependency
// TODO: Add back when serde_json is added to dev-dependencies

criterion_group!(
    benches,
    bench_orb_feature_extraction,
    bench_orb_descriptor_matching,
    bench_memory_allocation_patterns,
    bench_numerical_operations,
);

criterion_main!(benches);
