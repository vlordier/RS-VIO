#![allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use nalgebra::{Isometry3, Vector3};
use rs_vio::optimization::loop_closure::{
    CosineMatcher, DescriptorMatcher, GeometricVerifier, HammingMatcher, KeyframeDescriptor,
    LoopClosureConfig, LoopClosureDetector, RansacEpipolarVerifier, SimpleRelativePoseVerifier,
};

fn create_test_descriptor(id: u64, translation: f64) -> KeyframeDescriptor {
    let translation_vec = Vector3::new(translation, 0.0, 0.0);
    let pose = Isometry3::from_parts(translation_vec.into(), nalgebra::UnitQuaternion::identity());

    KeyframeDescriptor {
        keyframe_id: id,
        timestamp: (id as i64) * 100_000_000,
        descriptor: vec![translation; 128],
        num_features: 100,
        pose,
    }
}

fn bench_matcher_cosine(c: &mut Criterion) {
    let mut group = c.benchmark_group("matcher_cosine");

    for num_features in [50, 100, 200] {
        group.bench_with_input(
            BenchmarkId::from_parameter(num_features),
            &num_features,
            |b, &nf| {
                let matcher = CosineMatcher;
                let desc1 = KeyframeDescriptor {
                    num_features: nf,
                    ..create_test_descriptor(0, 0.0)
                };
                let desc2 = KeyframeDescriptor {
                    num_features: nf,
                    ..create_test_descriptor(1, 0.5)
                };

                b.iter(|| {
                    black_box(matcher.match_keyframes(&desc1, &desc2));
                });
            },
        );
    }
    group.finish();
}

fn bench_matcher_hamming(c: &mut Criterion) {
    let mut group = c.benchmark_group("matcher_hamming");

    for num_features in [50, 100, 200] {
        group.bench_with_input(
            BenchmarkId::from_parameter(num_features),
            &num_features,
            |b, &nf| {
                let matcher = HammingMatcher::default();
                let desc1 = KeyframeDescriptor {
                    num_features: nf,
                    ..create_test_descriptor(0, 0.0)
                };
                let desc2 = KeyframeDescriptor {
                    num_features: nf,
                    ..create_test_descriptor(1, 0.5)
                };

                b.iter(|| {
                    black_box(matcher.match_keyframes(&desc1, &desc2));
                });
            },
        );
    }
    group.finish();
}

fn bench_verifier_simple(c: &mut Criterion) {
    let mut group = c.benchmark_group("verifier_simple");

    let verifier = SimpleRelativePoseVerifier {
        min_similarity: 0.3,
    };
    let desc1 = create_test_descriptor(0, 0.0);
    let desc2 = create_test_descriptor(1, 0.5);
    let matcher = CosineMatcher;
    let metrics = matcher.match_keyframes(&desc1, &desc2);

    group.bench_function("verify", |b| {
        b.iter(|| {
            black_box(verifier.verify(&desc1, &desc2, &metrics));
        });
    });

    group.finish();
}

fn bench_verifier_ransac(c: &mut Criterion) {
    let mut group = c.benchmark_group("verifier_ransac");

    for iterations in [100, 500, 1000] {
        group.bench_with_input(
            BenchmarkId::from_parameter(iterations),
            &iterations,
            |b, &iters| {
                let verifier = RansacEpipolarVerifier {
                    max_iterations: iters,
                    inlier_threshold: 1e-3,
                    min_inlier_ratio: 0.3,
                };
                let desc1 = create_test_descriptor(0, 0.0);
                let desc2 = create_test_descriptor(1, 0.5);
                let matcher = CosineMatcher;
                let metrics = matcher.match_keyframes(&desc1, &desc2);

                b.iter(|| {
                    black_box(verifier.verify(&desc1, &desc2, &metrics));
                });
            },
        );
    }

    group.finish();
}

fn bench_detection_database_size(c: &mut Criterion) {
    let mut group = c.benchmark_group("detection_database_size");

    for db_size in [10, 50, 100, 500] {
        group.bench_with_input(
            BenchmarkId::from_parameter(db_size),
            &db_size,
            |b, &size| {
                let config = LoopClosureConfig {
                    min_frame_gap: 10,
                    descriptor_distance_threshold: 0.5,
                    max_keyframe_database_size: size,
                    ..Default::default()
                };
                let mut detector = LoopClosureDetector::new(config);

                // Populate database
                for i in 0..size {
                    let desc = create_test_descriptor(i as u64, i as f64 * 0.1);
                    let _ = detector.detect_loop_closure(i as u64, desc);
                }

                // Benchmark loop closure detection
                let query = create_test_descriptor((size + 100) as u64, 5.0);

                b.iter(|| {
                    let _ =
                        black_box(detector.detect_loop_closure((size + 100) as u64, query.clone()));
                });
            },
        );
    }

    group.finish();
}

fn bench_detection_matcher_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("detection_matcher_comparison");

    // Cosine matcher
    group.bench_function("cosine", |b| {
        let config = LoopClosureConfig {
            min_frame_gap: 10,
            max_keyframe_database_size: 100,
            ..Default::default()
        };
        let matcher: Box<dyn DescriptorMatcher> = Box::new(CosineMatcher);
        let verifier: Box<dyn GeometricVerifier> = Box::new(SimpleRelativePoseVerifier {
            min_similarity: 0.3,
        });
        let mut detector = LoopClosureDetector::new_with(config, matcher, verifier);

        // Populate database
        for i in 0..100 {
            let desc = create_test_descriptor(i, i as f64 * 0.1);
            let _ = detector.detect_loop_closure(i, desc);
        }

        let query = create_test_descriptor(200, 5.0);

        b.iter(|| {
            let _ = black_box(detector.detect_loop_closure(200, query.clone()));
        });
    });

    // Hamming matcher
    group.bench_function("hamming", |b| {
        let config = LoopClosureConfig {
            min_frame_gap: 10,
            max_keyframe_database_size: 100,
            ..Default::default()
        };
        let matcher: Box<dyn DescriptorMatcher> = Box::new(HammingMatcher::default());
        let verifier: Box<dyn GeometricVerifier> = Box::new(SimpleRelativePoseVerifier {
            min_similarity: 0.3,
        });
        let mut detector = LoopClosureDetector::new_with(config, matcher, verifier);

        // Populate database
        for i in 0..100 {
            let desc = create_test_descriptor(i, i as f64 * 0.1);
            let _ = detector.detect_loop_closure(i, desc);
        }

        let query = create_test_descriptor(200, 5.0);

        b.iter(|| {
            let _ = black_box(detector.detect_loop_closure(200, query.clone()));
        });
    });

    group.finish();
}

fn bench_detection_verifier_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("detection_verifier_comparison");

    // Simple verifier
    group.bench_function("simple", |b| {
        let config = LoopClosureConfig {
            min_frame_gap: 10,
            max_keyframe_database_size: 100,
            ..Default::default()
        };
        let matcher: Box<dyn DescriptorMatcher> = Box::new(CosineMatcher);
        let verifier: Box<dyn GeometricVerifier> = Box::new(SimpleRelativePoseVerifier {
            min_similarity: 0.3,
        });
        let mut detector = LoopClosureDetector::new_with(config, matcher, verifier);

        // Populate database
        for i in 0..100 {
            let desc = create_test_descriptor(i, i as f64 * 0.1);
            let _ = detector.detect_loop_closure(i, desc);
        }

        let query = create_test_descriptor(200, 5.0);

        b.iter(|| {
            let _ = black_box(detector.detect_loop_closure(200, query.clone()));
        });
    });

    // RANSAC verifier
    group.bench_function("ransac", |b| {
        let config = LoopClosureConfig {
            min_frame_gap: 10,
            max_keyframe_database_size: 100,
            min_inliers: 20,
            min_matches_for_candidate: 20,
            ..Default::default()
        };
        let matcher: Box<dyn DescriptorMatcher> = Box::new(CosineMatcher);
        let verifier: Box<dyn GeometricVerifier> = Box::new(RansacEpipolarVerifier::default());
        let mut detector = LoopClosureDetector::new_with(config, matcher, verifier);

        // Populate database
        for i in 0..100 {
            let desc = create_test_descriptor(i, i as f64 * 0.1);
            let _ = detector.detect_loop_closure(i, desc);
        }

        let query = create_test_descriptor(200, 5.0);

        b.iter(|| {
            let _ = black_box(detector.detect_loop_closure(200, query.clone()));
        });
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_matcher_cosine,
    bench_matcher_hamming,
    bench_verifier_simple,
    bench_verifier_ransac,
    bench_detection_database_size,
    bench_detection_matcher_comparison,
    bench_detection_verifier_comparison,
);

criterion_main!(benches);
