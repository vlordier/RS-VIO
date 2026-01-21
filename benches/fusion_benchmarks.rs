/// Comprehensive benchmarks for IMU-Vision fusion configurations
/// Tests combinations of:
/// - IMU filtering (denoise + higher-order)
/// - Subpixel super-resolution
/// - Different motion scenarios
use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use rs_vio::datasets::config::Config;
use rs_vio::datasets::ImuData;
use rs_vio::estimator::Estimator;

/// Create realistic test image with features
fn create_test_image(width: u32, height: u32) -> Vec<u8> {
    let len = usize::try_from(u64::from(width) * u64::from(height)).unwrap_or(usize::MAX);

    let mut img = vec![128u8; len];

    // Add gradient for texture
    for y in 0..height {
        for x in 0..width {
            let idx_u64 = u64::from(y) * u64::from(width) + u64::from(x);
            let idx = usize::try_from(idx_u64).unwrap_or(0);

            let grad_x_u32 = x.saturating_mul(50) / width;
            let grad_y_u32 = y.saturating_mul(30) / height;
            let grad_x = u8::try_from(grad_x_u32).unwrap_or(u8::MAX);
            let grad_y = u8::try_from(grad_y_u32).unwrap_or(u8::MAX);
            img[idx] = 100u8.saturating_add(grad_x).saturating_add(grad_y);
        }
    }

    // Add feature blobs
    let blobs = [
        (100, 100),
        (300, 100),
        (500, 100),
        (150, 240),
        (350, 240),
        (450, 240),
        (100, 380),
        (300, 380),
        (500, 380),
    ];

    for &(cx, cy) in &blobs {
        for dy in -15i32..15 {
            for dx in -15i32..15 {
                #[allow(clippy::cast_precision_loss)]
                let dist_sq = (dx * dx + dy * dy) as f32;
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let intensity = (200.0f32 * (-dist_sq / 150.0f32).exp()).min(255.0) as u8;

                let x = u32::try_from((cx + dx).clamp(0, width as i32 - 1)).unwrap_or(0);
                let y = u32::try_from((cy + dy).clamp(0, height as i32 - 1)).unwrap_or(0);
                let idx_u64 = u64::from(y) * u64::from(width) + u64::from(x);
                let idx = usize::try_from(idx_u64).unwrap_or(0);

                img[idx] = img[idx].saturating_add(intensity);
            }
        }
    }

    img
}

/// Create stereo pair with disparity (in pixels)
fn create_stereo_pair(width: u32, height: u32, disparity: u32) -> (Vec<u8>, Vec<u8>) {
    let left = create_test_image(width, height);
    let len = usize::try_from(u64::from(width) * u64::from(height)).unwrap_or(usize::MAX);
    let mut right = vec![128u8; len];

    // Shift left image by integer disparity to create right image
    for y in 0..height {
        for x in 0..width {
            let x_left = x;
            let x_right = x.saturating_sub(disparity);

            if x_right < width {
                let idx_right_u64 = u64::from(y) * u64::from(width) + u64::from(x_right);
                let idx_left_u64 = u64::from(y) * u64::from(width) + u64::from(x_left);
                let idx_right = usize::try_from(idx_right_u64).unwrap_or(0);
                let idx_left = usize::try_from(idx_left_u64).unwrap_or(0);
                right[idx_right] = left[idx_left];
            }
        }
    }

    (left, right)
}

/// Create realistic IMU sequence
fn create_imu_sequence(scenario: &str, count: usize) -> Vec<ImuData> {
    let mut imu_data = Vec::with_capacity(count);
    let dt = 0.005; // 200 Hz

    for i in 0..count {
        let t = f64::from(u32::try_from(i).unwrap_or(u32::MAX)) * dt;

        let (gyro, accel) = match scenario {
            "hover" => {
                // Hovering with small noise
                let noise = 0.01 * (t * 10.0).sin();
                (
                    [noise, noise * 0.5, noise * 0.3],
                    [0.0, 0.0, -9.81 + noise * 0.1],
                )
            },
            "gentle_motion" => {
                // Gentle acceleration
                let accel_x = 1.0 * (t * 2.0).sin();
                let accel_y = 0.5 * (t * 3.0).cos();
                (
                    [0.1 * (t * 5.0).sin(), 0.05, 0.02],
                    [accel_x, accel_y, -9.81],
                )
            },
            "aggressive_motion" => {
                // Fast maneuvers
                let accel_x = 5.0 * (t * 4.0).sin();
                let accel_y = 3.0 * (t * 5.0).cos();
                let gyro_z = 2.0 * (t * 3.0).sin();
                (
                    [0.5 * (t * 8.0).sin(), 0.3, gyro_z],
                    [accel_x, accel_y, -9.81 + accel_x * 0.5],
                )
            },
            "noisy" => {
                // High noise environment
                let noise_level = 0.5;
                let gyro_noise = noise_level * ((t * 100.0).sin() + (t * 73.0).cos());
                let accel_noise = noise_level * ((t * 120.0).sin() + (t * 89.0).cos());
                (
                    [gyro_noise, gyro_noise * 0.7, gyro_noise * 0.3],
                    [accel_noise, accel_noise * 0.5, -9.81 + accel_noise],
                )
            },
            _ => ([0.0; 3], [0.0, 0.0, -9.81]),
        };

        imu_data.push(ImuData {
            timestamp: i64::try_from(i).unwrap_or(i64::MAX).saturating_mul(5_000_000), // 5ms intervals
            gyro,
            accel,
        });
    }

    imu_data
}

/// Benchmark configuration for testing
struct BenchConfig {
    enable_imu_filtering: bool,
    enable_super_resolution: bool,
    scenario: &'static str,
}

impl BenchConfig {
    fn name(&self) -> String {
        format!(
            "imu_{}_superres_{}_{}",
            if self.enable_imu_filtering {
                "ON"
            } else {
                "OFF"
            },
            if self.enable_super_resolution {
                "ON"
            } else {
                "OFF"
            },
            self.scenario
        )
    }
}

fn bench_fusion_configurations(c: &mut Criterion) {
    // Load base config
    let config_yaml = include_str!("../config/euroc_vio.yaml");
    let mut base_config: Config = match serde_yaml::from_str(config_yaml) {
        Ok(cfg) => cfg,
        Err(err) => {
            eprintln!("Failed to parse config: {err}");
            return;
        },
    };

    let width = 640u32;
    let height = 480u32;

    // Test configurations
    let configs = vec![
        BenchConfig {
            enable_imu_filtering: false,
            enable_super_resolution: false,
            scenario: "hover",
        },
        BenchConfig {
            enable_imu_filtering: true,
            enable_super_resolution: false,
            scenario: "hover",
        },
        BenchConfig {
            enable_imu_filtering: false,
            enable_super_resolution: true,
            scenario: "hover",
        },
        BenchConfig {
            enable_imu_filtering: true,
            enable_super_resolution: true,
            scenario: "hover",
        },
        BenchConfig {
            enable_imu_filtering: false,
            enable_super_resolution: false,
            scenario: "gentle_motion",
        },
        BenchConfig {
            enable_imu_filtering: true,
            enable_super_resolution: false,
            scenario: "gentle_motion",
        },
        BenchConfig {
            enable_imu_filtering: false,
            enable_super_resolution: true,
            scenario: "gentle_motion",
        },
        BenchConfig {
            enable_imu_filtering: true,
            enable_super_resolution: true,
            scenario: "gentle_motion",
        },
        BenchConfig {
            enable_imu_filtering: false,
            enable_super_resolution: false,
            scenario: "aggressive_motion",
        },
        BenchConfig {
            enable_imu_filtering: true,
            enable_super_resolution: false,
            scenario: "aggressive_motion",
        },
        BenchConfig {
            enable_imu_filtering: false,
            enable_super_resolution: true,
            scenario: "aggressive_motion",
        },
        BenchConfig {
            enable_imu_filtering: true,
            enable_super_resolution: true,
            scenario: "aggressive_motion",
        },
        BenchConfig {
            enable_imu_filtering: false,
            enable_super_resolution: false,
            scenario: "noisy",
        },
        BenchConfig {
            enable_imu_filtering: true,
            enable_super_resolution: false,
            scenario: "noisy",
        },
        BenchConfig {
            enable_imu_filtering: false,
            enable_super_resolution: true,
            scenario: "noisy",
        },
        BenchConfig {
            enable_imu_filtering: true,
            enable_super_resolution: true,
            scenario: "noisy",
        },
    ];

    let mut group = c.benchmark_group("fusion_configurations");

    for config in configs {
        // Create stereo images
        let (left_img, right_img) = create_stereo_pair(width, height, 8);

        // Create IMU data
        let imu_data = create_imu_sequence(config.scenario, 20); // 20 samples @ 200Hz = 0.1s

        // Configure estimator
        base_config.debug.use_imu = config.enable_imu_filtering;

        group.bench_with_input(
            BenchmarkId::new("frame_processing", config.name()),
            &(&left_img, &right_img, &imu_data),
            |b, (left, right, imu)| {
                let mut estimator = Estimator::new(base_config.clone(), None);
                let timestamp = 1000000000i64; // 1 second

                b.iter(|| {
                    let _ = estimator.process_frame(
                        black_box(left),
                        black_box(right),
                        black_box(timestamp),
                        Some(black_box(imu)),
                    );
                });
            },
        );
    }

    group.finish();
}

fn bench_imu_filtering_only(c: &mut Criterion) {
    let mut group = c.benchmark_group("imu_filtering");

    let scenarios = ["hover", "gentle_motion", "aggressive_motion", "noisy"];

    for scenario in scenarios {
        let imu_data = create_imu_sequence(scenario, 100);

        group.bench_with_input(
            BenchmarkId::new("denoise_higher_order", scenario),
            &imu_data,
            |b, imu| {
                use rs_vio::imu::{DenoiseConfig, ImuDenoiseFilter};
                use rs_vio::imu::{HigherOrderFilter, HigherOrderFilterConfig};

                let mut denoise = ImuDenoiseFilter::new(DenoiseConfig::default());
                let mut higher_order = HigherOrderFilter::new(HigherOrderFilterConfig::default());

                b.iter(|| {
                    for sample in imu {
                        #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
                        let gyro = [
                            sample.gyro[0] as f32,
                            sample.gyro[1] as f32,
                            sample.gyro[2] as f32,
                        ];
                        #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
                        let accel = [
                            sample.accel[0] as f32,
                            sample.accel[1] as f32,
                            sample.accel[2] as f32,
                        ];

                        let accel_denoised = denoise.process_accel(black_box(&accel));
                        let _gyro_denoised = denoise.process_gyro(black_box(&gyro));

                        let _higher_order_output =
                            higher_order.process_accel(black_box(accel_denoised));
                    }
                });
            },
        );
    }

    group.finish();
}

fn bench_super_resolution_only(c: &mut Criterion) {
    use rs_vio::vision::{StereoSuperResolutionConfig, StereoSuperResolver};

    let mut group = c.benchmark_group("super_resolution");

    let width = 640u32;
    let height = 480u32;
    let (left_img, right_img) = create_stereo_pair(width, height, 8);

    // Different confidence levels
    let confidence_levels = [
        ("low_confidence", 0.2),
        ("medium_confidence", 0.5),
        ("high_confidence", 0.9),
    ];

    for (name, confidence) in confidence_levels {
        group.bench_with_input(
            BenchmarkId::new("refine_features", name),
            &confidence,
            |b, &conf| {
                let mut resolver = StereoSuperResolver::new(StereoSuperResolutionConfig::default());

                // Simulate 50 features
                let left_coords: Vec<(f64, f64)> = (0..50)
                    .map(|i| (f64::from(i * 12 + 50), f64::from(i * 8 + 50)))
                    .collect();
                let right_coords: Vec<(f64, f64)> =
                    left_coords.iter().map(|(x, y)| (x - 8.0, *y)).collect();
                let feature_ids: Vec<usize> = (0..50).collect();

                b.iter(|| {
                    let _refined = resolver.refine_features(
                        black_box(&left_img),
                        black_box(&right_img),
                        width,
                        height,
                        black_box(&left_coords),
                        black_box(&right_coords),
                        black_box(&feature_ids),
                        black_box(conf),
                        "hover",
                        &[0.0, 0.0, 0.0],
                    );
                });
            },
        );
    }

    group.finish();
}

fn bench_complete_pipeline_comparison(c: &mut Criterion) {
    let config_yaml = include_str!("../config/euroc_vio.yaml");
    let mut base_config: Config = match serde_yaml::from_str(config_yaml) {
        Ok(cfg) => cfg,
        Err(err) => {
            eprintln!("Failed to parse config: {err}");
            return;
        },
    };

    let width = 640u32;
    let height = 480u32;
    let (left_img, right_img) = create_stereo_pair(width, height, 8);
    let imu_data = create_imu_sequence("gentle_motion", 20);

    let mut group = c.benchmark_group("complete_pipeline");

    // Baseline: No filtering, no super-res
    base_config.debug.use_imu = false;
    group.bench_function("baseline_no_filtering_no_superres", |b| {
        let mut estimator = Estimator::new(base_config.clone(), None);
        b.iter(|| {
            let _ = estimator.process_frame(
                black_box(&left_img),
                black_box(&right_img),
                black_box(1000000000i64),
                Some(black_box(&imu_data)),
            );
        });
    });

    // With IMU filtering only
    base_config.debug.use_imu = true;
    group.bench_function("with_imu_filtering_only", |b| {
        let mut estimator = Estimator::new(base_config.clone(), None);
        b.iter(|| {
            let _ = estimator.process_frame(
                black_box(&left_img),
                black_box(&right_img),
                black_box(1000000000i64),
                Some(black_box(&imu_data)),
            );
        });
    });

    // Full fusion: IMU filtering + super-resolution
    group.bench_function("full_fusion_imu_and_superres", |b| {
        let mut estimator = Estimator::new(base_config.clone(), None);
        b.iter(|| {
            let _ = estimator.process_frame(
                black_box(&left_img),
                black_box(&right_img),
                black_box(1000000000i64),
                Some(black_box(&imu_data)),
            );
        });
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_fusion_configurations,
    bench_imu_filtering_only,
    bench_super_resolution_only,
    bench_complete_pipeline_comparison
);
criterion_main!(benches);
