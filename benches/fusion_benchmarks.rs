/// Comprehensive benchmarks for IMU-Vision fusion configurations
/// Tests combinations of:
/// - IMU filtering (denoise + higher-order)
/// - Subpixel super-resolution
/// - Different motion scenarios

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use rs_vio::estimator::Estimator;
use rs_vio::datasets::config::Config;
use rs_vio::datasets::ImuData;

/// Create realistic test image with features
fn create_test_image(width: u32, height: u32) -> Vec<u8> {
    let mut img = vec![128u8; (width * height) as usize];
    
    // Add gradient for texture
    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) as usize;
            let grad_x = ((x as f64 / width as f64) * 50.0) as u8;
            let grad_y = ((y as f64 / height as f64) * 30.0) as u8;
            img[idx] = 100u8.saturating_add(grad_x).saturating_add(grad_y);
        }
    }
    
    // Add feature blobs
    let blobs = [
        (100, 100), (300, 100), (500, 100),
        (150, 240), (350, 240), (450, 240),
        (100, 380), (300, 380), (500, 380),
    ];
    
    for &(cx, cy) in &blobs {
        for dy in -15i32..15 {
            for dx in -15i32..15 {
                let dist_sq = (dx * dx + dy * dy) as f32;
                let intensity = (200.0 * (-dist_sq / 150.0).exp()) as u8;
                
                let x = (cx as i32 + dx).max(0).min(width as i32 - 1) as u32;
                let y = (cy as i32 + dy).max(0).min(height as i32 - 1) as u32;
                let idx = (y * width + x) as usize;
                
                img[idx] = img[idx].saturating_add(intensity);
            }
        }
    }
    
    img
}

/// Create stereo pair with disparity
fn create_stereo_pair(width: u32, height: u32, disparity: f32) -> (Vec<u8>, Vec<u8>) {
    let left = create_test_image(width, height);
    let mut right = vec![128u8; (width * height) as usize];
    
    // Shift left image by disparity to create right image
    for y in 0..height {
        for x in 0..width {
            let x_left = x;
            let x_right = (x as f32 - disparity).max(0.0) as u32;
            
            if x_right < width {
                let idx_right = (y * width + x_right) as usize;
                let idx_left = (y * width + x_left) as usize;
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
        let t = i as f64 * dt;
        
        let (gyro, accel) = match scenario {
            "hover" => {
                // Hovering with small noise
                let noise = 0.01 * (t * 10.0).sin();
                (
                    [noise, noise * 0.5, noise * 0.3],
                    [0.0, 0.0, -9.81 + noise * 0.1]
                )
            },
            "gentle_motion" => {
                // Gentle acceleration
                let accel_x = 1.0 * (t * 2.0).sin();
                let accel_y = 0.5 * (t * 3.0).cos();
                (
                    [0.1 * (t * 5.0).sin(), 0.05, 0.02],
                    [accel_x, accel_y, -9.81]
                )
            },
            "aggressive_motion" => {
                // Fast maneuvers
                let accel_x = 5.0 * (t * 4.0).sin();
                let accel_y = 3.0 * (t * 5.0).cos();
                let gyro_z = 2.0 * (t * 3.0).sin();
                (
                    [0.5 * (t * 8.0).sin(), 0.3, gyro_z],
                    [accel_x, accel_y, -9.81 + accel_x * 0.5]
                )
            },
            "noisy" => {
                // High noise environment
                let noise_level = 0.5;
                let gyro_noise = noise_level * ((t * 100.0).sin() + (t * 73.0).cos());
                let accel_noise = noise_level * ((t * 120.0).sin() + (t * 89.0).cos());
                (
                    [gyro_noise, gyro_noise * 0.7, gyro_noise * 0.3],
                    [accel_noise, accel_noise * 0.5, -9.81 + accel_noise]
                )
            },
            _ => ([0.0; 3], [0.0, 0.0, -9.81]),
        };
        
        imu_data.push(ImuData {
            timestamp: (i as i64) * 5_000_000, // 5ms intervals
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
            if self.enable_imu_filtering { "ON" } else { "OFF" },
            if self.enable_super_resolution { "ON" } else { "OFF" },
            self.scenario
        )
    }
}

fn bench_fusion_configurations(c: &mut Criterion) {
    // Load base config
    let config_yaml = include_str!("../config/euroc_vio.yaml");
    let mut base_config: Config = serde_yaml::from_str(config_yaml)
        .expect("Failed to parse config");
    
    let width = 640u32;
    let height = 480u32;
    
    // Test configurations
    let configs = vec![
        BenchConfig { enable_imu_filtering: false, enable_super_resolution: false, scenario: "hover" },
        BenchConfig { enable_imu_filtering: true, enable_super_resolution: false, scenario: "hover" },
        BenchConfig { enable_imu_filtering: false, enable_super_resolution: true, scenario: "hover" },
        BenchConfig { enable_imu_filtering: true, enable_super_resolution: true, scenario: "hover" },
        
        BenchConfig { enable_imu_filtering: false, enable_super_resolution: false, scenario: "gentle_motion" },
        BenchConfig { enable_imu_filtering: true, enable_super_resolution: false, scenario: "gentle_motion" },
        BenchConfig { enable_imu_filtering: false, enable_super_resolution: true, scenario: "gentle_motion" },
        BenchConfig { enable_imu_filtering: true, enable_super_resolution: true, scenario: "gentle_motion" },
        
        BenchConfig { enable_imu_filtering: false, enable_super_resolution: false, scenario: "aggressive_motion" },
        BenchConfig { enable_imu_filtering: true, enable_super_resolution: false, scenario: "aggressive_motion" },
        BenchConfig { enable_imu_filtering: false, enable_super_resolution: true, scenario: "aggressive_motion" },
        BenchConfig { enable_imu_filtering: true, enable_super_resolution: true, scenario: "aggressive_motion" },
        
        BenchConfig { enable_imu_filtering: false, enable_super_resolution: false, scenario: "noisy" },
        BenchConfig { enable_imu_filtering: true, enable_super_resolution: false, scenario: "noisy" },
        BenchConfig { enable_imu_filtering: false, enable_super_resolution: true, scenario: "noisy" },
        BenchConfig { enable_imu_filtering: true, enable_super_resolution: true, scenario: "noisy" },
    ];
    
    let mut group = c.benchmark_group("fusion_configurations");
    
    for config in configs {
        // Create stereo images
        let (left_img, right_img) = create_stereo_pair(width, height, 8.0);
        
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
                use rs_vio::imu::{ImuDenoiseFilter, DenoiseConfig};
                use rs_vio::imu::{HigherOrderFilter, HigherOrderFilterConfig};
                
                let mut denoise = ImuDenoiseFilter::new(DenoiseConfig::default());
                let mut higher_order = HigherOrderFilter::new(HigherOrderFilterConfig::default());
                
                b.iter(|| {
                    for sample in imu {
                        let gyro = [sample.gyro[0] as f32, sample.gyro[1] as f32, sample.gyro[2] as f32];
                        let accel = [sample.accel[0] as f32, sample.accel[1] as f32, sample.accel[2] as f32];
                        
                        let accel_denoised = denoise.process_accel(black_box(&accel));
                        let _gyro_denoised = denoise.process_gyro(black_box(&gyro));
                        
                        let _higher_order_output = higher_order.process_accel(black_box(accel_denoised));
                    }
                });
            },
        );
    }
    
    group.finish();
}

fn bench_super_resolution_only(c: &mut Criterion) {
    use rs_vio::vision::{StereoSuperResolver, StereoSuperResolutionConfig};
    
    let mut group = c.benchmark_group("super_resolution");
    
    let width = 640u32;
    let height = 480u32;
    let (left_img, right_img) = create_stereo_pair(width, height, 8.0);
    
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
                    .map(|i| ((i * 12 + 50) as f64, (i * 8 + 50) as f64))
                    .collect();
                let right_coords: Vec<(f64, f64)> = left_coords.iter()
                    .map(|(x, y)| (x - 8.0, *y))
                    .collect();
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
    let mut base_config: Config = serde_yaml::from_str(config_yaml)
        .expect("Failed to parse config");
    
    let width = 640u32;
    let height = 480u32;
    let (left_img, right_img) = create_stereo_pair(width, height, 8.0);
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
