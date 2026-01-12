use std::path::PathBuf;
use std::process::Command;
use std::fs;

/// Integration tests validating tight-coupled VIO on real datasets
#[cfg(test)]
mod tight_coupling_integration_tests {
    #![allow(clippy::expect_used, clippy::unwrap_used)]
    use super::*;

    /// Helper to run a sequence and extract metrics
    fn run_sequence(
        binary: &str,
        config: &str,
        data_path: &str,
    ) -> (bool, f64, String) {
        let output = Command::new(binary)
            .arg(config)
            .arg(data_path)
            .output()
            .expect("Failed to run sequence");

        let success = output.status.success();
        let stdout = String::from_utf8_lossy(&output.stdout).to_string();
        let stderr = String::from_utf8_lossy(&output.stderr).to_string();

        // Extract RMSE/ATE from output
        let mut rmse = 0.0;
        for line in stdout.lines() {
            if line.contains("RMSE") || line.contains("ATE") {
                if let Some(val) = line.split_whitespace()
                    .find_map(|w| w.parse::<f64>().ok()) {
                    rmse = val;
                    break;
                }
            }
        }

        (success, rmse, stderr)
    }

    #[test]
    #[ignore]
    fn test_euroc_tight_coupling() {
        let test_cases = vec![
            ("MH_01_easy", 0.10),
            ("MH_03_medium", 0.15),
            ("MH_04_difficult", 0.25),
        ];

        for (sequence, max_ate) in test_cases {
            let binary = "target/release/run_euroc";
            let config = "config/euroc_vio.yaml";
            let data_path = format!("data/euroc/{}", sequence);

            if !PathBuf::from(&data_path).exists() {
                println!("⏭️  Skipping {} - data not found", sequence);
                continue;
            }

            println!("Testing {} with tight coupling...", sequence);
            let (success, ate, _stderr) = run_sequence(binary, config, &data_path);

            assert!(success, "Failed to run {}", sequence);
            assert!(ate < max_ate, "{}: ATE exceeds limit", sequence);
            println!("  ✅ {}: ATE = {:.4}m", sequence, ate);
        }
    }

    #[test]
    #[ignore]
    fn test_tum_vi_tight_coupling() {
        let test_cases = vec![
            ("room1", 0.20),
            ("room2", 0.20),
            ("room6", 0.35),
        ];

        for (sequence, max_ate) in test_cases {
            let binary = "target/release/run_tum";
            let config = "config/tum_vi.yaml";
            let data_path = format!("data/tum-vi/{}", sequence);

            if !PathBuf::from(&data_path).exists() {
                println!("⏭️  Skipping {} - data not found", sequence);
                continue;
            }

            println!("Testing {} with tight coupling...", sequence);
            let (success, ate, _stderr) = run_sequence(binary, config, &data_path);

            assert!(success, "Failed to run TUM-VI {}", sequence);
            assert!(ate < max_ate, "{}: ATE exceeds limit", sequence);
            println!("  ✅ {}: ATE = {:.4}m", sequence, ate);
        }
    }

    #[test]
    #[ignore]
    fn test_4seasons_tight_coupling() {
        let test_cases = vec![
            ("overcast", 0.30),
            ("fog", 0.35),
            ("dusk", 0.40),
        ];

        for (sequence, max_ate) in test_cases {
            let binary = "target/release/run_4seasons";
            let config = "config/4seasons.yaml";
            let data_path = format!("data/4seasons/{}", sequence);

            if !PathBuf::from(&data_path).exists() {
                println!("⏭️  Skipping {} - data not found", sequence);
                continue;
            }

            println!("Testing {} with tight coupling...", sequence);
            let (success, ate, _stderr) = run_sequence(binary, config, &data_path);

            assert!(success, "Failed to run 4Seasons {}", sequence);
            assert!(ate < max_ate, "{}: ATE exceeds limit", sequence);
            println!("  ✅ {}: ATE = {:.4}m", sequence, ate);
        }
    }

    #[test]
    fn test_tight_coupling_unit_features() {
        use rs_vio::estimator::state::State;
        use rs_vio::types::Matrix4x4;

        println!("Verifying tight coupling unit features...");

        let T_B_Cl = Matrix4x4::identity();
        let T_B_Cr = Matrix4x4::identity();
        let state = State::new(T_B_Cl, T_B_Cr);
        
        // Verify state is properly initialized
        assert_eq!(state.T_W_B, Matrix4x4::identity(), "Initial pose should be identity");
        println!("  ✅ State structure is properly initialized");
    }

    #[test]
    fn test_configuration_parsing() {
        println!("Validating configuration files...");

        let configs = vec![
            "config/euroc_vio.yaml",
            "config/tum_vi.yaml",
            "config/4seasons.yaml",
        ];

        for config in configs {
            let path = PathBuf::from(config);
            if path.exists() {
                let content = fs::read_to_string(&path).unwrap();
                // Check for camera or imu sections that indicate valid config
                assert!(
                    content.contains("camera:") || content.contains("image_width"),
                    "Config missing camera configuration"
                );
                assert!(content.len() > 100, "Config appears too small");
                println!("  ✅ {}: Valid", config);
            }
        }
    }
}

#[cfg(test)]
mod tight_coupling_performance_tests {
    #![allow(clippy::expect_used, clippy::unwrap_used)]
    use super::*;

    #[test]
    #[ignore]
    fn bench_euroc_mh01() {
        println!("\n📊 EuRoC MH_01_easy Performance Benchmark");
        
        let binary = "target/release/run_euroc";
        let config = "config/euroc_vio.yaml";
        let data_path = "data/euroc/MH_01_easy";

        if !PathBuf::from(data_path).exists() {
            println!("⏭️  Skipping - data not found");
            return;
        }

        let start = std::time::Instant::now();
        let output = Command::new(binary)
            .arg(config)
            .arg(data_path)
            .output()
            .expect("Failed to run");

        let elapsed = start.elapsed();

        println!("Execution time: {:.2}s", elapsed.as_secs_f32());
        println!("Status: {}", if output.status.success() { "✅ Success" } else { "❌ Failed" });
    }
}
