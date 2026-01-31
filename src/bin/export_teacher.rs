//! Teacher Data Export Binary
//!
//! Runs offline teacher processing on a TUM-VI dataset to generate high-quality
//! training data for the student network. This binary loads configurations from
//! a YAML file and exports pose, depth, optical flow, and IMU data to HDF5.
//!
//! Usage:
//! ```bash
//! cargo run --bin export_teacher --features export-teacher -- config/teacher_offline.yaml
//! ```

use rs_vio::{
    datasets::{player_trait::DatasetPlayer, Config, PlayerConfig, TUMVIPlayer},
    estimator::Estimator,
    init_colored_logging,
};
use std::path::PathBuf;

#[derive(Debug, Clone)]
struct ExportArgs {
    config: PathBuf,
    dataset_path: PathBuf,
    output_dir: PathBuf,
    sequence_name: Option<String>,
}

impl ExportArgs {
    fn parse() -> Self {
        let mut config = PathBuf::from("config/teacher_tumvi_export.yaml");
        let mut dataset_path = PathBuf::from(".");
        let mut output_dir = PathBuf::from("./teacher_data");
        let mut sequence_name = None;

        let args: Vec<String> = std::env::args().collect();
        let mut i = 1;
        while i < args.len() {
            match args[i].as_str() {
                "--config" => {
                    if i + 1 < args.len() {
                        config = PathBuf::from(&args[i + 1]);
                        i += 2;
                    } else {
                        i += 1;
                    }
                },
                "--dataset-path" => {
                    if i + 1 < args.len() {
                        dataset_path = PathBuf::from(&args[i + 1]);
                        i += 2;
                    } else {
                        i += 1;
                    }
                },
                "--output-dir" => {
                    if i + 1 < args.len() {
                        output_dir = PathBuf::from(&args[i + 1]);
                        i += 2;
                    } else {
                        i += 1;
                    }
                },
                "--sequence-name" => {
                    if i + 1 < args.len() {
                        sequence_name = Some(args[i + 1].clone());
                        i += 2;
                    } else {
                        i += 1;
                    }
                },
                "--help" | "-h" => {
                    Self::print_help();
                    std::process::exit(0);
                },
                _ => {
                    // Try as positional argument (first arg is config)
                    if i == 1 && !args[i].starts_with("--") {
                        config = PathBuf::from(&args[i]);
                    }
                    i += 1;
                },
            }
        }

        ExportArgs {
            config,
            dataset_path,
            output_dir,
            sequence_name,
        }
    }

    fn print_help() {
        println!(
            r#"
Teacher Data Export Binary

Generate high-quality training data from TUM-VI dataset using bundle adjustment.

USAGE:
    export_teacher [CONFIG] [OPTIONS]

ARGS:
    CONFIG                      Configuration file path (default: config/teacher_tumvi_export.yaml)

OPTIONS:
    --config <PATH>             Override config file path
    --dataset-path <PATH>       Path to TUM-VI dataset root directory
    --output-dir <PATH>         Output directory for HDF5 files (default: ./teacher_data)
    --sequence-name <NAME>      Override sequence name in config
    --help, -h                  Print this help message

EXAMPLES:
    # Export single sequence with default config
    export_teacher --dataset-path /data/tum-vi/room1 --output-dir ./room1_export

    # Export with custom config
    export_teacher config/teacher_tumvi_export.yaml --dataset-path /data/tum-vi --output-dir ./full_dataset

    # Export with sequence name override
    export_teacher --dataset-path /data/tum-vi/room2 --sequence-name room2_full
"#
        );
    }
}

fn main() -> anyhow::Result<()> {
    // Initialize logging
    init_colored_logging();

    // Parse command line arguments
    let args = ExportArgs::parse();

    log::info!("Loading configuration from: {}", args.config.display());

    // Load VIO configuration
    let mut vio_config = Config::load(&args.config.to_string_lossy())?;

    // Override configuration with command-line arguments
    vio_config.export_dir = args.output_dir.clone();
    if let Some(seq_name) = args.sequence_name {
        vio_config.sequence_name = seq_name;
    }
    vio_config.enable_export = true;

    vio_config.validate()?;

    // Print configuration summary
    log::info!("Configuration validated:");
    log::info!("  Export enabled: {}", vio_config.enable_export);
    log::info!("  Export dir: {:?}", vio_config.export_dir);
    log::info!("  Sequence: {}", vio_config.sequence_name);
    log::info!(
        "  Camera: {}x{}",
        vio_config.camera.image_width,
        vio_config.camera.image_height
    );
    log::info!(
        "  Features: {} per grid",
        vio_config.feature_detection.max_features_per_grid
    );
    log::info!(
        "  BA iterations: {}",
        vio_config.optimization.bundle_adjustment_max_iterations
    );

    // Create output directory
    std::fs::create_dir_all(&vio_config.export_dir)?;
    log::info!("Output directory: {}", vio_config.export_dir.display());

    // Create estimator with export manager (conditional on export-teacher feature)
    let _estimator = Estimator::new(vio_config.clone(), None);
    log::info!("Estimator initialized");

    // Create dataset player configuration
    let player_config = PlayerConfig {
        config_path: args.config.to_string_lossy().to_string(),
        dataset_path: args.dataset_path.to_string_lossy().to_string(),
        enable_statistics: true,
        enable_console_statistics: true,
        step_mode: false,
        stats_output_path: Some(
            args.output_dir
                .join("export_stats.json")
                .to_string_lossy()
                .to_string(),
        ),
    };

    // Create and run TUM-VI player for dataset processing
    log::info!("Creating TUM-VI player...");
    let player = TUMVIPlayer::new();

    log::info!("Starting offline teacher export...");
    match player.run(&player_config) {
        Ok(result) => {
            log::info!("Teacher export complete!");
            log::info!("  Frames processed: {}", result.processed_frames);
            if result.processed_frames > 0 {
                let _avg_fps = result.processed_frames as f64
                    / result.average_processing_time_ms as f64
                    * 1000.0;
                log::info!(
                    "  Average frame time: {:.2} ms",
                    result.average_processing_time_ms
                );
            }

            if vio_config.enable_export {
                log::info!("Export statistics:");
                log::info!("  Output directory: {:?}", vio_config.export_dir);
                log::info!("  Sequence name: {}", vio_config.sequence_name);
            }
        },
        Err(e) => {
            log::error!("Teacher export failed: {:?}", e);
            return Err(anyhow::anyhow!("Export failed: {}", e));
        },
    }

    Ok(())
}
