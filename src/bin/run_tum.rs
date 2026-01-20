use clap::Parser;
use log::{error, info};
use rand::rngs::StdRng;
use rand::SeedableRng;
use rs_vio::datasets::player_trait::DatasetPlayer;
use rs_vio::{PlayerConfig, TUMVIPlayer};
use std::process;

fn main() {
    // Set random seed for reproducibility
    let _rng = StdRng::seed_from_u64(42);

    // Initialize colored logger
    rs_vio::init_colored_logging();

    // Parse command line arguments
    let args = Args::parse();

    // Setup configuration
    let player_config = PlayerConfig {
        config_path: args.config_file.clone(),
        dataset_path: args.dataset_path.clone(),
        enable_statistics: true,         // File statistics
        enable_console_statistics: true, // Console statistics
        step_mode: false,
        stats_output_path: args.stats_out.clone(),
    };

    // Create and run TUM-VI player
    let player = TUMVIPlayer::new();
    match player.run(&player_config) {
        Ok(result) => {
            info!("[Main] processing completed successfully!");
            info!(
                "Processed {} frames with average {:.2}ms per frame",
                result.processed_frames, result.average_processing_time_ms
            );
            process::exit(0);
        },
        Err(e) => {
            error!("[Main] processing failed: {}", e);
            process::exit(-1);
        },
    }
}

#[derive(Parser, Debug)]
#[command(name = "tum_vi_vio")]
#[command(about = "TUM-VI VIO/VO Dataset Player")]
struct Args {
    /// Path to configuration file (YAML)
    #[arg(help = "Path to configuration file (e.g., config/tum_vi.yaml)")]
    config_file: String,

    /// Path to TUM-VI dataset directory
    #[arg(help = "Path to dataset root (e.g., /path/to/TUM-VI/dataset-room1_512_16)")]
    dataset_path: String,

    /// Optional path to write statistics; will also emit a CSV of per-frame timing
    #[arg(long, help = "Optional path to write statistics; emits *_frames.csv too")]
    stats_out: Option<String>,
}
