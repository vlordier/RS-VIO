use clap::Parser;
use log::{error, info};
use rand::rngs::StdRng;
use rand::SeedableRng;
use rs_vio::datasets::player_trait::DatasetPlayer;
use rs_vio::{FourSeasonsPlayer, PlayerConfig};
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
        stats_output_path: None,
    };

    // Create and run EuRoC player
    let player = FourSeasonsPlayer::new();
    match player.run(player_config) {
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
#[command(name = "run_4seasons")]
#[command(about = "4Seasons Dataset Player")]
struct Args {
    /// Path to configuration file (YAML)
    #[arg(help = "Path to configuration file (e.g., config/4seasons.yaml)")]
    config_file: String,

    /// Path to EuRoC dataset directory
    #[arg(help = "Path to EuRoC dataset directory (e.g., /path/to/old_town_1_train)")]
    dataset_path: String,
}
