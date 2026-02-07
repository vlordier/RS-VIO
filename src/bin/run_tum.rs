use clap::Parser;
use log::{error, info};
use rs_vio::{init_logger, DatasetPlayer, PlayerConfig, TUMVIPlayer};
use std::process::ExitCode;

fn main() -> ExitCode {
    init_logger();

    // Parse command line arguments
    let args = Args::parse();

    // Setup configuration
    let player_config = PlayerConfig {
        config_path: args.config_file.clone(),
        dataset_path: args.dataset_path.clone(),
        enable_statistics: true,         // File statistics
        enable_console_statistics: true, // Console statistics
        step_mode: false,
    };

    // Create and run TUM-VI player
    let player = TUMVIPlayer::new();
    let result = player.run(player_config);

    if result.success {
        info!("[Main] processing completed successfully!");
        ExitCode::SUCCESS
    } else {
        error!("[Main] processing failed: {}", result.error_message);
        ExitCode::FAILURE
    }
}

#[derive(Parser, Debug)]
#[command(name = "run_tum")]
#[command(about = "TUM-VI VIO/VO Dataset Player")]
struct Args {
    /// Path to configuration file (YAML)
    #[arg(help = "Path to configuration file (e.g., config/tum_vi.yaml)")]
    config_file: String,

    /// Path to TUM-VI dataset directory
    #[arg(help = "Path to TUM-VI dataset directory (e.g., /path/to/dataset-corridor1_512_16)")]
    dataset_path: String,
}
