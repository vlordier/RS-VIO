use clap::Parser;
use log::{error, info};
use rand::rngs::StdRng;
use rand::SeedableRng;
use rs_vio::datasets::player_trait::DatasetPlayer;
use rs_vio::feature_tracker::{BasicRANSACStrategy, StereoMatchingStrategy};
use rs_vio::{EurocPlayer, PlayerConfig};
use std::process;

fn main() {
    // Set random seed for reproducibility
    let _rng = StdRng::seed_from_u64(42);

    // Initialize colored logger
    rs_vio::init_colored_logging();

    // Parse command line arguments
    let args = Args::parse();

    // Create strategy based on argument
    let strategy: Box<dyn StereoMatchingStrategy> = match args.strategy.to_lowercase().as_str() {
        "basicransac" => {
            info!("[Main] Using BasicRANSAC strategy");
            Box::new(BasicRANSACStrategy::new(
                rs_vio::feature_tracker::BasicRANSACConfig::default(),
            ))
        },
        "imuguided" => {
            info!("[Main] IMUGuided unavailable; falling back to BasicRANSAC");
            Box::new(BasicRANSACStrategy::new(
                rs_vio::feature_tracker::BasicRANSACConfig::default(),
            ))
        },
        "temporalconsistency" => {
            info!("[Main] TemporalConsistency unavailable; falling back to BasicRANSAC");
            Box::new(BasicRANSACStrategy::new(
                rs_vio::feature_tracker::BasicRANSACConfig::default(),
            ))
        },
        "hybridopticalflow" => {
            info!("[Main] HybridOpticalFlow unavailable; falling back to BasicRANSAC");
            Box::new(BasicRANSACStrategy::new(
                rs_vio::feature_tracker::BasicRANSACConfig::default(),
            ))
        },
        _ => {
            error!("[Main] Unknown strategy: {}", args.strategy);
            eprintln!(
                "Valid strategies: basicransac, imuguided, temporalconsistency, hybridopticalflow"
            );
            process::exit(1);
        },
    };

    // Setup configuration
    let player_config = PlayerConfig {
        config_path: args.config_file.clone(),
        dataset_path: args.dataset_path.clone(),
        enable_statistics: true,         // File statistics
        enable_console_statistics: true, // Console statistics
        step_mode: false,
        stats_output_path: args.stats_out.clone(),
    };

    // Create and run EuRoC player
    // TODO: Inject strategy into player for runtime switching
    let player = EurocPlayer::new();
    match player.run(&player_config) {
        Ok(result) => {
            info!("[Main] processing completed successfully!");
            info!(
                "Processed {} frames with average {:.2}ms per frame",
                result.processed_frames, result.average_processing_time_ms
            );
            info!("Strategy: {}", strategy.name());
            process::exit(0);
        },
        Err(e) => {
            error!("[Main] processing failed: {}", e);
            process::exit(-1);
        },
    }
}

#[derive(Parser, Debug)]
#[command(name = "euroc_vio_strategies")]
#[command(about = "EuRoC VIO/VO Dataset Player with Strategy Support")]
struct Args {
    /// Path to configuration file (YAML)
    #[arg(value_name = "CONFIG")]
    config_file: String,

    /// Path to dataset directory
    #[arg(value_name = "DATASET_PATH")]
    dataset_path: String,

    /// Output path for statistics (optional)
    #[arg(short, long, value_name = "PATH")]
    stats_out: Option<String>,

    /// Stereo matching strategy to use
    /// Options: basicransac, imuguided, temporalconsistency, hybridopticalflow
    #[arg(short, long, value_name = "STRATEGY", default_value = "basicransac")]
    strategy: String,
}
