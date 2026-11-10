use clap::Parser;
use env_logger::{Builder, Env};
use log::{error, info, LevelFilter};
use rand::rngs::StdRng;
use rand::SeedableRng;
use rs_vio::{FourSeasonsPlayer, PlayerConfig};
use std::process;

fn main() {
    // Set random seed for reproducibility
    let _rng = StdRng::seed_from_u64(42);

    // Initialize logger for immediate colored output
    Builder::from_env(Env::default().default_filter_or("debug"))
        // Silence rerun noise unless it's a warning or worse
        .filter_module("rerun", LevelFilter::Warn)
        .format_timestamp_millis()
        .format(|buf, record| {
            use std::io::Write;
            let level = match record.level() {
                log::Level::Error => "\x1b[31mERROR\x1b[0m",
                log::Level::Warn => "\x1b[33mWARN\x1b[0m",
                log::Level::Info => "\x1b[32mINFO\x1b[0m",
                log::Level::Debug => "\x1b[34mDEBUG\x1b[0m",
                log::Level::Trace => "\x1b[36mTRACE\x1b[0m",
            };
            writeln!(
                buf,
                "[{}] [{}] {}",
                buf.timestamp_millis(),
                level,
                record.args()
            )
        })
        .init();

    // Parse command line arguments
    let args = Args::parse();

    // Setup configuration
    let player_config = PlayerConfig {
        config_path: args.config_file.clone(),
        dataset_path: args.dataset_path.clone(),
        enable_statistics: true,         // File statistics
        enable_console_statistics: true, // Console statistics
        step_mode: false,
        use_imu: false, // 4Seasons IMU not yet implemented
    };

    // Create and run EuRoC player
    let player = FourSeasonsPlayer::new();
    let result = player.run(player_config);

    if result.success {
        info!("[Main] processing completed successfully!");
        process::exit(0);
    } else {
        error!("[Main] processing failed: {}", result.error_message);
        process::exit(-1);
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
