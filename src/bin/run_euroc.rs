//! EuRoC MAV dataset runner binary.

use clap::Parser;
use rs_vio::{init_logger, run_dataset, EurocPlayer};
use std::process::ExitCode;

fn main() -> ExitCode {
    init_logger();
    let args = Args::parse();
    run_dataset::<EurocPlayer>(&args.config_file, &args.dataset_path)
}

#[derive(Parser, Debug)]
#[command(name = "euroc_vio")]
#[command(about = "EuRoC VIO/VO Dataset Player")]
struct Args {
    /// Path to configuration file (YAML)
    #[arg(help = "Path to configuration file (e.g., config/euroc_vio.yaml)")]
    config_file: String,

    /// Path to EuRoC dataset directory
    #[arg(help = "Path to EuRoC dataset directory (e.g., /path/to/MH_01_easy)")]
    dataset_path: String,
}
