//! TUM-VI dataset runner binary.

use clap::Parser;
use rs_vio::{init_logger, run_dataset, TUMVIPlayer};
use std::process::ExitCode;

fn main() -> ExitCode {
    init_logger();
    let args = Args::parse();
    run_dataset::<TUMVIPlayer>(&args.config_file, &args.dataset_path)
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
