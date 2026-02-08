//! 4Seasons dataset runner binary.

use clap::Parser;
use rs_vio::{init_logger, run_dataset, FourSeasonsPlayer};
use std::process::ExitCode;

fn main() -> ExitCode {
    init_logger();
    let args = Args::parse();
    run_dataset::<FourSeasonsPlayer>(&args.config_file, &args.dataset_path)
}

#[derive(Parser, Debug)]
#[command(name = "run_4seasons")]
#[command(about = "4Seasons Dataset Player")]
struct Args {
    /// Path to configuration file (YAML)
    #[arg(help = "Path to configuration file (e.g., config/4seasons.yaml)")]
    config_file: String,

    /// Path to 4Seasons dataset directory
    #[arg(help = "Path to 4Seasons dataset directory (e.g., /path/to/recording_2021-01-07_13-03-56)")]
    dataset_path: String,
}
