use clap::Parser;
use rs_vio::bin_common;
use rs_vio::EurocPlayer;

fn main() {
    let _rng = bin_common::init_runtime();
    let args = Args::parse();
    let config = bin_common::build_player_config(args.config_file, args.dataset_path);
    let player = EurocPlayer::new();
    bin_common::run_and_exit(player, config, "EuRoC");
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
