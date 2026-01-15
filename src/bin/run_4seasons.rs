use clap::Parser;
use rs_vio::bin_common;
use rs_vio::FourSeasonsPlayer;

fn main() {
    let _rng = bin_common::init_runtime();
    let args = Args::parse();
    let config = bin_common::build_player_config(args.config_file, args.dataset_path);
    let player = FourSeasonsPlayer::new();
    bin_common::run_and_exit(player, config, "4Seasons");
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
