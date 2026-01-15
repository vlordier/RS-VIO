//! Common utilities for dataset runner binaries
//!
//! This module centralizes shared logic across all dataset player binaries
//! (EuRoC, TUM-VI, 4Seasons), reducing duplication and ensuring consistent behavior.

#![allow(clippy::exit)] // Binary utilities need process::exit

use crate::datasets::player_trait::DatasetPlayer;
use crate::PlayerConfig;
use log::{error, info};
use rand::rngs::StdRng;
use rand::SeedableRng;
use std::process;

/// Initialize common runtime setup (logging and RNG)
pub fn init_runtime() -> StdRng {
    // Initialize colored logger
    crate::init_colored_logging();

    // Set random seed for reproducibility
    StdRng::seed_from_u64(42)
}

/// Build standard player configuration from paths
pub fn build_player_config(config_path: String, dataset_path: String) -> PlayerConfig {
    PlayerConfig {
        config_path,
        dataset_path,
        enable_statistics: true,         // File statistics
        enable_console_statistics: true, // Console statistics
        step_mode: false,
        stats_output_path: None,
    }
}

/// Run dataset player with standard error handling and logging
pub fn run_and_exit<P: DatasetPlayer>(player: P, config: PlayerConfig, dataset_name: &str) -> ! {
    info!("[Main] Starting {} dataset player", dataset_name);

    match player.run(config) {
        Ok(result) => {
            info!("[Main] {} processing completed successfully!", dataset_name);
            info!(
                "Processed {} frames with average {:.2}ms per frame",
                result.processed_frames, result.average_processing_time_ms
            );
            process::exit(0);
        },
        Err(e) => {
            error!("[Main] {} processing failed: {}", dataset_name, e);
            process::exit(1);
        },
    }
}
