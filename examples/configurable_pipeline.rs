//! Configurable VIO Pipeline Example
//!
//! Demonstrates how to use the VIO pipeline configuration system with
//! frame stabilization and track-first detection.

use rs_vio::feature_tracker::{
    StereoPatchTracker, VIOPipelineConfig,
    FusionStrategy, TrackingStrategy,
};
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== RS-VIO Configurable Pipeline Demo ===\n");

    // Example 1: Load configuration from file
    println!("Example 1: Loading configuration from file");
    println!("------------------------------------------");
    
    let config_path = PathBuf::from("configs/balanced.toml");
    if config_path.exists() {
        let config = VIOPipelineConfig::load_toml(&config_path)?;
        config.validate()?;
        println!("{}", config.summary());
        println!();
    } else {
        println!("Config file not found at {:?}", config_path);
        println!();
    }

    // Example 2: Using platform presets
    println!("Example 2: Platform Presets");
    println!("----------------------------");
    
    // CPU-only configuration
    let cpu_config = VIOPipelineConfig::cpu_only();
    println!("CPU-Only Platform:");
    println!("  Target FPS: {}", cpu_config.performance.target_fps);
    println!("  Fusion: {:?}", cpu_config.fusion.strategy);
    println!("  Detection: {:?}", cpu_config.detection.backend);
    println!();

    // GPU-enabled configuration
    let gpu_config = VIOPipelineConfig::gpu_enabled();
    println!("GPU-Enabled Platform:");
    println!("  Target FPS: {}", gpu_config.performance.target_fps);
    println!("  Fusion: {:?}", gpu_config.fusion.strategy);
    println!("  Descriptors: {:?}", gpu_config.matching.descriptor_type);
    println!();

    // Hard realtime configuration
    let rt_config = VIOPipelineConfig::hard_realtime();
    println!("Hard Realtime Platform:");
    println!("  Target FPS: {}", rt_config.performance.target_fps);
    println!("  Fusion: {:?}", rt_config.fusion.strategy);
    println!("  Loop Closure: {}", rt_config.enable_loop_closure);
    println!();

    // Example 3: Configure tracker with stabilization and track-first
    println!("Example 3: Configuring Tracker");
    println!("-------------------------------");
    
    let config = VIOPipelineConfig::balanced();
    
    // Create tracker
    let mut tracker = StereoPatchTracker::<3>::new(15, 30, 0.005);
    
    // Set camera intrinsics (example values)
    let fx = 458.654;
    let fy = 457.296;
    let cx = 367.215;
    let cy = 248.375;
    tracker.set_camera_intrinsics(fx, fy, cx, cy);
    
    // Enable frame stabilization if configured
    if config.fusion.strategy == FusionStrategy::RotationOnly {
        if let Some(stab_config) = config.fusion.stabilizer {
            println!("  ✓ Enabling frame stabilization");
            println!("    - Buffer size: {}", stab_config.buffer_size);
            println!("    - Accumulation weight: {}", stab_config.accumulation_weight);
            tracker.enable_frame_stabilization(stab_config, fx, fy, cx, cy);
        }
    }
    
    // Enable track-first detection if configured
    if config.detection.tracking_strategy == TrackingStrategy::TrackFirst {
        if let Some(tf_config) = config.detection.track_first {
            println!("  ✓ Enabling track-first detection");
            println!("    - Min features: {}", tf_config.min_features);
            println!("    - Max features: {}", tf_config.max_features);
            println!("    - Grid cell size: {}", tf_config.grid_cell_size);
            tracker.enable_track_first_detection(tf_config, 752, 480);
        }
    }
    println!();

    // Example 4: Custom configuration
    println!("Example 4: Custom Configuration");
    println!("--------------------------------");
    
    let mut custom_config = VIOPipelineConfig::balanced();
    
    // Adjust for specific requirements
    custom_config.performance.target_fps = 25.0;
    custom_config.fusion.num_frames = 4;
    
    if let Some(ref mut stab) = custom_config.fusion.stabilizer {
        stab.accumulation_weight = 0.8;
        stab.buffer_size = 4;
    }
    
    if let Some(ref mut tf) = custom_config.detection.track_first {
        tf.max_features = 250;
        tf.corner_quality_threshold = 0.008;
    }
    
    // Validate custom configuration
    custom_config.validate()?;
    println!("Custom configuration validated successfully");
    println!("  Target FPS: {}", custom_config.performance.target_fps);
    println!("  Fusion frames: {}", custom_config.fusion.num_frames);
    if let Some(tf) = custom_config.detection.track_first {
        println!("  Max features: {}", tf.max_features);
    }
    println!();

    // Save custom configuration
    let custom_path = PathBuf::from("my_custom_config.toml");
    custom_config.save_toml(&custom_path)?;
    println!("  ✓ Saved to {:?}", custom_path);
    println!();

    // Example 5: Platform comparison
    println!("Example 5: Platform Comparison");
    println!("-------------------------------");
    println!("{:<20} {:<10} {:<15} {:<15}", "Platform", "FPS", "Fusion", "Features");
    println!("{}", "-".repeat(60));
    
    let platforms = vec![
        ("CPU-Only", VIOPipelineConfig::cpu_only()),
        ("GPU-Enabled", VIOPipelineConfig::gpu_enabled()),
        ("Hard Realtime", VIOPipelineConfig::hard_realtime()),
        ("Balanced", VIOPipelineConfig::balanced()),
    ];
    
    for (name, config) in platforms {
        let features = config.detection.track_first
            .map(|tf| format!("{}-{}", tf.min_features, tf.max_features))
            .unwrap_or_else(|| "N/A".to_string());
        
        println!(
            "{:<20} {:<10.1} {:<15?} {:<15}",
            name,
            config.performance.target_fps,
            config.fusion.strategy,
            features
        );
    }
    println!();

    // Example 6: Module toggle pattern
    println!("Example 6: Module Toggles");
    println!("-------------------------");
    
    let mut config = VIOPipelineConfig::balanced();
    
    // Disable fusion for minimal latency
    println!("  Disabling fusion for minimal latency...");
    config.fusion.strategy = FusionStrategy::None;
    config.fusion.stabilizer = None;
    
    // Disable loop closure for realtime
    println!("  Disabling loop closure for realtime...");
    config.enable_loop_closure = false;
    
    // Disable bundle adjustment for speed
    println!("  Disabling bundle adjustment for speed...");
    config.enable_bundle_adjustment = false;
    
    println!("  Validating minimal latency configuration...");
    config.validate()?;
    println!("  ✓ Configuration valid for minimal latency mode");
    println!();

    println!("=== Demo Complete ===");
    Ok(())
}
