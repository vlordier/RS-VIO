//! Phase 6: Feature Detection SOTA CLI Example
//!
//! Demonstrates track-first detect-to-fill, SuperPoint descriptors,
//! LightGlue matching, and adaptive feature distribution.

use rs_vio::feature_tracker::{
    DistributionConfig, FeatureDistributor, LightGlueConfig, LightGlueMatcher,
    SuperPointConfig, SuperPointDescriptor, TrackFirstConfig, TrackFirstDetector,
};
use nalgebra::Point2;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("═══════════════════════════════════════════════════════════");
    println!("  Phase 6: Feature Detection SOTA - CLI Example");
    println!("═══════════════════════════════════════════════════════════\n");

    // ─────────────────────────────────────────────────────────────
    // 1. Feature Distribution Setup
    // ─────────────────────────────────────────────────────────────
    println!("▶ Initializing Feature Distributor");
    println!("  Grid cell size: 32×32 pixels");
    println!("  Target features per cell: 2");

    let dist_config = DistributionConfig::default();
    let mut distributor = FeatureDistributor::new(dist_config);
    distributor.initialize(640, 480);

    println!("  ✓ Grid: {}×{}", distributor.grid_width(), distributor.grid_height());

    // ─────────────────────────────────────────────────────────────
    // 2. Track-First Detector Setup
    // ─────────────────────────────────────────────────────────────
    println!("\n▶ Initializing Track-First Detector");
    let track_config = TrackFirstConfig::default();
    let _detector = TrackFirstDetector::new(track_config, 640, 480);

    println!("  ✓ Min features: {}", track_config.min_features);
    println!("  ✓ Max features: {}", track_config.max_features);
    println!("  ✓ Grid cell size: {}", track_config.grid_cell_size);

    // ─────────────────────────────────────────────────────────────
    // 3. SuperPoint Descriptor Extractor Setup
    // ─────────────────────────────────────────────────────────────
    println!("\n▶ Initializing SuperPoint Descriptor Extractor");
    let sp_config = SuperPointConfig::default();
    let _sp_extractor = SuperPointDescriptor::new(sp_config);

    println!("  ✓ Descriptor dimension: {} ({}B per feature)", 
             sp_config.descriptor_size,
             sp_config.descriptor_size * 4);
    println!("  ✓ Keypoint threshold: {:.4}", sp_config.keypoint_threshold);
    println!("  ✓ Mode: {} (fallback to NN)",
             if sp_config.use_onnx { "ONNX" } else { "CPU" });

    // ─────────────────────────────────────────────────────────────
    // 4. LightGlue Matcher Setup
    // ─────────────────────────────────────────────────────────────
    println!("\n▶ Initializing LightGlue Matcher");
    let lg_config = LightGlueConfig::default();
    let _matcher = LightGlueMatcher::new(lg_config);

    println!("  ✓ Attention heads: {}", lg_config.num_heads);
    println!("  ✓ Transformer depth: {} layers", lg_config.depth);
    println!("  ✓ Match threshold: {:.2}", lg_config.match_threshold);
    println!("  ✓ Max matches: {}", lg_config.max_matches);
    println!("  ✓ Mode: {} (fallback to NN + ratio test)",
             if lg_config.use_flash_attention { "Flash-v2" } else { "Standard" });

    // ─────────────────────────────────────────────────────────────
    // 5. Demonstration: Simulated Feature Tracking
    // ─────────────────────────────────────────────────────────────
    println!("\n▶ Simulating Feature Tracking (Frame 0 → Frame 1)");
    println!();

    // Simulated tracked features
    let feature_positions = vec![
        Point2::new(100.0, 100.0),
        Point2::new(200.0, 150.0),
        Point2::new(300.0, 200.0),
        Point2::new(450.0, 250.0),
        Point2::new(550.0, 350.0),
    ];

    let pyramid_levels = vec![0, 0, 1, 1, 0];

    // Update distribution
    distributor.update_occupancy(&feature_positions, &pyramid_levels);
    let stats = distributor.stats();

    println!("  Features tracked: {}", stats.total_features);
    println!("  ├─ Critical cells (empty): {}", stats.critical_cells);
    println!("  ├─ Underoccupied cells: {}", stats.underoccupied_cells);
    println!("  ├─ Balanced cells: {}", stats.balanced_cells);
    println!("  └─ Overoccupied cells: {}", stats.overoccupied_cells);
    println!();
    println!("  Coverage: {:.1}%", stats.coverage_percentage);
    println!("  Uniformity: {:.3} (0=non-uniform, 1=perfect)", stats.uniformity);

    // Identify regions needing new detections
    let detection_regions = distributor.get_detection_regions();
    println!("\n  Detection needed in {} regions", detection_regions.len());
    if !detection_regions.is_empty() {
        println!("  Top 5 detection priority regions:");
        for (i, (cx, cy)) in detection_regions.iter().take(5).enumerate() {
            let threshold = distributor.get_adaptive_quality_threshold(*cx, *cy);
            println!("    {}. Cell({}, {}) - quality threshold: {:.4}",
                     i + 1, cx, cy, threshold);
        }
    }

    // Identify overcrowded regions
    let overcrowded = distributor.get_overcrowded_regions();
    if !overcrowded.is_empty() {
        println!("\n  {} regions are overcrowded (pruning candidates)", overcrowded.len());
    }

    // ─────────────────────────────────────────────────────────────
    // 6. Descriptor Matching Example
    // ─────────────────────────────────────────────────────────────
    println!("\n▶ Feature Descriptor Matching");

    // Create sample descriptors
    let query_desc = rs_vio::feature_tracker::KeypointDescriptor {
        position: Point2::new(100.0, 100.0),
        descriptor: vec![0.1; 256],
        confidence: 0.95,
        octave: 0,
        grid_index: (3, 3),
    };

    let ref_desc_1 = rs_vio::feature_tracker::KeypointDescriptor {
        position: Point2::new(102.0, 101.0),
        descriptor: vec![0.101; 256],
        confidence: 0.93,
        octave: 0,
        grid_index: (3, 3),
    };

    let ref_desc_2 = rs_vio::feature_tracker::KeypointDescriptor {
        position: Point2::new(200.0, 200.0),
        descriptor: vec![0.5; 256],
        confidence: 0.88,
        octave: 0,
        grid_index: (6, 6),
    };

    // Compute distances
    let dist_1 = query_desc.l2_distance(&ref_desc_1);
    let dist_2 = query_desc.l2_distance(&ref_desc_2);

    let sim_1 = query_desc.cosine_similarity(&ref_desc_1);
    let sim_2 = query_desc.cosine_similarity(&ref_desc_2);

    println!("  Query descriptor vs Reference 1 (nearby):");
    println!("    L2 distance: {:.6}", dist_1);
    println!("    Cosine similarity: {:.6}", sim_1);

    println!("\n  Query descriptor vs Reference 2 (far):");
    println!("    L2 distance: {:.6}", dist_2);
    println!("    Cosine similarity: {:.6}", sim_2);

    // Ratio test (Lowe's)
    let ratio = dist_1 / dist_2;
    let threshold = 0.7;
    let is_good_match = ratio < threshold;

    println!("\n  Lowe's Ratio Test:");
    println!("    Ratio: {:.3} / {:.3} = {:.3}", dist_1, dist_2, ratio);
    println!("    Threshold: {:.2}", threshold);
    println!("    ✓ Good match: {}", if is_good_match { "YES" } else { "NO" });

    // ─────────────────────────────────────────────────────────────
    // 7. Performance Summary
    // ─────────────────────────────────────────────────────────────
    println!("\n▶ Phase 6 Performance Estimates (640×480)");
    println!();
    println!("  Component                  | CPU (ms) | GPU (ms) | Notes");
    println!("  ───────────────────────────┼──────────┼──────────┼─────────────────────");
    println!("  ImagePyramid               |  5-10    |  2-4     | 4 levels");
    println!("  FeatureDistributor         |  1-2     |    -     | CPU only");
    println!("  TrackFirstDetector         | 15-30    | 10-20    | With CUDA support");
    println!("  SuperPointDescriptor       | 50-150   | 10-30    | ONNX fallback");
    println!("  LightGlueMatcher           |  1-5     | 20-50    | NN fallback");
    println!("  ───────────────────────────┼──────────┼──────────┼─────────────────────");
    println!("  Total (CPU)                | 70-200   |    -     | 5-15 FPS");
    println!("  Total (GPU)                |    -     | 40-100   | 10-25 FPS");

    // ─────────────────────────────────────────────────────────────
    // 8. Configuration Recommendations
    // ─────────────────────────────────────────────────────────────
    println!("\n▶ Recommended Configurations");
    println!();
    println!("  • Standard (desktop CPU)");
    println!("    - Use NN matcher (fallback)");
    println!("    - Reduce feature count via Distribution");
    println!("    - Expected: ~100-200ms/frame");
    println!();
    println!("  • GPU-Accelerated (NVIDIA)");
    println!("    - Use ONNX SuperPoint + LightGlue");
    println!("    - Batch processing for multiple frames");
    println!("    - Expected: ~50-100ms/frame");
    println!();
    println!("  • Embedded (Jetson Nano)");
    println!("    - Larger grid cells (48-64 px)");
    println!("    - Fewer features per cell");
    println!("    - Use NN matcher");
    println!("    - Expected: ~300-400ms/frame");

    println!("\n═══════════════════════════════════════════════════════════");
    println!("  Phase 6: Feature Detection SOTA - Complete");
    println!("═══════════════════════════════════════════════════════════");

    Ok(())
}
