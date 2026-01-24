/// Integration test for full SLAM pipeline
/// Tests GlobalPoseGraph integration and operations

#[test]
fn test_slam_estimator_initialization() {
    // Test that Estimator properly initializes GlobalPoseGraph
    use rs_vio::datasets::config::Config;
    use std::path::Path;

    // Try to load a default config file if it exists
    let config_path = Path::new("config.yaml");
    let config = if config_path.exists() {
        Config::load(config_path.to_str().unwrap()).expect("Failed to load config")
    } else {
        // For now, just test that we can check for config
        println!("[SLAMTest] No config file found, skipping full estimator test");
        return;
    };

    // Initialize estimator
    let estimator = rs_vio::estimator::Estimator::new(config, None);

    // Verify GlobalPoseGraph is initialized
    let gpg = &estimator.global_pose_graph;
    assert_eq!(gpg.num_poses(), 0, "GlobalPoseGraph should start empty");
    assert_eq!(gpg.num_loop_closures(), 0, "No loop closures initially");

    println!("[SLAMTest] Estimator initialized successfully with GlobalPoseGraph");
}

#[test]
fn test_global_pose_graph_basic_operations() {
    use rs_vio::estimator::global_pose_graph::{GlobalPoseGraph, GlobalPoseGraphConfig};

    // Create a graph with small thresholds for testing
    let config = GlobalPoseGraphConfig {
        closure_threshold: 2,
        max_poses_before_marginalization: 100,
        max_iterations: 10,
        cost_tolerance: 1e-7,
        enable_logging: true,
    };

    let graph = GlobalPoseGraph::new(config);

    // Verify initial state
    assert_eq!(graph.num_poses(), 0, "Should start with no keyframes");
    assert_eq!(
        graph.num_loop_closures(),
        0,
        "Should start with no closures"
    );

    // Check optimization trigger - should be false initially
    let (should_opt, _reason) = graph.should_optimize();
    assert!(!should_opt, "Should not trigger with no poses");
}

#[test]
fn test_global_pose_graph_closure_constraints() {
    use nalgebra as na;
    use rs_vio::estimator::global_pose_graph::{GlobalPoseGraph, GlobalPoseGraphConfig};
    use rs_vio::optimization::loop_closure::LoopClosureConstraint;

    let config = GlobalPoseGraphConfig {
        closure_threshold: 2,
        max_poses_before_marginalization: 100,
        max_iterations: 10,
        cost_tolerance: 1e-7,
        enable_logging: false,
    };

    let mut graph = GlobalPoseGraph::new(config);

    // Add some loop closure constraints without keyframes
    for _ in 0..3 {
        let constraint = LoopClosureConstraint {
            keyframe_id_1: 0,
            keyframe_id_2: 1,
            relative_pose: na::Isometry3::identity(),
            information_matrix: na::Matrix6::identity() * 100.0,
        };
        graph.add_loop_closure_constraint(constraint);
    }

    // Verify closures were added
    assert_eq!(graph.num_loop_closures(), 3, "Should have 3 loop closures");

    // Now should trigger optimization based on closure threshold
    let (should_opt, _reason) = graph.should_optimize();
    assert!(
        should_opt,
        "Should trigger optimization with 3 closures (threshold=2)"
    );

    // Test optimization (should fail gracefully since no keyframes)
    match graph.optimize() {
        Ok(result) => {
            println!(
                "[Test] Optimization (empty): {:.1}ms, {} iterations",
                result.optimization_time_ms, result.iterations
            );
            assert_eq!(
                graph.new_closures_since_last_opt, 0,
                "Closure counter should be reset after successful optimization"
            );
        },
        Err(e) => {
            // Expected when no keyframes
            println!("[Test] Expected error with no keyframes: {}", e);
            assert!(
                e.contains("No keyframes"),
                "Should error about no keyframes"
            );
            // Counter is NOT reset on error, only on success
            assert_eq!(
                graph.new_closures_since_last_opt, 3,
                "Closure counter should remain on failed optimization"
            );
        },
    }
}
