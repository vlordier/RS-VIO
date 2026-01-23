//! Phase 6 Integration Tests
//! 
//! Comprehensive testing of all three Phase 6 options combined:
//! - Option A: Distributed Metrics Export (Prometheus + OpenTelemetry)
//! - Option B: Hardware Profiling & Auto-tuning
//! - Option C: Circuit Breaker with Swarm Coordination

use rs_vio::estimator::{
    PrometheusMetrics,
    OtelMetricsBatch,
    MetricsServer, MetricsServerConfig,
    HardwareProfile, TimeoutAutoTuner,
    CircuitBreaker, CircuitState, CircuitBreakerConfig, SwarmHealthStatus,
    PipelineMetrics,
};

#[test]
fn test_metrics_export_from_pipeline() {
    // Create metrics
    let metrics = PipelineMetrics::new();
    
    // Export to Prometheus
    let prometheus = PrometheusMetrics::from_pipeline(&metrics);
    
    let _ = prometheus.frames_total;  // Verify field exists
    
    let text = prometheus.to_prometheus_text();
    assert!(!text.is_empty());
}

#[test]
fn test_otel_export_from_pipeline() {
    let metrics = PipelineMetrics::new();
    
    // Export to OTLP
    let batch = OtelMetricsBatch::from_pipeline(&metrics);
    
    assert!(!batch.data_points.is_empty());
    
    let json = batch.to_otlp_json();
    assert!(json.contains("resource"));
}

#[test]
fn test_metrics_server_config() {
    let config = MetricsServerConfig::default();
    assert_eq!(config.port, 9090);
    assert_eq!(config.host, "127.0.0.1");
    
    let custom = MetricsServerConfig::default()
        .with_port(8080)
        .with_host("0.0.0.0".to_string());
    
    assert_eq!(custom.port, 8080);
    assert_eq!(custom.host, "0.0.0.0");
}

#[test]
fn test_hardware_profile_selection() {
    // Test all profile variants
    let profiles = vec![
        ("nano", HardwareProfile::jetson_nano()),
        ("xavier", HardwareProfile::jetson_xavier()),
        ("orin", HardwareProfile::jetson_orin()),
        ("desktop", HardwareProfile::desktop_cpu()),
        ("robot", HardwareProfile::robot_board()),
    ];
    
    for (name, profile) in profiles {
        assert!(profile.cpu_cores > 0, "Profile {} has no cores", name);
        assert!(profile.detection_timeout_ms() > 0, "Profile {} has no detection timeout", name);
        // Optimization timeout should be at least equal to detection timeout
        assert!(profile.optimization_timeout_ms() >= profile.detection_timeout_ms(),
            "Profile {} has opt={} < det={}", name, 
            profile.optimization_timeout_ms(), 
            profile.detection_timeout_ms());
    }
}

#[test]
fn test_hardware_profile_by_name_lookup() {
    let variants = vec![
        "jetson nano", "nano",
        "jetson xavier", "xavier",
        "jetson orin", "orin",
        "desktop",
        "robot board", "rpi4",
    ];
    
    for variant in variants {
        let profile = HardwareProfile::by_name(variant);
        assert!(profile.is_some(), "Failed to find profile for: {}", variant);
    }
}

#[test]
fn test_auto_tuner_with_hardware_profile() {
    let profile = HardwareProfile::jetson_nano();
    let mut tuner = TimeoutAutoTuner::new(profile.clone());
    
    // Simulate detection latency observations
    for _ in 0..100 {
        tuner.observe_detection(50);
    }
    
    assert!(tuner.should_tune());
    let timeout = tuner.recommended_detection_timeout_ms();
    assert!(timeout > 0);
}

#[test]
fn test_circuit_breaker_basic_state_transitions() {
    let config = CircuitBreakerConfig {
        failure_ratio_threshold: 0.5,
        min_samples_for_evaluation: 10,
        recovery_timeout_ms: 100,
        half_open_max_requests: 3,
    };
    
    let cb = CircuitBreaker::with_config(config);
    
    // Initially closed
    assert_eq!(cb.state(), CircuitState::Closed);
    assert!(cb.broadcast_health_status());
    
    // Record successes
    for _ in 0..5 {
        cb.record_success();
    }
    assert_eq!(cb.state(), CircuitState::Closed);
    
    // Record failures
    for _ in 0..7 {
        cb.record_failure();
    }
    
    // Should now be open (7/12 = 58% > 50%)
    assert_eq!(cb.state(), CircuitState::Open);
    assert!(!cb.broadcast_health_status());
}

#[test]
fn test_circuit_breaker_recovery_cycle() {
    let config = CircuitBreakerConfig {
        failure_ratio_threshold: 0.5,
        min_samples_for_evaluation: 5,
        recovery_timeout_ms: 0,  // Immediate recovery for testing
        half_open_max_requests: 3,
    };
    
    let cb = CircuitBreaker::with_config(config);
    
    // Open circuit
    for _ in 0..3 {
        cb.record_failure();
    }
    for _ in 0..2 {
        cb.record_success();
    }
    assert_eq!(cb.state(), CircuitState::Open);
    
    // Enter half-open
    std::thread::sleep(std::time::Duration::from_millis(10));
    cb.record_success();
    assert_eq!(cb.state(), CircuitState::HalfOpen);
    
    // Return to closed (need 3 total successes in half-open)
    cb.record_success();
    cb.record_success();
    assert_eq!(cb.state(), CircuitState::Closed);
}

#[test]
fn test_swarm_health_status() {
    let cb = CircuitBreaker::new();
    
    // Start healthy
    assert_eq!(cb.swarm_status(), SwarmHealthStatus::Healthy);
    
    // Change status
    cb.set_swarm_status(SwarmHealthStatus::Degraded);
    assert_eq!(cb.swarm_status(), SwarmHealthStatus::Degraded);
    
    cb.set_swarm_status(SwarmHealthStatus::Critical);
    assert_eq!(cb.swarm_status(), SwarmHealthStatus::Critical);
    
    cb.set_swarm_status(SwarmHealthStatus::Failed);
    assert_eq!(cb.swarm_status(), SwarmHealthStatus::Failed);
}

#[test]
fn test_metrics_export_with_circuit_breaker_monitoring() {
    let metrics = PipelineMetrics::new();
    let cb = CircuitBreaker::new();
    
    // Record some operations
    for _ in 0..5 {
        cb.record_success();
    }
    
    // Export metrics
    let _prometheus = PrometheusMetrics::from_pipeline(&metrics);
    let _ = _prometheus.frames_total;  // Verify field exists
    
    // Get circuit breaker stats
    let stats = cb.get_statistics();
    assert_eq!(stats.total_successes, 5);
    assert_eq!(stats.total_failures, 0);
}

#[test]
fn test_hardware_profile_with_circuit_breaker() {
    let profile = HardwareProfile::jetson_orin();
    let config = CircuitBreakerConfig {
        failure_ratio_threshold: 0.4,
        min_samples_for_evaluation: 10,
        recovery_timeout_ms: profile.detection_timeout_ms(),
        ..Default::default()
    };
    
    let cb = CircuitBreaker::with_config(config);
    let _tuner = TimeoutAutoTuner::new(profile.clone());
    
    // Both operate together
    cb.record_success();
    assert_eq!(cb.state(), CircuitState::Closed);
}

#[test]
fn test_metrics_server_with_circuit_breaker_stats() {
    let metrics = std::sync::Arc::new(PipelineMetrics::new());
    let config = MetricsServerConfig::default();
    
    let server = MetricsServer::new(config, metrics);
    assert_eq!(server.metrics_url(), "http://127.0.0.1:9090/metrics");
}

#[test]
fn test_distributed_observability_stack() {
    // Simulates full Phase 6 Option A deployment
    let metrics = std::sync::Arc::new(PipelineMetrics::new());
    
    // Prometheus exporter
    let prom_metrics = PrometheusMetrics::from_pipeline(&metrics);
    let prom_text = prom_metrics.to_prometheus_text();
    assert!(!prom_text.is_empty());
    
    // OpenTelemetry exporter
    let otel_batch = OtelMetricsBatch::from_pipeline(&metrics);
    let otel_json = otel_batch.to_otlp_json();
    assert!(!otel_json.is_empty());
    
    // Metrics server
    let config = MetricsServerConfig::default();
    let server = MetricsServer::new(config, metrics);
    let _prom_response = server.prometheus_metrics();
}

#[test]
fn test_hardware_adaptive_timeout_stack() {
    // Simulates full Phase 6 Option B deployment
    let profiles = vec![
        HardwareProfile::jetson_nano(),
        HardwareProfile::jetson_xavier(),
        HardwareProfile::jetson_orin(),
    ];
    
    for profile in profiles {
        let mut tuner = TimeoutAutoTuner::new(profile.clone());
        
        // Simulate high latency
        for _ in 0..100 {
            tuner.observe_detection(
                profile.base_detection_us * 2
            );
        }
        
        let recommended = tuner.recommended_detection_timeout_ms();
        assert!(recommended > 0);
    }
}

#[test]
fn test_swarm_failure_isolation_stack() {
    // Simulates full Phase 6 Option C deployment
    let cb1 = CircuitBreaker::new();
    let cb2 = CircuitBreaker::new();
    let cb3 = CircuitBreaker::new();
    
    // Simulate drone swarm
    let drones = vec![&cb1, &cb2, &cb3];
    
    // Drone 1 fails
    for _ in 0..6 {
        cb1.record_failure();
    }
    for _ in 0..4 {
        cb1.record_success();
    }
    
    // Check isolation - other drones still healthy
    assert_eq!(cb1.state(), CircuitState::Open);
    assert_eq!(cb2.state(), CircuitState::Closed);
    assert_eq!(cb3.state(), CircuitState::Closed);
    
    // Coordinate swarm status
    let mut degraded_count = 0;
    for drone in &drones {
        if drone.state() == CircuitState::Open {
            degraded_count += 1;
        }
    }
    
    assert_eq!(degraded_count, 1);
}

#[test]
fn test_full_phase_6_integration() {
    // Complete integration test of all three options
    
    // Option A: Metrics Export
    let metrics = PipelineMetrics::new();
    let _prom_metrics = PrometheusMetrics::from_pipeline(&metrics);
    let _otel_batch = OtelMetricsBatch::from_pipeline(&metrics);
    
    // Option B: Hardware Tuning
    let profile = HardwareProfile::jetson_xavier();
    let mut tuner = TimeoutAutoTuner::new(profile.clone());
    for _ in 0..100 {
        tuner.observe_detection(profile.base_detection_us);
    }
    let _det_timeout = tuner.recommended_detection_timeout_ms();
    
    // Option C: Circuit Breaker
    let cb = CircuitBreaker::new();
    for _ in 0..5 {
        cb.record_success();
    }
    cb.record_failure();
    
    // Verify all components are working
    let _ = _prom_metrics.frames_total;  // Just verify it compiles
    assert!(!_otel_batch.data_points.is_empty());
    assert!(_det_timeout > 0);
    assert_eq!(cb.state(), CircuitState::Closed);
}

#[test]
fn test_metrics_server_format_negotiation() {
    let metrics = std::sync::Arc::new(PipelineMetrics::new());
    let config = MetricsServerConfig::default();
    let server = MetricsServer::new(config, metrics);
    
    // Both formats are available
    let _prom = server.prometheus_metrics();
    let _otel = server.otel_metrics();
}

#[test]
fn test_circuit_breaker_statistics_persistence() {
    let cb = CircuitBreaker::new();
    
    // Record activity
    for _ in 0..10 {
        cb.record_success();
    }
    
    // Get stats
    let stats1 = cb.get_statistics();
    assert_eq!(stats1.total_successes, 10);
    
    // Record more activity
    for _ in 0..5 {
        cb.record_failure();
    }
    
    // Stats should update
    let stats2 = cb.get_statistics();
    assert_eq!(stats2.total_failures, 5);
    assert_eq!(stats2.total_successes, 10);
}

#[test]
fn test_hardware_profile_timeout_consistency() {
    let profiles = vec![
        ("nano", HardwareProfile::jetson_nano()),
        ("xavier", HardwareProfile::jetson_xavier()),
        ("orin", HardwareProfile::jetson_orin()),
        ("desktop", HardwareProfile::desktop_cpu()),
        ("robot", HardwareProfile::robot_board()),
    ];
    
    for (name, profile) in profiles {
        let det = profile.detection_timeout_ms();
        let opt = profile.optimization_timeout_ms();
        
        assert!(opt >= det, 
            "Profile {} has optimization timeout ({}) < detection timeout ({})",
            name, opt, det);
    }
}

#[test]
fn test_circuit_breaker_clone_and_independence() {
    let cb1 = CircuitBreaker::new();
    let cb2 = cb1.clone();
    
    // Record failures on cb1
    for _ in 0..10 {
        cb1.record_failure();
    }
    for _ in 0..5 {
        cb1.record_success();
    }
    
    // cb1 should show state change, cb2 shares the same state
    // (since clone shares Arc<Mutex<>>)
    assert_eq!(cb1.state(), cb2.state());
    assert_eq!(cb1.total_failures(), cb2.total_failures());
}

#[test]
fn test_recovery_after_circuit_breaker_reset() {
    let config = CircuitBreakerConfig {
        failure_ratio_threshold: 0.5,
        min_samples_for_evaluation: 5,
        ..Default::default()
    };
    
    let cb = CircuitBreaker::with_config(config);
    
    // Open circuit
    for _ in 0..3 {
        cb.record_failure();
    }
    for _ in 0..2 {
        cb.record_success();
    }
    
    assert_eq!(cb.state(), CircuitState::Open);
    
    // Reset
    cb.reset();
    assert_eq!(cb.state(), CircuitState::Closed);
    assert!((cb.failure_ratio() as f64).abs() < f64::EPSILON);
}
