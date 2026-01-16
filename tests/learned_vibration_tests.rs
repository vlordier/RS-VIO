//! Comprehensive tests for learned vibration scheduler
//! Tests happy paths, edge cases, training, and integration

use rs_vio::imu::learned_vibration::{
    LearnedVibrationScheduler, RuleBasedVibrationScheduler, VibrationInputs,
    VibrationTrainingSample,
};
use rs_vio::types::Float;

/// Test basic scheduler creation and default behavior
#[test]
fn test_learned_vibration_scheduler_creation() {
    let mut scheduler = LearnedVibrationScheduler::new();

    let inputs = VibrationInputs {
        throttle: 0.5,
        vibration_level: 0.2,
        time_since_keyframe: 0.5,
        imu_rate: 200.0,
    };

    let outputs = scheduler.predict(&inputs);

    // Should produce reasonable outputs
    assert!(outputs.covariance_scale >= 1.0);
    assert!(outputs.covariance_scale <= 10.0);
    assert!(outputs.confidence >= 0.0);
    assert!(outputs.confidence <= 1.0);
    assert!(outputs.filter_update_interval > 0);
}

/// Test rule-based fallback scheduler
#[test]
fn test_rule_based_scheduler() {
    let inputs = VibrationInputs {
        throttle: 0.8,
        vibration_level: 0.6,
        time_since_keyframe: 1.0,
        imu_rate: 200.0,
    };

    let outputs = RuleBasedVibrationScheduler::predict(&inputs);

    // High vibration + throttle should give high scale
    assert!(outputs.covariance_scale > 2.0);
    assert!(outputs.confidence < 0.7); // Lower confidence than learned
}

/// Test vibration history accumulation
#[test]
fn test_vibration_history_accumulation() {
    let mut scheduler = LearnedVibrationScheduler::new();

    let inputs = VibrationInputs {
        throttle: 0.5,
        vibration_level: 0.2,
        time_since_keyframe: 0.5,
        imu_rate: 200.0,
    };

    // Process multiple predictions to build history
    for _ in 0..150 {
        let _ = scheduler.predict(&inputs);
    }

    // Should have accumulated history
    assert!(scheduler.predict(&inputs).confidence > 0.5); // Higher confidence with history
}

/// Test training data integration
#[test]
fn test_training_data_integration() {
    let mut scheduler = LearnedVibrationScheduler::new();

    // Add some training samples
    let samples = vec![
        VibrationTrainingSample {
            inputs: VibrationInputs {
                throttle: 0.5,
                vibration_level: 0.2,
                time_since_keyframe: 0.5,
                imu_rate: 200.0,
            },
            optimal_scale: 2.0,
            performance_score: 0.9,
        },
        VibrationTrainingSample {
            inputs: VibrationInputs {
                throttle: 0.8,
                vibration_level: 0.6,
                time_since_keyframe: 1.0,
                imu_rate: 200.0,
            },
            optimal_scale: 4.0,
            performance_score: 0.8,
        },
    ];

    for sample in samples {
        scheduler.add_training_sample(sample);
    }

    // Test prediction with similar inputs
    let test_inputs = VibrationInputs {
        throttle: 0.6,
        vibration_level: 0.3,
        time_since_keyframe: 0.7,
        imu_rate: 200.0,
    };

    let outputs = scheduler.predict(&test_inputs);

    // Should produce reasonable scale (between 2.0 and 4.0)
    assert!(outputs.covariance_scale >= 2.0);
    assert!(outputs.covariance_scale <= 4.0);
}

/// Test extreme input conditions
#[test]
fn test_extreme_input_conditions() {
    let mut scheduler = LearnedVibrationScheduler::new();

    // Test with extreme vibration
    let extreme_inputs = VibrationInputs {
        throttle: 1.0,
        vibration_level: 1.0,
        time_since_keyframe: 5.0,
        imu_rate: 200.0,
    };

    let outputs = scheduler.predict(&extreme_inputs);

    // Should clamp outputs to reasonable ranges
    assert!(outputs.covariance_scale <= 10.0);
    assert!(outputs.covariance_scale >= 0.1);
    assert!(outputs.confidence >= 0.0);
    assert!(outputs.confidence <= 1.0);
}

/// Test zero and minimal inputs
#[test]
fn test_zero_minimal_inputs() {
    let mut scheduler = LearnedVibrationScheduler::new();

    let zero_inputs = VibrationInputs {
        throttle: 0.0,
        vibration_level: 0.0,
        time_since_keyframe: 0.0,
        imu_rate: 200.0,
    };

    let outputs = scheduler.predict(&zero_inputs);

    // Should still produce valid outputs
    assert!(outputs.covariance_scale >= 1.0);
    assert!(outputs.covariance_scale <= 10.0);
    assert!(outputs.confidence >= 0.0);
    assert!(outputs.confidence <= 1.0);
}

/// Test history reset behavior
#[test]
fn test_history_reset_behavior() {
    let mut scheduler = LearnedVibrationScheduler::new();

    let inputs = VibrationInputs {
        throttle: 0.5,
        vibration_level: 0.5,
        time_since_keyframe: 0.5,
        imu_rate: 200.0,
    };

    // Build history
    for _ in 0..50 {
        let _ = scheduler.predict(&inputs);
    }

    // Create new scheduler (simulate reset)
    let mut new_scheduler = LearnedVibrationScheduler::new();

    // Both should work but new one has lower confidence
    let old_outputs = scheduler.predict(&inputs);
    let new_outputs = new_scheduler.predict(&inputs);

    assert!(old_outputs.confidence > new_outputs.confidence);
}

/// Test training sample similarity matching
#[test]
fn test_training_sample_similarity() {
    let mut scheduler = LearnedVibrationScheduler::new();

    // Add a very specific training sample
    let training_sample = VibrationTrainingSample {
        inputs: VibrationInputs {
            throttle: 0.7,
            vibration_level: 0.4,
            time_since_keyframe: 0.8,
            imu_rate: 200.0,
        },
        optimal_scale: 3.5,
        performance_score: 0.95,
    };

    scheduler.add_training_sample(training_sample);

    // Test with identical inputs
    let identical_inputs = VibrationInputs {
        throttle: 0.7,
        vibration_level: 0.4,
        time_since_keyframe: 0.8,
        imu_rate: 200.0,
    };

    let outputs = scheduler.predict(&identical_inputs);

    // Should be very close to training sample
    assert!((outputs.covariance_scale - 3.5).abs() < 0.5);

    // Test with different inputs
    let different_inputs = VibrationInputs {
        throttle: 0.2,
        vibration_level: 0.1,
        time_since_keyframe: 0.2,
        imu_rate: 200.0,
    };

    let different_outputs = scheduler.predict(&different_inputs);

    // Should be different from training sample
    assert!((different_outputs.covariance_scale - 3.5).abs() > 0.5);
}

/// Test training data limits
#[test]
fn test_training_data_limits() {
    let mut scheduler = LearnedVibrationScheduler::new();

    // Add more than the limit (1000+ samples)
    for i in 0..1200 {
        let sample = VibrationTrainingSample {
            inputs: VibrationInputs {
                throttle: 0.5,
                vibration_level: 0.2 + (i as Float * 0.001),
                time_since_keyframe: 0.5,
                imu_rate: 200.0,
            },
            optimal_scale: 2.0,
            performance_score: 0.8,
        };
        scheduler.add_training_sample(sample);
    }

    let stats = scheduler.training_stats();
    assert!(stats.0 <= 1000); // Should not exceed limit
    assert!(stats.0 > 900); // But should keep most recent
}

/// Test adaptive filter update intervals
#[test]
fn test_adaptive_filter_intervals() {
    let mut scheduler = LearnedVibrationScheduler::new();

    // Low vibration should have longer intervals
    let low_vib_inputs = VibrationInputs {
        throttle: 0.2,
        vibration_level: 0.1,
        time_since_keyframe: 0.5,
        imu_rate: 200.0,
    };

    let low_outputs = scheduler.predict(&low_vib_inputs);
    assert!(low_outputs.filter_update_interval >= 10); // At least 10 samples

    // High vibration should have shorter intervals
    let high_vib_inputs = VibrationInputs {
        throttle: 0.8,
        vibration_level: 0.7,
        time_since_keyframe: 0.5,
        imu_rate: 200.0,
    };

    let high_outputs = scheduler.predict(&high_vib_inputs);
    assert!(high_outputs.filter_update_interval < low_outputs.filter_update_interval);
}

/// Test different IMU rates
#[test]
fn test_different_imu_rates() {
    let mut scheduler = LearnedVibrationScheduler::new();

    let rates = [100.0, 200.0, 400.0, 1000.0];

    for &rate in &rates {
        let inputs = VibrationInputs {
            throttle: 0.5,
            vibration_level: 0.3,
            time_since_keyframe: 0.5,
            imu_rate: rate,
        };

        let outputs = scheduler.predict(&inputs);

        // Update interval should scale with rate
        let expected_interval = if inputs.vibration_level > 0.3 {
            (rate * 0.05) as usize // 50ms at given rate
        } else {
            (rate * 0.1) as usize // 100ms at given rate
        };

        assert_eq!(outputs.filter_update_interval, expected_interval);
    }
}

/// Test vibration trend analysis
#[test]
fn test_vibration_trend_analysis() {
    let mut scheduler = LearnedVibrationScheduler::new();

    // Simulate increasing vibration trend
    let increasing_trend = vec![
        VibrationInputs {
            throttle: 0.5,
            vibration_level: 0.1,
            time_since_keyframe: 0.5,
            imu_rate: 200.0,
        },
        VibrationInputs {
            throttle: 0.5,
            vibration_level: 0.15,
            time_since_keyframe: 0.5,
            imu_rate: 200.0,
        },
        VibrationInputs {
            throttle: 0.5,
            vibration_level: 0.22,
            time_since_keyframe: 0.5,
            imu_rate: 200.0,
        },
        VibrationInputs {
            throttle: 0.5,
            vibration_level: 0.31,
            time_since_keyframe: 0.5,
            imu_rate: 200.0,
        },
    ];

    for inputs in increasing_trend {
        let outputs = scheduler.predict(&inputs);
        // Should be valid
        assert!(outputs.covariance_scale.is_finite());
        assert!(outputs.confidence.is_finite());
    }

    // Final prediction should account for increasing trend
    let final_inputs = VibrationInputs {
        throttle: 0.5,
        vibration_level: 0.35,
        time_since_keyframe: 0.5,
        imu_rate: 200.0,
    };

    let final_outputs = scheduler.predict(&final_inputs);
    assert!(final_outputs.covariance_scale > 1.5); // Should be elevated due to trend
}

/// Test integration with vibration filter
#[test]
fn test_integration_with_vibration_filter() {
    let mut scheduler = LearnedVibrationScheduler::new();

    // Create vibration filter
    let vibration_filter = rs_vio::imu::vibration_filter::VibrationNotchFilter::new(200.0, 512);

    // Integrate filter with scheduler
    scheduler = scheduler.with_vibration_filter(vibration_filter);

    let inputs = VibrationInputs {
        throttle: 0.6,
        vibration_level: 0.4,
        time_since_keyframe: 0.5,
        imu_rate: 200.0,
    };

    let outputs = scheduler.predict(&inputs);

    // Should have higher confidence with filter
    assert!(outputs.confidence > 0.7);

    // Should be able to access the filter
    assert!(scheduler.vibration_filter().is_some());
}

/// Test scheduler cloning and independence
#[test]
fn test_scheduler_cloning() {
    let mut scheduler1 = LearnedVibrationScheduler::new();
    let inputs = VibrationInputs {
        throttle: 0.5,
        vibration_level: 0.3,
        time_since_keyframe: 0.5,
        imu_rate: 200.0,
    };

    // Build history in scheduler1
    for _ in 0..10 {
        let _ = scheduler1.predict(&inputs);
    }

    // Scheduler2 should start fresh
    let mut scheduler2 = LearnedVibrationScheduler::new();

    let outputs1 = scheduler1.predict(&inputs);
    let outputs2 = scheduler2.predict(&inputs);

    // Should be different due to history
    assert!(outputs1.confidence > outputs2.confidence);
}

/// Test performance under load
#[test]
fn test_scheduler_performance() {
    let mut scheduler = LearnedVibrationScheduler::new();

    let inputs = VibrationInputs {
        throttle: 0.5,
        vibration_level: 0.3,
        time_since_keyframe: 0.5,
        imu_rate: 200.0,
    };

    let start = std::time::Instant::now();
    for _ in 0..1000 {
        let _ = scheduler.predict(&inputs);
    }
    let duration = start.elapsed();

    // Should be very fast (< 1ms per prediction)
    assert!(duration.as_millis() < 100);
}

/// Test training sample structure
#[test]
fn test_training_sample_structure() {
    let sample = VibrationTrainingSample {
        inputs: VibrationInputs {
            throttle: 0.6,
            vibration_level: 0.4,
            time_since_keyframe: 0.8,
            imu_rate: 200.0,
        },
        optimal_scale: 2.5,
        performance_score: 0.85,
    };

    assert_eq!(sample.inputs.throttle, 0.6);
    assert_eq!(sample.optimal_scale, 2.5);
    assert_eq!(sample.performance_score, 0.85);
}

/// Test scheduler statistics
#[test]
fn test_scheduler_statistics() {
    let mut scheduler = LearnedVibrationScheduler::new();

    // Initially empty
    let (count, avg_perf) = scheduler.training_stats();
    assert_eq!(count, 0);
    assert_eq!(avg_perf, 0.0);

    // Add samples
    scheduler.add_training_sample(VibrationTrainingSample {
        inputs: VibrationInputs {
            throttle: 0.5,
            vibration_level: 0.2,
            time_since_keyframe: 0.5,
            imu_rate: 200.0,
        },
        optimal_scale: 2.0,
        performance_score: 0.9,
    });

    scheduler.add_training_sample(VibrationTrainingSample {
        inputs: VibrationInputs {
            throttle: 0.5,
            vibration_level: 0.2,
            time_since_keyframe: 0.5,
            imu_rate: 200.0,
        },
        optimal_scale: 2.0,
        performance_score: 0.7,
    });

    let (count, avg_perf) = scheduler.training_stats();
    assert_eq!(count, 2);
    assert!((avg_perf - 0.8).abs() < 1e-6);
}
