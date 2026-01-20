// Integration tests for learned vibration scheduler

use rs_vio::imu::learned_vibration::{LearnedVibrationScheduler, VibrationInputs};
use rs_vio::imu::vibration_filter::VibrationNotchFilter;

#[test]
fn test_learned_scheduler_creation() {
    let mut scheduler = LearnedVibrationScheduler::new();
    let inputs = VibrationInputs {
        throttle: 0.5,
        vibration_level: 2.0,
        time_since_keyframe: 0.1,
        imu_rate: 200.0,
    };
    
    let outputs = scheduler.predict(&inputs);
    assert!(outputs.covariance_scale > 0.0);
    assert!(outputs.confidence >= 0.0 && outputs.confidence <= 1.0);
    assert!(outputs.filter_update_interval > 0);
}

#[test]
fn test_scheduler_with_vibration_filter() {
    let filter = VibrationNotchFilter::new(200.0, 512);
    let mut scheduler = LearnedVibrationScheduler::new().with_vibration_filter(filter);
    
    let inputs = VibrationInputs {
        throttle: 0.75,
        vibration_level: 3.5,
        time_since_keyframe: 0.05,
        imu_rate: 200.0,
    };
    
    let outputs = scheduler.predict(&inputs);
    
    // High vibration should increase covariance scale
    assert!(outputs.covariance_scale > 1.0);
}

#[test]
fn test_vibration_trend_detection() {
    let mut scheduler = LearnedVibrationScheduler::new();
    
    // Simulate increasing vibration trend
    for i in 0..20 {
        let vib_level = 1.0 + (i as f64) * 0.1; // Linearly increasing
        let inputs = VibrationInputs {
            throttle: 0.5,
            vibration_level: vib_level,
            time_since_keyframe: 0.1,
            imu_rate: 200.0,
        };
        
        scheduler.predict(&inputs);
    }
    
    // Latest prediction with high vibration
    let inputs = VibrationInputs {
        throttle: 0.5,
        vibration_level: 4.0,
        time_since_keyframe: 0.1,
        imu_rate: 200.0,
    };
    
    let outputs = scheduler.predict(&inputs);
    
    // Should adapt to increasing trend
    assert!(outputs.covariance_scale > 1.5);
}

#[test]
fn test_low_vibration_conditions() {
    let mut scheduler = LearnedVibrationScheduler::new();
    
    // Very low vibration (< 0.1)
    let inputs = VibrationInputs {
        throttle: 0.0,
        vibration_level: 0.05,
        time_since_keyframe: 0.1,
        imu_rate: 200.0,
    };
    
    let outputs = scheduler.predict(&inputs);
    
    // Very low vibration should have scale of 1.0 (base case)
    assert!(outputs.covariance_scale >= 1.0 && outputs.covariance_scale <= 1.5);
    assert!(outputs.confidence > 0.5);
}

#[test]
fn test_high_throttle_correlation() {
    let mut scheduler = LearnedVibrationScheduler::new();
    
    let high_throttle = VibrationInputs {
        throttle: 0.95,
        vibration_level: 3.0,
        time_since_keyframe: 0.1,
        imu_rate: 200.0,
    };
    
    let outputs_high = scheduler.predict(&high_throttle);
    
    let low_throttle = VibrationInputs {
        throttle: 0.2,
        vibration_level: 3.0,
        time_since_keyframe: 0.1,
        imu_rate: 200.0,
    };
    
    let outputs_low = scheduler.predict(&low_throttle);
    
    // High throttle typically correlates with more vibration uncertainty
    assert!(outputs_high.covariance_scale >= outputs_low.covariance_scale);
}

#[test]
fn test_time_since_keyframe_influence() {
    let mut scheduler = LearnedVibrationScheduler::new();
    
    let recent_keyframe = VibrationInputs {
        throttle: 0.5,
        vibration_level: 2.0,
        time_since_keyframe: 0.01, // Very recent
        imu_rate: 200.0,
    };
    
    let outputs_recent = scheduler.predict(&recent_keyframe);
    
    let old_keyframe = VibrationInputs {
        throttle: 0.5,
        vibration_level: 2.0,
        time_since_keyframe: 1.0, // Old
        imu_rate: 200.0,
    };
    
    let outputs_old = scheduler.predict(&old_keyframe);
    
    // Longer time since keyframe may increase uncertainty
    assert!(outputs_old.filter_update_interval >= outputs_recent.filter_update_interval);
}

#[test]
fn test_filter_update_interval_adaptation() {
    let mut scheduler = LearnedVibrationScheduler::new();
    
    // Low vibration (<= 0.3) gets 100ms interval (0.1 * imu_rate)
    let low_vib_inputs = VibrationInputs {
        throttle: 0.5,
        vibration_level: 0.2,
        time_since_keyframe: 0.1,
        imu_rate: 200.0,
    };
    
    let outputs_low = scheduler.predict(&low_vib_inputs);
    
    // High vibration (> 0.3) gets 50ms interval (0.05 * imu_rate)
    let high_vib_inputs = VibrationInputs {
        throttle: 0.5,
        vibration_level: 0.8,
        time_since_keyframe: 0.1,
        imu_rate: 200.0,
    };
    
    let outputs_high = scheduler.predict(&high_vib_inputs);
    
    // High vibration gets more frequent updates (smaller interval)
    assert!(outputs_high.filter_update_interval < outputs_low.filter_update_interval);
    // Specifically: 0.05*200=10 samples vs 0.1*200=20 samples
    assert_eq!(outputs_high.filter_update_interval, 10);
    assert_eq!(outputs_low.filter_update_interval, 20);
}

#[test]
fn test_confidence_degradation_with_uncertainty() {
    let mut scheduler = LearnedVibrationScheduler::new();
    
    // Stable conditions (low vibration and throttle)
    let stable = VibrationInputs {
        throttle: 0.3,
        vibration_level: 0.2,
        time_since_keyframe: 0.05,
        imu_rate: 200.0,
    };
    
    let outputs_stable = scheduler.predict(&stable);
    
    // Unstable conditions (extreme vibration > 0.8 AND throttle > 0.9)
    let unstable = VibrationInputs {
        throttle: 0.95,
        vibration_level: 0.85,
        time_since_keyframe: 0.5,
        imu_rate: 200.0,
    };
    
    let outputs_unstable = scheduler.predict(&unstable);
    
    // Confidence should be higher in stable conditions
    // Unstable triggers 0.8 multiplier due to extreme conditions
    assert!(outputs_stable.confidence > outputs_unstable.confidence);
}

#[test]
fn test_vibration_history_windowing() {
    let mut scheduler = LearnedVibrationScheduler::new();
    
    // Fill history beyond max capacity
    for i in 0..150 {
        let inputs = VibrationInputs {
            throttle: 0.5,
            vibration_level: (i % 10) as f64,
            time_since_keyframe: 0.1,
            imu_rate: 200.0,
        };
        scheduler.predict(&inputs);
    }
    
    // Should still produce valid outputs
    let final_inputs = VibrationInputs {
        throttle: 0.5,
        vibration_level: 2.5,
        time_since_keyframe: 0.1,
        imu_rate: 200.0,
    };
    
    let outputs = scheduler.predict(&final_inputs);
    assert!(outputs.covariance_scale > 0.0);
    assert!(outputs.confidence >= 0.0 && outputs.confidence <= 1.0);
}

#[test]
fn test_scheduler_reset() {
    let mut scheduler = LearnedVibrationScheduler::new();
    
    // Add some history
    for i in 0..30 {
        let inputs = VibrationInputs {
            throttle: 0.5,
            vibration_level: 1.0 + (i as f64) * 0.05,
            time_since_keyframe: 0.1,
            imu_rate: 200.0,
        };
        scheduler.predict(&inputs);
    }
    
    // Get confidence with history
    let inputs = VibrationInputs {
        throttle: 0.5,
        vibration_level: 0.3, // Low vibration to avoid confidence reduction
        time_since_keyframe: 0.1,
        imu_rate: 200.0,
    };
    let outputs_with_history = scheduler.predict(&inputs);
    
    // Reset scheduler
    scheduler.reset();
    
    // After reset, confidence should be lower (base 0.6, minimal history bonus)
    let outputs_after_reset = scheduler.predict(&inputs);
    
    // Reset state has lower confidence due to empty history
    assert!(outputs_after_reset.confidence < outputs_with_history.confidence);
    assert!(outputs_after_reset.confidence >= 0.5); // Should be at least base confidence
}
