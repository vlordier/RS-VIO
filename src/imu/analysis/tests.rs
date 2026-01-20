//! Integration tests and performance benchmarks
//!
//! Comprehensive test suite for signal analysis including motor state
//! transitions, frequency estimation, and large dataset processing.

#[cfg(test)]
mod tests {
    use crate::imu::analysis::analyzer::ImuSignalAnalyzer;
    use crate::imu::analysis::config::MotorState;
    use crate::datasets::ImuData;

    #[test]
    fn test_motor_state_hysteresis() {
        let mut analyzer = ImuSignalAnalyzer::new(50);

        // Start with high vibration (motors on)
        for i in 0..40 {
            let vib = 2.5 * ((i % 10) as f64 / 10.0 - 0.5);
            analyzer.process_measurement(&ImuData {
                timestamp: (i * 5_000_000) as i64,
                accel: [vib, vib, -9.81],
                gyro: [0.0, 0.0, 0.0],
            });
        }

        // Should be running or transitioning
        let decomp1 = analyzer.decompose_harmonics();
        assert!(
            decomp1.quality.rms[0] > 0.5 || decomp1.quality.rms[1] > 0.5,
            "Should have significant vibration: {:?}",
            decomp1.quality.rms
        );

        // Reduce vibration to stationary
        for i in 40..80 {
            analyzer.process_measurement(&ImuData {
                timestamp: (i * 5_000_000) as i64,
                accel: [0.0, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
            });
        }

        // Should transition to Off with no vibration
        let decomp2 = analyzer.decompose_harmonics();
        assert_eq!(
            decomp2.motor_state,
            MotorState::Off,
            "Should transition to Off with no vibration"
        );
    }

    #[test]
    fn test_frequency_estimation_range() {
        let mut analyzer = ImuSignalAnalyzer::new(100);

        // Create periodic signal at ~250 Hz
        let sample_rate = 200.0; // 200 Hz
        let target_freq = 250.0; // Target frequency in signal

        for i in 0..100 {
            let t = i as f64 / sample_rate;
            // Simulated rotor vibration at target frequency
            let vibration = 2.0 * (2.0 * std::f64::consts::PI * target_freq * t).sin();

            analyzer.process_measurement(&ImuData {
                timestamp: (i as f64 * 5_000_000.0) as i64,
                accel: [vibration, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
            });
        }

        let decomp = analyzer.decompose_harmonics();

        // Frequency estimation might not be exact but should be in valid range
        if decomp.quality.fundamental_freq_hz > 0.0 {
            assert!(decomp.quality.fundamental_freq_hz >= 10.0);
            assert!(decomp.quality.fundamental_freq_hz <= 1000.0);
        }
    }

    #[test]
    fn test_custom_motor_threshold() {
        // Very sensitive threshold
        let mut analyzer_sensitive = ImuSignalAnalyzer::new_with_threshold(50, 0.2);

        for i in 0..50 {
            let small_vib = 0.3 * ((i % 5) as f64 / 5.0 - 0.5);
            analyzer_sensitive.process_measurement(&ImuData {
                timestamp: (i * 5_000_000) as i64,
                accel: [small_vib, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
            });
        }

        let decomp_sensitive = analyzer_sensitive.decompose_harmonics();
        assert!(
            decomp_sensitive.quality.rms[0] > 0.0,
            "Should have measured vibration"
        );

        // High threshold
        let mut analyzer_tolerant = ImuSignalAnalyzer::new_with_threshold(50, 2.0);

        for i in 0..50 {
            let small_vib = 0.3 * ((i % 5) as f64 / 5.0 - 0.5);
            analyzer_tolerant.process_measurement(&ImuData {
                timestamp: (i * 5_000_000) as i64,
                accel: [small_vib, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
            });
        }

        let state_tolerant = analyzer_tolerant.decompose_harmonics().motor_state;
        assert_eq!(
            state_tolerant,
            MotorState::Off,
            "Tolerant threshold should not detect small vibrations"
        );
    }

    #[test]
    fn test_harmonic_decomposition_motors_off() {
        let mut analyzer = ImuSignalAnalyzer::new(50);

        // Pure stationary signal
        for _ in 0..50 {
            analyzer.process_measurement(&ImuData {
                timestamp: 0,
                accel: [0.0, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
            });
        }

        let decomp = analyzer.decompose_harmonics();

        assert_eq!(
            decomp.motor_state,
            MotorState::Off,
            "Motor state should be Off"
        );

        assert!(
            decomp.quality.rms[0] < 0.5 && decomp.quality.rms[1] < 0.5,
            "Vibration should be minimal: {:?}",
            decomp.quality.rms
        );
    }

    #[test]
    fn test_harmonic_decomposition_motors_running() {
        let mut analyzer = ImuSignalAnalyzer::new(100);

        // Add strong periodic vibration
        for i in 0..100 {
            let vib = 3.0 * ((i % 10) as f64 / 10.0 - 0.5);
            analyzer.process_measurement(&ImuData {
                timestamp: (i * 5_000_000) as i64,
                accel: [vib, vib * 0.8, -9.81 + vib * 0.5],
                gyro: [0.0, 0.0, 0.0],
            });
        }

        let decomp = analyzer.decompose_harmonics();

        // Should detect motors running
        assert!(matches!(
            decomp.motor_state,
            MotorState::Running | MotorState::Transitioning
        ));

        // Residual harmonics should exist
        assert!(
            !decomp.residual_harmonics.is_empty(),
            "Should have residual harmonics"
        );
    }

    #[test]
    fn test_multi_axis_vibration() {
        let mut analyzer = ImuSignalAnalyzer::new(100);

        // Vibration on all axes
        for i in 0..100 {
            let vib_x = 1.0 * (i as f64 * 0.1).sin();
            let vib_y = 1.2 * (i as f64 * 0.15).sin();
            let vib_z = 0.8 * (i as f64 * 0.12).sin();

            analyzer.process_measurement(&ImuData {
                timestamp: (i * 5_000_000) as i64,
                accel: [vib_x, vib_y, -9.81 + vib_z],
                gyro: [0.0, 0.0, 0.0],
            });
        }

        let decomp = analyzer.decompose_harmonics();

        // Should detect as running
        assert!(matches!(
            decomp.motor_state,
            MotorState::Running | MotorState::Transitioning
        ));

        // Quality metrics should be computed for all axes
        for axis in 0..3 {
            assert!(decomp.quality.rms[axis] > 0.0);
            assert!(decomp.quality.snr[axis].is_finite());
        }
    }

    #[test]
    fn test_large_dataset_processing() {
        let mut analyzer = ImuSignalAnalyzer::new(100);

        // Process 1,000 samples
        for i in 0..1_000_u32 {
            analyzer.process_measurement(&ImuData {
                timestamp: (i as i64) * 5_000_000,
                accel: [0.0, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
            });
        }

        // Should complete without issues
        let decomp = analyzer.decompose_harmonics();
        assert!(decomp.quality.snr[0].is_finite());
    }

    #[test]
    fn test_rapid_state_transitions() {
        let mut analyzer = ImuSignalAnalyzer::new(50);

        // Alternate between high and low vibration
        for cycle in 0..5 {
            // High vibration
            for i in 0..30 {
                let vib = 1.5 * ((i % 5) as f64 / 5.0 - 0.5);
                analyzer.process_measurement(&ImuData {
                    timestamp: ((cycle * 60 + i) * 5_000_000) as i64,
                    accel: [vib, 0.0, -9.81],
                    gyro: [0.0, 0.0, 0.0],
                });
            }

            // Low vibration
            for i in 0..30 {
                analyzer.process_measurement(&ImuData {
                    timestamp: ((cycle * 60 + 30 + i) * 5_000_000) as i64,
                    accel: [0.01, 0.0, -9.81],
                    gyro: [0.0, 0.0, 0.0],
                });
            }
        }

        // Should handle transitions without crashing
        let decomp = analyzer.decompose_harmonics();
        assert!(decomp.quality.snr[0].is_finite());
    }
}
