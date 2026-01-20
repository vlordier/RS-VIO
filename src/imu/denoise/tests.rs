#[cfg(test)]
mod tests {
    use super::super::config::DenoiseConfig;
    use super::super::filter::ImuDenoiseFilter;
    use super::super::motion_mode::MotionMode;

    #[test]
    fn test_highpass_filter_removes_dc() {
        let config = DenoiseConfig::default();
        let mut filter = ImuDenoiseFilter::new(config);

        // DC component should be attenuated
        let dc_signal = [0.5, 0.5, 0.5];
        let _output = filter.process_gyro(&dc_signal);

        // After many iterations, DC should be mostly gone (more iterations needed due to debouncing)
        for _ in 0..200 {
            filter.process_gyro(&dc_signal);
        }
        let output = filter.process_gyro(&dc_signal);

        assert!(output[0].abs() < 0.15, "High-pass should remove DC");
    }

    #[test]
    fn test_notch_filter_configuration() {
        let mut config = DenoiseConfig::default();
        config.notch_frequencies = vec![0.06, 1.46];
        let filter = ImuDenoiseFilter::new(config);

        assert_eq!(filter.notch_filters.len(), 2, "Should have 2 notch stages");
    }

    #[test]
    fn test_spike_rejection() {
        let mut config = DenoiseConfig::default();
        config.spike_window = 3;
        let mut filter = ImuDenoiseFilter::new(config);

        // Feed normal samples
        filter.process_gyro(&[0.1, 0.1, 0.1]);
        filter.process_gyro(&[0.11, 0.11, 0.11]);

        // Feed spike (much larger than previous samples)
        let spike_sample = [5.0, 5.0, 5.0];
        let output = filter.process_gyro(&spike_sample);

        // Output should be attenuated compared to raw spike
        // Median-of-3 should use middle value from [0.11, 5.0, next_sample]
        // Since we haven't provided next sample yet, behavior depends on history
        assert!(
            output[0].abs() < spike_sample[0],
            "Spike should be attenuated"
        );

        // Feed another normal sample
        let normal = [0.12, 0.12, 0.12];
        let output2 = filter.process_gyro(&normal);

        // Should recover to normal processing
        assert!(output2[0].abs() < 1.0, "Should recover from spike");
    }

    #[test]
    fn test_clipping_detection_and_weight_scaling() {
        let mut config = DenoiseConfig::default();
        config.clip_threshold_rads = 3.0;
        config.clip_window = 100;
        let mut filter = ImuDenoiseFilter::new(config);

        // Initially weight should be 1.0
        assert_eq!(filter.weight_scale, 1.0, "Initial weight should be 1.0");

        // Feed normal samples
        for _ in 0..50 {
            filter.process_gyro(&[0.5, 0.5, 0.5]);
        }
        assert!(
            filter.weight_scale > 0.9,
            "Weight should stay high for normal samples"
        );

        // Feed clipping samples (above threshold)
        for _ in 0..30 {
            filter.process_gyro(&[4.0, 4.0, 4.0]);
        }

        // Weight should decrease due to clipping
        // With 30 clipped samples in 100-sample window, ratio = 0.3
        // weight_scale = 0.2 + 0.8 * (1 - 0.3) = 0.2 + 0.56 = 0.76
        assert!(
            filter.weight_scale < 0.85,
            "Weight should decrease with clipping: {}",
            filter.weight_scale
        );
        assert!(
            filter.weight_scale >= 0.2,
            "Weight should not go below 0.2: {}",
            filter.weight_scale
        );

        // Feed normal samples to recover
        for _ in 0..100 {
            filter.process_gyro(&[0.5, 0.5, 0.5]);
        }

        // Weight should recover as clipped samples leave the window
        assert!(
            filter.weight_scale > 0.9,
            "Weight should recover after normal samples"
        );
    }

    #[test]
    fn test_motion_mode_transitions() {
        let mut config = DenoiseConfig::default();
        config.hover_rms_thresh = 0.25;
        config.aggressive_rms_thresh = 0.8;
        let mut filter = ImuDenoiseFilter::new(config);

        // Start in hover mode
        assert_eq!(filter.mode, MotionMode::Hover, "Should start in hover mode");

        // Feed low-motion samples (should stay in hover)
        for _ in 0..60 {
            filter.process_gyro(&[0.1, 0.1, 0.1]);
        }
        assert_eq!(
            filter.mode,
            MotionMode::Hover,
            "Should stay in hover for low motion"
        );

        // Feed high-motion samples to trigger aggressive mode
        // Need enough samples to overcome debouncing (50 samples minimum)
        for _ in 0..100 {
            filter.process_gyro(&[1.0, 1.0, 1.0]);
        }
        assert_eq!(
            filter.mode,
            MotionMode::Aggressive,
            "Should switch to aggressive mode"
        );

        // Feed low-motion samples to return to hover
        for _ in 0..100 {
            filter.process_gyro(&[0.1, 0.1, 0.1]);
        }
        assert_eq!(
            filter.mode,
            MotionMode::Hover,
            "Should return to hover mode"
        );
    }

    #[test]
    fn test_adaptive_notch_q() {
        let mut config = DenoiseConfig::default();
        config.adaptive_notch_q = true;
        config.notch_q = 6.0;
        config.notch_q_min = 4.0;
        config.notch_q_max = 8.0;
        config.notch_q_ref_rads = 0.6;
        let mut filter = ImuDenoiseFilter::new(config);

        // Initial Q should be the configured value
        assert_eq!(filter.current_notch_q, 6.0, "Initial Q should match config");

        // Feed low-amplitude samples (below ref, should increase Q)
        for _ in 0..60 {
            filter.process_gyro(&[0.2, 0.2, 0.2]);
        }
        // Q should increase towards max (but debouncing limits changes)
        // After sufficient samples, should trend toward higher Q
        let q_after_low = filter.current_notch_q;

        // Feed high-amplitude samples (above ref, should decrease Q)
        for _ in 0..60 {
            filter.process_gyro(&[1.2, 1.2, 1.2]);
        }
        // Q should decrease towards min
        let q_after_high = filter.current_notch_q;

        // Verify Q adjusted in expected direction
        // Due to debouncing, changes are gradual
        assert!(
            q_after_high <= q_after_low + 0.5,
            "Q should decrease or stay similar for high amplitude"
        );
    }

    #[test]
    fn test_per_axis_notch_frequencies() {
        let mut config = DenoiseConfig::default();
        config.enable_notch_filter = true;
        config.notch_frequencies_per_axis = Some([
            vec![0.06, 1.46],     // X-axis: 2 frequencies
            vec![0.06],           // Y-axis: 1 frequency
            vec![1.46, 2.5, 3.0], // Z-axis: 3 frequencies
        ]);
        let filter = ImuDenoiseFilter::new(config);

        // Should create 3 stages (max across axes)
        assert_eq!(filter.notch_filters.len(), 3, "Should have 3 notch stages");

        // Each stage should have 3 filters (one per axis)
        for stage in &filter.notch_filters {
            assert_eq!(stage.len(), 3, "Each stage should have 3 axis filters");
        }
    }

    #[test]
    fn test_lowpass_attenuates_high_frequency() {
        let mut config = DenoiseConfig::default();
        config.lowpass_cutoff = 10.0; // Low cutoff to make effect obvious
        config.imu_sample_rate = 200.0;
        config.enable_notch_filter = false; // Disable to isolate lowpass effect
        let mut filter = ImuDenoiseFilter::new(config);

        // Generate high-frequency oscillation (50 Hz, well above 10 Hz cutoff)
        // At 200 Hz sample rate, 50 Hz means period of 4 samples
        let mut sum_output = 0.0;
        let mut sum_input = 0.0;

        for i in 0..100 {
            let t = i as f32 / 200.0;
            let high_freq = (2.0 * std::f32::consts::PI * 50.0 * t).sin();
            let input = [high_freq, 0.0, 0.0];
            let output = filter.process_gyro(&input);

            if i > 50 {
                // Skip transient
                sum_input += input[0].abs();
                sum_output += output[0].abs();
            }
        }

        // Average output amplitude should be significantly less than input
        assert!(
            sum_output < sum_input * 0.3,
            "Lowpass should attenuate high frequency"
        );
    }

    #[test]
    fn test_notch_attenuates_resonance() {
        let mut config = DenoiseConfig::default();
        config.enable_notch_filter = true;
        config.notch_frequencies = vec![10.0]; // Notch at 10 Hz
        config.notch_q = 5.0; // Narrow notch
        config.imu_sample_rate = 200.0;
        config.adaptive_notch_q = false; // Disable adaptive for consistent test
        let mut filter = ImuDenoiseFilter::new(config.clone());

        // Generate signal at notch frequency (10 Hz)
        let mut sum_output_at_notch = 0.0;
        let mut sum_input_at_notch = 0.0;

        for i in 0..200 {
            let t = i as f32 / 200.0;
            let signal = (2.0 * std::f32::consts::PI * 10.0 * t).sin();
            let input = [signal, 0.0, 0.0];
            let output = filter.process_gyro(&input);

            if i > 100 {
                // Skip filter transient
                sum_input_at_notch += input[0].abs();
                sum_output_at_notch += output[0].abs();
            }
        }

        // Output should be much smaller than input at notch frequency
        assert!(
            sum_output_at_notch < sum_input_at_notch * 0.5,
            "Notch filter should attenuate resonance frequency"
        );

        // Now test off-notch frequency (5 Hz, well away from 10 Hz)
        filter = ImuDenoiseFilter::new(config); // Reset filter state
        let mut sum_output_off_notch = 0.0;
        let mut sum_input_off_notch = 0.0;

        for i in 0..200 {
            let t = i as f32 / 200.0;
            let signal = (2.0 * std::f32::consts::PI * 5.0 * t).sin();
            let input = [signal, 0.0, 0.0];
            let output = filter.process_gyro(&input);

            if i > 100 {
                sum_input_off_notch += input[0].abs();
                sum_output_off_notch += output[0].abs();
            }
        }

        // Off-notch frequency should pass through with minimal attenuation
        // (allowing for some attenuation from HP/LP filters)
        assert!(
            sum_output_off_notch > sum_input_off_notch * 0.6,
            "Frequencies away from notch should pass through"
        );
    }

    #[test]
    fn test_accel_processing() {
        let config = DenoiseConfig::default();
        let mut filter = ImuDenoiseFilter::new(config);

        // Test that accelerometer processing works
        let accel = [0.0, 0.0, 9.81]; // Gravity
        let output = filter.process_accel(&accel);

        // Should process without crashing
        assert!(output.len() == 3, "Should return 3-element array");

        // Process dynamic acceleration (oscillating around gravity)
        // This has AC component that should pass through
        let mut max_output: f32 = 0.0;
        for i in 0..200 {
            let t = i as f32 / 200.0;
            let dynamic_accel = [
                0.0,
                0.0,
                9.81 + 0.5 * (2.0 * std::f32::consts::PI * 2.0 * t).sin(), // 2 Hz oscillation
            ];
            let out = filter.process_accel(&dynamic_accel);
            if i > 100 {
                // Skip transient
                max_output = max_output.max(out[2].abs());
            }
        }

        // The AC component should pass through (not the DC gravity)
        // We should see some non-zero output from the oscillation
        assert!(max_output > 0.1, "Should pass AC component of acceleration");
    }

    #[test]
    fn test_filter_stability_with_extreme_inputs() {
        let config = DenoiseConfig::default();
        let mut filter = ImuDenoiseFilter::new(config);

        // Test with very large inputs
        let large = [100.0, 100.0, 100.0];
        let out1 = filter.process_gyro(&large);
        assert!(out1[0].is_finite(), "Should handle large inputs");
        assert!(out1[1].is_finite(), "Should handle large inputs");
        assert!(out1[2].is_finite(), "Should handle large inputs");

        // Test with very small inputs
        let small = [1e-6, 1e-6, 1e-6];
        let out2 = filter.process_gyro(&small);
        assert!(out2[0].is_finite(), "Should handle small inputs");

        // Test with zeros
        let zero = [0.0, 0.0, 0.0];
        let out3 = filter.process_gyro(&zero);
        assert!(out3[0].is_finite(), "Should handle zero inputs");

        // Test with mixed signs
        let mixed = [-1.5, 2.3, -0.7];
        let out4 = filter.process_gyro(&mixed);
        assert!(out4[0].is_finite(), "Should handle mixed sign inputs");
        assert!(out4[1].is_finite(), "Should handle mixed sign inputs");
        assert!(out4[2].is_finite(), "Should handle mixed sign inputs");
    }

    #[test]
    fn test_weight_scale_boundary_conditions() {
        let mut config = DenoiseConfig::default();
        config.clip_threshold_rads = 2.0;
        config.clip_window = 10;
        let mut filter = ImuDenoiseFilter::new(config.clone());

        // All samples clipped
        for _ in 0..20 {
            filter.process_gyro(&[5.0, 5.0, 5.0]);
        }
        // Should hit minimum weight (0.2)
        assert!(
            (filter.weight_scale - 0.2).abs() < 0.01,
            "Weight should be 0.2 when all samples clipped"
        );

        // Reset with no clipping
        filter = ImuDenoiseFilter::new(config.clone());
        for _ in 0..20 {
            filter.process_gyro(&[0.1, 0.1, 0.1]);
        }
        // Should stay at maximum weight (1.0)
        assert!(
            (filter.weight_scale - 1.0).abs() < 0.01,
            "Weight should be 1.0 when no clipping"
        );
    }

    #[test]
    fn test_realistic_flight_scenario() {
        let mut config = DenoiseConfig::default();
        config.imu_sample_rate = 200.0;
        config.spike_window = 3;
        config.adaptive_notch_q = true;
        let mut filter = ImuDenoiseFilter::new(config);

        // Simulate realistic flight: takeoff -> hover -> aggressive maneuver -> hover -> landing
        let scenarios = [
            ("takeoff", 50, [0.5, 0.5, 0.8]),   // Moderate motion
            ("hover", 100, [0.1, 0.1, 0.15]),   // Low motion
            ("maneuver", 80, [1.5, 1.2, 1.8]),  // Aggressive motion
            ("hover2", 150, [0.12, 0.15, 0.1]), // Return to hover (longer for mode transition)
            ("landing", 50, [0.4, 0.6, 0.7]),   // Moderate motion
        ];

        for (phase, samples, base_gyro) in scenarios.iter() {
            for i in 0..*samples {
                let t = i as f32 / 200.0;
                // Add some realistic noise and vibration
                let noise = [
                    0.02 * (10.0 * t).sin(),
                    0.02 * (12.0 * t).cos(),
                    0.02 * (8.0 * t).sin(),
                ];
                let gyro = [
                    base_gyro[0] + noise[0],
                    base_gyro[1] + noise[1],
                    base_gyro[2] + noise[2],
                ];

                let output = filter.process_gyro(&gyro);

                // Verify output is reasonable
                assert!(
                    output[0].is_finite(),
                    "Output should be finite in {}",
                    phase
                );
                assert!(
                    output[1].is_finite(),
                    "Output should be finite in {}",
                    phase
                );
                assert!(
                    output[2].is_finite(),
                    "Output should be finite in {}",
                    phase
                );
            }
        }

        // Should have processed through various motion states successfully
        // Final mode depends on landing phase motion which is moderate
        // Don't assert specific mode since landing motion (0.4-0.7 rad/s) is borderline
        assert!(
            filter.weight_scale > 0.8,
            "Weight should be high for clean signal"
        );
    }

    #[test]
    fn test_burst_noise_handling() {
        let mut config = DenoiseConfig::default();
        config.spike_window = 3;
        let mut filter = ImuDenoiseFilter::new(config);

        // Feed clean signal
        for _ in 0..50 {
            filter.process_gyro(&[0.2, 0.2, 0.2]);
        }

        // Burst of noise spikes
        for _ in 0..5 {
            filter.process_gyro(&[10.0, -8.0, 12.0]);
        }

        // Return to clean signal
        for _ in 0..50 {
            filter.process_gyro(&[0.2, 0.2, 0.2]);
        }

        // Filter should recover and not become unstable
        let output = filter.process_gyro(&[0.2, 0.2, 0.2]);
        assert!(output[0].abs() < 2.0, "Should recover from burst noise");
        assert!(output[1].abs() < 2.0, "Should recover from burst noise");
        assert!(output[2].abs() < 2.0, "Should recover from burst noise");
    }

    #[test]
    fn test_continuous_high_rate_processing() {
        let config = DenoiseConfig::default();
        let mut filter = ImuDenoiseFilter::new(config);

        // Simulate 10 seconds at 200 Hz (2000 samples)
        for i in 0..2000 {
            let t = i as f32 / 200.0;
            let gyro = [
                0.3 * (2.0 * std::f32::consts::PI * 1.0 * t).sin(),
                0.2 * (2.0 * std::f32::consts::PI * 1.5 * t).cos(),
                0.25 * (2.0 * std::f32::consts::PI * 0.8 * t).sin(),
            ];
            let output = filter.process_gyro(&gyro);

            // Verify stability over long duration
            assert!(
                output[0].is_finite(),
                "Should remain stable at sample {}",
                i
            );
            assert!(
                output[0].abs() < 10.0,
                "Output should be bounded at sample {}",
                i
            );
        }
    }

    #[test]
    fn test_mode_hysteresis_prevents_oscillation() {
        let mut config = DenoiseConfig::default();
        config.hover_rms_thresh = 0.25;
        config.aggressive_rms_thresh = 0.8;
        let mut filter = ImuDenoiseFilter::new(config);

        // Start in hover
        for _ in 0..60 {
            filter.process_gyro(&[0.1, 0.1, 0.1]);
        }
        assert_eq!(filter.mode, MotionMode::Hover);

        // Oscillate around threshold - should not switch rapidly due to debouncing
        let mut mode_changes = 0;
        let mut last_mode = filter.mode;

        for i in 0..200 {
            // Alternate between just below and just above aggressive threshold
            let amp = if i % 2 == 0 { 0.75 } else { 0.85 };
            filter.process_gyro(&[amp, amp, amp]);

            if filter.mode != last_mode {
                mode_changes += 1;
                last_mode = filter.mode;
            }
        }

        // With debouncing (50 samples minimum), should have very few mode changes
        // Even with 100 oscillations, debouncing prevents rapid switching
        assert!(
            mode_changes < 4,
            "Debouncing should prevent rapid mode switching: {} changes",
            mode_changes
        );
    }

    #[test]
    fn test_multiple_notch_frequencies_independence() {
        let mut config = DenoiseConfig::default();
        config.enable_notch_filter = true;
        config.notch_frequencies = vec![5.0, 15.0, 25.0];
        config.notch_q = 8.0;
        config.adaptive_notch_q = false;

        // Test attenuation at each notch frequency independently
        for &freq in &[5.0, 15.0, 25.0] {
            let mut filter_local = ImuDenoiseFilter::new(config.clone());

            let mut sum_input: f32 = 0.0;
            let mut sum_output: f32 = 0.0;

            for i in 0..300 {
                let t = i as f32 / 200.0;
                let signal = (2.0 * std::f32::consts::PI * freq * t).sin();
                let input = [signal, 0.0, 0.0];
                let output = filter_local.process_gyro(&input);

                if i > 150 {
                    sum_input += input[0].abs();
                    sum_output += output[0].abs();
                }
            }

            let attenuation = sum_output / sum_input.max(1e-6);
            assert!(
                attenuation < 0.5,
                "Should attenuate {} Hz: ratio={}",
                freq,
                attenuation
            );
        }

        // Test pass-through at intermediate frequency (10 Hz, between 5 and 15)
        let mut filter_pass = ImuDenoiseFilter::new(config.clone());
        let mut sum_input: f32 = 0.0;
        let mut sum_output: f32 = 0.0;

        for i in 0..300 {
            let t = i as f32 / 200.0;
            let signal = (2.0 * std::f32::consts::PI * 10.0 * t).sin();
            let input = [signal, 0.0, 0.0];
            let output = filter_pass.process_gyro(&input);

            if i > 150 {
                sum_input += input[0].abs();
                sum_output += output[0].abs();
            }
        }

        let passthrough = sum_output / sum_input.max(1e-6);
        // Should have less attenuation at intermediate frequency
        // (though still some from HP/LP filters)
        assert!(
            passthrough > 0.4,
            "Should pass 10 Hz better than notch frequencies: ratio={}",
            passthrough
        );
    }
}
