#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use proptest::prelude::*;

// Property-based checks for numerical helpers
proptest! {
    #[test]
    fn safe_divide_matches_reference(num in -1.0e6f64..1.0e6f64, denom in -1.0e6f64..1.0e6f64) {
        prop_assume!(denom.abs() > 1.0e-6);
        let expected = num / denom;
        let result = rs_vio::validation::safe_divide(num, denom)
            .expect("safe_divide should succeed for valid inputs");

        // Allow small tolerance scaled by magnitudes
        let tol = 1e-9 * (1.0 + num.abs() + denom.abs());
        prop_assert!((result - expected).abs() <= tol);
    }

    #[test]
    fn safe_divide_rejects_nonfinite_inputs(num in prop_oneof![Just(f64::NAN), Just(f64::INFINITY), Just(f64::NEG_INFINITY)],
                                           denom in prop_oneof![Just(0.0_f64), Just(f64::NAN), Just(f64::INFINITY), Just(f64::NEG_INFINITY)]) {
        prop_assert!(rs_vio::validation::safe_divide(num, denom).is_err());
    }

    #[test]
    fn approx_equal_is_symmetric(a in -1.0e3f64..1.0e3f64, delta in -1.0e-6f64..1.0e-6f64) {
        let b = a + delta;
        let forward = rs_vio::validation::approx_equal(a, b);
        let backward = rs_vio::validation::approx_equal(b, a);
        prop_assert_eq!(forward, backward);
    }

    // Property-based tests for robustness components

    #[test]
    fn test_prosac_never_crashes(
        matches in prop::collection::vec(
            (any::<f32>(), any::<f32>(), any::<f32>(), any::<f32>(), 0.0..1.0f32),
            0..50
        )
    ) {
        let prosac_matches: Vec<_> = matches.into_iter()
            .map(|(x1, y1, x2, y2, q)| (
                nalgebra::Vector2::new(x1, y1),
                nalgebra::Vector2::new(x2, y2),
                q
            ))
            .collect();

        // Should not crash regardless of input
        let _result = rs_vio::feature_tracker::ransac::ProsacFundamental::estimate(&prosac_matches, 50, 0.99);
    }

    #[test]
    fn test_ransac_never_crashes(
        matches in prop::collection::vec(
            (any::<f32>(), any::<f32>(), any::<f32>(), any::<f32>()),
            0..50
        )
    ) {
        let ransac_matches: Vec<_> = matches.into_iter()
            .map(|(x1, y1, x2, y2)| (
                nalgebra::Vector2::new(x1, y1),
                nalgebra::Vector2::new(x2, y2)
            ))
            .collect();

        // Should not crash regardless of input
        let _result = rs_vio::feature_tracker::ransac::RansacFundamental::estimate(&ransac_matches, 1.0, 0.99);
    }

    #[test]
    fn test_vibration_filter_stability(
        gyro_data in prop::collection::vec(
            (-10.0..10.0f64, -10.0..10.0f64, -10.0..10.0f64),
            1..20
        )
    ) {
        let mut filter = rs_vio::imu::vibration_filter::VibrationNotchFilter::new(200.0, 256);

        for (gx, gy, gz) in gyro_data {
            let imu = rs_vio::datasets::ImuData {
                timestamp: 0,
                gyro: [gx, gy, gz],
                accel: [0.0, 0.0, 9.81],
            };

            let filtered = filter.process_measurement(&imu);

            // Outputs should always be finite
            prop_assert!(filtered.gyro[0].is_finite());
            prop_assert!(filtered.gyro[1].is_finite());
            prop_assert!(filtered.gyro[2].is_finite());
            prop_assert!(filtered.accel[0].is_finite());
            prop_assert!(filtered.accel[1].is_finite());
            prop_assert!(filtered.accel[2].is_finite());
        }
    }

    #[test]
    fn test_scheduler_bounded_outputs(
        throttle in 0.0..1.0f64,
        vibration_level in 0.0..2.0f64,
        time_since_keyframe in 0.0..10.0f64,
        imu_rate in 100.0..1000.0f64
    ) {
        let mut scheduler = rs_vio::imu::learned_vibration::LearnedVibrationScheduler::new();

        let inputs = rs_vio::imu::learned_vibration::VibrationInputs {
            throttle,
            vibration_level,
            time_since_keyframe,
            imu_rate,
        };

        let outputs = scheduler.predict(&inputs);

        // Outputs should be in valid ranges
        prop_assert!(outputs.covariance_scale >= 0.1);
        prop_assert!(outputs.covariance_scale <= 10.0);
        prop_assert!(outputs.confidence >= 0.0);
        prop_assert!(outputs.confidence <= 1.0);
        prop_assert!(outputs.filter_update_interval > 0);
        prop_assert!(outputs.filter_update_interval <= (imu_rate * 0.2) as usize);
    }

    #[test]
    fn test_rolling_shutter_feature_bounds(
        features in prop::collection::vec(
            (0.0..640.0f64, 0.0..480.0f64),
            1..10
        ),
        readout_time in 0.001..0.1f64
    ) {
        let compensator = rs_vio::camera::rolling_shutter::RollingShutterCompensator::new(readout_time, 480);

        let imu_data = vec![rs_vio::datasets::ImuData {
            timestamp: 1000000000,
            gyro: [0.1, 0.0, 0.0],
            accel: [0.0, 0.0, 9.81],
        }];

        let compensated = compensator.compensate_features(&features, &imu_data, 1000000000, &nalgebra::Matrix4::identity());

        // Should have same number of features
        prop_assert_eq!(compensated.len(), features.len());

        // All outputs should be finite
        for (u, v) in compensated {
            prop_assert!(u.is_finite());
            prop_assert!(v.is_finite());
        }
    }
}
