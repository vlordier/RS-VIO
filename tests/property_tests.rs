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
}
