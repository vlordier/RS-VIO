#[cfg(test)]
mod tests {
    use crate::evaluation::metrics::calibration::CalibrationAwareAnalyzer;
    use crate::evaluation::metrics::distance_speed::DistanceSpeedAnalyzer;

    #[test]
    fn test_calibration_aware_metrics() {
        let mut analyzer = CalibrationAwareAnalyzer::new();

        // Simulate good calibration: weighted residuals significantly lower
        // Near field, static scene
        analyzer.record_weighted_visual_residual(0.5, 0.05, 0.1);
        analyzer.record_unweighted_visual_residual(0.5, 0.05, 0.25);

        // Far field, fast motion
        analyzer.record_weighted_visual_residual(8.0, 3.0, 0.4);
        analyzer.record_unweighted_visual_residual(8.0, 3.0, 0.8);

        let metrics = analyzer.analyze();

        // Weighted RMS should be significantly lower than unweighted
        assert!(metrics.weighted_residuals.visual_rms < metrics.unweighted_residuals.visual_rms);
        assert!(metrics.unweighted_residuals.visual_rms > 0.0);
    }

    #[test]
    fn test_track_survival_computation() {
        let mut analyzer = CalibrationAwareAnalyzer::new();

        // Record various track lengths
        for _ in 0..3 {
            analyzer.record_track_length(3); // 3 frames
        }
        for _ in 0..5 {
            analyzer.record_track_length(8); // 8 frames
        }
        for _ in 0..2 {
            analyzer.record_track_length(15); // 15 frames
        }

        let metrics = analyzer.analyze();
        let survival = &metrics.track_survival;

        assert_eq!(survival.total_features, 10);
        assert!(survival.median_track_length > 0.0);
        assert!(survival.survival_rate_5 > 0.0); // Some features last > 5 frames
        assert!(survival.survival_rate_10 > 0.0); // Some features last > 10 frames
    }

    #[test]
    fn test_distance_bin_improvements() {
        let mut analyzer = CalibrationAwareAnalyzer::new();

        // Near field: good improvement
        analyzer.record_weighted_visual_residual(0.5, 0.5, 0.1);
        analyzer.record_unweighted_visual_residual(0.5, 0.5, 0.3);

        // Far field: less improvement
        analyzer.record_weighted_visual_residual(12.0, 0.5, 0.5);
        analyzer.record_unweighted_visual_residual(12.0, 0.5, 0.6);

        let metrics = analyzer.analyze();
        let improvements = &metrics.distance_bin_improvements;

        // Near field should show improvement
        assert!(improvements.near_field.2 > 0.0); // Improvement %

        // Far field should show less improvement (or none)
        assert!(improvements.very_far_field.2 >= 0.0);

        // Weighted RMS should be less than unweighted in both bins
        assert!(improvements.near_field.0 < improvements.near_field.1);
    }

    #[test]
    fn test_speed_bin_improvements() {
        let mut analyzer = CalibrationAwareAnalyzer::new();

        // Static scene: good residuals
        analyzer.record_weighted_visual_residual(2.0, 0.05, 0.2);
        analyzer.record_unweighted_visual_residual(2.0, 0.05, 0.4);

        // Fast motion: worse residuals
        analyzer.record_weighted_visual_residual(2.0, 4.0, 0.4);
        analyzer.record_unweighted_visual_residual(2.0, 4.0, 0.7);

        let metrics = analyzer.analyze();
        let improvements = &metrics.speed_bin_improvements;

        // Both should show improvement
        assert!(improvements.static_scene.2 > 0.0);
        assert!(improvements.fast_motion.2 > 0.0);

        // Weighted RMS should be better than unweighted
        assert!(improvements.static_scene.0 < improvements.static_scene.1);
        assert!(improvements.fast_motion.0 < improvements.fast_motion.1);
    }

    #[test]
    fn test_outlier_detection() {
        let mut analyzer = CalibrationAwareAnalyzer::new();

        // Most good residuals (value 0.1)
        for _ in 0..9 {
            analyzer.record_weighted_visual_residual(2.0, 1.0, 0.1);
        }

        // One much larger outlier (10x the typical value)
        analyzer.record_weighted_visual_residual(2.0, 1.0, 1.0);

        let metrics = analyzer.analyze();

        // Should detect outlier rate (should be > 0% when there's a clear outlier)
        // With 10 samples, median is 0.1, 3x median threshold = 0.3
        // The 1.0 value exceeds this, so outlier rate should be 10%
        assert!(
            metrics.weighted_residuals.outlier_rate >= 5.0,
            "Expected outlier rate >= 5%, got {}",
            metrics.weighted_residuals.outlier_rate
        );
    }

    #[test]
    fn test_distance_binning() {
        let mut analyzer = DistanceSpeedAnalyzer::new();

        analyzer.record_depth_at_distance(0.5, 0.02);
        analyzer.record_depth_at_distance(2.0, 0.04);
        analyzer.record_depth_at_distance(5.0, 0.08);
        analyzer.record_depth_at_distance(15.0, 0.20);

        let analysis = analyzer.analyze();
        assert!(
            analysis.depth_by_distance.near_field.mean
                < analysis.depth_by_distance.very_far_field.mean
        );
    }

    #[test]
    fn test_speed_binning() {
        let mut analyzer = DistanceSpeedAnalyzer::new();

        analyzer.record_depth_at_speed(0.05, 0.01);
        analyzer.record_depth_at_speed(0.3, 0.02);
        analyzer.record_depth_at_speed(1.0, 0.04);
        analyzer.record_depth_at_speed(3.0, 0.08);
        analyzer.record_depth_at_speed(6.0, 0.15);

        let analysis = analyzer.analyze();
        assert!(
            analysis.depth_by_speed.static_scene.mean
                < analysis.depth_by_speed.very_fast_motion.mean
        );
    }
}
