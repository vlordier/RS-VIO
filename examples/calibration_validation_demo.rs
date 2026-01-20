/// Calibration validation demonstration
///
/// Shows how to:
/// 1. Record calibration metrics
/// 2. Validate against acceptance thresholds
/// 3. Generate quality reports
/// 4. Make production deployment decisions
use rs_vio::calibration::CalibrationAcceptanceValidator;

/// Example 1: Validate excellent camera calibration
fn example_excellent_calibration() {
    println!("\n{}", "=".repeat(70));
    println!("EXAMPLE 1: Excellent Camera + Stereo Calibration");
    println!("{}", "=".repeat(70));

    let mut validator = CalibrationAcceptanceValidator::new();

    // Record excellent calibration results
    validator.record_metric("reprojection_rms", 0.25); // Excellent (< 0.3px)
    validator.record_metric("vertical_disparity_rms", 0.12); // Excellent
    validator.record_metric("epipolar_residual", 0.20); // Excellent
    validator.record_metric("timing_observability", 0.85); // Excellent
    validator.record_metric("time_offset_jitter", 0.3e-3); // Excellent (0.3ms)

    // Generate report
    let report = validator.validate();
    report.print_report();

    println!("\n✓ Decision: READY FOR DEPLOYMENT");
}

/// Example 2: Validate marginal calibration
fn example_marginal_calibration() {
    println!("\n{}", "=".repeat(70));
    println!("EXAMPLE 2: Marginal Calibration (Needs Improvement)");
    println!("{}", "=".repeat(70));

    let mut validator = CalibrationAcceptanceValidator::new();

    // Record marginal calibration results
    validator.record_metric("reprojection_rms", 0.65); // Marginal
    validator.record_metric("vertical_disparity_rms", 0.28); // Marginal
    validator.record_metric("epipolar_residual", 0.90); // Marginal
    validator.record_metric("timing_observability", 0.52); // Marginal
    validator.record_metric("time_offset_jitter", 1.2e-3); // Marginal (1.2ms)

    // Generate report
    let report = validator.validate();
    report.print_report();

    if report.accepted {
        println!("\n⚠ Decision: ACCEPTED (Marginal) - Recommend recalibration");
    } else {
        println!("\n✗ Decision: REJECTED - Recalibrate required");
    }
}

/// Example 3: Failed calibration
fn example_failed_calibration() {
    println!("\n{}", "=".repeat(70));
    println!("EXAMPLE 3: Failed Calibration (Rejection)");
    println!("{}", "=".repeat(70));

    let mut validator = CalibrationAcceptanceValidator::new();

    // Record poor calibration results
    validator.record_metric("reprojection_rms", 1.8); // FAILED
    validator.record_metric("vertical_disparity_rms", 0.8); // FAILED
    validator.record_metric("timing_observability", 0.2); // FAILED

    // Generate report
    let report = validator.validate();
    report.print_report();

    println!("\n✗ Decision: DEPLOYMENT BLOCKED - Must recalibrate");
}

/// Example 4: Custom thresholds for mobile robot
fn example_custom_thresholds() {
    println!("\n{}", "=".repeat(70));
    println!("EXAMPLE 4: Custom Thresholds for Mobile Robot");
    println!("{}", "=".repeat(70));

    // Mobile robots are often more tolerant than UAVs
    let mut thresholds = rs_vio::calibration::types::AcceptanceThresholds::standard();
    thresholds.reprojection_rms_max = 1.2; // More tolerant (slower motion)
    thresholds.vertical_disparity_rms_max = 0.5;
    thresholds.epipolar_residual_max = 1.5;

    let mut validator = CalibrationAcceptanceValidator::with_thresholds(thresholds);

    // Record results that would fail standard thresholds
    validator.record_metric("reprojection_rms", 0.95); // OK for mobile robot
    validator.record_metric("vertical_disparity_rms", 0.42); // OK for mobile robot

    let report = validator.validate();
    report.print_report();

    println!("\n✓ Decision: ACCEPTED for mobile robot application");
}

/// Example 5: Batch validation of multiple cameras
fn example_batch_validation() {
    println!("\n{}", "=".repeat(70));
    println!("EXAMPLE 5: Batch Validation of Multiple Stereo Pairs");
    println!("{}", "=".repeat(70));

    let cameras = vec![
        ("Front Camera", 0.22, 0.15, 0.25),
        ("Left Camera", 0.28, 0.22, 0.35),
        ("Rear Camera", 0.18, 0.12, 0.20),
        ("Right Camera", 0.35, 0.28, 0.40), // Failed
    ];

    for (name, reproj, disp, epipolar) in cameras {
        println!("\nValidating: {}", name);
        let mut validator = CalibrationAcceptanceValidator::new();
        validator.record_metric("reprojection_rms", reproj);
        validator.record_metric("vertical_disparity_rms", disp);
        validator.record_metric("epipolar_residual", epipolar);

        let report = validator.validate();
        let status = if report.accepted {
            "✓ PASS"
        } else {
            "✗ FAIL"
        };
        println!("  {} (Score: {:.1}/100)", status, report.overall_score);
    }
}

/// Example 6: JSON export for CI/CD integration
fn example_json_export() {
    println!("\n{}", "=".repeat(70));
    println!("EXAMPLE 6: JSON Export for CI/CD Integration");
    println!("{}", "=".repeat(70));

    let mut validator = CalibrationAcceptanceValidator::new();
    validator.record_metric("reprojection_rms", 0.32);
    validator.record_metric("vertical_disparity_rms", 0.15);
    validator.record_metric("epipolar_residual", 0.28);

    let report = validator.validate();
    let json = report.to_json();

    println!("\nJSON Output (for CI/CD pipeline):");
    println!("{}", json);

    println!("\nThis JSON can be:");
    println!("  - Logged to database");
    println!("  - Sent to monitoring system");
    println!("  - Used to trigger automated decisions");
}

/// Example 7: Production deployment decision matrix
fn example_production_decision() {
    println!("\n{}", "=".repeat(70));
    println!("EXAMPLE 7: Production Deployment Decision Matrix");
    println!("{}", "=".repeat(70));

    let test_cases = vec![
        ("Indoor, well-lit", 0.28, 0.13, 0.22, "Deploy immediately"),
        (
            "Outdoor, bright sun",
            0.45,
            0.20,
            0.38,
            "Deploy with monitoring",
        ),
        (
            "Low light, challenging",
            0.72,
            0.35,
            0.68,
            "Recalibrate or increase exposure",
        ),
        (
            "Severe motion blur",
            1.5,
            0.8,
            1.2,
            "Reject - equipment issue",
        ),
    ];

    for (scenario, reproj, disp, epipolar, recommendation) in test_cases {
        println!("\nScenario: {}", scenario);

        let mut validator = CalibrationAcceptanceValidator::new();
        validator.record_metric("reprojection_rms", reproj);
        validator.record_metric("vertical_disparity_rms", disp);
        validator.record_metric("epipolar_residual", epipolar);

        let report = validator.validate();
        println!("  Score: {:.1}/100", report.overall_score);
        println!(
            "  Status: {}",
            if report.accepted { "PASS" } else { "FAIL" }
        );
        println!("  Recommendation: {}", recommendation);
    }
}

fn main() {
    println!("\n╔════════════════════════════════════════════════════════════════════╗");
    println!("║          RS-VIO Calibration Validation Demonstrations             ║");
    println!("╚════════════════════════════════════════════════════════════════════╝");

    example_excellent_calibration();
    example_marginal_calibration();
    example_failed_calibration();
    example_custom_thresholds();
    example_batch_validation();
    example_json_export();
    example_production_decision();

    println!("\n{}", "=".repeat(70));
    println!("All demonstrations completed successfully!");
    println!("Use these patterns in your calibration pipeline.");
    println!("{}", "=".repeat(70));
}
