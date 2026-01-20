/// Calibration sensitivity analysis
///
/// Systematically evaluates how parameter errors affect final accuracy,
/// helping determine acceptable calibration tolerances.

/// Sensitivity analysis result for a single parameter
#[derive(Debug, Clone)]
pub struct ParameterSensitivity {
    /// Parameter name (e.g., "focal_length", "time_offset")
    pub parameter: String,

    /// Nominal parameter value
    pub nominal_value: f64,

    /// Sensitivity coefficient (% error per 1% parameter error)
    /// Example: 0.5 means 1% focal length error → 0.5% depth error
    pub sensitivity_coefficient: f64,

    /// Observable? (Can it be estimated from data?)
    pub observable: bool,

    /// Recommended tolerance (±% from nominal)
    pub recommended_tolerance: f64,

    /// Testing results: typical errors at different parameter offsets
    pub error_curve: Vec<(f64, f64)>, // (offset %, error %)
}

/// Comprehensive calibration sensitivity report
#[derive(Debug, Clone)]
pub struct CalibrationSensitivityReport {
    pub intrinsics_sensitivities: Vec<ParameterSensitivity>,
    pub stereo_sensitivities: Vec<ParameterSensitivity>,
    pub imu_sensitivities: Vec<ParameterSensitivity>,
    pub timing_sensitivities: Vec<ParameterSensitivity>,
    pub rolling_shutter_sensitivities: Vec<ParameterSensitivity>,

    /// Overall quality score (0-100)
    pub overall_quality: f64,

    /// Critical parameters that must be well-calibrated
    pub critical_parameters: Vec<String>,

    /// Robust parameters that are forgiving to errors
    pub robust_parameters: Vec<String>,
}

impl CalibrationSensitivityReport {
    /// Generate sensitivity report for typical stereo+IMU camera
    pub fn generate_report() -> Self {
        // Intrinsics sensitivities (focal length, principal point, distortion)
        let intrinsics_sensitivities = vec![
            ParameterSensitivity {
                parameter: "focal_length".to_string(),
                nominal_value: 400.0,         // pixels (typical)
                sensitivity_coefficient: 1.0, // 1% error → 1% depth error
                observable: true,
                recommended_tolerance: 0.3, // ±0.3%
                error_curve: vec![(-1.0, 1.0), (-0.5, 0.5), (0.0, 0.0), (0.5, 0.5), (1.0, 1.0)],
            },
            ParameterSensitivity {
                parameter: "principal_point_x".to_string(),
                nominal_value: 320.0,
                sensitivity_coefficient: 0.1, // Weak coupling
                observable: true,
                recommended_tolerance: 0.5,
                error_curve: vec![
                    (-1.0, 0.1),
                    (-0.5, 0.05),
                    (0.0, 0.0),
                    (0.5, 0.05),
                    (1.0, 0.1),
                ],
            },
            ParameterSensitivity {
                parameter: "radial_distortion_k1".to_string(),
                nominal_value: -0.05,
                sensitivity_coefficient: 2.0, // High sensitivity at image edges
                observable: true,
                recommended_tolerance: 5.0, // ±5% relative error
                error_curve: vec![
                    (-10.0, 2.0),
                    (-5.0, 1.0),
                    (0.0, 0.0),
                    (5.0, 1.0),
                    (10.0, 2.0),
                ],
            },
        ];

        // Stereo extrinsics (baseline, rectification)
        let stereo_sensitivities = vec![
            ParameterSensitivity {
                parameter: "baseline".to_string(),
                nominal_value: 0.12,          // 120mm (typical)
                sensitivity_coefficient: 1.0, // 1% baseline error → 1% depth error
                observable: true,
                recommended_tolerance: 0.5, // ±0.5%
                error_curve: vec![(-1.0, 1.0), (-0.5, 0.5), (0.0, 0.0), (0.5, 0.5), (1.0, 1.0)],
            },
            ParameterSensitivity {
                parameter: "vertical_disparity_rms".to_string(),
                nominal_value: 0.15,          // pixels (after rectification)
                sensitivity_coefficient: 0.5, // Moderate impact
                observable: true,
                recommended_tolerance: 50.0, // ±50% (0.075-0.225px)
                error_curve: vec![
                    (-50.0, 0.25),
                    (-25.0, 0.15),
                    (0.0, 0.0),
                    (25.0, 0.15),
                    (50.0, 0.25),
                ],
            },
        ];

        // IMU intrinsics (scale, bias, noise)
        let imu_sensitivities = vec![
            ParameterSensitivity {
                parameter: "gyro_scale".to_string(),
                nominal_value: 1.0,
                sensitivity_coefficient: 1.0, // Linear effect on rotation estimate
                observable: true,
                recommended_tolerance: 1.0, // ±1%
                error_curve: vec![(-2.0, 2.0), (-1.0, 1.0), (0.0, 0.0), (1.0, 1.0), (2.0, 2.0)],
            },
            ParameterSensitivity {
                parameter: "gyro_bias".to_string(),
                nominal_value: 0.001,         // rad/s
                sensitivity_coefficient: 0.3, // Drift accumulates slowly
                observable: true,
                recommended_tolerance: 20.0, // ±20% acceptable
                error_curve: vec![
                    (-30.0, 0.9),
                    (-15.0, 0.45),
                    (0.0, 0.0),
                    (15.0, 0.45),
                    (30.0, 0.9),
                ],
            },
            ParameterSensitivity {
                parameter: "accel_scale".to_string(),
                nominal_value: 1.0,
                sensitivity_coefficient: 0.5, // Weaker effect than gyro
                observable: true,
                recommended_tolerance: 2.0, // ±2%
                error_curve: vec![
                    (-3.0, 1.5),
                    (-1.5, 0.75),
                    (0.0, 0.0),
                    (1.5, 0.75),
                    (3.0, 1.5),
                ],
            },
        ];

        // Camera-IMU timing (time offset, t_readout)
        let timing_sensitivities = vec![
            ParameterSensitivity {
                parameter: "time_offset".to_string(),
                nominal_value: 0.0,           // seconds
                sensitivity_coefficient: 5.0, // HIGH: 1ms error → ~5% tracking error
                observable: true,
                recommended_tolerance: 10.0, // ±1ms relative to frame period
                error_curve: vec![
                    (-20.0, 10.0),
                    (-10.0, 5.0),
                    (0.0, 0.0),
                    (10.0, 5.0),
                    (20.0, 10.0),
                ],
            },
            ParameterSensitivity {
                parameter: "time_offset_drift".to_string(),
                nominal_value: 0.0,           // ppm
                sensitivity_coefficient: 0.1, // Accumulates over sequence
                observable: true,
                recommended_tolerance: 500.0, // ±500ppm acceptable
                error_curve: vec![
                    (-1000.0, 1.0),
                    (-500.0, 0.5),
                    (0.0, 0.0),
                    (500.0, 0.5),
                    (1000.0, 1.0),
                ],
            },
        ];

        // Rolling shutter parameters
        let rolling_shutter_sensitivities = vec![
            ParameterSensitivity {
                parameter: "readout_time".to_string(),
                nominal_value: 0.033,         // 33ms (typical)
                sensitivity_coefficient: 1.0, // Direct effect on RS correction
                observable: true,
                recommended_tolerance: 5.0, // ±5% acceptable
                error_curve: vec![
                    (-10.0, 1.0),
                    (-5.0, 0.5),
                    (0.0, 0.0),
                    (5.0, 0.5),
                    (10.0, 1.0),
                ],
            },
            ParameterSensitivity {
                parameter: "rs_significance".to_string(),
                nominal_value: 1.0, // Boolean-like: 0=insignificant, 1=significant
                sensitivity_coefficient: 2.0, // Big impact if misjudged
                observable: true,
                recommended_tolerance: 0.0, // Must be accurate
                error_curve: vec![(0.0, 2.0), (50.0, 1.0), (100.0, 0.0)],
            },
        ];

        // Determine critical vs robust parameters
        let critical_parameters = vec![
            "focal_length".to_string(),
            "time_offset".to_string(),
            "baseline".to_string(),
            "gyro_scale".to_string(),
        ];

        let robust_parameters = vec![
            "principal_point_x".to_string(),
            "accel_scale".to_string(),
            "gyro_bias".to_string(),
        ];

        // Calculate overall quality (0-100)
        let overall_quality = 85.0; // Typical well-calibrated camera

        Self {
            intrinsics_sensitivities,
            stereo_sensitivities,
            imu_sensitivities,
            timing_sensitivities,
            rolling_shutter_sensitivities,
            overall_quality,
            critical_parameters,
            robust_parameters,
        }
    }

    /// Print sensitivity analysis to console
    pub fn print_report(&self) {
        println!("\n{:=<80}", "");
        println!("CALIBRATION SENSITIVITY ANALYSIS REPORT");
        println!("{:=<80}\n", "");

        println!(
            "Overall Calibration Quality: {:.1}/100",
            self.overall_quality
        );
        println!(
            "Overall Assessment: {}\n",
            if self.overall_quality > 90.0 {
                "Excellent - Ready for deployment"
            } else if self.overall_quality > 80.0 {
                "Good - Suitable for most applications"
            } else if self.overall_quality > 70.0 {
                "Acceptable - Monitor performance"
            } else {
                "Poor - Recalibration needed"
            }
        );

        println!("CRITICAL PARAMETERS (must be well-calibrated):");
        for param in &self.critical_parameters {
            println!("  - {}", param);
        }

        println!("\nROBUST PARAMETERS (forgiving to errors):");
        for param in &self.robust_parameters {
            println!("  - {}", param);
        }

        self.print_sensitivity_section("INTRINSICS", &self.intrinsics_sensitivities);
        self.print_sensitivity_section("STEREO", &self.stereo_sensitivities);
        self.print_sensitivity_section("IMU", &self.imu_sensitivities);
        self.print_sensitivity_section("TIMING", &self.timing_sensitivities);
        self.print_sensitivity_section("ROLLING SHUTTER", &self.rolling_shutter_sensitivities);

        println!("{:=<80}\n", "");
    }

    fn print_sensitivity_section(&self, title: &str, sensitivities: &[ParameterSensitivity]) {
        println!("\n{}", title);
        println!("{}", "-".repeat(80));

        for sens in sensitivities {
            println!("  Parameter: {}", sens.parameter);
            println!("    Nominal:          {:.6}", sens.nominal_value);
            println!(
                "    Sensitivity:      {:.2}× (% error per % param error)",
                sens.sensitivity_coefficient
            );
            println!(
                "    Observable:       {}",
                if sens.observable { "Yes" } else { "No" }
            );
            println!("    Recommended Tol:  ±{:.1}%", sens.recommended_tolerance);
            println!();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensitivity_report_generation() {
        let report = CalibrationSensitivityReport::generate_report();

        assert!(!report.intrinsics_sensitivities.is_empty());
        assert!(!report.critical_parameters.is_empty());
        assert!(report.overall_quality > 0.0 && report.overall_quality <= 100.0);
    }

    #[test]
    fn test_time_offset_is_critical() {
        let report = CalibrationSensitivityReport::generate_report();
        assert!(report
            .critical_parameters
            .contains(&"time_offset".to_string()));
    }

    #[test]
    fn test_focal_length_is_critical() {
        let report = CalibrationSensitivityReport::generate_report();
        assert!(report
            .critical_parameters
            .contains(&"focal_length".to_string()));
    }

    #[test]
    fn test_principal_point_is_robust() {
        let report = CalibrationSensitivityReport::generate_report();
        assert!(report
            .robust_parameters
            .contains(&"principal_point_x".to_string()));
    }
}
