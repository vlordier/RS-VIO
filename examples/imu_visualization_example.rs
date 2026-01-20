//! # IMU Visualization Integration Example
//!
//! This module demonstrates how to integrate the IMU visualization features
//! into an existing SLAM/VIO pipeline.

#![allow(dead_code)]

use rs_vio::datasets::ImuData;
use rs_vio::imu::ImuSignalAnalyzer;
use rs_vio::viewers::Viewer;

fn main() {
    println!("IMU Visualization Example - See function documentation for usage");
}

/// Example: Process and visualize IMU data with harmonic decomposition
pub fn example_imu_visualization(
    viewer: &mut Box<dyn Viewer>,
    imu_measurements: &[ImuData],
    timestamp_ns: i64,
) {
    // Create IMU signal analyzer with 100-sample window
    let mut analyzer = ImuSignalAnalyzer::new(100);

    // Collect raw measurements for visualization
    let mut raw_accel = Vec::new();
    let mut raw_gyro = Vec::new();

    // Process each IMU measurement
    for imu in imu_measurements {
        analyzer.process_measurement(imu);
        raw_accel.push([
            imu.accel[0] as f32,
            imu.accel[1] as f32,
            imu.accel[2] as f32,
        ]);
        raw_gyro.push([imu.gyro[0] as f32, imu.gyro[1] as f32, imu.gyro[2] as f32]);
    }

    // Step 1: Log raw measurements (before processing)
    viewer.log_imu_raw(timestamp_ns, &raw_accel, &raw_gyro, "imu/raw");

    // Step 2: Get bias estimates
    let (bias_accel, bias_gyro) = analyzer.get_bias_estimates();

    // Step 3: Compute processed (bias-corrected) measurements
    let mut processed_accel = Vec::new();
    let mut processed_gyro = Vec::new();

    for imu in imu_measurements {
        let corrected_a = [
            (imu.accel[0] - bias_accel.x) as f32,
            (imu.accel[1] - bias_accel.y) as f32,
            (imu.accel[2] - bias_accel.z) as f32,
        ];
        let corrected_g = [
            (imu.gyro[0] - bias_gyro.x) as f32,
            (imu.gyro[1] - bias_gyro.y) as f32,
            (imu.gyro[2] - bias_gyro.z) as f32,
        ];
        processed_accel.push(corrected_a);
        processed_gyro.push(corrected_g);
    }

    // Log processed (bias-corrected) measurements
    viewer.log_imu_processed(
        timestamp_ns,
        &processed_accel,
        &processed_gyro,
        "imu/processed",
    );

    // Step 4: Decompose into harmonics
    let harmonic_decomp = analyzer.decompose_harmonics();

    // Convert to f32 for visualization
    let gravity = [
        harmonic_decomp.gravity.x as f32,
        harmonic_decomp.gravity.y as f32,
        harmonic_decomp.gravity.z as f32,
    ];
    let bias_a = [
        harmonic_decomp.accel_bias.x as f32,
        harmonic_decomp.accel_bias.y as f32,
        harmonic_decomp.accel_bias.z as f32,
    ];
    let bias_g = [
        harmonic_decomp.gyro_bias.x as f32,
        harmonic_decomp.gyro_bias.y as f32,
        harmonic_decomp.gyro_bias.z as f32,
    ];
    let harmonics: Vec<[f32; 3]> = harmonic_decomp
        .residual_harmonics
        .iter()
        .map(|h| [h.x as f32, h.y as f32, h.z as f32])
        .collect();

    // Log harmonic decomposition
    viewer.log_imu_harmonics(
        timestamp_ns,
        gravity,
        bias_a,
        bias_g,
        &harmonics,
        "imu/harmonics",
    );

    // Step 5: Log signal quality metrics with motor state
    let motor_state_str = match harmonic_decomp.motor_state {
        rs_vio::imu::MotorState::Off => "Off",
        rs_vio::imu::MotorState::Running => "Running",
        rs_vio::imu::MotorState::Transitioning => "Transitioning",
    };

    viewer.log_imu_signal_quality(
        timestamp_ns,
        harmonic_decomp.quality.snr,
        harmonic_decomp.quality.rms,
        harmonic_decomp.quality.peak,
        motor_state_str,
        harmonic_decomp.quality.fundamental_freq_hz,
        "imu/quality",
    );

    // (Optional) Use quality metrics for adaptive filtering
    let avg_snr = (harmonic_decomp.quality.snr[0]
        + harmonic_decomp.quality.snr[1]
        + harmonic_decomp.quality.snr[2])
        / 3.0;

    // Adjust warnings based on motor state
    match harmonic_decomp.motor_state {
            rs_vio::imu::MotorState::Off => {
            if avg_snr < 20.0 {
                log::warn!(
                    "[IMU] Low signal quality in stationary mode (SNR: {:.1} dB), check sensor calibration",
                    avg_snr
                );
            }
        },
            rs_vio::imu::MotorState::Running => {
            if avg_snr < 10.0 {
                log::warn!(
                    "[IMU] Very high vibration during flight (SNR: {:.1} dB), f₀={:.1} Hz - consider vibration damping",
                    avg_snr, harmonic_decomp.quality.fundamental_freq_hz
                );
            } else {
                log::info!(
                    "[IMU] In-flight mode detected: f₀={:.1} Hz, SNR={:.1} dB",
                    harmonic_decomp.quality.fundamental_freq_hz,
                    avg_snr
                );
            }
        },
            rs_vio::imu::MotorState::Transitioning => {
            log::info!("[IMU] Motor state transitioning...");
        },
    }
}

/// Example: Integrate with Estimator's process_frame
/// This shows where to call IMU visualization in the main processing loop
#[allow(non_snake_case)]
pub fn example_estimator_integration(
    frame_idx: usize,
    timestamp_ns: i64,
    imu_data: &[ImuData],
    viewer: &mut Box<dyn Viewer>,
) {
    // Your normal VIO processing happens here...
    // estimator.process_frame(&left_image, &right_image, timestamp_ns, Some(imu_data))?;

    // After VIO processing, visualize IMU decomposition
    if !imu_data.is_empty() {
        // Update viewer timestamp for synchronization
        viewer.set_frame(frame_idx as i64);

        // Visualize before/after IMU processing
        example_imu_visualization(viewer, imu_data, timestamp_ns);

        // If you have bias estimates from the filter, use them:
        // viewer.log_imu_processed(
        //     timestamp_ns,
        //     &bias_corrected_accel,
        //     &bias_corrected_gyro,
        //     "imu/processed/from_filter"
        // );
    }
}

/// Example: Demonstrate what harmonics decomposition reveals about sensor quality
pub fn analyze_sensor_health(imu_measurements: &[ImuData]) -> String {
    let mut analyzer = ImuSignalAnalyzer::new(100);

    for imu in imu_measurements {
        analyzer.process_measurement(imu);
    }

    let decomp = analyzer.decompose_harmonics();
    let (bias_a, bias_g) = analyzer.get_bias_estimates();

    // Analyze harmonics
    let harmonic_rms: f32 = decomp
        .residual_harmonics
        .iter()
        .map(|h| {
            (h.x as f32 * h.x as f32 + h.y as f32 * h.y as f32 + h.z as f32 * h.z as f32).sqrt()
        })
        .sum::<f32>()
        / decomp.residual_harmonics.len().max(1) as f32;

    let avg_snr = (decomp.quality.snr[0] + decomp.quality.snr[1] + decomp.quality.snr[2]) / 3.0;

    format!(
        r#"=== IMU Sensor Health Report ===
Gravity Estimate:      [{:.3}, {:.3}, {:.3}] m/s²
  Magnitude: {:.3} (expected: 9.81)
Accelerometer Bias:    [{:.4}, {:.4}, {:.4}] m/s²
  Magnitude: {:.4}
Gyroscope Bias:        [{:.4}, {:.4}, {:.4}] rad/s
Signal Quality:
  Average SNR:         {:.1} dB
  RMS (X, Y, Z):       [{:.4}, {:.4}, {:.4}] m/s²
  Peak (X, Y, Z):      [{:.4}, {:.4}, {:.4}] m/s²
Harmonic Analysis:
  RMS Harmonics:       {:.4} m/s²
  Fundamental:         [{:.4}, {:.4}, {:.4}] m/s²
Quality Assessment:    {}
"#,
        decomp.gravity.x,
        decomp.gravity.y,
        decomp.gravity.z,
        decomp.gravity.norm(),
        bias_a.x,
        bias_a.y,
        bias_a.z,
        bias_a.norm(),
        bias_g.x,
        bias_g.y,
        bias_g.z,
        avg_snr,
        decomp.quality.rms[0],
        decomp.quality.rms[1],
        decomp.quality.rms[2],
        decomp.quality.peak[0],
        decomp.quality.peak[1],
        decomp.quality.peak[2],
        harmonic_rms,
        decomp.fundamental_harmonic.x,
        decomp.fundamental_harmonic.y,
        decomp.fundamental_harmonic.z,
        if avg_snr > 30.0 {
            "✓ Excellent"
        } else if avg_snr > 20.0 {
            "✓ Good"
        } else if avg_snr > 10.0 {
            "⚠ Fair"
        } else {
            "✗ Poor"
        }
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensor_health_analysis() {
        // Create synthetic stationary IMU data
        let imu_data: Vec<ImuData> = (0..100)
            .map(|i| ImuData {
                timestamp: (i * 10000) as i64, // 100 Hz
                accel: [
                    0.01 * (i as f64 % 10.0),
                    0.01 * ((i as f64 + 3.0) % 10.0),
                    -9.81,
                ],
                gyro: [0.001, 0.001, 0.001],
            })
            .collect();

        let report = analyze_sensor_health(&imu_data);
        println!("{}", report);
        assert!(report.contains("IMU Sensor Health Report"));
    }
}
