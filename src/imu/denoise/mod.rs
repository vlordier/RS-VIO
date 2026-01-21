/// Real-time IMU denoising filter for VIO systems
/// Handles camera frame rates (30-60 fps) vs IMU sampling (200 Hz)
/// Incorporates notch filtering for identified resonances (0.06 Hz, 1.46 Hz)
mod biquad;
mod config;
mod filter;
mod motion_mode;
mod tests;

pub use config::DenoiseConfig;
pub use filter::ImuDenoiseFilter;
pub use motion_mode::MotionMode;

/// Example usage showing integration with VIO pipeline
pub fn example_usage() {
    let mut config = DenoiseConfig::default();
    config.imu_sample_rate = 200.0;
    config.camera_frame_rate = 30.0;
    config.highpass_cutoff = 0.5; // Remove platform sway
    config.lowpass_cutoff = 50.0;
    config.enable_notch_filter = true;
    config.notch_frequencies = vec![0.06, 1.46]; // Our identified resonances
    config.notch_q = 5.0;
    config.enable_vision_fusion = true;
    config.vision_trust = 0.3; // Trust vision 30%, IMU 70%

    let mut filter = ImuDenoiseFilter::new(config);

    // Simulate 200 Hz IMU stream
    for sample in 0..200 {
        let time_ms = sample as f32 / 200.0 * 1000.0;
        let gyro = [0.1, 0.15, 0.09]; // Simulated gyro data

        // Process raw measurement through filter
        let filtered_gyro = filter.process_gyro(&gyro);

        // Buffer for camera frame integration
        filter.buffer_imu_sample(time_ms, &filtered_gyro);

        // When camera frame arrives (every ~33ms for 30 Hz)
        if sample % 7 == 0 {
            if let Some(integrated_gyro) = filter.process_camera_frame(time_ms) {
                println!(
                    "Camera frame at {}ms: integrated gyro={:.4}, quality={:.2}",
                    time_ms,
                    integrated_gyro.norm(),
                    filter.quality()
                );
            }
        }
    }
}
