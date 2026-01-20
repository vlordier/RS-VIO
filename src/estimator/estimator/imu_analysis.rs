use super::state::Estimator;

impl Estimator {
    /// Estimate the fundamental frequency (f0) from accelerometer data
    pub fn estimate_fundamental_frequency(&self, gyro_data: &[[f32; 3]]) -> f32 {
        if gyro_data.len() < 2 {
            return 0.0;
        }

        // Sample rate from IMU (EuRoC is typically 200 Hz)
        let sample_rate = 200.0_f32; // Hz

        // Compute magnitude of gyro vector
        let magnitudes: Vec<f32> = gyro_data
            .iter()
            .map(|g| (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt())
            .collect();

        // Remove DC component (subtract mean)
        let mean = magnitudes.iter().sum::<f32>() / magnitudes.len() as f32;
        let centered: Vec<f32> = magnitudes.iter().map(|&m| m - mean).collect();

        // Simple peak detection: count local maxima (simplified zero-crossing on derivative)
        let mut peak_count = 0;
        for i in 1..centered.len().saturating_sub(1) {
            if centered[i] > centered[i - 1] && centered[i] > centered[i + 1] {
                peak_count += 1;
            }
        }

        // Frequency = peaks * (sample_rate / window_duration)
        // With 2 peaks per cycle: f0 = (peak_count / 2) * sample_rate / duration
        if peak_count > 0 {
            let duration_seconds = centered.len() as f32 / sample_rate;
            (peak_count as f32 / 2.0) / duration_seconds
        } else {
            // Fallback: estimate from RMS of signal as approximate frequency indicator
            let rms = (centered.iter().map(|x| x * x).sum::<f32>() / centered.len() as f32).sqrt();
            // Map RMS to rough frequency (typical sensor vibration 5-50 Hz)
            if rms > 0.01 {
                5.0 + rms * 100.0 // Simple linear mapping
            } else {
                0.0
            }
        }
    }

    /// Decompose processed IMU data into gravity and vibration components
    pub fn compute_imu_decomposition(
        &self,
        accel: &[[f32; 3]],
        _gyro: &[[f32; 3]],
    ) -> Option<([f32; 3], [f32; 3])> {
        if accel.is_empty() {
            return None;
        }

        // Estimate gravity as mean of acceleration (assuming motion is small)
        let mut gravity = [0.0_f32; 3];
        for acc in accel {
            gravity[0] += acc[0];
            gravity[1] += acc[1];
            gravity[2] += acc[2];
        }
        gravity[0] /= accel.len() as f32;
        gravity[1] /= accel.len() as f32;
        gravity[2] /= accel.len() as f32;

        // Normalize gravity to standard gravity (9.81 m/s²)
        let gravity_mag =
            (gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]).sqrt();
        if gravity_mag > 0.1 {
            gravity[0] = gravity[0] / gravity_mag * 9.81;
            gravity[1] = gravity[1] / gravity_mag * 9.81;
            gravity[2] = gravity[2] / gravity_mag * 9.81;
        } else {
            gravity = [0.0, 0.0, -9.81];
        }

        let vibration = [
            gravity[0] - accel.get(0).map(|a| a[0]).unwrap_or(0.0),
            gravity[1] - accel.get(0).map(|a| a[1]).unwrap_or(0.0),
            gravity[2] - accel.get(0).map(|a| a[2]).unwrap_or(0.0),
        ];

        Some((gravity, vibration))
    }
}
