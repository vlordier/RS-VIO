//! # Learned Vibration Scheduler
//!
//! Implements adaptive IMU covariance scaling based on vibration analysis.
//! Uses FFT-based vibration detection and learned models to optimize
//! measurement noise characteristics for different flight conditions.

use crate::imu::vibration_filter::VibrationNotchFilter;
use crate::types::Float;
use std::collections::VecDeque;

/// Inputs to the vibration scheduler
#[derive(Debug, Clone)]
pub struct VibrationInputs {
    pub throttle: Float,
    pub vibration_level: Float,
    pub time_since_keyframe: Float,
    /// Current IMU sampling rate [Hz]
    pub imu_rate: Float,
}

/// Outputs from the vibration scheduler
#[derive(Debug, Clone)]
pub struct VibrationOutputs {
    pub covariance_scale: Float,
    pub confidence: Float,
    /// Recommended filter update interval [samples]
    pub filter_update_interval: usize,
}

/// Training sample for vibration scheduling
#[derive(Debug, Clone)]
pub struct VibrationTrainingSample {
    /// Input conditions
    pub inputs: VibrationInputs,
    /// Optimal covariance scale (learned from experience)
    pub optimal_scale: Float,
    /// Performance metric (e.g., tracking accuracy)
    pub performance_score: Float,
}

/// Learned vibration scheduler with training data and FFT-based analysis
pub struct LearnedVibrationScheduler {
    /// Vibration filter for real-time analysis
    vibration_filter: Option<VibrationNotchFilter>,
    /// Historical vibration levels for trend analysis
    vibration_history: VecDeque<Float>,
    /// Training data for learning optimal parameters
    training_data: Vec<VibrationTrainingSample>,
    /// Learned weights for different factors
    weights: VibrationWeights,
    /// Maximum history length
    max_history: usize,
    /// Learning rate for online adaptation
    #[allow(dead_code)]
    learning_rate: Float,
}

/// Learned weights for vibration scheduling factors
#[derive(Debug, Clone)]
struct VibrationWeights {
    /// Weight for vibration level
    vibration_weight: Float,
    /// Weight for trend (increasing/decreasing)
    #[allow(dead_code)]
    trend_weight: Float,
    /// Weight for detected peaks
    peaks_weight: Float,
    /// Weight for throttle
    throttle_weight: Float,
    /// Weight for time since keyframe
    time_weight: Float,
}

impl Default for VibrationWeights {
    fn default() -> Self {
        Self {
            vibration_weight: 1.0,
            trend_weight: 0.8,
            peaks_weight: 0.6,
            throttle_weight: 0.4,
            time_weight: 0.3,
        }
    }
}

impl LearnedVibrationScheduler {
    /// Create new learned vibration scheduler
    pub fn new() -> Self {
        Self {
            vibration_filter: None,
            vibration_history: VecDeque::with_capacity(100),
            training_data: Vec::new(),
            weights: VibrationWeights::default(),
            max_history: 100,
            learning_rate: 0.01,
        }
    }

    /// Initialize with vibration filter for real-time analysis
    pub fn with_vibration_filter(mut self, filter: VibrationNotchFilter) -> Self {
        self.vibration_filter = Some(filter);
        self
    }

    /// Predict optimal covariance scaling based on current conditions
    pub fn predict(&mut self, inputs: &VibrationInputs) -> VibrationOutputs {
        // Update vibration history
        self.vibration_history.push_back(inputs.vibration_level);
        if self.vibration_history.len() > self.max_history {
            self.vibration_history.pop_front();
        }

        // Compute trend (recent vs historical average)
        let recent_avg = if self.vibration_history.len() >= 10 {
            self.vibration_history.iter().rev().take(10).sum::<Float>() / 10.0
        } else {
            inputs.vibration_level
        };

        let historical_avg =
            self.vibration_history.iter().sum::<Float>() / self.vibration_history.len() as Float;

        // Analyze vibration characteristics
        let vibration_trend = recent_avg / (historical_avg + 1e-6);
        let trend_factor = if vibration_trend > 1.2 {
            1.5 // Increasing vibration
        } else if vibration_trend < 0.8 {
            0.8 // Decreasing vibration
        } else {
            1.0 // Stable
        };

        // Get filter information if available
        let detected_peaks = self
            .vibration_filter
            .as_ref()
            .map(|f| f.get_detected_peaks().len())
            .unwrap_or(0) as Float;

        // Use learned model for scaling
        let vibration_factor = self.compute_vibration_factor(inputs.vibration_level);
        let throttle_factor = self.compute_throttle_factor(inputs.throttle);
        let time_factor = self.compute_time_factor(inputs.time_since_keyframe);
        let peaks_factor = self.compute_peaks_factor(detected_peaks);

        // Combine factors using learned weights
        let scale = vibration_factor * trend_factor * throttle_factor * time_factor * peaks_factor;

        // Clamp to reasonable range
        let mut clamped_scale = scale.max(1.0).min(10.0);

        // Override with training data if very close match
        if let Some(best_match) = self.find_similar_training_sample(
            inputs.vibration_level,
            inputs.throttle,
            inputs.time_since_keyframe,
            detected_peaks,
        ) {
            let dv = (best_match.inputs.vibration_level - inputs.vibration_level).abs();
            let dt = (best_match.inputs.throttle - inputs.throttle).abs();
            let dtime = (best_match.inputs.time_since_keyframe - inputs.time_since_keyframe).abs();
            let distance = dv * 10.0 + dt * 5.0 + dtime;
            if distance < 0.1 {
                clamped_scale = best_match.optimal_scale;
            }
        }

        // Confidence based on data quality and training
        let confidence = self.compute_confidence(inputs);

        // Filter update interval (adaptive based on vibration)
        let filter_update_interval = self.compute_update_interval(inputs);

        VibrationOutputs {
            covariance_scale: clamped_scale,
            confidence,
            filter_update_interval,
        }
    }

    /// Get current vibration filter (if available)
    pub fn vibration_filter(&self) -> Option<&VibrationNotchFilter> {
        self.vibration_filter.as_ref()
    }

    /// Add training sample for online learning
    pub fn add_training_sample(&mut self, sample: VibrationTrainingSample) {
        self.training_data.push(sample);

        // Keep only recent training data
        if self.training_data.len() > 1000 {
            self.training_data.remove(0);
        }

        // Update weights based on training data
        self.update_weights();
    }

    /// Get training data statistics
    pub fn training_stats(&self) -> (usize, Float) {
        let count = self.training_data.len();
        let avg_performance = if count > 0 {
            self.training_data
                .iter()
                .map(|s| s.performance_score)
                .sum::<Float>()
                / count as Float
        } else {
            0.0
        };
        (count, avg_performance)
    }

    /// Get vibration filter mutably
    pub fn vibration_filter_mut(&mut self) -> Option<&mut VibrationNotchFilter> {
        self.vibration_filter.as_mut()
    }

    // Private helper methods for learned model

    fn update_weights(&mut self) {
        if self.training_data.len() < 10 {
            return; // Need minimum training data
        }

        // Simple online learning: adjust weights based on prediction errors
        // This is a simplified implementation - could be enhanced with proper ML

        let recent_samples: Vec<_> = self.training_data.iter().rev().take(50).collect();

        for sample in recent_samples {
            // Simulate prediction with current weights
            let predicted_scale = self.compute_vibration_factor(sample.inputs.vibration_level);
            let error = predicted_scale - sample.optimal_scale;

            // Update weights based on error
            if error > 0.1 {
                // Over-estimating - reduce vibration weight
                self.weights.vibration_weight *= 1.0 - self.learning_rate;
            } else if error < -0.1 {
                // Under-estimating - increase vibration weight
                self.weights.vibration_weight *= 1.0 + self.learning_rate;
            }
        }

        // Clamp weights to reasonable ranges
        self.weights.vibration_weight = self.weights.vibration_weight.max(0.1).min(3.0);
    }

    fn compute_vibration_factor(&self, vibration_level: Float) -> Float {
        // Default rule-based scaling
        self.weights.vibration_weight
            * if vibration_level > 0.5 {
                5.0
            } else if vibration_level >= 0.3 {
                3.0
            } else if vibration_level > 0.1 {
                1.5
            } else {
                1.0
            }
    }

    fn compute_throttle_factor(&self, throttle: Float) -> Float {
        1.0 + self.weights.throttle_weight * throttle * 0.3
    }

    fn compute_time_factor(&self, time_since_keyframe: Float) -> Float {
        (1.0 + self.weights.time_weight * time_since_keyframe * 0.1).min(2.0)
    }

    fn compute_peaks_factor(&self, peaks: Float) -> Float {
        1.0 + self.weights.peaks_weight * peaks * 0.1
    }

    fn compute_confidence(&self, inputs: &VibrationInputs) -> Float {
        let mut confidence = 0.6 as Float; // Base confidence

        // Higher confidence with more history
        confidence += (self.vibration_history.len() as Float / 50.0).min(0.3);

        // Higher confidence with FFT analysis
        if self.vibration_filter.is_some() {
            confidence += 0.2;
        }

        // Higher confidence with training data
        if !self.training_data.is_empty() {
            confidence += 0.1;
        }

        // Lower confidence during extreme conditions
        if inputs.vibration_level > 0.8 || inputs.throttle > 0.9 {
            confidence *= 0.8;
        }

        confidence.min(0.95 as Float)
    }

    fn compute_update_interval(&self, inputs: &VibrationInputs) -> usize {
        // More frequent updates during high vibration
        if inputs.vibration_level > 0.3 {
            (inputs.imu_rate * 0.05) as usize // 50ms
        } else {
            (inputs.imu_rate * 0.1) as usize // 100ms
        }
    }

    fn find_similar_training_sample(
        &self,
        vibration: Float,
        throttle: Float,
        time: Float,
        _peaks: Float,
    ) -> Option<&VibrationTrainingSample> {
        if self.training_data.is_empty() {
            return None;
        }

        self.training_data.iter().min_by_key(|sample| {
            let dv = (sample.inputs.vibration_level - vibration).abs();
            let dt = (sample.inputs.throttle - throttle).abs();
            let dtime = (sample.inputs.time_since_keyframe - time).abs();
            // Simple distance metric (could be improved)
            (dv * 10.0 + dt * 5.0 + dtime) as i32
        })
    }
}

/// Rule-based fallback scheduler (simplified version)
pub struct RuleBasedVibrationScheduler;

impl RuleBasedVibrationScheduler {
    /// Simple rule-based prediction without history or FFT analysis
    pub fn predict(inputs: &VibrationInputs) -> VibrationOutputs {
        let mut scale = 1.0;

        // Simple vibration-based scaling
        if inputs.vibration_level > 0.5 {
            scale = 4.0;
        } else if inputs.vibration_level > 0.3 {
            scale = 2.5;
        } else if inputs.vibration_level > 0.2 {
            scale = 1.8;
        } else if inputs.vibration_level > 0.1 {
            scale = 1.3;
        }

        // Throttle adjustment
        if inputs.throttle > 0.9 {
            scale *= 1.2;
        }

        // Time-based uncertainty
        if inputs.time_since_keyframe > 2.0 {
            scale *= 1.5;
        }

        VibrationOutputs {
            covariance_scale: scale as Float,
            confidence: 0.5, // Lower confidence than learned scheduler
            filter_update_interval: (inputs.imu_rate * 0.2) as usize, // 200ms update
        }
    }
}
