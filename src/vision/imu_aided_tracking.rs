/// IMU-aided feature tracking for enhanced temporal stability
///
/// Leverages gyroscope measurements to predict feature motion between frames,
/// improving KLT (Kanade-Lucas-Tomasi) tracking initialization and robustness.
use nalgebra::{Matrix3, Vector2, Vector3};
use std::collections::VecDeque;

/// IMU measurement for tracking
#[derive(Clone, Debug)]
pub struct IMUMeasurement {
    /// Timestamp (seconds)
    pub timestamp: f64,
    /// Angular velocity (rad/s)
    pub gyro: Vector3<f64>,
    /// Accelerometer reading (m/s^2)
    pub accel: Vector3<f64>,
}

/// Feature point being tracked
#[derive(Clone, Debug)]
pub struct TrackedFeature {
    /// Feature ID
    pub id: u32,
    /// Current position (pixels)
    pub position: Vector2<f64>,
    /// Predicted position (from IMU)
    pub predicted_position: Vector2<f64>,
    /// Feature descriptor (e.g., 32x32 patch)
    pub descriptor: Vec<f32>,
    /// Tracking confidence (0-1)
    pub confidence: f64,
    /// Number of frames tracked
    pub track_length: usize,
    /// Last update timestamp
    pub last_update: f64,
}

impl TrackedFeature {
    pub fn new(id: u32, position: Vector2<f64>, descriptor: Vec<f32>) -> Self {
        Self {
            id,
            position,
            predicted_position: position,
            descriptor,
            confidence: 1.0,
            track_length: 1,
            last_update: 0.0,
        }
    }

    /// Update tracking confidence based on prediction error
    pub fn update_confidence(&mut self, prediction_error: f64) {
        // Confidence decays with prediction error
        // error < 2px: full confidence
        // error > 10px: confidence drops significantly
        self.confidence = 1.0 - (prediction_error / 10.0).min(1.0);
        self.confidence = self.confidence.max(0.0);
    }
}

/// IMU-aided tracking engine
#[allow(dead_code)]
pub struct IMUAidedTracker {
    /// Camera intrinsics
    focal_length: f64,
    principal_point: Vector2<f64>,
    image_size: Vector2<u32>,
    /// Currently tracked features
    features: Vec<TrackedFeature>,
    /// IMU measurement history
    imu_history: VecDeque<IMUMeasurement>,
    /// Camera-IMU rotation
    rotation_ic: Matrix3<f64>,
    /// Next feature ID
    next_feature_id: u32,
    /// Min tracking confidence to keep feature
    min_confidence: f64,
}

impl IMUAidedTracker {
    pub fn new(
        focal_length: f64,
        principal_point: Vector2<f64>,
        image_size: Vector2<u32>,
        rotation_ic: Matrix3<f64>,
    ) -> Self {
        Self {
            focal_length,
            principal_point,
            image_size,
            features: Vec::new(),
            imu_history: VecDeque::new(),
            rotation_ic,
            next_feature_id: 0,
            min_confidence: 0.3,
        }
    }

    /// Add IMU measurement
    pub fn add_imu_measurement(&mut self, measurement: IMUMeasurement) {
        let timestamp = measurement.timestamp;
        self.imu_history.push_back(measurement);

        // Keep only last 1 second of IMU data
        while let Some(front) = self.imu_history.front() {
            if timestamp - front.timestamp > 1.0 {
                self.imu_history.pop_front();
            } else {
                break;
            }
        }
    }

    /// Add a new feature to track
    pub fn add_feature(&mut self, position: Vector2<f64>, descriptor: Vec<f32>) -> u32 {
        let id = self.next_feature_id;
        self.next_feature_id += 1;

        let feature = TrackedFeature::new(id, position, descriptor);
        self.features.push(feature);
        id
    }

    /// Predict feature motion from IMU
    pub fn predict_feature_motion(
        &self,
        feature_pos: &Vector2<f64>,
        time_start: f64,
        time_end: f64,
    ) -> Vector2<f64> {
        let mut total_rotation = Vector3::zeros();

        // Integrate gyro over time interval
        for measurement in self.imu_history.iter() {
            if measurement.timestamp >= time_start && measurement.timestamp <= time_end {
                let dt = if measurement.timestamp == self.imu_history.front().unwrap().timestamp {
                    0.001 // Assume 1ms sample if not enough info
                } else {
                    0.01 // Typical gyro sample rate
                };
                total_rotation += measurement.gyro * dt;
            }
        }

        // Convert rotation to pixel displacement
        let pixel_motion = self.rotation_to_pixel_motion(feature_pos, &total_rotation);
        feature_pos + pixel_motion
    }

    /// Convert 3D rotation to 2D pixel motion
    fn rotation_to_pixel_motion(
        &self,
        pixel_pos: &Vector2<f64>,
        rotation: &Vector3<f64>,
    ) -> Vector2<f64> {
        // Unproject pixel to normalized ray
        let x = (pixel_pos.x - self.principal_point.x) / self.focal_length;
        let y = (pixel_pos.y - self.principal_point.y) / self.focal_length;
        let ray = Vector3::new(x, y, 1.0).normalize();

        // Rotate ray
        let rotation_matrix = self.small_angle_rotation_matrix(rotation);
        let rotated_ray = rotation_matrix * ray;

        // Project back to image plane
        let dx = self.focal_length * (rotated_ray.x / rotated_ray.z - ray.x / ray.z);
        let dy = self.focal_length * (rotated_ray.y / rotated_ray.z - ray.y / ray.z);

        Vector2::new(dx, dy)
    }

    /// Compute rotation matrix for small angles (axis-angle)
    fn small_angle_rotation_matrix(&self, rotation: &Vector3<f64>) -> Matrix3<f64> {
        let angle = rotation.norm();

        if angle < 1e-10 {
            return Matrix3::identity();
        }

        let axis = rotation / angle;

        // Rodrigues formula for small angles
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        let one_minus_cos = 1.0 - cos_a;

        let ax = axis.x;
        let ay = axis.y;
        let az = axis.z;

        Matrix3::new(
            cos_a + ax * ax * one_minus_cos,
            ax * ay * one_minus_cos - az * sin_a,
            ax * az * one_minus_cos + ay * sin_a,
            ay * ax * one_minus_cos + az * sin_a,
            cos_a + ay * ay * one_minus_cos,
            ay * az * one_minus_cos - ax * sin_a,
            az * ax * one_minus_cos - ay * sin_a,
            az * ay * one_minus_cos + ax * sin_a,
            cos_a + az * az * one_minus_cos,
        )
    }

    /// Update feature positions with KLT tracking
    pub fn update_features(
        &mut self,
        time_current: f64,
        time_previous: f64,
        _image: &[u8],
        image_width: u32,
        image_height: u32,
    ) {
        // Predict feature motion from IMU
        let predictions: Vec<_> = self
            .features
            .iter()
            .map(|feature| {
                self.predict_feature_motion(&feature.position, time_previous, time_current)
            })
            .collect();

        for (feature, predicted) in self.features.iter_mut().zip(predictions) {
            // Clamp to image bounds
            let predicted = Vector2::new(
                predicted.x.max(0.0).min((image_width - 1) as f64),
                predicted.y.max(0.0).min((image_height - 1) as f64),
            );

            feature.predicted_position = predicted;

            // Track with KLT (simplified: just use prediction, full KLT would refine)
            let prediction_error = (predicted - feature.position).norm();
            feature.update_confidence(prediction_error);

            // Accept prediction if confidence is high
            if feature.confidence > self.min_confidence {
                feature.position = predicted;
                feature.track_length += 1;
            }
        }

        // Remove low-confidence features
        self.features.retain(|f| f.confidence > self.min_confidence);
    }

    /// Get all tracked features
    pub fn get_features(&self) -> &[TrackedFeature] {
        &self.features
    }

    /// Get tracked features as mutable (for external refinement)
    pub fn get_features_mut(&mut self) -> &mut [TrackedFeature] {
        &mut self.features
    }

    /// Compute feature statistics
    pub fn get_statistics(&self) -> TrackingStatistics {
        let total_features = self.features.len();
        let avg_confidence = if total_features > 0 {
            self.features.iter().map(|f| f.confidence).sum::<f64>() / total_features as f64
        } else {
            0.0
        };

        let avg_track_length = if total_features > 0 {
            self.features.iter().map(|f| f.track_length).sum::<usize>() as f64
                / total_features as f64
        } else {
            0.0
        };

        TrackingStatistics {
            total_features,
            avg_confidence,
            avg_track_length,
            imu_history_size: self.imu_history.len(),
        }
    }
}

/// Tracking statistics
#[derive(Clone, Debug)]
pub struct TrackingStatistics {
    pub total_features: usize,
    pub avg_confidence: f64,
    pub avg_track_length: f64,
    pub imu_history_size: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_tracker() {
        let tracker = IMUAidedTracker::new(
            500.0,                      // focal length
            Vector2::new(320.0, 240.0), // principal point
            Vector2::new(640, 480),     // image size
            Matrix3::identity(),        // rotation IC
        );

        assert_eq!(tracker.next_feature_id, 0);
        assert_eq!(tracker.features.len(), 0);
    }

    #[test]
    fn test_add_feature() {
        let mut tracker = IMUAidedTracker::new(
            500.0,
            Vector2::new(320.0, 240.0),
            Vector2::new(640, 480),
            Matrix3::identity(),
        );

        let id = tracker.add_feature(Vector2::new(100.0, 100.0), vec![]);
        assert_eq!(id, 0);
        assert_eq!(tracker.features.len(), 1);

        let id2 = tracker.add_feature(Vector2::new(200.0, 200.0), vec![]);
        assert_eq!(id2, 1);
        assert_eq!(tracker.features.len(), 2);
    }

    #[test]
    fn test_rotation_to_pixel_motion() {
        let tracker = IMUAidedTracker::new(
            500.0,
            Vector2::new(320.0, 240.0),
            Vector2::new(640, 480),
            Matrix3::identity(),
        );

        let pixel_pos = Vector2::new(320.0, 240.0); // Center
        let rotation = Vector3::new(0.01, 0.0, 0.0); // Small rotation around X axis

        let motion = tracker.rotation_to_pixel_motion(&pixel_pos, &rotation);

        // Rotation around X axis should cause vertical motion
        assert!(motion.y.abs() > motion.x.abs());
    }

    #[test]
    fn test_feature_confidence_update() {
        let mut feature = TrackedFeature::new(0, Vector2::new(100.0, 100.0), vec![]);

        assert_eq!(feature.confidence, 1.0);

        // Small prediction error: confidence should remain high
        feature.update_confidence(1.0);
        assert!(feature.confidence > 0.8);

        // Large prediction error: confidence should drop
        feature.update_confidence(10.0);
        assert!(feature.confidence < 0.2);
    }
}
