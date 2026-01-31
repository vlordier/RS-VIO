/// IMU preintegration collection for teacher export
///
/// Collects and formats IMU data into the 15D preintegration vector
/// expected by the teacher training network.
use nalgebra::{SVector, Vector3};

/// Collects IMU preintegration data
#[derive(Clone, Debug)]
pub struct ImuPreintegrationCollector {
    /// Delta position (m)
    pub delta_p: Vector3<f64>,
    /// Delta velocity (m/s)
    pub delta_v: Vector3<f64>,
    /// Delta rotation as axis-angle (rad)
    pub delta_w: Vector3<f64>,
    /// Gyroscope bias (rad/s)
    pub bias_w: Vector3<f64>,
    /// Accelerometer bias (m/s²)
    pub bias_a: Vector3<f64>,
}

impl ImuPreintegrationCollector {
    /// Create new collector
    pub fn new() -> Self {
        Self {
            delta_p: Vector3::zeros(),
            delta_v: Vector3::zeros(),
            delta_w: Vector3::zeros(),
            bias_w: Vector3::zeros(),
            bias_a: Vector3::zeros(),
        }
    }

    /// Convert to 15D vector for export: [Δp_x, Δp_y, Δp_z, Δv_x, Δv_y, Δv_z, Δw_x, Δw_y, Δw_z, b_w_x, b_w_y, b_w_z, b_a_x, b_a_y, b_a_z]
    pub fn to_vector15(&self) -> SVector<f64, 15> {
        SVector::from([
            self.delta_p.x, // 0
            self.delta_p.y, // 1
            self.delta_p.z, // 2
            self.delta_v.x, // 3
            self.delta_v.y, // 4
            self.delta_v.z, // 5
            self.delta_w.x, // 6
            self.delta_w.y, // 7
            self.delta_w.z, // 8
            self.bias_w.x,  // 9
            self.bias_w.y,  // 10
            self.bias_w.z,  // 11
            self.bias_a.x,  // 12
            self.bias_a.y,  // 13
            self.bias_a.z,  // 14
        ])
    }

    /// Reset collector
    pub fn reset(&mut self) {
        self.delta_p = Vector3::zeros();
        self.delta_v = Vector3::zeros();
        self.delta_w = Vector3::zeros();
        self.bias_w = Vector3::zeros();
        self.bias_a = Vector3::zeros();
    }
}

impl Default for ImuPreintegrationCollector {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_collector_to_vector15() {
        let mut collector = ImuPreintegrationCollector::new();
        collector.delta_p = Vector3::new(1.0, 2.0, 3.0);
        collector.delta_v = Vector3::new(0.1, 0.2, 0.3);
        collector.delta_w = Vector3::new(0.01, 0.02, 0.03);
        collector.bias_w = Vector3::new(0.001, 0.002, 0.003);
        collector.bias_a = Vector3::new(0.01, 0.02, 0.03);

        let vec = collector.to_vector15();
        assert_eq!(vec[0], 1.0);
        assert_eq!(vec[3], 0.1);
        assert_eq!(vec[6], 0.01);
        assert_eq!(vec[9], 0.001);
        assert_eq!(vec[12], 0.01);
    }
}
