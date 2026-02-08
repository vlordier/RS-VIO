//! # IMU Measurement Buffer
//!
//! Circular buffer for IMU measurements with:
//! - Fixed-size ring buffer for memory efficiency
//! - Timestamp-based retrieval
//! - Linear interpolation for arbitrary timestamps
//! - Out-of-order insertion handling
//!
//! ## Usage
//!
//! ```rust,ignore
//! let mut buffer = ImuBuffer::new(1000); // 1000 measurements capacity
//! buffer.add(imu_measurement);
//! let measurements = buffer.get_range(t_start, t_end);
//! let interpolated = buffer.interpolate_at(t_query);
//! ```

use crate::datasets::ImuData;
use std::collections::VecDeque;

/// Circular buffer for IMU measurements
pub struct ImuBuffer {
    /// Ring buffer of measurements (sorted by timestamp)
    measurements: VecDeque<ImuData>,

    /// Maximum buffer capacity
    capacity: usize,

    /// Oldest timestamp in buffer
    oldest_time: Option<i64>,

    /// Newest timestamp in buffer
    newest_time: Option<i64>,
}

impl ImuBuffer {
    /// Create new IMU buffer with given capacity
    pub fn new(capacity: usize) -> Self {
        Self {
            measurements: VecDeque::with_capacity(capacity),
            capacity,
            oldest_time: None,
            newest_time: None,
        }
    }

    /// Add IMU measurement to buffer
    ///
    /// Handles out-of-order insertions by sorting
    pub fn add(&mut self, imu: ImuData) {
        // Update time bounds
        if let Some(oldest) = self.oldest_time {
            if imu.timestamp < oldest {
                self.oldest_time = Some(imu.timestamp);
            }
        } else {
            self.oldest_time = Some(imu.timestamp);
        }

        if let Some(newest) = self.newest_time {
            if imu.timestamp > newest {
                self.newest_time = Some(imu.timestamp);
            }
        } else {
            self.newest_time = Some(imu.timestamp);
        }

        // Find insertion position (binary search for efficiency)
        let insert_pos = self
            .measurements
            .binary_search_by_key(&imu.timestamp, |m| m.timestamp)
            .unwrap_or_else(|pos| pos);

        // Insert at correct position
        self.measurements.insert(insert_pos, imu);

        // Remove oldest if at capacity
        while self.measurements.len() > self.capacity {
            if let Some(_removed) = self.measurements.pop_front() {
                // Update oldest time
                if let Some(front) = self.measurements.front() {
                    self.oldest_time = Some(front.timestamp);
                } else {
                    self.oldest_time = None;
                }
            }
        }
    }

    /// Get all measurements in time range [t_start, t_end]
    pub fn get_range(&self, t_start: i64, t_end: i64) -> Vec<ImuData> {
        // Binary search for start position (first element >= t_start)
        let start = self
            .measurements
            .binary_search_by_key(&t_start, |m| m.timestamp)
            .unwrap_or_else(|pos| pos);
        // Binary search for end position (first element > t_end)
        let end = self
            .measurements
            .binary_search_by_key(&t_end.saturating_add(1), |m| m.timestamp)
            .unwrap_or_else(|pos| pos);
        self.measurements.range(start..end).cloned().collect()
    }

    /// Get measurements strictly between two timestamps
    pub fn get_between(&self, t_start: i64, t_end: i64) -> Vec<ImuData> {
        // Binary search for start position (first element > t_start)
        let start = self
            .measurements
            .binary_search_by_key(&t_start.saturating_add(1), |m| m.timestamp)
            .unwrap_or_else(|pos| pos);
        // Binary search for end position (first element >= t_end)
        let end = self
            .measurements
            .binary_search_by_key(&t_end, |m| m.timestamp)
            .unwrap_or_else(|pos| pos);
        self.measurements.range(start..end).cloned().collect()
    }

    /// Interpolate IMU measurement at arbitrary timestamp
    ///
    /// Returns None if timestamp is outside buffer range
    pub fn interpolate_at(&self, t: i64) -> Option<ImuData> {
        if self.measurements.is_empty() {
            return None;
        }

        // Check if timestamp is in range
        let oldest = self.measurements.front()?.timestamp;
        let newest = self.measurements.back()?.timestamp;

        if t < oldest || t > newest {
            return None;
        }

        // Find bracketing measurements
        let idx = self
            .measurements
            .binary_search_by_key(&t, |m| m.timestamp)
            .unwrap_or_else(|pos| pos);

        // Exact match
        if idx < self.measurements.len() && self.measurements[idx].timestamp == t {
            return Some(self.measurements[idx].clone());
        }

        // Interpolate between measurements
        if idx == 0 || idx >= self.measurements.len() {
            return None;
        }

        let m0 = &self.measurements[idx - 1];
        let m1 = &self.measurements[idx];

        let dt = (m1.timestamp - m0.timestamp) as f64;
        if dt <= 0.0 {
            // Duplicate timestamps — return the earlier measurement
            return Some(self.measurements[idx - 1].clone());
        }
        let alpha = ((t - m0.timestamp) as f64) / dt;

        // Linear interpolation
        let gyro = [
            m0.gyro[0] + alpha * (m1.gyro[0] - m0.gyro[0]),
            m0.gyro[1] + alpha * (m1.gyro[1] - m0.gyro[1]),
            m0.gyro[2] + alpha * (m1.gyro[2] - m0.gyro[2]),
        ];

        let accel = [
            m0.accel[0] + alpha * (m1.accel[0] - m0.accel[0]),
            m0.accel[1] + alpha * (m1.accel[1] - m0.accel[1]),
            m0.accel[2] + alpha * (m1.accel[2] - m0.accel[2]),
        ];

        Some(ImuData {
            timestamp: t,
            gyro,
            accel,
        })
    }

    /// Get most recent measurement
    pub fn latest(&self) -> Option<&ImuData> {
        self.measurements.back()
    }

    /// Get oldest measurement
    pub fn oldest(&self) -> Option<&ImuData> {
        self.measurements.front()
    }

    /// Clear all measurements
    pub fn clear(&mut self) {
        self.measurements.clear();
        self.oldest_time = None;
        self.newest_time = None;
    }

    /// Get number of measurements in buffer
    pub fn len(&self) -> usize {
        self.measurements.len()
    }

    /// Check if buffer is empty
    pub fn is_empty(&self) -> bool {
        self.measurements.is_empty()
    }

    /// Get time span covered by buffer [oldest, newest]
    pub const fn time_span(&self) -> Option<(i64, i64)> {
        if let (Some(oldest), Some(newest)) = (self.oldest_time, self.newest_time) {
            Some((oldest, newest))
        } else {
            None
        }
    }

    /// Remove measurements older than given timestamp
    pub fn trim_before(&mut self, t: i64) {
        while let Some(front) = self.measurements.front() {
            if front.timestamp < t {
                self.measurements.pop_front();
            } else {
                break;
            }
        }

        // Update oldest time
        if let Some(front) = self.measurements.front() {
            self.oldest_time = Some(front.timestamp);
        } else {
            self.oldest_time = None;
            self.newest_time = None;
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    fn create_imu(timestamp: i64, gyro_val: f64, accel_val: f64) -> ImuData {
        ImuData {
            timestamp,
            gyro: [gyro_val, gyro_val, gyro_val],
            accel: [accel_val, accel_val, accel_val],
        }
    }

    #[test]
    fn test_buffer_add_and_retrieve() {
        let mut buffer = ImuBuffer::new(100);

        buffer.add(create_imu(1000, 0.1, 9.8));
        buffer.add(create_imu(2000, 0.2, 9.9));
        buffer.add(create_imu(3000, 0.3, 10.0));

        assert_eq!(buffer.len(), 3);

        let range = buffer.get_range(1000, 3000);
        assert_eq!(range.len(), 3);
    }

    #[test]
    fn test_buffer_out_of_order() {
        let mut buffer = ImuBuffer::new(100);

        buffer.add(create_imu(3000, 0.3, 10.0));
        buffer.add(create_imu(1000, 0.1, 9.8));
        buffer.add(create_imu(2000, 0.2, 9.9));

        // Should be sorted
        let measurements: Vec<_> = buffer.measurements.iter().map(|m| m.timestamp).collect();
        assert_eq!(measurements, vec![1000, 2000, 3000]);
    }

    #[test]
    fn test_buffer_interpolation() {
        let mut buffer = ImuBuffer::new(100);

        buffer.add(create_imu(1000, 0.0, 10.0));
        buffer.add(create_imu(2000, 1.0, 20.0));

        // Interpolate at midpoint
        let interp = buffer.interpolate_at(1500).unwrap();
        assert!((interp.gyro[0] - 0.5).abs() < 1e-6);
        assert!((interp.accel[0] - 15.0).abs() < 1e-6);
    }

    #[test]
    fn test_buffer_capacity() {
        let mut buffer = ImuBuffer::new(3);

        buffer.add(create_imu(1000, 0.1, 9.8));
        buffer.add(create_imu(2000, 0.2, 9.9));
        buffer.add(create_imu(3000, 0.3, 10.0));
        buffer.add(create_imu(4000, 0.4, 10.1));

        // Should only have 3 measurements (oldest dropped)
        assert_eq!(buffer.len(), 3);
        assert_eq!(buffer.oldest().unwrap().timestamp, 2000);
        assert_eq!(buffer.latest().unwrap().timestamp, 4000);
    }

    #[test]
    fn test_trim_before() {
        let mut buffer = ImuBuffer::new(100);

        buffer.add(create_imu(1000, 0.1, 9.8));
        buffer.add(create_imu(2000, 0.2, 9.9));
        buffer.add(create_imu(3000, 0.3, 10.0));

        buffer.trim_before(2500);

        assert_eq!(buffer.len(), 1);
        assert_eq!(buffer.oldest().unwrap().timestamp, 3000);
    }
}
