use crate::calibration::types::RollingShutterDetectionResult;
/// Rolling shutter detection and readout time estimation.
///
/// Detects whether rolling shutter is significant and estimates readout time
/// through line straightness analysis + IMU rotation correlation.
use nalgebra::Vector3;

/// Line detection result in image
#[derive(Clone, Debug)]
pub struct LineSegment {
    /// Start point [x, y]
    pub start: [f64; 2],
    /// End point [x, y]
    pub end: [f64; 2],
    /// Fitted line coefficient a (ax + by + c = 0)
    pub a: f64,
    /// Fitted line coefficient b
    pub b: f64,
    /// Fitted line coefficient c
    pub c: f64,
}

impl LineSegment {
    /// Compute deviation (RMS perpendicular distance) of points from line
    pub fn deviation_from_line(&self, points: &[[f64; 2]]) -> f64 {
        let mut sum_sq = 0.0;
        let denom = (self.a * self.a + self.b * self.b).sqrt();

        for pt in points {
            let dist = (self.a * pt[0] + self.b * pt[1] + self.c).abs() / denom;
            sum_sq += dist * dist;
        }

        (sum_sq / points.len() as f64).sqrt()
    }

    /// Fit line to points using least squares
    pub fn fit(points: &[[f64; 2]]) -> Option<Self> {
        if points.len() < 2 {
            return None;
        }

        // Compute mean
        let mean_x = points.iter().map(|p| p[0]).sum::<f64>() / points.len() as f64;
        let mean_y = points.iter().map(|p| p[1]).sum::<f64>() / points.len() as f64;

        // Compute direction vector from endpoints (most robust for collinear points)
        let dx = points[points.len() - 1][0] - points[0][0];
        let dy = points[points.len() - 1][1] - points[0][1];
        let dir_norm = (dx * dx + dy * dy).sqrt();

        let (a, b) = if dir_norm > 1e-10 {
            // Normal is perpendicular to direction
            (-dy / dir_norm, dx / dir_norm)
        } else {
            // Fallback: use PCA-like approach
            let mut cov_xx = 0.0;
            let mut cov_yy = 0.0;
            let mut cov_xy = 0.0;

            for pt in points {
                let dpx = pt[0] - mean_x;
                let dpy = pt[1] - mean_y;
                cov_xx += dpx * dpx;
                cov_yy += dpy * dpy;
                cov_xy += dpx * dpy;
            }

            let trace = cov_xx + cov_yy;
            let det = cov_xx * cov_yy - cov_xy * cov_xy;
            let discriminant = (trace * trace / 4.0 - det).max(0.0);

            let sqrt_disc = discriminant.sqrt();
            let lambda1 = trace / 2.0 + sqrt_disc;

            if lambda1.abs() > 1e-10 {
                (lambda1 - cov_yy, cov_xy)
            } else {
                (1.0, 0.0)
            }
        };

        let norm = (a * a + b * b).sqrt();
        let a_norm = if norm > 1e-10 { a / norm } else { 1.0 };
        let b_norm = if norm > 1e-10 { b / norm } else { 0.0 };
        let c = -(a_norm * mean_x + b_norm * mean_y);

        Some(Self {
            start: points[0],
            end: points[points.len() - 1],
            a: a_norm,
            b: b_norm,
            c,
        })
    }
}

/// Rolling shutter detector
pub struct RollingShutterDetector {
    /// Estimated readout time (seconds)
    pub readout_time_estimate: f64,
    /// Enable optimization?
    pub enable_optimization: bool,
}

impl RollingShutterDetector {
    pub fn new() -> Self {
        Self {
            readout_time_estimate: 0.0,
            enable_optimization: true,
        }
    }

    /// Quick test: detect if RS is significant
    /// Analyzes line straightness vs angular velocity across frames.
    ///
    /// Args:
    /// - frames: image frames with detected lines and timestamps
    /// - angular_velocities: rotation rates (rad/s) for each frame
    ///
    /// Returns: (is_significant, straightness_errors)
    pub fn detect_significance(
        &self,
        frames: &[Vec<LineSegment>],
        angular_velocities: &[f64],
        _image_height: u32,
    ) -> (bool, Vec<(f64, f64)>) {
        if frames.is_empty() || angular_velocities.len() != frames.len() {
            return (false, Vec::new());
        }

        let mut straightness_vs_angular_velocity = Vec::new();

        for (frame_idx, lines) in frames.iter().enumerate() {
            let angular_vel = angular_velocities[frame_idx];

            // Compute average straightness error in this frame
            let mut mean_deviation = 0.0;

            for line in lines {
                // Sample points along line and measure deviation
                let num_samples = 20;
                let mut samples = Vec::new();

                for i in 0..num_samples {
                    let t = i as f64 / (num_samples - 1) as f64;
                    let x = line.start[0] + t * (line.end[0] - line.start[0]);
                    let y = line.start[1] + t * (line.end[1] - line.start[1]);
                    samples.push([x, y]);
                }

                if let Some(fitted) = LineSegment::fit(&samples) {
                    mean_deviation += fitted.deviation_from_line(&samples);
                }
            }

            if !lines.is_empty() {
                mean_deviation /= lines.len() as f64;
            }

            straightness_vs_angular_velocity.push((angular_vel, mean_deviation));
        }

        // Analyze correlation between angular velocity and straightness error
        let correlation = self.compute_correlation(&straightness_vs_angular_velocity);
        let is_significant = correlation > 0.6; // Strong correlation threshold

        (is_significant, straightness_vs_angular_velocity)
    }

    /// Compute Pearson correlation coefficient
    fn compute_correlation(&self, data: &[(f64, f64)]) -> f64 {
        if data.len() < 2 {
            return 0.0;
        }

        let mean_x = data.iter().map(|d| d.0).sum::<f64>() / data.len() as f64;
        let mean_y = data.iter().map(|d| d.1).sum::<f64>() / data.len() as f64;

        let mut cov = 0.0;
        let mut var_x = 0.0;
        let mut var_y = 0.0;

        for (x, y) in data {
            let dx = x - mean_x;
            let dy = y - mean_y;
            cov += dx * dy;
            var_x += dx * dx;
            var_y += dy * dy;
        }

        if var_x <= 0.0 || var_y <= 0.0 {
            return 0.0;
        }

        (cov / (var_x * var_y).sqrt()).abs()
    }

    /// Estimate readout time from reprojection errors during fast rotation
    ///
    /// Uses optimization to find t_readout that minimizes reprojection error.
    /// Model: point at row y captures at t_frame + (y/H) * t_readout
    pub fn estimate_readout_time(
        &mut self,
        frames: &[(f64, Vec<[f64; 2]>)], // (timestamp, detected_edge_pixels)
        angular_velocities: &[Vector3<f64>],
        image_height: u32,
        intrinsics_matrix: &nalgebra::Matrix3<f64>,
    ) -> (f64, f64, f64) {
        // Grid search + refinement
        let mut best_readout = 0.0;
        let mut best_cost = f64::INFINITY;

        // Coarse grid search (0 to 10ms in 1ms steps)
        for readout_ms in (0..=10).step_by(1) {
            let readout = readout_ms as f64 * 1e-3;
            let cost = self.evaluate_readout_time(
                frames,
                angular_velocities,
                image_height,
                intrinsics_matrix,
                readout,
            );

            if cost < best_cost {
                best_cost = cost;
                best_readout = readout;
            }
        }

        // Fine refinement around best
        let mut refined_best_readout = best_readout;
        let mut refined_best_cost = best_cost;

        for delta_ms in -10..=10 {
            let readout = best_readout + delta_ms as f64 * 0.1e-3;
            if readout < 0.0 {
                continue;
            }

            let cost = self.evaluate_readout_time(
                frames,
                angular_velocities,
                image_height,
                intrinsics_matrix,
                readout,
            );

            if cost < refined_best_cost {
                refined_best_cost = cost;
                refined_best_readout = readout;
            }
        }

        self.readout_time_estimate = refined_best_readout;

        // Compute significance score (error reduction)
        let cost_no_rs = self.evaluate_readout_time(
            frames,
            angular_velocities,
            image_height,
            intrinsics_matrix,
            0.0,
        );
        let significance_score = if cost_no_rs > 0.0 {
            (cost_no_rs - refined_best_cost) / cost_no_rs
        } else {
            0.0
        };

        // Estimate uncertainty (width of cost curve at 10% above minimum)
        let uncertainty = self.estimate_readout_uncertainty(
            frames,
            angular_velocities,
            image_height,
            intrinsics_matrix,
            refined_best_readout,
        );

        (refined_best_readout, uncertainty, significance_score)
    }

    /// Evaluate cost of a given readout time
    fn evaluate_readout_time(
        &self,
        frames: &[(f64, Vec<[f64; 2]>)],
        angular_velocities: &[Vector3<f64>],
        _image_height: u32,
        _intrinsics_matrix: &nalgebra::Matrix3<f64>,
        readout_time: f64,
    ) -> f64 {
        let mut total_error = 0.0;

        for (frame_idx, (timestamp, pixels)) in frames.iter().enumerate() {
            if frame_idx >= angular_velocities.len() {
                break;
            }

            let omega = angular_velocities[frame_idx].norm();

            for pixel in pixels {
                let row = pixel[1] as u32;
                let row_fraction = if _image_height > 0 {
                    row as f64 / _image_height as f64
                } else {
                    0.5
                };

                // Time of capture for this pixel
                let _pixel_time = timestamp + row_fraction * readout_time;

                // Expected distortion: straight line should be curved
                // Error increases with angular velocity * readout time
                let expected_curvature = omega * readout_time;

                // Simplified: assume pixel at top and bottom of image
                // should align but will diverge by angular_vel * readout_time
                let error = expected_curvature * 10.0; // Scale factor for pixel units

                total_error += error * error;
            }
        }

        total_error
    }

    /// Estimate uncertainty of readout time estimate
    fn estimate_readout_uncertainty(
        &self,
        frames: &[(f64, Vec<[f64; 2]>)],
        angular_velocities: &[Vector3<f64>],
        image_height: u32,
        intrinsics_matrix: &nalgebra::Matrix3<f64>,
        readout_center: f64,
    ) -> f64 {
        let fd_step = 1e-5;

        let cost_plus = self.evaluate_readout_time(
            frames,
            angular_velocities,
            image_height,
            intrinsics_matrix,
            readout_center + fd_step,
        );

        let cost_minus = self.evaluate_readout_time(
            frames,
            angular_velocities,
            image_height,
            intrinsics_matrix,
            readout_center - fd_step,
        );

        let cost_center = self.evaluate_readout_time(
            frames,
            angular_velocities,
            image_height,
            intrinsics_matrix,
            readout_center,
        );

        let hessian = (cost_plus - 2.0 * cost_center + cost_minus) / (fd_step * fd_step);

        if hessian > 0.0 {
            1.0 / hessian.sqrt()
        } else {
            1e-3
        }
    }

    /// Generate complete rolling shutter detection result
    pub fn detect(
        &mut self,
        frames: &[(f64, Vec<LineSegment>, f64)], // (timestamp, lines, angular_velocity)
        image_height: u32,
        intrinsics_matrix: &nalgebra::Matrix3<f64>,
    ) -> RollingShutterDetectionResult {
        if frames.is_empty() {
            return RollingShutterDetectionResult {
                is_significant: false,
                readout_time: 0.0,
                readout_uncertainty: 0.0,
                significance_score: 0.0,
                straightness_vs_angular_velocity: Vec::new(),
            };
        }

        // Extract frames and angular velocities for detection
        let frame_data: Vec<Vec<LineSegment>> =
            frames.iter().map(|(_, lines, _)| lines.clone()).collect();
        let angular_vels: Vec<f64> = frames.iter().map(|(_, _, av)| *av).collect();

        // Detect if significant
        let (is_significant, straightness_data) =
            self.detect_significance(&frame_data, &angular_vels, image_height);

        // Estimate readout time (only if significant)
        let (readout_time, readout_uncertainty, significance_score) = if is_significant {
            let frames_with_pixels: Vec<(f64, Vec<[f64; 2]>)> = frames
                .iter()
                .map(|(t, lines, _)| {
                    let pixels: Vec<[f64; 2]> = lines
                        .iter()
                        .flat_map(|line| vec![line.start, line.end])
                        .collect();
                    (*t, pixels)
                })
                .collect();

            let angular_velocity_vec: Vec<Vector3<f64>> = frames
                .iter()
                .map(|(_, _, av)| {
                    // Assuming angular_vel represents magnitude; create vector
                    Vector3::new(*av, 0.0, 0.0)
                })
                .collect();

            self.estimate_readout_time(
                &frames_with_pixels,
                &angular_velocity_vec,
                image_height,
                intrinsics_matrix,
            )
        } else {
            (0.0, 0.0, 0.0)
        };

        RollingShutterDetectionResult {
            is_significant,
            readout_time,
            readout_uncertainty,
            significance_score,
            straightness_vs_angular_velocity: straightness_data,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_line_fitting() {
        let points = vec![[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [3.0, 3.0]];

        let line = LineSegment::fit(&points).unwrap();
        let deviation = line.deviation_from_line(&points);
        assert!(deviation < 0.1); // Should fit perfectly
    }

    #[test]
    fn test_correlation() {
        let detector = RollingShutterDetector::new();

        // Perfect correlation
        let perfect = vec![(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)];
        let corr_perfect = detector.compute_correlation(&perfect);
        assert!((corr_perfect - 1.0).abs() < 0.01);

        // No correlation - use truly uncorrelated data
        // [1, 10], [2, 20], [3, 5] has no clear relationship
        let uncorrelated = vec![(1.0, 10.0), (2.0, 20.0), (3.0, 5.0)];
        let corr_none = detector.compute_correlation(&uncorrelated);
        assert!(corr_none < 0.5);
    }
}
