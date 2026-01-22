use super::state::Estimator;
use crate::calibration::online_intrinsics::CalibrationObservation;
use crate::debug_log;
use crate::estimator::Frame;

impl Estimator {
    /// Update camera intrinsics from online refiner if available
    pub fn update_intrinsics_from_refiner(&mut self) {
        if let Some(refiner) = &self.intrinsics_refiner {
            if refiner.is_converged() {
                debug_log!(
                    "[Estimator] Intrinsics converged after {} observations",
                    refiner.total_observations()
                );
            }
        }
    }

    /// Update camera extrinsics from online extrinsic calibrator if available
    ///
    /// Called periodically to apply calibrated IMU-to-camera extrinsics.
    /// The calibrator accumulates pose measurements and refines T_BC.
    pub fn update_extrinsics_from_calibrator(&mut self) {
        let measurement_count = self.imu_processor.extrinsic_calibrator.measurement_count();

        // Only apply after collecting enough measurements
        if measurement_count < 100 {
            return;
        }

        // Run calibration periodically (every 100 measurements after initial collection)
        if measurement_count % 100 == 0 {
            let _error = self.imu_processor.extrinsic_calibrator.calibrate_iteration();
            debug_log!(
                "[Estimator] IMU extrinsic calibration iteration {}, error: {:.6} rad",
                self.imu_processor.extrinsic_calibrator.iterations(),
                _error
            );
        }

        // Apply calibrated extrinsics periodically
        if self.imu_processor.extrinsic_calibrator.iterations() > 0 && measurement_count % 500 == 0 {
            let calibrated_T_BC = self.imu_processor.extrinsic_calibrator.get_extrinsics();

            // Update the stored extrinsics
            self.T_B_Cl = calibrated_T_BC;

            // Also update in all frame states if window has frames
            for frame in self.backend.sliding_window.keyframes_mut() {
                frame.state.T_B_Cl = calibrated_T_BC;
            }

            debug_log!(
                "[Estimator] Applied calibrated extrinsics: T_B_Cl updated (iterations: {})",
                self.extrinsic_calibrator.iterations()
            );
        }
    }

    /// Add observations to intrinsics refiner for self-calibration
    pub fn add_intrinsics_observations(&mut self, frame: &Frame) {
        if let Some(ref mut refiner) = self.intrinsics_refiner {
            let observations: Vec<CalibrationObservation> = frame
                .left_features
                .iter()
                .filter(|f| f.undistorted_coord[0] >= 0.0 && f.undistorted_coord[1] >= 0.0)
                .map(|f| CalibrationObservation {
                    observation_2d: nalgebra::Vector2::new(
                        f.undistorted_coord[0] as crate::types::Float,
                        f.undistorted_coord[1] as crate::types::Float,
                    ),
                    ..Default::default()
                })
                .collect();
            if !observations.is_empty() {
                refiner.update(&observations);
            }
        }
    }
}
