use super::state::Estimator;
use crate::debug_log;
use crate::estimator::Frame;
use crate::types::Matrix4x4;
use image::GrayImage;
use std::io::Write;

impl Estimator {
    /// Helper: set the current frame index on the attached viewer, if any.
    pub fn set_viewer_frame(&mut self, frame_id: i64) {
        if let Some(v) = &mut self.viewer {
            v.set_frame(frame_id);
        }
    }

    /// Visualize tracking results: stereo images with tracked features.
    pub fn view_patch_tracking_results(
        &mut self,
        frame: &Frame,
        left_img: &GrayImage,
        right_img: &GrayImage,
        width: u32,
        height: u32,
    ) {
        if let Some(v) = &mut self.viewer {
            // Collect pixel coordinates into temporary vectors so we can pass slices.
            let left_points: Vec<(usize, [f32; 2])> = frame
                .left_features()
                .iter()
                .map(|f| (f.feature_id, f.pixel_coord))
                .collect();
            let right_points: Vec<(usize, [f32; 2])> = frame
                .right_features()
                .iter()
                .map(|f| (f.feature_id, f.pixel_coord))
                .collect();

            v.log_image_with_features_colored(
                left_img.as_raw(),
                width,
                height,
                &left_points,
                "stereo/left",
            );
            v.log_image_with_features_colored(
                right_img.as_raw(),
                width,
                height,
                &right_points,
                "stereo/right",
            );
        }
    }

    pub fn view_motion_tracking_results(&mut self, T_W_B: &Matrix4x4) {
        if let Some(v) = &mut self.viewer {
            let pose_path = "pose_current".to_string();
            v.log_pose(*T_W_B, pose_path.as_str());

            let width = self.config.camera.image_width;
            let height = self.config.camera.image_height;

            // Log left camera frustum at the pose location (left camera is at the pose)
            let left_focal_length = self.config.camera.left_intrinsics[0] as f32;
            let left_cam_path = format!("{}_left", pose_path);
            let T_W_Cl = T_W_B * self.T_B_Cl;
            v.log_pose(T_W_Cl, left_cam_path.as_str());
            v.log_camera_frustum(
                left_focal_length,
                width,
                height,
                left_cam_path.as_str(),
                0.4,
            );
        }
    }

    /// Visualize optimization results: map points, keyframe poses, and camera frustums.
    pub fn view_optimization_results(&mut self, timestamp_ns: i64) {
        // History of keyframe poses with timestamps (update even without viewer for saving)
        let keyframe_poses = self.backend.sliding_window.get_keyframe_poses();
        if let Some(&mat) = keyframe_poses.last() {
            self.trajectory.push((timestamp_ns, mat));
        }

        // Viewer-only visualization (skip if no viewer)
        if let Some(v) = &mut self.viewer {
            // Map points
            let colored_points: Vec<(usize, [f32; 3])> = self
                .backend.sliding_window
                .map_points
                .iter()
                .map(|(&feature_id, &point)| (feature_id, point))
                .collect();
            v.log_points_colored(&colored_points, "map/points");

            // Keyframe poses with left and right camera frustrums
            let system_poses = self.backend.sliding_window.get_keyframe_poses();
            for (pose_id, T_W_B) in system_poses.iter().enumerate() {
                // pose is T_W_B
                let pose_path = format!("pose_{}", pose_id);
                v.log_pose(*T_W_B, pose_path.as_str());

                let width = self.config.camera.image_width;
                let height = self.config.camera.image_height;

                // Log left camera frustum at the pose location (left camera is at the pose)
                let left_focal_length = self.config.camera.left_intrinsics[0] as f32;
                let left_cam_path = format!("{}_left", pose_path);
                let T_W_Cl = T_W_B * self.T_B_Cl;
                v.log_pose(T_W_Cl, left_cam_path.as_str());
                let size = 0.2; // if pose_id == system_poses.len() - 1 { 0.5 } else { 0.2 };
                v.log_camera_frustum(
                    left_focal_length,
                    width,
                    height,
                    left_cam_path.as_str(),
                    size,
                );

                // Log right camera pose and frustum for last pose
                // Removed for now as it made too much clutter
                /*
                if pose_id == system_poses.len() - 1 {
                    let right_cam_path = format!("{}_right", pose_path);
                    let right_focal_length = self.config.camera.right_intrinsics[0] as f32;
                    let T_W_Cr = T_W_B * self.T_B_Cr;
                    v.log_pose(T_W_Cr, right_cam_path.as_str());
                    v.log_camera_frustum(right_focal_length, width, height, right_cam_path.as_str(), size);
                }
                */
            }

            // Display trajectory as a continuous 3D path
            let trajectory_poses: Vec<Matrix4x4> =
                self.trajectory.iter().map(|(_, pose)| *pose).collect();
            v.log_trajectory(&trajectory_poses, "trajectory/path");

            // Visualize loop closure constraints (global drift correction)
            let loop_closures = &self.backend.sliding_window.loop_closure_constraints;
            if !loop_closures.is_empty() {
                v.log_loop_closure(loop_closures, "loop_closure/edges");
            }
        }
    }

    /// Visualize IMU data: raw measurements, bias-corrected data, and harmonic decomposition
    pub fn view_imu_results(
        &mut self,
        imu_data: &[crate::datasets::ImuData],
        processed_accel: &[[f32; 3]],
        processed_gyro: &[[f32; 3]],
        timestamp_ns: i64,
    ) {
        if imu_data.is_empty() {
            return;
        }

        debug_log!(
            "[Estimator] Logging {} IMU samples to Rerun viewer",
            imu_data.len()
        );

        // Convert IMU data to format for visualization (don't hold mutable borrow of self)
        let mut raw_accel = Vec::with_capacity(imu_data.len());
        let mut raw_gyro = Vec::with_capacity(imu_data.len());

        for imu_sample in imu_data {
            raw_accel.push([
                imu_sample.accel[0] as f32,
                imu_sample.accel[1] as f32,
                imu_sample.accel[2] as f32,
            ]);
            raw_gyro.push([
                imu_sample.gyro[0] as f32,
                imu_sample.gyro[1] as f32,
                imu_sample.gyro[2] as f32,
            ]);
        }

        // Convert to f32 slices for visualization
        let raw_accel_f32: Vec<[f32; 3]> = raw_accel;
        let raw_gyro_f32: Vec<[f32; 3]> = raw_gyro;
        let processed_accel_f32: Vec<[f32; 3]> = processed_accel.to_vec();
        let processed_gyro_f32: Vec<[f32; 3]> = processed_gyro.to_vec();

        // Compute decomposition before getting viewer mutable borrow
        let decomp_result =
            self.compute_imu_decomposition(&processed_accel_f32, &processed_gyro_f32);
        let fundamental_freq = self.estimate_fundamental_frequency(&processed_gyro_f32);

        // Log denoising filter quality
        let _filter_quality = self.imu_processor.denoise_filter.quality();
        debug_log!(
            "[DENOISE] Frame {}: quality={:.2}, samples={}",
            self.frame_count,
            _filter_quality,
            imu_data.len()
        );

        // Log f0 to CSV file
        if let Ok(mut writer) = self.f0_log_writer.lock() {
            if let Some(ref mut f) = *writer {
                let _ = writeln!(
                    f,
                    "{},{},{:.2}",
                    self.frame_count, timestamp_ns, fundamental_freq
                );
                let _ = f.flush();
            }
        }

        // Log gyroscope spectrum (RMS values per axis) to CSV file
        let gyro_x_rms = (processed_gyro_f32.iter().map(|g| g[0] * g[0]).sum::<f32>()
            / processed_gyro_f32.len() as f32)
            .sqrt();
        let gyro_y_rms = (processed_gyro_f32.iter().map(|g| g[1] * g[1]).sum::<f32>()
            / processed_gyro_f32.len() as f32)
            .sqrt();
        let gyro_z_rms = (processed_gyro_f32.iter().map(|g| g[2] * g[2]).sum::<f32>()
            / processed_gyro_f32.len() as f32)
            .sqrt();
        let gyro_mag_rms = (processed_gyro_f32
            .iter()
            .map(|g| g[0] * g[0] + g[1] * g[1] + g[2] * g[2])
            .sum::<f32>()
            / processed_gyro_f32.len() as f32)
            .sqrt();

        if let Ok(mut writer) = self.spectrum_log_writer.lock() {
            if let Some(ref mut f) = *writer {
                let _ = writeln!(
                    f,
                    "{},{},{:.6},{:.6},{:.6},{:.6}",
                    self.frame_count,
                    timestamp_ns,
                    gyro_x_rms,
                    gyro_y_rms,
                    gyro_z_rms,
                    gyro_mag_rms
                );
                let _ = f.flush();
            }
        }

        // Now get mutable borrow for viewer
        if let Some(v) = &mut self.viewer {
            // Log raw IMU data
            v.log_imu_raw(timestamp_ns, &raw_accel_f32, &raw_gyro_f32, "imu/raw");

            // Log processed (bias-corrected) IMU data
            v.log_imu_processed(
                timestamp_ns,
                &processed_accel_f32,
                &processed_gyro_f32,
                "imu/processed",
            );

            // Log harmonic decomposition if available
            if let Some((gravity, vibration)) = decomp_result {
                // Log harmonic decomposition (gravity, bias, harmonics)
                let bias_accel = [
                    self.imu_processor.bias_estimator.accel_bias[0] as f32,
                    self.imu_processor.bias_estimator.accel_bias[1] as f32,
                    self.imu_processor.bias_estimator.accel_bias[2] as f32,
                ];
                let bias_gyro = [
                    self.imu_processor.bias_estimator.gyro_bias[0] as f32,
                    self.imu_processor.bias_estimator.gyro_bias[1] as f32,
                    self.imu_processor.bias_estimator.gyro_bias[2] as f32,
                ];

                v.log_imu_harmonics(
                    timestamp_ns,
                    gravity,
                    bias_accel,
                    bias_gyro,
                    &[vibration],
                    "imu/harmonics",
                );

                // Log signal quality metrics (including fundamental frequency f0)
                let snr_values = [1.0_f32, 1.0_f32, 1.0_f32]; // Placeholder
                let rms_values = [0.5_f32, 0.5_f32, 0.5_f32]; // Placeholder
                let peak_values = [1.0_f32, 1.0_f32, 1.0_f32]; // Placeholder
                v.log_imu_signal_quality(
                    timestamp_ns,
                    snr_values,
                    rms_values,
                    peak_values,
                    "running",
                    fundamental_freq,
                    "imu/quality",
                );
            }
        }
    }
}
