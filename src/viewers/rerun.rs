use super::get_feature_color;
use super::Viewer;
use crate::datasets::config::VisualizationConfig;
use crate::debug_log;
use crate::traits::Convert;
use crate::types::{Array3, Float, Matrix3x3, Matrix4x4};
use crate::{Result, VIOError};
use image::{DynamicImage, ImageBuffer, Luma};
use rerun::components::Color;
use rerun::time::Timestamp;
use rerun::LineStrips3D;
use rerun::Pinhole;
use rerun::{RecordingStream, RecordingStreamBuilder};
use std::io::Cursor;

/// Basic RerunViewer implementation
pub struct RerunViewer {
    rec: Option<RecordingStream>,
    initialized: bool,
    frame_id: i64,
    timestamp_ns: i64,
    #[allow(dead_code)] // Reserved for future timestamp normalization
    first_timestamp_ns: Option<i64>,
    stream_name: String,
    startup_delay_ms: u64,
    log_axes: bool,
}

impl Default for RerunViewer {
    fn default() -> Self {
        Self::new()
    }
}

impl RerunViewer {
    #[allow(clippy::should_implement_trait)]
    pub fn default() -> Self {
        Self::new()
    }

    pub fn new() -> Self {
        Self::new_with_config(&VisualizationConfig::default())
    }

    pub fn new_with_config(config: &VisualizationConfig) -> Self {
        RerunViewer {
            rec: None,
            initialized: false,
            frame_id: 0,
            timestamp_ns: 0,
            first_timestamp_ns: None,
            stream_name: config.stream_name.clone(),
            startup_delay_ms: config.startup_delay_ms,
            log_axes: config.log_axes,
        }
    }

    /// Helper function to safely convert raw image data to JPEG bytes
    fn image_to_jpeg_bytes(
        &self,
        image: &[u8],
        width: u32,
        height: u32,
        entity_path: &str,
    ) -> Option<Vec<u8>> {
        // Validate image dimensions
        let expected_size = (width as usize) * (height as usize);
        let actual_size = image.len();

        if actual_size != expected_size {
            log::warn!(
                "[RerunViewer] Image size mismatch for {}: expected {} bytes ({}x{}), got {} bytes. Skipping.",
                entity_path, expected_size, width, height, actual_size
            );
            return None;
        }

        // Convert raw pixel data to DynamicImage
        let img_buffer = match ImageBuffer::<Luma<u8>, Vec<u8>>::from_raw(
            width,
            height,
            image.to_vec(),
        ) {
            Some(buffer) => buffer,
            None => {
                log::warn!(
                    "[RerunViewer] Failed to create image buffer from raw data for {} ({}x{}, {} bytes). Skipping.",
                    entity_path, width, height, actual_size
                );
                return None;
            },
        };

        let dynamic_img = DynamicImage::ImageLuma8(img_buffer);

        // Encode as JPEG
        let mut bytes: Vec<u8> = Vec::new();
        if let Err(e) = dynamic_img.write_to(&mut Cursor::new(&mut bytes), image::ImageFormat::Jpeg)
        {
            log::warn!(
                "[RerunViewer] Failed to encode image as JPEG for {}: {}. Skipping.",
                entity_path,
                e
            );
            return None;
        }

        Some(bytes)
    }
}

impl Viewer for RerunViewer {
    fn initialize(&mut self) -> Result<()> {
        // Spawn a new rerun viewer
        // This will start the rerun viewer application if it's not already running
        log::info!("[RerunViewer] Spawning rerun viewer...");
        let rec = RecordingStreamBuilder::new(self.stream_name.as_str())
            .spawn()
            .map_err(|e| {
                log::error!("[RerunViewer] Failed to spawn viewer: {}", e);
                VIOError::Viewer(e.to_string())
            })?;

        self.rec = Some(rec);
        self.initialized = true;

        // Give the viewer a moment to fully start up
        std::thread::sleep(std::time::Duration::from_millis(self.startup_delay_ms));

        // Set up coordinate system
        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", 0);
            if self.log_axes {
                match rec.log("origin", &rerun::ViewCoordinates::RDF()) {
                    Ok(_) => debug_log!("[RerunViewer] Successfully logged coordinate system"),
                    Err(e) => {
                        log::warn!("[RerunViewer] Failed to log coordinate system: {}", e);
                    },
                }

                let origin = [0.0, 0.0, 0.0];
                let origins = vec![origin, origin, origin];
                let vectors = vec![[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
                let colors = vec![
                    rerun::Color::from_rgb(255, 0, 0),
                    rerun::Color::from_rgb(0, 255, 0),
                    rerun::Color::from_rgb(0, 0, 255),
                ];

                match rec.log(
                    "origin/axes",
                    &rerun::Arrows3D::from_vectors(vectors)
                        .with_origins(origins)
                        .with_colors(colors),
                ) {
                    Ok(_) => debug_log!("[RerunViewer] Successfully logged axis arrows"),
                    Err(e) => log::warn!("[RerunViewer] Failed to log axis arrows: {}", e),
                }
            }
        }
        log::info!("[RerunViewer] Viewer initialized successfully");
        Ok(())
    }

    fn log_pose(&mut self, t_w_b: Matrix4x4, entity_path: &str) {
        if !self.initialized {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            let translation = Array3::from(t_w_b.fixed_view::<3, 1>(0, 3));
            let rotation = Matrix3x3::from(t_w_b.fixed_view::<3, 3>(0, 0));

            // Convert rotation matrix to quaternion
            let quat = matrix_to_quaternion(rotation.convert());
            let quaternion = rerun::Quaternion::from_xyzw([
                quat[0] as f32,
                quat[1] as f32,
                quat[2] as f32,
                quat[3] as f32,
            ]);

            if let Err(e) = rec.log(
                entity_path,
                &rerun::Transform3D::from_translation_rotation(
                    translation,
                    rerun::Rotation3D::Quaternion(rerun::components::RotationQuat(quaternion)),
                ),
            ) {
                log::warn!("[RerunViewer] Failed to log pose to {}: {}", entity_path, e);
            }
        }
    }

    fn log_image_raw(&mut self, image: &[u8], width: u32, height: u32, entity_path: &str) {
        if !self.initialized || image.is_empty() {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            // Convert image to JPEG bytes using helper
            let bytes = match self.image_to_jpeg_bytes(image, width, height, entity_path) {
                Some(b) => b,
                None => return, // Error already logged in helper
            };

            // Log using EncodedImage
            let rr_image = rerun::EncodedImage::from_file_contents(bytes);
            if let Err(e) = rec.log(entity_path, &rr_image) {
                let err_str = e.to_string();
                // Only warn if it's not a broken pipe (which can happen during rerun's internal retries)
                if !err_str.contains("Broken pipe") {
                    log::warn!(
                        "[RerunViewer] Failed to log image to {}: {} (frame: {})",
                        entity_path,
                        e,
                        self.frame_id
                    );
                }
                // Check if it's a connection error - if so, mark as not initialized
                if err_str.contains("Connection refused") {
                    log::error!("[RerunViewer] Connection lost, viewer may have closed");
                    self.initialized = false;
                }
            }
        }
    }

    fn log_image_equalized(&mut self, image: &[u8], width: u32, height: u32, entity_path: &str) {
        if !self.initialized || image.is_empty() {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            // Convert image to JPEG bytes using helper
            let bytes = match self.image_to_jpeg_bytes(image, width, height, entity_path) {
                Some(b) => b,
                None => return, // Error already logged in helper
            };

            // Log using EncodedImage
            let rr_image = rerun::EncodedImage::from_file_contents(bytes);
            match rec.log(entity_path, &rr_image) {
                Ok(_) => {
                    debug_log!(
                        "[RerunViewer] Successfully logged equalized image to {}",
                        entity_path
                    );
                },
                Err(e) => {
                    log::warn!(
                        "[RerunViewer] Failed to log equalized image to {}: {} (frame: {})",
                        entity_path,
                        e,
                        self.frame_id
                    );
                    // Check if it's a connection error - if so, mark as not initialized
                    if e.to_string().contains("Broken pipe")
                        || e.to_string().contains("Connection refused")
                    {
                        log::error!("[RerunViewer] Connection lost, viewer may have closed");
                        self.initialized = false;
                    }
                },
            }
        }
    }

    fn log_image_with_features(
        &mut self,
        image: &[u8],
        width: u32,
        height: u32,
        features: &[[f32; 2]],
        entity_path: &str,
    ) {
        if !self.initialized || image.is_empty() {
            return;
        }

        // Log the image first
        self.log_image_raw(image, width, height, entity_path);

        // Then log features
        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            // Log features as 2D points
            if !features.is_empty() {
                let points: Vec<[f32; 2]> = features.to_vec();

                if let Err(e) = rec.log(
                    format!("{}/features", entity_path).as_str(),
                    &rerun::Points2D::new(points),
                ) {
                    log::warn!("[RerunViewer] Failed to log features: {}", e);
                }
            }
        }
    }

    fn log_image_with_features_colored(
        &mut self,
        image: &[u8],
        width: u32,
        height: u32,
        features: &[(usize, [f32; 2])],
        entity_path: &str,
    ) {
        if !self.initialized || image.is_empty() {
            return;
        }

        // Log the image first
        self.log_image_raw(image, width, height, entity_path);

        // Then log features with colors
        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            if !features.is_empty() {
                let points: Vec<[f32; 2]> = features.iter().map(|(_, coord)| *coord).collect();
                let colors: Vec<rerun::Color> = features
                    .iter()
                    .map(|(feature_id, _)| {
                        let rgb = get_feature_color(*feature_id);
                        rerun::Color::from_rgb(rgb[0], rgb[1], rgb[2])
                    })
                    .collect();

                let radii: Vec<f32> = vec![3.0; points.len()];
                if let Err(e) = rec.log(
                    format!("{}/features", entity_path).as_str(),
                    &rerun::Points2D::new(points)
                        .with_colors(colors)
                        .with_radii(radii),
                ) {
                    log::warn!("[RerunViewer] Failed to log colored features: {}", e);
                }
            }
        }
    }

    fn log_points(&mut self, points: &[[f32; 3]], entity_path: &str) {
        if !self.initialized || points.is_empty() {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            let _points_3d: Vec<[f32; 3]> = points.to_vec();
            // Exclude points that are further than 300m
            let points_3d: Vec<[f32; 3]> = points
                .iter()
                .cloned()
                .filter(|p| {
                    let distance = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
                    distance <= 300.0
                })
                .collect();

            if let Err(e) = rec.log(entity_path, &rerun::Points3D::new(points_3d)) {
                log::warn!(
                    "[RerunViewer] Failed to log points to {}: {}",
                    entity_path,
                    e
                );
            }
        }
    }

    fn log_points_colored(&mut self, points: &[(usize, [f32; 3])], entity_path: &str) {
        if !self.initialized || points.is_empty() {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            let points_3d: Vec<[f32; 3]> = points.iter().map(|(_, coord)| *coord).collect();
            let colors: Vec<rerun::Color> = points
                .iter()
                .map(|(feature_id, _)| {
                    let rgb = get_feature_color(*feature_id);
                    rerun::Color::from_rgb(rgb[0], rgb[1], rgb[2])
                })
                .collect();

            if let Err(e) = rec.log(
                entity_path,
                &rerun::Points3D::new(points_3d).with_colors(colors),
            ) {
                log::warn!(
                    "[RerunViewer] Failed to log colored points to {}: {}",
                    entity_path,
                    e
                );
            }
        }
    }

    fn set_frame(&mut self, frame_id: i64) {
        self.frame_id = frame_id;
        // Use frame_id as a relative timestamp in nanoseconds (assuming ~30fps = 33ms per frame)
        // This allows rerun to play back at a reasonable speed
        self.timestamp_ns = frame_id * 33_333_333; // ~30 fps

        // Set both sequence and time for proper playback
        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));
        }
    }

    fn log_camera_frustum(
        &mut self,
        focal_length: f32,
        width: u32,
        height: u32,
        entity_path: &str,
        size: f32,
    ) {
        if !self.initialized {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            // Create Pinhole camera model with focal length and resolution
            // The frustum will be visualized at the entity_path location
            // API expects Vec2D for focal_length (fx, fy) and resolution (width, height)
            let focal_vec = (focal_length, focal_length); // Use same focal length for fx and fy
            let resolution_vec = (width as f32, height as f32);
            let pinhole = Pinhole::from_focal_length_and_resolution(focal_vec, resolution_vec)
                .with_image_plane_distance(size);

            if let Err(e) = rec.log(entity_path, &pinhole) {
                log::warn!(
                    "[RerunViewer] Failed to log camera frustum to {}: {}",
                    entity_path,
                    e
                );
            }
        }
    }

    fn log_trajectory(&mut self, trajectory: &[Matrix4x4], entity_path: &str) {
        if !self.initialized || trajectory.is_empty() {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            // Extract translation (position) from each transformation matrix
            // Position is in the last column, rows 0-2
            let positions: Vec<[Float; 3]> = trajectory
                .iter()
                .map(|mat| [mat[(0, 3)], mat[(1, 3)], mat[(2, 3)]])
                .collect();

            // Create a single line strip connecting all trajectory points
            let line_strip = LineStrips3D::new([positions]);

            // Use a distinct color for the trajectory (e.g., yellow/orange)
            let trajectory_color = Color::from_rgb(255, 165, 0); // Orange
            let line_strip = line_strip.with_colors([trajectory_color]);

            if let Err(e) = rec.log(entity_path, &line_strip) {
                log::warn!(
                    "[RerunViewer] Failed to log trajectory to {}: {}",
                    entity_path,
                    e
                );
            }
        }
    }

    /// Visualize PROSAC/MAGSAC++ geometric verification results
    fn log_robustness_verification(
        &mut self,
        inliers: &[(f32, f32)],
        outliers: &[(f32, f32)],
        entity_path: &str,
    ) {
        if !self.initialized {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            // Log inliers as green points
            if !inliers.is_empty() {
                let inlier_positions: Vec<[f32; 3]> =
                    inliers.iter().map(|(x, y)| [*x, *y, 0.0]).collect();

                let inlier_colors = vec![Color::from_rgb(0, 255, 0); inliers.len()]; // Green

                if let Err(e) = rec.log(
                    format!("{}/inliers", entity_path),
                    &rerun::Points3D::new(inlier_positions)
                        .with_colors(inlier_colors)
                        .with_radii([2.0]),
                ) {
                    log::warn!("[RerunViewer] Failed to log inliers: {}", e);
                }
            }

            // Log outliers as red points
            if !outliers.is_empty() {
                let outlier_positions: Vec<[f32; 3]> =
                    outliers.iter().map(|(x, y)| [*x, *y, 0.0]).collect();

                let outlier_colors = vec![Color::from_rgb(255, 0, 0); outliers.len()]; // Red

                if let Err(e) = rec.log(
                    format!("{}/outliers", entity_path),
                    &rerun::Points3D::new(outlier_positions)
                        .with_colors(outlier_colors)
                        .with_radii([2.0]),
                ) {
                    log::warn!("[RerunViewer] Failed to log outliers: {}", e);
                }
            }

            // Log verification statistics
            let stats = format!("Inliers: {}, Outliers: {}", inliers.len(), outliers.len());
            if let Err(e) = rec.log(
                format!("{}/stats", entity_path),
                &rerun::TextDocument::new(stats),
            ) {
                log::warn!("[RerunViewer] Failed to log verification stats: {}", e);
            }
        }
    }

    /// Log raw IMU measurements before processing
    fn log_imu_raw(
        &mut self,
        _timestamp: i64,
        accel_raw: &[[f32; 3]],
        gyro_raw: &[[f32; 3]],
        entity_path: &str,
    ) {
        if !self.initialized || (accel_raw.is_empty() && gyro_raw.is_empty()) {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            // Log raw accelerometer data as line strips (time series)
            if !accel_raw.is_empty() {
                let accel_3d: Vec<[f32; 3]> = accel_raw
                    .iter()
                    .enumerate()
                    .map(|(i, &accel)| [i as f32 * 0.01, accel[0], accel[1]])
                    .collect();

                let accel_line =
                    LineStrips3D::new([accel_3d]).with_colors([Color::from_rgb(255, 100, 100)]); // Light red

                if let Err(e) = rec.log(format!("{}/accel_raw", entity_path), &accel_line) {
                    log::debug!("[RerunViewer] Failed to log raw accel: {}", e);
                }
            }

            // Log raw gyroscope data as line strips (time series)
            if !gyro_raw.is_empty() {
                let gyro_3d: Vec<[f32; 3]> = gyro_raw
                    .iter()
                    .enumerate()
                    .map(|(i, &gyro)| [i as f32 * 0.01, gyro[0], gyro[1]])
                    .collect();

                let gyro_line =
                    LineStrips3D::new([gyro_3d]).with_colors([Color::from_rgb(100, 100, 255)]); // Light blue

                if let Err(e) = rec.log(format!("{}/gyro_raw", entity_path), &gyro_line) {
                    log::debug!("[RerunViewer] Failed to log raw gyro: {}", e);
                }
            }

            // Log statistics about raw measurements
            if !accel_raw.is_empty() {
                let accel_mag: f32 = accel_raw
                    .iter()
                    .map(|a| (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt())
                    .sum::<f32>()
                    / accel_raw.len() as f32;

                let stats = format!(
                    "Raw Accel Measurements: {} | Avg Magnitude: {:.3} m/s²",
                    accel_raw.len(),
                    accel_mag
                );
                if let Err(e) = rec.log(
                    format!("{}/stats_raw", entity_path),
                    &rerun::TextDocument::new(stats),
                ) {
                    log::debug!("[RerunViewer] Failed to log raw stats: {}", e);
                }
            }
        }
    }

    /// Log processed IMU measurements after bias correction and filtering
    fn log_imu_processed(
        &mut self,
        _timestamp: i64,
        accel_processed: &[[f32; 3]],
        gyro_processed: &[[f32; 3]],
        entity_path: &str,
    ) {
        if !self.initialized || (accel_processed.is_empty() && gyro_processed.is_empty()) {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            // Log processed accelerometer data
            if !accel_processed.is_empty() {
                let accel_3d: Vec<[f32; 3]> = accel_processed
                    .iter()
                    .enumerate()
                    .map(|(i, &accel)| [i as f32 * 0.01, accel[0], accel[1]])
                    .collect();

                let accel_line =
                    LineStrips3D::new([accel_3d]).with_colors([Color::from_rgb(255, 0, 0)]); // Bright red

                if let Err(e) = rec.log(format!("{}/accel_processed", entity_path), &accel_line) {
                    log::debug!("[RerunViewer] Failed to log processed accel: {}", e);
                }
            }

            // Log processed gyroscope data
            if !gyro_processed.is_empty() {
                let gyro_3d: Vec<[f32; 3]> = gyro_processed
                    .iter()
                    .enumerate()
                    .map(|(i, &gyro)| [i as f32 * 0.01, gyro[0], gyro[1]])
                    .collect();

                let gyro_line =
                    LineStrips3D::new([gyro_3d]).with_colors([Color::from_rgb(0, 0, 255)]); // Bright blue

                if let Err(e) = rec.log(format!("{}/gyro_processed", entity_path), &gyro_line) {
                    log::debug!("[RerunViewer] Failed to log processed gyro: {}", e);
                }
            }

            // Log processing improvement metrics
            if !accel_processed.is_empty() {
                let processed_mag: f32 = accel_processed
                    .iter()
                    .map(|a| (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt())
                    .sum::<f32>()
                    / accel_processed.len() as f32;

                let stats = format!(
                    "Processed Accel Measurements: {} | Avg Magnitude: {:.3} m/s²",
                    accel_processed.len(),
                    processed_mag
                );
                if let Err(e) = rec.log(
                    format!("{}/stats_processed", entity_path),
                    &rerun::TextDocument::new(stats),
                ) {
                    log::debug!("[RerunViewer] Failed to log processed stats: {}", e);
                }
            }
        }
    }

    /// Log IMU harmonics: gravity, bias, and harmonic components (simplified)
    fn log_imu_harmonics(
        &mut self,
        _timestamp: i64,
        gravity_component: [f32; 3],
        bias_accel: [f32; 3],
        bias_gyro: [f32; 3],
        _harmonic_accel: &[[f32; 3]],
        entity_path: &str,
    ) {
        if !self.initialized {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            // Calculate gravity magnitude
            let gravity_mag = (gravity_component[0] * gravity_component[0]
                + gravity_component[1] * gravity_component[1]
                + gravity_component[2] * gravity_component[2])
                .sqrt();

            // Log metrics as text for time-series analysis
            if let Err(e) = rec.log(
                format!("{}/metrics", entity_path),
                &rerun::TextLog::new(format!("Gravity: {:.2} m/s²", gravity_mag)),
            ) {
                log::debug!("[RerunViewer] Failed to log gravity magnitude: {}", e);
            }

            // Log concise summary
            let summary = format!(
                "Gravity: {:.2} m/s² | Accel Bias: [{:.4}, {:.4}, {:.4}] | Gyro Bias: [{:.4}, {:.4}, {:.4}]",
                gravity_mag, bias_accel[0], bias_accel[1], bias_accel[2],
                bias_gyro[0], bias_gyro[1], bias_gyro[2]
            );
            if let Err(e) = rec.log(
                format!("{}/summary", entity_path),
                &rerun::TextDocument::new(summary),
            ) {
                log::debug!("[RerunViewer] Failed to log harmonics summary: {}", e);
            }
        }
    }

    /// Log IMU signal quality metrics (simplified for efficiency)
    fn log_imu_signal_quality(
        &mut self,
        _timestamp: i64,
        signal_snr: [f32; 3],
        signal_rms: [f32; 3],
        _signal_peak: [f32; 3],
        motor_state: &str,
        fundamental_freq_hz: f32,
        entity_path: &str,
    ) {
        if !self.initialized {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            // Log concise quality summary
            let avg_snr = (signal_snr[0] + signal_snr[1] + signal_snr[2]) / 3.0;
            let avg_rms = (signal_rms[0] + signal_rms[1] + signal_rms[2]) / 3.0;

            let summary = format!(
                "Motor: {} | f₀: {:.1} Hz | SNR: {:.1} dB | RMS: {:.4} m/s²",
                motor_state, fundamental_freq_hz, avg_snr, avg_rms
            );
            if let Err(e) = rec.log(
                format!("{}/summary", entity_path),
                &rerun::TextDocument::new(summary),
            ) {
                log::debug!("[RerunViewer] Failed to log quality summary: {}", e);
            }
        }
    }

    /// Visualize loop closure constraints and verification
    fn log_loop_closure(
        &mut self,
        constraints: &[crate::optimization::loop_closure::LoopClosureConstraint],
        entity_path: &str,
    ) {
        if !self.initialized || constraints.is_empty() {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            // Visualize loop closure edges as lines between keyframes
            let mut lines = Vec::new();

            for constraint in constraints {
                // Create a line from keyframe 1 to keyframe 2
                // In a real implementation, you'd need keyframe poses to get actual positions
                // For now, just visualize the constraint strength
                let start_point = [constraint.keyframe_id_1 as f32 * 0.1, 0.0, 0.0];
                let end_point = [constraint.keyframe_id_2 as f32 * 0.1, 0.0, 0.0];
                lines.push([start_point, end_point]);
            }

            if !lines.is_empty() {
                let line_strips =
                    LineStrips3D::new(lines).with_colors([Color::from_rgb(255, 0, 255)]); // Magenta for loop closures

                if let Err(e) = rec.log(entity_path, &line_strips) {
                    log::warn!("[RerunViewer] Failed to log loop closures: {}", e);
                }
            }

            // Log loop closure statistics
            let stats = format!(
                "Loop Closures: {} | Avg Info: {:.2}",
                constraints.len(),
                constraints
                    .iter()
                    .map(|c| c.information_matrix.trace())
                    .sum::<crate::types::Float>()
                    / constraints.len() as crate::types::Float
            );
            if let Err(e) = rec.log(
                format!("{}/stats", entity_path),
                &rerun::TextDocument::new(stats),
            ) {
                log::warn!("[RerunViewer] Failed to log loop closure stats: {}", e);
            }
        }
    }
}

// Helper function to convert 3x3 rotation matrix to quaternion [x, y, z, w]
fn matrix_to_quaternion(rot: [[Float; 3]; 3]) -> [Float; 4] {
    let trace = rot[0][0] + rot[1][1] + rot[2][2];
    let mut quat = [0.0 as Float; 4];

    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        quat[3] = 0.25 * s;
        quat[0] = (rot[2][1] - rot[1][2]) / s;
        quat[1] = (rot[0][2] - rot[2][0]) / s;
        quat[2] = (rot[1][0] - rot[0][1]) / s;
    } else if rot[0][0] > rot[1][1] && rot[0][0] > rot[2][2] {
        let s = (1.0 + rot[0][0] - rot[1][1] - rot[2][2]).sqrt() * 2.0;
        quat[3] = (rot[2][1] - rot[1][2]) / s;
        quat[0] = 0.25 * s;
        quat[1] = (rot[0][1] + rot[1][0]) / s;
        quat[2] = (rot[0][2] + rot[2][0]) / s;
    } else if rot[1][1] > rot[2][2] {
        let s = (1.0 + rot[1][1] - rot[0][0] - rot[2][2]).sqrt() * 2.0;
        quat[3] = (rot[0][2] - rot[2][0]) / s;
        quat[0] = (rot[0][1] + rot[1][0]) / s;
        quat[1] = 0.25 * s;
        quat[2] = (rot[1][2] + rot[2][1]) / s;
    } else {
        let s = (1.0 + rot[2][2] - rot[0][0] - rot[1][1]).sqrt() * 2.0;
        quat[3] = (rot[1][0] - rot[0][1]) / s;
        quat[0] = (rot[0][2] + rot[2][0]) / s;
        quat[1] = (rot[1][2] + rot[2][1]) / s;
        quat[2] = 0.25 * s;
    }

    quat
}

// Helper function to create a RerunViewer
pub fn create_viewer(
    config: &crate::datasets::config::VisualizationConfig,
) -> Result<Box<dyn Viewer>> {
    let mut viewer = RerunViewer::new_with_config(config);
    viewer.initialize()?;
    Ok(Box::new(viewer))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matrix_to_quaternion_identity_returns_unit_w() {
        let rot = [[1.0_f64, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        let quat = matrix_to_quaternion(rot);
        // Identity rotation should be zero vector with scalar 1
        assert!((quat[0]).abs() < 1e-12);
        assert!((quat[1]).abs() < 1e-12);
        assert!((quat[2]).abs() < 1e-12);
        assert!((quat[3] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn image_to_jpeg_bytes_handles_valid_and_mismatched_sizes() {
        let viewer = RerunViewer::new();

        // 2x2 grayscale image with correct size
        let valid_img = vec![0_u8, 255, 128, 64];
        let jpeg = viewer.image_to_jpeg_bytes(&valid_img, 2, 2, "test/entity");
        let bytes = jpeg.expect("expected JPEG bytes for valid image");
        assert!(bytes.len() > 10, "JPEG output should have bytes");
        assert_eq!(bytes[0], 0xFF);
        assert_eq!(bytes[1], 0xD8);

        // Mismatched size should return None and not panic
        let bad_img = vec![0_u8; 3]; // expects 4 for 2x2
        let jpeg_bad = viewer.image_to_jpeg_bytes(&bad_img, 2, 2, "test/entity");
        assert!(jpeg_bad.is_none());
    }
}
