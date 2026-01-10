use super::get_feature_color;
use super::Viewer;
use crate::types::{Array3, Float, Matrix3x3, Matrix4x4, ToArray};
use anyhow::Result;
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
        RerunViewer {
            rec: None,
            initialized: false,
            frame_id: 0,
            timestamp_ns: 0,
            first_timestamp_ns: None,
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
            }
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
        let rec = RecordingStreamBuilder::new("sivo_viewer")
            .spawn()
            .map_err(|e| {
                log::error!("[RerunViewer] Failed to spawn viewer: {}", e);
                e
            })?;

        self.rec = Some(rec);
        self.initialized = true;

        // Give the viewer a moment to fully start up
        std::thread::sleep(std::time::Duration::from_millis(500));

        // Set up coordinate system
        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", 0);
            match rec.log("origin", &rerun::ViewCoordinates::RDF()) {
                Ok(_) => log::debug!("[RerunViewer] Successfully logged coordinate system"),
                Err(e) => {
                    log::warn!("[RerunViewer] Failed to log coordinate system: {}", e);
                    // Don't fail initialization if this fails
                }
            }
            // Log an origin arrow for the coordinate system at the origin
            // The colors used are: X - red, Y - green, Z - blue (conventional)
            let origin = [0.0, 0.0, 0.0];

            // Plot coordinate axes at the origin using rerun::Arrows3D
            let origins = vec![
                origin, // x
                origin, // y
                origin, // z
            ];
            let vectors = vec![
                [1.0, 0.0, 0.0], // X axis
                [0.0, 1.0, 0.0], // Y axis
                [0.0, 0.0, 1.0], // Z axis
            ];
            let colors = vec![
                rerun::Color::from_rgb(255, 0, 0), // X - red
                rerun::Color::from_rgb(0, 255, 0), // Y - green
                rerun::Color::from_rgb(0, 0, 255), // Z - blue
            ];

            match rec.log(
                "origin/axes",
                &rerun::Arrows3D::from_vectors(vectors)
                    .with_origins(origins)
                    .with_colors(colors),
            ) {
                Ok(_) => log::debug!("[RerunViewer] Successfully logged axis arrows"),
                Err(e) => log::warn!("[RerunViewer] Failed to log axis arrows: {}", e),
            }
        }
        log::info!("[RerunViewer] Viewer initialized successfully");
        Ok(())
    }

    fn log_pose(&mut self, T_W_B: Matrix4x4, entity_path: &str) {
        if !self.initialized {
            return;
        }

        if let Some(ref rec) = self.rec {
            rec.set_time_sequence("frame", self.frame_id);
            rec.set_time("time", Timestamp::from_nanos_since_epoch(self.timestamp_ns));

            let translation = Array3::from(T_W_B.fixed_view::<3, 1>(0, 3));
            let rotation = Matrix3x3::from(T_W_B.fixed_view::<3, 3>(0, 0));

            // Convert rotation matrix to quaternion
            let quat = matrix_to_quaternion(Matrix3x3::from(rotation).to_array());
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
                    log::debug!(
                        "[RerunViewer] Successfully logged equalized image to {}",
                        entity_path
                    );
                }
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
                }
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
pub fn create_viewer() -> Result<Box<dyn Viewer>> {
    let mut viewer = RerunViewer::new();
    viewer.initialize()?;
    Ok(Box::new(viewer))
}
