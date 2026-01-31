/// Teacher Label Exporter for Training Data Generation (JSON-based)
///
/// This module exports high-quality "teacher" labels from offline VIO processing
/// using JSON format with separate image and metadata files.
use anyhow::{Context, Result};
use ndarray::Array2;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::path::{Path, PathBuf};

/// Frame export data in JSON format
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TeacherFrameJson {
    pub frame_id: usize,
    pub timestamp: f64,

    // ===== Ground Truth Pose (7D: tx, ty, tz, qx, qy, qz, qw) =====
    pub pose_tx: f64,
    pub pose_ty: f64,
    pub pose_tz: f64,
    pub pose_qx: f64,
    pub pose_qy: f64,
    pub pose_qz: f64,
    pub pose_qw: f64,

    // ===== Required Input 8: Previous Pose (7D prior as array) =====
    pub previous_pose: Vec<f64>,

    // ===== Required Input 9: Previous Velocity (3D as array) =====
    pub previous_velocity: Vec<f64>,

    // ===== Required Input 3-4: IMU Data (15D preint + 15D covariance) =====
    pub imu_preintegration: Vec<f64>,
    pub imu_covariance: Vec<f64>,
    pub imu_sample_count: usize,

    // ===== Required Input 4-5: Optical Flow (96-element grid + quality scalar) =====
    pub flow: Vec<f64>,    // 96-element flattened grid [dx, dy per cell]
    pub flow_quality: f64, // Scalar inlier ratio [0-1]

    // ===== Required Input 7: Feature Match Quality (32-element scores) =====
    pub match_quality: Vec<f64>, // 32-element array or padded

    // ===== Required Input 10: Time Since Keyframe (scalar) =====
    pub time_since_keyframe: f64, // Frame count since last keyframe

    // ===== Quality metrics =====
    pub reprojection_errors: Vec<f32>,
    pub mean_reprojection_error: f32,
    pub ba_iterations: usize,
    pub ba_converged: bool,

    // Depth quality
    pub depth_coverage: f32,

    // File references (for images and depth)
    pub image_files: ImageFileRefs,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ImageFileRefs {
    pub left_full: String,
    pub right_full: String,
    pub left_downscaled: String,
    pub right_downscaled: String,
    pub depth_map: String,
    pub depth_confidence: String,
}

/// Teacher Data Exporter with JSON + PNG images
pub struct TeacherLabelExporterJson {
    output_dir: PathBuf,
    sequence_name: String,
    frame_count: usize,
    frames_metadata: Vec<TeacherFrameJson>,
}

impl TeacherLabelExporterJson {
    /// Create new exporter for a sequence
    pub fn new(output_path: &Path, sequence_name: &str) -> Result<Self> {
        let output_dir = output_path.to_path_buf();

        // Create subdirectories
        std::fs::create_dir_all(&output_dir)?;
        std::fs::create_dir_all(output_dir.join("images"))?;
        std::fs::create_dir_all(output_dir.join("depth"))?;

        log::info!("Created teacher export directories in: {:?}", output_dir);

        Ok(Self {
            output_dir,
            sequence_name: sequence_name.to_string(),
            frame_count: 0,
            frames_metadata: Vec::new(),
        })
    }

    /// Export a single frame
    pub fn export_frame(&mut self, frame: super::TeacherFrame) -> Result<()> {
        let idx = self.frame_count;

        log::debug!(
            "Exporting teacher frame {}: id={}, timestamp={:.3}s",
            idx,
            frame.frame_id,
            frame.timestamp
        );

        // Save images as PNG
        let img_dir = self.output_dir.join("images");
        let left_full_path = format!("frame_{:06}_left_full.png", idx);
        let right_full_path = format!("frame_{:06}_right_full.png", idx);
        let left_down_path = format!("frame_{:06}_left_256.png", idx);
        let right_down_path = format!("frame_{:06}_right_256.png", idx);

        image::save_buffer(
            img_dir.join(&left_full_path),
            &frame.left_image,
            frame.left_image.width(),
            frame.left_image.height(),
            image::ColorType::L8,
        )?;

        image::save_buffer(
            img_dir.join(&right_full_path),
            &frame.right_image,
            frame.right_image.width(),
            frame.right_image.height(),
            image::ColorType::L8,
        )?;

        image::save_buffer(
            img_dir.join(&left_down_path),
            &frame.left_downscaled,
            frame.left_downscaled.width(),
            frame.left_downscaled.height(),
            image::ColorType::L8,
        )?;

        image::save_buffer(
            img_dir.join(&right_down_path),
            &frame.right_downscaled,
            frame.right_downscaled.width(),
            frame.right_downscaled.height(),
            image::ColorType::L8,
        )?;

        // Save depth maps as binary (little-endian f32)
        let depth_dir = self.output_dir.join("depth");
        let depth_path = format!("frame_{:06}_depth.bin", idx);
        let confidence_path = format!("frame_{:06}_confidence.bin", idx);

        save_array2_f32(&depth_dir.join(&depth_path), &frame.depth_map)?;
        save_array2_f32(&depth_dir.join(&confidence_path), &frame.depth_confidence)?;

        // Compute depth coverage
        let depth_coverage = frame.depth_map.iter().filter(|&&d| d > 0.0).count() as f32
            / frame.depth_map.len() as f32;

        // Convert optical flow grid to 96-element flat vector [dx, dy per cell]
        let flow_vec: Vec<f64> = frame
            .flow_grid
            .iter()
            .flat_map(|point| {
                let dx = (point.curr.0 - point.prev.0) as f64;
                let dy = (point.curr.1 - point.prev.1) as f64;
                vec![dx, dy]
            })
            .collect();
        // Pad to 96 elements if needed (8x6 grid = 48 points = 96 values)
        let flow_vec = if flow_vec.len() < 96 {
            let mut padded = flow_vec;
            padded.resize(96, 0.0);
            padded
        } else {
            flow_vec.into_iter().take(96).collect()
        };

        // Extract pose data
        let pose = frame.pose_world_cam;
        let pose_trans = pose.translation;
        let pose_quat = pose.rotation.quaternion();

        // Extract previous pose data as array
        let prev_pose = frame.previous_pose;
        let prev_pose_trans = prev_pose.translation;
        let prev_pose_quat = prev_pose.rotation.quaternion();
        let previous_pose_array = vec![
            prev_pose_trans.x,
            prev_pose_trans.y,
            prev_pose_trans.z,
            prev_pose_quat.i,
            prev_pose_quat.j,
            prev_pose_quat.k,
            prev_pose_quat.w,
        ];

        // Previous velocity as array
        let previous_velocity_array = vec![
            frame.previous_velocity.x,
            frame.previous_velocity.y,
            frame.previous_velocity.z,
        ];

        // Pad match quality to 32 elements
        let mut match_qual = frame.match_quality.clone();
        match_qual.resize(32, 0.0);

        // Create metadata entry with all required fields
        let frame_json = TeacherFrameJson {
            frame_id: frame.frame_id,
            timestamp: frame.timestamp,

            // Current pose (7D)
            pose_tx: pose_trans.x,
            pose_ty: pose_trans.y,
            pose_tz: pose_trans.z,
            pose_qx: pose_quat.i,
            pose_qy: pose_quat.j,
            pose_qz: pose_quat.k,
            pose_qw: pose_quat.w,

            // Previous pose (7D as array)
            previous_pose: previous_pose_array,

            // Previous velocity (3D as array)
            previous_velocity: previous_velocity_array,

            // IMU data (15D preint + 15D covariance)
            imu_preintegration: frame.imu_preintegration.as_slice().to_vec(),
            imu_covariance: frame.imu_covariance.as_slice().to_vec(),
            imu_sample_count: frame.imu_sample_count,

            // Optical flow (96-element + quality scalar)
            flow: flow_vec,
            flow_quality: frame.flow_quality,

            // Feature match quality (32-element padded)
            match_quality: match_qual,

            // Time since keyframe
            time_since_keyframe: frame.time_since_keyframe,

            // Quality metrics
            reprojection_errors: frame.reprojection_errors,
            mean_reprojection_error: frame.mean_reprojection_error,
            ba_iterations: frame.ba_iterations,
            ba_converged: frame.ba_converged,

            depth_coverage,

            image_files: ImageFileRefs {
                left_full: left_full_path,
                right_full: right_full_path,
                left_downscaled: left_down_path,
                right_downscaled: right_down_path,
                depth_map: depth_path,
                depth_confidence: confidence_path,
            },
        };

        self.frames_metadata.push(frame_json);
        self.frame_count += 1;

        if self.frame_count % 50 == 0 {
            log::info!(
                "Exported {} teacher frames for sequence '{}'",
                self.frame_count,
                self.sequence_name
            );
        }

        Ok(())
    }

    /// Finalize export and write summary
    pub fn finalize(&mut self) -> Result<()> {
        log::info!(
            "Finalizing teacher data export: {} frames for sequence '{}'",
            self.frame_count,
            self.sequence_name
        );

        // Write frames metadata
        let metadata_path = self
            .output_dir
            .join(format!("{}_metadata.json", self.sequence_name));
        let metadata_file = File::create(&metadata_path)
            .with_context(|| format!("Failed to create metadata file: {:?}", metadata_path))?;

        serde_json::to_writer_pretty(metadata_file, &self.frames_metadata)
            .context("Failed to write metadata JSON")?;

        // Write summary
        let summary = ExportSummary {
            sequence_name: self.sequence_name.clone(),
            total_frames: self.frame_count,
            output_dir: self.output_dir.to_string_lossy().to_string(),
            format: "JSON+PNG".to_string(),
            timestamp_generated: chrono::Utc::now().to_rfc3339(),
            average_depth_coverage: self
                .frames_metadata
                .iter()
                .map(|f| f.depth_coverage as f64)
                .sum::<f64>()
                / self.frame_count.max(1) as f64,
            average_reprojection_error: self
                .frames_metadata
                .iter()
                .map(|f| f.mean_reprojection_error as f64)
                .sum::<f64>()
                / self.frame_count.max(1) as f64,
        };

        let summary_path = self
            .output_dir
            .join(format!("{}_summary.json", self.sequence_name));
        let summary_file = File::create(&summary_path)
            .with_context(|| format!("Failed to create summary file: {:?}", summary_path))?;

        serde_json::to_writer_pretty(summary_file, &summary)
            .context("Failed to write summary JSON")?;

        log::info!(
            "✅ Teacher data export complete: {} frames",
            self.frame_count
        );

        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ExportSummary {
    pub sequence_name: String,
    pub total_frames: usize,
    pub output_dir: String,
    pub format: String,
    pub timestamp_generated: String,
    pub average_depth_coverage: f64,
    pub average_reprojection_error: f64,
}

/// Save Array2<f32> as binary file (little-endian)
fn save_array2_f32(path: &Path, array: &Array2<f32>) -> Result<()> {
    use std::io::Write;

    let mut file = File::create(path)?;

    // Write dimensions
    let (rows, cols) = array.dim();
    file.write_all(&(rows as u32).to_le_bytes())?;
    file.write_all(&(cols as u32).to_le_bytes())?;

    // Write data
    for value in array.iter() {
        file.write_all(&value.to_le_bytes())?;
    }

    Ok(())
}
