/// Teacher Label Exporter for Training Data Generation
///
/// This module exports high-quality "teacher" labels from offline VIO processing
/// for training a lightweight "student" network. The teacher runs with generous
/// settings (large window, many features, high BA iterations) to produce accurate
/// ground truth poses and depth maps.
///
/// Exported data includes:
/// - Low-res stereo images (256x256) for student input
/// - IMU preintegration
/// - Sparse optical flow grid
/// - High-quality pose from BA (ground truth)
/// - Dense/semi-dense depth map with confidence
/// - Residuals and quality metrics
use anyhow::{Context, Result};
use hdf5::File;
use image::GrayImage;
use nalgebra::{Isometry3, Matrix6, SVector, Vector3};
use ndarray::{Array2, Array3};
use std::path::Path;

/// Simple optical flow point for export
#[derive(Clone, Debug)]
pub struct OpticalFlowPoint {
    /// Previous frame location
    pub prev: (f32, f32),
    /// Current frame location
    pub curr: (f32, f32),
}

/// Teacher frame data for export
pub struct TeacherFrame {
    /// Frame ID
    pub frame_id: usize,

    /// Timestamp (seconds)
    pub timestamp: f64,

    // ===== Input Images =====
    /// Left image (full resolution, e.g., 512x512)
    pub left_image: GrayImage,

    /// Right image (full resolution)
    pub right_image: GrayImage,

    /// Left image downscaled for student (256x256)
    pub left_downscaled: GrayImage,

    /// Right image downscaled for student (256x256)
    pub right_downscaled: GrayImage,

    // ===== IMU Data =====
    /// IMU preintegration [Δp, Δv, Δq] + biases (15D)
    pub imu_preintegration: SVector<f64, 15>,

    /// IMU covariance diagonal (15D) - uncertainty of preintegration
    pub imu_covariance: SVector<f64, 15>,

    /// IMU samples count in this interval
    pub imu_sample_count: usize,

    // ===== Flow Data =====
    /// Sparse optical flow grid (8x6 = 48 points) - FLATTENED TO 96-ELEMENT VECTOR
    pub flow_grid: Vec<OpticalFlowPoint>,

    /// Flow quality - inlier ratio (0-1)
    pub flow_quality: f64,

    // ===== Teacher Labels (Ground Truth) =====
    /// Refined pose from BA (world to camera)
    pub pose_world_cam: Isometry3<f64>,

    /// Pose covariance (6x6)
    pub pose_covariance: Matrix6<f64>,

    /// Current velocity (m/s)
    pub velocity: Vector3<f64>,

    /// Previous frame pose (for pose prior input)
    pub previous_pose: Isometry3<f64>,

    /// Previous frame velocity (for velocity prior input)
    pub previous_velocity: Vector3<f64>,

    /// Feature match quality scores (one per match, up to 32)
    pub match_quality: Vec<f64>,

    /// Time since last keyframe (frame count)
    pub time_since_keyframe: f64,

    /// Dense/semi-dense depth map (full resolution)
    /// -1.0 for invalid/no depth
    pub depth_map: Array2<f32>,

    /// Per-pixel depth confidence (0-1)
    /// Higher = more confident
    pub depth_confidence: Array2<f32>,

    // ===== Quality Metrics =====
    /// Reprojection errors for all landmarks
    pub reprojection_errors: Vec<f32>,

    /// Mean reprojection error
    pub mean_reprojection_error: f32,

    /// BA iterations used
    pub ba_iterations: usize,

    /// BA converged successfully
    pub ba_converged: bool,
}

/// Teacher label exporter writes training data to HDF5 files
pub struct TeacherLabelExporter {
    /// HDF5 file handle
    file: File,

    /// Frame count
    frame_count: usize,

    /// Sequence name
    sequence_name: String,
}

impl TeacherLabelExporter {
    /// Create new exporter for a sequence
    pub fn new(output_path: &Path, sequence_name: &str) -> Result<Self> {
        let filename = output_path.join(format!("{}.h5", sequence_name));

        log::info!("Creating teacher data export file: {:?}", filename);

        let file = File::create(&filename)
            .with_context(|| format!("Failed to create HDF5 file: {:?}", filename))?;

        // Create groups for different data types
        file.create_group("images")?;
        file.create_group("imu")?;
        file.create_group("flow")?;
        file.create_group("labels")?;
        file.create_group("quality")?;

        Ok(Self {
            file,
            frame_count: 0,
            sequence_name: sequence_name.to_string(),
        })
    }

    /// Export a single frame
    pub fn export_frame(&mut self, frame: TeacherFrame) -> Result<()> {
        let idx = self.frame_count;

        log::debug!(
            "Exporting teacher frame {}: id={}, timestamp={:.3}s",
            idx,
            frame.frame_id,
            frame.timestamp
        );

        // Convert images to arrays
        let left_full = image_to_array(&frame.left_image)?;
        let right_full = image_to_array(&frame.right_image)?;
        let left_low = image_to_array(&frame.left_downscaled)?;
        let right_low = image_to_array(&frame.right_downscaled)?;

        // Write images
        self.write_image(&format!("images/left_full_{}", idx), &left_full)?;
        self.write_image(&format!("images/right_full_{}", idx), &right_full)?;
        self.write_image(&format!("images/left_low_{}", idx), &left_low)?;
        self.write_image(&format!("images/right_low_{}", idx), &right_low)?;

        // Write IMU
        self.write_vector(
            &format!("imu/preintegration_{}", idx),
            frame.imu_preintegration.as_slice(),
        )?;
        self.write_vector(
            &format!("imu/covariance_{}", idx),
            frame.imu_covariance.as_slice(),
        )?;
        self.write_scalar(
            &format!("imu/sample_count_{}", idx),
            frame.imu_sample_count as f64,
        )?;

        // Write flow
        self.write_flow_grid(&format!("flow/grid_{}", idx), &frame.flow_grid)?;
        self.write_scalar(&format!("flow/quality_{}", idx), frame.flow_quality)?;

        // Write tracking data
        self.write_vector(
            &format!("tracking/match_quality_{}", idx),
            &frame.match_quality,
        )?;
        self.write_scalar(
            &format!("tracking/time_since_keyframe_{}", idx),
            frame.time_since_keyframe,
        )?;

        // Write previous frame data
        let prev_pose_se3 = frame.previous_pose;
        self.write_se3(&format!("history/previous_pose_{}", idx), &prev_pose_se3)?;
        self.write_vector(
            &format!("history/previous_velocity_{}", idx),
            frame.previous_velocity.as_slice(),
        )?;

        // Write labels (ground truth)
        self.write_se3(&format!("labels/pose_{}", idx), &frame.pose_world_cam)?;
        self.write_matrix6(
            &format!("labels/covariance_{}", idx),
            &frame.pose_covariance,
        )?;
        self.write_vector(
            &format!("labels/velocity_{}", idx),
            frame.velocity.as_slice(),
        )?;
        self.write_array2(&format!("labels/depth_{}", idx), &frame.depth_map)?;
        self.write_array2(
            &format!("labels/depth_confidence_{}", idx),
            &frame.depth_confidence,
        )?;

        // Write quality metrics
        self.write_vector_f32(
            &format!("quality/reproj_errors_{}", idx),
            &frame.reprojection_errors,
        )?;
        self.write_scalar(
            &format!("quality/mean_reproj_error_{}", idx),
            frame.mean_reprojection_error as f64,
        )?;
        self.write_scalar(
            &format!("quality/ba_iterations_{}", idx),
            frame.ba_iterations as f64,
        )?;
        self.write_scalar(
            &format!("quality/ba_converged_{}", idx),
            if frame.ba_converged { 1.0 } else { 0.0 },
        )?;

        // Write metadata
        self.write_scalar(&format!("metadata/frame_id_{}", idx), frame.frame_id as f64)?;
        self.write_scalar(&format!("metadata/timestamp_{}", idx), frame.timestamp)?;

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

        // Write summary metadata
        let metadata = self.file.group("metadata")?;
        metadata
            .new_dataset::<u64>()
            .create("total_frames")?
            .write_scalar(&(self.frame_count as u64))?;

        self.file.flush()?;

        log::info!(
            "✅ Teacher data export complete: {} frames",
            self.frame_count
        );

        Ok(())
    }

    // Helper methods for writing different data types

    fn write_image(&self, path: &str, data: &Array3<u8>) -> Result<()> {
        let dataset = self
            .file
            .new_dataset::<u8>()
            .shape(data.shape())
            .create(path)?;
        dataset.write_raw(data.as_slice().unwrap())?;
        Ok(())
    }

    fn write_array2(&self, path: &str, data: &Array2<f32>) -> Result<()> {
        let dataset = self
            .file
            .new_dataset::<f32>()
            .shape(data.shape())
            .create(path)?;
        dataset.write_raw(data.as_slice().unwrap())?;
        Ok(())
    }

    fn write_vector(&self, path: &str, data: &[f64]) -> Result<()> {
        let dataset = self
            .file
            .new_dataset::<f64>()
            .shape([data.len()])
            .create(path)?;
        dataset.write_raw(data)?;
        Ok(())
    }

    fn write_vector_f32(&self, path: &str, data: &[f32]) -> Result<()> {
        let dataset = self
            .file
            .new_dataset::<f32>()
            .shape([data.len()])
            .create(path)?;
        dataset.write_raw(data)?;
        Ok(())
    }

    fn write_scalar(&self, path: &str, value: f64) -> Result<()> {
        let dataset = self.file.new_dataset::<f64>().create(path)?;
        dataset.write_scalar(&value)?;
        Ok(())
    }

    fn write_se3(&self, path: &str, pose: &Isometry3<f64>) -> Result<()> {
        // Store as 4x4 matrix
        let mat = pose.to_homogeneous();
        let data: Vec<f64> = mat.as_slice().to_vec();

        let dataset = self.file.new_dataset::<f64>().shape([4, 4]).create(path)?;
        dataset.write_raw(&data)?;
        Ok(())
    }

    fn write_matrix6(&self, path: &str, mat: &Matrix6<f64>) -> Result<()> {
        let data: Vec<f64> = mat.as_slice().to_vec();

        let dataset = self.file.new_dataset::<f64>().shape([6, 6]).create(path)?;
        dataset.write_raw(&data)?;
        Ok(())
    }

    fn write_flow_grid(&self, path: &str, flow: &[OpticalFlowPoint]) -> Result<()> {
        // Flatten flow points to [N, 4] array: [x1, y1, x2, y2]
        let mut data = Vec::with_capacity(flow.len() * 4);
        for point in flow {
            data.push(point.prev.0);
            data.push(point.prev.1);
            data.push(point.curr.0);
            data.push(point.curr.1);
        }

        let dataset = self
            .file
            .new_dataset::<f32>()
            .shape([flow.len(), 4])
            .create(path)?;
        dataset.write_raw(&data)?;
        Ok(())
    }
}

impl Drop for TeacherLabelExporter {
    fn drop(&mut self) {
        if let Err(e) = self.file.flush() {
            log::error!("Failed to flush HDF5 file on drop: {}", e);
        }
    }
}

/// Convert Image to ndarray
fn image_to_array(img: &GrayImage) -> Result<Array3<u8>> {
    let height = img.height() as usize;
    let width = img.width() as usize;

    let mut array = Array3::<u8>::zeros((height, width, 1));

    for y in 0..height {
        for x in 0..width {
            let pixel = img.get_pixel(x as u32, y as u32);
            array[[y, x, 0]] = pixel[0]; // Grayscale
        }
    }

    Ok(array)
}

/// Downscale image for student network input
pub fn downscale_image(img: &GrayImage, target_size: u32) -> GrayImage {
    use image::imageops::FilterType;

    image::imageops::resize(img, target_size, target_size, FilterType::Lanczos3)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    #[test]
    fn test_exporter_creation() {
        let dir = tempdir().unwrap();
        let exporter = TeacherLabelExporter::new(dir.path(), "test_sequence");
        assert!(exporter.is_ok());
    }
}
