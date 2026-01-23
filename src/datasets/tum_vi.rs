//! TUM-VI Dataset Loader
//! 
//! Loads TUM Visual-Inertial Dataset (https://vision.in.tum.de/data/datasets/visual-inertial-dataset)
//! Format: EuRoC-compatible (stereo images + IMU + ground truth poses)

use std::path::{Path, PathBuf};
use std::fs;
use std::io::{self, BufRead};
use nalgebra as na;

/// TUM-VI dataset sequence
#[derive(Debug, Clone)]
pub struct TumViSequence {
    /// Sequence name (e.g., "room1")
    pub name: String,
    /// Root directory path
    pub root_dir: PathBuf,
    /// Left camera images
    pub cam0_timestamps: Vec<u64>,
    pub cam0_images: Vec<PathBuf>,
    /// Right camera images
    pub cam1_timestamps: Vec<u64>,
    pub cam1_images: Vec<PathBuf>,
    /// IMU measurements
    pub imu_data: Vec<ImuMeasurement>,
    /// Ground truth poses
    pub ground_truth: Vec<GroundTruthPose>,
}

/// IMU measurement (timestamp, gyro, accel)
#[derive(Debug, Clone, Copy)]
pub struct ImuMeasurement {
    pub timestamp_ns: u64,
    pub gyro_x: f64,
    pub gyro_y: f64,
    pub gyro_z: f64,
    pub accel_x: f64,
    pub accel_y: f64,
    pub accel_z: f64,
}

/// Ground truth pose (timestamp, position, quaternion)
#[derive(Debug, Clone, Copy)]
pub struct GroundTruthPose {
    pub timestamp_ns: u64,
    pub position: na::Vector3<f64>,
    pub orientation: na::UnitQuaternion<f64>,
}

impl TumViSequence {
    /// Load TUM-VI sequence from directory
    /// 
    /// Expected structure:
    /// ```
    /// room1/
    ///   mav0/
    ///     cam0/
    ///       data.csv
    ///       data/
    ///         000000.png
    ///         000001.png
    ///         ...
    ///     cam1/
    ///       data.csv
    ///       data/
    ///     imu0/
    ///       data.csv
    ///     state_groundtruth_estimate0/
    ///       data.csv
    /// ```
    pub fn load(root_dir: impl AsRef<Path>) -> io::Result<Self> {
        let root_dir = root_dir.as_ref();
        let name = root_dir.file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("unknown")
            .to_string();
        
        let mav0 = root_dir.join("mav0");
        
        // Load camera 0 (left)
        let (cam0_timestamps, cam0_images) = Self::load_camera(&mav0.join("cam0"))?;
        
        // Load camera 1 (right)
        let (cam1_timestamps, cam1_images) = Self::load_camera(&mav0.join("cam1"))?;
        
        // Load IMU
        let imu_data = Self::load_imu(&mav0.join("imu0/data.csv"))?;
        
        // Load ground truth
        let ground_truth = Self::load_ground_truth(&mav0.join("state_groundtruth_estimate0/data.csv"))?;
        
        Ok(Self {
            name,
            root_dir: root_dir.to_path_buf(),
            cam0_timestamps,
            cam0_images,
            cam1_timestamps,
            cam1_images,
            imu_data,
            ground_truth,
        })
    }
    
    /// Load camera data (timestamps + image paths)
    fn load_camera(cam_dir: &Path) -> io::Result<(Vec<u64>, Vec<PathBuf>)> {
        let data_csv = cam_dir.join("data.csv");
        let data_dir = cam_dir.join("data");
        
        let file = fs::File::open(data_csv)?;
        let reader = io::BufReader::new(file);
        
        let mut timestamps = Vec::new();
        let mut images = Vec::new();
        
        for line in reader.lines().skip(1) { // Skip CSV header
            let line = line?;
            let parts: Vec<&str> = line.split(',').collect();
            
            if parts.len() >= 2 {
                let timestamp: u64 = parts[0].parse().unwrap_or(0);
                let filename = parts[1].trim();
                let image_path = data_dir.join(filename);
                
                timestamps.push(timestamp);
                images.push(image_path);
            }
        }
        
        Ok((timestamps, images))
    }
    
    /// Load IMU measurements
    fn load_imu(imu_csv: &Path) -> io::Result<Vec<ImuMeasurement>> {
        let file = fs::File::open(imu_csv)?;
        let reader = io::BufReader::new(file);
        
        let mut measurements = Vec::new();
        
        for line in reader.lines().skip(1) { // Skip CSV header
            let line = line?;
            let parts: Vec<&str> = line.split(',').collect();
            
            if parts.len() >= 7 {
                measurements.push(ImuMeasurement {
                    timestamp_ns: parts[0].parse().unwrap_or(0),
                    gyro_x: parts[1].parse().unwrap_or(0.0),
                    gyro_y: parts[2].parse().unwrap_or(0.0),
                    gyro_z: parts[3].parse().unwrap_or(0.0),
                    accel_x: parts[4].parse().unwrap_or(0.0),
                    accel_y: parts[5].parse().unwrap_or(0.0),
                    accel_z: parts[6].parse().unwrap_or(0.0),
                });
            }
        }
        
        Ok(measurements)
    }
    
    /// Load ground truth poses
    fn load_ground_truth(gt_csv: &Path) -> io::Result<Vec<GroundTruthPose>> {
        let file = fs::File::open(gt_csv)?;
        let reader = io::BufReader::new(file);
        
        let mut poses = Vec::new();
        
        for line in reader.lines().skip(1) { // Skip CSV header
            let line = line?;
            let parts: Vec<&str> = line.split(',').collect();
            
            // Format: timestamp, px, py, pz, qw, qx, qy, qz, ...
            if parts.len() >= 8 {
                let timestamp_ns: u64 = parts[0].parse().unwrap_or(0);
                let px: f64 = parts[1].parse().unwrap_or(0.0);
                let py: f64 = parts[2].parse().unwrap_or(0.0);
                let pz: f64 = parts[3].parse().unwrap_or(0.0);
                let qw: f64 = parts[4].parse().unwrap_or(1.0);
                let qx: f64 = parts[5].parse().unwrap_or(0.0);
                let qy: f64 = parts[6].parse().unwrap_or(0.0);
                let qz: f64 = parts[7].parse().unwrap_or(0.0);
                
                poses.push(GroundTruthPose {
                    timestamp_ns,
                    position: na::Vector3::new(px, py, pz),
                    orientation: na::UnitQuaternion::from_quaternion(
                        na::Quaternion::new(qw, qx, qy, qz)
                    ),
                });
            }
        }
        
        Ok(poses)
    }
    
    /// Get number of frames in sequence
    pub fn num_frames(&self) -> usize {
        self.cam0_timestamps.len().min(self.cam1_timestamps.len())
    }
    
    /// Get frame duration (average time between frames)
    pub fn avg_frame_duration_ns(&self) -> u64 {
        if self.cam0_timestamps.len() < 2 {
            return 0;
        }
        
        let total_duration = self.cam0_timestamps.last().unwrap() - self.cam0_timestamps.first().unwrap();
        total_duration / (self.cam0_timestamps.len() as u64 - 1)
    }
    
    /// Calculate frame rate (Hz)
    pub fn frame_rate(&self) -> f64 {
        let duration_ns = self.avg_frame_duration_ns();
        if duration_ns == 0 {
            return 0.0;
        }
        1_000_000_000.0 / duration_ns as f64
    }
}

/// Load all TUM-VI sequences from a directory
pub fn load_all_sequences(dataset_dir: impl AsRef<Path>) -> io::Result<Vec<TumViSequence>> {
    let dataset_dir = dataset_dir.as_ref();
    let mut sequences = Vec::new();
    
    // Expected sequence names
    let sequence_names = vec![
        "dataset-room1_512_16",
        "dataset-room2_512_16",
        "dataset-room3_512_16",
        "dataset-room4_512_16",
        "dataset-room5_512_16",
        "dataset-room6_512_16",
    ];
    
    for name in sequence_names {
        let seq_dir = dataset_dir.join(name);
        if seq_dir.exists() {
            match TumViSequence::load(&seq_dir) {
                Ok(seq) => {
                    println!("Loaded sequence: {} ({} frames @ {:.1} Hz)", 
                             seq.name, seq.num_frames(), seq.frame_rate());
                    sequences.push(seq);
                }
                Err(e) => {
                    eprintln!("Warning: Failed to load {}: {}", name, e);
                }
            }
        }
    }
    
    Ok(sequences)
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_tum_vi_sequence_structure() {
        // This test would run if dataset is downloaded
        // Skip for now since data may not be available in CI
        // For local testing: cargo test --test tum_vi -- --ignored
    }
    
    #[test]
    fn test_imu_measurement_creation() {
        let imu = ImuMeasurement {
            timestamp_ns: 1000,
            gyro_x: 0.1,
            gyro_y: 0.2,
            gyro_z: 0.3,
            accel_x: 9.8,
            accel_y: 0.0,
            accel_z: 0.0,
        };
        
        assert_eq!(imu.timestamp_ns, 1000);
        assert_eq!(imu.accel_x, 9.8);
    }
    
    #[test]
    fn test_ground_truth_pose_creation() {
        let pose = GroundTruthPose {
            timestamp_ns: 1000,
            position: na::Vector3::new(1.0, 2.0, 3.0),
            orientation: na::UnitQuaternion::identity(),
        };
        
        assert_eq!(pose.position.x, 1.0);
        assert_eq!(pose.position.y, 2.0);
        assert_eq!(pose.position.z, 3.0);
    }
}
