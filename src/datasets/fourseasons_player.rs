//! 4Seasons dataset player implementation.

use crate::datasets::io::load_grayscale_image;
use crate::datasets::player::DatasetPlayer;
use crate::datasets::{ImageData, ImuData};
use anyhow::{Context, Result};
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

pub struct FourSeasonsPlayer;

impl Default for FourSeasonsPlayer {
    fn default() -> Self {
        Self::new()
    }
}

impl FourSeasonsPlayer {
    pub const fn new() -> Self {
        FourSeasonsPlayer
    }
}

impl DatasetPlayer for FourSeasonsPlayer {
    fn name() -> &'static str {
        "4SeasonsPlayer"
    }

    fn load_image_timestamps(dataset_path: &str) -> Result<Vec<ImageData>> {
        let data_file = Path::new(dataset_path).join("times.txt");
        let file = File::open(&data_file)
            .with_context(|| format!("Cannot open times.txt file: {}", data_file.display()))?;

        let reader = BufReader::new(file);
        let mut image_data = Vec::new();

        for (line_num, line) in reader.lines().enumerate() {
            let line = line?;

            // Skip header and empty lines
            if line_num == 0 || line.trim().is_empty() || line.trim_start().starts_with('#') {
                continue;
            }

            // Format: timestamp filename timestamp (space-separated)
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 2 {
                let timestamp_str = parts[0].trim();
                let filename = parts[0].trim().to_string() + ".png";
                if let Ok(timestamp) = timestamp_str.parse::<i64>() {
                    image_data.push(ImageData {
                        timestamp,
                        filename,
                    });
                }
            }
        }

        log::info!(
            "[4SeasonsPlayer] Loaded {} image timestamps",
            image_data.len()
        );
        Ok(image_data)
    }

    fn load_image(dataset_path: &str, filename: &str, cam_id: u32) -> Result<Vec<u8>> {
        let cam_folder = if cam_id == 0 { "cam0" } else { "cam1" };
        let full_path = Path::new(dataset_path)
            .join("undistorted_images")
            .join(cam_folder)
            .join(filename);
        load_grayscale_image(&full_path)
    }

    fn load_imu_data(dataset_path: &str) -> Result<Vec<ImuData>> {
        // 4Seasons dataset may have IMU data in imu.txt or imu.csv
        // Special handling: supports both comma and space-separated formats
        let mut imu_file = Path::new(dataset_path).join("imu.txt");

        if !imu_file.exists() {
            let imu_csv = Path::new(dataset_path).join("imu.csv");
            if imu_csv.exists() {
                log::debug!(
                    "[4SeasonsPlayer] IMU file imu.txt not found, falling back to {:?}",
                    imu_csv
                );
                imu_file = imu_csv;
            } else {
                log::debug!(
                    "[4SeasonsPlayer] IMU file not found (tried {:?} and {:?}), skipping",
                    imu_file,
                    imu_csv
                );
                return Ok(Vec::new());
            }
        }

        let file = File::open(&imu_file)
            .with_context(|| format!("Cannot open IMU file: {}", imu_file.display()))?;

        let reader = BufReader::new(file);
        let mut imu_data = Vec::new();

        for (line_num, line) in reader.lines().enumerate() {
            let line = line?;

            // Skip header and empty lines
            if line_num == 0 || line.trim().is_empty() || line.trim_start().starts_with('#') {
                continue;
            }

            // Try to parse both space-separated and comma-separated formats
            let parts: Vec<&str> = if line.contains(',') {
                line.split(',').map(|s| s.trim()).collect()
            } else {
                line.split_whitespace().collect()
            };

            // Format: timestamp[ns] w.x w.y w.z a.x a.y a.z
            if parts.len() >= 7 {
                if let Ok(timestamp) = parts[0].trim().parse::<i64>() {
                    if let (Ok(wx), Ok(wy), Ok(wz), Ok(ax), Ok(ay), Ok(az)) = (
                        parts[1].trim().parse::<f64>(),
                        parts[2].trim().parse::<f64>(),
                        parts[3].trim().parse::<f64>(),
                        parts[4].trim().parse::<f64>(),
                        parts[5].trim().parse::<f64>(),
                        parts[6].trim().parse::<f64>(),
                    ) {
                        // Validate ranges
                        let gyro_magnitude = (wx * wx + wy * wy + wz * wz).sqrt();
                        let accel_magnitude = (ax * ax + ay * ay + az * az).sqrt();

                        if gyro_magnitude <= 1000.0 && accel_magnitude <= 200.0 {
                            imu_data.push(ImuData {
                                timestamp,
                                gyro: [wx, wy, wz],
                                accel: [ax, ay, az],
                            });
                        } else {
                            log::debug!(
                                "[4SeasonsPlayer] Skipped malformed IMU line {}: out-of-range values (gyro={:.1}, accel={:.1})",
                                line_num, gyro_magnitude, accel_magnitude
                            );
                        }
                    } else {
                        log::debug!("[4SeasonsPlayer] Skipped malformed IMU line {}: invalid numeric values", line_num);
                    }
                } else {
                    log::debug!(
                        "[4SeasonsPlayer] Skipped malformed IMU line {}: invalid timestamp",
                        line_num
                    );
                }
            } else {
                log::debug!("[4SeasonsPlayer] Skipped malformed IMU line {}: insufficient fields (expected 7+, got {})", line_num, parts.len());
            }
        }

        log::info!(
            "[4SeasonsPlayer] Loaded {} IMU samples from {}",
            imu_data.len(),
            imu_file.display()
        );
        Ok(imu_data)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::datasets::test_utils::{sample_imu_data, timestamps, write_file};
    use tempfile::tempdir;

    #[test]
    fn load_imu_data_missing_file_returns_empty() {
        let dir = tempdir().expect("tempdir");
        let dataset_path = dir.path().to_str().expect("path utf-8");

        let imu_data = FourSeasonsPlayer::load_imu_data(dataset_path).expect("load imu data");
        assert!(imu_data.is_empty());
    }

    #[test]
    fn load_imu_data_skips_malformed_lines() {
        let dir = tempdir().expect("tempdir");
        let imu_path = dir.path().join("imu.txt");

        write_file(
            &imu_path,
            "timestamp w.x w.y w.z a.x a.y a.z\n\
not_a_timestamp 0 0 0 0 0 0\n\
1 0 0 0 0 0\n\
2,0,0,0,0,0,bad\n\
3,0.1,0.2,0.3,1.0,1.1,1.2\n\
4 0.1 0.2 0.3 1.0 1.1 1.2\n",
        );

        let dataset_path = dir.path().to_str().expect("path utf-8");
        let imu_data = FourSeasonsPlayer::load_imu_data(dataset_path).expect("load imu data");
        assert_eq!(timestamps(&imu_data), vec![3, 4]);
    }

    #[test]
    fn get_imu_data_between_frames_boundaries() {
        let imu_data = sample_imu_data();

        let between = FourSeasonsPlayer::get_imu_data_between_frames(2, 4, &imu_data);
        assert_eq!(timestamps(&between), vec![3, 4]);

        let between = FourSeasonsPlayer::get_imu_data_between_frames(0, 1, &imu_data);
        assert_eq!(timestamps(&between), vec![1]);

        let between = FourSeasonsPlayer::get_imu_data_between_frames(3, 3, &imu_data);
        assert!(between.is_empty());
    }
}
