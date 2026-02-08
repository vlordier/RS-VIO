//! Dataset I/O: image loading, IMU parsing, and ground truth readers.

use crate::datasets::{ImageData, ImuData};
use anyhow::{bail, Context, Result};
use image::ImageReader;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

// ============================================================================
// Constants
// ============================================================================

/// Expected number of fields in IMU CSV/text records
const IMU_EXPECTED_FIELDS: usize = 7;
/// Expected number of fields in image CSV records
const IMAGE_CSV_FIELDS: usize = 2;

/// Maximum physically reasonable gyroscope magnitude (rad/s)
/// Typical IMU: ~500 rad/s, we allow up to 1000 as safety margin
const MAX_GYRO_MAGNITUDE: f64 = 1000.0;

/// Maximum physically reasonable accelerometer magnitude (m/s²)
/// Typical IMU: ~100 m/s², we allow up to 200 as safety margin (>20g)
const MAX_ACCEL_MAGNITUDE: f64 = 200.0;

// ============================================================================
// Parse Statistics
// ============================================================================

/// Statistics from loading IMU or image data
#[derive(Debug, Clone, Default)]
pub struct LoadStats {
    /// Total lines processed (including header)
    pub total_lines: usize,
    /// Valid samples successfully parsed
    pub valid_samples: usize,
    /// Lines skipped due to being header
    pub skipped_header: usize,
    /// Lines skipped due to being empty or comment
    pub skipped_empty_or_comment: usize,
    /// Lines skipped due to invalid timestamp
    pub skipped_invalid_timestamp: usize,
    /// Lines skipped due to invalid numeric values
    pub skipped_invalid_values: usize,
    /// Lines skipped due to insufficient fields
    pub skipped_insufficient_fields: usize,
    /// Lines skipped due to out-of-range values (physical limits)
    pub skipped_out_of_range: usize,
}

impl LoadStats {
    /// Total number of lines skipped
    pub const fn total_skipped(&self) -> usize {
        self.skipped_header
            + self.skipped_empty_or_comment
            + self.skipped_invalid_timestamp
            + self.skipped_invalid_values
            + self.skipped_insufficient_fields
            + self.skipped_out_of_range
    }

    /// Parse success rate as percentage
    pub fn success_rate(&self) -> f64 {
        if self.total_lines == 0 {
            0.0
        } else {
            (self.valid_samples as f64 / self.total_lines as f64) * 100.0
        }
    }
}

// ============================================================================
// IMU Format Specification
// ============================================================================

/// Specifies how IMU data is formatted in the file
#[derive(Debug, Clone, Copy)]
pub enum ImuFormat {
    /// CSV format with comma-separated values (EuRoC)
    CsvComma,
    /// Whitespace-delimited format, space or tab separated (TUM-VI, 4Seasons)
    WhitespaceDelimited,
}

// ============================================================================
// Image Loading
// ============================================================================

/// Load image timestamps from a CSV file.
///
/// Parses a CSV file with the format: `timestamp,filename`
/// - Skips header line (line 0), empty lines, and comment lines (starting with `#`)
/// - Timestamps are expected to be i64 nanoseconds
/// - Filenames are extracted as-is from the CSV
///
/// # Arguments
/// * `data_file` - Path to the CSV file containing image metadata
///
/// # Returns
/// * `Ok(Vec<ImageData>)` - List of parsed image entries (timestamp, filename)
/// * `Err(anyhow::Error)` - File I/O or parsing error with context
///
/// # Example CSV Format
/// ```text
/// timestamp,filename
/// 1640000000000000000,image_00000.png
/// 1640000033333333333,image_00001.png
/// ```
pub(crate) fn load_csv_image_timestamps(data_file: &Path) -> Result<Vec<ImageData>> {
    let (data, _stats) = load_csv_image_timestamps_with_stats(data_file)?;
    Ok(data)
}

/// Load image timestamps from a CSV file, returning both data and statistics.
///
/// Parses a CSV file with the format: `timestamp,filename`
/// - Skips header line (line 0), empty lines, and comment lines (starting with `#`)
/// - Timestamps are expected to be i64 nanoseconds
/// - Filenames are extracted as-is from the CSV
///
/// # Arguments
/// * `data_file` - Path to the CSV file containing image metadata
///
/// # Returns
/// * `Ok((Vec<ImageData>, LoadStats))` - Parsed images and parsing statistics
/// * `Err(anyhow::Error)` - File I/O or parsing error with context
pub(crate) fn load_csv_image_timestamps_with_stats(
    data_file: &Path,
) -> Result<(Vec<ImageData>, LoadStats)> {
    let file = File::open(data_file)
        .with_context(|| format!("Cannot open image timestamps file: {}", data_file.display()))?;

    let reader = BufReader::new(file);
    let mut image_data = Vec::new();
    let mut stats = LoadStats::default();

    for (line_num, line) in reader.lines().enumerate() {
        let line = line?;
        stats.total_lines += 1;

        // Skip header and empty lines
        if line_num == 0 {
            stats.skipped_header += 1;
            continue;
        }
        if line.trim().is_empty() || line.trim_start().starts_with('#') {
            stats.skipped_empty_or_comment += 1;
            continue;
        }

        let parts: Vec<&str> = line.split(',').collect();
        if parts.len() < IMAGE_CSV_FIELDS {
            stats.skipped_insufficient_fields += 1;
            continue;
        }

        let timestamp_str = parts[0].trim();
        let filename = parts[1].trim().to_string();

        if let Ok(timestamp) = timestamp_str.parse::<i64>() {
            image_data.push(ImageData {
                timestamp,
                filename,
            });
            stats.valid_samples += 1;
        } else {
            stats.skipped_invalid_timestamp += 1;
        }
    }

    Ok((image_data, stats))
}

/// Load and decode a grayscale image from disk.
///
/// Reads an image file and converts it to 8-bit grayscale format.
/// Supports any image format that the `image` crate can decode (PNG, JPEG, etc.).
///
/// # Arguments
/// * `full_path` - Absolute path to the image file
///
/// # Returns
/// * `Ok(Vec<u8>)` - Raw grayscale image bytes (single channel, 8 bits per pixel)
/// * `Err(anyhow::Error)` - If file doesn't exist, can't be opened, or can't be decoded
///
/// # Errors
/// - Returns `bail!` if file doesn't exist at the given path
/// - Returns context error if file can't be read
/// - Returns context error if image format can't be decoded
pub(crate) fn load_grayscale_image(full_path: &Path) -> Result<Vec<u8>> {
    if !full_path.exists() {
        bail!("Cannot load image: {}", full_path.display());
    }

    let img = ImageReader::open(full_path)
        .with_context(|| format!("Failed to open image: {}", full_path.display()))?
        .decode()
        .with_context(|| format!("Failed to decode image: {}", full_path.display()))?;

    let gray_img = img.into_luma8();
    Ok(gray_img.into_raw())
}

// ============================================================================
// IMU Loading
// ============================================================================

/// Load IMU data from a file in specified format.
///
/// Parses IMU data with automatic error recovery and validation.
/// - Validates gyroscope and accelerometer magnitudes against physical limits
/// - Provides detailed parse statistics for debugging
/// - Logs skipped lines for dataset quality inspection
///
/// # Arguments
/// * `imu_file` - Path to the IMU data file
/// * `format` - Format specification (CSV or whitespace-delimited)
/// * `player_name` - Name for logging (e.g., "EurocPlayer", "TUMVIPlayer")
///
/// # Returns
/// * `Ok((Vec<ImuData>, LoadStats))` - Parsed IMU samples and parsing statistics
/// * `Err(anyhow::Error)` - File I/O error with context
///
/// # Example
/// ```text
/// Typical output for malformed data:
/// [EurocPlayer] IMU line 42: invalid timestamp (timestamp_value='1640000000bad')
/// [EurocPlayer] IMU line 43: out-of-range gyro (magnitude=1500.5 rad/s, max=1000.0)
/// [EurocPlayer] IMU line 45: insufficient fields (expected 7, got 6)
/// [EurocPlayer] Loaded 5000 IMU samples from /path/to/imu0/data.csv (success rate: 99.95%)
/// ```
pub(crate) fn load_imu_data(
    imu_file: &Path,
    format: ImuFormat,
    player_name: &str,
) -> Result<(Vec<ImuData>, LoadStats)> {
    if !imu_file.exists() {
        log::debug!(
            "[{}] IMU file not found at {:?}, skipping",
            player_name,
            imu_file
        );
        return Ok((Vec::new(), LoadStats::default()));
    }

    let file = File::open(imu_file)
        .with_context(|| format!("Cannot open IMU file: {}", imu_file.display()))?;

    let reader = BufReader::new(file);
    let mut imu_data = Vec::new();
    let mut stats = LoadStats::default();

    for (line_num, line) in reader.lines().enumerate() {
        let line = line?;
        stats.total_lines += 1;

        // Skip header and empty lines
        if line_num == 0 {
            stats.skipped_header += 1;
            continue;
        }
        if line.trim().is_empty() || line.trim_start().starts_with('#') {
            stats.skipped_empty_or_comment += 1;
            continue;
        }

        // Split based on format
        let parts: Vec<&str> = match format {
            ImuFormat::CsvComma => line.split(',').map(|s| s.trim()).collect(),
            ImuFormat::WhitespaceDelimited => line.split_whitespace().collect(),
        };

        // Check field count
        if parts.len() < IMU_EXPECTED_FIELDS {
            stats.skipped_insufficient_fields += 1;
            log::debug!(
                "[{}] IMU line {}: insufficient fields (expected {}, got {})",
                player_name,
                line_num,
                IMU_EXPECTED_FIELDS,
                parts.len()
            );
            continue;
        }

        // Parse timestamp
        let timestamp = match parts[0].parse::<i64>() {
            Ok(ts) => ts,
            Err(_) => {
                stats.skipped_invalid_timestamp += 1;
                log::debug!(
                    "[{}] IMU line {}: invalid timestamp (value='{}')",
                    player_name,
                    line_num,
                    parts[0]
                );
                continue;
            },
        };

        // Parse numeric values (gyro + accel)
        let numeric_parse_result = (
            parts[1].parse::<f64>(),
            parts[2].parse::<f64>(),
            parts[3].parse::<f64>(),
            parts[4].parse::<f64>(),
            parts[5].parse::<f64>(),
            parts[6].parse::<f64>(),
        );

        let (wx, wy, wz, ax, ay, az) = match numeric_parse_result {
            (Ok(a), Ok(b), Ok(c), Ok(d), Ok(e), Ok(f)) => (a, b, c, d, e, f),
            _ => {
                stats.skipped_invalid_values += 1;
                log::debug!(
                    "[{}] IMU line {}: invalid numeric values (values=[{}])",
                    player_name,
                    line_num,
                    parts[1..=6].join(", ")
                );
                continue;
            },
        };

        // Validate ranges
        let gyro_magnitude = (wx * wx + wy * wy + wz * wz).sqrt();
        let accel_magnitude = (ax * ax + ay * ay + az * az).sqrt();

        if gyro_magnitude > MAX_GYRO_MAGNITUDE {
            stats.skipped_out_of_range += 1;
            log::debug!(
                "[{}] IMU line {}: out-of-range gyro (magnitude={:.1} rad/s, max={})",
                player_name,
                line_num,
                gyro_magnitude,
                MAX_GYRO_MAGNITUDE
            );
            continue;
        }

        if accel_magnitude > MAX_ACCEL_MAGNITUDE {
            stats.skipped_out_of_range += 1;
            log::debug!(
                "[{}] IMU line {}: out-of-range accel (magnitude={:.1} m/s², max={})",
                player_name,
                line_num,
                accel_magnitude,
                MAX_ACCEL_MAGNITUDE
            );
            continue;
        }

        // All checks passed, add valid sample
        imu_data.push(ImuData {
            timestamp,
            gyro: [wx, wy, wz],
            accel: [ax, ay, az],
        });
        stats.valid_samples += 1;
    }

    log::info!(
        "[{}] Loaded {} IMU samples from {} (success rate: {:.2}%)",
        player_name,
        imu_data.len(),
        imu_file.display(),
        stats.success_rate()
    );

    Ok((imu_data, stats))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn test_load_stats_total_skipped() {
        let stats = LoadStats {
            total_lines: 10,
            valid_samples: 5,
            skipped_header: 1,
            skipped_empty_or_comment: 1,
            skipped_invalid_timestamp: 1,
            skipped_invalid_values: 1,
            skipped_insufficient_fields: 0,
            skipped_out_of_range: 1,
        };
        assert_eq!(stats.total_skipped(), 5);
    }

    #[test]
    fn test_load_stats_success_rate() {
        let stats = LoadStats {
            total_lines: 10,
            valid_samples: 8,
            ..LoadStats::default()
        };
        assert!((stats.success_rate() - 80.0).abs() < 1e-10);
    }

    #[test]
    fn test_load_stats_success_rate_zero_lines() {
        let stats = LoadStats::default();
        assert_eq!(stats.success_rate(), 0.0);
    }

    #[test]
    fn test_load_csv_image_timestamps() {
        let dir = tempfile::tempdir().unwrap();
        let csv_path = dir.path().join("images.csv");
        let mut f = File::create(&csv_path).unwrap();
        writeln!(f, "timestamp,filename").unwrap();
        writeln!(f, "1000000000,image_0000.png").unwrap();
        writeln!(f, "2000000000,image_0001.png").unwrap();
        writeln!(f, "# comment line").unwrap();
        writeln!(f).unwrap();
        writeln!(f, "bad_ts,image_0002.png").unwrap();
        f.flush().unwrap();

        let (data, stats) = load_csv_image_timestamps_with_stats(&csv_path).unwrap();
        assert_eq!(data.len(), 2);
        assert_eq!(data[0].timestamp, 1_000_000_000);
        assert_eq!(data[0].filename, "image_0000.png");
        assert_eq!(data[1].timestamp, 2_000_000_000);
        assert_eq!(stats.skipped_header, 1);
        assert_eq!(stats.skipped_invalid_timestamp, 1);
        assert!(stats.skipped_empty_or_comment >= 2);
    }

    #[test]
    fn test_load_imu_data_tum_vi_format() {
        let dir = tempfile::tempdir().unwrap();
        let imu_path = dir.path().join("imu.txt");
        let mut f = File::create(&imu_path).unwrap();
        // header
        writeln!(f, "timestamp gx gy gz ax ay az").unwrap();
        // valid line
        writeln!(f, "1000000000 0.01 -0.02 0.03 0.1 0.2 9.81").unwrap();
        // another valid line
        writeln!(f, "2000000000 0.0 0.0 0.0 0.0 0.0 9.81").unwrap();
        // insufficient fields
        writeln!(f, "3000000000 0.1 0.2").unwrap();
        // bad timestamp
        writeln!(f, "bad 0.0 0.0 0.0 0.0 0.0 9.81").unwrap();
        f.flush().unwrap();

        let (data, stats) =
            load_imu_data(&imu_path, ImuFormat::WhitespaceDelimited, "test").unwrap();
        assert_eq!(data.len(), 2);
        assert_eq!(data[0].timestamp, 1_000_000_000);
        assert!((data[0].accel[2] - 9.81).abs() < 1e-10);
        assert_eq!(stats.skipped_insufficient_fields, 1);
        assert_eq!(stats.skipped_invalid_timestamp, 1);
    }

    #[test]
    fn test_load_imu_data_missing_file() {
        let dir = tempfile::tempdir().unwrap();
        let missing = dir.path().join("nonexistent.csv");
        let (data, _stats) =
            load_imu_data(&missing, ImuFormat::CsvComma, "test").unwrap();
        assert!(data.is_empty());
    }

    #[test]
    fn test_load_grayscale_image() {
        let dir = tempfile::tempdir().unwrap();
        let img_path = dir.path().join("test.png");
        // Create a 4x4 gray PNG
        let img = image::GrayImage::from_fn(4, 4, |x, y| {
            image::Luma([(x * 60 + y * 30) as u8])
        });
        img.save(&img_path).unwrap();

        let pixels = load_grayscale_image(&img_path).unwrap();
        assert_eq!(pixels.len(), 16); // 4*4
    }

    #[test]
    fn test_load_grayscale_image_missing() {
        let dir = tempfile::tempdir().unwrap();
        let missing = dir.path().join("no_such.png");
        assert!(load_grayscale_image(&missing).is_err());
    }
}
