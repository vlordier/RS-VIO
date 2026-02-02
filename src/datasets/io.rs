use crate::datasets::ImageData;
use anyhow::{bail, Context, Result};
use image::ImageReader;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

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
    let file = File::open(data_file)
        .with_context(|| format!("Cannot open image timestamps file: {}", data_file.display()))?;

    let reader = BufReader::new(file);
    let mut image_data = Vec::new();

    for (line_num, line) in reader.lines().enumerate() {
        let line = line?;

        // Skip header and empty lines
        if line_num == 0 || line.trim().is_empty() || line.trim_start().starts_with('#') {
            continue;
        }

        let parts: Vec<&str> = line.split(',').collect();
        if parts.len() >= 2 {
            let timestamp_str = parts[0].trim();
            let filename = parts[1].trim().to_string();

            if let Ok(timestamp) = timestamp_str.parse::<i64>() {
                image_data.push(ImageData {
                    timestamp,
                    filename,
                });
            }
        }
    }

    Ok(image_data)
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

    let gray_img = img.to_luma8();
    Ok(gray_img.as_raw().clone())
}
