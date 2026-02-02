use crate::datasets::ImageData;
use anyhow::{bail, Context, Result};
use image::ImageReader;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

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
