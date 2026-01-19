//! Integration test utilities for capturing and storing test screenshots
//!
//! This module provides helpers to capture screenshots during integration tests
//! and store them in a structured artifacts directory for debugging and validation.
//!
//! # Usage
//!
//! ```rust,ignore
//! let artifacts = ScreenshotArtifacts::new_test("my_test")?;
//! // ... run test code ...
//! artifacts.save_screenshot("key_frame", &pixel_data)?;
//! artifacts.finalize()?;  // Saves to target/test-artifacts/my_test/
//! ```

use std::fs;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};
use std::error::Error;

/// Manager for test screenshot artifacts
#[derive(Debug)]
pub struct ScreenshotArtifacts {
    /// Test name (used as subdirectory)
    test_name: String,
    /// Base artifacts directory
    artifacts_dir: PathBuf,
    /// Test-specific directory
    test_dir: PathBuf,
    /// Collected screenshots and their metadata
    screenshots: Vec<(String, Vec<u8>)>,
}

impl ScreenshotArtifacts {
    /// Create a new artifacts manager for a test
    ///
    /// # Arguments
    /// * `test_name` - Name of the test (becomes subdirectory under artifacts/)
    ///
    /// # Returns
    /// New ScreenshotArtifacts instance with test directory prepared
    pub fn new_test(test_name: &str) -> Result<Self, Box<dyn Error>> {
        let artifacts_dir = PathBuf::from("target/test-artifacts");
        let test_dir = artifacts_dir.join(test_name);
        
        // Create directory structure if it doesn't exist
        fs::create_dir_all(&test_dir)?;
        
        Ok(ScreenshotArtifacts {
            test_name: test_name.to_string(),
            artifacts_dir,
            test_dir,
            screenshots: Vec::new(),
        })
    }

    /// Queue a screenshot to be saved
    ///
    /// # Arguments
    /// * `name` - Short name for this screenshot (e.g., "initial_frame", "final_state")
    /// * `png_data` - PNG-encoded image data
    pub fn queue_screenshot(&mut self, name: &str, png_data: Vec<u8>) {
        self.screenshots.push((name.to_string(), png_data));
    }

    /// Save a single screenshot immediately
    ///
    /// # Arguments
    /// * `name` - Name for the screenshot (PNG extension added automatically)
    /// * `png_data` - PNG-encoded image data
    pub fn save_screenshot(&mut self, name: &str, png_data: Vec<u8>) -> Result<PathBuf, Box<dyn Error>> {
        let filename = format!("{}.png", name);
        let path = self.test_dir.join(&filename);
        fs::write(&path, &png_data)?;
        Ok(path)
    }

    /// Get path to the test artifacts directory
    pub fn test_dir(&self) -> &Path {
        &self.test_dir
    }

    /// Get the full artifacts base directory
    pub fn artifacts_dir(&self) -> &Path {
        &self.artifacts_dir
    }

    /// Write all queued screenshots to disk
    pub fn finalize(&mut self) -> Result<Vec<PathBuf>, Box<dyn Error>> {
        let mut saved_paths = Vec::new();
        
        for (name, data) in self.screenshots.iter() {
            let filename = format!("{}.png", name);
            let path = self.test_dir.join(&filename);
            fs::write(&path, data)?;
            saved_paths.push(path);
        }
        
        // Write manifest file listing all screenshots
        let manifest_path = self.test_dir.join("manifest.txt");
        let manifest = format!(
            "Test Artifacts: {}\nGenerated: {}\n\nScreenshots:\n{}\n",
            self.test_name,
            timestamp_string(),
            saved_paths.iter()
                .map(|p| format!("  - {}", p.file_name().unwrap_or_default().to_string_lossy()))
                .collect::<Vec<_>>()
                .join("\n")
        );
        fs::write(&manifest_path, manifest)?;
        saved_paths.push(manifest_path);
        
        Ok(saved_paths)
    }

    /// Clean up artifacts for this test (removes test directory)
    pub fn cleanup(&self) -> Result<(), Box<dyn Error>> {
        if self.test_dir.exists() {
            fs::remove_dir_all(&self.test_dir)?;
        }
        Ok(())
    }

    /// Get current artifact paths for viewing/debugging
    pub fn artifact_paths(&self) -> Result<Vec<PathBuf>, Box<dyn Error>> {
        let mut paths = Vec::new();
        
        if self.test_dir.exists() {
            for entry in fs::read_dir(&self.test_dir)? {
                let entry = entry?;
                paths.push(entry.path());
            }
        }
        
        Ok(paths)
    }
}

/// Helper to generate timestamp strings for artifact naming
fn timestamp_string() -> String {
    match SystemTime::now().duration_since(UNIX_EPOCH) {
        Ok(duration) => {
            let secs = duration.as_secs();
            let millis = duration.subsec_millis();
            format!("{}.{:03}", secs, millis)
        },
        Err(_) => "unknown".to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_artifacts_directory_creation() -> Result<(), Box<dyn Error>> {
        let artifacts = ScreenshotArtifacts::new_test("test_artifacts_creation")?;
        assert!(artifacts.test_dir().exists());
        assert!(artifacts.test_dir().is_dir());
        artifacts.cleanup()?;
        Ok(())
    }

    #[test]
    fn test_save_screenshot() -> Result<(), Box<dyn Error>> {
        let mut artifacts = ScreenshotArtifacts::new_test("test_save_screenshot")?;
        
        // Create dummy PNG data (minimal valid PNG)
        let png_data = vec![
            0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, // PNG signature
        ];
        
        let path = artifacts.save_screenshot("test_image", png_data.clone())?;
        assert!(path.exists());
        
        let saved_data = fs::read(&path)?;
        assert_eq!(saved_data, png_data);
        
        artifacts.cleanup()?;
        Ok(())
    }

    #[test]
    fn test_queue_and_finalize() -> Result<(), Box<dyn Error>> {
        let mut artifacts = ScreenshotArtifacts::new_test("test_queue_finalize")?;
        
        let png1 = b"PNG_DATA_1".to_vec();
        let png2 = b"PNG_DATA_2".to_vec();
        
        artifacts.queue_screenshot("frame_1", png1);
        artifacts.queue_screenshot("frame_2", png2);
        
        let saved = artifacts.finalize()?;
        assert!(saved.len() >= 3); // 2 images + manifest
        
        // Verify files exist
        assert!(artifacts.test_dir().join("frame_1.png").exists());
        assert!(artifacts.test_dir().join("frame_2.png").exists());
        assert!(artifacts.test_dir().join("manifest.txt").exists());
        
        artifacts.cleanup()?;
        Ok(())
    }

    #[test]
    fn test_artifact_paths() -> Result<(), Box<dyn Error>> {
        let mut artifacts = ScreenshotArtifacts::new_test("test_artifact_paths")?;
        
        artifacts.save_screenshot("test1", b"data1".to_vec())?;
        artifacts.save_screenshot("test2", b"data2".to_vec())?;
        
        let paths = artifacts.artifact_paths()?;
        assert_eq!(paths.len(), 2);
        
        artifacts.cleanup()?;
        Ok(())
    }
}
