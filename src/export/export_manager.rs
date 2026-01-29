/// Export state management for teacher data collection
///
/// Manages the lifecycle of teacher data export during VIO processing.
/// Only compiled when the `export-teacher` feature is enabled.

use anyhow::Result;
use std::path::PathBuf;

#[cfg(feature = "export-teacher")]
use crate::export::TeacherLabelExporterJson;

#[cfg(feature = "export-teacher")]
use crate::export::teacher_exporter::TeacherFrame;

/// Configuration for teacher data export
#[derive(Clone, Debug)]
pub struct ExportConfig {
    /// Output directory for JSON metadata and images
    pub output_dir: PathBuf,
    /// Sequence name (e.g., "room1", "room2")
    pub sequence_name: String,
    /// Enable/disable export
    pub enabled: bool,
}

/// Manages teacher data export state
pub struct ExportManager {
    #[cfg(feature = "export-teacher")]
    exporter: Option<TeacherLabelExporterJson>,
    config: ExportConfig,
}

impl ExportManager {
    /// Create new export manager
    pub fn new(config: ExportConfig) -> Result<Self> {
        #[cfg(feature = "export-teacher")]
        {
            if !config.enabled {
                return Ok(Self {
                    exporter: None,
                    config,
                });
            }
            
            // Create output directory if it doesn't exist
            std::fs::create_dir_all(&config.output_dir)?;
            
            let exporter = TeacherLabelExporterJson::new(
                &config.output_dir,
                &config.sequence_name,
            )?;
            
            log::info!("Teacher export enabled (JSON format) for sequence: {}", config.sequence_name);
            
            Ok(Self {
                exporter: Some(exporter),
                config,
            })
        }
        
        #[cfg(not(feature = "export-teacher"))]
        {
            Ok(Self { config })
        }
    }
    
    /// Check if export is enabled
    pub fn is_enabled(&self) -> bool {
        self.config.enabled
            && cfg!(feature = "export-teacher")
    }
    
    /// Get sequence name
    pub fn sequence_name(&self) -> &str {
        &self.config.sequence_name
    }
    
    /// Get output directory
    pub fn output_dir(&self) -> &PathBuf {
        &self.config.output_dir
    }
    
    /// Export a frame (no-op if export disabled or feature not compiled)
    #[cfg(feature = "export-teacher")]
    pub fn export_frame(&mut self, frame: TeacherFrame) -> Result<()> {
        if let Some(ref mut exporter) = self.exporter {
            exporter.export_frame(frame)?;
        }
        Ok(())
    }
    
    #[cfg(not(feature = "export-teacher"))]
    pub fn export_frame(&mut self, _frame: impl std::any::Any) -> Result<()> {
        Ok(())
    }
    
    /// Finalize export (no-op if export disabled or feature not compiled)
    #[cfg(feature = "export-teacher")]
    pub fn finalize(&mut self) -> Result<()> {
        if let Some(ref mut exporter) = self.exporter {
            exporter.finalize()?;
        }
        Ok(())
    }
    
    #[cfg(not(feature = "export-teacher"))]
    pub fn finalize(&mut self) -> Result<()> {
        Ok(())
    }
}

#[cfg(feature = "export-teacher")]
impl Drop for ExportManager {
    fn drop(&mut self) {
        // Finalize export on drop to ensure metadata is written
        if let Err(e) = self.finalize() {
            log::error!("Failed to finalize export: {:?}", e);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;
    
    #[test]
    fn test_export_manager_creation() {
        let dir = tempdir().unwrap();
        let config = ExportConfig {
            output_dir: dir.path().to_path_buf(),
            sequence_name: "test".to_string(),
            enabled: false,  // Disabled for test
        };
        let manager = ExportManager::new(config);
        assert!(manager.is_ok());
    }
}
