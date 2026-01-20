//! Stub for Rerun viewer - used when rerun-viewer feature is disabled
//! Maintains API compatibility while eliminating ~30% IR bloat

use crate::datasets::config::VisualizationConfig;
use crate::types::Matrix4x4;
use crate::Result;
use super::Viewer;

pub struct RerunViewer;

impl RerunViewer {
    pub fn new() -> Self {
        Self
    }

    pub fn new_with_config(_config: &VisualizationConfig) -> Self {
        Self
    }
}

impl Default for RerunViewer {
    fn default() -> Self {
        Self::new()
    }
}

impl Viewer for RerunViewer {
    fn initialize(&mut self) -> Result<()> {
        Ok(())
    }

    fn log_pose(&mut self, _: Matrix4x4, _: &str) {}
    fn log_image_raw(&mut self, _: &[u8], _: u32, _: u32, _: &str) {}
    fn log_image_equalized(&mut self, _: &[u8], _: u32, _: u32, _: &str) {}
    fn log_image_with_features(&mut self, _: &[u8], _: u32, _: u32, _: &[[f32; 2]], _: &str) {}
    fn log_image_with_features_colored(&mut self, _: &[u8], _: u32, _: u32, _: &[(usize, [f32; 2])], _: &str) {}
    fn log_points(&mut self, _: &[[f32; 3]], _: &str) {}
    fn log_points_colored(&mut self, _: &[(usize, [f32; 3])], _: &str) {}
    fn set_frame(&mut self, _: i64) {}
    fn log_camera_frustum(&mut self, _: f32, _: u32, _: u32, _: &str, _: f32) {}
    fn log_trajectory(&mut self, _: &[Matrix4x4], _: &str) {}
}

pub fn create_viewer(config: &VisualizationConfig) -> Result<Box<dyn Viewer>> {
    Ok(Box::new(RerunViewer::new_with_config(config)))
}

