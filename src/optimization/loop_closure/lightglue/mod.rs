//! LightGlue ONNX-based feature matching for loop closure verification.
//!
//! This module provides deep learning-based feature matching using LightGlue
//! via ONNX Runtime. LightGlue is designed for robust matching in challenging
//! conditions where traditional geometric methods may fail.
//!
//! References:
//! - Lindenberger et al., "LightGlue: Local Feature Matching at Light Speed", ICCV 2023
//! - ONNX Runtime for Rust: https://github.com/pykeio/ort
//! - LightGlue-ONNX: https://github.com/fabio-sim/LightGlue-ONNX

pub mod config;
pub mod downloader;
pub mod feature_extraction;
pub mod matching;
pub mod model;

#[cfg(test)]
mod tests;

#[cfg(feature = "lightglue")]
use ort::session::Session;

use super::{DescriptorMatcher, MatchMetrics};
pub use config::LightGlueConfig;
pub use downloader::LightGlueModelDownloader;

/// LightGlue-based descriptor matcher using ONNX Runtime
#[allow(dead_code)]
#[derive(Debug)]
pub struct LightGlueMatcher {
    config: LightGlueConfig,
    #[cfg(feature = "lightglue")]
    pub session: Option<Session>,
}

impl LightGlueMatcher {
    /// Create new LightGlue matcher
    pub fn new(config: LightGlueConfig) -> Result<Self, String> {
        #[cfg(feature = "lightglue")]
        {
            let session = model::load_model(&config)?;
            Ok(Self {
                config,
                session: Some(session),
            })
        }

        #[cfg(not(feature = "lightglue"))]
        {
            log::warn!("LightGlue feature not enabled. Compile with --features lightglue");
            Ok(Self { config })
        }
    }

    /// Get reference to config
    pub fn config(&self) -> &LightGlueConfig {
        &self.config
    }

    #[cfg(feature = "lightglue")]
    /// Run inference with keypoints and descriptors
    pub fn run_inference(
        &mut self,
        keypoints0: &ndarray::Array2<f32>,
        keypoints1: &ndarray::Array2<f32>,
        descriptors0: &ndarray::Array2<f32>,
        descriptors1: &ndarray::Array2<f32>,
    ) -> Result<(Vec<(usize, usize)>, Vec<f32>), String> {
        let session = self
            .session
            .as_mut()
            .ok_or_else(|| "ONNX session not initialized".to_string())?;

        matching::run_inference(
            session,
            self.config.confidence_threshold,
            keypoints0,
            keypoints1,
            descriptors0,
            descriptors1,
        )
    }
}

impl DescriptorMatcher for LightGlueMatcher {
    fn match_keyframes(
        &self,
        _query: &super::KeyframeDescriptor,
        _candidate: &super::KeyframeDescriptor,
    ) -> MatchMetrics {
        #[cfg(feature = "lightglue")]
        {
            log::warn!("LightGlue matching not fully implemented - requires image data");

            MatchMetrics {
                similarity: 0.0,
                match_count: 0,
                match_ratio: 0.0,
            }
        }

        #[cfg(not(feature = "lightglue"))]
        {
            log::error!("LightGlue feature not enabled. Compile with --features lightglue");
            MatchMetrics {
                similarity: 0.0,
                match_count: 0,
                match_ratio: 0.0,
            }
        }
    }
}
