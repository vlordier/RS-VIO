// Neural network-based position refinement module
//
// This module integrates an ONNX-based refinement network that learns to correct
// VIO position estimates using visual features, IMU data, and optical flow.
//
// The network is trained to predict position corrections (deltas), not absolute positions.
// This makes it robust to VIO drift and allows it to refine existing estimates.

#[cfg(feature = "onnx-refinement")]
mod onnx_runtime;

#[cfg(feature = "onnx-refinement")]
pub use onnx_runtime::RefinementNetwork;

use nalgebra as na;

/// Position refinement result
#[derive(Debug, Clone)]
pub struct RefinementResult {
    /// Original VIO position estimate
    pub vio_position: na::Vector3<f32>,
    /// Predicted correction delta
    pub correction: na::Vector3<f32>,
    /// Refined position (vio_position + correction)
    pub refined_position: na::Vector3<f32>,
    /// Inference latency in microseconds
    pub latency_us: u64,
}

/// Stub implementation when onnx-refinement feature is disabled
#[cfg(not(feature = "onnx-refinement"))]
pub struct RefinementNetwork;

#[cfg(not(feature = "onnx-refinement"))]
impl RefinementNetwork {
    pub fn new(_model_path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self)
    }

    pub fn refine(
        &self,
        _image: &[u8],
        _flow: &[f32],
        _imu: &[f32],
        vio_position: na::Vector3<f32>,
    ) -> Result<RefinementResult, Box<dyn std::error::Error>> {
        // Return identity refinement (no correction)
        Ok(RefinementResult {
            vio_position,
            correction: na::Vector3::zeros(),
            refined_position: vio_position,
            latency_us: 0,
        })
    }
}
