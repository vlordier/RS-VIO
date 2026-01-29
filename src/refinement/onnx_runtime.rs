// ONNX Runtime integration for refinement network

use super::RefinementResult;
use nalgebra as na;
use ort::{GraphOptimizationLevel, Session};
use std::time::Instant;

/// Neural network refinement using ONNX Runtime
pub struct RefinementNetwork {
    session: Session,
    image_width: usize,
    image_height: usize,
}

impl RefinementNetwork {
    /// Create new refinement network from ONNX model file
    pub fn new(model_path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        // Configure ONNX Runtime for CPU execution
        let session = Session::builder()?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_intra_threads(4)? // Use 4 threads for parallel ops
            .commit_from_file(model_path)?;
        
        log::info!("[RefinementNetwork] Loaded ONNX model from {}", model_path);
        log::info!("[RefinementNetwork] Input names: {:?}", 
            session.inputs.iter().map(|i| &i.name).collect::<Vec<_>>());
        log::info!("[RefinementNetwork] Output names: {:?}", 
            session.outputs.iter().map(|o| &o.name).collect::<Vec<_>>());
        
        Ok(Self {
            session,
            image_width: 640,
            image_height: 480,
        })
    }
    
    /// Refine VIO position estimate using neural network
    ///
    /// # Arguments
    /// * `image` - Grayscale image (H x W bytes, row-major)
    /// * `flow` - Optical flow (96 values: 8x6 grid)
    /// * `imu` - IMU preintegration (15 values: dt, delta_p, delta_v, delta_q)
    /// * `vio_position` - Current VIO position estimate
    ///
    /// # Returns
    /// Refinement result with correction delta and refined position
    pub fn refine(
        &self,
        image: &[u8],
        flow: &[f32],
        imu: &[f32],
        vio_position: na::Vector3<f32>,
    ) -> Result<RefinementResult, Box<dyn std::error::Error>> {
        let start = Instant::now();
        
        // Validate inputs
        if image.len() != self.image_width * self.image_height {
            return Err(format!(
                "Invalid image size: expected {}x{}, got {} bytes",
                self.image_width, self.image_height, image.len()
            ).into());
        }
        if flow.len() != 96 {
            return Err(format!("Invalid flow size: expected 96, got {}", flow.len()).into());
        }
        if imu.len() != 15 {
            return Err(format!("Invalid IMU size: expected 15, got {}", imu.len()).into());
        }
        
        // Prepare input tensors
        // Image: [batch=1, channels=1, height, width]
        let mut image_tensor = vec![0.0f32; 1 * 1 * self.image_height * self.image_width];
        for (i, &pixel) in image.iter().enumerate() {
            image_tensor[i] = pixel as f32 / 255.0;
        }
        
        // Flow: [batch=1, 96]
        let flow_tensor: Vec<f32> = flow.to_vec();
        
        // IMU: [batch=1, 15]
        let imu_tensor: Vec<f32> = imu.to_vec();
        
        // VIO estimate: [batch=1, 3]
        let vio_tensor = vec![vio_position.x, vio_position.y, vio_position.z];
        
        // Create input arrays with proper shapes
        use ndarray::{Array, IxDyn};
        
        let left_image = Array::from_shape_vec(
            IxDyn(&[1, 1, self.image_height, self.image_width]),
            image_tensor
        )?;
        
        let flow_array = Array::from_shape_vec(IxDyn(&[1, 96]), flow_tensor)?;
        let imu_array = Array::from_shape_vec(IxDyn(&[1, 15]), imu_tensor)?;
        let vio_array = Array::from_shape_vec(IxDyn(&[1, 3]), vio_tensor)?;
        
        // Run inference
        let outputs = self.session.run(ort::inputs![
            "left_image" => left_image.view(),
            "flow" => flow_array.view(),
            "imu" => imu_array.view(),
            "vio_estimate" => vio_array.view(),
        ]?)?;
        
        // Extract correction from output
        let correction_tensor = outputs["correction"].try_extract_tensor::<f32>()?;
        let correction_slice = correction_tensor.as_slice().ok_or("Failed to extract correction")?;
        
        if correction_slice.len() != 3 {
            return Err(format!("Invalid correction size: expected 3, got {}", correction_slice.len()).into());
        }
        
        let correction = na::Vector3::new(
            correction_slice[0],
            correction_slice[1],
            correction_slice[2],
        );
        
        let refined_position = vio_position + correction;
        let latency_us = start.elapsed().as_micros() as u64;
        
        Ok(RefinementResult {
            vio_position,
            correction,
            refined_position,
            latency_us,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    #[ignore] // Only run when ONNX model is available
    fn test_refinement_network() {
        let model_path = "results/refinement_model.onnx";
        let network = RefinementNetwork::new(model_path).unwrap();
        
        // Create dummy inputs
        let image = vec![128u8; 640 * 480];
        let flow = vec![0.0f32; 96];
        let imu = vec![0.0f32; 15];
        let vio_pos = na::Vector3::new(1.0, 2.0, 3.0);
        
        let result = network.refine(&image, &flow, &imu, vio_pos).unwrap();
        
        println!("VIO position: {:?}", result.vio_position);
        println!("Correction: {:?}", result.correction);
        println!("Refined position: {:?}", result.refined_position);
        println!("Latency: {} μs", result.latency_us);
        
        // Check that correction is reasonable (< 1m)
        assert!(result.correction.norm() < 1.0);
    }
}
