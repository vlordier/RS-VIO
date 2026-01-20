//! Feature extraction and preprocessing utilities.

#[cfg(feature = "lightglue")]
use ndarray::Array2;

/// Convert byte descriptors to f32 for ONNX
#[cfg(feature = "lightglue")]
pub fn descriptors_to_f32(descriptors: &[Vec<u8>]) -> Array2<f32> {
    let n = descriptors.len();
    let d = descriptors[0].len();

    let mut array = Array2::zeros((n, d));
    for (i, desc) in descriptors.iter().enumerate() {
        for (j, &byte) in desc.iter().enumerate() {
            array[[i, j]] = byte as f32 / 255.0;
        }
    }
    array
}
