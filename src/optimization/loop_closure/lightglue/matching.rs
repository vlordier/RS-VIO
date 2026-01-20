//! ONNX inference and matching algorithms for LightGlue.

#[cfg(feature = "lightglue")]
use ndarray::Array2;
#[cfg(feature = "lightglue")]
use ort::value::Value;

#[cfg(feature = "lightglue")]
/// Run ONNX inference with keypoints and descriptors
pub fn run_inference(
    session: &mut ort::session::Session,
    confidence_threshold: f32,
    keypoints0: &Array2<f32>,
    keypoints1: &Array2<f32>,
    descriptors0: &Array2<f32>,
    descriptors1: &Array2<f32>,
) -> Result<(Vec<(usize, usize)>, Vec<f32>), String> {
    // Prepare inputs: keypoints (N, 2), descriptors (N, D)
    let kpts0_shape = vec![keypoints0.nrows() as i64, keypoints0.ncols() as i64];
    let kpts1_shape = vec![keypoints1.nrows() as i64, keypoints1.ncols() as i64];
    let desc0_shape = vec![descriptors0.nrows() as i64, descriptors0.ncols() as i64];
    let desc1_shape = vec![descriptors1.nrows() as i64, descriptors1.ncols() as i64];

    let kpts0 =
        Value::from_array((kpts0_shape, keypoints0.clone().into_raw_vec_and_offset().0))
            .map_err(|e| format!("Failed to create keypoints0 tensor: {}", e))?;
    let kpts1 =
        Value::from_array((kpts1_shape, keypoints1.clone().into_raw_vec_and_offset().0))
            .map_err(|e| format!("Failed to create keypoints1 tensor: {}", e))?;
    let desc0 = Value::from_array((
        desc0_shape,
        descriptors0.clone().into_raw_vec_and_offset().0,
    ))
    .map_err(|e| format!("Failed to create descriptors0 tensor: {}", e))?;
    let desc1 = Value::from_array((
        desc1_shape,
        descriptors1.clone().into_raw_vec_and_offset().0,
    ))
    .map_err(|e| format!("Failed to create descriptors1 tensor: {}", e))?;

    // Run inference
    let inputs = ort::inputs![
        "keypoints0" => kpts0,
        "keypoints1" => kpts1,
        "descriptors0" => desc0,
        "descriptors1" => desc1
    ];

    let outputs = session
        .run(inputs)
        .map_err(|e| format!("Inference failed: {}", e))?;

    // Extract matches: indices (M, 2) and scores (M,)
    let (matches_shape, matches_data) = outputs[0]
        .try_extract_tensor::<i64>()
        .map_err(|e| format!("Failed to extract matches: {}", e))?;
    let (_scores_shape, scores_data) = outputs[1]
        .try_extract_tensor::<f32>()
        .map_err(|e| format!("Failed to extract scores: {}", e))?;

    let mut matches = Vec::new();
    let mut confidences = Vec::new();

    // Filter by confidence threshold
    let num_matches = matches_shape[0] as usize;
    for i in 0..num_matches {
        let score = scores_data[i];
        if score >= confidence_threshold {
            let idx0 = matches_data[i * 2] as usize;
            let idx1 = matches_data[i * 2 + 1] as usize;
            matches.push((idx0, idx1));
            confidences.push(score);
        }
    }

    Ok((matches, confidences))
}
