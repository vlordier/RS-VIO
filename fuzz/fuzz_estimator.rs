#![no_main]

use libfuzzer_sys::fuzz_target;
use rs_vio::config::VioConfig;
use rs_vio::estimator::estimator::Estimator;
use rs_vio::types::{Float, Vector3};

fuzz_target!(|data: &[u8]| {
    // Need minimum data for a basic frame
    if data.len() < 20 {
        return;
    }

    let config = VioConfig::default();
    let mut estimator = Estimator::new(config);

    // Create a synthetic frame from fuzz data
    let frame = create_frame_from_data(data);

    // Fuzz the frame processing
    let _result = estimator.process_frame(frame);

    // Additional fuzzing: try with corrupted frames
    if data.len() > 100 {
        let corrupted_frame = create_corrupted_frame(&data[50..]);
        let _corrupted_result = estimator.process_frame(corrupted_frame);
    }
});

fn create_frame_from_data(data: &[u8]) -> rs_vio::types::Frame {
    let mut left_features = Vec::new();
    let mut right_features = Vec::new();

    // Convert bytes to features (simplified)
    for (i, chunk) in data.chunks(6).enumerate() {
        if chunk.len() >= 6 && i < 50 {
            // Limit features for performance
            let x = f32::from_le_bytes([chunk[0], chunk[1], 0, 0]) as Float;
            let y = f32::from_le_bytes([chunk[2], chunk[3], 0, 0]) as Float;
            let feature = Vector3::new(x, y, 1.0);

            if i % 2 == 0 {
                left_features.push(feature);
            } else {
                right_features.push(feature);
            }
        }
    }

    rs_vio::types::Frame {
        timestamp: 1000000,
        left_features,
        right_features,
        ..Default::default()
    }
}

fn create_corrupted_frame(data: &[u8]) -> rs_vio::types::Frame {
    let mut frame = create_frame_from_data(data);

    // Introduce corruption
    for feature in &mut frame.left_features {
        feature.x = Float::NAN; // or extreme values
        feature.y = Float::INFINITY;
    }

    frame
}
