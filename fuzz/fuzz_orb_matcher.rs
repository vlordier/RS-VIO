#![no_main]

use libfuzzer_sys::fuzz_target;
use rs_vio::optimization::loop_closure::orb_matcher::OrbMatcher;
use rs_vio::optimization::loop_closure::KeyframeDescriptor;
use rs_vio::types::{Float, Matrix4x4};

fuzz_target!(|data: &[u8]| {
    // Only proceed if we have enough data for basic structures
    if data.len() < 100 {
        return;
    }

    let matcher = OrbMatcher::new();

    // Create two keyframe descriptors from the fuzz data
    let desc1 = create_descriptor_from_data(&data[0..50]);
    let desc2 = create_descriptor_from_data(&data[50..100]);

    // Fuzz the matching process
    let _metrics = matcher.match_keyframes(&desc1, &desc2);

    // Additional fuzzing: try with corrupted descriptors
    if data.len() > 200 {
        let corrupted_desc = create_corrupted_descriptor(&data[100..200]);
        let _corrupted_metrics = matcher.match_keyframes(&desc1, &corrupted_desc);
    }
});

fn create_descriptor_from_data(data: &[u8]) -> KeyframeDescriptor {
    let mut descriptor = Vec::new();

    // Convert bytes to Float descriptors (simplified)
    for chunk in data.chunks(4) {
        if chunk.len() == 4 {
            let value = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]) as Float;
            // Normalize to reasonable range
            let normalized = (value.sin() + 1.0) / 2.0;
            descriptor.push(normalized);
        }
    }

    // Ensure we have at least some descriptors
    while descriptor.len() < 32 {
        descriptor.push(0.5);
    }

    // Truncate to reasonable size
    descriptor.truncate(256);

    KeyframeDescriptor {
        keyframe_id: 1,
        timestamp: 1000000,
        descriptor,
        num_features: 100,
        pose: Matrix4x4::identity(),
    }
}

fn create_corrupted_descriptor(data: &[u8]) -> KeyframeDescriptor {
    let mut desc = create_descriptor_from_data(data);

    // Introduce corruption
    for i in 0..desc.descriptor.len().min(10) {
        desc.descriptor[i] = Float::NAN; // or INF, or extreme values
    }

    desc
}
