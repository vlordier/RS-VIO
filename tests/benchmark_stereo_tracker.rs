use image::{GrayImage, Luma};
use rs_vio::feature_tracker::StereoPatchTracker;
use rs_vio::estimator::Frame;
use std::time::Instant;
// Unused imports removed


fn create_test_image(width: u32, height: u32, pattern_offset: u32) -> GrayImage {
    let mut img = GrayImage::new(width, height);
    for y in 0..height {
        for x in 0..width {
            // Create a pattern that can be tracked (gradients)
            let val = ((x + y + pattern_offset) % 255) as u8;
            img.put_pixel(x, y, Luma([val]));
        }
    }
    img
}

#[test]
fn bench_stereo_tracker_process_frame() {
    let width = 640;
    let height = 480;
    
    // Create tracker with typical settings
    let mut tracker = StereoPatchTracker::<4>::new(30, 30, 0.01);
    
    // Warmup frames
    let img0 = create_test_image(width, height, 0);
    let img1 = create_test_image(width, height, 5); // Stereo offset

    // Create a dummy frame
    let mut frame = Frame::new(0, 0);
    
    // Warmup
    tracker.process_frame(&img0, &img1, &mut frame);

    let start = Instant::now();
    let iterations = 10;
    for i in 0..iterations {
        let mut f = Frame::new((i * 100) as i64, i as i32);
        // Change images slightly to force tracking
        let curr0 = create_test_image(width, height, i as u32);
        let curr1 = create_test_image(width, height, i as u32 + 5);
        tracker.process_frame(&curr0, &curr1, &mut f);
    }
    let duration = start.elapsed();
    
    println!("BENCHMARK: StereoPatchTracker::process_frame avg time: {:?} per frame", duration / iterations);
}
