use image::{GrayImage, Luma};
use rs_vio::estimator::Frame;
use rs_vio::feature_tracker::StereoPatchTracker;
use std::time::Instant;
// Unused imports removed

fn create_test_image(width: u32, height: u32, pattern_offset: u32) -> GrayImage {
    let mut img = GrayImage::new(width, height);
    for y in 0..height {
        for x in 0..width {
            // Checkerboard pattern to ensure FAST corners are detected
            // Shift pattern with offset to simulate motion
            let check_x = (x + pattern_offset) / 30;
            let check_y = (y + pattern_offset) / 30;
            let val: u8 = if (check_x + check_y).is_multiple_of(2) {
                200
            } else {
                50
            };

            // Add some noise to prevent perfect gradients if using gradient-based tracking
            let noise = ((x * 37 + y * 73) % 20) as u8;
            img.put_pixel(x, y, Luma([val.saturating_add(noise)]));
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
    let mut total_features = 0;
    for i in 0..iterations {
        let mut f = Frame::new((i * 100) as i64, i as i32);
        // Change images slightly to force tracking
        let curr0 = create_test_image(width, height, i);
        let curr1 = create_test_image(width, height, i + 5);
        tracker.process_frame(&curr0, &curr1, &mut f);
        total_features += f.left_features.len();
    }
    let duration = start.elapsed();

    println!(
        "BENCHMARK: StereoPatchTracker::process_frame avg time: {:?} per frame",
        duration / iterations
    );
    println!(
        "Average features per frame: {}",
        total_features / iterations as usize
    );
}
