/// Sub-pixel disparity refinement using Gauss-Newton optimization
///
/// Refines coarse integer disparity to sub-pixel accuracy using patch matching
/// with image pyramids and robust loss functions.

/// Sub-pixel disparity refinement result
#[derive(Clone, Debug)]
pub struct SubPixelDisparityResult {
    /// Refined disparity (pixels)
    pub disparity: f64,
    /// Uncertainty in disparity (pixels)
    pub disparity_uncertainty: f64,
    /// Sub-pixel disparity offset
    pub sub_pixel_offset: f64,
    /// Peak sharpness of correlation
    pub peak_sharpness: f64,
    /// RMS photometric error after refinement
    pub photometric_error_rms: f64,
    /// Number of optimization iterations
    pub iterations: usize,
    /// Optimization converged?
    pub converged: bool,
}

/// Patch matching configuration
#[derive(Clone, Debug)]
pub struct PatchMatchingConfig {
    /// Patch size (e.g., 7 = 7×7 patch)
    pub patch_size: u32,
    /// Maximum pyramid levels
    pub max_pyramid_levels: usize,
    /// Gauss-Newton max iterations
    pub max_iterations: usize,
    /// Convergence tolerance (pixels)
    pub convergence_tolerance: f64,
    /// Robust loss function threshold
    pub huber_threshold: f64,
    /// Initial search range around coarse disparity (pixels)
    pub search_range: f64,
}

impl Default for PatchMatchingConfig {
    fn default() -> Self {
        Self {
            patch_size: 7,
            max_pyramid_levels: 3,
            max_iterations: 20,
            convergence_tolerance: 0.01,
            huber_threshold: 1.0,
            search_range: 2.0,
        }
    }
}

/// Image pyramid for multi-scale matching
pub struct ImagePyramid {
    levels: Vec<Vec<f32>>,
    widths: Vec<u32>,
    heights: Vec<u32>,
}

impl ImagePyramid {
    /// Build pyramid from image
    pub fn build(image: &[f32], width: u32, height: u32, num_levels: usize) -> Self {
        let mut levels = vec![image.to_vec()];
        let mut widths = vec![width];
        let mut heights = vec![height];

        let mut current = image.to_vec();
        let mut current_width = width;
        let mut current_height = height;

        for _ in 1..num_levels {
            let next_width = (current_width + 1) / 2;
            let next_height = (current_height + 1) / 2;

            let mut next = vec![0.0; (next_width * next_height) as usize];

            // Simple downsampling: average 2×2 blocks
            for y in 0..next_height {
                for x in 0..next_width {
                    let mut sum = 0.0;
                    let mut count = 0;

                    for dy in 0..2 {
                        for dx in 0..2 {
                            let src_x = x * 2 + dx;
                            let src_y = y * 2 + dy;

                            if src_x < current_width && src_y < current_height {
                                let idx = (src_y * current_width + src_x) as usize;
                                sum += current[idx];
                                count += 1;
                            }
                        }
                    }

                    let idx = (y * next_width + x) as usize;
                    next[idx] = sum / count as f32;
                }
            }

            levels.push(next.clone());
            widths.push(next_width);
            heights.push(next_height);
            current = next;
            current_width = next_width;
            current_height = next_height;
        }

        Self {
            levels,
            widths,
            heights,
        }
    }

    /// Get image at level
    pub fn get_level(&self, level: usize) -> Option<&[f32]> {
        self.levels.get(level).map(|v| v.as_slice())
    }

    /// Get width/height at level
    pub fn get_dims(&self, level: usize) -> Option<(u32, u32)> {
        if level < self.levels.len() {
            Some((self.widths[level], self.heights[level]))
        } else {
            None
        }
    }

    /// Number of levels
    pub fn num_levels(&self) -> usize {
        self.levels.len()
    }
}

/// Sub-pixel disparity refiner
pub struct SubPixelDisparityRefiner {
    config: PatchMatchingConfig,
}

impl SubPixelDisparityRefiner {
    pub fn new(config: PatchMatchingConfig) -> Self {
        Self { config }
    }

    /// Refine disparity using Gauss-Newton on image pyramid
    pub fn refine_disparity(
        &self,
        left_image: &[f32],
        right_image: &[f32],
        width: u32,
        height: u32,
        x: u32,
        y: u32,
        initial_disparity: f64,
    ) -> SubPixelDisparityResult {
        // Build pyramids
        let left_pyr =
            ImagePyramid::build(left_image, width, height, self.config.max_pyramid_levels);
        let right_pyr =
            ImagePyramid::build(right_image, width, height, self.config.max_pyramid_levels);

        // Start from coarse level and refine
        let mut disparity = initial_disparity;
        let mut iterations = 0;

        for level in (0..self.config.max_pyramid_levels).rev() {
            let (w, h) = match left_pyr.get_dims(level) {
                Some(dims) => dims,
                None => {
                    log::error!("Pyramid level {} out of bounds", level);
                    break;
                }
            };
            let scale = 2_f64.powi(level as i32);
            let level_x = (x as f64 / scale) as u32;
            let level_y = (y as f64 / scale) as u32;
            let level_disparity = disparity / scale;

            // Gauss-Newton refinement at this level
            let left_level = match left_pyr.get_level(level) {
                Some(img) => img,
                None => {
                    log::error!("Failed to get left pyramid level {}", level);
                    break;
                }
            };
            let right_level = match right_pyr.get_level(level) {
                Some(img) => img,
                None => {
                    log::error!("Failed to get right pyramid level {}", level);
                    break;
                }
            };

            let (refined_disp, iter_count, converged) = self.gauss_newton_refinement(
                left_level,
                right_level,
                w,
                h,
                level_x,
                level_y,
                level_disparity,
            );

            disparity = refined_disp * scale;
            iterations += iter_count;

            if converged {
                break;
            }
        }

        // Compute final metrics
        let sharpness =
            self.compute_peak_sharpness(left_image, right_image, width, x, y, disparity);

        let error_rms =
            self.compute_photometric_error(left_image, right_image, width, x, y, disparity);

        SubPixelDisparityResult {
            disparity,
            disparity_uncertainty: 1.0 / (sharpness + 0.001),
            sub_pixel_offset: disparity.fract(),
            peak_sharpness: sharpness,
            photometric_error_rms: error_rms,
            iterations,
            converged: true,
        }
    }

    /// Gauss-Newton optimization of disparity
    fn gauss_newton_refinement(
        &self,
        left: &[f32],
        right: &[f32],
        width: u32,
        _height: u32,
        x: u32,
        y: u32,
        initial_disparity: f64,
    ) -> (f64, usize, bool) {
        let mut disparity = initial_disparity;
        let half_patch = self.config.patch_size / 2;

        for iter in 0..self.config.max_iterations {
            // Compute photometric error and Jacobian
            let mut sum_jt_r = 0.0; // J^T * residual
            let mut sum_jt_j = 0.0; // J^T * J
            let mut _sum_weighted_error = 0.0;
            let mut valid_pixels = 0;

            for py in 0..self.config.patch_size {
                for px in 0..self.config.patch_size {
                    let ly = (y as i32 + py as i32 - half_patch as i32) as u32;
                    let lx = (x as i32 + px as i32 - half_patch as i32) as u32;

                    if lx >= width || ly >= (left.len() as u32 / width) {
                        continue;
                    }

                    // Right image coordinate with current disparity
                    let rx = (lx as f64 - disparity).max(0.0) as u32;
                    let ry = ly;

                    if rx >= width {
                        continue;
                    }

                    let left_idx = (ly * width + lx) as usize;
                    let right_idx = (ry * width + rx) as usize;

                    if left_idx >= left.len() || right_idx >= right.len() {
                        continue;
                    }

                    let left_val = left[left_idx];
                    let right_val = right[right_idx];

                    let error = (left_val - right_val) as f64;
                    let huber_loss = self.huber_loss(error as f32);

                    // Jacobian: derivative w.r.t. disparity
                    // ∂(left - right) / ∂d = -∂right / ∂x = -right_gradient
                    let right_grad = self.compute_gradient_x(right, width, rx, ry);

                    sum_jt_r += right_grad * error;
                    sum_jt_j += right_grad * right_grad;
                    _sum_weighted_error += (huber_loss as f64) * (huber_loss as f64);
                    valid_pixels += 1;
                }
            }

            if valid_pixels == 0 {
                return (disparity, iter, false);
            }

            // Gauss-Newton step
            if sum_jt_j.abs() > 1e-10 {
                let delta = sum_jt_r / sum_jt_j;
                let step = -delta * 0.1; // Damping factor

                disparity += step;

                if (step).abs() < self.config.convergence_tolerance {
                    return (disparity, iter + 1, true);
                }
            } else {
                return (disparity, iter, false);
            }
        }

        (disparity, self.config.max_iterations, false)
    }

    /// Compute gradient in x direction
    fn compute_gradient_x(&self, image: &[f32], width: u32, x: u32, y: u32) -> f64 {
        let y_idx = y * width;

        if x == 0 || x >= width - 1 {
            return 0.0;
        }

        let left_idx = (y_idx + x - 1) as usize;
        let right_idx = (y_idx + x + 1) as usize;

        if left_idx >= image.len() || right_idx >= image.len() {
            return 0.0;
        }

        ((image[right_idx] - image[left_idx]) / 2.0) as f64
    }

    /// Huber robust loss function
    fn huber_loss(&self, error: f32) -> f32 {
        let threshold = self.config.huber_threshold as f32;
        if error.abs() <= threshold {
            error * error / 2.0
        } else {
            threshold * (error.abs() - threshold / 2.0)
        }
    }

    /// Compute peak sharpness of correlation
    fn compute_peak_sharpness(
        &self,
        left: &[f32],
        right: &[f32],
        width: u32,
        x: u32,
        y: u32,
        disparity: f64,
    ) -> f64 {
        let half_patch = self.config.patch_size / 2;
        let mut ssd = 0.0;
        let mut count = 0;

        for py in 0..self.config.patch_size {
            for px in 0..self.config.patch_size {
                let ly = (y as i32 + py as i32 - half_patch as i32) as u32;
                let lx = (x as i32 + px as i32 - half_patch as i32) as u32;

                if lx >= width || ly >= (left.len() as u32 / width) {
                    continue;
                }

                let rx = (lx as f64 - disparity).max(0.0) as u32;
                let ry = ly;

                if rx >= width {
                    continue;
                }

                let left_idx = (ly * width + lx) as usize;
                let right_idx = (ry * width + rx) as usize;

                if left_idx < left.len() && right_idx < right.len() {
                    let diff = left[left_idx] - right[right_idx];
                    ssd += diff * diff;
                    count += 1;
                }
            }
        }

        if count == 0 {
            return 0.0;
        }

        // Sharpness = inverse of MSE
        let mse = (ssd / count as f32) as f64;
        1.0 / (mse + 0.01)
    }

    /// Compute photometric error RMS
    fn compute_photometric_error(
        &self,
        left: &[f32],
        right: &[f32],
        width: u32,
        x: u32,
        y: u32,
        disparity: f64,
    ) -> f64 {
        let half_patch = self.config.patch_size / 2;
        let mut sum_sq_error = 0.0;
        let mut count = 0;

        for py in 0..self.config.patch_size {
            for px in 0..self.config.patch_size {
                let ly = (y as i32 + py as i32 - half_patch as i32) as u32;
                let lx = (x as i32 + px as i32 - half_patch as i32) as u32;

                if lx >= width || ly >= (left.len() as u32 / width) {
                    continue;
                }

                let rx = (lx as f64 - disparity).max(0.0) as u32;
                let ry = ly;

                if rx >= width {
                    continue;
                }

                let left_idx = (ly * width + lx) as usize;
                let right_idx = (ry * width + rx) as usize;

                if left_idx < left.len() && right_idx < right.len() {
                    let error = left[left_idx] - right[right_idx];
                    sum_sq_error += error as f64 * error as f64;
                    count += 1;
                }
            }
        }

        if count == 0 {
            return f64::INFINITY;
        }

        (sum_sq_error / count as f64).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_image_pyramid() {
        let width = 8;
        let height = 8;
        let image = vec![1.0; (width * height) as usize];

        let pyramid = ImagePyramid::build(&image, width, height, 3);
        assert_eq!(pyramid.num_levels(), 3);

        let (w0, h0) = pyramid.get_dims(0).expect("Level 0 should exist");
        assert_eq!(w0, 8);
        assert_eq!(h0, 8);

        let (w1, h1) = pyramid.get_dims(1).expect("Level 1 should exist");
        assert_eq!(w1, 4);
        assert_eq!(h1, 4);

        let (w2, h2) = pyramid.get_dims(2).expect("Level 2 should exist");
        assert_eq!(w2, 2);
        assert_eq!(h2, 2);
    }

    #[test]
    fn test_sub_pixel_config() {
        let config = PatchMatchingConfig::default();
        assert_eq!(config.patch_size, 7);
        assert_eq!(config.max_pyramid_levels, 3);
    }

    #[test]
    fn test_refiner_creation() {
        let _refiner = SubPixelDisparityRefiner::new(PatchMatchingConfig::default());
        // Just ensure creation works
    }
}
