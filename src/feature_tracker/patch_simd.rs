#![allow(unsafe_code)]

/// SIMD-optimized patch matching for real-time performance
///
/// This module provides vectorized implementations of patch-based tracking operations
/// using platform-specific SIMD intrinsics when available.
///
/// # Safety
/// This module uses unsafe code for SIMD intrinsics (AVX2, SSE4.1).
/// All unsafe blocks are guarded by runtime feature detection.
use nalgebra as na;

#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::*;

use super::patch::PATTERN52_SIZE;

/// Compute normalized residuals using SIMD operations
///
/// Computes residuals as: (sampled - template) normalized by mean and count
/// This vectorizes across the 52 pattern points, processing 4-8 points simultaneously.
///
/// # Arguments
/// * `sampled` - Current frame intensities at pattern points
/// * `template` - Template patch intensities
/// * `_template_mean` - Pre-computed mean of template patch (for consistency checking)
/// * `num_valid` - Number of valid (in-bounds) samples
/// * `sample_sum` - Sum of sampled intensities
#[inline]
pub fn compute_residuals_simd(
    sampled: &[f32; PATTERN52_SIZE],
    template: [f32; PATTERN52_SIZE],
    _template_mean: f32,
    num_valid: f32,
    sample_sum: f32,
) -> na::SVector<f32, PATTERN52_SIZE> {
    let mut residuals = na::SVector::<f32, PATTERN52_SIZE>::zeros();

    #[cfg(target_arch = "x86_64")]
    {
        if is_x86_feature_detected!("avx2") {
            unsafe {
                compute_residuals_avx2(
                    sampled,
                    template,
                    _template_mean,
                    num_valid,
                    sample_sum,
                    &mut residuals,
                )
            }
        } else if is_x86_feature_detected!("sse4.1") {
            unsafe {
                compute_residuals_sse(
                    sampled,
                    template,
                    _template_mean,
                    num_valid,
                    sample_sum,
                    &mut residuals,
                )
            }
        } else {
            compute_residuals_scalar(
                sampled,
                template,
                _template_mean,
                num_valid,
                sample_sum,
                &mut residuals,
            );
        }
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        compute_residuals_scalar(
            sampled,
            template,
            _template_mean,
            num_valid,
            sample_sum,
            &mut residuals,
        );
    }

    residuals
}

/// Scalar fallback for residual computation
#[inline]
fn compute_residuals_scalar(
    sampled: &[f32; PATTERN52_SIZE],
    template: [f32; PATTERN52_SIZE],
    _template_mean: f32,
    num_valid: f32,
    sample_sum: f32,
    residuals: &mut na::SVector<f32, PATTERN52_SIZE>,
) {
    for i in 0..PATTERN52_SIZE {
        if sampled[i] >= 0.0 && template[i] >= 0.0 {
            let val = sampled[i];
            residuals[i] = num_valid * val / sample_sum - template[i];
        } else {
            residuals[i] = 0.0;
        }
    }
}

/// AVX2-accelerated residual computation (8 floats at once)
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2")]
unsafe fn compute_residuals_avx2(
    sampled: &[f32; PATTERN52_SIZE],
    template: [f32; PATTERN52_SIZE],
    _template_mean: f32,
    num_valid: f32,
    sample_sum: f32,
    residuals: &mut na::SVector<f32, PATTERN52_SIZE>,
) {
    let zero = _mm256_setzero_ps();
    let norm_factor = _mm256_set1_ps(num_valid / sample_sum);

    // Process 8 elements at a time
    for i in (0..PATTERN52_SIZE).step_by(8) {
        let remaining = PATTERN52_SIZE - i;
        if remaining < 8 {
            // Handle remainder with scalar code
            for j in i..PATTERN52_SIZE {
                if sampled[j] >= 0.0 && template[j] >= 0.0 {
                    let val = sampled[j];
                    residuals[j] = num_valid * val / sample_sum - template[j];
                } else {
                    residuals[j] = 0.0;
                }
            }
            break;
        }

        // Load values
        let sampled_vec = _mm256_loadu_ps(sampled.as_ptr().add(i));
        let template_vec = _mm256_loadu_ps(template.as_ptr().add(i));

        // Check both >= 0
        let sampled_valid = _mm256_cmp_ps(sampled_vec, zero, _CMP_GE_OQ);
        let template_valid = _mm256_cmp_ps(template_vec, zero, _CMP_GE_OQ);
        let both_valid = _mm256_and_ps(sampled_valid, template_valid);

        // Compute: norm_factor * sampled - template
        let normalized = _mm256_mul_ps(sampled_vec, norm_factor);
        let diff = _mm256_sub_ps(normalized, template_vec);

        // Mask invalid values to zero
        let masked_diff = _mm256_and_ps(diff, both_valid);

        // Store results
        _mm256_storeu_ps(residuals.as_mut_slice().as_mut_ptr().add(i), masked_diff);
    }
}

/// SSE4.1-accelerated residual computation (4 floats at once)
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "sse4.1")]
unsafe fn compute_residuals_sse(
    sampled: &[f32; PATTERN52_SIZE],
    template: [f32; PATTERN52_SIZE],
    _template_mean: f32,
    num_valid: f32,
    sample_sum: f32,
    residuals: &mut na::SVector<f32, PATTERN52_SIZE>,
) {
    let zero = _mm_setzero_ps();
    let norm_factor = _mm_set1_ps(num_valid / sample_sum);

    // Process 4 elements at a time
    for i in (0..PATTERN52_SIZE).step_by(4) {
        let remaining = PATTERN52_SIZE - i;
        if remaining < 4 {
            // Handle remainder
            for j in i..PATTERN52_SIZE {
                if sampled[j] >= 0.0 && template[j] >= 0.0 {
                    let val = sampled[j];
                    residuals[j] = num_valid * val / sample_sum - template[j];
                } else {
                    residuals[j] = 0.0;
                }
            }
            break;
        }

        let sampled_vec = _mm_loadu_ps(sampled.as_ptr().add(i));
        let template_vec = _mm_loadu_ps(template.as_ptr().add(i));

        let sampled_valid = _mm_cmpge_ps(sampled_vec, zero);
        let template_valid = _mm_cmpge_ps(template_vec, zero);
        let both_valid = _mm_and_ps(sampled_valid, template_valid);

        let normalized = _mm_mul_ps(sampled_vec, norm_factor);
        let diff = _mm_sub_ps(normalized, template_vec);
        let masked_diff = _mm_and_ps(diff, both_valid);

        _mm_storeu_ps(residuals.as_mut_slice().as_mut_ptr().add(i), masked_diff);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_residuals_scalar_vs_simd() {
        let mut sampled = [0.5f32; PATTERN52_SIZE];
        let template = [0.3f32; PATTERN52_SIZE];
        sampled[0] = -1.0; // Invalid value

        let num_valid = 51.0;
        let sample_sum = 25.5;

        let mut scalar_result = na::SVector::<f32, PATTERN52_SIZE>::zeros();
        compute_residuals_scalar(
            &sampled,
            template,
            0.3,
            num_valid,
            sample_sum,
            &mut scalar_result,
        );

        let simd_result = compute_residuals_simd(&sampled, template, 0.3, num_valid, sample_sum);

        for i in 0..PATTERN52_SIZE {
            assert!(
                (scalar_result[i] - simd_result[i]).abs() < 1e-4,
                "Mismatch at index {}: scalar={}, simd={}",
                i,
                scalar_result[i],
                simd_result[i]
            );
        }
    }
}
