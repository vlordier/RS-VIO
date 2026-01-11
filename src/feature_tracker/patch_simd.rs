/// SIMD-optimized patch matching for real-time performance
///
/// This module provides vectorized implementations of patch-based tracking operations
/// using platform-specific SIMD intrinsics when available.

use nalgebra as na;

#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::*;

use super::patch::{Pattern52, PATTERN52_SIZE};

/// Compute patch residuals using SIMD operations
///
/// This vectorizes the residual computation across the 52 pattern points,
/// processing 4-8 points simultaneously depending on SIMD width.
#[inline]
pub fn compute_residuals_simd(
    pattern: &Pattern52,
    current_data: &[f32; PATTERN52_SIZE],
) -> na::SVector<f32, PATTERN52_SIZE> {
    let mut residuals = na::SVector::<f32, PATTERN52_SIZE>::zeros();

    #[cfg(target_arch = "x86_64")]
    {
        if is_x86_feature_detected!("avx2") {
            unsafe { compute_residuals_avx2(pattern, current_data, &mut residuals) }
        } else if is_x86_feature_detected!("sse4.1") {
            unsafe { compute_residuals_sse(pattern, current_data, &mut residuals) }
        } else {
            compute_residuals_scalar(pattern, current_data, &mut residuals);
        }
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        compute_residuals_scalar(pattern, current_data, &mut residuals);
    }

    residuals
}

/// Scalar fallback for residual computation
#[inline]
fn compute_residuals_scalar(
    pattern: &Pattern52,
    current_data: &[f32; PATTERN52_SIZE],
    residuals: &mut na::SVector<f32, PATTERN52_SIZE>,
) {
    for i in 0..PATTERN52_SIZE {
        if pattern.data[i] >= 0.0 && current_data[i] >= 0.0 {
            residuals[i] = current_data[i] - pattern.data[i];
        }
    }
}

/// AVX2-accelerated residual computation (8 floats at once)
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2")]
unsafe fn compute_residuals_avx2(
    pattern: &Pattern52,
    current_data: &[f32; PATTERN52_SIZE],
    residuals: &mut na::SVector<f32, PATTERN52_SIZE>,
) {
    let zero = _mm256_setzero_ps();
    
    // Process 8 elements at a time
    for i in (0..PATTERN52_SIZE).step_by(8) {
        let remaining = PATTERN52_SIZE - i;
        if remaining < 8 {
            // Handle remainder with scalar code
            for j in i..PATTERN52_SIZE {
                if pattern.data[j] >= 0.0 && current_data[j] >= 0.0 {
                    residuals[j] = current_data[j] - pattern.data[j];
                }
            }
            break;
        }

        // Load 8 pattern values
        let pattern_vec = _mm256_loadu_ps(pattern.data.as_ptr().add(i));
        // Load 8 current values
        let current_vec = _mm256_loadu_ps(current_data.as_ptr().add(i));

        // Check both >= 0
        let pattern_valid = _mm256_cmp_ps(pattern_vec, zero, _CMP_GE_OQ);
        let current_valid = _mm256_cmp_ps(current_vec, zero, _CMP_GE_OQ);
        let both_valid = _mm256_and_ps(pattern_valid, current_valid);

        // Compute residual = current - pattern
        let diff = _mm256_sub_ps(current_vec, pattern_vec);

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
    pattern: &Pattern52,
    current_data: &[f32; PATTERN52_SIZE],
    residuals: &mut na::SVector<f32, PATTERN52_SIZE>,
) {
    let zero = _mm_setzero_ps();
    
    // Process 4 elements at a time
    for i in (0..PATTERN52_SIZE).step_by(4) {
        let remaining = PATTERN52_SIZE - i;
        if remaining < 4 {
            // Handle remainder
            for j in i..PATTERN52_SIZE {
                if pattern.data[j] >= 0.0 && current_data[j] >= 0.0 {
                    residuals[j] = current_data[j] - pattern.data[j];
                }
            }
            break;
        }

        let pattern_vec = _mm_loadu_ps(pattern.data.as_ptr().add(i));
        let current_vec = _mm_loadu_ps(current_data.as_ptr().add(i));

        let pattern_valid = _mm_cmpge_ps(pattern_vec, zero);
        let current_valid = _mm_cmpge_ps(current_vec, zero);
        let both_valid = _mm_and_ps(pattern_valid, current_valid);

        let diff = _mm_sub_ps(current_vec, pattern_vec);
        let masked_diff = _mm_and_ps(diff, both_valid);

        _mm_storeu_ps(residuals.as_mut_slice().as_mut_ptr().add(i), masked_diff);
    }
}

/// Vectorized mean and standard deviation computation
#[inline]
pub fn compute_stats_simd(data: &[f32; PATTERN52_SIZE]) -> (f32, f32) {
    #[cfg(target_arch = "x86_64")]
    {
        if is_x86_feature_detected!("avx2") {
            unsafe { return compute_stats_avx2(data); }
        }
    }

    // Scalar fallback
    let mut sum = 0.0f32;
    let mut count = 0;
    for &val in data.iter() {
        if val >= 0.0 {
            sum += val;
            count += 1;
        }
    }
    let mean = sum / count as f32;
    
    let mut var_sum = 0.0f32;
    for &val in data.iter() {
        if val >= 0.0 {
            let diff = val - mean;
            var_sum += diff * diff;
        }
    }
    let std_dev = (var_sum / count as f32).sqrt();
    
    (mean, std_dev)
}

#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2")]
unsafe fn compute_stats_avx2(data: &[f32; PATTERN52_SIZE]) -> (f32, f32) {
    let zero = _mm256_setzero_ps();
    let mut sum_vec = _mm256_setzero_ps();
    let mut count_vec = _mm256_setzero_ps();
    let one = _mm256_set1_ps(1.0);

    // First pass: compute sum and count
    for i in (0..(PATTERN52_SIZE & !7)).step_by(8) {
        let val_vec = _mm256_loadu_ps(data.as_ptr().add(i));
        let valid_mask = _mm256_cmp_ps(val_vec, zero, _CMP_GE_OQ);
        
        let masked_val = _mm256_and_ps(val_vec, valid_mask);
        sum_vec = _mm256_add_ps(sum_vec, masked_val);
        
        let count_inc = _mm256_and_ps(one, valid_mask);
        count_vec = _mm256_add_ps(count_vec, count_inc);
    }

    // Horizontal sum
    let sum = hsum_avx(sum_vec);
    let count = hsum_avx(count_vec);
    
    let mean = sum / count;
    let mean_vec = _mm256_set1_ps(mean);
    
    // Second pass: compute variance
    let mut var_vec = _mm256_setzero_ps();
    for i in (0..(PATTERN52_SIZE & !7)).step_by(8) {
        let val_vec = _mm256_loadu_ps(data.as_ptr().add(i));
        let valid_mask = _mm256_cmp_ps(val_vec, zero, _CMP_GE_OQ);
        
        let diff = _mm256_sub_ps(val_vec, mean_vec);
        let sq_diff = _mm256_mul_ps(diff, diff);
        let masked_sq = _mm256_and_ps(sq_diff, valid_mask);
        var_vec = _mm256_add_ps(var_vec, masked_sq);
    }
    
    let var_sum = hsum_avx(var_vec);
    let std_dev = (var_sum / count).sqrt();
    
    (mean, std_dev)
}

#[cfg(target_arch = "x86_64")]
#[inline]
unsafe fn hsum_avx(v: __m256) -> f32 {
    let hi = _mm256_extractf128_ps(v, 1);
    let lo = _mm256_castps256_ps128(v);
    let sum128 = _mm_add_ps(hi, lo);
    
    let shuf = _mm_movehdup_ps(sum128);
    let sums = _mm_add_ps(sum128, shuf);
    let shuf2 = _mm_movehl_ps(shuf, sums);
    let result = _mm_add_ss(sums, shuf2);
    
    _mm_cvtss_f32(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_residuals_match() {
        let pattern = Pattern52::default();
        let current_data = [1.0f32; PATTERN52_SIZE];
        
        let scalar_result = {
            let mut r = na::SVector::<f32, PATTERN52_SIZE>::zeros();
            compute_residuals_scalar(&pattern, &current_data, &mut r);
            r
        };
        
        let simd_result = compute_residuals_simd(&pattern, &current_data);
        
        for i in 0..PATTERN52_SIZE {
            assert!((scalar_result[i] - simd_result[i]).abs() < 1e-5,
                "Mismatch at index {}: scalar={}, simd={}", i, scalar_result[i], simd_result[i]);
        }
    }
}
