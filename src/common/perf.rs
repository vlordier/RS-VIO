//! Performance optimization utilities to reduce code bloat and improve runtime.
//!
//! This module provides optimized alternatives to common patterns that can
//! cause excessive monomorphization or poor codegen.

use crate::types::Float;

/// Optimized vector operations that avoid iterator overhead in hot paths.
pub mod vec_ops {
    use super::*;

    /// Compute dot product without iterator overhead.
    #[inline(always)]
    pub fn dot_product(a: &[Float], b: &[Float]) -> Float {
        debug_assert_eq!(a.len(), b.len());
        let mut sum = 0.0;
        for i in 0..a.len() {
            sum += a[i] * b[i];
        }
        sum
    }

    /// Compute squared L2 norm without iterator overhead.
    #[inline(always)]
    pub fn norm_squared(v: &[Float]) -> Float {
        let mut sum = 0.0;
        for &x in v {
            sum += x * x;
        }
        sum
    }

    /// Find maximum value and its index without iterator overhead.
    #[inline]
    pub fn argmax(values: &[Float]) -> (usize, Float) {
        debug_assert!(!values.is_empty());
        let mut max_idx = 0;
        let mut max_val = values[0];
        for (i, &val) in values.iter().enumerate().skip(1) {
            if val > max_val {
                max_val = val;
                max_idx = i;
            }
        }
        (max_idx, max_val)
    }

    /// Find minimum value and its index without iterator overhead.
    #[inline]
    pub fn argmin(values: &[Float]) -> (usize, Float) {
        debug_assert!(!values.is_empty());
        let mut min_idx = 0;
        let mut min_val = values[0];
        for (i, &val) in values.iter().enumerate().skip(1) {
            if val < min_val {
                min_val = val;
                min_idx = i;
            }
        }
        (min_idx, min_val)
    }

    /// Sum vector elements without iterator allocation.
    #[inline(always)]
    pub fn sum(values: &[Float]) -> Float {
        let mut total = 0.0;
        for &val in values {
            total += val;
        }
        total
    }

    /// Compute mean without iterator overhead.
    #[inline]
    pub fn mean(values: &[Float]) -> Float {
        if values.is_empty() {
            return 0.0;
        }
        sum(values) / values.len() as Float
    }

    /// Element-wise addition: result[i] = a[i] + b[i].
    #[inline]
    pub fn add_into(a: &[Float], b: &[Float], result: &mut [Float]) {
        debug_assert_eq!(a.len(), b.len());
        debug_assert_eq!(a.len(), result.len());
        for i in 0..a.len() {
            result[i] = a[i] + b[i];
        }
    }

    /// Element-wise scaling: result[i] = a[i] * scale.
    #[inline]
    pub fn scale_into(a: &[Float], scale: Float, result: &mut [Float]) {
        debug_assert_eq!(a.len(), result.len());
        for i in 0..a.len() {
            result[i] = a[i] * scale;
        }
    }

    /// Count elements satisfying a predicate without closure overhead.
    #[inline]
    pub fn count_nonzero(values: &[Float]) -> usize {
        let mut count = 0;
        for &val in values {
            if val != 0.0 {
                count += 1;
            }
        }
        count
    }

    /// Count values above threshold.
    #[inline]
    pub fn count_above(values: &[Float], threshold: Float) -> usize {
        let mut count = 0;
        for &val in values {
            if val > threshold {
                count += 1;
            }
        }
        count
    }
}

/// Optimized collection operations.
pub mod collection_ops {
    /// Filter and collect without intermediate allocations.
    #[inline]
    pub fn filter_collect<T, F>(items: &[T], predicate: F) -> Vec<T>
    where
        T: Clone,
        F: Fn(&T) -> bool,
    {
        let mut result = Vec::with_capacity(items.len() / 2); // Heuristic
        for item in items {
            if predicate(item) {
                result.push(item.clone());
            }
        }
        result.shrink_to_fit();
        result
    }

    /// Map and collect with explicit loop to avoid iterator overhead.
    #[inline]
    pub fn map_collect<T, U, F>(items: &[T], f: F) -> Vec<U>
    where
        F: Fn(&T) -> U,
    {
        let mut result = Vec::with_capacity(items.len());
        for item in items {
            result.push(f(item));
        }
        result
    }

    /// Partition elements into two vectors based on predicate.
    #[inline]
    pub fn partition<T, F>(items: Vec<T>, predicate: F) -> (Vec<T>, Vec<T>)
    where
        F: Fn(&T) -> bool,
    {
        let mut true_items = Vec::with_capacity(items.len() / 2);
        let mut false_items = Vec::with_capacity(items.len() / 2);

        for item in items {
            if predicate(&item) {
                true_items.push(item);
            } else {
                false_items.push(item);
            }
        }

        true_items.shrink_to_fit();
        false_items.shrink_to_fit();
        (true_items, false_items)
    }
}

/// Inline optimization hints (disabled in this codebase due to unsafe-code lint).
pub mod hints {
    // Note: These optimizations are disabled because the codebase uses #![deny(unsafe_code)].
    // They would require unsafe blocks for std::hint::unreachable_unchecked().

    /// Branch prediction hint (currently no-op).
    #[inline(always)]
    pub fn likely(b: bool) -> bool {
        b
    }

    /// Branch prediction hint (currently no-op).
    #[inline(always)]
    pub fn unlikely(b: bool) -> bool {
        b
    }
}

#[cfg(test)]
mod tests {
    use super::collection_ops;
    use super::vec_ops;

    #[test]
    fn test_dot_product() {
        let a = [1.0, 2.0, 3.0];
        let b = [4.0, 5.0, 6.0];
        assert_eq!(vec_ops::dot_product(&a, &b), 32.0);
    }

    #[test]
    fn test_norm_squared() {
        let v = [3.0, 4.0];
        assert_eq!(vec_ops::norm_squared(&v), 25.0);
    }

    #[test]
    fn test_argmax() {
        let values = [1.0, 5.0, 3.0, 2.0];
        let (idx, val) = vec_ops::argmax(&values);
        assert_eq!(idx, 1);
        assert_eq!(val, 5.0);
    }

    #[test]
    fn test_argmin() {
        let values = [5.0, 1.0, 3.0, 2.0];
        let (idx, val) = vec_ops::argmin(&values);
        assert_eq!(idx, 1);
        assert_eq!(val, 1.0);
    }

    #[test]
    fn test_sum() {
        let values = [1.0, 2.0, 3.0, 4.0];
        assert_eq!(vec_ops::sum(&values), 10.0);
    }

    #[test]
    fn test_mean() {
        let values = [1.0, 2.0, 3.0, 4.0];
        assert_eq!(vec_ops::mean(&values), 2.5);
    }

    #[test]
    fn test_count_above() {
        let values = [1.0, 5.0, 3.0, 7.0, 2.0];
        assert_eq!(vec_ops::count_above(&values, 3.0), 2);
    }

    #[test]
    fn test_filter_collect() {
        let items = [1, 2, 3, 4, 5];
        let evens = collection_ops::filter_collect(&items, |&x| x % 2 == 0);
        assert_eq!(evens, vec![2, 4]);
    }

    #[test]
    fn test_map_collect() {
        let items = [1, 2, 3];
        let doubled = collection_ops::map_collect(&items, |&x| x * 2);
        assert_eq!(doubled, vec![2, 4, 6]);
    }

    #[test]
    fn test_partition() {
        let items = vec![1, 2, 3, 4, 5];
        let (evens, odds) = collection_ops::partition(items, |&x| x % 2 == 0);
        assert_eq!(evens, vec![2, 4]);
        assert_eq!(odds, vec![1, 3, 5]);
    }
}
