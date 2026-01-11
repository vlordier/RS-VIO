//! Parametrized tests for common test scenarios across modules
//!
//! These tests use macro-based parametrization to efficiently test multiple scenarios.

#[cfg(test)]
mod parametrized_tests {
    use rs_vio::types::*;

    /// Macro for testing matrix operations with different sizes
    macro_rules! test_matrix_identity {
        ($name:ident, $matrix_type:ty, $size:expr) => {
            #[test]
            fn $name() {
                let m = <$matrix_type>::identity();
                for i in 0..$size {
                    for j in 0..$size {
                        if i == j {
                            assert!(
                                (m[(i, j)] - 1.0).abs() < 1e-10,
                                "Diagonal element [{},{}] should be 1.0",
                                i,
                                j
                            );
                        } else {
                            assert!(
                                m[(i, j)].abs() < 1e-10,
                                "Off-diagonal element [{},{}] should be 0.0",
                                i,
                                j
                            );
                        }
                    }
                }
            }
        };
    }

    /// Macro for testing vector operations
    macro_rules! test_vector_operations {
        ($name:ident, $vector_type:ty, $dim:expr, [$($val:expr),*]) => {
            #[test]
            fn $name() {
                let v = <$vector_type>::new($($val),*);
                assert_eq!(v.len(), $dim);

                // Test norm is non-negative
                assert!(v.norm() >= 0.0);

                // Test zero vector
                let zero = <$vector_type>::zeros();
                assert!((zero.norm() - 0.0).abs() < 1e-10);
            }
        };
    }

    /// Macro for testing mathematical properties
    macro_rules! test_mathematical_property {
        ($name:ident, $description:expr, $property:expr) => {
            #[test]
            fn $name() {
                assert!($property, "Mathematical property failed: {}", $description);
            }
        };
    }

    // Test matrix identity with various sizes
    test_matrix_identity!(test_matrix2x2_identity, Matrix2x2, 2);
    test_matrix_identity!(test_matrix3x3_identity, Matrix3x3, 3);
    test_matrix_identity!(test_matrix4x4_identity, Matrix4x4, 4);

    // Test vector operations
    test_vector_operations!(test_vector2_ops, Vector2, 2, [1.0, 2.0]);
    test_vector_operations!(test_vector3_ops, Vector3, 3, [1.0, 2.0, 3.0]);

    // Test mathematical properties
    test_mathematical_property!(
        test_vector_commutativity,
        "Vector addition is commutative",
        {
            let v1 = Vector3::new(1.0, 2.0, 3.0);
            let v2 = Vector3::new(4.0, 5.0, 6.0);
            (v1 + v2 - (v2 + v1)).norm() < 1e-10
        }
    );

    test_mathematical_property!(
        test_vector_associativity,
        "Vector addition is associative",
        {
            let v1 = Vector3::new(1.0, 0.0, 0.0);
            let v2 = Vector3::new(0.0, 1.0, 0.0);
            let v3 = Vector3::new(0.0, 0.0, 1.0);
            ((v1 + v2) + v3 - (v1 + (v2 + v3))).norm() < 1e-10
        }
    );

    test_mathematical_property!(
        test_vector_distributivity,
        "Scalar multiplication distributes over addition",
        {
            let v1 = Vector3::new(1.0, 2.0, 3.0);
            let v2 = Vector3::new(4.0, 5.0, 6.0);
            let scalar = 2.5;
            (scalar * (v1 + v2) - (scalar * v1 + scalar * v2)).norm() < 1e-10
        }
    );

    test_mathematical_property!(
        test_dot_product_commutativity,
        "Dot product is commutative",
        {
            let v1 = Vector3::new(1.0, 2.0, 3.0);
            let v2 = Vector3::new(4.0, 5.0, 6.0);
            (v1.dot(&v2) - v2.dot(&v1)).abs() < 1e-10
        }
    );

    test_mathematical_property!(
        test_cross_product_anticommutativity,
        "Cross product is anti-commutative",
        {
            let v1 = Vector3::new(1.0, 2.0, 3.0);
            let v2 = Vector3::new(4.0, 5.0, 6.0);
            (v1.cross(&v2) + v2.cross(&v1)).norm() < 1e-10
        }
    );

    test_mathematical_property!(
        test_norm_zero_iff_zero_vector,
        "A vector has zero norm iff it is a zero vector",
        {
            let zero = Vector3::zeros();
            zero.norm() < 1e-10
        }
    );

    test_mathematical_property!(
        test_triangle_inequality,
        "Triangle inequality holds for vector norms",
        {
            let v1 = Vector3::new(1.0, 2.0, 3.0);
            let v2 = Vector3::new(4.0, 5.0, 6.0);
            let sum_norms = v1.norm() + v2.norm();
            let norm_sum = (v1 + v2).norm();
            norm_sum <= sum_norms + 1e-10 // Allow floating point error
        }
    );

    test_mathematical_property!(
        test_matrix_determinant_product,
        "Determinant of product equals product of determinants",
        {
            let m1 = Matrix3x3::identity();
            let m2 = Matrix3x3::from_element(2.0);
            let _product = m1 * m2;
            ((m1 * m2).determinant() - (m1.determinant() * m2.determinant())).abs() < 1e-10
        }
    );

    test_mathematical_property!(
        test_matrix_inverse_identity,
        "Matrix multiplied by its inverse gives identity",
        {
            let m = Matrix3x3::new(1.0, 2.0, 3.0, 0.0, 1.0, 4.0, 5.0, 6.0, 0.0);
            if let Some(inv) = m.try_inverse() {
                let product = m * inv;
                let id = Matrix3x3::identity();
                let diff = (product - id).norm();
                diff < 1e-8
            } else {
                false // Matrix wasn't invertible
            }
        }
    );

    test_mathematical_property!(
        test_transpose_twice_identity,
        "Transposing twice gives the original matrix",
        {
            let m = Matrix3x3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
            (m.transpose().transpose() - m).norm() < 1e-10
        }
    );

    test_mathematical_property!(
        test_transpose_changes_off_diagonal,
        "Transpose properly exchanges off-diagonal elements",
        {
            let m = Matrix3x3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
            let t = m.transpose();
            (t[(0, 1)] - 4.0).abs() < 1e-10 && (t[(1, 0)] - 2.0).abs() < 1e-10
        }
    );

    /// Test suite for floating-point edge cases
    #[test]
    fn test_very_small_vector() {
        let v = Vector3::new(1e-100, 1e-100, 1e-100);
        assert!(v.norm().is_finite());
    }

    #[test]
    fn test_very_large_vector() {
        let v = Vector3::new(1e100, 1e100, 1e100);
        assert!(v.norm().is_finite());
    }

    #[test]
    fn test_mixed_magnitude_vector() {
        let v = Vector3::new(1e-50, 1.0, 1e50);
        assert!(v.norm().is_finite());
    }

    #[test]
    fn test_vector_with_negative_components() {
        let v = Vector3::new(-1.0, -2.0, -3.0);
        let norm_pos = Vector3::new(1.0, 2.0, 3.0).norm();
        assert!((v.norm() - norm_pos).abs() < 1e-10);
    }

    #[test]
    fn test_multiple_matrix_multiplications() {
        let m1 = Matrix3x3::identity();
        let m2 = Matrix3x3::identity() * 2.0;
        let m3 = Matrix3x3::identity() * 3.0;

        let result = m1 * m2 * m3;
        let expected = Matrix3x3::identity() * 6.0;

        assert!((result - expected).norm() < 1e-10);
    }

    #[test]
    fn test_vector_scalar_operations_chain() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        let ones = Vector3::new(1.0, 1.0, 1.0);
        let result = (v * 2.0 + ones) * 0.5;

        assert!((result[0] - 1.5).abs() < 1e-10);
        assert!((result[1] - 2.5).abs() < 1e-10);
        assert!((result[2] - 3.5).abs() < 1e-10);
    }

    #[test]
    fn test_matrix_scalar_multiplication() {
        let m = Matrix2x2::identity();
        let scaled = m * 5.0;

        assert_eq!(scaled[(0, 0)], 5.0);
        assert_eq!(scaled[(1, 1)], 5.0);
        assert_eq!(scaled[(0, 1)], 0.0);
    }

    #[test]
    fn test_batch_matrix_operations() {
        for scale in &[0.1, 0.5, 1.0, 2.0, 10.0] {
            let m = Matrix2x2::identity() * *scale;
            assert!((m.determinant() - (scale * scale)).abs() < 1e-10);
        }
    }

    #[test]
    fn test_batch_vector_normalizations() {
        let test_vectors = vec![
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(3.0, 4.0, 0.0),
        ];

        for v in test_vectors {
            let normalized = v.normalize();
            assert!((normalized.norm() - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_orthogonality_preservation() {
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);

        assert!((v1.dot(&v2)).abs() < 1e-10); // Orthogonal vectors

        let scaled_v1 = v1 * 5.0;
        assert!((scaled_v1.dot(&v2)).abs() < 1e-10); // Still orthogonal after scaling
    }
}
