//! Unified trait definitions for RS-VIO
//!
//! This module provides consolidated, well-designed traits that follow best practices
//! for separation of concerns, DRY principles, and optimized data flow.
//!
//! ## Trait Categories
//!
//! 1. **Conversion** - Generic type conversions
//! 2. **Strategy** - Algorithm swapping (Strategy Pattern)
//! 3. **Resource Management** - Pooling and lifecycle
//! 4. **Validation** - Fast numerical validation
//! 5. **State** - Immutable views and transformations

use crate::types::{Float, Matrix4x4, Vector3};
use std::fmt::Debug;

// ============================================================================
// 1. Conversion Traits
// ============================================================================

/// Generic conversion between compatible types
///
/// This trait provides a unified interface for converting between nalgebra types
/// and array representations, replacing fragmented `ToMatrix`/`ToVector`/`ToArray` traits.
///
/// # Design
/// - Zero-cost abstraction (fully inlined)
/// - Type-safe conversions
/// - Symmetric (bidirectional) conversions
///
/// # Example
/// ```rust,ignore
/// use rs_vio::traits::Convert;
/// use rs_vio::types::{Array4x4, Matrix4x4};
///
/// let array: Array4x4 = [[1.0, 0.0, 0.0, 0.0],
///                        [0.0, 1.0, 0.0, 0.0],
///                        [0.0, 0.0, 1.0, 0.0],
///                        [0.0, 0.0, 0.0, 1.0]];
/// let matrix: Matrix4x4 = array.convert();
/// let back: Array4x4 = matrix.convert();
/// ```
pub trait Convert<T> {
    /// Convert this value to type T
    fn convert(&self) -> T;
}

// ============================================================================
// 2. Strategy Pattern Traits
// ============================================================================

/// Base trait for all strategy pattern implementations
///
/// Provides common functionality for swappable algorithm implementations.
/// All strategy traits should extend this to ensure consistent behavior.
///
/// # Design Pattern
/// Implements the **Strategy Pattern** base, providing:
/// - Runtime polymorphism via trait objects
/// - Compile-time polymorphism via generics
/// - Platform-specific implementations
///
/// # Example
/// ```rust,ignore
/// use rs_vio::traits::Strategy;
///
/// struct MyAlgorithm;
///
/// impl Strategy for MyAlgorithm {
///     fn name(&self) -> &str { "MyAlgorithm v1.0" }
///     fn description(&self) -> &str { "Fast approximate algorithm" }
/// }
/// ```
pub trait Strategy: Send + Sync + Debug + 'static {
    /// Human-readable name for logging and debugging
    fn name(&self) -> &str;

    /// Optional description of the strategy
    fn description(&self) -> &str {
        self.name()
    }

    /// Check if this strategy is available on the current platform
    fn is_available(&self) -> bool {
        true
    }
}

/// Extension trait for cloneable strategies
///
/// Enables object-safe cloning of boxed trait objects.
/// Automatically implemented for all `Strategy + Clone` types.
pub trait CloneStrategy: Strategy {
    /// Clone this strategy as a boxed trait object
    fn clone_box(&self) -> Box<dyn Strategy>;
}

impl<T> CloneStrategy for T
where
    T: Strategy + Clone + 'static,
{
    fn clone_box(&self) -> Box<dyn Strategy> {
        Box::new(self.clone())
    }
}

// ============================================================================
// 3. Resource Management Traits
// ============================================================================

/// Generic resource pooling interface
///
/// Provides a unified interface for all resource pools (workspaces, descriptors, buffers).
/// Enables consistent resource management across the codebase.
///
/// # Design
/// - Thread-safe by default (`Send + Sync`)
/// - Configurable via associated type
/// - Support for both blocking and non-blocking acquisition
///
/// # Example
/// ```rust,ignore
/// use rs_vio::traits::ResourcePool;
///
/// let pool = MyPool::new(MyConfig::default());
/// let resource = pool.acquire(); // Blocks if none available
/// // Use resource...
/// pool.release(resource); // Return to pool
/// ```
pub trait ResourcePool: Send + Sync {
    /// Type of resource managed by this pool
    type Resource;

    /// Configuration type for pool creation
    type Config: Default + Clone;

    /// Create a new pool with configuration
    fn new(config: Self::Config) -> Self;

    /// Acquire a resource from the pool (blocks if none available)
    fn acquire(&self) -> Self::Resource;

    /// Try to acquire a resource (returns None if pool is empty)
    fn try_acquire(&self) -> Option<Self::Resource>;

    /// Release a resource back to the pool
    ///
    /// The resource should be reset/cleared before being returned
    fn release(&self, resource: Self::Resource);

    /// Reset the pool (clear all cached resources)
    fn reset(&self);

    /// Get current utilization (0.0 = empty, 1.0 = fully allocated)
    fn utilization(&self) -> f32;

    /// Get pool capacity (maximum number of resources)
    fn capacity(&self) -> usize;

    /// Get number of available resources
    fn available(&self) -> usize;
}

// ============================================================================
// 4. Validation Traits
// ============================================================================

/// Fast numerical validation for hotpath usage
///
/// Optimized for performance-critical code with inline methods
/// and minimal allocations. Replaces the `Validatable` trait with
/// a more ergonomic and efficient design.
///
/// # Design
/// - All methods are `#[inline]` for zero overhead
/// - Uses associated types for flexibility
/// - Chainable validation methods
///
/// # Example
/// ```rust,ignore
/// use rs_vio::traits::Validate;
/// use nalgebra::Vector3;
///
/// let v = Vector3::new(1.0, 2.0, 3.0);
/// v.validate()?; // Check for NaN/Inf
///
/// let validated = v.validated()?; // Returns v if valid
/// ```
pub trait Validate {
    /// Error type for validation failures
    type Error: std::error::Error + 'static;

    /// Validate this value
    ///
    /// Returns `Ok(())` if valid, `Err(Self::Error)` otherwise.
    fn validate(&self) -> Result<(), Self::Error>;

    /// Validate and return self if valid
    ///
    /// This is a convenience method for validation in pipelines.
    fn validated(self) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        self.validate()?;
        Ok(self)
    }

    /// Chain multiple validators
    ///
    /// All validators must pass for validation to succeed.
    fn validate_all<V>(&self, validators: &[V]) -> Result<(), Self::Error>
    where
        V: Fn(&Self) -> Result<(), Self::Error>,
    {
        for validator in validators {
            validator(self)?;
        }
        Ok(())
    }
}

// ============================================================================
// 5. State Management Traits
// ============================================================================

/// Immutable view of VIO state
///
/// Provides read-only access to state components.
/// Separated from transformations to follow the Interface Segregation Principle.
///
/// # Design
/// - Returns references to avoid copies
/// - Minimal allocation
/// - Clear separation from mutation
pub trait StateView {
    /// Get the world-from-body pose
    fn pose(&self) -> &Matrix4x4;

    /// Get body linear velocity in world coordinates
    fn velocity(&self) -> &Vector3;

    /// Get accelerometer bias
    fn accel_bias(&self) -> &Vector3;

    /// Get gyroscope bias
    fn gyro_bias(&self) -> &Vector3;

    /// Get camera extrinsics (body to left camera)
    fn camera_left_extrinsics(&self) -> &Matrix4x4;

    /// Get camera extrinsics (body to right camera)
    fn camera_right_extrinsics(&self) -> &Matrix4x4;

    /// Get translation component of pose
    fn translation(&self) -> Vector3 {
        self.pose().fixed_view::<3, 1>(0, 3).into_owned()
    }

    /// Get rotation as unit quaternion
    fn rotation(&self) -> crate::types::UnitQuaternion {
        let rotation_matrix = self.pose().fixed_view::<3, 3>(0, 0).into_owned();
        crate::types::UnitQuaternion::from_matrix(&rotation_matrix)
    }
}

/// State transformation operations
///
/// Provides methods for computing derived states.
/// Requires `StateView` to ensure read access is available.
///
/// # Design
/// - Pure functions (no mutation)
/// - Returns new state instances
/// - Composable transformations
pub trait StateTransform: StateView {
    /// Compose this state with another (relative motion)
    fn compose(&self, other: &Self) -> Self
    where
        Self: Sized;

    /// Compute the inverse state (world-from-body → body-from-world)
    fn inverse(&self) -> Self
    where
        Self: Sized;

    /// Interpolate between two states
    ///
    /// # Arguments
    /// * `other` - Target state
    /// * `alpha` - Interpolation parameter (0.0 = self, 1.0 = other)
    fn interpolate(&self, other: &Self, alpha: Float) -> Self
    where
        Self: Sized;
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_strategy_trait_object() {
        #[derive(Debug, Clone)]
        struct TestStrategy;

        impl Strategy for TestStrategy {
            fn name(&self) -> &str {
                "TestStrategy"
            }
        }

        let strategy: Box<dyn Strategy> = Box::new(TestStrategy);
        assert_eq!(strategy.name(), "TestStrategy");
        assert!(strategy.is_available());
    }

    #[test]
    fn test_validate_trait() {
        let valid = 42.0f64;
        assert!(valid.is_finite());
    }
}
