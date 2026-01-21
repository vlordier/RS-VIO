//! Common configuration traits and utilities for the RS-VIO system.
//!
//! This module provides shared traits and helpers for configuration types
//! across the codebase, promoting consistency and reducing duplication.

/// Trait for configuration types that can be validated.
///
/// Implementing this trait ensures that configuration values are checked
/// for validity before use, preventing runtime errors from invalid settings.
///
/// # Example
///
/// ```
/// use rs_vio::common::config::Validatable;
///
/// #[derive(Debug, Clone)]
/// struct MyConfig {
///     threshold: f64,
///     max_iterations: usize,
/// }
///
/// impl Validatable for MyConfig {
///     type Error = String;
///
///     fn validate(&self) -> Result<(), Self::Error> {
///         if self.threshold <= 0.0 || self.threshold >= 1.0 {
///             return Err("threshold must be in range (0, 1)".to_string());
///         }
///         if self.max_iterations == 0 {
///             return Err("max_iterations must be positive".to_string());
///         }
///         Ok(())
///     }
/// }
/// ```
pub trait Validatable {
    /// The error type returned when validation fails.
    type Error;

    /// Validates the configuration.
    ///
    /// Returns `Ok(())` if the configuration is valid, or an error describing
    /// what is invalid.
    fn validate(&self) -> Result<(), Self::Error>;

    /// Validates and returns self, useful for chaining.
    fn validated(self) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        self.validate()?;
        Ok(self)
    }
}

/// Trait for configuration types that support clamping values to valid ranges.
///
/// This is useful for configurations that should tolerate out-of-range values
/// by automatically adjusting them rather than failing validation.
pub trait Clampable {
    /// Clamps all configuration values to their valid ranges.
    ///
    /// This modifies the configuration in-place and logs warnings when
    /// values are adjusted.
    fn clamp(&mut self);

    /// Returns a clamped copy of the configuration.
    fn clamped(mut self) -> Self
    where
        Self: Sized,
    {
        self.clamp();
        self
    }
}

/// Trait for configurations that can be merged/overridden.
///
/// This is useful for layered configuration systems where defaults can be
/// overridden by user settings.
pub trait Mergeable {
    /// Merges another configuration into this one, overriding values.
    ///
    /// Typically, `other` takes precedence over `self` for non-None values.
    fn merge(&mut self, other: &Self);

    /// Returns a new configuration merging self with other.
    fn merged(&self, other: &Self) -> Self
    where
        Self: Clone,
    {
        let mut result = self.clone();
        result.merge(other);
        result
    }
}

/// Helper function to clamp a value to a range and log if clamped.
///
/// # Arguments
///
/// * `value` - The value to clamp
/// * `min` - Minimum allowed value
/// * `max` - Maximum allowed value
/// * `name` - Name of the parameter (for logging)
///
/// # Returns
///
/// The clamped value
pub fn clamp_with_warning<T: PartialOrd + Copy + std::fmt::Display>(
    value: T,
    min: T,
    max: T,
    name: &str,
) -> T {
    if value < min {
        log::warn!(
            "{} value {} below minimum {}, clamping to {}",
            name,
            value,
            min,
            min
        );
        min
    } else if value > max {
        log::warn!(
            "{} value {} above maximum {}, clamping to {}",
            name,
            value,
            max,
            max
        );
        max
    } else {
        value
    }
}

/// Builder pattern helper for configurations.
///
/// Provides a common pattern for creating configurations with method chaining.
///
/// # Example
///
/// ```ignore
/// let config = MyConfig::builder()
///     .threshold(0.5)
///     .max_iterations(100)
///     .build()?;
/// ```
pub trait ConfigBuilder: Sized {
    /// The configuration type this builder creates.
    type Config;

    /// Builds the configuration, validating if the Config implements Validatable.
    fn build(self) -> Result<Self::Config, Box<dyn std::error::Error>>;
}

/// Common configuration defaults for numeric ranges.
pub mod defaults {
    /// Default small epsilon value for floating point comparisons.
    pub const EPSILON: f64 = 1e-10;

    /// Default small positive value for regularization.
    pub const SMALL_REGULARIZATION: f64 = 1e-6;

    /// Default damping factor for optimization.
    pub const DEFAULT_DAMPING: f64 = 1e-5;

    /// Default maximum iterations for iterative algorithms.
    pub const DEFAULT_MAX_ITERATIONS: usize = 100;

    /// Default convergence threshold.
    pub const DEFAULT_CONVERGENCE_THRESHOLD: f64 = 1e-4;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone)]
    struct TestConfig {
        threshold: f64,
        count: usize,
    }

    impl Validatable for TestConfig {
        type Error = String;

        fn validate(&self) -> Result<(), Self::Error> {
            if self.threshold <= 0.0 {
                return Err("threshold must be positive".to_string());
            }
            if self.count == 0 {
                return Err("count must be positive".to_string());
            }
            Ok(())
        }
    }

    impl Clampable for TestConfig {
        fn clamp(&mut self) {
            self.threshold = clamp_with_warning(self.threshold, 0.01, 1.0, "threshold");
            self.count = clamp_with_warning(self.count, 1, 1000, "count")
                .min(1000)
                .max(1);
        }
    }

    #[test]
    fn test_validate_success() {
        let config = TestConfig {
            threshold: 0.5,
            count: 10,
        };
        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_validate_failure() {
        let config = TestConfig {
            threshold: -1.0,
            count: 10,
        };
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_validated_chaining() {
        let config = TestConfig {
            threshold: 0.5,
            count: 10,
        };
        assert!(config.validated().is_ok());
    }

    #[test]
    fn test_clamp() {
        let mut config = TestConfig {
            threshold: 2.0,
            count: 0,
        };
        config.clamp();
        assert!(config.threshold <= 1.0);
        assert!(config.count >= 1);
    }

    #[test]
    fn test_clamped_chaining() {
        let config = TestConfig {
            threshold: 2.0,
            count: 0,
        };
        let clamped = config.clamped();
        assert!(clamped.threshold <= 1.0);
        assert!(clamped.count >= 1);
    }
}
