//! Error handling utilities and context helpers.
//!
//! This module provides utilities for enriching error messages with context
//! and common error handling patterns to promote consistency across the codebase.

use std::fmt::Display;

/// Extension trait for Results to add context to errors.
///
/// This trait provides a fluent API for adding contextual information
/// to error types, making debugging easier.
pub trait ResultExt<T, E> {
    /// Add context to an error.
    ///
    /// # Examples
    ///
    /// ```
    /// use rs_vio::common::error::ResultExt;
    ///
    /// fn read_config() -> Result<String, std::io::Error> {
    ///     std::fs::read_to_string("config.yaml")
    ///         .with_context("Failed to read configuration file")
    /// }
    /// ```
    fn with_context<C: Display>(self, context: C) -> Result<T, String>;

    /// Add context to an error using a closure (lazy evaluation).
    ///
    /// Use this when the context message is expensive to compute.
    fn with_context_lazy<C, F>(self, f: F) -> Result<T, String>
    where
        C: Display,
        F: FnOnce() -> C;
}

impl<T, E: Display> ResultExt<T, E> for Result<T, E> {
    fn with_context<C: Display>(self, context: C) -> Result<T, String> {
        self.map_err(|e| format!("{}: {}", context, e))
    }

    fn with_context_lazy<C, F>(self, f: F) -> Result<T, String>
    where
        C: Display,
        F: FnOnce() -> C,
    {
        self.map_err(|e| format!("{}: {}", f(), e))
    }
}

/// Extension trait for Options to convert to Results with context.
pub trait OptionExt<T> {
    /// Convert None to an error with context.
    ///
    /// # Examples
    ///
    /// ```
    /// use rs_vio::common::error::OptionExt;
    ///
    /// fn get_value(map: &std::collections::HashMap<String, i32>, key: &str) -> Result<i32, String> {
    ///     map.get(key)
    ///         .copied()
    ///         .ok_or_context(format!("Key '{}' not found", key))
    /// }
    /// ```
    fn ok_or_context<C: Display>(self, context: C) -> Result<T, String>;

    /// Convert None to an error with lazy context.
    fn ok_or_context_lazy<C, F>(self, f: F) -> Result<T, String>
    where
        C: Display,
        F: FnOnce() -> C;
}

impl<T> OptionExt<T> for Option<T> {
    fn ok_or_context<C: Display>(self, context: C) -> Result<T, String> {
        self.ok_or_else(|| format!("{}", context))
    }

    fn ok_or_context_lazy<C, F>(self, f: F) -> Result<T, String>
    where
        C: Display,
        F: FnOnce() -> C,
    {
        self.ok_or_else(|| format!("{}", f()))
    }
}

/// Helper macro for early return with context.
///
/// Similar to the `?` operator but adds context to the error.
///
/// # Examples
///
/// ```
/// # use rs_vio::context_bail;
/// fn process() -> Result<(), String> {
///     let file = std::fs::File::open("data.txt")
///         .map_err(|e| format!("Failed to open file: {}", e))?;
///     Ok(())
/// }
/// ```
#[macro_export]
macro_rules! context_bail {
    ($result:expr, $context:expr) => {
        match $result {
            Ok(val) => val,
            Err(e) => return Err(format!("{}: {}", $context, e)),
        }
    };
}

/// Helper macro for ensuring a condition is true, otherwise return error.
///
/// # Examples
///
/// ```
/// # use rs_vio::ensure;
/// fn validate_positive(x: f64) -> Result<(), String> {
///     ensure!(x > 0.0, "Value must be positive, got {}", x);
///     Ok(())
/// }
/// ```
#[macro_export]
macro_rules! ensure {
    ($cond:expr, $($arg:tt)*) => {
        if !$cond {
            return Err(format!($($arg)*));
        }
    };
}

/// Collect multiple errors into a single error message.
///
/// Useful for validation that needs to report all errors at once
/// rather than stopping at the first error.
///
/// # Examples
///
/// ```
/// use rs_vio::common::error::ErrorCollector;
///
/// let mut errors = ErrorCollector::new("Configuration validation failed");
/// errors.push_if(value < 0.0, "Value must be non-negative");
/// errors.push_if(count == 0, "Count must be positive");
/// errors.into_result(())
/// ```
#[derive(Default)]
pub struct ErrorCollector {
    context: String,
    errors: Vec<String>,
}

impl ErrorCollector {
    /// Create a new error collector with a context message.
    pub fn new<S: Into<String>>(context: S) -> Self {
        Self {
            context: context.into(),
            errors: Vec::new(),
        }
    }

    /// Add an error if the condition is true.
    pub fn push_if<S: Into<String>>(&mut self, condition: bool, message: S) {
        if condition {
            self.errors.push(message.into());
        }
    }

    /// Add an error unconditionally.
    pub fn push<S: Into<String>>(&mut self, message: S) {
        self.errors.push(message.into());
    }

    /// Check if any errors were collected.
    pub fn has_errors(&self) -> bool {
        !self.errors.is_empty()
    }

    /// Convert to a Result, returning Ok if no errors were collected.
    pub fn into_result<T>(self, ok_value: T) -> Result<T, String> {
        if self.errors.is_empty() {
            Ok(ok_value)
        } else {
            Err(format!("{}: {}", self.context, self.errors.join("; ")))
        }
    }

    /// Get a reference to the collected errors.
    pub fn errors(&self) -> &[String] {
        &self.errors
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_result_with_context() {
        let result: Result<i32, &str> = Err("original error");
        let with_ctx = result.with_context("additional context");

        assert!(with_ctx.is_err());
        assert_eq!(with_ctx.unwrap_err(), "additional context: original error");
    }

    #[test]
    fn test_result_with_context_lazy() {
        let result: Result<i32, &str> = Err("error");
        let with_ctx = result.with_context_lazy(|| format!("context {}", 42));

        assert_eq!(with_ctx.unwrap_err(), "context 42: error");
    }

    #[test]
    fn test_option_ok_or_context() {
        let opt: Option<i32> = None;
        let result = opt.ok_or_context("value missing");

        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), "value missing");
    }

    #[test]
    fn test_option_ok_or_context_lazy() {
        let opt: Option<i32> = None;
        let result = opt.ok_or_context_lazy(|| format!("missing {}", "value"));

        assert_eq!(result.unwrap_err(), "missing value");
    }

    #[test]
    fn test_ensure_macro() {
        fn check_positive(x: f64) -> Result<(), String> {
            ensure!(x > 0.0, "Value must be positive, got {}", x);
            Ok(())
        }

        assert!(check_positive(1.0).is_ok());
        assert!(check_positive(-1.0).is_err());
        assert_eq!(
            check_positive(-1.0).unwrap_err(),
            "Value must be positive, got -1"
        );
    }

    #[test]
    fn test_error_collector_empty() {
        let collector = ErrorCollector::new("Test");
        assert!(!collector.has_errors());
        assert!(collector.into_result(()).is_ok());
    }

    #[test]
    fn test_error_collector_with_errors() {
        let mut collector = ErrorCollector::new("Validation");
        collector.push_if(true, "Error 1");
        collector.push_if(false, "Should not appear");
        collector.push("Error 2");

        assert!(collector.has_errors());
        assert_eq!(collector.errors().len(), 2);

        let result = collector.into_result(());
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), "Validation: Error 1; Error 2");
    }

    #[test]
    fn test_context_bail_macro() {
        fn helper() -> Result<i32, String> {
            let result: Result<i32, &str> = Err("failed");
            Ok(context_bail!(result, "Operation"))
        }

        let err = helper().unwrap_err();
        assert_eq!(err, "Operation: failed");
    }
}
