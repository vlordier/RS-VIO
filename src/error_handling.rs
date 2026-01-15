//! Error handling traits for better error composition and conversion
//!
//! This module provides trait abstractions for error handling, enabling
//! better composition and cleaner error conversions throughout the codebase.

use std::fmt;

/// Trait for types that can be converted to a user-friendly error message
///
/// This enables consistent error reporting across the application while
/// maintaining the underlying error context.
pub trait ErrorMessage: std::error::Error {
    /// Get a user-friendly message for this error
    fn user_message(&self) -> String {
        self.to_string()
    }

    /// Get technical details for logging/debugging
    fn technical_details(&self) -> String {
        format!("{:?}", self)
    }

    /// Check if this error is recoverable
    fn is_recoverable(&self) -> bool {
        true
    }
}

/// Context information for error reporting
#[derive(Debug, Clone)]
pub struct ErrorContext {
    pub component: String,
    pub operation: String,
    pub details: String,
}

impl ErrorContext {
    pub fn new(component: impl Into<String>, operation: impl Into<String>) -> Self {
        ErrorContext {
            component: component.into(),
            operation: operation.into(),
            details: String::new(),
        }
    }

    pub fn with_details(mut self, details: impl Into<String>) -> Self {
        self.details = details.into();
        self
    }
}

impl fmt::Display for ErrorContext {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[{}::{}]{}",
            self.component,
            self.operation,
            if self.details.is_empty() {
                String::new()
            } else {
                format!(" {}", self.details)
            }
        )
    }
}

/// Trait for adding context to errors
pub trait WithContext<T> {
    /// Add context information to an error
    fn with_context(self, context: ErrorContext) -> Result<T, String>;
}

impl<T, E: std::error::Error> WithContext<T> for Result<T, E> {
    fn with_context(self, context: ErrorContext) -> Result<T, String> {
        self.map_err(|e| format!("{}: {}", context, e))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_context_creation() {
        let ctx = ErrorContext::new("DataLoader", "load_images");
        assert_eq!(ctx.component, "DataLoader");
        assert_eq!(ctx.operation, "load_images");
    }

    #[test]
    fn test_error_context_with_details() {
        let ctx = ErrorContext::new("DataLoader", "load_images")
            .with_details("File not found: /path/to/file");
        assert_eq!(ctx.details, "File not found: /path/to/file");
    }

    #[test]
    fn test_error_context_display() {
        let ctx = ErrorContext::new("Estimator", "process_frame")
            .with_details("Invalid image dimensions");
        let msg = ctx.to_string();
        assert!(msg.contains("Estimator::process_frame"));
        assert!(msg.contains("Invalid image dimensions"));
    }
}
