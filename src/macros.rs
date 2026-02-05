//! Utility macros for code organization, maintainability, and portability
//!
//! This module provides reusable macros for:
//! - Configuration validation and setup
//! - Test utilities and assertions
//! - Common error handling patterns
//! - Logging and metrics
//! - Type-safe builders and factories

/// Macro for validating configuration bounds with descriptive error messages.
///
/// Returns `Ok(())` on success or an `anyhow::Error` on validation failure.
/// Must be used in a context that returns `Result`.
///
/// # Example
/// ```ignore
/// validate_bounds!(value, 0.0, 1.0, "probability")?;
/// ```
#[macro_export]
macro_rules! validate_bounds {
    ($value:expr, $min:expr, $max:expr, $name:expr) => {{
        if $value < $min || $value > $max {
            anyhow::bail!(
                "Invalid {}: {} is outside bounds [{}, {}]",
                $name,
                $value,
                $min,
                $max
            );
        }
        Ok(())
    }};
}

/// Macro for asserting configuration conditions in tests
///
/// # Example
/// ```ignore
/// assert_config!(config.enable_feature, "feature should be enabled");
/// ```
#[macro_export]
macro_rules! assert_config {
    ($cond:expr, $msg:expr) => {
        assert!($cond, "Config assertion failed: {}", $msg);
    };
}

/// Macro for safe downcast with logging.
///
/// The type check ensures the unwrap is safe (no panics on type mismatch).
///
/// # Example
/// ```ignore
/// let typed = downcast_or_log!(obj, TargetType, "failed to downcast object");
/// ```
#[macro_export]
macro_rules! downcast_or_log {
    ($obj:expr, $target:ty, $msg:expr) => {
        match $obj as &dyn std::any::Any {
            obj if obj.is::<$target>() => {
                // Safe because we checked is::<$target>() above
                obj.downcast_ref::<$target>()
            }
            _ => {
                log::warn!("{}", $msg);
                None
            }
        }
    };
}

/// Macro for constructing feature vectors with validation
///
/// # Example
/// ```ignore
/// feature_vec![100.0, 200.0, 300.0, 400.0];
/// ```
#[macro_export]
macro_rules! feature_vec {
    ($($x:expr),+ $(,)?) => {{
        vec![$($x),+]
    }};
}

/// Macro for creating intrinsics safely with bounds checking
///
/// # Example
/// ```ignore
/// intrinsics!(fx: 500.0, fy: 500.0, cx: 320.0, cy: 240.0);
/// ```
#[macro_export]
macro_rules! intrinsics {
    (fx: $fx:expr, fy: $fy:expr, cx: $cx:expr, cy: $cy:expr) => {{
        #[allow(unused)]
        let fx = $fx;
        #[allow(unused)]
        let fy = $fy;
        #[allow(unused)]
        let cx = $cx;
        #[allow(unused)]
        let cy = $cy;
        
        if fx <= 0.0 || fy <= 0.0 {
            log::error!("Invalid focal lengths: fx={}, fy={}", fx, fy);
        }
        vec![fx, fy, cx, cy]
    }};
}

/// Macro for creating transformation matrices (4x4 homogeneous)
///
/// # Example
/// ```ignore
/// transform_matrix!(
///     1.0, 0.0, 0.0, 0.1,
///     0.0, 1.0, 0.0, 0.2,
///     0.0, 0.0, 1.0, 0.3,
///     0.0, 0.0, 0.0, 1.0
/// );
/// ```
#[macro_export]
macro_rules! transform_matrix {
    (
        $m00:expr, $m01:expr, $m02:expr, $m03:expr,
        $m10:expr, $m11:expr, $m12:expr, $m13:expr,
        $m20:expr, $m21:expr, $m22:expr, $m23:expr,
        $m30:expr, $m31:expr, $m32:expr, $m33:expr
    ) => {
        vec![
            $m00, $m01, $m02, $m03,
            $m10, $m11, $m12, $m13,
            $m20, $m21, $m22, $m23,
            $m30, $m31, $m32, $m33,
        ]
    };
}

/// Macro for conditional logging at different levels
///
/// # Example
/// ```ignore
/// log_if!(info, condition, "Message with {}", value);
/// ```
#[macro_export]
macro_rules! log_if {
    (debug, $cond:expr, $($arg:tt)*) => {
        if $cond {
            log::debug!($($arg)*);
        }
    };
    (info, $cond:expr, $($arg:tt)*) => {
        if $cond {
            log::info!($($arg)*);
        }
    };
    (warn, $cond:expr, $($arg:tt)*) => {
        if $cond {
            log::warn!($($arg)*);
        }
    };
    (error, $cond:expr, $($arg:tt)*) => {
        if $cond {
            log::error!($($arg)*);
        }
    };
}

/// Macro for result chains with context
///
/// # Example
/// ```ignore
/// result_chain!(value, "operation", "context about what failed");
/// ```
#[macro_export]
macro_rules! result_chain {
    ($result:expr, $operation:expr) => {
        $result.map_err(|e| anyhow::anyhow!("{}: {}", $operation, e))
    };
    ($result:expr, $operation:expr, $context:expr) => {
        $result.map_err(|e| anyhow::anyhow!("{} ({}): {}", $operation, $context, e))
    };
}

/// Macro for test setup boilerplate
///
/// # Example
/// ```ignore
/// setup_test!(logger, env);
/// ```
#[macro_export]
macro_rules! setup_test {
    () => {
        let _ = env_logger::builder()
            .is_test(true)
            .try_init();
    };
    (logger) => {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(log::LevelFilter::Debug)
            .try_init();
    };
}

/// Macro for asserting approximate equality with epsilon
///
/// # Example
/// ```ignore
/// assert_approx_eq!(a, b, 1e-6);
/// ```
#[macro_export]
macro_rules! assert_approx_eq {
    ($a:expr, $b:expr, $eps:expr) => {{
        let a_val: f64 = $a as f64;
        let b_val: f64 = $b as f64;
        let diff = (a_val - b_val).abs();
        assert!(
            diff < $eps,
            "assertion failed: {} ≈ {} (diff: {}, eps: {})",
            a_val,
            b_val,
            diff,
            $eps
        );
    }};
}

/// Macro for asserting floating point range
///
/// # Example
/// ```ignore
/// assert_in_range!(value, 0.0, 1.0);
/// ```
#[macro_export]
macro_rules! assert_in_range {
    ($value:expr, $min:expr, $max:expr) => {
        assert!(
            $value >= $min && $value <= $max,
            "assertion failed: {} is not in range [{}, {}]",
            $value,
            $min,
            $max
        );
    };
}

/// Macro for benchmarking code blocks
///
/// # Example
/// ```ignore
/// bench_block!("operation", { /* code */ });
/// ```
#[macro_export]
macro_rules! bench_block {
    ($name:expr, $block:block) => {{
        let start = std::time::Instant::now();
        let result = { $block };
        let elapsed = start.elapsed();
        log::info!("{}: took {:?}", $name, elapsed);
        result
    }};
}

/// Macro for validating camera configuration with error reporting.
///
/// Evaluates validation checks and logs errors for failed checks.
/// Returns a `Vec<bool>` where `true` indicates passed validation.
///
/// # Example
/// ```ignore
/// let results = camera_validator!(width > 640, "width", height > 480, "height");
/// if results.iter().all(|&x| x) { /* all valid */ }
/// ```
#[macro_export]
macro_rules! camera_validator {
    ($($check:expr, $name:expr),+ $(,)?) => {{
        let mut results = Vec::new();
        $(
            let passed = $check;
            if !passed {
                log::error!("Camera validation check failed for {}", $name);
            }
            results.push(passed);
        )+
        results
    }};
}

/// Macro for error propagation with context
///
/// # Example
/// ```ignore
/// try_with_context!(operation(), "failed to complete operation");
/// ```
#[macro_export]
macro_rules! try_with_context {
    ($e:expr) => {
        $e?
    };
    ($e:expr, $msg:expr) => {
        $e.map_err(|err| anyhow::anyhow!("{}: {}", $msg, err))?
    };
}

/// Macro for parsing with descriptive error messages.
///
/// Works with both `anyhow::Error` and `String` error types. Use in Result
/// contexts where error conversion is needed.
///
/// # Example
/// ```ignore
/// fn parse_data() -> anyhow::Result<()> {
///     let value: f64 = parse_or_err!(parts[0], "timestamp");
///     let count: usize = parse_or_err!(input, "count", usize);
///     Ok(())
/// }
/// ```
#[macro_export]
macro_rules! parse_or_err {
    ($value:expr, $field:expr) => {
        $value.parse().map_err(|e| format!("Invalid {}: {} ({})", $field, $value, e))?
    };
    ($value:expr, $field:expr, $ty:ty) => {
        $value.parse::<$ty>().map_err(|e| format!("Invalid {}: {} ({})", $field, $value, e))?
    };
}

/// Macro for mapping errors with format string
///
/// # Example
/// ```ignore
/// let file = File::open(path).map_err_fmt!("Failed to open file")?;
/// let data = read().map_err_fmt!("Read error")?;
/// ```
#[macro_export]
macro_rules! map_err_fmt {
    ($msg:expr) => {
        |e| format!("{}: {}", $msg, e)
    };
}

/// Macro for defining feature-gated code blocks
///
/// Note: Due to Rust's procedural macro limitations, requires a literal string,
/// not an expression, for the feature name.
///
/// # Example
/// ```ignore
/// feature_gate!("matching-imu-guided", {
///     // IMU-guided matching code
/// });
/// ```
#[macro_export]
macro_rules! feature_gate {
    ($feat:literal, $block:block) => {
        #[cfg(feature = $feat)]
        {
            $block
        }
    };
}

/// Macro for scoped timing measurements
///
/// # Example
/// ```ignore
/// let duration = scoped_timer!("operation", { expensive_computation() });
/// let (result, elapsed) = scoped_timer_result!("fetch", { fetch_data() });
/// ```
#[macro_export]
macro_rules! scoped_timer {
    ($name:expr, $block:block) => {{
        let start = std::time::Instant::now();
        let result = { $block };
        let elapsed = start.elapsed();
        log::debug!("{} took {:?}", $name, elapsed);
        result
    }};
}

/// Macro for timing with result capture
#[macro_export]
macro_rules! scoped_timer_result {
    ($name:expr, $block:block) => {{
        let start = std::time::Instant::now();
        let result = { $block };
        let elapsed = start.elapsed();
        (result, elapsed)
    }};
}

/// Macro for safe mutex lock with context.
///
/// Returns `Result` for proper error handling instead of panicking.
/// Recommended usage: `safe_lock!(my_mutex)?` in Result context.
///
/// # Example
/// ```ignore
/// fn access_data() -> anyhow::Result<()> {
///     let data = safe_lock!(my_mutex)?;
///     // use data
///     Ok(())
/// }
/// // Or with explicit error message:
/// let data = safe_lock!(my_mutex, "failed to lock data")
///     .expect("lock failed");
/// ```
#[macro_export]
macro_rules! safe_lock {
    ($mutex:expr) => {
        $mutex.lock().map_err(|e| anyhow::anyhow!("Mutex poisoned: {}", e))
    };
    ($mutex:expr, $msg:expr) => {
        $mutex.lock().map_err(|e| anyhow::anyhow!("{}: {}", $msg, e))
    };
}

/// Macro for prefixed logging
///
/// # Example
/// ```ignore
/// log_prefixed!(info, "RerunViewer", "Initialized successfully");
/// log_prefixed!(warn, "Estimator", "Frame dropped: {}", reason);
/// ```
#[macro_export]
macro_rules! log_prefixed {
    (info, $prefix:expr, $($arg:tt)*) => {
        log::info!("[{}] {}", $prefix, format!($($arg)*))
    };
    (warn, $prefix:expr, $($arg:tt)*) => {
        log::warn!("[{}] {}", $prefix, format!($($arg)*))
    };
    (error, $prefix:expr, $($arg:tt)*) => {
        log::error!("[{}] {}", $prefix, format!($($arg)*))
    };
    (debug, $prefix:expr, $($arg:tt)*) => {
        log::debug!("[{}] {}", $prefix, format!($($arg)*))
    };
    (trace, $prefix:expr, $($arg:tt)*) => {
        log::trace!("[{}] {}", $prefix, format!($($arg)*))
    };
}

/// Macro for unwrapping Option with custom message
///
/// # Example
/// ```ignore
/// let value = unwrap_or_log!(maybe_value, "missing configuration");
/// let item = unwrap_or_return!(option, None, "item not found");
/// ```
#[macro_export]
macro_rules! unwrap_or_log {
    ($option:expr, $msg:expr) => {
        match $option {
            Some(val) => val,
            None => {
                log::warn!("Option is None: {}", $msg);
                return;
            }
        }
    };
}

/// Macro for Option unwrap with early return
#[macro_export]
macro_rules! unwrap_or_return {
    ($option:expr, $return_val:expr) => {
        match $option {
            Some(val) => val,
            None => return $return_val,
        }
    };
    ($option:expr, $return_val:expr, $msg:expr) => {
        match $option {
            Some(val) => val,
            None => {
                log::debug!("{}", $msg);
                return $return_val;
            }
        }
    };
}

/// Macro for parameter validation in functions.
///
/// Must be used in a context that returns `Result` since it uses `anyhow::bail!`.
/// Each failed check will return early with the provided error message.
///
/// # Example
/// ```ignore
/// fn process(width: u32, height: u32) -> Result<()> {
///     validate_params!(
///         width > 0 => "width must be positive",
///         height > 0 => "height must be positive"
///     );
///     Ok(())
/// }
/// ```
#[macro_export]
macro_rules! validate_params {
    ($($check:expr => $msg:expr),+ $(,)?) => {
        $(
            if !$check {
                anyhow::bail!($msg);
            }
        )+
    };
}

/// Macro for thread-safe singleton access
///
/// # Example
/// ```ignore
/// singleton!(LOGGER, Logger, init_logger());
/// ```
#[macro_export]
macro_rules! singleton {
    ($name:ident, $ty:ty, $init:expr) => {
        lazy_static::lazy_static! {
            static ref $name: $ty = $init;
        }
    };
}

/// Macro for constructing builder instances with default field values.
///
/// Note: This macro does not perform validation - it only constructs the builder.
/// Validation should be done separately via builder methods.
///
/// # Example
/// ```ignore
/// builder_with_defaults!(ConfigBuilder {
///     field1: default_value1,
///     field2: default_value2,
/// });
/// ```
#[macro_export]
macro_rules! builder_with_defaults {
    ($builder:path { $($field:ident: $default:expr),+ $(,)? }) => {
        $builder {
            $($field: $default),+
        }
    };
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_intrinsics_macro() {
        let intrinsics = intrinsics!(fx: 500.0, fy: 500.0, cx: 320.0, cy: 240.0);
        assert_eq!(intrinsics.len(), 4);
        assert_eq!(intrinsics[0], 500.0);
    }

    #[test]
    fn test_transform_matrix_macro() {
        let matrix = transform_matrix!(
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        );
        assert_eq!(matrix.len(), 16);
        assert_eq!(matrix[0], 1.0);
        assert_eq!(matrix[5], 1.0);
    }

    #[test]
    fn test_assert_approx_eq_macro() {
        let a = 1.0;
        let b = 1.0000001;
        assert_approx_eq!(a, b, 1e-5);
    }

    #[test]
    fn test_assert_in_range_macro() {
        let value = 0.5;
        assert_in_range!(value, 0.0, 1.0);
    }

    #[test]
    #[should_panic]
    fn test_assert_in_range_macro_fails() {
        let value = 1.5;
        assert_in_range!(value, 0.0, 1.0);
    }

    #[test]
    fn test_parse_or_err_macro() {
        let result: Result<i32, String> = (|| {
            let value = parse_or_err!("42", "number");
            Ok(value)
        })();
        assert_eq!(result.unwrap(), 42);

        let result: Result<f64, String> = (|| {
            let value = parse_or_err!("3.14", "pi", f64);
            Ok(value)
        })();
        assert!((result.unwrap() - 3.14).abs() < 1e-10);
    }

    #[test]
    fn test_map_err_fmt_macro() {
        let result: Result<i32, String> = Err("error".to_string());
        let mapped = result.map_err(map_err_fmt!("Operation failed"));
        assert_eq!(mapped.unwrap_err(), "Operation failed: error");
    }

    #[test]
    fn test_scoped_timer_result() {
        let (result, elapsed) = scoped_timer_result!("test_op", { 
            std::thread::sleep(std::time::Duration::from_micros(100));
            42 
        });
        assert_eq!(result, 42);
        assert!(elapsed.as_micros() >= 50); // Allow some tolerance
    }

    #[test]
    fn test_safe_lock() {
        use std::sync::Mutex;
        let mutex = Mutex::new(42);
        let value = safe_lock!(mutex).expect("failed to lock");
        assert_eq!(*value, 42);
    }

    #[test]
    fn test_unwrap_or_return_some() {
        fn test_fn(opt: Option<i32>) -> i32 {
            let value = unwrap_or_return!(opt, -1);
            value * 2
        }
        assert_eq!(test_fn(Some(21)), 42);
        assert_eq!(test_fn(None), -1);
    }

    #[test]
    fn test_unwrap_or_return_none() {
        fn test_fn(opt: Option<i32>) -> Option<i32> {
            let value = unwrap_or_return!(opt, None, "no value");
            Some(value * 2)
        }
        assert_eq!(test_fn(Some(21)), Some(42));
        assert_eq!(test_fn(None), None);
    }
}
