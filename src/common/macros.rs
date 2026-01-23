//! Macros to reduce code bloat from repetitive patterns.

/// RAII helper that runs a closure when dropped.
pub struct Defer<F: FnOnce()> {
    f: Option<F>,
}

impl<F: FnOnce()> Defer<F> {
    #[inline]
    pub fn new(f: F) -> Self {
        Self { f: Some(f) }
    }
}

impl<F: FnOnce()> Drop for Defer<F> {
    fn drop(&mut self) {
        if let Some(f) = self.f.take() {
            f();
        }
    }
}

/// Early-return with a formatted error.
#[macro_export]
macro_rules! bail {
    ($($arg:tt)*) => {
        return Err(format!($($arg)*));
    };
}

/// Run a closure when the surrounding scope exits.
#[macro_export]
macro_rules! defer {
    ($body:expr) => {
        let _guard = $crate::common::macros::Defer::new(|| $body);
        _guard
    };
}

/// Lightweight scoped timing with debug-only logging.
#[macro_export]
macro_rules! span {
    ($label:expr) => {{
        #[cfg(debug_assertions)]
        log::trace!("▶ {}", $label);
        let __start = std::time::Instant::now();
        let _guard = $crate::common::macros::Defer::new(|| {
            #[cfg(debug_assertions)]
            log::trace!("■ {} ({:?})", $label, __start.elapsed());
        });
        _guard
    }};
    ($label:expr, $($arg:tt)*) => {{
        #[cfg(debug_assertions)]
        log::trace!("▶ {} {}", $label, format!($($arg)*));
        let __start = std::time::Instant::now();
        let _guard = $crate::common::macros::Defer::new(|| {
            #[cfg(debug_assertions)]
            log::trace!("■ {} {} ({:?})", $label, format!($($arg)*), __start.elapsed());
        });
        _guard
    }};
}

/// Clamp a value into [min, max]; on violation, log and return fallback.
#[macro_export]
macro_rules! clamp_or {
    ($value:expr, $min:expr, $max:expr, $fallback:expr, $($arg:tt)*) => {{
        let v = $value;
        if v < $min || v > $max {
            log::warn!($($arg)*);
            $fallback
        } else {
            v
        }
    }};
}

/// Warn once when a config or path is ignored or missing.
#[macro_export]
macro_rules! cfg_warn {
    ($($arg:tt)*) => {{
        static ONCE: std::sync::Once = std::sync::Once::new();
        ONCE.call_once(|| {
            log::warn!($($arg)*);
        });
    }};
}

/// Numeric approximate assertion (debug-only).
#[macro_export]
macro_rules! debug_assert_approx {
    ($a:expr, $b:expr, $tol:expr) => {{
        #[cfg(debug_assertions)]
        {
            let diff = ($a - $b).abs();
            let tol = $tol;
            if diff > tol {
                panic!(
                    "approx assertion failed: |{} - {}| = {} > {}",
                    stringify!($a),
                    stringify!($b),
                    diff,
                    tol
                );
            }
        }
    }};
}

/// Log and return a default when Option is None.
#[macro_export]
macro_rules! unwrap_or_log {
    ($expr:expr, $default:expr, $($arg:tt)*) => {{
        match $expr {
            Some(val) => val,
            None => {
                log::warn!($($arg)*);
                $default
            }
        }
    }};
}

/// Log and return a default when Result is Err.
#[macro_export]
macro_rules! ok_or_log {
    ($expr:expr, $default:expr, $($arg:tt)*) => {{
        match $expr {
            Ok(val) => val,
            Err(e) => {
                log::warn!("{}: {}", format!($($arg)*), e);
                $default
            }
        }
    }};
}

/// Add IO context to errors, returning Result with String.
#[macro_export]
macro_rules! instrument_io {
    ($expr:expr, $context:expr) => {{
        match $expr {
            Ok(val) => Ok(val),
            Err(e) => Err(format!("{}: {}", $context, e)),
        }
    }};
}

/// Unwrap a Result or return early with formatted context.
#[macro_export]
macro_rules! ok_or_bail {
    ($expr:expr, $($arg:tt)*) => {{
        match $expr {
            Ok(val) => val,
            Err(e) => return Err(format!("{}: {}", format!($($arg)*), e)),
        }
    }};
}

/// Unwrap an Option or return early with formatted error.
#[macro_export]
macro_rules! some_or_bail {
    ($expr:expr, $($arg:tt)*) => {{
        match $expr {
            Some(val) => val,
            None => return Err(format!($($arg)*)),
        }
    }};
}

/// Match Some(...) and execute a branch; otherwise run an else expression.
#[macro_export]
macro_rules! match_some {
    ($expr:expr, $pat:pat => $body:expr, else $else_expr:expr) => {{
        match $expr {
            Some($pat) => $body,
            None => $else_expr,
        }
    }};
}
/// Macro to reduce serde deserialization boilerplate.
///
/// This macro generates optimized deserialize implementations that reduce
/// code size compared to derived implementations.
#[macro_export]
macro_rules! impl_validated_config {
    ($type:ty, $validate:ident) => {
        impl $type {
            /// Deserialize and validate in one step.
            #[inline]
            pub fn from_yaml_str(yaml: &str) -> Result<Self, String> {
                let mut config: Self = serde_yaml::from_str(yaml)
                    .map_err(|e| format!("Failed to parse YAML: {}", e))?;
                config.$validate();
                Ok(config)
            }

            /// Deserialize from file and validate.
            #[inline]
            pub fn from_file(path: impl AsRef<std::path::Path>) -> Result<Self, String> {
                let content = std::fs::read_to_string(path.as_ref())
                    .map_err(|e| format!("Failed to read file: {}", e))?;
                Self::from_yaml_str(&content)
            }
        }
    };
}

/// Macro to create inline getter methods that avoid function call overhead.
#[macro_export]
macro_rules! inline_getters {
    ($(fn $name:ident(&self) -> $ret:ty { self.$field:ident })*) => {
        $(
            #[inline(always)]
            pub fn $name(&self) -> $ret {
                self.$field
            }
        )*
    };
}

/// Macro for logging with reduced overhead in release builds.
#[macro_export]
macro_rules! debug_trace {
    ($($arg:tt)*) => {
        #[cfg(debug_assertions)]
        log::trace!($($arg)*);
    };
}

/// Macro for conditional compilation of expensive debug checks.
#[macro_export]
macro_rules! debug_check {
    ($cond:expr, $msg:expr) => {
        #[cfg(debug_assertions)]
        {
            if !$cond {
                panic!("Debug assertion failed: {}", $msg);
            }
        }
    };
}

/// Optimized min/max that compiles to branchless code.
#[macro_export]
macro_rules! fast_min {
    ($a:expr, $b:expr) => {{
        let a = $a;
        let b = $b;
        if a < b {
            a
        } else {
            b
        }
    }};
}

#[macro_export]
macro_rules! fast_max {
    ($a:expr, $b:expr) => {{
        let a = $a;
        let b = $b;
        if a > b {
            a
        } else {
            b
        }
    }};
}

/// Batch create color constants to reduce repetition.
#[macro_export]
macro_rules! color_const {
    ($name:ident = rgb($r:expr, $g:expr, $b:expr)) => {
        pub const $name: rerun::components::Color =
            rerun::components::Color::from_rgb($r, $g, $b);
    };
    ($($name:ident = rgb($r:expr, $g:expr, $b:expr);)*) => {
        $(color_const!($name = rgb($r, $g, $b));)*
    };
}

/// Assert that a value is finite (not NaN or infinite); debug-only.
#[macro_export]
macro_rules! assert_finite {
    ($val:expr) => {{
        #[cfg(debug_assertions)]
        {
            let v = $val;
            if !v.is_finite() {
                panic!("Value not finite: {} = {}", stringify!($val), v);
            }
        }
    }};
    ($val:expr, $msg:expr) => {{
        #[cfg(debug_assertions)]
        {
            let v = $val;
            if !v.is_finite() {
                panic!("{}: {} = {}", $msg, stringify!($val), v);
            }
        }
    }};
}

/// Check if value is NaN; return error with message if so.
#[macro_export]
macro_rules! ensure_not_nan {
    ($val:expr, $($arg:tt)*) => {{
        let v = $val;
        if v.is_nan() {
            log::warn!($($arg)*);
            return Err(format!($($arg)*));
        }
        v
    }};
}

/// Guard: run action only if collection is not empty.
#[macro_export]
macro_rules! if_not_empty {
    ($coll:expr, $body:expr) => {{
        if !$coll.is_empty() {
            $body
        }
    }};
}

/// Guard: run action only if collection is empty, else log warning.
#[macro_export]
macro_rules! if_empty_warn {
    ($coll:expr, $body:expr, $($arg:tt)*) => {{
        if $coll.is_empty() {
            $body
        } else {
            log::warn!($($arg)*);
        }
    }};
}

/// Slice or vector guard: return error if empty.
#[macro_export]
macro_rules! ensure_not_empty {
    ($coll:expr, $($arg:tt)*) => {{
        if $coll.is_empty() {
            return Err(format!($($arg)*));
        }
        $coll
    }};
}

/// Slice or vector guard: return error if length != expected.
#[macro_export]
macro_rules! ensure_len {
    ($coll:expr, $expected:expr, $($arg:tt)*) => {{
        if $coll.len() != $expected {
            return Err(format!("{}: expected length {}, got {}", format!($($arg)*), $expected, $coll.len()));
        }
        $coll
    }};
}

/// Vector/array all-finite check; log if any non-finite.
#[macro_export]
macro_rules! assert_all_finite {
    ($coll:expr) => {{
        #[cfg(debug_assertions)]
        {
            if let Some(bad) = $coll.iter().find(|x| !x.is_finite()) {
                panic!("Non-finite value in {}: {}", stringify!($coll), bad);
            }
        }
    }};
    ($coll:expr, $msg:expr) => {{
        #[cfg(debug_assertions)]
        {
            if let Some(bad) = $coll.iter().find(|x| !x.is_finite()) {
                panic!("{}: {}", $msg, bad);
            }
        }
    }};
}

/// Vector guard: ensure all elements are finite; log and use fallback if not.
#[macro_export]
macro_rules! ensure_all_finite {
    ($coll:expr, $fallback:expr, $($arg:tt)*) => {{
        if !$coll.iter().all(|x| x.is_finite()) {
            log::warn!($($arg)*);
            $fallback
        } else {
            $coll
        }
    }};
}

/// Numeric guard: value in range; log and return fallback if out of bounds.
#[macro_export]
macro_rules! ensure_in_range {
    ($val:expr, $min:expr, $max:expr, $fallback:expr, $($arg:tt)*) => {{
        let v = $val;
        if v < $min || v > $max {
            log::warn!($($arg)*);
            $fallback
        } else {
            v
        }
    }};
}

/// Conditional trace log (debug-only); include structured data.
#[macro_export]
macro_rules! trace_cond {
    ($cond:expr, $($arg:tt)*) => {{
        #[cfg(debug_assertions)]
        {
            if $cond {
                log::trace!($($arg)*);
            }
        }
    }};
}
