/// Logging helpers with compile-time feature gating for hot paths.
///
/// Enable with `--features debug-logging` to keep debug/trace logs.
/// When the feature is disabled (default), these macros compile to no-ops
/// to avoid overhead in real-time builds.
#[cfg(feature = "debug-logging")]
#[macro_export]
macro_rules! debug_log {
    ($($arg:tt)*) => {
        log::debug!($($arg)*)
    };
}

#[cfg(not(feature = "debug-logging"))]
#[macro_export]
macro_rules! debug_log {
    ($($arg:tt)*) => {
        ()
    };
}

#[cfg(feature = "debug-logging")]
#[macro_export]
macro_rules! trace_log {
    ($($arg:tt)*) => {
        log::trace!($($arg)*);
    };
}

#[cfg(not(feature = "debug-logging"))]
#[macro_export]
macro_rules! trace_log {
    ($($arg:tt)*) => {
        ()
    };
}
