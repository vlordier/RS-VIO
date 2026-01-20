//! Macros to reduce code bloat from repetitive patterns.

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
        if a < b { a } else { b }
    }};
}

#[macro_export]
macro_rules! fast_max {
    ($a:expr, $b:expr) => {{
        let a = $a;
        let b = $b;
        if a > b { a } else { b }
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
