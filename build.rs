/// Build script for RS-VIO
///
/// Validates feature flag combinations at compile time to prevent invalid configurations,
/// and handles platform-specific linking (Metal on macOS)
#[allow(clippy::panic)]
fn main() {
    // ============================================================================
    // FEATURE FLAG VALIDATION
    // ============================================================================
    
    // Mutually exclusive matching strategy features
    let matching_strategies = [
        "matching-basic-ransac",
        "matching-imu-guided",
        "matching-temporal",
        "matching-hybrid-of",
    ];

    // Count how many matching strategies are enabled
    let enabled_strategies: Vec<&str> = matching_strategies
        .iter()
        .filter(|strategy| {
            let env_var = format!(
                "CARGO_FEATURE_{}",
                strategy.to_uppercase().replace('-', "_")
            );
            std::env::var_os(&env_var).is_some()
        })
        .copied()
        .collect();

    // Validate: exactly one matching strategy must be enabled
    match enabled_strategies.len() {
        0 => {
            // Default to basic-ransac if none specified
            println!("cargo:rustc-cfg=feature=\"matching-basic-ransac\"");
        },
        1 => {
            // Valid: exactly one strategy enabled
            println!(
                "cargo:warning=Using matching strategy: {}",
                enabled_strategies[0]
            );
        },
        _ => {
            panic!(
                "Feature flag error: Exactly ONE matching strategy must be enabled.\n\
                 Found {} enabled: {:?}\n\
                 Valid options:\n  - matching-basic-ransac (default)\n  - matching-imu-guided\n  - matching-temporal\n  - matching-hybrid-of",
                enabled_strategies.len(),
                enabled_strategies
            );
        },
    }

    // Validate feature combinations
    let has_rerun = std::env::var_os("CARGO_FEATURE_RERUN_VIEWER").is_some();
    let has_gpu = std::env::var_os("CARGO_FEATURE_GPU").is_some();
    let has_lightglue = std::env::var_os("CARGO_FEATURE_LIGHTGLUE").is_some();
    let has_metal = std::env::var_os("CARGO_FEATURE_METAL_GPU").is_some();

    // GPU + LightGlue combination is beneficial
    if has_gpu && has_lightglue {
        println!(
            "cargo:warning=Optimizing for GPU-accelerated feature matching (LightGlue + WGPU)"
        );
    }

    // Rerun + any feature is fine
    if has_rerun {
        println!("cargo:warning=Rerun visualization enabled (adds ~30% binary size)");
    }

    // Embedded + GPU might conflict
    let has_embedded = std::env::var_os("CARGO_FEATURE_EMBEDDED").is_some();
    if has_embedded && has_gpu {
        println!("cargo:warning=Embedded + GPU: Ensure Jetson/ARM has WGPU support");
    }

    // ============================================================================
    // PLATFORM-SPECIFIC LINKING
    // ============================================================================
    
    // Metal frameworks on macOS for student data export
    #[cfg(target_os = "macos")]
    {
        if has_metal {
            println!("cargo:rustc-link-lib=framework=Metal");
            println!("cargo:rustc-link-lib=framework=MetalPerformanceShaders");
            println!("cargo:rustc-link-lib=framework=Foundation");
            println!("cargo:rustc-link-lib=framework=CoreGraphics");
            println!("cargo:rustc-link-lib=framework=CoreImage");
            println!("cargo:rustc-link-lib=framework=AppKit");
            println!("cargo:warning=Metal GPU acceleration enabled for export_student_data");
        }
    }

    // Print summary
    println!("cargo:warning=RS-VIO feature validation passed");
}

