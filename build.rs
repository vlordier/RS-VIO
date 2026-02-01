// Build script to enforce mutual exclusivity of matching strategies

fn main() {
    let strategies = [
        "matching-basic-ransac",
        "matching-imu-guided",
        "matching-temporal",
        "matching-hybrid-of",
    ];

    let enabled: Vec<_> = strategies
        .iter()
        .filter(|&s| {
            std::env::var(format!(
                "CARGO_FEATURE_{}",
                s.to_uppercase().replace('-', "_")
            ))
            .is_ok()
        })
        .collect();

    if enabled.len() > 1 {
        panic!(
            "Only one matching strategy can be enabled at a time, but found: {:?}. \
             Use --features <strategy> to select one.",
            enabled
        );
    }

    if enabled.is_empty() {
        eprintln!("warning: No matching strategy enabled, using default (matching-basic-ransac)");
    }

    // ========================================================================
    // Metal GPU Framework Configuration
    // ========================================================================
    // Only link the Metal framework when GPU support is explicitly enabled
    // on macOS. This keeps the build cleaner and avoids linking unnecessary
    // frameworks when GPU features aren't used.
    if std::env::var("CARGO_FEATURE_GPU").is_ok() {
        if let Ok("macos") = std::env::var("CARGO_CFG_TARGET_OS").as_deref() {
            println!("cargo:rustc-link-arg=-framework");
            println!("cargo:rustc-link-arg=Metal");
        }
    }
}
