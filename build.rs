// Build script to enforce mutual exclusivity of matching strategies

fn main() {
    // Only re-run when the build script itself changes (not on every build)
    println!("cargo::rerun-if-changed=build.rs");
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

    // Separate default strategy from explicit non-default strategies
    let non_default_strategies: Vec<_> = enabled
        .iter()
        .filter(|&&s| *s != "matching-basic-ransac")
        .collect();

    // Check mutual exclusivity only among non-default strategies
    if non_default_strategies.len() > 1 {
        panic!(
            "Only one matching strategy can be enabled at a time, but found: {:?}. \
             Use --no-default-features --features <strategy> to select one.",
            non_default_strategies
        );
    }

    // If a non-default strategy is enabled, it takes precedence over the default
    if !non_default_strategies.is_empty() {
        println!(
            "cargo::warning=Using matching strategy: {}",
            non_default_strategies[0]
        );
    } else if enabled.is_empty() {
        println!(
            "cargo::warning=No matching strategy enabled, using default (matching-basic-ransac)"
        );
    } else {
        // Only matching-basic-ransac is enabled (either explicitly or via default)
        println!(
            "cargo::warning=Using matching strategy: matching-basic-ransac"
        );
    }
}
