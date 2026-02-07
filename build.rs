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

    if enabled.len() > 1 {
        panic!(
            "Only one matching strategy can be enabled at a time, but found: {:?}. \
             Use --features <strategy> to select one.",
            enabled
        );
    }

    if enabled.is_empty() {
        println!(
            "cargo::warning=No matching strategy enabled, using default (matching-basic-ransac)"
        );
    }
}
