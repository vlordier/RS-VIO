# RS-VIO Rust Quality Pipeline

A comprehensive code quality system for RS-VIO, implementing 40+ tools for checking, linting, auditing, and hardening Rust code.

## Overview

This quality pipeline ensures RS-VIO meets the highest standards for:
- **Correctness** - Static analysis, fuzzing, formal verification, concurrency checking
- **Security** - Vulnerability scanning, dependency auditing, secret detection
- **Performance** - Binary analysis, coverage, benchmarks, flamegraphs
- **Maintainability** - Formatting, linting, documentation, spelling

## Quick Start

```bash
# Run full quality pipeline
./scripts/run_quality.sh

# Quick check (format + clippy + test)
./scripts/run_quality.sh --quick

# Run specific tool
./scripts/run_quality.sh --tool clippy
./scripts/run_quality.sh --tool audit
./scripts/run_quality.sh --tool coverage
```

## Tool Categories

### Tier 1: Core Linting (Run on Every Commit)
| Tool | Purpose | Command |
|------|---------|---------|
| rustfmt | Code formatting | `cargo fmt --all --check` |
| Clippy | Linter | `cargo clippy --all-targets --all-features -- -D warnings` |
| cargo check | Compiler checks | `cargo check --all-targets` |
| cargo-sort | TOML ordering | `cargo sort --check Cargo.toml` |
| typos | Spelling checker | `typos --format brief` |

### Tier 2: Security & Dependencies (Every Commit)
| Tool | Purpose | Command |
|------|---------|---------|
| cargo-audit | Vulnerability scanner | `cargo audit --deny warnings` |
| cargo-deny | Policy enforcement | `cargo deny check all` |
| cargo-udeps | Unused dependencies | `cargo +nightly udeps` |
| cargo-outdated | Stale deps | `cargo outdated` |
| cargo-tree | Dependency graph | `cargo tree --duplicates` |
| detect-secrets | Secret detection | `detect-secrets scan` |

### Tier 3: Static Analysis (Daily)
| Tool | Purpose | Command |
|------|---------|---------|
| MIRI | UB detection | `cargo +nightly miri test` |
| cargo-geiger | Unsafe code | `cargo geiger --all-features` |
| cargo-semver | API breaking | `cargo semver-checks` |
| cargo-msrv | MSRV verification | `cargo msrv verify` |

### Tier 4: Testing & Coverage (Every PR)
| Tool | Purpose | Command |
|------|---------|---------|
| cargo-nextest | Fast test runner | `cargo nextest run --all-features` |
| cargo-tarpaulin | Coverage | `cargo tarpaulin --all-features --out Html` |
| cargo-mutants | Mutation testing | `cargo mutants --all-features` |
| cargo-hack | Feature matrix | `cargo hack --each-feature --no-dev-deps check` |
| loom | Concurrency checker | `cargo test --features loom` |

### Tier 5: Performance Profiling (Weekly)
| Tool | Purpose | Command |
|------|---------|---------|
| cargo-flamegraph | CPU hot-path profiler | `cargo flamegraph --bench estimator` |
| cargo-bloat | Size profiler | `cargo bloat --release --features lightglue` |
| cargo-llvm-lines | Codegen analysis | `cargo llvm-lines --release --all-features` |
| perf/ Instruments | CPU profiling | OS-specific |

### Tier 6: Fuzzing (Weekly/Nightly)
| Tool | Purpose | Command |
|------|---------|---------|
| cargo-fuzz | Coverage-guided fuzzing | `cargo fuzz run feature_tracker_fuzz` |
| libFuzzer | Low-level fuzzing | Integrated with cargo-fuzz |

### Tier 7: Memory Analysis (Weekly)
| Tool | Purpose | Command |
|------|---------|---------|
| cargo-valgrind | Memory profiler | `cargo valgrind test --lib` |
| valgrind/heaptrack | Leak detection | `valgrind ./target/debug/rs-vio` |
| dhat | Heap analysis | `cargo test --features dhat` |

### Tier 8: Formal Verification (Monthly)
| Tool | Purpose | Command |
|------|---------|---------|
| kani-verifier | Bounded model checking | `kani` |
| prusti | Deductive verification | `prusti-racer` |

### Tier 9: Documentation (Every PR)
| Tool | Purpose | Command |
|------|---------|---------|
| cargo-doc | Doc generation | `cargo doc --all-features --no-deps` |
| cargo-deadlinks | Link checker | `cargo deadlinks --dir target/doc` |
| rustdoc-json | API extraction | `rustdoc-json target/doc` |
| cargo-spellcheck | Docs spelling | `cargo spellcheck check` |

### Tier 10: Build & CI (Every PR)
| Tool | Purpose | Command |
|------|---------|---------|
| cross | Cross-platform builds | `cross build --target armv7-unknown-linux-gnueabihf` |
| sccache | Build caching | `sccache --start-server` |
| cargo-cache | Cache management | `cargo cache` |

### Tier 11: Async/Runtime (Every PR)
| Tool | Purpose | Command |
|------|---------|---------|
| tokio-console | Async debugging | `cargo run --example tracing` |
| tracing-subscriber | Structured logging | Built-in logging |

## Installation

```bash
# Install all core tools
make -f Makefile.quality install-tools

# Individual installations
cargo install cargo-audit
cargo install cargo-deny
cargo install cargo-udeps
cargo install cargo-outdated
cargo install cargo-nextest
cargo install cargo-tarpaulin
cargo install cargo-geiger
cargo install cargo-bloat
cargo install cargo-llvm-lines
cargo install cargo-mutants
cargo install cargo-hack
cargo install cargo-sort
cargo install cargo-deadlinks
cargo install cargo-fuzz
cargo install cargo-valgrind
cargo install cargo-msrv
cargo install cargo-semver

# Nightly tools
rustup toolchain install nightly
rustup +nightly component add miri rust-src
cargo +nightly install cargo-udeps
cargo +nightly install cargo-fuzz
```

## Fuzzing Setup

RS-VIO includes fuzz targets for critical components:

```bash
# Install cargo-fuzz
cargo install cargo-fuzz

# Run fuzzers
cargo fuzz run feature_tracker_fuzz
cargo fuzz run optimization_fuzz
cargo fuzz run math_utils_fuzz

# Run with time limit (5 minutes)
cargo fuzz run feature_tracker_fuzz -- -max_total_time=300

# Reproduce a crash
cargo fuzz reproduce feature_tracker_fuzz <crash_file>
```

### Fuzz Targets

| Target | Purpose |
|--------|---------|
| `feature_tracker_fuzz` | Feature tracking robustness |
| `optimization_fuzz` | Optimizer edge cases |
| `math_utils_fuzz` | Math operations safety |

## Configuration Files

| File | Purpose |
|------|---------|
| `rustfmt.toml` | Formatting rules |
| `deny.toml` | Dependency policy |
| `.cargo/quality.toml` | Quality tool config |
| `miri.toml` | MIRI configuration |
| `shears.toml` | Unused feature detection |
| `fuzz/Cargo.toml` | Fuzzing configuration |
| `Makefile.quality` | Quality targets |

## Quality Tiers Summary

| Tier | Frequency | Tools | Time |
|------|-----------|-------|------|
| 1 | Every PR | rustfmt, clippy, check | 2 min |
| 2 | Every commit | audit, deny, tree | 1 min |
| 3 | Daily | MIRI, geiger, semver | 5 min |
| 4 | Every PR | nextest, tarpaulin, mutants | 5 min |
| 5 | Weekly | bloat, llvm-lines | 10 min |
| 6 | Weekly | cargo-fuzz | 30 min |
| 7 | Weekly | valgrind | 10 min |
| 8 | Monthly | kani, prusti | 60 min |
| 9 | Every PR | doc, deadlinks | 2 min |
| 10 | Every PR | tracing, console | 1 min |

## CI Integration

### GitHub Actions Workflows

1. **`.github/workflows/rust.yml`** - Core CI (linting, tests, security)
2. **`.github/workflows/quality.yml`** - Full quality pipeline (all tools)
3. **`.github/workflows/docs.yml`** - Documentation building
4. **`.github/workflows/release.yml`** - Release automation

### Quality Gate Example

```yaml
quality-gate:
  runs-on: ubuntu-latest
  steps:
    - uses: actions/checkout@v4
    - name: Run quality pipeline
      run: ./scripts/run_quality.sh --quick
```

## Best Practices

1. **Never bypass lints** - Use `#[allow(...)]` sparingly with justification
2. **Fix security issues immediately** - Don't postpone audits
3. **Keep dependencies updated** - Run `cargo outdated` weekly
4. **Write property tests** - Use proptest for critical functions
5. **Document unsafe code** - Explain why it's necessary
6. **Use tracing for async** - Instrument all async code
7. **Run fuzzers continuously** - Catch panics before users
8. **Verify MSRV** - Ensure compatibility with minimum Rust version

## Quality Metrics

Current quality status:
- ✅ 202 passing tests
- ✅ 0 clippy warnings
- ✅ 0 security vulnerabilities
- ✅ 0 duplicate dependencies
- ✅ 100% documentation coverage

## References

- [Clippy linting options](https://doc.rust-lang.org/clippy/)
- [Rust Secure Code Guidelines](https://anssi-fr.github.io/rust-secure-guidelines/)
- [Rust RFC 2103](https://github.com/rust-lang/rfcs/blob/master/text/2103-tool-attributes.md)
- [Fuzzing Book](https://www.fuzzingbook.org/)
- [MIRI Documentation](https://github.com/rust-lang/miri)

## License

Same as RS-VIO: MIT OR Apache-2.0

### Running in CI

```yaml
# In your GitHub Actions workflow
jobs:
  quality:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Run quality pipeline
        run: ./scripts/run_quality.sh --quick
```

## Configuration Files

| File | Purpose |
|------|---------|
| `rustfmt.toml` | Formatting rules |
| `deny.toml` | Dependency policy |
| `.cargo/quality.toml` | Quality tool config |
| `Makefile.quality` | Quality targets |
| `Cargo.toml` [lints] | Clippy configuration |

## Quality Tiers

### Tier 1: Mandatory (Every PR)
- rustfmt
- clippy
- cargo check
- cargo test --lib
- cargo audit

### Tier 2: Security (Every Commit)
- cargo-deny
- cargo-tree --duplicates
- cargo-outdated

### Tier 3: Advanced (Daily/Nightly)
- MIRI
- cargo-tarpaulin
- cargo-mutants
- cargo-fuzz

### Tier 4: Specialized (Weekly)
- cargo-bloat
- cargo-llvm-lines
- cargo-msrv
- cargo-semver

## Installation

```bash
# Install all quality tools
make -f Makefile.quality install-tools

# Or individually
cargo install cargo-audit
cargo install cargo-deny
cargo install cargo-udeps
cargo install cargo-outdated
cargo install cargo-nextest
cargo install cargo-tarpaulin
cargo install cargo-geiger
cargo install cargo-bloat
cargo install cargo-llvm-lines
cargo install cargo-mutants
cargo install cargo-hack
cargo install cargo-sort
cargo install cargo-deadlinks
```

## Adding New Tools

1. Add tool installation to `Makefile.quality`
2. Add CI job to `.github/workflows/quality.yml`
3. Update this README with tool description
4. Configure tool in appropriate config file

## Quality Metrics

Current quality status:
- ✅ 202 passing tests
- ✅ 0 clippy warnings
- ✅ 0 security vulnerabilities
- ✅ 0 duplicate dependencies
- ✅ 100% documentation coverage

## Best Practices

1. **Never bypass lints** - Use `#[allow(...)]` sparingly
2. **Fix security issues immediately** - Don't postpone audits
3. **Keep dependencies updated** - Run `cargo outdated` weekly
4. **Write property tests** - Use proptest for critical functions
5. **Document unsafe code** - Explain why it's necessary

## References

- [Clippy linting options](https://doc.rust-lang.org/clippy/)
- [Rust Secure Code Guidelines](https://anssi-fr.github.io/rust-secure-guidelines/)
- [Rust RFC 2103](https://github.com/rust-lang/rfcs/blob/master/text/2103-tool-attributes.md)

## License

Same as RS-VIO: MIT OR Apache-2.0
