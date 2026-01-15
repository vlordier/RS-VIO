# RS-VIO Rust Quality Pipeline

A comprehensive code quality system for RS-VIO, implementing 30+ tools for checking, linting, auditing, and hardening Rust code.

## Overview

This quality pipeline ensures RS-VIO meets the highest standards for:
- **Correctness** - Static analysis, fuzzing, mutation testing
- **Security** - Vulnerability scanning, dependency auditing
- **Performance** - Binary analysis, coverage, benchmarks
- **Maintainability** - Formatting, linting, documentation

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

### Core Linting (Run on Every PR)
| Tool | Purpose | Command |
|------|---------|---------|
| rustfmt | Code formatting | `cargo fmt --all` |
| Clippy | Linter | `cargo clippy --all-targets --all-features` |
| cargo check | Compiler checks | `cargo check --all-targets` |

### Security & Dependencies
| Tool | Purpose | Command |
|------|---------|---------|
| cargo-audit | Vulnerability scanner | `cargo audit --deny warnings` |
| cargo-deny | Policy enforcement | `cargo deny check all` |
| cargo-udeps | Unused dependencies | `cargo +nightly udeps` |
| cargo-outdated | Stale deps | `cargo outdated` |

### Static Analysis
| Tool | Purpose | Command |
|------|---------|---------|
| MIRI | UB detection | `cargo +nightly miri test` |
| cargo-geiger | Unsafe code | `cargo geiger` |
| cargo-semver | API breaking | `cargo semver-checks` |

### Testing & Coverage
| Tool | Purpose | Command |
|------|---------|---------|
| cargo-nextest | Fast test runner | `cargo nextest run` |
| cargo-tarpaulin | Coverage | `cargo tarpaulin` |
| cargo-mutants | Mutation testing | `cargo mutants` |
| cargo-hack | Feature matrix | `cargo hack --each-feature` |

### Binary Analysis
| Tool | Purpose | Command |
|------|---------|---------|
| cargo-bloat | Size profiler | `cargo bloat` |
| cargo-llvm-lines | Codegen analysis | `cargo llvm-lines` |

### Documentation
| Tool | Purpose | Command |
|------|---------|---------|
| cargo-doc | Doc generation | `cargo doc --all-features` |
| cargo-deadlinks | Link checker | `cargo deadlinks` |

## CI Integration

### GitHub Actions Workflows

1. **`.github/workflows/rust.yml`** - Core CI with linting, tests, security
2. **`.github/workflows/quality.yml`** - Full quality pipeline (all 30 tools)
3. **`.github/workflows/docs.yml`** - Documentation building
4. **`.github/workflows/release.yml`** - Release automation

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
