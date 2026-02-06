# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2026-01-11

### Added
- **Safety lint configuration** for embedded and safety-critical systems
- **SAFETY.md**: Comprehensive safety documentation
- **Build profiles**: Three-tier strategy (release, embedded-safe, ultra-critical)
- **Deny-level lints**: todo, unimplemented, box_collection, rc_buffer
- **Sanitizer support**: Memory, thread, and address sanitizers for pre-deployment testing

### Changed
- Promoted unsafe_code to forbid in Cargo.toml
- Enhanced integer overflow protection in all build profiles
- Updated description to emphasize safety-critical capabilities

### Security
- **unsafe_code = forbid**: 100% safe Rust guarantee
- **expect_used/unwrap_used = warn**: Gradually tightening toward deny

## [Unreleased]

### Added
- Comprehensive unit and integration tests
- Performance benchmarks using Criterion
- Profiling examples with pprof support
- Docker containerization for deployment
- Structured logging with tracing
- Custom error types with thiserror
- Security audits with cargo-audit
- Code coverage reporting with cargo-tarpaulin
- GitHub Actions CI/CD pipeline
- API documentation publishing

### Changed
- Improved code formatting and linting
- Enhanced error handling throughout codebase
- Updated dependency management

### Fixed
- Compilation errors in estimator and datasets modules
- Various clippy warnings and code quality issues

## [0.1.0] - 2024-01-15

### Added
- Initial implementation of Visual-Inertial Odometry system
- Support for stereo camera processing
- Bundle adjustment optimization
- Dataset players for EuRoC, 4Seasons, and TUM-VI
- Real-time visualization with rerun
- Feature tracking and motion estimation
- Basic configuration management
- Core VIO pipeline functionality

### Technical Details
- Implemented in Rust with focus on performance and safety
- Uses nalgebra for linear algebra operations
- Camera intrinsic models with distortion correction
- Sliding window optimization framework
- Modular architecture for extensibility

---

## Release Process

### Version Numbering
This project uses [Semantic Versioning](https://semver.org/):

- **MAJOR**: Breaking changes to the public API
- **MINOR**: New features that are backward compatible
- **PATCH**: Bug fixes and minor improvements

### Release Checklist
- [ ] Update version in `Cargo.toml`
- [ ] Update `CHANGELOG.md` with release notes
- [ ] Run full test suite: `cargo test`
- [ ] Run benchmarks: `cargo bench`
- [ ] Check code coverage: `cargo tarpaulin`
- [ ] Security audit: `cargo audit`
- [ ] Build documentation: `cargo doc`
- [ ] Create git tag: `git tag vX.Y.Z`
- [ ] Push to crates.io: `cargo publish`
- [ ] Create GitHub release with changelog
