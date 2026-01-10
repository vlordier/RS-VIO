# Contributing to RS-VIO

Thank you for your interest in contributing to RS-VIO! This document provides guidelines and information for contributors.

## Code of Conduct

This project follows a code of conduct to ensure a welcoming environment for all contributors. By participating, you agree to:

- Be respectful and inclusive
- Focus on constructive feedback
- Accept responsibility for mistakes
- Show empathy towards other contributors
- Help create a positive community

## How to Contribute

### Development Setup

1. **Prerequisites**
   - Rust 1.75 or later
   - Git
   - (Optional) Docker for containerized development

2. **Clone and Setup**
   ```bash
   git clone https://github.com/your-org/rs-vio.git
   cd rs-vio
   cargo build
   cargo test
   ```

3. **Development Workflow**
   ```bash
   # Create feature branch
   git checkout -b feature/your-feature-name

   # Make changes
   # Add tests
   # Run quality checks
   cargo fmt
   cargo clippy
   cargo test
   cargo audit

   # Commit with conventional commit message
   git commit -m "feat: add new feature"

   # Push and create PR
   git push origin feature/your-feature-name
   ```

### Commit Convention

We use [Conventional Commits](https://conventionalcommits.org/) for commit messages:

```
<type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

Types:
- `feat`: New features
- `fix`: Bug fixes
- `docs`: Documentation changes
- `style`: Code style changes (formatting, etc.)
- `refactor`: Code refactoring
- `test`: Adding or fixing tests
- `chore`: Maintenance tasks

Examples:
- `feat: add IMU integration support`
- `fix: correct camera parameter validation`
- `docs: update installation instructions`
- `test: add benchmark for optimization`

### Pull Request Process

1. **Create a PR**
   - Use a descriptive title
   - Reference related issues
   - Provide context and motivation

2. **PR Requirements**
   - All tests pass
   - Code is formatted (`cargo fmt`)
   - No clippy warnings (`cargo clippy`)
   - Security audit passes (`cargo audit`)
   - Documentation is updated if needed

3. **Review Process**
   - At least one maintainer review required
   - CI checks must pass
   - Changes may be requested for style or functionality

### Code Guidelines

#### Rust Best Practices
- Follow the [Rust API Guidelines](https://rust-lang.github.io/api-guidelines/)
- Use `rustfmt` for consistent formatting
- Address all `clippy` warnings
- Write comprehensive tests
- Document public APIs with `///` comments

#### Project Structure
```
src/
├── lib.rs              # Main library interface
├── datasets/           # Dataset handling
├── estimator/          # VIO estimation logic
├── feature_tracker/    # Feature tracking
├── optimization/       # Bundle adjustment
├── types.rs            # Common types and traits
└── viewers/            # Visualization

tests/                  # Integration tests
benches/               # Performance benchmarks
examples/              # Usage examples
config/                # Configuration files
scripts/               # Utility scripts
```

#### Naming Conventions
- Use `snake_case` for variables and functions
- Use `CamelCase` for types and traits
- Use `SCREAMING_SNAKE_CASE` for constants
- Coordinate frames use `T_A_B` convention (transform from B to A)

#### Error Handling
- Use custom error types with `thiserror`
- Prefer `Result<T, Error>` over panics
- Provide meaningful error messages
- Log errors appropriately

#### Testing
- Write unit tests for all public functions
- Use integration tests for end-to-end functionality
- Aim for high code coverage (>80%)
- Use property-based testing where applicable

#### Documentation
- Document all public APIs
- Include code examples in documentation
- Keep README and docs up to date
- Use `cargo doc` to generate docs

### Performance Considerations

- Profile performance-critical code
- Use appropriate data structures
- Minimize allocations in hot paths
- Consider cache-friendly memory layouts
- Use SIMD where beneficial

### Security

- Follow secure coding practices
- Validate all inputs
- Use safe Rust constructs
- Run security audits regularly
- Report security issues responsibly

## Getting Help

- **Issues**: Use GitHub issues for bugs and feature requests
- **Discussions**: Use GitHub discussions for questions and ideas
- **Documentation**: Check [docs.rs/rs-vio](https://docs.rs/rs-vio) for API docs

## Recognition

Contributors will be acknowledged in release notes and the project's contributor list. Significant contributions may lead to maintainer status.

Thank you for contributing to RS-VIO! 🚀