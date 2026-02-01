# Dependency & CI Maintenance Strategy

This document describes how RS-VIO keeps its dependencies, CI tools, and pre-commit hooks up-to-date monthly.

## Overview

The project uses **three complementary systems** to ensure all components stay current:

1. **Dependabot** - Automated dependency updates
2. **Monthly Maintenance Workflow** - Scheduled updates for pre-commit hooks and GitHub Actions
3. **Manual Code Review** - Human review of all automated PRs

## 1. Dependabot Configuration

**File:** [`.github/dependabot.yml`](.github/dependabot.yml)

### What it does:
- Monitors Cargo dependencies (Rust packages)
- Monitors GitHub Actions versions
- Creates PRs automatically on a monthly schedule
- Runs on the first Monday of each month at 9 AM UTC

### How it works:
1. Dependabot checks for new versions
2. Creates separate PRs for each package/action that needs updating
3. PRs are labeled with `dependencies` and `rust`/`github-actions`
4. Human review ensures breaking changes are handled properly

### Behavior:
- Limits to 5 open PRs at a time to avoid noise
- Uses `auto` rebase strategy to stay up-to-date
- Commits are prefixed with `chore(deps):` or `chore(actions):`

## 2. Monthly Maintenance Workflow

**File:** [`.github/workflows/monthly-update.yml`](.github/workflows/monthly-update.yml)

### What it does:
Runs three jobs monthly to update:
1. **Pre-commit hooks** - Updates all hook versions
2. **Cargo dependencies** - Aggressive dependency update
3. **GitHub Actions** - Updates action versions

### Schedule:
- **When:** First Monday of each month at 9 AM UTC
- **Can also trigger manually:** Via workflow_dispatch button in GitHub Actions UI

### How each job works:

#### Pre-Commit Update Job
```bash
pre-commit autoupdate
```
- Updates all hooks in `.pre-commit-config.yaml` to latest stable versions
- Creates a PR if changes are found
- Includes testing checklist in PR description

#### Cargo Update Job
```bash
cargo update --aggressive
```
- Updates dependencies to latest patch and minor versions
- Keeps major versions from breaking (respects semver constraints in Cargo.toml)
- Creates a PR if `Cargo.lock` changes

#### GitHub Actions Update Job
- Updates action versions to latest stable (e.g., `@v4` → `@v5`)
- Creates a PR if workflows changed

## 3. Manual Maintenance

For updates outside the monthly schedule:

### Update pre-commit hooks immediately:
```bash
pre-commit autoupdate
git add .pre-commit-config.yaml
git commit -m "chore: Update pre-commit hooks"
git push origin develop
```

### Update Cargo dependencies immediately:
```bash
cargo update --aggressive
git add Cargo.lock
git commit -m "chore: Update dependencies"
git push origin develop
```

### Run full pre-commit locally:
```bash
pre-commit run --all-files
```

## Workflow Integration

The monthly updates integrate with the CI pipeline:

```
Monthly Schedule (1st Monday @ 9 AM UTC)
    ↓
Dependabot runs & creates PRs
    ↓
Monthly-Update workflow runs & creates PRs
    ↓
PRs trigger CI pipeline
    ↓
Pre-commit hooks run (checks formatting, linting)
    ↓
Clippy runs (Rust linting)
    ↓
Tests run (unit + integration)
    ↓
Build verification
    ↓
Manual review & merge
```

## Review Checklist for Update PRs

When reviewing monthly update PRs:

### Pre-Commit Updates
- [ ] Run `pre-commit run --all-files` locally
- [ ] Check hook release notes for breaking changes
- [ ] Verify all files pass formatting/linting
- [ ] Ensure no new false positives in linting

### Cargo Updates
- [ ] Check for breaking changes in dependency release notes
- [ ] Run `cargo test --all` locally
- [ ] Verify performance hasn't degraded
- [ ] Check GitHub Actions CI passes
- [ ] Look for security advisories in updated versions

### GitHub Actions Updates
- [ ] Check action release notes for breaking changes
- [ ] Verify workflows run successfully
- [ ] Test edge cases if action behavior changed

## Configuration Details

### Pre-Commit Config
- Python 3.11 for running pre-commit
- Local Rust tools: rustfmt, clippy, cargo-sort
- External tools: typos, trailing-whitespace, etc.

### Dependabot Config
- Cargo: Updates patch & minor versions
- GitHub Actions: Updates to latest stable versions
- Monthly schedule on first Monday at 9 AM UTC
- Maximum 5 open PRs to avoid overwhelming reviews

### Monthly Update Cron
```
0 9 1-7 * 1
│ │ │   │ └─ Monday
│ │ │   └──── All months
│ │ └──────── Days 1-7
│ └────────── 09:00 UTC
└──────────── Minute 0
```

This runs on the first Monday of every month.

## Troubleshooting

### Pre-commit updates fail
- Check if hook repos have moved or changed
- Manually check `.pre-commit-config.yaml` syntax
- Test with `pre-commit run --all-files`

### Cargo updates break tests
- Check dependency release notes for breaking changes
- May need to update code to use new API
- Consider pinning specific version if breaking change
- Create issue to track incompatibility

### GitHub Actions fail after update
- Check action release notes in PR
- Some actions change their input/output signatures
- Review GitHub Actions documentation

## Best Practices

1. **Review promptly** - Don't let dependency PRs pile up
2. **Test locally** - Run full test suite before merging
3. **Check security** - Use GitHub's security vulnerability scanning
4. **Read changelogs** - Don't just auto-merge without checking release notes
5. **Batch minor updates** - Merge stable pre-commit/action updates quickly
6. **Be cautious with major updates** - Spend more time reviewing major version changes

## CI Status

All update PRs must pass:
- ✅ `cargo fmt --all -- --check` (formatting)
- ✅ `cargo clippy --all-targets -- -D warnings` (linting)
- ✅ `cargo build --verbose` (compilation)
- ✅ `cargo test --verbose` (tests)
- ✅ `pre-commit run --all-files` (hook checks)

## Related Files

- [CI Workflow](.github/workflows/ci.yml) - Runs on every push/PR
- [Monthly Update Workflow](.github/workflows/monthly-update.yml) - Monthly scheduled updates
- [Dependabot Config](.github/dependabot.yml) - Dependency monitoring
- [Pre-commit Config](.pre-commit-config.yaml) - Hook definitions
- [Cargo.toml](Cargo.toml) - Rust dependencies
- [Cargo.lock](Cargo.lock) - Locked dependency versions
