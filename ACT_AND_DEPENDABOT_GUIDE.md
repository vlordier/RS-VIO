# Local CI Testing with act & Automated Dependency Updates

## 🚀 Act Configuration (Local GitHub Actions Testing)

### Setup
The repository is now configured for local CI testing using [nektos/act](https://github.com/nektos/act).

**Configuration file:** `.actrc`
- Uses `catthehacker/ubuntu:full-latest` image (~8GB, includes all tools)
- Platform: `linux/amd64` (compatible with macOS and Linux hosts)
- Environment: Sets `ACT=true` to skip cache actions

### Running Local CI

```bash
# List all available jobs
act -l

# Run quick check (formatting + clippy + tests)
act -j quick-check push

# Run specific workflow
act -j lint push

# Run all workflows (resource-intensive)
act push
```

### Act-Compatible Workflow

**File:** `.github/workflows/act-test.yml`

A lightweight workflow designed for act:
- ✅ Checkout code
- ✅ Install Rust toolchain
- ✅ Check formatting with rustfmt
- ✅ Run clippy linting
- ✅ Run library tests
- ⚠️ Skips cache actions when `ACT=true` (not needed locally)

**Why this workflow?**
- Standard workflows use `actions/cache@v4` which requires Node.js paths that act struggles to resolve
- This workflow conditionally skips caching when running locally with act
- Much faster feedback loop for developers

### Limitations

1. **Cache actions**: Standard workflows using `actions/cache@v4` may fail in act due to Node.js path issues
   - Solution: Use the `act-test.yml` workflow for local testing
   
2. **Artifacts**: Upload/download artifact actions also require Node.js
   - Solution: Check job output directly in terminal

3. **Large image**: The full-latest image is ~8GB
   - First run downloads the image (one-time cost)
   - Subsequent runs reuse the cached image

### Tips

- **Faster testing**: Use `act -j quick-check push` for rapid feedback
- **Debug mode**: Add `-v` flag for verbose output: `act -v -j quick-check push`
- **Selective jobs**: Run only specific jobs with `-j <job-id>`

---

## 🔄 Automated Dependency Updates with Dependabot

### Configuration

**File:** `.github/dependabot.yml`

Dependabot automatically checks for dependency updates on these schedules:

| Ecosystem       | Frequency | Day    | Time (UTC) | Open PRs Limit |
|-----------------|-----------|--------|------------|----------------|
| Cargo (Rust)    | Weekly    | Monday | 03:00      | 10             |
| GitHub Actions  | Monthly   | Monday | 03:00      | 5              |
| Docker          | Monthly   | Monday | 03:00      | 3              |

### Features

1. **Grouped Updates**
   - Minor and patch updates are grouped together to reduce PR noise
   - Major updates create separate PRs for careful review

2. **Auto-labeling**
   - `dependencies` label on all PRs
   - `rust` / `ci` / `docker` specific labels
   - Easy filtering and tracking

3. **Commit Message Format**
   - `chore(deps): update <package>` for Cargo
   - `chore(ci): update <action>` for GitHub Actions
   - `chore(docker): update <image>` for Docker
   - Follows conventional commits

4. **Automatic Reviewers**
   - All PRs request review from `@charleshamesse`

### Auto-Merge Workflow

**File:** `.github/workflows/dependabot-auto-merge.yml`

Automatically handles Dependabot PRs:

- ✅ **Auto-approve** minor and patch updates (safe, backward-compatible)
- ✅ **Enable auto-merge** for approved minor/patch updates
- ⚠️ **Comment on major updates** requiring manual review
- 🔒 **Runs CI first** - auto-merge only triggers after all checks pass

**Security**: Only processes PRs from `dependabot[bot]`, preventing unauthorized auto-merges.

### Manual Override

To disable auto-merge for a specific PR:
```bash
gh pr ready --undo <PR-number>
```

To manually merge a Dependabot PR:
```bash
gh pr merge <PR-number> --auto --squash
```

---

## 📊 Workflow Summary

### Current Workflows

1. **Rust CI** (`rust.yml`)
   - Triggered on push/PR
   - Comprehensive testing and analysis
   - Security audits with cargo-audit, cargo-deny

2. **Quality Pipeline** (`quality.yml`)
   - Multi-stage quality checks
   - Linting, formatting, security, documentation
   - Advanced testing (fuzzing, mutation testing)
   - Weekly security audit (Sunday 00:00 UTC)

3. **Documentation** (`docs.yml`)
   - Generate and validate docs
   - Deploy to GitHub Pages on push

4. **Act Test** (`act-test.yml`) ⭐ NEW
   - Lightweight local CI testing
   - Act-compatible (skips cache actions)

5. **Dependabot Auto-Merge** (`dependabot-auto-merge.yml`) ⭐ NEW
   - Auto-approve safe dependency updates
   - Comment on major version bumps

### Recommended Development Workflow

1. **Before committing:**
   ```bash
   cargo fmt --all
   cargo clippy --all-targets
   cargo test --lib
   ```

2. **Local CI validation:**
   ```bash
   act -j quick-check push
   ```

3. **Push to GitHub:**
   ```bash
   git push origin <branch>
   ```

4. **Monitor CI:**
   - GitHub Actions runs full test suite
   - Dependabot creates PRs for outdated dependencies
   - Auto-merge handles safe updates automatically

---

## 🛠️ Maintenance

### Weekly Automated Tasks
- **Sunday 00:00 UTC**: Security audit via `quality.yml` schedule
- **Monday 03:00 UTC**: Dependabot checks for Cargo updates

### Monthly Automated Tasks
- **First Monday 03:00 UTC**: Dependabot checks GitHub Actions and Docker updates

### Manual Tasks (as needed)
- Review major version Dependabot PRs
- Update MSRV when required by dependencies
- Monitor CI failures and adjust workflows

---

## 🔧 Troubleshooting

### Act Issues

**Problem:** `node: executable file not found in $PATH`
- **Cause:** Cache/artifact actions require Node.js
- **Solution:** Use `act-test.yml` workflow instead

**Problem:** `Cannot parse container options`
- **Cause:** Malformed `.actrc` configuration
- **Solution:** Check `.actrc` syntax, especially `--container-options` format

**Problem:** Act times out or hangs
- **Cause:** Large image download or resource-intensive job
- **Solution:** 
  - First run: Wait for image download to complete
  - Use `-j quick-check` for faster feedback
  - Check Docker resource limits

**Problem:** Build fails with `error: usage of an 'unsafe' block` or `forbid unsafe_code`
- **Cause:** `Cargo.toml` has `unsafe_code = "forbid"` but code uses SIMD intrinsics
- **Solution:**
   - Set `unsafe_code = "warn"` in `[lints.rust]` section of `Cargo.toml`
   - Add `#![allow(unsafe_code)]` at the top of modules needing SIMD
   - Document safety invariants with `// SAFETY:` comments
   - Note: `warnings = "deny"` in Cargo.toml escalates all warnings to errors

### Dependabot Issues

**Problem:** PRs not created
- **Check:** Dependabot logs in Settings → Security → Dependabot
- **Verify:** `.github/dependabot.yml` syntax with [Dependabot Validator](https://github.com/dependabot/dependabot-core)

**Problem:** Auto-merge not working
- **Check:** CI must pass first (auto-merge is conditional)
- **Verify:** PR is from `dependabot[bot]`
- **Check:** Update type is minor/patch (not major)

**Problem:** Too many PRs
- **Adjust:** Lower `open-pull-requests-limit` in `.github/dependabot.yml`
- **Group:** Increase grouping patterns

---

## 📚 References

- [nektos/act Documentation](https://github.com/nektos/act)
- [Dependabot Configuration](https://docs.github.com/en/code-security/dependabot/dependabot-version-updates/configuration-options-for-the-dependabot.yml-file)
- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [Catthehacker Act Images](https://github.com/catthehacker/docker_images)
