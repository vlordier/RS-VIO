# Safety in RS-VIO

## 1. Zero-Unsafe Code Guarantee

RS-VIO enforces `unsafe_code = "forbid"` in `Cargo.toml`. Any `unsafe` block will fail compilation.

## 2. Compile-Time Safety Lints

All lints are configured in `[lints.rust]` and `[lints.clippy]` in `Cargo.toml`.

**Deny-level (compilation fails on violation):**

| Lint | Rationale |
|------|-----------|
| `unnecessary_unwrap` | Prevent redundant unwrap patterns |
| `todo` | No unresolved TODOs in committed code |
| `unimplemented` | No stub implementations |
| `exit` | Never call `process::exit` |
| `dbg_macro` | No debug macros left in code |
| `inefficient_to_string` | Avoid unnecessary allocations |
| `clone_on_copy` | Use Copy semantics where applicable |
| `implicit_clone` | Make clones explicit |
| `large_stack_arrays` | Prevent stack pressure from large arrays |
| `large_types_passed_by_value` | Pass large types by reference |
| `box_collection` | No `Box<Vec<T>>` double indirection |
| `rc_buffer` | No `Rc<Vec<T>>` patterns |

**Warn-level (flagged but not yet enforced as errors):**

| Lint | Note |
|------|------|
| `expect_used` | Temporarily warn; goal is deny |
| `unwrap_used` | Temporarily warn; goal is deny |
| `missing_const_for_fn` | Incremental const-correctness |
| `let_unit_value` | Style lint |
| `float_cmp` | VIO math requires careful float handling |

**Rust-level lints:**
- `unsafe_code = "forbid"` — absolute; no unsafe code permitted
- `non_snake_case = "allow"` — needed for transformation matrix naming (`T_A_B`)
- `warnings = "deny"` is commented out due to existing codebase issues

## 3. Memory Safety

- **Rust ownership system** — compile-time borrow checking, no GC
- **nalgebra** — bounds-checked linear algebra
- **image crate** — safe image manipulation

No `unsafe` code exists anywhere in this crate.

## 4. Build Profiles

Three profiles are defined in `Cargo.toml`:

| Profile | `opt-level` | `lto` | `debug` | `incremental` |
|---------|-------------|-------|---------|----------------|
| `dev` | 0 | — | true | true |
| `release` | 3 | thin | false | false |
| `bench` | inherits `release` | thin | true | false |

There are no `embedded-safe`, `safety-critical`, or `ultra-critical` profiles.

## 5. Dependency Security

Dependencies are audited with `cargo audit`. The `deny.toml` file configures
`cargo-deny` for license and advisory checks.

## 6. Testing

The codebase includes unit tests, integration tests (in `tests/`), and
benchmarks (in `benches/`). See `CONTRIBUTING.md` for how to run them.

## 7. Future Work

- Promote `expect_used` and `unwrap_used` from warn to deny
- Re-enable `warnings = "deny"` once existing warnings are resolved
- Add `overflow-checks = true` to release profile
- Add `panic = "abort"` to release profile
- Investigate `cargo-geiger` for transitive unsafe auditing

---

**Last Updated**: February 2026
