# Macro Improvement Plan

## Goals
- Reduce boilerplate for error/reporting paths
- Add lightweight instrumentation without runtime cost in release builds
- Harden config validation and IO surfaces

## Scope (Current Pass)
- Introduce reusable RAII guard and utility macros
- Keep changes source-compatible; no call-site rewrites yet

## Status
- [x] Add RAII guard helper (`Defer`) for scope-exit actions
- [x] Add error/validation helpers (`bail!`, `ok_or_bail!`, `some_or_bail!`, `clamp_or!`, `cfg_warn!`, `instrument_io!`)
- [x] Add diagnostics helpers (`span!`, `debug_assert_approx!`)
- [x] Add graceful fallback helpers (`unwrap_or_log!`, `ok_or_log!`, `match_some!`)
- [ ] Wire macros into hot paths (estimator/optimization/config) — next pass

## Next Steps
1. ✅ Replace ad-hoc range checks with `clamp_or!` in config validators (loop closure, marginalization).
2. ✅ Wrap keyframe creation/solver path with `span!` (motion tracking + bundle adjust) in estimator.
3. ✅ Use `debug_assert_approx!` in IMU preintegration sanity checks (delta_rot norm).
4. In progress: convert silent defaults to `unwrap_or_log!`/`ok_or_log!` (started in State::inverse for T_W_B inversion fallback).

## Notes
- Macros live in [src/common/macros.rs](src/common/macros.rs).
- Designed to be zero-cost in release for debug-only assertions and spans.
- Error macros return `String` errors to match existing error style.
