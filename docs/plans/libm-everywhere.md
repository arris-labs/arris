# Plan: libm-everywhere

- Started: 2026-10-16
- Milestone: beside C10 (docs/ROADMAP.md §C10): a consumer's bug, not a
  cycle item
- Idea: none; verbatim from the backlog's measured block
  `arc-sweep-differs-on-wasm32` (docs/BACKLOG.md), found by the plugin CAD:
  "call one implementation everywhere. Transcendental functions go through
  `libm` … Then the dump-twice determinism tests run on wasm32 in CI."

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal

Every transcendental `f64` function the workspace calls (`sin`, `cos`,
`sin_cos`, `tan`, `asin`, `acos`, `atan`, `atan2`, `sinh`, `cosh`, `exp`,
`ln`, `log2`, `cbrt`, `hypot`, `powf`, `powi`, and their kin) runs one pure
Rust implementation, the `libm` crate, through `arris_math::real`, on every
target. Nalgebra's own rotations are routed there too (`libm-force`).
Clippy bans the `std` methods workspace-wide, so a new call cannot slip back.
A determinism corpus, a gear outline of 40 lines and 200 three-point arcs
extruded plus one recipe per operation family, holds each body's
dump digest to a committed golden. It runs natively in the gate and on
`wasm32-wasip1` under Node in CI, and the two produce the same bytes.

## Non-goals

- **Other wasm targets in CI** (`wasm32-unknown-unknown` under a
  wasm-bindgen runner): `wasip1` exercises the same `libm` code; one
  backlog line if a consumer reports a difference there.
- **Correctly rounded math** (CORE-MATH or the like): `libm` is within
  1 ulp, far below any model tolerance. Same everywhere matters, best
  possible does not.
- **Performance**: `libm` against glibc is measured once (step 9) and
  recorded. A gap gets a backlog line, not a fix here.
- **`aarch64` and macOS runners**: those platforms are covered by the
  same argument (one implementation), not by a job.

## Design deltas

- **`arris_math::real`** (new public module): free functions mirroring
  `libm`'s signatures, `real::atan2(y, x)`, `real::sin_cos(x) -> (sin, cos)`,
  `real::hypot(x, y)`, …, each `#[inline]` over `libm`. One exception is
  `real::powi(x, n)`, which is repeated squaring in a fixed order, so
  `powi(x, 2)` stays `x * x` bit for bit. Rustdoc states the guarantee
  (same bits on every target) and why it exists. `libm` joins
  `[workspace.dependencies]`, pinned with `=`, because a `libm` release
  that changes a last bit changes every golden.
- **Not wrapped**, because IEEE 754 already makes them exact or correctly
  rounded on every target: `sqrt`, `mul_add`, `abs`, `floor`, `ceil`,
  `round`, `trunc`, `rem_euclid`, `min`, `max`, `to_radians`, and float
  parse and format.
- **`clippy.toml`** (new, at the root): `disallowed-methods` lists each
  `f64::` and `f32::` transcendental with the reason "use
  `arris_math::real` (ADR-0064)". It covers tests, benches and fuzz targets
  too: a test input computed with glibc's `sin` would break the wasm
  golden. While the migration runs, `[workspace.lints.clippy]
  disallowed_methods = "allow"` in the root manifest stands in (a
  crate-level allow would not cover a crate's `tests/` and `benches/`); each
  step checks its crates with `cargo clippy -p <crate> --all-targets -- -D
  clippy::disallowed_methods`, and step 7 deletes the table.
- **`nalgebra`** in `[workspace.dependencies]` gains `features =
  ["libm-force"]`: `Rotation3::from_axis_angle`, `.angle()`,
  `from_euler_angles` and `rotation_between` go through simba's `libm`
  instead of `std`.
- **Determinism corpus**: `crates/arris/tests/determinism.rs` builds each
  recipe and records the SHA-256 of its native-format bytes
  (`arris_io::body::write`, lossless; `dump_text` rounds to 11 digits
  and hides an ulp) and its volume and area printed shortest-round-trip.
  It compares them against `crates/arris/tests/determinism/digests.json`,
  kept out of `tests/fixtures/` because the corpus lint wants a recipe in
  every directory there. The file is embedded with `include_str!`, so the
  wasm run needs no filesystem. Recipe inputs are computed with `libm`
  (an `arris` dev dependency from step 1). Recipes: the gear
  extrude, a revolve, a cylinder–cylinder boolean, a fillet, an analytic
  sweep and a NURBS sweep, a STEP read of one committed `real/` part, and
  a tessellation. `ARRIS_BLESS=1` rewrites the digests natively, never on
  wasm.
- **Wasm runner**: `tools/wasi-run.mjs`, a dozen lines over `node:wasi`,
  serves as `CARGO_TARGET_WASM32_WASIP1_RUNNER`, so `cargo test --target
  wasm32-wasip1 -p arris --test determinism` runs the corpus. CI's `wasm`
  job gains that step.
- **ADR-0064**: transcendentals go through one pure Rust `libm`. It
  explains why (glibc against the Rust `libm` port, 44 of 200 arcs
  differ), records the rejected options (std's per-target libm; a
  tolerance compare instead of bit equality, which breaks the id
  contract), and sets the rule for new calls. It amends ADR-0011's
  "f64 inside" with "and one libm".
- **No public signature changes.** `real` is additive. Every blessed dump
  whose last digits move is re-blessed in its migration step, with
  `fixtures:` in the message saying it is a last-ulp move from the libm
  switch; oracle values do not change.
- **Changelog** (Unreleased, fix): a body built on wasm32 is now
  byte-identical to one built natively.

## Steps

**[1]** routine · **[2]** careful · **[3]** unproven.

- [x] Step 1 **[3]** — The harness, run before any fix. Add
  `tools/wasi-run.mjs`, the determinism test with the gear recipe only, and
  its native digest. Run it on `wasm32-wasip1` locally and see it fail,
  then commit with the wasm case `#[ignore = "arc-sweep-differs-on-wasm32"]`
  and that failure written in the body. This retires the unknowns: does
  `arris-debug` build for wasip1, and does Node's WASI run libtest?
  Answered: both work. On wasip1, std's math is wasi-libc's (musl), not
  Rust's `libm` port, and it still differs from glibc.
- [x] Step 2 **[1]** — `arris_math::real` with rustdoc and an example, the
  `libm` pin, ADR-0064, `clippy.toml`, and the temporary workspace allow. Add the
  backlog's single call as a unit test: `real::atan2(1.9422128270990565e-6,
  1.271166962267458e-5)` has the same bits on every target.
- [x] Step 3 **[2]** — `nalgebra` `libm-force`; migrate `arris-math` (6)
  and `arris-mesh` (5) and pass the per-crate clippy check; re-bless what moves.
- [x] Step 4 **[2]** — `arris-geom`, the curves (`profile.rs` first, where
  the report's `arc_about` is) and the 2-D code: about half of its 402
  calls. Re-bless.
- [x] Step 5 **[2]** — `arris-geom`, the surfaces and intersections, the
  rest of the 402; pass its clippy check. Re-bless.
- [x] Step 6 **[1]** — `arris-ops` (32) and `arris-io` (25); pass the clippy check;
  re-bless.
- [x] Step 7 **[1]** — `arris-debug` (75), `arris-py` (2), and the ~550
  calls in tests, benches and fuzz targets; delete the workspace allow table. Clippy
  is now the guard.
- [x] Step 8 **[2]** — Grow the determinism corpus to the full recipe list
  and bless it natively. Un-ignore the wasm case: it passes on wasip1
  locally. Add the `wasm` job's test step to CI.
- [ ] Step 9 **[1]** — Time the fast nextest profile and the `real_*`
  corpus before step 3 and after step 8; record both in ADR-0064. A
  slowdown above 10% gets a backlog line.

## Acceptance

`cargo test --target wasm32-wasip1 -p arris --test determinism`, with
the Node runner, matches `digests.json` byte for byte, including the
backlog's gear. The full gate passes with the `clippy.toml` ban and no
`disallowed_methods` allow left (`grep -r disallowed_methods Cargo.toml crates`
finds only `clippy.toml`). Oracle values are unchanged; every re-blessed
dump differs only in last digits.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Threading and wasm and the f64 bullet (ADR-0011
  line): one `libm`, `arris_math::real`, the determinism corpus.
- `.agents/rules/kernel.md` §Correctness, the Deterministic bullet:
  "transcendentals through `arris_math::real`; clippy holds it".
- `.agents/rules/git.md` full-gate paragraph: CI's `wasm` job now runs
  the determinism corpus.
- `docs/BACKLOG.md`: delete the `arc-sweep-differs-on-wasm32` block, and
  the slowdown line if step 9 wrote one.
- `CHANGELOG.md` Unreleased: the fix bullet above.
- `GLOSSARY.md`: **determinism corpus**, the recipes whose dump digests
  are held equal on every target.
- `AGENTS.md` current state: one clause, once retired.

## Open questions

- Settled in step 2: the module is `real`. `det` collides with the local
  `det` of `arris-ops`' offset code, as feared (ADR-0064).
