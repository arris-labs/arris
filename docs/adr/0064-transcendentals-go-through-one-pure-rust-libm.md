# ADR-0064 — Transcendentals go through one pure Rust `libm`: `arris_math::real`, banned in `std` form by clippy, held by a determinism corpus

- Status: accepted (2026-10-16)
- Plan: `libm-everywhere` step 2 (the module, the pin, the ban); the
  migration is steps 3 to 7, the corpus step 1 and step 8
- Amends: ADR-0011 ("`f64` inside" becomes "`f64` and one `libm` inside")
- Follows: `SEED.md` §9 (deterministic ids), `.agents/rules/kernel.md`
  §Correctness
- Reference trees read: none

## Context

The kernel promises the same input gives the same body, bytes and ids,
on every platform. `f64::sin` and its kin are not part of that promise:
natively they are glibc's, on `wasm32-wasip1` wasi-libc's (musl), on
`wasm32-unknown-unknown` Rust's `libm` port, and these disagree in the
last place for some inputs. The first consumer's backlog report
(`arc-sweep-differs-on-wasm32`) measured it: of the 200 three-point arcs
of a spur gear's outline, `atan2` in `arc_about` differs on 44 between
glibc and the port. The arc's sweep is the circle's parameter interval in
the body, so the bytes differ, and so do the volume and area. Everything
else the consumer tested (holes, fillets, booleans, STEP, mass
properties) matched to the bit: the arithmetic of IEEE 754 is the same
everywhere, the transcendental functions are not.

## Decision

1. **One implementation.** Every transcendental `f64` function the
   workspace calls is the `libm` crate's, pure Rust, through
   `arris_math::real` (`sin`, `cos`, `sin_cos`, `tan`, `asin`, `acos`,
   `atan`, `atan2`, the hyperbolics, `exp`, `ln`, `log2`, `log10`, `cbrt`,
   `hypot`, `powf`). `libm` is pinned with `=` in `[workspace.dependencies]`:
   a release that changes one last bit changes every digest.
2. **`powi` is ours**: repeated squaring in a fixed order, because
   `f64::powi` is a compiler intrinsic whose expansion is the backend's.
   `real::powi(x, 2)` is `x * x` bit for bit.
3. **Not wrapped**: `sqrt`, `mul_add`, `abs`, `floor`, `ceil`, `round`,
   `trunc`, `rem_euclid`, `min`, `max`, `to_radians`, float parsing and
   printing. IEEE 754 makes each exact or correctly rounded on every
   target.
4. **Clippy holds it.** `clippy.toml` lists each `f64::` and `f32::`
   method in `disallowed-methods` with the reason; tests, benches and fuzz
   targets are covered too, since a test input computed with glibc breaks
   a golden on wasm.
5. **`nalgebra`'s `libm-force`** routes its rotations through `libm`.
6. **A determinism corpus** (`crates/arris/tests/determinism.rs`): recipes
   whose native bytes, volume and area are held to committed digests,
   natively in the gate and on `wasm32-wasip1` under Node in CI.

The module is `real`, not `det`: `det` is the local name for a
determinant in `arris-ops`' offset code.

## Consequences

- A body built on wasm32 is byte-identical to one built natively; a new
  call that would break it fails clippy, and a new operation family that
  would break it fails the corpus.
- Last-ulp moves in blessed dumps, re-blessed with `fixtures:` in each
  migration step; oracle values do not change.
- `libm` is within 1 ulp, not correctly rounded: same everywhere, not best
  possible, far below any model tolerance.
- **Cost, measured** (2026-10-16, 16 threads, 256 proptest cases, the
  workspace's `dev` profile): the `fast` nextest profile ran in 1817 s
  before the migration (the commit before `nalgebra` moved to `libm-force`)
  and 1794 s after the corpus landed, the 38 `real_*` tests in 1179 s and
  1159 s. Both are within run-to-run noise (a first pair of runs differed
  by 7% on the same code), so `libm` against glibc costs nothing a gate
  can see.

## Alternatives considered

- **`std`'s per-target libm.** The status quo, and the bug.
- **A tolerance compare in the consumer's test.** Hides the difference and
  breaks the id contract: a body that differs by an ulp can split a
  different face, and ids follow.
- **Correctly rounded math (CORE-MATH).** Better, not required: the
  property that matters is one answer, not the nearest.
- **`arris_math::libm` as the module name**, which would shadow the crate.
