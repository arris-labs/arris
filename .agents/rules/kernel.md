# Kernel rules: what a library owes its callers

Rules an application never writes down. They apply to every crate; the
reasons are in `SEED.md` §9.

## Correctness

- **No panics on geometry.** Bad input, a degenerate case, an unsupported
  pair each return a typed error naming the entities. `unwrap`/`expect`/
  indexing on geometry-derived data is a bug even when it "cannot fail".
  Panics are for the kernel's own invariants, which the checker catches.
- **The checker runs after every operation in debug builds.** An `Ok` shape
  passes `arris-check`; a test that builds an invalid one says so by name.
- **Every operation returns provenance** — generated / modified / deleted
  for every entity touched. Without it the operation is unfinished.
- **Deterministic**: same input, same output and ids on every platform. No
  `HashMap`/`HashSet` iteration where order reaches a geometric decision or
  an id (`BTreeMap`, a `Vec`, or sort). No randomness outside seeded
  property tests. Transcendentals go through `arris_math::real`; clippy
  holds it (ADR-0064).
- **f64 inside**, the tessellation boundary included (ADR-0011); f32 only
  where a consumer or a format demands it, cast at that boundary.
- **Tolerances are the model's, never a literal.** A `1e-6` in an algorithm
  is a bug; use the entity's tolerance or a named, commented constant in
  `arris-math`. The hook refuses a new one (`tools/check-tolerance-literals.sh`);
  a deliberate literal says why: `// tol-literal: <reason>`.

## Testing

- **Every failure becomes a fixture**: reproduced, shrunk, committed under
  `tests/fixtures/regression/<slug>/` with the *desired* assertion and
  `#[ignore = "…"]`. The fixing commit moves it into its area with its
  blessed dump; the corpus lint holds areas to passing fixtures. Never kept
  as accepted behaviour, never deleted once fixed. Procedure: `inspect`
  skill.
- **Every fixture has an oracle**: volume, area, centroid, counts and point
  classifications from Open CASCADE (`tools/oracle/`), matched within the
  fixture's tolerance. Where the oracle is demonstrably wrong and a closed
  form is exact, `analytic.measure_differs` holds the fixture to the closed
  form instead, with ADR-0015's evidence; the oracle's values stay as the
  record.
- **Property tests over hand-picked cases**: random operands in random
  poses held to algebraic identities; a hand-picked case is a regression
  fixture, not coverage.

## API

- Doc comments say what an item guarantees; every operation has an
  example. Every crate forbids `unsafe`; a performance case for it is an
  idea, not a step.
- **Geometry enums are exhaustive on purpose**: a new kind is a breaking
  change that fails every `match` to compile, so an intersection or
  classification dispatch names every arm (no `_ =>`).
- **A break updates the binding in the same commit.** `crates/arris-py`
  maps every error enum exhaustively, so a new variant needs its Python
  class, attributes and `python/arris/_arris.pyi` line; a changed bound
  signature changes its method, stub and docstring example. The hook
  compiles the crate; `tools/py-test.sh` runs pytest, the docstrings,
  `mypy.stubtest` and `mypy --strict` as CI's `python` job does (a slice:
  `tools/py-test.sh tests/test_mesh.py`), in one shared environment.
- **A public type or signature change is a design delta** in the plan and is
  named in the commit body. Allowed pre-1.0, never silent.

## References

- The reference trees (Open CASCADE, truck, monstertruck, Fornjot) are
  read for algorithms and mistakes and reimplemented for our
  representation: no copied code, comments or identifiers, no
  transliteration. An ADR informed by one names the module read.
- Open CASCADE is **run** as the test oracle through Python, never a build
  or runtime dependency.
