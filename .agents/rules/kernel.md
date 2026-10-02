# Kernel rules: what a library owes its callers

These are the rules a geometric kernel needs that an application never wrote
down. They apply to every crate; `SEED.md` §9 holds the reasons.

## Correctness

- **No panics on geometry.** Bad input, a degenerate case, an unsupported
  surface pair — each returns a typed error naming the entities involved.
  `unwrap`/`expect`/indexing on data derived from geometry is a bug even
  when it "cannot fail". Panics are reserved for programming errors on the
  kernel's own invariants, and the checker is where those are caught.
- **The checker runs after every operation in debug builds.** An operation
  that returns `Ok` returns a shape that passes `arris-check`. A test that
  needs to construct an invalid shape says so by name.
- **Every operation returns provenance.** Generated / modified / deleted
  for every entity it touched. An operation without a provenance record
  is unfinished, not "simplified".
- **Deterministic.** Same input, same output, same entity ids, on every
  platform. No iteration over `HashMap`/`HashSet` where the order reaches a
  geometric decision or an id — use `BTreeMap`, a `Vec`, or sort first. No
  randomness outside property tests, and those are seeded.
- **f64 everywhere inside; f32 only where a consumer asks for it, or a
  format requires it.** The tessellation boundary is `f64` too
  (ADR-0011), and a renderer's cast belongs at its own boundary. Binary
  STL's facets are `f32` because the format says so, at the writer, not
  because anything the kernel computes with is.
- **Tolerances are the model's, never a literal.** A `1e-6` in an algorithm
  is a bug; the entity's tolerance, or a named constant in `arris-math`
  with a comment saying why, is the fix.

## Testing

- **Every failure becomes a fixture.** Reproduce, shrink to the smallest
  case that still fails, commit it under `tests/fixtures/regression/<slug>/`
  with the *desired* assertion, `#[ignore = "…"]`d until it passes. The
  commit that fixes it moves it into its area (`boolean/`, `sweep/`, …)
  with its blessed dump: the corpus lint holds those areas to passing
  fixtures only. The failure is never kept as accepted behaviour, and
  never deleted once fixed.
- **Every fixture has an oracle.** Volume, area, centroid, counts, and
  point classifications computed by Open CASCADE (`tools/oracle/`), stored
  beside the fixture. Arris must match within the fixture's stated
  tolerance. Where the oracle is demonstrably wrong and a closed form is
  exact, the fixture says so in `analytic.measure_differs` and is held to
  the closed forms instead, with the evidence ADR-0015 asks for; the
  oracle's values stay beside it as the record.
- **Acceptance is a corpus run, not a demo.** A milestone closes on
  numbers: checker green, oracle matched, property tests green.
- **Property tests over hand-picked cases.** Random operands in random
  poses, algebraic identities (volume additivity, cut-then-fuse,
  commutativity, STEP round-trip). A hand-picked case is a regression
  fixture, not coverage. A property may be split across shards
  (`prop_shards!`) so the machine runs them at once; each shard is seeded
  from the base seed and its own index, and the shards together run the
  configured case count and never fewer, so "those are seeded" and the
  count both still hold of the sharded form.

## API

- **`#![warn(missing_docs)]` on every crate**, and a doc comment on every
  public item that says what it guarantees, not what it does. An example on
  every operation.
- **`#![forbid(unsafe_code)]`** on every crate. A performance case that
  wants `unsafe` is an idea, not a step.
- **Lower crates never name upper crates' types.** `math` ← `geom` ←
  `topo` ← `check` ← `ops`/`mesh` ← `io` ← `debug` ← `arris` (ADR-0013:
  `io` depends on `mesh`, so the two are no longer flat siblings). CI
  checks `cargo tree` for the pairs most tempted to break it.
- **Geometry enums are exhaustive on purpose.** A new surface or curve
  kind is a breaking change that makes every `match` fail to compile —
  that is the feature. Never add a wildcard arm to an intersection or
  classification dispatch.
- **A break updates the binding in the same commit.** `crates/arris-py`
  maps every kernel error enum with an exhaustive `match`, so a variant
  added in the kernel stops it compiling until it has its Python class, its
  attributes and its line in `python/arris/_arris.pyi`; a changed signature
  of a bound operation changes its method, stub and docstring example. The
  hook compiles the crate, so the Rust half fails locally; the `python`
  CI job (pytest, the docstring examples, `mypy.stubtest`) holds the rest.
- **A change to a public type or signature is a design delta** in the plan
  and is named in the commit body. Pre-1.0 it is allowed; it is never
  silent.

## References

- Open CASCADE, truck, monstertruck and Fornjot checkouts are **read-only
  reference trees**: read for algorithms and mistakes, reimplemented to fit
  our representation. No copied code, comments or identifiers; no
  port-by-transliteration. An ADR informed by one names the module read.
- Open CASCADE is **run** as the test oracle through Python; it is never a
  build or runtime dependency of any crate.
