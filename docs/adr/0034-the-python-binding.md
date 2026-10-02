# ADR-0034 — The Python binding: a thin layer above the facade, the forbid kept, PyPI in lockstep, handles that carry their model

- Status: accepted (2026-10-02)
- Plan: `python-binding` step 1
- Amends: ADR-0020 §2, the sentence on the first-party binding ("the
  `#![forbid(unsafe_code)]` exception a binding needs, `publish`, the wasm
  job, PyPI beside crates.io — is its own idea and its own ADR"). There is
  no exception to grant; the rest is decided here.
- Follows: ADR-0013 (the layer rule), ADR-0027 (the changelog), ADR-0030
  (`Control`)

## Context

ADR-0020 put a first-party binding for code-first and agent-driven
modelling in this repository, beside the cycles, and left five questions to
its own ADR: where it sits in the layers, whether `unsafe` is needed, what
`publish` means for it, what the wasm job does with it, and how PyPI sits
beside crates.io. C3 has closed and C5's API (roles, body bytes,
cancellation, the product tree) is what a script needs, so the questions
are answered now.

The one that could have changed the plan was the `unsafe` one: a Python
extension is usually written with `unsafe` somewhere. It was checked
rather than assumed, in a scratch crate on 2026-10-02:

- `pyo3 0.26.0` with `abi3-py310` and `extension-module`, under
  `#![forbid(unsafe_code)]` and `[lints.rust] unsafe_code = "forbid"`,
  on rustc 1.99.0.
- One `#[pyclass]` (`#[pymethods]` with `&mut self` and `#[new]`), one
  `#[pyclass(frozen, hash, eq)]` value type, a three-deep
  `create_exception!` chain (`ArrisError` ← `OpError` ← `Unsupported`),
  `py.detach` around a closure that takes a `Mutex`, and a `#[pymodule]`
  that adds all of them: **compiles.** pyo3's macros expand to code that
  needs `unsafe` internally; the result is what was observed, the build
  passes with `forbid` on.
- The built `cdylib`, copied to `name.abi3.so`, **imports in CPython 3.12**:
  the class constructs, `bump` returns, the exception chain answers
  `issubclass`, the handle hashes.
- With `pyo3` declared under `[target.'cfg(not(target_arch = "wasm32"))']`
  and the crate body behind `#![cfg(not(target_arch = "wasm32"))]`,
  `cargo build --target wasm32-unknown-unknown` **succeeds** and the crate
  is empty there. `cargo test` on the `rlib` half links.

## Decision

1. **A thin 1:1 binding over the facade**, in `crates/arris-py`, one layer
   above `arris` (layer 8 in `tools/check-layers.sh` and
   `docs/ARCHITECTURE.md` §Crates). It depends on `arris` alone and nothing
   depends on it. It binds the facade's operations in the facade's own
   vocabulary; it is not an ergonomic modelling layer (workplanes,
   selectors, fluent chains), which belongs in another repository on top of
   this one, and not a recipe interpreter. Recipes and `arris-debug` stay
   dev tooling.
2. **`#![forbid(unsafe_code)]` stays on the crate**, workspace lints
   inherited like every other. The evidence is the section above. If a
   later pyo3 upgrade makes it fail, the exception is confined to this
   crate, its own ADR amends this one, and the plan or backlog line that
   hit it stops for the human; no `unsafe` is hand-written either way.
   pyo3 is pinned to the minor (`0.26`) in `[workspace.dependencies]` and
   upgraded as a plan step that re-runs this check.
3. **PyPI as `arris`, in lockstep from the same `v*` tag.** The crate is
   `publish = false` on crates.io (as `arris-debug`), so
   `cargo publish --workspace`, `cargo package --workspace` and
   cargo-semver-checks skip it. The package version is the workspace
   version (`0.5.0-dev` is PEP 440 `0.5.0.dev0`); `release.yml` gains a
   `pypi` job behind its own environment reviewer, using trusted
   publishing, that refuses a pre-release as the crates job does. The first
   upload is the human's act: the name reservation and the trusted-publisher
   registration are theirs.
4. **Python tests run in CI only**: pytest over a `maturin develop` build in
   a `uv` venv. Never in the pre-commit hook (ADR-0032); the hook's Rust
   checks still compile the crate, so a Rust API break fails locally.
5. **The mesh crosses as `bytes`** (little-endian `f64` positions and
   `u32` indices, counts alongside) with a pure-Python `Mesh.to_numpy()`
   that imports `numpy` only when called. No `rust-numpy`, no buffer
   protocol, no PNG in the wheel: an agent gets the mesh and renders it
   with its own tools.
6. **A handle carries its model.** `Model` is a Python class owning an
   `arris::topo::Model` behind a lock (so a long operation can release the
   GIL); `Body`, `Shell`, `Face`, `Edge` and `Vertex` are frozen, hashable
   classes holding a reference to their model plus the slot and
   generation. An id from one model passed to another raises
   `ForeignHandleError` before it reaches the kernel. This closes
   `docs/ARCHITECTURE.md` §The model's "ids have no model identity" for
   Python callers with **no kernel change**: the kernel's ids stay plain
   `(slot, generation)` and a Rust caller keeps the discipline it has.
7. **The wasm gate.** pyo3 does not build for `wasm32-unknown-unknown`, so
   the dependency is gated off it and the crate is empty there; CI's
   `cargo build --workspace --target wasm32-unknown-unknown` is unchanged.
   `crate-type = ["cdylib", "rlib"]`: the `rlib` keeps `cargo test` and
   the hook's clippy and nextest runs working.
8. **Errors are typed exceptions.** A common `ArrisError`, one subclass per
   variant of every kernel error enum the binding can raise, built by an
   exhaustive `match` per enum with no wildcard arm. A variant added in the
   kernel therefore fails the binding's compile, which is the kernel rule's
   intent (`.agents/rules/kernel.md` §API); the same commit updates the
   binding. Each enum has a family class (`OpError`, `GeomError`, …) under
   `ArrisError` and each variant a class under its family, named by the
   family's prefix (`OpUnsupportedError`). Two conditions recur in several
   enums and have one class each, so a caller catches them once: every
   `NotFound` is `StaleHandleError` and every `Interrupted` is `Interrupted`
   (directly under `ArrisError`). A nested kernel error with no structure a
   caller would branch on is the exception's `detail` text; entities are
   handles of the model the call was made on.

## Consequences

- A new crate and a new layer; `tools/check-layers.sh` and the architecture
  table grow a row. Nothing below changes: no public type or signature of a
  published crate is touched.
- Every kernel API break now also edits `crates/arris-py` in the same
  commit (`.agents/rules/kernel.md` §API, written by the plan's last step).
- The release has a second half, PyPI, the human approves through its own
  environment, beside the crates.io one (`.agents/rules/git.md` §Tags).
- A pyo3 upgrade is a deliberate step, never a drive-by `cargo update`.
- The Linux wheel is built and tested in CI; which platform wheels PyPI
  carries at a release is a release-time decision, not this one's.

## Alternatives considered

- **A binding in a separate repository.** Already rejected in ADR-0020 §2:
  it lags every pre-1.0 break by a release.
- **Lift `forbid` to `deny` for the crate, or allow `unsafe`.** Not needed:
  the check above holds. Granting an exception nobody needs would remove
  the property for the one crate most likely to grow `unsafe` later.
- **The facade's `Model` re-exported as the Python class.** It would leave
  ids without model identity, so a handle from one model could resolve to a
  different real entity in another, silently. Rejected for decision 6.
- **A kernel-side model identity** (a model id inside every id). Fixes it
  for Rust callers too, at the price of a public-type change in every
  crate and the body-bytes format (ADR-0029). Not taken for a binding's
  sake; the binding's own handle carries it.
- **A recipe or JSON surface as the agent's way in.** A script already
  has a language; a second grammar in the wheel is a second thing to keep in
  step with the kernel.
- **`rust-numpy` or the buffer protocol for the mesh.** A dependency (or
  `unsafe`) for what `bytes` plus a three-line shim already gives.
