# Plan: face-surface-properties

- Started: 2026-11-14
- Milestone: beside C12 (a consumer's ask ranks first, ADR-0020's
  amendment of 2026-09-26; docs/ROADMAP.md §Beside the cycles)
- Idea: docs/ideas/face-surface-properties.md (absorbed; option A, accepted
  by `/plan` on 2026-11-14)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal

A face has its own surface properties. `arris_ops::measure::
face_surface_properties(&model, face, &control)` returns the
`SurfaceProperties` (area, centroid, inertia about the centroid, unit area
density) of one face, by the same quadrature `surface_properties` takes a
body's by, so a body's figure is the sum of its faces' to rounding. Python's
`Model.surface_properties` takes a `Face` as well as a `Body`. Each surface
kind's face matches its closed form (a half cylinder's centroid 2r/π off its
axis, a frustum's wall, a hemisphere, a plane, a torus), each face of the
surface battery matches Open CASCADE's `BRepGProp::SurfaceProperties` of that
face, and the values are the same bits on every platform. A script consumer
whose CadQuery-style axis selectors rank faces by their centre of mass, and
place a workplane through it, then has the exact value instead of a
tessellation's estimate.

## Non-goals

- A selector, ranking or clustering of faces by centroid: that is the
  consumer's (the idea's constraint). Option B, a `SurfaceProperties` on
  `arris.build`'s `Selection`, is the consumer's to build on A.
- Per-face *mass* properties (no volume belongs to a face) and per-edge
  length properties: a backlog line if a consumer asks.
- The quadrature's cost (backlog: a planar face's closed form over its
  boundary); this plan reuses the integrator as it is.
- The arris-side checker run on a face: a face is not a body, so the query
  verifies what it resolves, as `query::face_frame` does.

## Design deltas

- **`arris-ops` `measure`** (public, additive): `pub fn
  face_surface_properties(m: &Model, face: Face, control: &Control<'_>) ->
  Result<SurfaceProperties, OpError>`. The body's `surface_properties` and the
  new function share one private `properties_of(m, faces, reference, named,
  meter)` (the first pass, the `NotPositive` check naming `named`, the second
  pass about the centroid, the tensor); behaviour of the body's is unchanged
  bit for bit. The face's reference point is the centre of the box of its own
  loops' vertices (origin when it has none), on `reference_point`'s
  reasoning. Orientation does not enter: a reversed face use gives the same
  value. Errors: `OpError::NotFound` for a face, surface or loop that does not
  resolve; `OpError::Degenerate` with `InputReason::NotPositive { what: "the
  area" }` naming the face for a face with no area; `Interrupted` from the
  meter.
- **Docs of the identity**: `SurfaceProperties`' rustdoc states that a body's
  is its faces' combined — areas summed, centroids area-weighted, tensors
  carried to the body's centroid by `inertia_about` and summed.
- **`arris-debug` `oracle`**: `occt_surface_faces(dir, variant)` →
  `Vec<SurfaceMeasure>`, one per face of the result, from a `--faces` mode of
  `tools/oracle/occt_surface.py` (`measure.surface_properties` of each
  `TopoDS_Face`, the same integration choice `_area` makes per face). Arris
  and Open CASCADE number faces differently, so the test pairs them one to
  one by nearest centroid and fails on a face left unpaired.
- **`arris-py`** (public, additive): `Model.surface_properties(entity: Body |
  Face, *, cancel, budget)`, through a `FromPyObject` enum; the stub line in
  `python/arris/_arris.pyi` widens to `Body | Face`; the docstring gains a
  face example. No error class changes.
- **Determinism corpus** (`crates/arris/tests/determinism.rs`): a record of
  each face's surface properties of one recipe, digested from `to_bits`, so
  the wasm job holds the per-face value to the same bits as the native run.
- No new term: "surface properties" is used as the body's.

## Steps

Grades: **[1]** routine; **[2]** careful, a geometric or numeric case to get
right within a given design; **[3]** unproven.

- [x] Step 1 **[2]** — `measure::face_surface_properties` and the shared
  `properties_of`; closed-form tests in `crates/arris-ops/tests/
  surface_properties.rs`: a box's planar faces; a cylinder's full wall
  (centroid on the axis at mid height); a half cylinder from `split` by a
  plane through the axis (centroid 2r/π from it, `I_axis = A r²`); a cone
  frustum's wall (`z̄ = h (R + 2r) / (3 (R + r))`); a hemisphere's dome from
  a sphere split at its equator (centroid r/2 above it); a torus face. Errors:
  a face id that does not resolve is `NotFound`; reversed and forward uses
  agree to the bit. The body's existing tests stay byte-identical.
- [x] Step 2 **[2]** — the sum identity as a property test: random boxes,
  cylinders and a cut of the two in random poses; for each body the faces'
  areas, area-weighted centroids and parallel-axis-carried tensors combine to
  its `surface_properties` within 1e-12 relative (area, centroid) and 1e-10
  of the tensor's scale.
- [ ] Step 3 **[2]** — Open CASCADE per face: `occt_surface.py --faces`,
  `oracle::occt_surface_faces`, and a test over the existing `BATTERY` in
  `surface_properties.rs` pairing faces by nearest centroid and holding each
  pair to the battery's tolerances (area 1e-9 relative, centroid 1e-7 √A,
  tensor 1e-7 of its scale). Needs the oracle environment (`tools/oracle`),
  as the battery's body test does.
- [ ] Step 4 **[1]** — the determinism record: per-face surface properties
  of the boolean recipe digested from `to_bits` into
  `crates/arris/tests/determinism.rs`'s expected record, re-blessed in this
  commit (`fixtures:` in the message, why: a new record, no value moved).
- [ ] Step 5 **[1]** — the binding: `Model.surface_properties` takes `Body |
  Face`; stub, docstring example (a box's top face: area and centroid);
  `crates/arris-py/tests/test_queries.py` covers a half cylinder's face
  centroid against 2r/π and a stale face handle raising as `surface()` does.
  `tools/py-test.sh tests/test_queries.py` and the stubtest pass.

## Acceptance

- `cargo nextest run -p arris-ops --test surface_properties` green: every
  closed form of step 1, the property of step 2 at 256 cases, and every face
  of the battery matching Open CASCADE's per-face surface properties (step 3).
- `cargo nextest run -p arris --test determinism` green with the per-face
  record; CI's `wasm` job green on it.
- `tools/py-test.sh` green (pytest, docstrings, `mypy.stubtest`, `mypy
  --strict`).
- The body's `surface_properties` unchanged: the existing battery test and
  every corpus dump byte-identical.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Operations, the mass-properties row (line ~2454):
  `ops::measure::face_surface_properties` → `SurfaceProperties` of one face.
- `docs/DATA-MODEL.md` §Body kinds (line ~1637): "measured by
  `measure::surface_properties` … and per face by
  `measure::face_surface_properties`".
- `docs/ROADMAP.md` §Beside the cycles: one line, the per-face surface
  properties landed beside C12 at a consumer's ask.
- `CHANGELOG.md` `Unreleased`: `face_surface_properties` in Rust and
  `Model.surface_properties(face)` in Python give one face's area, centroid
  and inertia, a body's being their sum.
- `AGENTS.md` current state: only if this is the last work before C12's first
  plan (one clause).

## Open questions

- ⚠ OPEN: the Rust name — `face_surface_properties` (a second function)
  over a `surface_properties` taking `impl Into<Entity>`, which would break the
  body signature. Agent's pick: the second function, additive and named for
  what it takes; decided, revisit only if step 1 finds a reason.
- ⚠ OPEN: a face with no vertices (a full sphere's face, if a primitive makes
  one) takes the origin as its reference, as a body without vertices does; on
  a body posed far away that costs rounding in the first moments. Agent
  decides in step 1: if the sphere test shows it beyond 1e-12 relative, use
  the centre of the face's `bounds` instead.
