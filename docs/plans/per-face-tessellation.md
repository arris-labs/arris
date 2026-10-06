# Plan: per-face-tessellation

- Started: 2026-10-06
- Milestone: beside C7 (docs/ROADMAP.md §C7, the side plan; ask A9 of
  `docs/ideas/plugin-cad-consumer-asks.md`)
- Idea (verbatim from the human): "per-face-tessellation" — the roadmap's
  line: "per-face incremental tessellation (A9), an edge's discretisation a
  pure function of the edge and the chord and `tessellate_faces` over a
  subset, so faces meshed at different times stay watertight (ADR-0010)."

## Goal

A consumer can mesh any subset of a body's faces, at any time, and stitch
the pieces into a closed mesh. `arris_mesh::tessellate_faces` meshes a
subset; `TriMesh::weld` joins meshes by their edge ranges and refuses a
shared edge whose samples differ. An edge's samples depend only on the
edge, meaning its curve, its range, its coedges' pcurves and their
surfaces, and on the chord. They no longer depend on how far a
neighbouring face's region reaches. A face kept by an operation therefore
keeps its cached mesh. Its re-meshed neighbours meet it bit for bit, and
the welded result equals the one-call mesh of the new body. The plugin CAD
can cache meshes per `FaceId` across edits, as ADR-0010's stable ids
promise. Every existing tessellation guarantee holds: every position
within the chord, the mesh closed by construction, deterministic, the same
with `parallel` on or off.

## Non-goals

- Adaptive or curvature-driven refinement beyond the chord bound (M3's
  "out" stays out). The cone rings of step 2 are a fixed lattice of
  constant-`v` lines at a fixed ratio, not error-driven refinement.
- A mesh cache inside the kernel. The cache, its keys and its eviction
  belong to the consumer. The kernel promises only the purity that makes a
  cache sound.
- Smooth shading, `f32` output, LOD (ADR-0011, ADR-0012 unchanged).
- The NURBS deviation backlog line (measuring by projection), and the
  section beside a pole (`BesideSingularity`). Both stay backlog lines.
- Meshing a face that belongs to no body, or a face of a model state other
  than the one the call reads.

## Design deltas

**Why it is not already true.** `tessellate_with`'s pass one sizes each
edge by the largest count any coedge asks for. A coedge asks for
`travel / steps`, where `steps` is its face's `Surface::chord_steps(chord,
bounds)` and `bounds` is that face's whole (u, v) box. For a plane, a
cylinder, an elliptic cylinder, a sphere or a torus the steps ignore the
box. For a **cone** (its radius at the box's `v` ends) and a **NURBS**
surface (curvature sampled over the box) they do not. So trimming a cone
or a NURBS face changes the sample count of every edge it has, including
edges that a boolean keeps by id together with the untouched face on
their other side. A second, unproven dependence: whether an operation that
modifies a face keeps its kept edges' pcurves bit for bit (step 1).

**ADR-0052, "an edge's samples are a pure function of the edge"** (step
2). It closes no `⚠ OPEN:`.
- A coedge's requirement is read from the pcurve's own (u, v) box over the
  edge's range, not from the face's box. For a cone, that means the radii
  the edge actually reaches. For a NURBS surface it means the curvature
  over the surface's whole domain. That is conservative, since a face's
  steps are never finer than the domain's, and it is a function of the
  surface alone. A degenerate edge keeps its face's count, per coedge:
  its range is one index, so the count never reaches a neighbour, and
  its (u, v) row is the face's own.
- **Rings on a cone face.** Under a per-edge rule, a frustum's
  small-radius chain is sampled for its own radius. A cone face whose
  radii span more than `RING_RATIO = 2` takes interior rings:
  constant-`v` lines, radii halving down from the widest, each sampled
  at its own radius's step. These are Steiner points like the sphere's
  lattice, face-local, so purity is untouched; a cone reaching its apex
  takes none. *(Step 2 corrected the reason: the estimate
  `δ·max_t (1 + t(k − 1))(1 − t)²` first written here is pessimistic,
  and two constant-`v` chains need no ring at any ratio. The rings are
  for chains oblique to the ruling. ADR-0052 §2–3 has the bound and the
  measurements.)*
- The weld contract: by edge id, positions compared bit for bit, mismatch
  refused rather than snapped (no tolerance anywhere, ADR-0003).

**`arris-geom`.** One new public method on `Surface`. Working name
`chord_steps_along(chord, band: [Interval; 2]) -> [f64; 2]`: the steps a
triangle standing on a curve inside `band` needs. It reads the cone at
`band`'s radii and a NURBS surface over its domain, and matches every
other kind to `chord_steps`. Additive.

**`arris-mesh`, public.**
- `tessellate_faces(&Model, Body, faces: &[FaceId], &MeshRequest,
  &Control) -> Result<TriMesh, MeshError>`. Its `FaceRange`s are the
  subset in the body's iteration order, duplicates collapsed. Its
  `EdgeRange`s are every edge the subset's loops use, in the body's order.
  Positions cover only what those faces reach. `tessellate_with` becomes
  `tessellate_faces` over every face, unchanged in output.
- `TriMesh::weld(parts: &[TriMesh]) -> Result<TriMesh, MeshError>`. Parts
  are joined by union-find over the indices of equal `EdgeId` runs, with
  first-seen order kept. A corner block is carried when every part has
  one, re-indexed, since a corner is face-local.
- **Breaking:** `MeshError` gains `NotInBody { face, body }` and
  `WeldMismatch { edge }`. The enum is exhaustive by rule, so this is a
  `CHANGELOG.md` `### Breaking` bullet, and `arris-py` gets the two classes
  in the same commit.
- The documented edge count (crate doc, `tessellate`'s rustdoc,
  ARCHITECTURE §Tessellation) is restated: "per coedge, the count that
  keeps each step's travel under its surface's steps along the pcurve's
  own box".
- The cancellation step list gains "a ring point" (ADR-0030's counting).
  `tests/cancel_counts.txt` holds no cone, so step 2 left it unchanged.

**`arris-py`.** `Model.tessellate_faces(body, faces, chord)` and
`arris.weld(meshes)`, with their stub lines, docstring examples and error
classes.

**Crate boundaries:** none change. The across-an-edit property lives in
`arris-ops`'s tests, which already dev-depend on `arris-mesh`. The corpus
runner check lives in `arris-debug`.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — **Kept edges keep their pcurves; the failure as
  fixtures.** First, a property in `crates/arris-ops/tests/` over `cut`,
  `fuse`, `cut_many`, `split`, `fillet`, `chamfer`, `offset_faces` and
  `shell` on the property operands, blended and drilled ones among them.
  For every edge kept by id, each output coedge on a face whose surface
  equals an input face's surface must carry a pcurve equal over the edge's
  range to the input's: the same `Curve2Id`, or a value equal bit for bit.
  An operation that re-fits one is fixed in this step to reuse the input's
  pcurve. If the fix is larger than a step, the case is recorded and the
  step stops for a plan amendment. Second, a regression fixture under
  `tests/fixtures/regression/`, `#[ignore]`d, held by a new recipe key
  `mesh_keeps` (the runner meshes the named steps and holds every edge
  shared by id with the result to the same polyline, bit for bit):
  - a disc with a conical boss, widening upward, its top cut away: the
    base circle the cut keeps by id, between the cone and the untouched
    plane (`regression/cone-boss-cut-kept-edge-mesh`; fails today at 28
    points before the cut and 24 after).

  Landed: the property (`kept_pcurve_prop.rs`) holds at 1500 cases on
  boxes, cylinders and rounded boxes, plain and drilled, so **no operation
  re-fits a kept edge's pcurve** and step 3's purity needs no fix in the
  operations (the first open question below is answered). The NURBS
  fixture is **not buildable**: every operation that would trim a NURBS
  face (the booleans, `split`) refuses one by name, as the NURBS cycle's
  work (`boolean/nurbs-box-cavity-cut`), so no kernel body has a NURBS
  face whose box differs between two states. Step 3 covers the NURBS rule
  with a unit test of pass one over the NURBS box's face with two bounds
  instead, and the fixture waits for the NURBS cycle.
- [x] Step 2 **[3]** — **Rings on a cone face, and ADR-0052.** A cone
  face whose loops span a radius ratio above 3 takes interior constant-`v`
  rings, geometric in radius, each sampled at its own radius's `u` step,
  kept where the loops wind around them and off every loop segment. They
  are counted against `MAX_INTERIOR_POINTS` and metered. This is
  face-local and changes no edge, so the mesh is still closed by
  construction. ADR-0052 is written here with the deviation bound above,
  the per-edge rule step 3 applies, and the weld contract. Tests: frusta
  with radius ratios 2, 3, 10 and 100, and a cone with its apex, meshed
  within the chord by the existing `assert_within_chord`. The random patch
  property is extended to draw frusta of ratio up to 100. Before this step
  those frusta pass only because their edges are over-sampled at the far
  radius. A test with edges forced to their own radius (a unit test on the
  pass) shows the excess the rings remove.

  Landed, with the design corrected (ADR-0052). Two constant-`v` chains,
  each sampled at its own radius, need no ring at any ratio: a segment
  between samples offset by half of each one's step deviates exactly
  `δ`, and Delaunay puts them no farther apart. Frusta of ratio up to 100
  came out at 0.82 chords with no ring. What does need rings is a chain
  oblique to the ruling beside a coarse narrow one. A drilled frustum of
  ratio 22 came out at 1.49 chords, the unit test's case. Rings at 3 left
  1.02 over a thousand random split and drilled frusta, and at 2 none
  over two thousand, so `RING_RATIO = 2` (the third open question). **A
  cone reaching its apex takes no ring**: a ring sparser than the
  collapsed apex row opens the apex's fan (113 of 144 drilled cones
  open). With every edge at its own radius and no ring, 944 random
  split and drilled cones stayed closed and within 0.9996 chords. The
  test seam is a private `Rules { own_box, rings }` in `tessellate.rs`;
  step 3 makes `own_box` the kernel's rule and removes the seam. The
  patch property's cone draws keep their own strategy; the frusta of
  ratio up to 100 are a new property beside it,
  `a_frustum_of_any_ratio_meshes_within_its_chord`, probed on a
  barycentric grid: a segment between two radii is farthest from the
  cone at `1 / (1 + √k)` of the way, which the midpoint probes miss.
- [x] Step 3 **[2]** — **An edge's samples a pure function of the edge.**
  Add `Surface::chord_steps_along` in `arris-geom`, with rustdoc and an
  example. Pass one reads each coedge's requirement from its pcurve's
  (u, v) box over the edge's range; a degenerate coedge keeps its face's
  count (ADR-0052 §1), and step 2's `Rules` seam goes. The box is computed from the pcurve
  alone, at the chord, by the same sampling `FaceDomain` uses for a loop.
  The interior lattice and the domain scaling still use the face's box,
  since both are face-local. The two step 1 fixtures pass and move to
  `mesh/` with their oracle-free assertions (the closed forms they hold
  are the polylines themselves). Every existing tessellation test, the
  corpus's `mesh_volume_rel` and the corner invariants stay green;
  `cancel_counts.txt` is regenerated and the commit body says why. Before
  committing, measure the real-part tier's total positions and wall time
  against `main`; see the open question below.

  Landed. Pass one reads each `Curve` coedge's (u, v) box from 65 samples
  of its pcurve over the edge's range and asks `chord_steps_along`; a
  degenerate coedge keeps its face's steps; `Rules` is gone, leaving a
  private `with_rings` flag for the one test that shows what rings buy.
  `cancel_counts.txt` is unchanged (no cone or NURBS in its cases). The
  real-part tier at 1e-3 holds 977,217 positions in 3.74 s against
  `main`'s 977,451 in 3.60 s, so the first open question is answered: the
  whole-domain NURBS rule stays. The fixture moved to `boolean/`, not a
  new `mesh/` area: a cut is its area, and a new area would need the lint,
  `DUMPED_AREAS` and the gate's table for one fixture. There is one
  fixture, not two (step 1: the NURBS one is not buildable); the NURBS
  rule is held by a property in `arris-geom/tests/chord_steps_along.rs`
  (every band gives the whole domain's steps) instead of a unit test of
  pass one.
- [ ] Step 4 **[2]** — **`tessellate_faces`.** The public function and
  `MeshError::NotInBody`. `tessellate_with` is rewritten as the full-set
  case; its output is unchanged, held by the existing tests and a
  bit-for-bit comparison against a mesh dumped before the change.
  Rustdoc with an example: mesh the cylinder's wall alone, then its caps.
  A sharded property (`prop_shards!`) over the sample bodies, and over
  booleans' and blends' results from `prop::recipe`, draws a random subset
  and checks it against one call. Each edge's polyline must have the same
  count and bit-identical positions. Each face's triangles, read as
  position triples, must be identical. The binding's error class and stub
  are added in this commit (the `Breaking` bullet as well).
- [ ] Step 5 **[2]** — **`TriMesh::weld`, and the corpus meshes face by
  face.** `weld` and `MeshError::WeldMismatch`, with rustdoc and an
  example. Properties: a random partition of a body's faces, meshed part
  by part and welded in a random order, gives a closed mesh whose
  triangles (as position triples) and edge polylines equal the one-call
  mesh's, with the corner block when asked. Two meshes of the same edge at
  different chords are refused by name. The corpus runner
  (`arris-debug::corpus`) also meshes every fixture face by face, welds
  the result and holds it to the one-call mesh. This is the roadmap's
  side-plan acceptance, run on every fixture.
- [ ] Step 6 **[2]** — **Across an edit.** A sharded property in
  `crates/arris-ops/tests/mesh_cache_prop.rs`. Draw an operand in a random
  pose and mesh every face separately. Apply one of `cut`, `fuse`,
  `cut_many`, `split` (each side), `fillet`, `chamfer`, `offset_faces` or
  `shell`. Keep the mesh of every face `Provenance::is_kept` says
  survived, mesh only the others, then weld. The weld must succeed, be
  closed, and equal the one-call mesh of the result. Each kept face's old
  mesh must equal its new one bit for bit. Cone and NURBS operands are
  among the draws: the revolve and the NURBS box.
- [ ] Step 7 **[1]** — **Python.** Bind `Model.tessellate_faces` and
  `arris.weld`, with docstring examples and stub lines in
  `python/arris/_arris.pyi`. A pytest meshes a box's faces one by one,
  welds them and compares the result to `Model.tessellate`. `mypy.stubtest`
  must stay green.

## Acceptance

- The roadmap's side-plan line, as a corpus run: every corpus fixture
  meshed face by face and welded equals its one-call mesh (edge polylines
  bit for bit, triangles as position triples) and is closed (step 5's
  runner check). `ARRIS_GATE=full` is green at 256 cases.
- Step 4's subset property, step 5's partition and weld property, and step
  6's across-an-edit property, sharded, at the configured case count.
- The chord bound still holds on every surface kind: the random patch
  property with frusta up to a ratio of 100, the sphere, torus and oblique
  hole tests, and the corpus's `mesh_volume_rel` on every fixture.
- Steps 1 and 3's regression fixtures pass in `mesh/` with no
  `#[ignore]`.
- The `python` CI job's pytest, docstring examples and `mypy.stubtest`.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Tessellation: the edge-count rule (per coedge,
  read along its own pcurve), cone rings, `tessellate_faces`, `weld` and
  its no-snap contract, `NotInBody` and `WeldMismatch`, the ring point as
  a metered step. The purity guarantee goes into the list of guarantees.
- `docs/ROADMAP.md` §C7: the status line records per-face tessellation as
  landed, which leaves C7 ready to close. The side-plan bullet is
  compressed into it.
- `docs/ideas/plugin-cad-consumer-asks.md`: the status line records A9 as
  landed.
- `AGENTS.md` "Current state": the C7 line drops "per-face tessellation
  beside them".
- `CHANGELOG.md` `## Unreleased`: what a consumer can now do (mesh a
  subset, weld, cache per face across edits) and the `### Breaking`
  bullet for the two `MeshError` variants, with its one-line fix: add the
  arms. The latter is written in step 4 and step 5, the former at
  retirement.
- `crates/arris-mesh/src/lib.rs` crate doc: purity and the subset API
  among the guarantees.
- `docs/BACKLOG.md`: any finding from step 1 or step 3's measurement.

## Open questions

- Answered at step 3 (977,217 positions in 3.74 s against 977,451 in
  3.60 s, no growth): does reading a NURBS coedge's requirement over the surface's
  whole domain over-sample faces trimmed out of large B-spline surfaces
  enough to matter? Decided by the agent at step 3, from the real-part
  tier's positions and time against `main`. If either grows more than 2×,
  the requirement is read instead over the knot-span cells the pcurve
  crosses, widened by one face step, and the ADR records why. Purity
  holds either way, since neither reads the face's box.
- ⚠ OPEN: does any operation re-fit a kept edge's pcurve on a modified
  face? Answered by step 1's property. If an operation's fix is larger
  than a step, the agent amends this plan with a step before step 3, since
  step 3's purity rests on it.
- Answered at step 2: the ring ratio is 2 (ADR-0052 §3). The closed
  form's 3 came from a pessimistic model, so measurement set the ratio
  instead: 3 left 1.02 chords on oblique frusta, and 2 none.
