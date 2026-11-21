# Plan: tight-body-bounds

- Started: 2026-11-21
- Milestone: C12, the training harness's refusals (docs/ROADMAP.md §C12 **In**,
  its last line)
- Idea: docs/ideas/tight-body-bounds.md (absorbed; option B, a separate
  `tight_bounds` with `bounds` unchanged, decided under the charter's
  delegation of technical decisions)

## Goal

`ops::query::tight_bounds` and the binding's `Model.tight_bounds` return the
**tight box** of a vertex, edge, face, shell or body. Every point of its
geometry lies inside the box, and each of the six sides lies within the
model's linear tolerance of a point the entity reaches. A NURBS edge is
subdivided only where its hull reaches past the running box. A face's interior
adds the points where its surface is extreme along an axis and that lie
inside its loops. `bounds` stays the cheap outer box that the booleans and
`interference_many` reject with. The harness's `shaft_collar`, `spacer_ring`
and `clevis_pin` cases match their closed forms within 1e-6.

## Non-goals

- `bounds` itself, `FaceDomain::bounds`, and the boolean's face-pair reject:
  unchanged.
- An oriented (minimal-volume) box, and a box in a frame other than the world.
- Growing the tight box by entity tolerances. A caller who wants the tolerance
  envelope uses `bounds` (ADR-0093).
- A tight box for a mesh or a sheet's free edges beyond what its faces and
  edges give.

## Design deltas

- **Curve** (`arris-geom`): `Curve::tight_bounds(range, eps) -> Option<Aabb>`.
  Line and conics return `bounds`, which is already exact. A NURBS curve
  keeps an inner box from evaluated points: the range ends and each split's
  midpoint. A queue holds the sub-ranges whose clipped hull reaches more than
  `eps` past that inner box on some side. Each one is split at a knot inside
  it, or else at its midpoint. The result is the inner box joined with the
  hulls left in the queue. It is outer, and within `eps` of the curve on every side.
- **Face interior**: by the exhaustive surface match. On a plane, cylinder,
  elliptic cylinder or cone, each coordinate is affine along a ruling, so
  its extremes lie on the boundary. The apex of a cone face that reaches it
  is a vertex. On a sphere, the six points `centre ± r·e_k` count. On a
  torus, the points whose normal is `±e_k` count: up to four isolated points,
  or one sample on each crest circle when `e_k` is along the axis. A point
  counts when `FaceDomain::side` puts its (u, v) inside or on the loops.
  On a NURBS surface, subdivide the face's `uv_box` and drop each patch that
  `FaceDomain::misses`. A kept patch is split like a curve range until its
  hull's excess is at most `eps`. The bound for a patch that the loops
  graze is established in step 1.
- **Query** (`arris-ops::query`): `tight_bounds(&Model, EntityId, &Control)
  -> Result<Option<Aabb>, OpError>`. It is the union over the entity's
  closure of vertex points, edges' `Curve::tight_bounds` and faces'
  interior points, with `eps = precision().default_tolerance`, the linear tolerance. It returns
  `None` where `bounds` does (an unbounded face). Errors: `NotFound`'s
  existing mapping, and `Interrupted`. No new error variant. A public
  addition, not a break.
- **Binding**: `Model.tight_bounds(entity, *, cancel=None, budget=None)`,
  returning corners like `bounds`. It gets a stub line and a docstring
  example with the `shaft_collar`.
- **Fixtures**: the `query` runner (`arris-debug/src/query.rs`) reads a
  `"tight_bounds"` query with `expect_box`. The oracle's `query.py` answers
  it with `BRepBndLib.AddOptimal(shape, box, False, False)`. Where that is
  off the closed form, `analytic.measure_differs` holds the fixture
  (ADR-0015).
- **ADR-0093**, *a tight box is outer within the linear tolerance*. It
  records option B, the guarantee, why tolerances are left out, and the
  ruling argument for analytic faces.
- **Glossary**: *tight box*, set against *box*, the outer bound grown by tolerances.

## Steps

Grades: **[1]** routine; **[2]** careful, a geometric or numeric case within
the design; **[3]** unproven, an algorithm whose bound is established here.

- [ ] Step 1 **[3]** — Bound a trimmed NURBS face by subdividing it and pruning
  with `misses`, as a private helper in `ops::query` that takes a
  `FaceDomain`, `eps` and a `Meter`. Establish the stopping rule for a patch
  that the loops graze. A property test over random bicubic patches, whole
  and trimmed by a random circle, holds the result outer and within `eps`
  of a dense sample refined by golden section.
- [ ] Step 2 **[2]** — `Curve::tight_bounds` for NURBS, as in the deltas.
  Test with rational and periodic curves, including a range that wraps the
  seam: outer, within `eps` of refined samples, and equal to `bounds` for
  lines and conics.
- [ ] Step 3 **[2]** — Interior extremes of analytic faces (sphere, torus)
  classified by `FaceDomain::side`, with closed-form unit tests. Cases: a
  ball cut by a box that keeps one pole and removes another; a quarter
  torus about z and about x; a cone face that reaches its apex.
- [ ] Step 4 **[2]** — `ops::query::tight_bounds` assembling steps 1–3. Add
  ADR-0093, its README line, the glossary's *tight box*, rustdoc with the
  `shaft_collar` example, and `tight ⊆ bounds` unit tests on every primitive,
  with equality on the analytic ones.
- [ ] Step 5 **[2]** — Fixtures and oracle: the runner's `tight_bounds`
  query and the oracle's `AddOptimal`. Add `query/tight-bounds-cross-hole`
  (`shaft_collar`, closed form (−2, −2, 0)..(2, 2, 2)),
  `tight-bounds-chamfer-slot` (`spacer_ring`) and
  `tight-bounds-clevis-pin`, each with its closed form. Add
  `tight-bounds-ball-pole` and `tight-bounds-quarter-torus` from step 3,
  and the bicubic dome, whole and cut. Each is a `corpus_tests!` row.
- [ ] Step 6 **[1]** — Binding: the method, stub line, docstring example and
  a pytest on the three harness bodies. Run `tools/py-test.sh tests/test_queries.py`.
- [ ] Step 7 **[2]** — Corpus property: for each `boolean/`, `blend/`,
  `sweep/` and `loft/` fixture result, require `tight ⊆ bounds`, every mesh
  vertex inside `tight` grown by the chord and tolerance, and every side of
  `tight` within chord + `eps` of the mesh box. It runs as a nextest corpus
  test that is in the full gate, not `fast`.

## Acceptance

- `cargo nextest run -p arris --test corpus query_tight_bounds` passes the seven
  new `tight-bounds-*` fixtures within 1e-6 of their closed forms. Each
  oracle value is the record, and `measure_differs` applies where
  `AddOptimal` is off.
- Step 1's and step 2's property tests pass at 1000 cases, outer and within
  `eps`, and step 7's corpus property passes over its four areas.
- `tools/py-test.sh tests/test_queries.py`: `tight_bounds` of the
  `shaft_collar` is (−2, −2, 0)..(2, 2, 2) to 1e-9, while `bounds` stays
  2.0012288786 wide in y.
- `tools/full-gate.sh` is green, with no new `bounds` caller in `ops` and no
  change to any fixture's `expected.json` apart from the new ones.

## Docs to update on completion

- `docs/DATA-MODEL.md` §Curve and surface bounds: replace "a tighter,
  piecewise bound is the caller's" with `Curve::tight_bounds`.
- `docs/ARCHITECTURE.md` §Operations (queries) and the crate table's
  `arris-ops` row: add `tight_bounds`. §How a consumer's kernel facade maps
  on: point there for a part's extent.
- `docs/ROADMAP.md` §C12: mark the tight-bounds line done, with its numbers.
- `AGENTS.md` current state: C12's in-list is done; next is `/close-cycle`
  after the publish retry.
- `GLOSSARY.md`: *tight box* (step 4). `CHANGELOG.md` gets an Unreleased
  bullet at retirement.

## Open questions

- ⚠ OPEN: the stopping rule for a NURBS patch the loops graze. The agent
  decides it in step 1 and records it in ADR-0093. The candidate: stop
  when the patch's hull diagonal is at most `eps`, since every point of it
  is then within `eps` of the boundary edge's own tight box.
- ⚠ OPEN: a speed bar. The agent picks it in step 4: under 0.1 s for the
  269-face real part used to time `classify_points`. A miss there is a
  finding, not a refusal.
