# Plan: ray-fire

- Started: 2026-10-08
- Milestone: C8, the query cycle (docs/ROADMAP.md §C8)
- Idea (verbatim from the human): "first slice of C8"

## Goal
A consumer fires a ray at a body and gets the first hit or every hit, each
with the entity it landed on (the face, or the edge or vertex when it lands
within that entity's tolerance), the ray parameter, the face's `(u, v)`, the
point and the outward normal. This works on every face kind the kernel
holds, NURBS included. The query runs through a face-box hierarchy that the
other three C8 queries reuse, and the classifier runs through it too. The
answers are held to Open CASCADE's (`IntCurvesFace_ShapeIntersector`) by
a new `query` fixture kind, which the distance, interference and section
plans then extend instead of inventing their own.

The slice is chosen because it sets up what the whole cycle shares. Those
are the hierarchy and the query fixture kind, and C8's roadmap entry asks
for both. It proves them on the query whose geometry is nearest to code
that already exists, the classifier's ray cast. The one geometric unknown,
a line against a NURBS surface, goes first. It is the arm
`intersect_curve_surface` lacks today, and filling it lifts the
classifier's NURBS refusal too.

## Non-goals
- Minimum distance, interference and the planar section: the next C8 plans.
- A curve other than a line against a NURBS surface (a conic or a NURBS
  curve against one): the NURBS cycle's. `intersect_curve_surface` keeps
  `Unsupported` for them.
- Booleans with NURBS operands. The classifier stops refusing them, but
  the face–face intersector still refuses them (backlog line unchanged
  apart from its classifier half).
- Ray fire against a mesh, a sheet body or a whole model at once. The
  query takes one body; a consumer loops over its bodies.
- Caching the hierarchy across calls on a `Model` (the model is immutable
  to a query; a long-lived handle is a later ask if profiling shows it).

## Design deltas
- **`arris-geom`**: `intersect_curve_surface` gets a `(Curve::Line,
  Surface::Nurbs)` arm (`nurbs/line.rs`). The surface is split into Bézier
  patches; a patch whose control points all lie beyond the `tol.linear`
  tube round the line is rejected. On each of the rest the line's two
  plane offsets over the homogeneous net are two Bernstein polynomials.
  Their common zeros, isolated and Krawczyk-certified by
  `bernstein2::common_zeros` and polished by Newton on `(u, v)` (`t` is
  then the point's), are the crossings. *As built, not Newton on
  `(t, u, v)`: eliminating `t` leaves a square system the existing
  certifier takes.* A touch is decided on the distance along the line, as
  the analytic arms decide it. A crossing whose second-order extremum is
  within `tol.linear` is one tangent hit at that extremum, found by Newton
  on the gradient of `|Φ|²` and confirmed by projection. A line that
  passes within the tube without meeting the surface touches it at a
  critical point of `|Φ|²`, found by the gradient's polynomials gated to
  the fold and the tube. A pole within the tube is divided out of the
  system. `Coincident` is a touch parallel within `tol.angular` to a
  direction the surface does not curve along, and within the tube across
  the control net. Every other curve against a NURBS surface stays
  `Unsupported`. DATA-MODEL §Curve–surface intersection is updated with
  it. Crate-private `bernstein2` changes the arm needs: a `Gate` may be
  one-sided and there may be several; a `Continuum` carries the boxes
  alive when it gave up and the zeros found before; and
  `common_zeros_with(…, Halving::LookAhead, …)` halves a box the
  Krawczyk test is within reach of along the direction whose halves'
  spread is smaller. Under the old score a system linear in `t` (a
  cone's offsets from a line) was halved along `s` to the bottom, `t`
  never pinned. The torus tracer keeps the old `Halving::Scored`: under
  the new rule six blessed dumps move in their last digits. Switching it
  is a backlog line. *The classifier stops refusing NURBS faces in this
  step, not step 4: it dispatches through this arm.*
- **`arris-check`**: new public `tree::FaceTree`, a bounding-volume
  hierarchy over `FaceDomain::bounds` (each face's box, already inflated
  by its tolerance). It is built by a deterministic median split on the
  longest axis, with ties broken by `FaceId`. It offers
  `FaceTree::of_body(&Model, Body)`, `along(ray, range)` (the faces whose
  boxes a ray segment enters, in ascending entry order) and `near(&Aabb)`
  (the faces whose boxes overlap a box, in `FaceId` order). *As built,
  `along` takes `(origin, direction, range)` since `Ray` is `arris-ops`'s,
  a face with no box is in every answer, and `of_boxes` builds a tree from
  boxes a caller already holds.* The tree
  lives beside `classify` because `Classifier` uses it and `check` is
  below `ops` (ADR-0013). `Classifier::contains` casts through the
  tree, and its answers are unchanged.
- **`arris-ops`**: the API adds `ops::query::ray_fire(&Model, Body, &Ray,
  RayHits) -> Result<Vec<RayHit>, OpError>` with new public types `Ray {
  origin, direction }`, `RayHits::{First, All}` and `RayHit { shape:
  Shape, t, uv, point, normal: UnitVec3, tangent: bool }`. `shape` is the
  most specific entity (a vertex, then an edge, then a face), as
  `Classification::On` names it. `normal` is the face's outward normal
  (`frame_at`'s `Z`). On an edge or vertex it is the normal of the hit
  face with the lowest id, and the docs say so. New `QueryReason`
  variants: `RayInFace` for a ray lying in a face (see Open questions)
  and `ZeroDirection`. Adding these variants to `QueryReason` is a
  breaking change and goes under `CHANGELOG.md` `### Breaking`.
- **`arris-debug` / `tools/oracle`**: new fixture kind, `"kind":
  "query"`. It holds a solid recipe (the existing interpreter, unchanged)
  and a `queries` list. In this plan each query is `{"ray": {origin,
  direction}, "hits": "first"|"all"}`. `expected.py` answers it with
  `IntCurvesFace_ShapeIntersector` (t, point, `(u, v)`, the face's
  transition). The Rust runner builds the body, runs the checker, and
  compares each hit's count, `t`, point and normal within the fixture's
  tolerance, and the face it lands on by the face's centroid, since ids
  differ between kernels. `tests/fixtures/README.md` gets a §Query
  fixtures section, and the corpus lint covers the kind.
- **`arris-py`**: `Body.ray_fire(origin, direction, all=False)` returns a
  list of `RayHit`, plus the new `QueryReason` leaves in the error
  mapping and the stub.
- **ADR-0053**: *a ray hit names the most specific entity, and the query
  corpus is one fixture kind*. It records the hit semantics (entity
  precedence, tangent hits, the ray from inside a solid, the in-face
  case), the hierarchy's place and build rule, and the query fixture
  kind's comparison rules (match by face centroid, not by id). It also
  records the decision on line × NURBS (subdivision, not marching), and
  names the reference-tree modules read (Open CASCADE's `IntCurvesFace`,
  `BVH`).

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[3]** — Line × NURBS surface in `intersect_curve_surface`:
  Bézier extraction, hull-box rejection, subdivision to a bracket the
  Newton step is proven to converge in, tangent and coincident detection
  within `tol.linear`, deterministic hit order. Geometry fixtures
  `geom/c8-line-nurbs-hits` (generated by `geom/generate.py`):
  transversal hits on a bicubic patch and a rational sphere patch, a
  graze, a line along an isoline of a ruled NURBS (coincident), hits at
  patch seams and knot lines, and a miss by one tolerance. `pairs` lifts
  "nurbs surfaces are never paired" for a line `a`, and `expected.py`
  answers with `GeomAPI_IntCS`. A property test fires random lines at
  random NURBS patches and holds each hit on both the line and the
  surface, with no transversal crossing missed. Sign-change samples on
  a dense grid are the brute-force check. *As built: the grid check
  starts Newton on the surface from each steep piercing of the grid's
  flat triangles and requires the crossing it converges to among the
  hits. Two more properties hold the exact NURBS twin of every analytic
  surface to the analytic arm, for random lines and for lines in a
  random tangent plane off it by ±½ and ±2 tolerances; poles, knot
  lines, seams, rulings and grazes are pinned in `tests/line_nurbs.rs`.
  The classifier's two tests that pinned the NURBS refusal now pin the
  answer.*
- [x] Step 2 **[1]** — ADR-0053 (above), with the `⚠ OPEN` answers below
  settled in it.
- [x] Step 3 **[2]** — `arris_check::tree::FaceTree`: build, `along`,
  `near`, rustdoc with an example. A property test over random bodies
  from `prop::body` checks that `along` and `near` return exactly the
  faces a brute-force box test finds, in the stated order, the same on
  every run.
- [x] Step 4 **[2]** — `Classifier` casts through `FaceTree` (a NURBS
  face is already crossed by step 1's arm, which the classifier
  dispatches through). A property test
  checks the old linear cast and the tree cast agree point for point.
  Fixture: a real-part `classify` probe on a NURBS face of a committed
  NIST part, which used to be `ClassifyError::Geometry` and now has
  Open CASCADE's class. *As built, a test (`arris-io/tests/
  classify_nurbs.rs`, `real_*`) rather than a `build/` fixture: the whole
  part as a recipe costs 219 s of corpus stages and misses the oracle's
  mass properties by 3e-7 (a tolerance the recipe would have to widen),
  while the probes are what the step is about. The classes are
  `BRepClass3d_SolidClassifier`'s, recorded in the test. The tree cast
  is held to the linear one by `Classifier::classify_linear`
  (`#[doc(hidden)]`), where the linear cast decides; the tree also stops
  a ray that lies in a plane far from its face abandoning a direction.* *The backlog's "NURBS operands in booleans"
  line was narrowed to the face–face half in step 1, with the behaviour.*
- [x] Step 5 **[2]** — The `query` fixture kind: `kind_of`, the recipe
  loader, `expected.py`'s ray answers through
  `IntCurvesFace_ShapeIntersector`, `selftest.py`'s smoke case, the
  corpus lint, and `tests/fixtures/README.md` §Query fixtures. One
  fixture, `query/ray-box-through`, runs end to end with the comparison
  stubbed to the hit count, so the infrastructure lands before the API.
- [ ] Step 6 **[2]** — `ops::query::ray_fire` with `Ray`, `RayHits`,
  `RayHit` and the new `QueryReason` leaves. The query walks
  `FaceTree::along`, filters hits by `FaceDomain::side`, dedupes hits that
  land on a shared edge or vertex into one hit naming it, and stops early
  for `First`. The full runner comparison (t, point, normal, face by
  centroid) replaces step 5's stub. Rustdoc with an example;
  `docs/ARCHITECTURE.md` §Operations is extended in this commit.
  `CHANGELOG.md` gets the bullet and the `Breaking` line.
- [ ] Step 7 **[2]** — The ray corpus under `tests/fixtures/query/`:
  primitives (box, cylinder, sphere, cone, torus) hit transversally,
  through an edge, through a vertex, grazing a cylinder, from inside, and
  missing. Also boolean results (a drilled plate, a pocket seen through
  its hole), a filleted block hit on the blend, a STEP read of a NURBS
  face (step 1's arm through the whole stack), and a body far from the
  origin. Each one has Open CASCADE's answer. Where Open CASCADE reports
  an edge hit twice (once per face), `analytic.query_differs` records why
  and holds Arris to one hit.
- [ ] Step 8 **[2]** — Property tests over random poses (`prop_shards!`):
  each hit lies on its face and on the ray within tolerance; `First` is
  the minimum `t` of `All`; for a ray starting `Outside` a closed solid,
  the number of crossings past the last tangent-free hit is even; from
  `Inside` it is odd and agrees with `classify_point`; and a ray fired
  back from past the last hit returns the same hits reversed.
- [ ] Step 9 **[1]** — Python binding: `Body.ray_fire`, a `RayHit` class,
  the new error leaves, the stub and a docstring example.
  `CHANGELOG.md` gets the binding's bullet.

## Acceptance
- `cargo nextest run --workspace` at 256 cases: geometry fixtures
  `geom/c8-line-nurbs-hits` green against `GeomAPI_IntCS`; every
  `query/ray-*` fixture green against `IntCurvesFace_ShapeIntersector`
  with the checker at `Level::Full` on its body; the step 1, 3, 4 and 8
  property tests green.
- The existing corpus is unchanged by step 4: no `expected.json`
  or `dump.txt` moves, and the differential's agreement count at 1000
  draws of the fixed seed is no lower than ROADMAP §Beside the cycles
  records.
- The Python job's pytest, docstrings and `mypy.stubtest` pass for
  `ray_fire`.

## Docs to update on completion
- `docs/ROADMAP.md` §C8 — ray fire and the hierarchy marked done in the
  status line, with the ADR.
- `docs/ARCHITECTURE.md` §Operations (queries) — `ray_fire` and the
  `FaceTree` it shares; the crate table's `arris-check` row gains the
  tree, and its `arris-ops` row gains ray fire.
- `docs/ARCHITECTURE.md` error table — `QueryReason::RayInFace`,
  `ZeroDirection`.
- `docs/DATA-MODEL.md` §Curve–surface intersection — the line × NURBS
  arm. §The checker B1 row says a NURBS face is classified against, no
  longer unchecked. (Both done in step 1, with the classifier's and the
  boolean's paragraphs of `docs/ARCHITECTURE.md`.)
- `docs/adr/README.md` — ADR-0053 indexed.
- `tests/fixtures/README.md` — §Query fixtures, and the geometry
  section's `nurbs` surface row ("never paired" → paired with a line,
  done in step 1).
- `tools/oracle/README.md` — `expected.py`'s query kind.
- `docs/BACKLOG.md` — the NURBS-booleans line narrowed (done in step 1).
- `CHANGELOG.md` — the plan's consumer bullets, checked complete.
- `AGENTS.md` current state — C8's first slice landed.

## Open questions
- ~~⚠ OPEN: a ray lying in a face (in a plane face, or along a cylinder's
  ruling) — refuse it as `QueryReason::RayInFace` naming the face, or
  report the face's entry and exit on its boundary as two hits? *Agent
  decides in ADR-0053 (step 2) after reading what Open CASCADE returns for
  it in step 1's fixtures. (Step 1: `GeomAPI_IntCS` returns a line along
  a NURBS ruling as 83 scattered points of it and no segment, which is
  no answer to build on.) The recommendation is to refuse: a pick ray
  in a face is measure zero, and a silent pair of boundary hits would be
  a guess about what the consumer meant.* *Settled in ADR-0053 §4: refuse.*~~
- ~~⚠ OPEN: an edge hit's `normal` — the lowest-id face's, as drafted, or
  `None` with both faces' normals on request? *Agent decides in ADR-0053
  (step 2). Snapping wants one normal and a section view wants neither,
  so the draft stands unless the plugin CAD's facade says otherwise.* *Settled in ADR-0053 §2: the lowest-id face's.*~~
- ~~⚠ OPEN: whether step 1's subdivision bound holds for a degree-high,
  highly rational surface within the corpus's time budget.~~ *Settled in
  step 1: it holds, and no bound is needed. One release-build Bézier patch
  of 20 lines through it, half of them exact grazes, costs on average per
  line: 11 ms at degree 3, 24 ms at degree 9 (weight ratio 4), 18 ms at
  9 (ratio 100), 44 ms at 15 (ratio 10), and 133 ms at 25 (ratio 10 or
  1000), `MAX_DEGREE`. A transversal line alone is 0.1–28 ms over the
  same range. The weight ratio does not move it. The numbers go in
  ADR-0053.*
