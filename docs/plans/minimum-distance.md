# Plan: minimum-distance

- Started: 2026-10-10
- Milestone: C8, the query cycle (docs/ROADMAP.md §C8)
- Idea (verbatim from the human): "on minimum distance, the next C8 slice. Interference and the planar section both build on distance, so it comes first. They reuse FaceTree::near, and the query fixture kind already exists, so the cost is mostly the extremum algorithms."
- Idea: `docs/ideas/minimum-distance.md` (absorbed). Option C was accepted, and the human delegated its four decisions ("decide yourself on open questions"). They are settled below and recorded by ADR-0054 in step 2.

## Goal
A consumer asks for the minimum distance between two operands. An operand
is a point, a vertex, an edge, a face or a body, in the same body or in two
bodies. The answer carries:
- the distance;
- the closest point on each operand, with the most specific entity it lies
  on (vertex, else edge, else face, as a ray hit names it, ADR-0053 §1) and
  that entity's parameter;
- whether the minimum is unique;
- whether one operand is contained in a solid.

The answer is exact to the operands' tolerances: it comes from enumerating
critical points with a certificate, never from sampling or a mesh. It covers
every pair of analytic kinds the kernel holds, except the three surface
pairs listed in Non-goals, and NURBS wherever the roadmap's C8 entry allows.
Answers are held to Open CASCADE's `BRepExtrema_DistShapeShape` by a
`distance` form of the `query` fixture kind. Interference and the planar
section then build on this query's contact test and on its `FaceTree`
nearest-first walk.

## Non-goals
- **Interference and the planar section.** They are the next C8 plans. This
  plan's contact test (step 8) is what interference will reuse.
- **NURBS × NURBS surface interiors, and any curve against a NURBS
  surface's interior** (roadmap C8 "Out", the NURBS cycle's). A point
  against a NURBS face (the existing projection) is in scope, and so are a
  NURBS surface against a plane or a sphere and a NURBS curve against any
  curve or any analytic surface.
- **Cone–cone, cone–elliptic-cylinder and elliptic-cylinder–elliptic-cylinder
  surface interiors.** No reduction brings these below three unknowns.
  They refuse as `Unsupported`, and only when their box distance could
  beat the best answer (Design deltas, ops). One backlog line records them.
  This reverses the idea's preferred answer to decision 3; Open questions
  gives the reason.
- **Every equal-distance solution, as Open CASCADE lists them.** One pair is
  returned, and `unique` says whether it was the only one.
- **Signed or penetration depth.** Overlapping operands are at distance 0;
  how deep they overlap is interference's to report.
- **Caching the face tree on a `Model`.** This is unchanged since ray fire.

## Design deltas
- **`arris-geom`, new module `extrema`**: the critical points of the
  squared distance between two *untrimmed* carriers. *As built (step 1),
  not an enum: `Extrema { isolated: Vec<ExtremumPair>, continua:
  Vec<ExtremumPair> }`, with `nearest()`. A pair of curves can have both:
  two coaxial circles have a continuum near and one far, and a near-twin
  has isolated pairs beside a continuum. `Feet<P> { isolated, continua }`
  likewise.* Each pair carries both parameters, both points and the
  distance.
  - **How the unknowns come down to two.** Each family is reduced to at
    most two unknowns:
    - Offsets: sphere → its centre, cylinder → its axis, torus → its
      centre circle. The critical points of `|f − r|` away from `f = r`
      are those of `f`.
    - Against a plane, the extremum of the height above it.
    - Against a cone, the signed distance in its meridian plane, which is a
      root of a squared trigonometric polynomial; spurious roots are
      removed by checking against the unsquared equation.
    - Against an elliptic cylinder, the projection onto its section plane
      against the ellipse.
    - *As built (step 1): any bounded curve against a circle is one
      unknown, the circle's own projection eliminating its parameter. The
      squared stationarity condition is a polynomial of degree `8n − 2`
      whose sign changes, and its derivative's, hold the pairs at either
      foot. The two-unknown system stalled for 130–400 ms on any pair of
      nearly coaxial circles, which is a pin in a hole.*
  - **How the remaining system is solved.** One unknown goes to `roots` or
    Bernstein. Two unknowns are the gradient system in rational or
    half-angle Bernstein form, solved by `bernstein2::common_zeros`
    (Krawczyk-certified, as ADR-0053 §7 chose for line × NURBS).
    *As built: where the isolation gives up, or leaves a box uncertified,
    the region is searched in one variable along its longer side, with the
    other curve's parameter at its foot. That replaced quartering the
    region, which cost 130–400 ms on a near-continuum and returned
    thousands of near-stationary points.*
  - **Continua.** Parallel lines, coaxial circles, concentric or coaxial
    offsets, and parallel planes are detected in closed form first. The
    solver's own `Continuum` is the backstop. A curve of critical points is
    a level set, so it is reported as a `Continuum`, not an error.
    *As built: every closed-form predicate reads the tolerance as a bound
    on how much the distance varies along the family (`2(δ + R sin θ) ≤
    tol.linear`), not as an angle. A region is a continuum only where both
    the distance stays within `tol.linear` and the tangential offset stays
    within `tol.linear` of zero at every sample.*
  - **Symmetry.** *As built: a pair is solved in one canonical order of
    its two curves (kind, then their defining numbers) and swapped back. In
    a valley flat to within the tolerance, the two orders' polishes
    otherwise settled at points 1e-4 apart.*
  - **Entry points.** `Curve::feet(p)` and `Surface::feet(p)` return every
    critical foot of a point, so the distance query never sees projection's
    `Ambiguous`: a point on an axis is a `Continuum`. The pair functions
    are `extrema_curve_curve`, `extrema_curve_surface` and
    `extrema_surface_surface`, each taking `(a, b, tol, meter)`. *As built:
    `feet` takes `(p, tol, meter)`.* The pairs
    in Non-goals return `GeomError::Unsupported`.
  - DATA-MODEL §Curves and §Surfaces gain the table of feet and
    extrema.
- **`arris-math`**: `Aabb::distance(&Aabb) -> f64`, zero on overlap.
- **`arris-check`**: `FaceTree::nearest(&Aabb, bound: f64) -> Vec<(f64,
  FaceId)>`. It returns the faces whose box is within `bound` of a box, by
  ascending box distance, with ties broken by `FaceId`. `near` is
  unchanged.
- **`arris-ops`**: `ops::query::distance(&Model, Operand, Operand,
  &Control) -> Result<Distance, OpError>`, a long query that takes a
  `Control` (ADR-0030).
  - **Public types:**
    - `Operand::{Point(Point3), Shape(Shape)}`, where the shape is a
      vertex, edge, face or body (a shell is refused with
      `QueryReason::NotMeasurable`);
    - `Distance { distance, a: Foot, b: Foot, unique, contained }`;
    - `Foot { point, shape: Option<Shape>, param: FootParam }`, with
      `FootParam::{None, Edge(f64), Face(Point2)}`.
  - **The trimmed search** is a recursion on boundaries. Between two
    entities, the minimum is the least of:
    - the carriers' isolated critical pairs whose feet lie inside both
      trims (`FaceDomain::side`, the edge's range);
    - each operand's boundary against the other: face → its edges,
      edge → its vertices.

    A `Continuum` adds no interior candidate. Its level set inside the
    trims reaches a boundary, because every closed family of the
    analytic kinds crosses a periodic seam, which is an edge, so a
    boundary term attains it (argued in ADR-0054).
  - **Body against anything** walks `FaceTree::nearest`, nearest first.
    The bound shrinks to the best distance found so far, and a pair whose
    box distance is not below it is skipped. An unsupported pair refuses
    only if it is not skipped, as `OpError::Unsupported` naming both faces.
  - **Contact.** Before the walk, the face pairs whose boxes overlap are
    intersected by the boolean's face–face section. A section point inside
    both domains gives distance 0 at that point.
  - **Containment.** A body operand is its solid (point-set semantics). A
    point, or any vertex of the other operand, that `classify_point` finds
    `Inside` gives `contained: true` and distance 0.
  - **Uniqueness.** `unique` is false when the minimum came from a
    continuum, or when two candidates within `tol.linear` of it have feet
    farther apart than the tolerance. The pair returned is the lowest one
    by `(a`'s entity id, `b`'s entity id, parameters)`, so the answer is
    deterministic.
  - **Errors and changelog.** New `QueryReason::NotMeasurable` for a
    shell, or a body that is not a solid (`InputReason::NotSolid` stays
    the body-kind check). This is a breaking change under `### Breaking`.
    ARCHITECTURE §Operations and the errors table are extended.
- **`arris-debug` / `tools/oracle`**: a `distance` query form,
  `{"distance": {"a": <operand>, "b": <operand>}}`.
  - An operand is `{"point": [x,y,z]}`, `{"step": "<recipe step>"}` (a
    body the recipe builds), or `{"step": …, "face" | "edge" | "vertex":
    {"near": [x,y,z]}}`. The last picks an entity by the point nearest
    it, since ids differ between kernels.
  - `expected.py` answers with `BRepExtrema_DistShapeShape`, recording:
    - the distance, every solution's points and support kinds;
    - `InnerSolution`, which gives `contained`;
    - each support face's centroid.
  - The runner compares the distance within the fixture's `linear`
    tolerance, and the points to *any* oracle solution. It compares a
    support face by centroid and the support kind (vertex, edge or face)
    exactly. `unique` must equal "the oracle has one solution", unless
    `analytic.query_differs` records why (for example, Open CASCADE splits
    one continuum into its sampled ends).
  - `tests/fixtures/README.md` §Query fixtures is extended.
- **`arris-py`**: `Model.distance(a, b)`, where `a` and `b` are a point
  tuple or a handle, returning `Distance` with `a`, `b` as `Foot`. It also
  adds `NotMeasurable` to the error mapping and the stub.
- **ADR-0054**: *the minimum distance is between point sets, by
  certified critical points and a boundary recursion*. It records:
  - point-set semantics and `contained`;
  - the uniqueness rule and the deterministic pick;
  - the reduction table and the pairs it refuses, with the pruning rule
    that refuses them only when they could matter;
  - why a continuum needs no interior candidate;
  - contact by the section;
  - the `distance` query form;
  - step 1's measured costs;
  - the reference-tree modules read: Open CASCADE's `BRepExtrema` and
    `Extrema` packages, for what is reported and where they sample.

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[3]** — `geom::extrema` for curve × curve and point × curve.
  - **Scope:** line–line in closed form; every other pair of {line,
    circle, ellipse, NURBS} as the gradient system through
    `bernstein2::common_zeros`; continuum predicates; `Curve::feet`.
  - **This is the step that settles the method.** It establishes that
    the certified solver survives the systems a distance produces, which
    have double roots at tangencies and near-continua (two circles
    1e-9 rad off coaxial, lines 1e-10 off parallel). It measures the cost
    per pair in a release build, as ADR-0053 §7 did. If near-continua
    stall the solver, the fix is a widened continuum predicate, decided
    here and recorded in the ADR.
  - **Fixtures:** `geom/c8-curve-extrema` (generated by `geom/generate.py`,
    answered by `GeomAPI_ExtremaCurveCurve`), covering skew lines,
    parallel lines, a circle and a line through its axis, two linked
    circles, coaxial circles, ellipse pairs, and a NURBS curve against a
    circle.
  - **Property:** random pairs in random poses. The minimum is ≤ a dense
    sample's minimum plus `tol.linear`, every pair is orthogonal to both
    tangents to rounding, and swapping the operands swaps the answer.
  - *As built:*
    - Two unknowns only for ellipse and NURBS pairs. A curve against a
      circle is one unknown (Design deltas).
    - The fixture asks `"extrema": true` pairs of the existing geometry
      kind. A new top-level key would have changed every geometry
      fixture's hash. The oracle is `Extrema_ExtCC` over each whole domain,
      and it is not complete: two circles may come back with one pair of
      four, and a family at one distance as points along it. So
      `oracle.rs` holds each oracle pair to one of ours, or to a continuum
      at its distance, and our nearest to its nearest. Completeness is the
      grid property's. The fixture also has a circle on the tube about
      another: a continuum with no common axis.
    - A third property holds circles, ellipses and NURBS arcs against a
      twin moved along the normal and tilted by up to 1e-3 rad.
    - Release cost per pair is in DATA-MODEL §Curves for the ADR:
      - two cubics of four spans: 2 ms;
      - degree 5 at six spans: 6 ms;
      - degree 15: 38 ms;
      - degree 25: 128 ms;
      - any of them against a circle: under 5 ms;
      - a near-continuum on the two-unknown route: up to 20 ms.
- [x] Step 2 **[1]** — ADR-0054 (above), with step 1's costs and the
  continuum rule.
- [x] Step 3 **[2]** — Point × surface and curve × surface:
  `Surface::feet` for every kind (closed forms, NURBS through its
  projection), and `extrema_curve_surface` by the reduction table. A curve
  against a NURBS surface is `Unsupported`.
  - **Fixtures:** `geom/c8-curve-surface-extrema` (`GeomAPI_ExtremaCurveSurface`):
    a line parallel to a cylinder, a circle against a torus, an ellipse
    against a cone, a NURBS curve against a plane and an elliptic
    cylinder, and a point on each axis.
  - **Property:** as in step 1, against a dense `(t, u, v)` sample.
  - *As built:*
    - `Surface::feet` and `extrema_curve_surface` are in
      `geom::extrema::surface`, a child of the extrema module that shares
      its pieces. Sphere, cylinder and torus are offsets of their base
      (a point, the axis, the centre circle): the curve's pairs against the
      base, lifted to `B ± R·n̂`. The plane is the roots of `N · n`. The
      cone is the squared meridian condition, a polynomial of degree
      `6n − 2`. The elliptic cylinder is the curve flattened against its
      section ellipse. Crossings come from `intersect_curve_surface` as
      distance 0.
    - *Reversed from the plan:* the sampled property is the surface
      eliminated by `Surface::project` over a dense sample of the curve,
      not a `(t, u, v)` grid: its sampled local minima must be a pair of
      the answer. It is cheaper and exact on the surface side. Stationary
      pairs on a whole circle of feet that are saddles (a curve through an
      axis) are not reported; the module docs say so.
    - *A step 1 fault found here and fixed:* two circles with the second
      centred on the first's point and standing in a plane through the
      first's axis (the tube) were five isolated pairs in the tilt pose,
      because the double zero at the axis is placed only to the
      subdivision's depth. `against_circle` now bisects the zeros of the
      components across the axis, and drops isolated pairs at the distance
      of such a family. It is a named test in `tests/extrema.rs`.
    - Fixture `geom/c8-curve-surface-extrema` is held to
      `GeomAPI_ExtremaCurveSurface`, which reports `parallel` with the
      nearer family only and is incomplete elsewhere (the curve pairs'
      comparison). A point's feet are tested by name and by property
      (`tests/extrema_surface.rs`), not by a fixture: the oracle has no
      all-feet query for a point.
    - A NURBS curve's minimum at a knot where it is only C⁰ is a kink of
      the distance, not a stationary pair: the topology's recursion has to
      treat such a knot as a vertex (step 7).
- [ ] Step 4 **[2]** — `extrema_surface_surface` by the reductions:
  - plane–X by height (a NURBS surface by its Bernstein gradient);
  - sphere–X → point–X, cylinder–X → line–X, torus–X → circle–X;
  - parallel planes, coaxial pairs and concentric pairs as continua;
  - the three Non-goals pairs and NURBS × other surfaces `Unsupported`.

  Fixtures: `geom/c8-surface-extrema` (`GeomAPI_ExtremaSurfaceSurface`).
  Property as above.
- [ ] Step 5 **[1]** — `Aabb::distance` and `FaceTree::nearest`, with
  rustdoc and an example each. A property over `prop::body` bodies checks
  that `nearest` returns exactly the faces brute force finds within the
  bound, in the stated order.
- [ ] Step 6 **[2]** — `ops::query::distance` for a point operand, with
  the public types and `NotMeasurable`:
  - a point against a vertex, an edge (feet in range, else the ends), a
    face (feet in the domain, else its edges), and a body (`nearest` walk
    plus containment);
  - the most specific entity by `boundary_entity`.

  The `distance` query form lands in this step: `kind_of`, the loader,
  `expected.py`, `selftest.py`, the lint and the README section. Fixture
  `query/distance-point-box`, which snaps a point to a face, an edge, a
  corner, and from inside. The step also adds the rustdoc example, the
  ARCHITECTURE §Operations entry, and the CHANGELOG bullet with its
  `Breaking` line.
- [ ] Step 7 **[2]** — Entity × entity, i.e. vertex, edge and face pairs,
  by the boundary recursion over steps 1–4. It includes the continuum
  rule, the uniqueness flag and the deterministic pick. Fixture
  `query/distance-faces` covers two parallel plates offset so that the
  minimum is a continuum reached at an edge, a cylinder face against a
  plane face, and an edge against a torus face.
- [ ] Step 8 **[2]** — Body against anything:
  - the nearest-first walk with the shrinking bound;
  - contact by the boolean's face–face section;
  - containment of a body;
  - an unsupported pair refused only if it was not pruned;
  - the `Control` tick per face pair.

  A body far from the other ends after visiting a handful of pairs (an
  assertion on the meter). Fixture `query/distance-bodies` covers two
  boxes apart, touching on a face, crossing, and one inside the other.
- [ ] Step 9 **[2]** — The distance corpus under `tests/fixtures/query/`:
  - primitive pairs in oblique poses: sphere–cylinder, cone–torus,
    cylinder–cylinder skew, plane–torus;
  - a drilled plate against a pin through the hole (clearance), a filleted
    block against a sphere on the blend;
  - a NURBS face from STEP against a point, a far-from-origin pair, a
    countersink against a plane (cone–plane);
  - a cone–cone pair that pruning skips, which answers, and one it
    cannot skip, which is refused naming both faces (a regression
    fixture `#[ignore]`d with the desired value, per the rules).

  Each fixture has Open CASCADE's answer. Where Open CASCADE lists several
  solutions or splits a continuum, `analytic.query_differs` records it.
- [ ] Step 10 **[2]** — Property tests over random poses of `prop::body`
  operands:
  - distance is symmetric;
  - it is zero exactly when the boolean `common` is non-empty or the
    operands touch (`common` refusing `Empty` with contact found);
  - it equals the closed form for sphere–sphere and sphere–box;
  - each foot lies on its entity and within the entity's tolerance;
  - translating `b` by `δ` along the feet's line moves the distance by
    `δ` while it stays positive.
- [ ] Step 11 **[1]** — `Model.distance` in `arris-py`, with
  `Distance`/`Foot`, the `NotMeasurable` class, the `_arris.pyi` lines
  and a docstring example. Pytest covers a point snap and two bodies.

## Acceptance
- `cargo nextest run --workspace` (the full gate, `real_*` included) with
  all of the following green:
  - the `geom/c8-*-extrema` fixtures;
  - every `query/distance-*` fixture matched to `BRepExtrema_DistShapeShape`
    within its tolerance, the checker at `Full` on every operand;
  - steps 1, 3, 4, 5 and 10's properties at 256 cases.
- The `python` job's pytest, docstrings and `mypy.stubtest` pass for
  `Model.distance`.

## Docs to update on completion
- `docs/ROADMAP.md` §C8 status: distance landed (ADR-0054); interference
  and the section remain.
- `docs/ARCHITECTURE.md` crate table: `arris-geom` gains extrema,
  `arris-check` gains `FaceTree::nearest`, and `arris-ops` gains
  `distance`. Also a drift check of §Operations and §Errors against the
  code.
- `docs/DATA-MODEL.md` §Curves / §Surfaces: the feet and extrema table,
  and that `feet` replaces `Ambiguous` for a distance (drift check against
  steps 1–4).
- `tests/fixtures/README.md` §Query fixtures and `tools/oracle/README.md`:
  the `distance` form and `BRepExtrema` (written in step 6; drift check
  only).
- `docs/BACKLOG.md`: the three refused surface pairs, and any step
  finding.
- `CHANGELOG.md` `Unreleased`: the distance bullet (written in step 6;
  check it reads for a consumer).
- `AGENTS.md` current state: C8 line, distance landed.

## Open questions
The idea's four decisions, settled by the agent at the human's
delegation (2026-10-10) and recorded in ADR-0054 at step 2:
1. **Solids are point sets.** A body inside another, or a point inside
   one, is at distance 0 with `contained: true`.
2. **One pair per answer.** The lowest by entity ids then parameters, with
   `unique: false` when the minimum is not the only one.
3. **Reversed from the idea's preferred answer.** Cone–cone,
   cone–elliptic-cylinder and elliptic-cylinder–elliptic-cylinder
   interiors are refused, not solved. Planning showed that the reductions
   bring every other pair to at most two unknowns, which the existing
   certified solver takes, while these three need a solver in three or
   four unknowns that the kernel lacks. That solver would make this plan's
   riskiest step riskier still. The cases are rare: a countersink is a
   cone against a plane, a cylinder or a sphere, and every one of those is
   covered. The refusal fires only when the pair's box distance could beat
   the best answer, so a body–body query with cones on both sides still
   answers whenever its minimum lies elsewhere. A backlog line keeps the
   solver, to be picked up when a corpus shows the refusal.
4. **One ADR**, ADR-0054, in step 2.

- ⚠ OPEN: does the continuum rule's argument hold for every trimmed face
  the kernel can build? A closed level set inside both trims that crosses
  no seam would be missed: a face whose domain is a disk around a pole,
  with no seam inside it. The agent decides at step 7, by a fixture that
  builds such a face if the kernel can, and by a representative point of
  the continuum as a candidate if it can.
