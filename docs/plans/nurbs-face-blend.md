# Plan: nurbs-face-blend

- Started: 2026-10-13
- Milestone: C10, the NURBS cycle, first item's second half (docs/ROADMAP.md §C10)
- Idea (verbatim from the human): "the blend on a NURBS face (fillet and chamfer via the offset of a NURBS face)."

## Goal

`fillet` and `chamfer` take an edge between a NURBS face and an analytic
face, and an edge between two analytic faces that the closed-form table
does not hold. Such an edge blends to a **walked stripe**. For a fillet,
the ball's centre runs on the spine, which is the section of the two
faces' offsets by the radius. An analytic face's offset is exact
(`Surface::offset`). A NURBS face's offset is fitted to a fraction of the
tolerance, and the section is traced by ADR-0061's tracer. The contacts
are the spine's feet on the two faces. The blend surface is a rational
NURBS whose sections are the ball's arcs. A chamfer is the ruled NURBS
surface between its two contacts, placed by the ring's chord rule. A
walked stripe's end on an analytic face across is traced as ADR-0037
traces one; at a tangent vertex of a chain it ends on the ball's arc in
the vertex's normal plane, which the chain's next stripe shares. A closed
edge has no ends. Everything else is refused by
name. When done, the twin property holds: fillet or chamfer an edge with
one face replaced by its exact NURBS twin, and the result agrees with the
table's closed form on the original. The real parts' fillet column has no
NURBS row left that this plan covers: CTC-05's NURBS × torus in both
editions, the three B-spline-edge parts (CTC-02, FTC-11, FTC-10) and
CTC-01's crease. Each either agrees with Open CASCADE or is refused as
another item's, by name.

## Non-goals

- **Two NURBS faces.** That edge's spine is NURBS against NURBS, the
  cycle's third item, so it stays `Unsupported` naming both. The same
  holds for a walked stripe whose face across is NURBS.
- **A walked stripe meeting another blend at a sharp vertex.** This
  covers a miter and a sphere or triangle corner. Each stays
  `VertexBlend` as it is refused today, and a backlog line records which.
  Step 1's census moved the chain junction at a tangent vertex in: four of
  the five parts need it (step 5).
- **The shell or offset of a NURBS face** (C7's out list). The fitted
  offset this plan adds is their primitive, and their plan consumes it.
- **Variable radius, the other chamfer modes, blends over blends, the
  parallel-cylinder row** (it has a closed form, ADR-0036 §5) **and the
  horn and spindle tori.** All stay on the backlog.
- **NURBS × NURBS in the checker.** S5's pair of a NURBS blend and a NURBS
  face stays unchecked, which the corpus already allows for NURBS face
  pairs (`check_leaving_nurbs`).
- **Speeding up the ADR-0061 walk.** Its backlog line stands. This plan
  records the timings it meets.

## Design deltas

- **ADR-0062 (step 2): a blend with no closed form is a walked stripe.**
  The spine is the section of the two offsets: a NURBS offset is fitted,
  an analytic one is exact. The surface is fitted through the ball's
  exact arcs. The ADR records:
  - the error budget, split between the offset fit, the spine's
    section fit and the surface fit, against the stripe's tolerance;
  - how the offset is extended past the face's domain where the ball
    reaches beyond it;
  - the chamfer's chord rule;
  - which refusals carry which names.

  It amends ADR-0007 ("every surface is exact") and ADR-0037 §1 (a
  fitted surface was ruled out until the NURBS cycle) with an amendment
  line each. Alternative recorded: marching the ball's centre by Newton
  on the two distance constraints (Open CASCADE's `BRepBlend` walk). It
  is rejected because the spine would be found by an algorithm the
  checker does not share, and ADR-0037 ruled that out for ends.
- **`arris-geom`, public:**
  - `NurbsSurface::offset_fitted(distance, over: [Interval; 2], tol,
    meter) -> Result<NurbsSurface, GeomError>`, the fitted offset over a
    parameter box. A singular normal (a collapsed row) and a fold (the
    distance past a principal radius on its side) are refused by name.
  - A section skinning fit that the blend surface uses now and loft
    uses next, fixed in step 4: `fit_skin(section: impl Fn(f64) ->
    Result<NurbsCurve, E>, range: Interval, degree, tol, meter) ->
    Result<NurbsSurface, E>` with `E: From<GeomError>`. The sections share
    one degree, knot vector and count; each control point's `(w·P, w)` is
    fitted along `v`.
  - `Surface::offset` is unchanged: exact or `None`, and its doc points
    to the fitted one.
  - `GeomError::OffsetFolds { kind, at, distance, radius }` (step 2: no
    existing variant names a fold). A break: binding and changelog in
    that commit.
- **`arris-ops::blend`, crate-private:** a `walked.rs` module beside
  `stripe.rs` and `ring.rs`, and the `Blend` record holds either kind of
  stripe. The dispatch tries the table first and the walked stripe after
  it. No public signature changes: `fillet` and `chamfer` take more
  edges, and their rustdoc says which.
- **`arris-debug`:** `prop::twin::nurbs_twin_faces`, the twin of chosen
  faces only. The twin property for blends goes in `arris-ops`'s
  `twin_prop`.
- **`docs/DATA-MODEL.md` §Invariants, S5:** a walked blend against a
  neighbour it is tangent to is decided through its contact isoline
  (step 6), or the pair is listed as unchecked.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[1]** — **Census of the fillet column's NURBS rows.** Run
  the battery's `fillet` stage on the six rows of `nurbs-analytic-section`'s
  census. These are CTC-05 AP203 and AP242 e1 (NURBS × torus), CTC-02
  AP242 e2, FTC-11 AP242 e2 and the committed FTC-10 (B-spline edges
  between analytic faces), and CTC-01 AP203 (the crossing cylinders'
  crease). For each edge refused, record:
  - the two surfaces and the edge's curve;
  - the radius against the NURBS face's least principal radius on the
    ball's side;
  - each end: the face across's kind, the vertex's degree, and whether
    another sampled edge meets it there;
  - whether the ball reaches past the NURBS surface's domain.

  Shrink each reachable edge to a `regression/` fixture with its Open
  CASCADE fillet as oracle, `#[ignore]`d. Write the table into this plan.
  It fixes step 10's acceptance list and answers the extension question
  below. It also checks whether the B-spline edges hide a conic (a
  circle or an ellipse within tolerance) or a genuine non-conic.

  **Census (2026-10-13, `real_parts --nurbs-fillet`,
  `nurbs_census::fillet_census_dir`).** Each of the six rows' battery
  samples, every edge filleted alone in both kernels at the battery's
  radius today. That radius is FTC-10's 0.5 and CTC-01's 1.910, not the
  values the committed fixtures' battery cases record. Eight refused
  edges, all convex:

  | Part | Edge | Faces, curve | Ends | Chain | Open CASCADE | Fixture |
  |---|---|---|---|---|---|---|
  | CTC-05 AP203 #249, r 0.889 | e256 | NURBS (3, 2) 2×1 rational × torus; cubic B-spline, no conic | v84: cylinder across, degree 3; v115: cylinder across, degree 3, a junction | 3: e256 → e133 (cylinder × torus off its axis, a quartic as a B-spline 1.6e-6 off the traced section) → e32 (plane × cylinder, line) | builds **invalid**: volume −1.2e8 | `regression/nist-ctc-05-ap203-nurbs-torus-fillet` |
  | CTC-05 AP242 e1 #1205, r 0.889 | e256 | the same face as one Bézier patch; B-spline of 11 spans | as AP203 | as AP203 (e131 for e133) | builds **invalid**: volume +29 mm³ over the part's where the fillet removes ≈ 2; its area −4.6 fits | `regression/nist-ctc-05-ap242-nurbs-torus-fillet` |
  | FTC-10 #6434, r 0.5 | e0 | cylinder × cylinder, axes skew; closed B-spline of 16 spans, a quartic (4.0e-4 off the traced section, tolerance 6e-4) | none, closed | none | builds | `regression/nist-ftc-10-cylinder-pair-bspline-fillet` |
  | FTC-11 AP242 e2 #1299, r 0.0105 | e0 | plane × cylinder; one-span B-spline hiding a circle (9.3e-5, tolerance 2.5e-3) | planes across, degree 3, both junctions | 3: line, e0, line (a slot's end) | builds | `regression/nist-ftc-11-ap242-bspline-circle-fillet` (excerpt of #1299) |
  | FTC-11 AP242 e2 #1299, r 0.0105 | e24 | plane × plane, line; refused at e25, plane × cylinder, a B-spline hiding a circle (4.4e-4) | — | 3: e24, e25, e26 | builds | `regression/nist-ftc-11-ap242-slot-chain-fillet` |
  | CTC-02 AP242 e2 #14911, r 1.316 | e283 | plane × plane, line; refused at the chain's B-spline edges | — | 13: lines alternating with six plane × oblique-cylinder edges, B-splines 8.4e-4 off the ellipses (tolerance 8.5e-4) | builds | `regression/nist-ctc-02-ap242-ellipse-chain-fillet` |
  | CTC-01 AP203 #335 = committed #101, r 1.910 | e290 | cylinder × cylinder, equal radii, axes meeting; ellipse | v211: plane across, degree 3; v113: a fan of two planes, degree 4 | none | builds | `regression/nist-ctc-01-crossing-cylinders-crease-fillet` (committed file) |

  What it settles:
  - **NURBS faces.** Only CTC-05 has one. Its least principal radius on
    the ball's side over the stripe is 122 (AP242) and 127 (AP203),
    against r 0.889, so no census edge folds; the fold refusal is tested
    on step 2's bump.
  - **The extension question: needed.** Both of CTC-05's ends lie on the
    B-spline's domain boundary, and the contacts carried past them by r
    leave the domain by a tenth of its width. ADR-0062 fixes the
    mechanism (see the open question).
  - **Hidden conics.** FTC-11's two B-spline edges hide circles and
    CTC-02's six hide ellipses, each within its tolerance. FTC-10's is a
    genuine quartic, as is CTC-05's e133. So the walked stripe takes them
    all as "a table pair whose edge is not the table's curve" (step 5).
    Reading the circle so that the table takes FTC-11 stays the backlog's
    B-spline-conic line.
  - **Chains.** Four of the five parts reach a tangent vertex where the
    chain goes on, with a walked member beside a table stripe or another
    walked one. The chain junction is moved in (non-goals, step 5).
    CTC-01's crease is no chain, but its end at v113 is a fan of two
    planes, which step 9 takes or names.
  - **The oracle.** Open CASCADE's CTC-05 body is invalid in both
    editions and its measures cannot hold the fixture (open question).
    The other five agree with a fillet in dV/dA ≈ r/2 or are within the
    quadrature's noise (FTC-11, a 5135 mm³ part).

  Step 10's acceptance list is these seven fixtures. Steps 5 and 9 build
  each piece they need: the walked stripe on a NURBS face (CTC-05), over
  two exact offsets (FTC-10, CTC-01, CTC-02's ellipses) and on a table
  pair whose curve is a B-spline (FTC-11), the chain junction (CTC-05,
  FTC-11, CTC-02) and a fan end (CTC-01).
- [x] Step 2 **[3]** — **ADR-0062, then the fitted offset of a NURBS
  surface.** Write the ADR (design delta above). Then
  `NurbsSurface::offset_fitted`: sample `S + d·N` on each knot span's
  grid, fit a tensor-product B-spline, and refine the spans until no
  sample on a denser grid is farther than the offset fraction from
  `S + d·N`. Past the domain, the end spans' polynomials are extended.
  Refuse a singular normal and a fold by name. Tests:
  - the `to_nurbs` twins of a plane, a cylinder, a sphere and a torus
    patch, each offset both ways, held to `Surface::offset` of the
    original within the fraction;
  - a bicubic bump at a distance short of its least radius fits;
  - past that radius, it is refused;
  - a collapsed row is refused;
  - CTC-05's (3, 2) face at its census radius fits, with its span count
    and time recorded.

  Properties over random poses of the twins.

  **Done (2026-10-13).** ADR-0062 fixes the budget (§1), the fit (§2),
  the extension (§3: the end span carried on, which `eval` already
  evaluates), the chord rule (§4), the ends (§5) and the refusals (§6).
  The fold needed its own variant, `GeomError::OffsetFolds`: the blend
  answers it with `BlendTooLarge`, and `Degenerate` could not tell it from
  a pole. It is a break, with its Python class and changelog line.
  Measured:
  - The twins (plane, cylinder, sphere and torus patches) offset both ways
    to `1e-6` within it of `Surface::offset`. In random poses each shard
    takes about 25 s.
  - Refining per direction matters. Charging a miss to both directions
    took the cylinder's twin to 32 × 32 spans; each direction's own
    residual takes it to 32 × 2.
  - CTC-05's f109, over its domain plus a sixth each side to `τ/4 =
    2.5e-8`, takes 64 × 128 bicubic spans and 4.4 s each way in the test
    profile. Step 3 offsets only the ball's band, since 8192 patches
    would be the tracer's cost otherwise.
- [x] Step 3 **[3]** — **The spine and the contacts.**
  1. Trace the section of the two offsets with `trace_section`: the
     NURBS offset against an analytic one through ADR-0061, or two exact
     offsets through the quadric and torus tracers.
  2. The stretch is the branch holding the ball's centre at each end of
     the edge. Each centre is found in the edge's normal plane by Newton
     from the edge point and located on the branch by ADR-0037 §2's
     search. The stretch runs past each end by the ball's reach, so that
     a face across can cut it.
  3. Each spine sample is snapped by Newton onto the two exact distances
     from the faces (ADR-0062 §1), and each contact is the snapped
     centre's foot on its face: exact on an analytic face,
     `Surface::project` on a NURBS one. The offsets are fitted over the
     band the ball sweeps, not the whole face (step 2's timing).

  Tests:
  - the twin of a plane against a cylinder along a ruling and along a
    circle: the spine within the budget of the closed form's axis line
    and circle, and the contacts within it of the closed form's;
  - CTC-05's NURBS × torus edge: its spine traced, with timing;
  - a property over random poses of the twin pairs.

  **Done (2026-10-14).** `blend/walked.rs`: `spine` finds the ball in
  the edge's normal plane at 17 samples, offsets each NURBS face over the
  band their contacts span widened by the reach (two radii past each
  end) plus a radius, traces the offsets' section and takes the one piece
  within the tolerance of both end balls. `Spine::ball` snaps a traced
  centre by the least Newton step, halved until it shortens the residual.
  Measured and found:
  - **CTC-05's e256 vanishes at v84.** Its faces' normals are 8e-6 rad
    apart there, tangent within the edge's tolerance (7.7e-3, ADR-0045's
    test), and the dihedral grows to about 9° at v115. No ball touches
    both faces exactly in the normal plane at v84, and the offsets' section
    crosses itself there. So an end where the faces are tangent is a
    **vanishing end**: its ball is pinned to the edge's point on the
    common normal and located on the piece within `√(2r·tangency)`, and the
    spine stops there. Steps 4 and 5 take it (below). The spine is traced
    in 0.53 s, so the 60 s question is closed.
  - The snap moves a traced centre by at most `0.02 τ / sin(θ/2)` on
    CTC-05 and the twins, against the bound `τ/2 / sin(θ/2)` the tests
    hold, with `θ` the dihedral there.
  - **The offset's span bound was too low.** A rational twin of a cylinder
    of radius 3 to 10 takes 512 bicubic spans round a whole turn at
    `2.5e-8`. `MAX_OFFSET_SPANS` is now 2048 a direction, with
    `MAX_OFFSET_PATCHES` (65536) on the product: an ADR-0062 §2
    amendment. Each property shard of the twin pairs takes about 13 s.
  - A NURBS isoline the tracer returns exact over a closed direction is a
    loop over clamped knots, so the shared `Piece` now treats it as one
    with its end points equal to rounding. Before, a closed isoline counted
    as open and an open one as a loop of `2π`.
- [x] Step 4 **[3]** — **The rolling-ball surface.** At each spine
  sample, the ball's arc between the two contacts is an exact rational
  quadratic, always two segments, so the `u` structure is fixed. Skin the
  arcs' homogeneous nets along `v` with the section fit (design delta),
  and refine the samples until the surface is within the surface fraction
  of the ball's sphere and both boundary isocurves are within it of their
  faces. The contact edges are the isocurves `u = 0` and `u = 1`: their
  pcurve on the blend is a `Line`, and on each face it is `pcurve_on`.
  At a vanishing end (step 3) the arc collapses to the edge's point, so
  the surface's end isocurve is a pole of it: the arcs are fitted up to it
  and the net's last row is collapsed onto it.
  Tests:
  - the twin pairs of step 3 against the closed form's cylinder and
    torus (radial deviation, and both contacts' tolerance under the
    budget);
  - an S-shaped spine on a bicubic bump, with its span count recorded;
  - a property over random poses.

  **Done (2026-10-14).** `fit_skin` in `arris-geom` (`nurbs/skin.rs`),
  and `Spine::surface`, `arc` and `contacts` in `blend/walked.rs`. The
  surface is held to `τ/4` of every exact arc at its checks, and so its
  contact isocurves to the exact feet. Measured and found:
  - **The vanishing end is no pole.** On CTC-05 the exact balls run
    smoothly to the spine's end at v84 and stop 1.04e-2 from the pinned
    ball. Their arc there is 1.98e-4 wide, its narrowest, with contacts
    9.9e-3 from the edge's point. The faces cross at 2.2e-4 rad there, and
    the edge runs off that crossing within its 7.7e-3. Skinning up to the
    pinned ball left a jump of 1e-2 the fit could not close. So
    `Spine::ball` now snaps at a vanishing end too, the surface ends on
    that narrow arc, and the pinned ball is kept apart (`Spine::pinned`)
    for step 5 (ADR-0062 §5 amendment).
  - **Spans along the spine at `τ/4 = 2.5e-8`:**

    | Stripe | Spans | Fitted in |
    |---|---|---|
    | Twins along a ruling (cylinder) | 4 | — |
    | Plane twin about a circle, whole turn (R 1, r 0.1 / R 2, r 0.5) | 128 / 256 | — |
    | Cylinder twin about the same circles | 454 / 502 | — |
    | Arcs of 2.1 rad, either twin | 146 to 260 | — |
    | S-shaped spine on the bicubic bump | 558 | 0.77 s |
    | CTC-05's e256 | 755 | 1.05 s |

    The skin itself converges at O(h⁴) on a smooth family (127 spans for
    rings of radius `1 + sin(v)/2` at 1e-8 over 6). The balls converge at
    O(h²): as a function of the traced piece's parameter they are only
    C¹. The tracer's walked coordinate switches between arcs and rides the
    fitted offset's knots, so a cylinder's twin takes twice its plane
    twin's spans. A contact on a C² face kinks at its knots too. Skinning
    by the edge's parameter in its normal planes took the S-bump to 213.
    That is a backlog line, not this step: the counts stay inside
    `MAX_FIT_SPANS`, and the cost lands on step 5's traced ends and step
    6's checker.
  - Each property shard of the twin pairs, spine and surface, takes about
    23 s.
- [x] Step 5 **[2]** — **Assemble a walked fillet.** *(Split by this
  commit: the open edge's face-across ends and the closed edge's ring are
  here; the chain junction, the vanishing end and the OCCT-oracled
  synthetic fixtures are step 5b.)*
  - `blend/walked_blend.rs`: `walked_stripe` (a `Stripe` whose section is
    `Section::Walked`, its surface and contact isocurves the skin's),
    `walked_end`, `walked_contacts` and `walked_ring`. `Blend` is
    unchanged: it holds the `Stripe`, and the walked section tells the
    assembly apart.
  - The dispatch tries the table, then the walked stripe, for a fillet
    only: `make_stripes` on the table's `Unsupported`, `classify` for a
    closed edge or a circle `ring::in_row` does not take.
  - Ends: a closed edge has none (its seam is the ball's arc at the
    vertex, the faces' own seams cut to the contacts: `ring::cut_seams`,
    shared with the table's ring). An analytic face across gets
    `traced_end` over the NURBS blend through ADR-0061, then
    `ends::finish_end`, the tail of `face_end` extracted. A NURBS face
    across is `Unsupported`; a fan, a cusp, a vanishing end and any
    vertex shared with another blend are refused by name.
  - `BlendTooLarge` from the spine's `OffsetFolds` and from
    `FaceDomain::side` on the contacts and the end arcs, as now.
  - Provenance is ADR-0007's: a walked ring and stripe are recorded by the
    table's own `record`.

  **Done (2026-10-14).** Tests in `crates/arris-ops/tests/walked_fillet.rs`
  hold the twin property in miniature against the closed form on the
  original: a cylinder's wall or cap as its NURBS twin, either rim, convex
  and concave, agree in volume, area and centroid to `1e-8` with the
  checker clean at `Full` and nothing unchecked but face pairs; a box's
  twinned top face with one top edge filleted (a walked open edge, planes
  across); a radius past the wall's curvature is `BlendTooLarge`; two
  NURBS faces are `Unsupported`. `nurbs_twin_faces` (step 8's primitive)
  landed with them. Measured and found:
  - **The integrator's split cap.** `MAX_INNER_INTERVALS` was 256; a ring
    of the cylinder's twin has 538 knots round it, so its blend face
    integrated to `1e-5` of the part's volume (`0.0098` of `601`). The cap
    is 4096 now (ADR-0062 amendment, `CHANGELOG.md`). Keeping the skin's
    `u` across, two spans, also keeps the inner integral cheap.
  - **Two exact offsets** whose pair has a closed intersection are not
    `trace_section`'s: the spine takes `intersect_surfaces`' curves (FTC-11's
    plane against a cylinder, CTC-02's).
  - **The orientation of a closed edge's face** is read at the ball it is
    measured against, the start's.
  - **The real parts** (`regression/`, still ignored): FTC-10's closed
    cylinder pair builds with the oracle's counts and its volume `3e-9`
    and area `2e-9` relative off, past the fixture's tolerance and under
    1e-6 (the oracle's blend is itself walked, ADR-0015); FTC-11's two and
    CTC-02's now find their spines and stop at the chain junction
    (`VertexBlend`); CTC-01 and CTC-05 stop at a vanishing end
    (`TangentChain`); CTC-05's NURBS × torus offset trace is the first
    edge past the vertex. Three `blend/` fixtures that expected
    `Unsupported` (`chain-through-torus-unsupported`, `tee-cylinders-fillet`,
    `drilled-torus-fillet`) now refuse at the vertex, `vertex-blend`.
- [x] Step 5b **[3]** — **Chain junctions and the synthetic fixtures.**
  *(The vanishing end moved to step 9 by this commit: it unblocks no
  fixture before CTC-01's fan end, which is step 9's, and CTC-05's oracle,
  which is step 10's.)*
  - A tangent vertex where the chain goes on: the walked stripe ends on
    the ball's arc in the vertex's normal plane, its surface's isocurve at
    the end `v`; the chain's next stripe, table or walked, shares the arc;
    `junction.rs` takes `Section::Walked` members (the guard in
    `join_blends` keeps only miters and corners).
  - Fixtures in `blend/` with Open CASCADE's fillet as oracle and the
    checker at `Full`, their operands written by
    `tools/oracle/occt_nurbs_blend.py`: a box whose top is a bicubic patch
    with one top edge filleted; a NURBS cap on a cylinder; a plate with a
    slot whose round end is NURBS, the slot's rim filleted (a chain of a
    table stripe, a walked stripe and a table stripe at two tangent
    vertices); two NURBS faces, `Unsupported`. The Rust tests of step 5
    stay as the closed-form twin checks.

  **Done (2026-10-14).** `Section::Walked` carries its end balls'
  centres, and `Run::at` reads a walked stripe at its `v` end. A Rust twin
  check, the bar whose wall runs into a quarter cylinder with that
  cylinder twinned, agrees with the table's cylinder-into-torus to `1e-8`.
  Measured and found:
  - **The balls are two where a face turns tangent along a curve the
    normal plane crosses.** `chain-through-torus`'s annulus runs into its
    torus along a circle, and the walked ball rides 1.5e-6 off the
    table's at the vertex. A walked run whose points or ball miss is
    `VertexBlend` (ADR-0062 amendment), not the junction's internal fault.
    The fixture stays a refusal; walking the chain as one is a backlog
    line.
  - **A contact on a periodic NURBS face was placed at the wrong turn.**
    `walked_contacts` read the edge's pcurve at the contact's `v`, not at
    the edge's own parameter. On a twinned cylinder that put the contact
    six turns away and refused it as `BlendTooLarge`.
  - **Open CASCADE's walked blend misses the ball.** At 451 samples, Arris'
    blend holds `r` to `1e-8`. Open CASCADE's misses by a median of
    `1.2e-7` (the cap), `2.5e-7` (the slot), `1.5e-5` (the tee), `2.1e-5`
    (the bicubic top, at `1e-8` approximation too) and `1.4e-4` (the
    drilled torus). That is the whole gap in measures, `2e-9` to `3e-6`
    relative. ADR-0062's amendment lets a walked fixture widen its
    tolerances to the oracle's measured miss, up to `1e-5`, with both
    misses in its description. `tee-cylinders-fillet` and
    `drilled-torus-fillet` now build and hold to it; they were refusals at
    the vertex.
  - **A radius past a face's reach** is not an oracle fixture. Open
    CASCADE refuses it too: the cap at r 4.5 and 9, the bicubic box at
    r 4. So no `expected.json` can be written, and step 5's twinned-wall
    test holds `BlendTooLarge`.
  - The census's chains now pass their junctions. FTC-11's `e0` stops at
    `BlendTooLarge(e1, f37)` on a line's contact. Its `e24` stops at a
    line's end on a cylinder across, which the table's ends do not take.
    CTC-02 stops at a sharp corner (`VertexBlend`). Each ignore reason
    says so, for step 10.
- [x] Step 6 **[3]** — **S5 on a walked blend's tangent neighbours.** A
  NURBS blend against the analytic face it touches along a contact has a
  double zero along the isoline `u = 0` (or `u = 1`) to within the fit.
  ADR-0061's isoline search finds it with `near` set to the contact edge's
  tolerance and divides it out, and then the remaining section decides
  the pair. Where the division cannot be made, the pair stays unchecked
  by name. Tests: step 5's fixtures, whose lists of unchecked pairs
  shrink to the NURBS × NURBS pairs only, and a fixture whose blend
  crosses its analytic neighbour away from the contact (a radius past a
  concave neighbour's reach), reported as an S5 violation. The S5 row in
  `docs/DATA-MODEL.md` changes in this commit.

  **Done (2026-10-14).** The plan's case was decided already: a walked
  blend against the face it touches along a contact is ADR-0061's isoline,
  found and divided out at the face's tolerance (the cap, the bicubic box,
  the tee and the drilled torus left only their NURBS × NURBS pair). What
  was refused were the slot's four **junction** pairs, where the walked
  stripe meets the table's on the ball's arc and its surface runs on past
  the arc by the spine's reach:
  - the slot's walls, tangent to the blend at the arc's contact on the
    round end: a saddle on the blend's domain edge, now traced from a
    cell cut at the edge (and, with no isoline, with the parameters
    swapped where the edge is one of `v`);
  - the table's cylinders, tangent to the blend along the arc and agreeing
    with its top contact to fourth order at the vertex: after the arc is
    divided out, a touch of the contact edge that does not enter the
    domain, now an `EdgeTouch`, returned as the `4.1e-4` stretch of the
    edge rounding cannot sign, a tangent isoline piece within the
    tolerance.

  Both are ADR-0061 amendments (2026-10-14). A **convex** junction is not
  decided: on a bar whose wall runs into a twinned quarter cylinder the
  table's cylinder, carried on, enters the walked blend beside the
  corner, a branch hugging the contact edge within rounding that the
  tracer refuses. That pair stays unchecked by name
  (`walked_fillet.rs`'s `the_convex_junction_is_decided`, ignored) with a
  backlog line; step 10's chains (CTC-05, FTC-11, CTC-02) will show which
  kind they meet. The violation fixture is a
  checker test rather than an `ops` result, since the operation refuses a
  radius past a neighbour's reach: a cubic NURBS face tangent to a plane
  along their shared edge and back through it inside the plane's face
  (`arris-check` `full.rs`), with its passing twin cut short. The corpus
  now holds a blend's result to unchecked pairs of two NURBS faces only.
  Open CASCADE's walked blends read back, which miss the ball by up to
  `1.4e-4`, the real parts, and the other results with read B-spline
  faces keep any pair with a NURBS face: the bicubic dome's section and
  fuse and CTC-05's box cuts each leave a plane or a cylinder against a
  read B-spline face that the tracer refuses.
- [x] Step 7 **[2]** — **The walked chamfer.** Each contact lies in the
  edge's normal plane at each sample, at chord `d` from the edge point on
  its face (the ring's rule, ADR-0036 §2, and Open CASCADE's symmetric
  chamfer). It is solved per sample along the face's section with that
  plane and fitted to the budget. The surface is the ruled NURBS (degree
  1 in `u`) between the two contact curves, made compatible, so each
  contact is the surface's own isocurve. Ends and refusals are step 5's.
  Fixtures: step 5's three geometries chamfered, with Open CASCADE's
  chamfer as oracle.

  **Done (2026-10-14).** `blend/walked_chamfer.rs`: `Chord` finds each
  contact by Newton on the face's `(u, v)` (the edge's normal plane and
  the sphere of `d` about the edge's point) and `fit_skin`s the segments
  between them, so no spine and no ball. `walked_blend.rs` reads an edge
  once for both kinds (`Path::Ball` or `Path::Chord`), and the dispatch
  walks a chamfer's `Unsupported` as a fillet's; `Section::Walked`'s
  `centres` is `None` for a chamfer, whose junction with a chain's next
  stripe is the chord. The twin tests (`walked_fillet.rs`) hold a
  cylinder's twinned rim at both ends and both faces, and a box's twinned
  edge, to the table's cone and plane to `1e-8`. The three fixtures
  (`blend/bicubic-top-box-edge-chamfer`, `nurbs-cap-cylinder-rim-chamfer`,
  `nurbs-slot-plate-rim-chamfer`) agree with Open CASCADE's chamfer to
  `1.4e-7`, `4e-9` and `2.5e-9` relative in volume. Measured and found:
  - **Open CASCADE's contact misses the chord** on a B-spline face by a
    median of `2.8e-7` (the bicubic top; `5.5e-6` at most) and `1.9e-7`
    (the cap); on a plane or a cylinder it holds it to rounding.
  - **A NURBS edge is carried on past its ends** on its end spans'
    polynomials: the bicubic top's wall edges are not square to the walls
    across, so a contact stopping at the edge's end never reaches them.
  - **Open CASCADE's own STEP of the cap's chamfer** is a walked B-spline
    face that Arris' reader does not finish in minutes, like its fillet's
    (`analytic.occt_step_unread`).
  - **The table's `Unsupported` chamfers are walked now.** The plane
    against a cylinder along a ruling, which `chamfer.rs` pinned as
    `Unsupported`, builds at the chord: `blend/d-chord-edge-chamfer` agrees
    with Open CASCADE to 9 digits (the arc rule would differ by `3e-5`),
    and the test now holds the closed form. The backlog line becomes the
    plane's exactness (the face is a NURBS surface, not a `Plane`). The
    rest of the blend and regression corpus (109 fixtures) is unchanged.
- [ ] Step 8 **[2]** — **The twin property.** `nurbs_twin_faces` (landed in step 5) twins
  one face of an edge. For each table row (plane × plane, plane ×
  cylinder along a ruling and a circle, cone, sphere and torus coaxial),
  in random poses and at random sizes, the fillet and chamfer of the
  twinned edge agree with the closed form on the original in volume,
  area and counts, or both refuse. `prop_shards!`, 256 cases at
  retirement. Each failure found becomes a `regression/` fixture.
- [ ] Step 9 **[2]** — **The analytic pairs with no closed form.** The
  walked stripe over two exact offsets, one fixture each against Open
  CASCADE:
  - CTC-01's crease (two equal cylinders crossing), fillet only;
  - a torus against a cylinder off its axis;
  - a cylinder against a sphere;
  - the B-spline edge between a plane and a cylinder that step 1 shrank.

  CTC-01 needs its two ends: a fan of two planes at v113, and at v211 a
  **vanishing end** (moved here from step 5b, open question below), where
  the faces turn tangent within the edge's tolerance. CTC-05's v84 is the
  same end.

  The backlog's lines 47 and 50 are updated or removed in this commit.
- [ ] Step 10 **[2]** — **The real parts.** Re-run `tools/real-parts.sh`
  on the fetched tier and `--census-committed`. Move every step 1
  fixture that now agrees out of `regression/` into `blend/` with its
  blessed dump. Name each that still refuses by its other item. Print the
  fillet column's NURBS row before and after, with timings, into this
  plan.

## Acceptance

- The corpus: every fixture from steps 5, 6, 7 and 9 and every step 1
  fixture this plan moves out of `regression/` passes against Open
  CASCADE's measures within its tolerances, with the checker at `Full`
  and nothing unchecked but NURBS × NURBS face pairs. Every existing
  `blend/` fixture passes against its blessed dump unchanged, since the
  table's results do not move.
- The twin property (step 8) and steps 2–4's properties are green at 256
  cases (`ARRIS_GATE=full`).
- `tools/real-parts.sh` (both tiers): the fillet column's NURBS rows are
  each agreeing or refused as another item's (NURBS × NURBS, a vertex
  blend), never a fault or a panic. The numbers are printed.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Operations, blends: the walked stripe beside
  the table and what it takes.
- `docs/DATA-MODEL.md`: §Surfaces (the fitted offset beside
  `Surface::offset`, done in step 2), §Invariants S5 (step 6, if not already done in its
  commit), and §Provenance if a walked end adds a record.
- `docs/ROADMAP.md` §C10: the first item's blend half done, with the
  fillet column's numbers.
- ADR-0007 and ADR-0037: amendment lines pointing to ADR-0062 (done in
  step 2).
- `docs/BACKLOG.md`: lines 47, 50 and 70 updated or removed. New lines
  for the vertex cases a walked stripe refuses, the NURBS face across,
  and any timing found.
- `CHANGELOG.md` `Unreleased`: fillet and chamfer on a free-form face,
  and on the analytic pairs with no closed form, plus the refusals a user
  will hit.
- `crates/arris-py`: `fillet`/`chamfer` docstrings (what they take).
  Step 2's `GeomOffsetFoldsError` is bound and stubbed in its commit.
- `AGENTS.md` current state: C10's first item done.

## Open questions

- The error budget: **decided** in ADR-0062 §1. A quarter of the stripe's
  tolerance goes to the offset fit and the section's own quarter to the
  spine, with each sample snapped onto the exact distances. A quarter
  goes to the surface, so a contact edge holds the stripe's tolerance
  without growth. Steps 3 and 4 measure it.
- The extension past the domain: **decided** in ADR-0062 §3, the end
  span carried on. Step 1 found it needed: CTC-05's ball reaches past
  both ends by r, a tenth of the domain's width.
- ⚠ OPEN: what holds CTC-05's two fixtures, since Open CASCADE builds an
  invalid body in both editions (step 1). A walked fixture whose oracle is
  only inaccurate is settled by ADR-0062's step 5b amendment; an invalid
  body is not. The candidates are the two
  editions agreeing with each other and the part's own volume less a
  fillet deficit integrated independently along the spine; there is no
  closed form. **Agent**, at step 10, with ADR-0015's evidence where the
  oracle is set aside.
- ⚠ OPEN: the chamfer is placed by the chord in the edge's normal plane,
  not by an offset. This matches the ring's chamfer and Open CASCADE's
  symmetric one. **Agent**, at step 7, unless the human meant a chamfer
  defined by the offsets.
- ⚠ OPEN: step 9 (the analytic pairs with no closed form) widens the
  plan beyond NURBS faces. It is in because CTC-01 and the B-spline edges
  are in the census and the stripe takes them for free. **Human**, before
  `/work` reaches step 9: keep it, or cut it to a follow-up plan.
- ⚠ OPEN: how a vanishing end meets its vertex (step 4). The candidates
  are growing the vertex's tolerance to the end arc's contacts (9.9e-3 on
  CTC-05, past the edge's 7.7e-3) under ADR-0060's operation bound, or
  refusing the end by name. Step 5 refuses it (`TangentChain`).
  **Agent**, at step 9 (moved from 5b, where it unblocked nothing).
- The spine's trace on CTC-05 against the ADR-0026 §6 budget (60 s):
  **closed** at step 3, which traces it in 0.53 s.
