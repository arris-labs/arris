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
    planes, which step 9b takes or names.
  - **The oracle.** Open CASCADE's CTC-05 body is invalid in both
    editions and its measures cannot hold the fixture (open question).
    The other five agree with a fillet in dV/dA ≈ r/2 or are within the
    quadrature's noise (FTC-11, a 5135 mm³ part).

  Step 10's acceptance list is these seven fixtures. Steps 5, 9 and 9b build
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
  fixture before CTC-01's fan end, which is step 9b's, and CTC-05's oracle,
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
- [x] Step 8 **[2]** — **The twin property.** `nurbs_twin_faces` (landed in step 5) twins
  one face of an edge. For each table row (plane × plane, plane ×
  cylinder along a ruling and a circle, cone, sphere and torus coaxial),
  in random poses and at random sizes, the fillet and chamfer of the
  twinned edge agree with the closed form on the original in volume,
  area and counts, or both refuse. `prop_shards!`, 256 cases at
  retirement. Each failure found becomes a `regression/` fixture.

  **Done (2026-10-14).** `twin_prop.rs`: three `prop_shards!` families at
  256 cases, in a random pose, fillet or chamfer chosen, either face the
  twin: a box's edge (planes), a half disc's chord (a plane against a
  cylinder along a ruling) and a turned part's corner (the meridian row:
  plane, cylinder, cone, sphere, torus). All pass. Measured and found:
  - **A loop did not close to the bit.** A part posed 100 from the origin
    failed L3 on the walked ring: the ball at the loop's two ends is found
    twice and the two differ by the snap's rounding, so the surface's first
    and last rows of control points were not the same and the face was not
    closed. The skin now takes the start's ball for both ends (ADR-0062
    amendment). The same for a closed chamfer's chord.
  - **A fillet on a NURBS face against a coaxial cylinder, cone or sphere
    is refused.** The offsets' section is a circle about the shared axis and
    ADR-0061's tracer reports its turning points as ones it cannot tell
    apart; the cylinder's twin against a cone finds no piece holding the
    ends' balls. A chamfer there builds. The property rejects the pair
    (`walked-fillet-coaxial-nurbs`) and two ignored tests beside it hold the
    shrunk cases (`a_twinned_cylinder_fillets_against_a_cone`, `..._a_sphere`),
    with a backlog line. Every pair the strategy draws that the rejection
    does not take is held, fillet and chamfer; how many draws it takes is
    not counted.
- [x] Step 9 **[2]** — **The analytic pairs with no closed form.** The
  walked stripe over two exact offsets, one fixture each against Open
  CASCADE:
  - a torus against a cylinder off its axis (`blend/drilled-torus-fillet`,
    landed in step 5b);
  - two cylinders crossing (`blend/tee-cylinders-fillet`, step 5b);
  - a cylinder against a sphere (`blend/sphere-cylinder-fillet`);
  - the B-spline edge between a plane and a cylinder
    (`blend/bspline-rim-fillet`).

  *(CTC-01's crease, with its two ends, moved to step 9b by this commit;
  see below.)* The backlog's lines 47 and 50 are updated in this commit.

  **Done (2026-10-14).** Decided at the start (the human's open question,
  delegated): step 9 stays in the plan. The first two fixtures already
  stood from step 5b. Measured and found:
  - **The sphere against a cylinder off its axis builds** when the loop
    crosses the sphere's seam square: the half disc of a ball cut flat at
    `z = ±1.5` (no pole, whose edge counts Open CASCADE writes otherwise)
    and a cylinder along `+x` with its axis at `(0, 0, 0.5)`, in the plane
    of the seam. Open CASCADE's walked blend misses the ball by 8.6e-6 at
    the median of 3042 samples, 4.1e-5 at most; its volume lies 5.3e-8
    relative and its area 2.7e-7 from Arris'.
  - **A slanted crossing of the seam was a `kernel bug`.** With the axis at
    `(0, 0.5, 0)` the loop meets the seam at a slant, and the junction's
    arc, in the vertex's normal plane, ends on a contact that is not on the
    seam; `cut_corner` found no corner edge through it. A walked junction
    now refuses that case `VertexBlend` (`ends::on_corner_edge`), and
    `regression/sphere-cylinder-oblique-seam-fillet` holds the build, with
    a backlog line: the seam wants cutting where the contact crosses it.
  - **The B-spline edge needs no new code.** A rim written as the exact
    rational quadratic B-spline of the half circle
    (`tools/oracle/occt_nurbs_blend.py bspline-rim-plate`) falls to the
    walked stripe as "a table pair whose edge is not the table's curve",
    ends on the chord's plane, and agrees with the torus's closed form to
    2.6e-14 relative in volume. Open CASCADE walks it too, 6.9e-8 below
    the closed form in volume and 4.8e-8 in area.
  - **FTC-11's excerpt is not this fixture.** Its slot has a 2.3e-3 gap
    between a plane and the cylinder it should be tangent to, a 7° kink at
    the chain's far vertex; the lone line there is a table stripe whose
    end arc, across a cylinder at that kink, leaves the face. It is step
    10's real part, not a pair without a closed form.
- [x] Step 9b **[3]** — **CTC-01's crease and its two ends.** Two equal
  cylinders crossing, fillet only, against Open CASCADE (`real/nist-ctc-01`
  as the fixture's file). The spine is found (an ellipse); both ends are
  refused today:
  - v211, a **vanishing end** (moved here from step 5b, open question
    below), where the faces turn tangent within the edge's tolerance;
    CTC-05's v84 is the same end;
  - v113, a **fan** of two planes across, which the walked stripe's end
    refuses by name.

  Split out of step 9 by that commit's finding that each end is a design of
  its own (a vanishing end pins the ball to the edge's point; a fan cuts
  the end curve at the fan edge), which is not a fixture on a pair. May
  rewrite the backlog's CTC-01 line and CTC-05's expectations in step 10.

  **Done (2026-10-14).** `blend/nist-ctc-01-crossing-cylinders-crease-fillet`
  builds and holds to Open CASCADE, checker clean at `Full` with nothing
  unchecked. Measured and found:
  - **v113 is no fan.** The ellipse's tangent there is vertical, as is
    `e130`, the corner between the two planes, and each plane is tangent to
    one cylinder: a tangent vertex of four edges (ADR-0039 §1). Filleting
    `e130` alone walks the chain back up the crease. Step 5b's junction
    takes it, and Open CASCADE runs the fillet on down `e130` as a
    cylinder too. Step 1's census read the vertex's degree, not its
    tangency.
  - **v211 is a pole** (ADR-0062 amendment). The cylinders are tangent there
    exactly, and the end ball touches both at the vertex to `4.3e-14`. The
    skin's row there collapses onto it, the contacts end at the kept vertex,
    and the loop crosses the row on a degenerate edge (`EndKind::Pole`), as
    Open CASCADE's blend does. CTC-05's `v84` stays `TangentChain`: its
    end arc is `1.98e-4` wide and `9.9e-3` from the vertex (the open
    question below, decided).
  - **S5 needed two tracer changes** (ADR-0061 amendment). Against each
    cylinder the contact and the pole are isolines both ways; the pole is
    now divided out beside the contact and returned as a point. Against the
    wall plane at the junction, the fit put the saddle on the contact edge
    `5e-9` past the domain; critical points in a slab past an open edge
    within the tolerance of it are now put on it.
  - **Open CASCADE's contacts miss the ball** by `5.3e-4` at the median and
    `2.2e-3` at worst (Arris': `5.0e-10`, `7.8e-10`), so the blend faces'
    areas differ by `8.6e-4` relative and the part's by `5.5e-9`; the
    fixture's tolerances are `1e-8`. Its counts are Arris' own: Open
    CASCADE's healed reading of the part has two vertices and two edges
    more (`counts_differ`).
- [x] Step 10a **[3]** — **CTC-05 AP242's spine.** Step 10's run (2026-10-14)
  found `regression/nist-ctc-05-ap242-nurbs-torus-fillet` refused
  `Unsupported(NURBS × torus)` from `Refusal::NoSpine { why: "no piece holds
  the ends' balls" }`: the offsets' section is traced, but none of its pieces
  passes the end balls (the edge is an 11-span B-spline `1.6e-3` off the best
  curve, on one Bézier patch). The AP203 edition gets past this point.
  First measure: print the pieces' distances to each end ball against the
  budget (ADR-0062 §1) and say whether the pieces are cut short at the
  domain boundary, the edge's samples are off the exact offsets, or the
  match tolerance is the edge's alone. Then fix it in the walk, never by
  widening a tolerance. Settle the CTC-05 oracle's open question here: AP203
  also stops at `v84` (`TangentChain`, step 9b), so the pair of fixtures is
  held by the two editions agreeing and an independent spine integral, or
  one is re-scoped to the edge up to the vanishing end. ADR-0062 amendment.

  **Done (2026-10-14).** Measured: the AP242 edge's end at `v80` (AP203's
  `v84`) is a vanishing end, and of the two pieces traced the one through
  the far end's ball passed `2.6e-2` from the pinned ball, against an
  allowance `√(2r·τ)` of `2.1e-2` with `τ` the edge's `2.4e-4`. Of the three
  causes, it was the third:
  - the piece is not cut short: the end sits at `t = 1.80` of `[0, 3.24]`;
  - the edge's samples are on both faces, to `8.7e-6` at `v80`, the faces
    tangent there to `r·sin = 5.5e-5`;
  - the match tolerance was the edge's alone, while the file gives the
    vertices `2.1e-2` (AP203's edge and vertices share `7.7e-3`).
  A vanishing end is now held to `√(2r·τ)` with `τ` the wider of the
  tangency and the end vertex's tolerance (`Rolling::ends`), as step 9b's
  pole already reads the vertex's. AP242 then stops where AP203 does,
  `TangentChain`: the exact ball's arc is `5.5e-4` wide and `2.5e-2` from
  the vertex, past its tolerance. Both fixtures move to `blend/` as
  `tangent-chain` refusals, which settles the open question below.
- [x] Step 10b **[3]** — **Chain junctions on a walked member.** Three
  fixtures refuse at a tangent vertex where the chain goes on:
  - FTC-11 `bspline-circle`: "the blend leaves its face through an edge that
    is not the corner's own" (`e1`, `f37`);
  - FTC-11 `slot-chain`: "no closed form for `e24` against `f17`", a
    cylinder × cylinder end;
  - CTC-02 `ellipse-chain`: "the corner at the edge's end is one the closed
    forms do not cover" (`e184`, `e274`, `v167`).
  Reduce each to its smallest failing junction (walked beside a table
  stripe, table beside walked), then fix step 5b's junction for it or name
  the kind of vertex it is as another item's. Depends on 10a only if the
  match tolerance there is shared. ADR-0062 amendment.

  **Done (2026-10-14).** None of the three is a step 5b junction fault:
  - **bspline-circle builds.** Its chain runs along the line e1 to v2, a
    convex 7.3° kink where f38 stands `2.3e-3` off tangent to the boss
    f37. The end arc across f37 meets the rim at a grazing angle inside
    the band its read pcurves stand off by (`3.75e-3`, the rim's
    tolerance). The side test now takes a sample within a boundary edge's
    tolerance of it as on it, and a cut read edge's pcurves are ended on
    the blend's exact curves there (L2). Fast clean, Open CASCADE's
    volume, area and centroid. The counts are ADR-0038's lengthened corner
    edge. The S5 rows go to steps 10c and 10d.
  - **slot-chain's v24 is a near-tangent vertex.** A convex 3.9° kink
    where the hole f17 runs on from f18, tangent within the vertex's
    `3.6e-3` but not the faces'. The hole curves away, so the contact
    misses it. It is refused `TangentChain(e24, e60, v24)`, where it was
    `Unsupported(cylinder × cylinder)`. Open CASCADE chains through by its
    0.1 rad literal. Backlog line.
  - **ellipse-chain's v167 is a skew junction.** f15 and f40 are tangent
    along the ruling e183, which leans out of the ball's normal plane.
    The runs' balls agree to `2.1e-4`, but their side contacts lie 0.06
    off e183 and `9.1e-3` apart. It stays `VertexBlend`, joined to the
    backlog's chain through a torus.

  ADR-0062 amendment. The match tolerance was not shared with 10a.

- [x] Step 10c **[3]** — **A saddle at a corner of the walked blend's
  domain.** *(Split from the old 10c by its 2026-10-14 measurement.)*
  `regression/nist-ftc-10-cylinder-pair-bspline-fillet` builds but the
  checker leaves eight pairs undecided (`f93`–`f96`, `f105`, `f106`,
  `f181`, `f182`, each against the NURBS blend face `f183`). `f183` is an
  open 5 × 12 cubic patch, not closed, so its four corners are real. In
  seven of the eight (the cylinders of radius 1 and 8, the two spheres)
  the refusal is `CrowdedSingularity` from `Tracer::singular_cell`, with no
  isoline divided out. The singular point is a corner of the domain, `u` at
  an edge and `v` at one within `1e-12`: `F ≈ A·dv² − B·du²` there
  (`A = 4.8`, `B = 31`), a saddle whose one arm runs into the quadrant at
  `dv = 2.5·du`. Step 6 cut the cell at an edge of `u` (`sides`); every
  cell the corner needs reaches past an edge of `v` too, and `fits` fails
  at all sixteen halvings, in either orientation, so swapping the
  parameters cannot help. The cell is cut at both edges (its arm that
  survives is the one inside the quadrant, and the march and the assembly
  take one arm of a crossing, not a pair), or the corner is shown to be
  an isolated touch where it is. `f94` (a cylinder of radius 6) is another
  refusal, `UnresolvedTurning`, not measured. Acceptance: the eight pairs
  decided or each refused by the name of the corner it is, and the fixture
  at `Full` with only NURBS × NURBS left.

  **Done (2026-10-14).** The cell is cut at both edges (`SingularCell::rows`
  beside `sides`), and the one arm inside the quadrant is followed; seven
  pairs decide. `f94` was the same corner: an uncertified box of turning
  points with the saddle on its edge, which the absorption test decided by
  rounding and sent to the edge touch (`UnresolvedTurning`); it is now the
  saddle's within `EDGE_TOUCH`. The checker passes at `Full`; the fixture
  stays ignored on its oracle row (volume 3e-9, area 2e-9 relative), for
  step 10. ADR-0061 amendment; the test
  `a_saddle_at_a_corner_of_the_domain_keeps_the_one_arm_inside_it` fails
  with the change taken out.
- [x] Step 10d **[3]** — **A convex junction's branch against the
  tracer's resolution.** `regression/nist-ftc-11-ap242-bspline-circle-fillet`
  leaves `f48` against `f49` (`UnresolvedTurning`) and `f50`
  (`CrowdedSingularity`), the two table stripes' cylinders at the chain's
  ends. This is the convex junction step 6 left open (a 7.3° kink), not
  a fault of its S5 rule. Measured on `f49`, the quotient section after
  the arc (an order-2 isoline at `v = 0.363`, swapped) is one branch,
  a parabola `v ≈ c·s²` (`s` the distance from the arc, `c ≈ 1.5`) from
  the start of the blend's domain to its contact edge `u = 1` at 38% of the
  way along, tangent to the edge `v = 0` at the arc's foot. Past
  the arc the table cylinder lies inside the blend face's carried-on
  surface, so it is outside `f49`'s domain and the checker would excuse
  the branch; the tracer must still return it. Two causes, in order:
  - the branch's turning point on the contact edge `u = 1` is `1.6e-11`
    inside the edge, past `EDGE_TOUCH` (`1.4e-12`), so `turning_cell`
    cuts a cell at a row `1.6e-11` from the point and the sign test on
    that row is rounding's. A point is on the edge when `c·d²` is under
    `ANGLE_SLACK` (`d ≤ 1e-6` here): tried in `turning_cell` and it moved
    the failure on, it did not fix it, so it was not kept;
  - past that, the march toward the arc's foot stops at `v = 5e-6`,
    `s = 1.9e-3` (a length of about `4e-4`) because the sign of `F` on the rows `v ± 1.5e-6` is
    `±1.2e-7`, under the coefficients' floor; the edge row's two roots
    (`0.36270` and `0.36389`, the dip between them `−4e-8`) are one
    candidate to `edge_candidates`, and only the left one becomes an edge
    cell, which a march from the right (`dir = −1`) cannot be captured by.

  The branch within a tolerance of the foot of the arc is not the tracer's
  to resolve, and a march that gets there with no cell to be captured by
  should end at the foot: how far that is allowed to go is the decision
  (ADR-0061's "put on the edge when within `tol.linear`" is the
  precedent for critical points). `f50` is a different refusal on the
  same junction: an uncertified critical box `0.0019 × 0.098` against the
  `v = 0` edge, `1e-4` from the order-1 isoline, refused as a point too
  flat to decide (`singular_points`). Decide it with the same rule.

  **Done (2026-10-14).** Both pairs decide; four changes, each needed:
  - **f49's turning point is on the contact edge.** `∂f/∂v` vanishes along
    the whole edge, where the blend and the cylinder share the face's
    tangent plane. A point is on an edge of `v` also where the fold's move
    `c·d²` is under `ANGLE_SLACK` (`c = 0.023`, so `d ≤ 1e-6`).
  - **The march runs aground** (the decision asked for). Where no step is
    proven and the line of constant `u` down to the edge is within
    `tol.linear` of the cylinder all the way, the branch follows the edge
    line, held to the tolerance, to an edge cell whose arm points back,
    the isoline's foot, or a root by bisection, and is clipped there. f49's
    two arms end at the edge's two roots, `0.36270` (captured by its edge
    cell) and `0.36389` (the root the one candidate did not split).
  - **f50 is a tangency, not a flat point.** The fit crosses the cylinder
    twice along the end arc, `8.4e-4` of `u` apart, within `1.5e-9`
    between. The bracket saw one crossing, so the isoline was divided at
    order one and left the second as a line of turning points the whole
    height of the domain; past the flat box that was the refusal. A
    crossing whose slope-zero line a Newton step away is held to the
    tolerance and tangent is divided out there at order two, and both
    arms then meet at the foot.
  - **Each patch keeps its own floor.** The order-2 division raised the
    section's floor 400 times (`3.7e-9`), and the undivided patch beside,
    where `f` falls with the square of the way to the line, could not sign
    a step `0.09` of `u` away. Dividing that patch too, as a torus's column
    within `DIVIDED_WITHIN` is, was tried and rejected: a patch's
    polynomial past its span does not vanish on the line, and the torus
    twin's branches went `0.14` off.

  ADR-0061 amendment. The fixture test fails with any of the four taken
  out. Past the checker the fixture meets its oracle row (step 10e).
- [ ] Step 10e **[2]** — **The FTC-10 and FTC-11 fixtures leave
  `regression/`.** 10c and 10d decided their pairs; both now stop at the
  oracle row. FTC-10 is `3e-9` volume and `2e-9` area relative off Open
  CASCADE's walked blend, for ADR-0062's step 5b amendment to cover once
  its miss of the ball is measured. FTC-11 holds to Open CASCADE's
  measures in process, but Open CASCADE reading Arris's STEP finds area
  `1.2e-2` more (`2e-6` relative, about the walked face's own area) and
  volume `6e-8` relative: find which face it reads differently, and fix the
  writer or fence it by name. Then move both fixtures into `blend/` with
  blessed dumps, drop the `#[ignore]`s in `corpus.rs` and tick the S5 rows
  in step 10b's note. The test
  `a_read_rims_pcurves_standing_off_hold_an_end_arc` (`arris-ops/tests/fillet.rs`)
  reads the FTC-11 fixture by its path and follows it out of `regression/`.
- [ ] Step 10 **[2]** — **The real parts.** Re-run `tools/real-parts.sh`
  on the fetched tier and `--census-committed`. Move every step 1
  fixture that now agrees out of `regression/` into `blend/` with its
  blessed dump. Name each that still refuses by its other item. Also first: CTC-02 AP242 `drill_x`'s counts disagree with the oracle (643 faces
  against 649, edges 1446 against 1452) in the 2026-10-14 fetched run and
  are on no waits line; find whether a step of this plan caused it, and
  fix it or fence it as the nightly rule says. `real_nist_ftc_10` fails
  at `main` since this plan's blends: #6434's committed fillet record is
  still `Unsupported(NURBS curve × cylinder surface)`, blocks `NURBS`,
  where the run now finds a blend network's; re-record it. Print the
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
- What holds CTC-05's two fixtures, since Open CASCADE builds an
  invalid body in both editions (step 1): **decided** at step 10a. Both
  editions are refused `TangentChain` at the same vanishing end, so each
  fixture holds that refusal in `blend/`, and the invalid bodies'
  measures stay as the record. The candidates below pass to the backlog
  line that joins such an end. The question as first posed: A walked fixture whose oracle is
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
- **Decided** (2026-10-14, delegated to the agent): step 9 (the analytic
  pairs with no closed form) stays, though it widens the plan beyond NURBS
  faces: the stripe takes them for free, and CTC-01 and the B-spline edges
  are in the census.
- How a vanishing end meets its vertex: **decided** at step 9b (ADR-0062
  amendment). Where the end ball touches both faces at the vertex, the
  stripe ends on a pole there; where its arc stays apart (CTC-05's, `9.9e-3`
  from the vertex past the edge's `7.7e-3`), the end is refused
  `TangentChain`, since joining it would take the surface off the exact
  balls by more than the budget.
- The spine's trace on CTC-05 against the ADR-0026 §6 budget (60 s):
  **closed** at step 3, which traces it in 0.53 s.
