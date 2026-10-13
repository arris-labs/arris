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
    uses next. Its name and signature are fixed in step 4.
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
- [ ] Step 3 **[3]** — **The spine and the contacts.**
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
- [ ] Step 4 **[3]** — **The rolling-ball surface.** At each spine
  sample, the ball's arc between the two contacts is an exact rational
  quadratic, always two segments, so the `u` structure is fixed. Skin the
  arcs' homogeneous nets along `v` with the section fit (design delta),
  and refine the samples until the surface is within the surface fraction
  of the ball's sphere and both boundary isocurves are within it of their
  faces. The contact edges are the isocurves `u = 0` and `u = 1`: their
  pcurve on the blend is a `Line`, and on each face it is `pcurve_on`.
  Tests:
  - the twin pairs of step 3 against the closed form's cylinder and
    torus (radial deviation, and both contacts' tolerance under the
    budget);
  - an S-shaped spine on a bicubic bump, with its span count recorded;
  - a property over random poses.
- [ ] Step 5 **[2]** — **Assemble a walked fillet.**
  - `blend/walked.rs`; `Blend` holds either stripe.
  - The dispatch tries the table, then the walked stripe for a NURBS ×
    analytic pair, for an analytic pair outside the table, and for a
    table pair whose edge is not the table's curve.
  - Ends: a closed edge has none. An analytic face across gets
    `traced_end` over the NURBS blend through ADR-0061. A NURBS face
    across is `Unsupported`, and a sharp vertex with another blend keeps
    its refusal.
  - Chains (step 1's census): a tangent vertex where the chain goes on is
    a junction. There the walked stripe ends on the ball's arc in the
    vertex's normal plane, exact, the spine's end pinned to that ball's
    centre. The chain's next stripe, from the table or walked, shares the
    arc, and `blend/chain.rs` takes walked members beside the table's.
  - `BlendTooLarge` comes from `FaceDomain::side` on both contacts and
    the end arcs, as now.
  - Provenance is ADR-0007's.

  Fixtures in `blend/`, each with Open CASCADE's fillet as oracle and the
  checker at `Full`, with the pairs `check_leaving_nurbs` allows left
  unchecked:
  - the rim of `nurbs-bicubic-dome-fuse`'s dome on its plane (closed
    edge);
  - a box whose top is a bicubic patch, one top edge filleted (a NURBS ×
    plane edge with two plane faces across);
  - a NURBS cap on a cylinder (a NURBS × cylinder ring);
  - a plate with a slot whose round end is the NURBS twin of its
    cylinder, the slot's rim filleted: a chain of a table stripe, a walked
    stripe and a table stripe meeting at two tangent vertices;
  - a radius past the dome's least radius, refused `BlendTooLarge`;
  - two NURBS faces, refused `Unsupported`.
- [ ] Step 6 **[3]** — **S5 on a walked blend's tangent neighbours.** A
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
- [ ] Step 7 **[2]** — **The walked chamfer.** Each contact lies in the
  edge's normal plane at each sample, at chord `d` from the edge point on
  its face (the ring's rule, ADR-0036 §2, and Open CASCADE's symmetric
  chamfer). It is solved per sample along the face's section with that
  plane and fitted to the budget. The surface is the ruled NURBS (degree
  1 in `u`) between the two contact curves, made compatible, so each
  contact is the surface's own isocurve. Ends and refusals are step 5's.
  Fixtures: step 5's three geometries chamfered, with Open CASCADE's
  chamfer as oracle.
- [ ] Step 8 **[2]** — **The twin property.** `nurbs_twin_faces` twins
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
  invalid body in both editions (step 1). The candidates are the two
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
- ⚠ OPEN: if the spine's trace on CTC-05 runs past the ADR-0026 §6
  budget (60 s), profile first or record and continue. **Agent**, at
  step 3. The default is to record it and add a backlog line beside
  ADR-0061's.
