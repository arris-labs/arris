# ADR-0062 — A blend with no closed form is a walked stripe: the spine is the section of the two faces' offsets, the surface fitted through the ball's exact arcs

- Status: accepted (2026-10-13)
- Plan: `nurbs-face-blend` step 2 (the decision and the fitted offset);
  the spine and the contacts land in its step 3, the surface in step 4,
  the assembly and the chain junction in step 5, the chamfer in step 7
- Amends: ADR-0007 §Decision ("every surface is exact; nothing is fitted
  but a pcurve") and ADR-0037 §1 ("a fitted *surface* stays ADR-0007's
  rejected alternative … until the NURBS cycle")
- Follows: ADR-0061 (the NURBS section traced in the NURBS surface's
  (u, v), the domain's edge ending a branch), ADR-0018 and ADR-0019 (the
  quadric and torus tracers, `SECTION_FIT_FRACTION`), ADR-0022 (two fits
  of one section within twice the fraction), ADR-0035 (a stripe follows a
  chain through a tangent vertex), ADR-0036 §2 (the ring's chamfer at the
  chord)
- Reference trees read: none. Open CASCADE is run as the oracle
  (`BRepFilletAPI_MakeFillet`, `BRepFilletAPI_MakeChamfer` through
  `tools/oracle/`), not read

## Context

ADR-0007 builds a blend from a table of closed forms: two planes, a plane
and a cylinder along a ruling or a circle, the coaxial pairs of ADR-0036.
Every other pair is `Unsupported`. The plan's census (step 1) finds what
that leaves in the real parts' fillet column:
- CTC-05's rational B-spline face against a torus, in both editions;
- FTC-10's two cylinders with skew axes, meeting in a quartic;
- CTC-01's two equal cylinders whose axes meet, an elliptic crease;
- FTC-11's and CTC-02's plane-against-cylinder edges written as
  B-splines, hiding circles and ellipses. The ellipse has no closed form
  either: the plane is oblique to the cylinder.

Four of the five parts reach a tangent vertex where a chain goes on, so
a stripe with no closed form has to meet a table stripe there. CTC-05's
edge ends on its B-spline's domain boundary at both ends, and the ball
reaches a tenth of the domain's width past it.

A rolling ball of radius `r` touching two faces has its centre on both
faces' offsets by `r` toward the ball. The locus of centres, the spine,
is the section of the two offsets. An analytic surface's offset is exact
(`Surface::offset`), so two analytic faces give a section the quadric and
torus tracers already trace. A B-spline face's offset is no B-spline. The
NURBS tracer (ADR-0061) traces a B-spline against an analytic surface, so
a fitted offset is all it lacks.

## Decision

**A blend the table has no closed form for is a walked stripe.** Its
spine is the section of the two faces' offsets toward the ball, traced
by `trace_section`. Its contacts are the spine's feet on the faces. Its
surface is a B-spline fitted through the ball's exact arcs.

**1. The error budget.** Let `τ` be the stripe's tolerance, the faces'
as the table's stripe takes it (`faces_tolerance`, or the edge's where
wider). Then:
- **The offset fit** holds a B-spline face's offset within `τ/4` of
  `S + r·N`, at the same `(u, v)`.
- **The spine's section** is the tracer's own: its branch, fitted under
  `SECTION_FIT_FRACTION` (a quarter) of `τ` against the fitted offset.
- **Each spine sample is then snapped**: Newton on the two exact
  distances `dist(c, F₁) = dist(c, F₂) = r`, from the traced point, by
  the faces' own projections. So the ball at every sample touches both
  true faces, its arc is an exact rational quadratic between two exact
  feet, and neither the offset fit nor the section fit reaches the
  stripe's shape. Those two decide only which branch is walked and where
  its samples start. That is why a quarter each is enough: any fraction
  that keeps the snap inside its basin would do, and a quarter is the
  section's own.
- **The surface fit** holds the surface within `τ/4` of the exact arcs,
  and its two boundary isocurves within `τ/4` of their faces. This is
  `SECTION_FIT_FRACTION` again, for the reason a section's 3D curve
  takes it: each contact edge's pcurve is fitted afterwards
  (`pcurve_on`) at half the edge's tolerance, and the quarter leaves it
  the rest.

So a contact edge holds the stripe's tolerance without growth: it is
the surface's own isocurve, within a quarter of `τ` of its face, its
pcurves within the edge's. Steps 3 and 4 measure it.

**2. The fitted offset** (`NurbsSurface::offset_fitted`). It is
non-rational, with degree `max(3, p)` by `max(3, q)`.
- It is fitted over a parameter box the caller names, by the tensor
  product of the curve fit's least squares (§9.4 of *The NURBS Book*,
  each direction's ends interpolated).
- Each interior knot of the face inside the box is a knot of the fit,
  one continuity order lower, since the normal is a derivative. A
  twin's double knot, `C0` in its parameter and smooth in space, is
  a `C0` joint of the fit, and the offset of a crease tears, which the
  fit cannot meet and refuses as `Fit(Diverged)`.
- It refines per direction. Each sample line is fitted alone along its
  direction and held to half the margin there, and a span that misses
  is that direction's to halve. Then the tensor product is held to the
  margin over the whole grid of checks, a miss halving both spans. A
  cylinder's twin so refines only around its circle (32 × 2 spans at
  `1e-6`, where charging both directions gave 32 × 32).
- No direction refines past `MAX_OFFSET_SPANS` (256), a structural
  bound like the curve fit's.
  *Amended 2026-10-14 (plan step 3):* the bound is 2048 spans a
  direction and 65536 patches in all (`MAX_OFFSET_PATCHES`, the old
  bound's square). A rational twin of a cylinder of radius 3 to 10
  takes 512 spans round a whole turn at `2.5e-8` and 2 along its axis,
  which 256 a direction refused; the product is what the grid and the
  tracer pay for.

**3. Past the domain, the end span carried on.** Where the ball reaches
past the face's domain, the offset is of the end span's own polynomial
continued: `NurbsSurface::eval` already evaluates it there, so the
offset's box simply reaches past the domain. A closed direction wraps
instead. The ADR-0061 rule that a branch ends at the domain's edge then
applies to the *offset's* domain, the box the caller asked, not the
face's. The face across trims the stripe before the extension matters
anywhere else.

**4. The chamfer at the chord.** A walked chamfer puts each contact in
the edge's normal plane at chord `d` from the edge point, on its face's
section with that plane. That is the ring's rule (ADR-0036 §2) and Open
CASCADE's symmetric chamfer. Its surface is the ruled B-spline between
the two contact curves, made compatible, so each contact is the
surface's own isocurve. No offset is involved.

**5. The ends.**
- A closed edge has none.
- An analytic face across is traced against the walked blend as
  ADR-0037 traces one, through ADR-0061 now that the blend is a
  B-spline.
- A tangent vertex where the chain goes on (ADR-0035) is a junction. The
  walked stripe ends on the ball's arc in the vertex's normal plane,
  exact, with the spine's end pinned to that ball's centre. The chain's
  next stripe, from the table or walked, shares that arc.
- *Amended 2026-10-14 (plan step 3):* an end where the edge's two faces
  are tangent within ADR-0045's test is a **vanishing end**. No ball
  touches both faces exactly there, and the offsets' section crosses
  itself, so the ball is pinned to the edge's point on the faces' common
  normal. It is located on the traced piece within `√(2r·τₑ)`, with
  `τₑ` the tolerance the faces were found tangent within: surfaces a gap
  `τₑ` apart that curve away by `1/r` meet that far along. The spine
  stops there, and the stripe narrows to the vertex. CTC-05's e256
  vanishes at v84, its normals 8e-6 rad apart against an edge tolerance
  of 7.7e-3. A face turning tangent inside the edge is refused.
- *Amended 2026-10-14 (plan step 4):* the exact balls do not narrow to
  the pinned one. On CTC-05 they run smoothly to the spine's end at v84
  and stop there 1.04e-2 from the pinned ball, within the `√(2r·τₑ)` =
  0.117 above. Their arc is 1.98e-4 wide there, its narrowest, and its
  contacts are 9.9e-3 from the edge's point. The faces cross at 2.2e-4
  rad there, never tangent: the edge runs off their crossing, within its
  tolerance of each face. So the surface skins the exact balls to the
  spine's end, and its end isocurve is that narrow arc, not a pole. The
  pinned ball is kept apart (`Spine::pinned`). The assembly joins the end
  arc to the vertex, whose tolerance has to grow to cover it, or refuses
  the end by name (plan step 5).

**6. The refusals, each by name:**

| Case | Refusal |
|---|---|
| Two B-spline faces | `Unsupported` naming both faces (NURBS × NURBS, the cycle's third item) |
| A B-spline face across a walked stripe's end | `Unsupported` naming the stripe's edge and the face across |
| A walked stripe meeting another blend at a sharp vertex (a miter, a corner) | `VertexBlend`, as today |
| The radius reaching a principal radius of a face on the ball's side | `BlendTooLarge`, from the offset's `GeomError::OffsetFolds` |
| A pole of a face inside the band the ball sweeps | `Unsupported` naming the face and its partner, from the offset's `Degenerate` |
| The offset or the surface fit diverging | `OpError::Internal`, a kernel finding with its fixture, never a fallback |

`GeomError::OffsetFolds { kind, at, distance, radius }` is new. No other
variant names a fold: `Degenerate` is an operand with no valid
representation, and a fold is a valid surface asked too far. The blend
needs the distinction to answer `BlendTooLarge`. The variant is a
breaking change for an exhaustive match, recorded in `CHANGELOG.md`
with its Python class, `GeomOffsetFoldsError`.

## Consequences

- ADR-0007's "every surface is exact" holds for the table only. A walked
  stripe's surface is the one fitted surface a body carries. Its arcs
  are exact at the samples, the surface within a quarter of the
  tolerance between them.
- The checker's S5 meets a fitted surface tangent to its neighbours along
  the contacts. A double zero along the isoline `u = 0` or `u = 1` is
  what step 6 divides out through ADR-0061's isoline search. Until then
  such a pair is listed unchecked.
- The fitted offset is the primitive the shell and the face offset of a
  B-spline face need (C7's out list); they consume it from here.
- CTC-05's face (degree (3, 2), 2 × 1 spans) offsets by the census's
  radius both ways, over its domain carried a sixth of its width past
  each side, to `τ/4 = 2.5e-8`. That takes 64 × 128 bicubic spans and
  4.4 s each way in the debug-optimised test profile. The walked stripe
  asks only the band the ball sweeps, not the whole face; step 3 records
  what that costs the tracer.
- A rational face's offset is fitted non-rational. Two radians of a
  radius-2 cylinder's twin, two rational spans, take 32 at `1e-6`. Exactness was never on offer for a free-form face, and
  the tracer's patch count is the cost to watch, not the error.

## Amendments

- **Step 5 (the assembly).** Two analytic faces whose pair has a closed
  intersection (a plane against a cylinder, whose offsets meet in a
  circle) are not the tracers' pairs: `trace_section` answers
  `Unsupported`, and the spine takes the intersector's exact curves as
  its pieces instead. A closed edge's ring keeps the surface as the
  skin makes it, `u` across and `v` once round the edge, and its loop
  is the open stripe's with the two ends one seam, so that the
  integrals' inner direction (the surface's `u`, split at every knot)
  stays two spans: the integrator's cap on the splits was 256 and a ring
  of the cylinder's twin has over 450, which integrated to `1e-5`;
  `MAX_INNER_INTERVALS` is now 4096, as many as a fitted curve has
  spans (`MAX_FIT_SPANS`). Walking is tried for a fillet only, after
  the table's `Unsupported`; a walked stripe meets no other blend yet
  (plan step 5b), and a vanishing end is `TangentChain`.
- **Step 5b (the chain junction).** A walked stripe meets the next
  stripe of its chain at a tangent vertex as a table stripe does
  (ADR-0035 §3): on the ball's arc in the vertex's normal plane, its
  surface's isocurve at the end `v`, the junction's circle given a pcurve
  on it by projection. A miter or a corner against a walked stripe stays
  `VertexBlend`. The two balls are one only where the face that turns
  is tangent to its predecessor along a curve the normal plane holds, a
  ruling of a cylinder. Where it turns tangent along a curve the plane
  crosses (an annulus running into a torus along a circle), the ball
  touching the true faces rides off the table's by the second order of
  the contact's offset from that curve. A walked run whose points or
  ball miss by more than the tolerance is then `VertexBlend`, not the
  fault it is between two table runs.
- **Step 5b (what holds a walked fixture).** Open CASCADE's blend with no
  closed form is walked too, and it misses the exact ball by more than
  its tolerance claims. At 451 samples of each blend surface, Arris'
  holds the ball at `r` from both faces to `1e-8`. Open CASCADE's misses
  it by a median of `1.5e-5` (the tee), `2.1e-5` (a bicubic top, at its
  default approximation and at `1e-8` alike) and `1.4e-4` (the drilled
  torus). Over the blend's area, that is the whole of the gap between
  the two bodies' measures, `1e-7` to `3e-6` relative. ADR-0015's
  `measure_differs` needs a closed form, and none holds such a blend.
  So a walked fixture may widen its measure tolerances to cover the
  oracle's measured miss, up to `1e-5` relative. Its description states
  both kernels' miss of the ball at the same samples. Counts, genus,
  probes and the read-back stay the oracle's. A miss of the ball wider
  than that, or one Arris shares, is a finding, not a tolerance. A wrong
  radius, junction or end moves the measures by `1e-3` and more, which
  `1e-5` still catches. The cap is the price of a second oracle that
  ADR-0015 left open for a band with no closed form.

- **Step 7 (the walked chamfer).** The contact on a face is the root of
  two equations on its `(u, v)`: the edge's normal plane, and the sphere
  of radius `d` about the edge's point, by Newton from the face's foot of
  that point moved by `d` along the face on the side the face lies. The
  side is the face's outward normal against the edge's direction along
  the face's use, as the table reads a dihedral. The edge's curve is
  carried past its ends by two chords for a face across to cut (a NURBS
  edge on its end spans' polynomials, as its faces are carried on in §3;
  a circle not past its period), and the surface is `fit_skin` of the
  segment between the two contacts, degree one across, so each contact is
  its own isocurve. There is no vanishing end: the chord is defined where
  the faces turn tangent. The face's orientation looks at the edge the
  chamfer cuts as a fillet's looks away from its ball; at a tangent vertex
  the junction is the chord (`centres` is `None`), so a walked and a
  table chamfer share it. Open CASCADE's contact on a B-spline face
  misses the chord by `2.8e-7` (the bicubic top) and `1.9e-7` (the cap) at
  the median, `5.5e-6` at most; the step-5b allowance for a walked
  fixture covers it.

- **Step 8 (a loop closes to the bit).** A closed edge's ring surface is
  closed for the checker only if its first and last rows of control
  points are the same to rounding (`NurbsSurface::closure`), and a ring
  edge is used twice by its face, which L3 allows on a closed surface and
  nowhere else. The ball at the loop's two ends is found twice, by Newton
  from two starts, and the two differ by the snap's rounding, which
  grows with the coordinates: a part posed 100 from the origin failed L3.
  The skin now asks for the ball at the loop's start for both ends
  (`Spine::skin`, `Chord::section`), so the rows are equal to the bit.

- **Step 9 (a slanted seam).** A walked junction ends on the ball's arc in
  the vertex's normal plane, and a curved face's seam through the vertex is
  cut there only if the contact is on it, which holds where the loop
  crosses the seam square (a cylinder off a sphere's centre in the plane of
  the sphere's seam does). At a slant the contact is off the seam by
  `r·cot` of the angle, and the seam is cut where the contact crosses it, a
  point of the contact edge beside the junction. That cut is not built: the
  junction is refused `VertexBlend` (`ends::on_corner_edge`), where it was a
  `kernel bug` from `cut_corner`. A B-spline that hides a circle exactly
  (rational quadratic) walks as any table pair whose edge is not the
  table's curve, and meets the torus's closed form to rounding; Open
  CASCADE's walked blend of the same edge lies `6.9e-8` below it.

- **Step 9b (the pole).** A vanishing end (§5) is a **pole** where the
  ball there touches both faces at one point within the surface's share
  of the tolerance (`τ/4`), and that point is within the vertex's
  tolerance of the vertex. The arc there is that point. The skin's row at
  the end is collapsed onto it (`point_arc`), the stripe's two contacts
  end at the vertex, which is kept (`VertexSpec::Keep`), and the blend's
  loop crosses the row on a degenerate edge (`EndKind::Pole`). No corner
  edge is cut, no face across is met, and provenance records no vertex.
  A pole at a vertex another blend meets is `VertexBlend`. CTC-01's crease
  is the case: two equal cylinders whose axes meet are tangent at the top
  of their crossing, and the end ball there touches both at `v211` to
  `4.3e-14`, its arc of width zero. Open CASCADE ends its blend on the
  same pole, a degenerate edge at the vertex kept. Where the faces only
  run within the edge's tolerance of tangent, the exact ball's arc stays
  apart from the vertex: CTC-05's at `v84` is `1.98e-4` wide, its
  contacts `9.9e-3` from the vertex. That end stays `TangentChain`. To
  join it, the vertex's tolerance would have to grow past the edge's own
  `7.7e-3`, and the surface would have to leave the exact balls by `1e-2`
  to reach it, which the budget of §1 has no room for. The plan's other
  candidate, a vertex grown under ADR-0060's operation bound, is rejected
  for that reason. CTC-01's other end, `v113`, was named a fan in step 1;
  it is a tangent vertex of four edges (ADR-0039 §1), where the crease runs
  on into the vertical corner between two planes, each tangent to one
  cylinder, and the chain's junction takes it as any other. Open CASCADE's
  contacts there miss the ball by `5.3e-4` at the median and `2.2e-3` at
  worst, against Arris' `5.0e-10` and `7.8e-10`. The blend faces differ by
  `8.6e-4` of their area, the part by `5.5e-9`, and the fixture's
  tolerances cover that under the step-5b allowance. The checker's S5 on
  the pole and on that junction needed two tracer changes (ADR-0061,
  amendment of 2026-10-14).

- **Step 10a (a vanishing end is held to its vertex).** §5's vanishing
  end held the pinned ball to `√(2r·τₑ)` of the traced piece, `τₑ` the
  faces' and the edge's tolerance. The faces only meet within the
  vertex's tolerance, so the gap the offsets may cross across is the
  wider of `τₑ` and the end vertex's own; the step-9b pole already
  reads the vertex's. CTC-05's AP242 edition is the case: the edge's
  tolerance is `2.4e-4`, its vertices' `2.1e-2`, and the section passed
  `2.6e-2` from the pinned ball against an allowance of `2.1e-2`, so no
  piece held the ends (`NoSpine`, read as NURBS × torus `Unsupported`).
  The pieces were not cut short (the end lies inside the piece's
  domain), and the edge's samples lie on both faces to `8.7e-6`. Under
  the vertex's tolerance the allowance is `0.19`, and the AP242 edition
  reaches the end as AP203 does, whose edge and vertices share `7.7e-3`.
  There the exact ball's arc is `5.5e-4` wide and `2.5e-2` from the
  vertex, past its tolerance, so the end is `TangentChain` like AP203's.
  Both editions' fixtures are refusals in `blend/`. Open CASCADE's
  invalid bodies stay as the record and hold nothing. The two candidate
  oracles, the editions agreeing and a deficit integrated along the
  spine, wait for the backlog item that joins such an end; until then
  no measure is compared.
- **Step 10b (a read edge's pcurves standing off its curve, and two
  chain junctions named).** A read edge's pcurves may stand off its 3D
  curve by as much as its tolerance: FTC-11's rim of the boss f37 dips
  `3.75e-3` below the curve at its ends. A table end arc meeting that
  rim at a grazing angle, at a 7.3° kink, runs inside the band, and the
  side test read it as leaving the face. Two rules follow, both from the
  model's tolerances:
  - An end arc's or a contact's sample off its side of a face, but
    within a boundary edge's own tolerance of that edge's curve in 3D,
    is on that edge (`on_side_of_face`).
  - A corner edge cut by the blend keeps its stored pcurves, so at the
    cut they meet the blend's exact curves only within the edge's
    tolerance, and L2 holds a junction to the parametric tolerance. The
    shortened edge's pcurve on each face is ended on the new edge's
    (u, v) there, by its end control point (`pcurve_ending_on`). The
    shortened edge is already a new edge, the move is checked against its
    tolerance at the model's samples, and its cut vertex already holds
    that tolerance. The blend's own edges stay exact and keep theirs.
    The boolean moves the section edge onto the operand's pave instead,
    raising the section edge's tolerance. Here that would raise the far
    vertex of an exact contact (E5), where the read edge already carries
    the band.

  The fillet then builds as Open CASCADE does, within its volume, area
  and centroid. The counts differ by ADR-0038's lengthened corner edge.
  S5 between the walked face and its chain neighbours is step 10c's.
  Open CASCADE's chain walk allows a fixed 0.1 rad between faces it
  calls tangent. Arris keeps ADR-0040's faces' tolerance. So FTC-11's
  other slot vertex v24 is no chain vertex for Arris: a convex 3.9° kink
  where the hole f17 runs on from the plane f18, tangent within the
  vertex's `3.6e-3`, not the faces'. The hole curves away from the ball,
  so the contact on the top never meets it. That end is now
  `TangentChain` naming the edge, the corner edge and the vertex, where
  it was `Unsupported` for a pair the table has.
  Reading the vertex's tolerance in the chain walk would take v2 as
  well. It would part from Open CASCADE there, where the boss curves back
  into the contact and the end is built, and its junction would meet
  balls apart by the kink. That is a backlog item, not this step's.
  CTC-02's v167 is not a junction this design places. Its side faces
  are tangent along a ruling that leans out of the ball's normal plane,
  so the runs' side contacts miss the ruling, and each other by
  `9.1e-3`. The shared ball is where the ruling's offset meets the top
  face's offset. The refusal stays `VertexBlend`, joined to the backlog
  line of the chain through a torus, which is the same kind of vertex.

- **Step 10e (what holds FTC-10 and FTC-11).** Both leave `regression/` for
  `blend/` with tolerances widened under the step-5b allowance, each
  stating the evidence in its description.
  - FTC-10's closed edge between two skew cylinders is walked by Open
    CASCADE too, and its blend misses the ball of radius `r` at 441
    samples by a median of `2.4e-4` (`7.5e-4` at most) against Arris'
    `6.1e-10` (`2.1e-9`). The bodies differ by `3.0e-9` in volume and
    `1.9e-8` in area; the tolerances are `1e-8` (volume, inertia) and
    `5e-8` (area).
  - FTC-11's gap is in the *oracle's plane*, not the blend. Open CASCADE
    reading Arris's STEP finds the plane `f0` larger by `1.19e-2`
    (`2.2e-6` of the area; the volume `6.4e-8`), all of it from the one
    B-spline edge of the boss's rim: Open CASCADE's own wire is open
    there by `2.0e-3` where Arris' is by `2.4e-4`, both inside the
    vertex's tolerance. The area of an open wire in a plane's frame moves
    by the gap times the distance of the gap from the frame's origin
    (51 here), which is that `1e-2`. Every other edge of `f0` contributes
    the same area in both files, and a `ShapeFix` pass does not change
    either value. The tolerances are `2e-7` (volume), `5e-6` (area, and
    centroid in length) and `3e-7` (inertia), the oracle's own
    uncertainty, still three orders under what a wrong radius, junction or
    end moves.

## Alternatives considered

- **Marching the ball's centre by Newton on the two distance constraints**
  (Open CASCADE's `BRepBlend` walk). The spine would be found by an
  algorithm the checker and the intersector do not share, and ADR-0037
  ruled that out for ends. The walked stripe keeps Newton for the
  *snap* of a sample the shared tracer found, never for finding the
  branch.
- **A ruled extension past the domain**, along the boundary's
  cross-tangents. It is only `C1`, so its offset's curvature jumps at
  the domain's edge and the spine kinks there. The end span carried on
  is `C∞` and costs nothing.
- **A rational offset fit**, keeping a twin's circles exact. The offset
  of a rational surface is not rational in general, so the fit would be
  rational only to look like its base. The weights add unknowns the
  linear least squares does not take.
- **A higher fit degree.** A quintic converges in fewer spans, but
  ADR-0061 puts each patch into the other surface's implicit form at
  bidegree `(d·p, d·q)`. Against a torus's offset a quintic patch is
  (20, 20) where a cubic is (12, 12).
- **The fold as `Degenerate` with words.** The blend could not tell a
  fold, its own `BlendTooLarge`, from a pole without reading the
  message.
