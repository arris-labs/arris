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
