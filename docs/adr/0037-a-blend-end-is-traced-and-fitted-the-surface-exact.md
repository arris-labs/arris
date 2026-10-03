# ADR-0037 — A blend's end the face across cuts in no closed form is traced by the intersector's tracers and fitted as a section is; the blend's surface stays exact

- Status: accepted (2026-10-03)
- Plan: `blend-residue` step 5 (the decision, the tracer, its fit); the
  open arc's torus ending on a plane parallel to its axis and on a
  cylinder across lands in step 6, the ruling stripe ending on a cylinder
  or a cone across in step 7
- Amends: ADR-0007 §Consequences ("a fitted pcurve on the blend is the
  one non-exact item") — an end curve may be fitted too; closes ADR-0036
  §6's deferral of the two end families
- Follows: ADR-0018 (the ruled tracer, the fit held to the branch),
  ADR-0019 (the torus tracer, the storage rule, `SECTION_FIT_FRACTION`),
  ADR-0022 (two fits of one section within twice the fraction)

## Context

ADR-0036 left the two largest refusals of the blend network as *ends*,
not pairs: the table builds the stripe, and the face across its end cuts
it in a curve no variant carries. An open arc's torus on a plane parallel
to its axis and off it meets it in a spiric section (321 edges, 12 parts
in ADR-0036's census); a ruling stripe's cylinder on a cylinder or a cone
across whose axis is square to it meets it in a quartic (422, 14). The
plan's census by part (`fillet-by-part.md`, step 1) adds a third: the
open arc's torus on a cylinder across, off its axis, which leads two of
the fetched tier's parts and one committed. ADR-0007 ruled every such
curve out — "nothing is fitted but a pcurve" — because C2 had no tracer;
C3 and C4 built two, and the intersector already fits every section
without a closed form to `Curve::Nurbs` within `SECTION_FIT_FRACTION` of
the tolerance (ADR-0018, ADR-0019). The checker's S5 decides a torus
against a plane or a cylinder in every pose, and two cylinders on
crossing axes, by the same tracers.

Read in the reference trees: Open CASCADE trims a stripe's end on the
face across in `ChFi3d_Builder::PerformIntersectionAtEnd`, and finds the
curve with `ChFi3d_ComputeCurves` (`ChFi3d_Builder_0.cxx`), which is
handed the two surfaces and the curve's two ends, known beforehand, in
both parameter planes: a plane against a cylinder and two planes by
`IntAna_QuadQuadGeo`, everything else through `GeomInt_IntSS`, kept
between the given ends, and failing that by `IntWalk_PWalking` marched
from one end to the other. The shape is this decision's — the ends in
closed form, the section between them taken from the general
intersector — with the march replaced by the tracers, whose branches are
proven rather than started from a point. Read for the structure; nothing
taken.

## Decision

**1. The surface stays exact; only the end curve is fitted.** The
stripe's torus, cylinder or cone is the table's closed form, unchanged.
Where the face across cuts it in no closed form, the end curve is the
section of the blend's exact surface with the face across, traced and
fitted — the one item at its end not exact, as the pcurves already are.
A fitted *surface* stays ADR-0007's rejected alternative: the pairs with
no closed form (crossing cylinders, a torus against a cylinder off its
axis) stay `Unsupported` until the NURBS cycle.

**2. The intersector's tracers, through one dispatch.** The end is
traced by `arris_geom::trace_section`, the dispatch the intersector's
own sections take: in the torus's parameter plane when either surface is
a torus (`trace_torus`, ADR-0019), along the rulings of a quadric inside
a region otherwise (`trace_quadrics`, ADR-0018). The section's branches
and tube circles are searched for the one that holds both trim points —
where each contact pierces the face across, in closed form — within
`tol.linear`: each point located on a branch by samples a 128th of a
half turn of the walked parameter apart, narrowed by a golden-section
search. Of the stretches from one point to the other (one on an open
branch, two round a loop) the end is the one whose midpoint lies in the
stripe's band between its contacts. A tube circle the tracer returns
exact stays exact: a plane through the torus's axis within the
tolerance, which the closed form takes first anyway.

**3. The fit is the intersector's rule on the stretch.** The stretch is
fitted by `arris_geom::fit_branch` — the function the intersector's
sections now call for a whole branch — at the branch's own parameter, at
`SECTION_FIT_DEGREE` (5), until it is nowhere farther than
`SECTION_FIT_FRACTION` (a quarter) of the end's tolerance from the exact
branch (the plan's first open question). The end's tolerance is the
stripe's and the face across's, as for a closed-form end arc. Nothing
tighter: the end is one edge with one 3D curve that both faces' pcurves
are fitted to, so the remaining three quarters cover the pcurves exactly
as they do for a boolean's section edge (`SECTION_FIT_FRACTION`'s doc);
a neighbour never shares a *second* fit of it. Measured on 80 posed
ends (8 of them a tube circle returned exact; a torus on a plane at
`d/R` from 0 to 0.9999, on a cylinder across at five overlaps, a ruling stripe on a cylinder across with a contact
up to a few hundredths of `R` from grazing it; `R/r` 1.25 to 100, at the
origin and turned 150 away): none missed, 9 to 71 control points, the
worst sample 1.2e-8 of both surfaces against the bound's 2.5e-8, each
trim point within 4e-13 of the curve's end.

**4. The pcurves are fitted from the 3D curve, as C3's sections are**
(the plan's second open question): `pcurve_on` on the blend and on the
face across, the rule every fitted section edge follows (ADR-0022). The
torus tracer's own `(u, v)` would serve the blend's pcurve only where the
blend is the walked torus, not on a ruling stripe's cylinder, nor on the
face across; one path for every end keeps E4 deciding all of them alike.

**5. The checker decides the end by the arms it has.** E4 holds the
fitted pcurves to the edge's tolerance, as every fitted edge; S5 decides
the blend face against the face across by the same tracer, its section
fitted whole, which lies within twice the fraction of the end's fit
(ADR-0022) and so within the tolerance of the edge the two faces share.
No checker row is added; steps 6 and 7 run their fixtures at `Full` to
show it.

**6. What is refused, by name.** The caller's `Unsupported` naming the
blend's surface kind and the face across — the refusal those ends give
today — where the tracer does not decide the pose (`SectionFault`, poses
of measure zero), where the fit does not get under the fraction within
`MAX_FIT_SPANS`, where no single stretch of one branch or tube circle
holds both trim points on the stripe's side (a singular point between
them, the trim points on two branches), or where the two trim points are
one. `Reason` gains no variant; a stop is the caller's
`OpError::Interrupted`; any other geometry error is a fault. An end
leaving the face across is `BlendTooLarge` by `FaceDomain::side`, as a
closed-form end arc is.

## Consequences

- A blend end is exact where a closed form exists (a plane square to the
  stripe, a plane through the torus's axis) and traced only where none
  does; the closed forms keep their frames, parameters and blessed dumps.
- A traced end costs a trace and a fit of a stretch, not of the whole
  section: under 0.2 s for the 56 torus-on-plane ends of the test above in the
  dev profile, the fit dominating as ADR-0019 found.
- `arris-geom` gains `trace_section` and `fit_branch`, both public, both
  what `section::traced` already did inside; the intersector's results
  are unchanged bit for bit.
- The trim vertices' tolerance covers the trace's reach of each trim
  point and the fit's, reported per end (`gaps`), as well as the edges
  meeting there.

## Alternatives considered

- **A new tracer for blend ends** (marching the ball along the face
  across, as Open CASCADE does). The two tracers already prove their
  branch topology and are what S5 checks the result with; a third walk
  would be checked by a different algorithm than it was built by.
- **Taking the whole fitted branch from `intersect_surfaces` and an edge
  range on it.** One fit fewer per end in code, but the fit of a whole
  torus section is 3 to 90 ms (ADR-0019) where the end is a quarter of a
  tube, and a range across a loop's seam needs a periodic edge range the
  model has nowhere else.
- **A tighter fraction for an end** (half the section's). Measured
  unnecessary: the worst sample is under half the bound already, and the
  pcurves' share is the same as on any section edge.
