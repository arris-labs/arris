# ADR-0038 — A blend running into a step lengthens the corner edge of its own convexity past the vertex, and the face across takes the end

- Status: accepted (2026-10-03)
- Plan: `blend-run-over` step 3 (the decision, the trims and their test);
  the end assembled on a plane across in step 4, on a cylinder or a cone
  across and at the ring rows' open arcs in step 5
- Amends: ADR-0007 §"Each end is trimmed by the face across the corner"
  (a corner edge may be lengthened, not only shortened) and §"The
  `BlendTooLarge` bound" (a trim past the vertex at a mixed corner is no
  run-over); the corner of mixed convexity, `VertexBlend` until now
  (`end_side`), is built
- Follows: ADR-0037 (an end on a curved face across traced and fitted),
  ADR-0036 §3 (the ring's contact at the axis)

## Context

`blend-run-over` step 1 counted the fetched tier's `BlendTooLarge` edges
by the entities each refusal names, and its largest cause, *a corner edge
shorter than the trim* (257 of 515 edges), looked like a radius too large
for its corner. Step 3 probed it before deciding: four of the fetched
parts (CTC-01, CTC-03, FTC-08 and STC-06, 277 of the 515 edges), each
edge blended alone at the battery's radius, with each refusal's site and
its numbers printed. That reading changes what the plan is about.

- **Every one of the 178 corner-edge refusals is a trim past the
  vertex**, on the vertex's side of the corner edge, on its extension.
  Not one is past the far end, and no corner edge is shorter than its
  trim. Their shape is CTC-03's sampled edge: a convex edge on a sheet's
  flange runs into a step. At the step's corner the tall part's front
  vertical edge is convex, the step's foot is concave, and the front face
  is one face round the corner. The ball's contact on that front face
  reaches the step's plane *below* the vertex, on the convex edge's line
  continued.
- **The 98 *contact leaves a plane / cylinder face* refusals are all the
  ring's contact at the axis.** A chain runs from a line edge into a
  rounded corner whose radius is at most `r`, and the torus there would
  be a horn or a spindle torus, ADR-0036 §3's refusal. Open CASCADE
  builds most of them. That is a surface-kind question, not a run-over.
- **The case the plan opened for, a contact running onto a wall nearer
  the edge than `r` (ADR-0007's "third face"), occurs in none of them.**

With a candidate rule for the trim written, it was run over every corner
end those four parts reach with a plane or a cylinder across. All 170
ends at a corner of mixed convexity lengthen exactly the corner edge of
the blend's own convexity. That includes the concave blends, whose
mirror image the step is. Each lengthened stretch lies inside its face.
No other corner changed.

Open CASCADE builds the shrunk case (`regression/fillet-into-a-step`)
with the closed forms' volume and area and one vertex and one edge more
than this decision gives: it keeps the consumed corner vertex on the
lengthened line. No reference module was read for this decision; the
oracle's result is what was compared.

## Decision

**1. A corner whose two edges differ in convexity is a mixed corner, and
there the trim on the edge of the blend's convexity lies past the
vertex.** The blend runs into a step: its contact on the face it shares
with that edge continues past the vertex, because the face does. That
corner edge is **lengthened** along its own curve to the trim point. It is
the same edge with its end moved, `Modified` like a shortened one, and the
corner vertex is consumed as at every end. The other corner edge is cut
short at its trim, as at any corner.

**2. The end is the end any corner has.** The arc is the section of the
blend with the face across: a conic exact on a plane (ADR-0007), traced
and fitted on a cylinder or a cone (ADR-0037). The face across takes it
in its loop between the two corner edges at the consumed vertex. Nothing
about the arc, its pcurves or the loop's re-cut is new. What is new is
which side the arc lies on. At a mixed corner it lies outside the face
across as it stands, and the face grows by the region the arc bounds with
the lengthened stretch and the cut-off piece of the other corner edge.
That is the side a convex blend at a concave corner already has, so the
side rule reads: inside where both corner edges have the blend's
convexity, outside otherwise.

**3. The lengthening is checked, and refused by name where it does not
hold.** A trim past the vertex is accepted only:

- at a mixed corner;
- on the corner edge of the blend's convexity;
- on an analytic curve, since a fitted curve has no extension past its
  domain;
- where the stretch from the vertex to the trim point lies inside the
  face of the blended edge that the corner edge bounds, by
  `FaceDomain::side` at the precision's check samples. This is the test
  every contact already passes.

Anything else is `Reason::BlendTooLarge` naming the blended edge and that
corner edge, the refusal every corner gives today: a trim past the far
vertex, past the vertex at a corner that is not mixed or on the wrong
edge of one, or a stretch that runs out of its face. The last one is a
real run-over (a hole in the front face just below the step). The arc
outside the face across is held by the same `on_side_of_face` test as
every end arc. `Reason` gains no variant. `blend/mixed.rs`
(`corner_trims`) decides all of this in place of `cut_corner` and
`end_side` at a face end.

**4. The lengthened edge's pcurves are derived again over its new
range**, from its curve by `pcurve_on`, as a contact's are. They are not
evaluated past the range they were stored for, which a fitted pcurve
could not be. They are exact for a line or a conic on a plane or a
cylinder.

**5. One edge, not two.** The lengthened stretch shares its curve and its
two faces with the edge it continues, so Arris keeps one edge where Open
CASCADE splits it at the old vertex. A fixture states this convention as
`counts_differ`, and the oracle's measures hold it.

**6. Provenance and the checker need nothing new.** The consumed vertex
and the blended edge are `Deleted`. The trim vertices are `Generated`.
Both corner edges, the face across and the blended edge's faces are
`Modified`, exactly as ADR-0007 roots an end. The checker's S5 decides
the blend against the grown face across by the arms it has (a plane or,
by ADR-0037, the tracers), and no row is added.

## Consequences

- The run-over this plan builds is the mixed corner, on a plane across
  (step 4) and on a cylinder or a cone across and the ring rows' open
  arcs (step 5). CTC-03's and FTC-08's sampled edges are of this kind.
  STC-06's two are the ring's contact at the axis, so STC-06 stays in
  the `fillet` column behind the horn and spindle torus.
- The census's label *a corner edge shorter than the trim* names the site
  that refused, not the geometry. On the parts probed, every one is a
  trim past the vertex.
- A blend that meets a third face while its contacts stay inside their
  faces is still not detected by the operation (ADR-0007). The census
  shows none, so it stays a backlog line with its fixture still owed.
- A spindle or horn torus on a rounded corner of radius at most `r`
  stays ADR-0036 §3's refusal. Taking it needs the surface (BACKLOG's
  spindle-torus line), and it is that line's sizing, not a run-over's.

## Alternatives considered

- **Split the corner edge at the old vertex, as Open CASCADE does.** That
  keeps a vertex of two edges of one curve between the same two faces. It
  says nothing the one edge does not, and it gives the consumer a
  `Generated` edge beside the `Modified` one for what is one edge
  lengthened.
- **A clip on the face across** (the plan's opening assumption): a face
  the blend runs onto, which cuts it short. The census holds no such end:
  the face across is the corner's own, and the end is the arc every
  corner has.
- **Keep refusing the mixed corner** (`end_side`'s `VertexBlend`, behind
  `cut_corner`'s `BlendTooLarge`). That leaves the largest refusal of the
  blend network unbuilt where Open CASCADE builds it, for no geometric
  reason: the end is the same arc, on the same face.
