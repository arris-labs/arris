# ADR-0074 — A vanishing end off its vertex leaves the rest of its edge: the stripe stops on its narrowest exact arc, and the edge runs on from there to the vertex

- Status: accepted (2026-10-29)
- Plan: `tolerant-tangency` step 2
- Amends: ADR-0062 §5 and its step 9b (a vanishing end whose ball does not
  close on the vertex is `TangentChain`)
- Follows: ADR-0045 (the blended edge's own dihedral read at its
  tolerance), ADR-0060 (`max_tolerance` bounds what an operation grows)
- Reference trees read: none. Open CASCADE and the second oracle
  (ADR-0073) are run, not read.

## Context

A walked stripe has a **vanishing end** where its two faces turn tangent
within the edge's tolerance (ADR-0062 §5). Where the ball there touches
both faces at the vertex, the stripe narrows to a pole (step 9b). Where
the faces only *run* within the edge's tolerance of tangent, they still
cross at a small angle. The exact balls then stop at a narrowest arc that
stands off the vertex, and step 9b refused that end as `TangentChain`.
Joining it to the vertex would have taken the surface off the exact balls
by `1e-2`, which ADR-0062 §1's budget has no room for. The other route
grows the kept vertex, which was rejected for the same reason.

The refused end is CTC-05's e256 at v84 (AP203, `r 0.889`). Measured
there:

| | AP203 e256 | AP242 e258, the battery's edge (`r 0.978`) |
|---|---|---|
| the narrowest arc's width | `1.98e-4` | `6.0e-4` |
| its contacts from the vertex | `9.94e-3`, `9.74e-3` | `2.52e-2`, `2.46e-2` |
| the vertex's tolerance | `7.74e-3` | `2.05e-2` |
| its contacts from the **edge's curve** | `7.39e-3`, `7.26e-3` | `1.97e-2`, `1.93e-2` |
| the edge's tolerance | `7.74e-3` | `2.37e-4` |
| the curve's nearest point from the vertex | `6.6e-3` along | `1.55e-2` along |

On AP203 the arc lies within the edge's own tolerance of the edge's curve.
The file states that the faces meet within that tolerance all along the
edge. So past the arc, toward the vertex, the edge is the tangent
dihedral ADR-0045 reads it as, and nothing is left for a ball to round.
The second oracle builds e256's blend as a valid solid. Open CASCADE
builds an invalid one (`plans/tolerant-tangency` step 1).

## Decision

**1. The stripe stops short.** At a vanishing end whose ball does not
close on the vertex, the stripe ends on its narrowest exact arc, the
surface's row there, kept exact rather than collapsed. The blend's loop
crosses that row on a degenerate edge at a new vertex `w`, as it crosses a
pole's collapsed row (step 9b). Both contacts end at `w`.

**2. The cut.** `w` is the edge's point nearest the arc's middle. It must
lie strictly inside the edge, on the vanishing end's side of its middle.
Its tolerance is the larger of the edge's own and the arc's farthest point
from `w`, plus the surface's share of the tolerance (`SECTION_FIT_FRACTION`
of it, ADR-0062 §1). The share covers the contacts' pcurves, which are
fitted from the surface's isocurves. E6 holds the degenerate edge's row,
`1.98e-4` across on AP203, within that tolerance. No checker row bounds an
edge's length, so a remnant shorter than its vertices' tolerance is
valid.

**3. The remnant.** The edge is cut at `w`. Its stretch from `w` to the
old vertex is a new edge on the same curve and pcurves, with the edge's
own tolerance, recorded `Modified` from the edge. Each face's loop then
walks the edge as its remnant and the contact on that face, in the edge's
direction. The old vertex and every other edge at it are kept.

**4. The ceiling.** Where `w`'s tolerance would exceed `max_tolerance`,
the end stays `TangentChain` naming the edge and the vertex, as before.
That covers AP242, whose arc is `1.97e-2` off a curve of tolerance
`2.37e-4`, inside a vertex the read healed to `2.05e-2`. The plan's open
question 1 decides that case as an amendment to ADR-0060.

**5. A missed face across beside a crease.** A walked end whose contact
misses the face across is named as a table stripe's end is
(`near_tangent_corner`, ADR-0062 step 10b): `TangentChain` naming the
edge, the corner edge and the vertex, where a corner edge is tangent
within the vertex's tolerance. A corner edge whose crease is within its
*own* tolerance qualifies whatever its sense. CTC-05's v115 is the case:
f109 runs on into the cylinder f19 `7.0e-6` rad off tangent, and that
reads concave beside a convex blend. That is a sense the file does not
state, just as ADR-0045 reads a blended edge's dihedral. Before, the
same end was `Unsupported` for a NURBS face against a cylinder.

`Reason` gains no variant, and no public type or signature changes.

## Consequences

- CTC-05 AP203's e256 no longer stops at v84. Its refusal moves to the
  chain's other end, v115, `TangentChain` naming e256, e132 and v115.
  The B-spline face runs on into a cylinder there, tangent within the
  vertex's tolerance and not the faces', which is the chain rule the plan
  writes next (FTC-11's v24 is the same rule). The fixture stays a
  refusal until that rule lands.
- The second oracle's edge-mode verdict for e256 asks a question Arris
  will not answer: Arris's blend chains through v115. Its record for
  this fixture is the tangent chain's (the plan's step 1 amended).
- A vanishing end whose arc lies off the edge by more than the ceiling,
  AP242's, is unchanged.

## Alternatives considered

- **Grow the kept vertex to hold the arc** (`9.94e-3` on AP203). The
  operand's vertex is shared with the body it came from, so growing it
  would re-make every kept edge at the vertex on a new vertex. It also
  sits at the ceiling. The remnant needs nothing past the edge's own
  tolerance.
- **Collapse the row to a point at the vertex.** That is step 9b's `1e-2`
  off the exact balls, refused by ADR-0062 §1's budget.
- **Run the stripe on past the arc**, its faces crossing at `2.2e-4` rad
  there. That stripe rolls in a crease the file says is not there. It
  stays the backlog's other reading.
