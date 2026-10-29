# ADR-0075 — A chain runs through a vertex tangent within its tolerance: where the end across meets nothing, the vertex's corner edge is read tangent at the vertex's tolerance

- Status: accepted (2026-10-29)
- Plan: `tolerant-tangency` steps 4 (the selection) and 5 (the junction)
- Amends: ADR-0040 §2–3 (every tangency a blend reads is at the faces'
  tolerance), ADR-0035 §1 (the selection is closed under tangent
  continuation)
- Follows: ADR-0062 step 10b (`near_tangent_corner`: a contact that
  misses a face across beside a corner edge tangent within the vertex's
  tolerance is `TangentChain`), ADR-0074
- Reference trees read: none. Open CASCADE and the second oracle
  (ADR-0073) are run, not read.

## Context

ADR-0040 reads every tangency a blend asks about at the faces' tolerance.
§3 kept the vertex's wider tolerance out on purpose: a tangency admitted
at that width would leave the two runs' contacts further apart than a
junction accepts.

Two real parts have a chain end where the faces' tolerance says "kink"
and the vertex's says "tangent", and where the end across that kink has
nothing to meet:

- FTC-11's slot (AP242 edition 2, solid #1299, `r 0.0105` and the
  battery's `0.0255`). The line `e24` meets the hole f17 at `v24`, a
  convex kink of 3.9° between f18 and f17. The hole curves away from the
  ball, so the contact on the top never pierces it. Open CASCADE chains
  through the kink and builds five blend faces that pass `BRepCheck`.
  The second oracle refuses it.
- CTC-05 AP203's e256 (`r 0.889`). At `v115` the B-spline face f109 runs
  on into the cylinder f19, `7.0e-6` rad off tangent. That is within the
  vertex's `7.74e-3` and not within the faces' own. The second oracle
  chains through it and builds three edges as a valid solid.

ADR-0062 step 10b refuses both ends as `TangentChain` by name
(`near_tangent_corner`). The refusal says what is missing: the blend
would run on round the face across, as a chain does.

FTC-11's other slot fixture, `blend/nist-ftc-11-ap242-bspline-circle-fillet`,
has a near-tangent kink too, at `v2`: the plane f38 stands `2.3e-3` off
tangent to the boss f37. There the face across curves back into the
contact's path. The contact pierces it at a grazing angle, the end is
built, and Open CASCADE builds the same end. A rule that read every
vertex at its own tolerance would chain through `v2` too, and change a
result that agrees with the oracle.

## Decision

**1. The selection.** At a chain end where the walk does not continue
(ADR-0035 §1 read at the faces' tolerance), a vertex of exactly three
edges is a **kink** when it would be a tangent vertex with its corner
edge `w` read tangent within the vertex's tolerance
(`kinked_vertex`). The walk continues through a kink only where the end
across it meets nothing. The end is made exactly as the build makes it,
and it must be `near_tangent_corner`'s `TangentChain` naming the edge,
`w` and the vertex: each contact misses the face across, which runs on
from the stripe's face within the vertex's tolerance. Any other end,
built or refused, stays the build's. The chain is then continued from
the next edge, through tangent vertices and further kinks, to where it
next ends.

**2. The junction across the kink** (step 5). The two runs' end balls
are `2r·sin(θ/2)` apart at a kink of `θ` (`7.2e-4` at `v24`). The
junction's shared edge is the arc of their mean ball, and its pcurves are
each face's own end isocurve. The edge and its junction vertices take the
distance to either face as their tolerance, bounded by the vertex's
tolerance. That bound meets §3's reason: the vertex's tolerance is what
the file says the faces meet within there, and the junction asks no more.

*Settled at step 5:* the gap lies along the edges, not across them. At
FTC-11's `v24` the two far contacts meet the corner edge at one point,
and the two contacts on the shared face cross `r·tan(θ/2)` short of the
vertex. So each contact on the shared face ends where they cross, by
Gauss–Newton from its end, and the far ones end on the corner edge
(`shared_crossing`): every face but the blends closes its loop exactly.
The arc runs from that crossing to the far point about the mean of the
two end balls. Its pcurve on each blend is the arc's projection, pinned
at both ends to that blend's own contacts there (`pin_kinks`): the
parameter is made a knot of full multiplicity and its control point
moved, which is exact whatever the weights. An isocurve of each blend
would not do, since one blend's end is not the other's. The arc and its
two vertices take the pin's shift on top of the half gap the mean
closes. At `v24` that is `3.6e-4`, against the vertex's `3.6e-3`. A
crossing that does not settle, a ring as one run, or a pin past the
vertex's tolerance is `TangentChain` naming both edges and the vertex.

**3. What stays.** A kink of four edges, a cusp, and a kink whose end
across is pierced (FTC-11's `v2`) are read as before.
`BlendReason::TangentChain` stays the refusal past the vertex's tolerance
and at a kink the vertex does not cover. `Reason` gains no variant, and
no public type or signature changes.

## Consequences

- FTC-11's slot chains `e22`–`e26` from `e24` through `v24`, the battery's
  edge included: five edges, as Open CASCADE's five blend faces.
  CTC-05 AP203's e256 chains `e32`, `e133` and `e256` through `v115`:
  the second oracle's three edges.
- From step 5 the slot builds, valid at `Full`, held to Open CASCADE's
  measures. Its counts are two vertices and two edges fewer, for the
  corner vertex Open CASCADE keeps at each end of the chain (ADR-0038).
  CTC-05 AP203's chain gets past `v115`. It is then refused
  `VertexBlend` at `v31`, the tangent vertex between the walked `e32` and
  `e133`, whose balls stand `8.9e-5` apart (step 6).
- The kink's end is made once more than before, to probe it, with its
  stripe: one more trace per kink on a walked stripe. CTC-05's refusal
  takes 4.2 s where it took 4.0 s; the slot's stays 0.05 s.
- `blend/nist-ftc-11-ap242-bspline-circle-fillet` is unchanged: its `v2`
  end is pierced.

## Alternatives considered

- **Every tangency at the larger of the faces' and the vertex's
  tolerance.** That chains through FTC-11's `v2`, whose end across is
  built and agrees with Open CASCADE, and it re-reads every vertex of
  every part. ADR-0040 §3's reason against it stands wherever an end
  across is met.
- **Open CASCADE's fixed `0.1` rad G1 allowance.** Rejected by ADR-0040:
  a fixed angle calls a 5° edge tangent on any part.
- **A geometric test of "curves away" instead of making the end.** It
  would be a second reading of the same question, and the two could
  disagree. Making the end asks the code that would build it.
- **Retry the selection after the build's refusal.** The refusal comes
  after every stripe and every other end is made, and a second kink means
  a third build. Probing the one end in the walk asks the same question
  once, before anything else is built.
