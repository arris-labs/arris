# ADR-0035 — A stripe follows a chain: tangent vertices propagate the selection, two stripes meet on the ball's cross-section, an open arc blends to a torus section trimmed on its meridian

- Status: accepted (2026-10-02)
- Plan: `c6-blend-network` step 2; the open arc lands in step 3, the
  junction in step 4, closed chains in step 5, the refusals in step 6,
  the chamfers in step 7
- Amends: ADR-0007 §"Everything outside the table" (what `TangentChain`
  means) and §"Each end is trimmed by the face across the corner" (an end
  may instead be a junction)
- Follows: ADR-0007 (one stripe per edge in closed form, assembled through
  `ops::rebuild`, provenance rooted at the edge)

## Context

C6 is chosen on a census of the real parts' edges, each blended alone
(`docs/plans/c6-blend-network.md`): the largest refusal that is Arris's
own is an open circular arc between a plane and a cylinder, refused as
`Unsupported(circle curve × plane)` on every edge it touches — in the
NIST files a hole's or a boss's rim split at its seam into two half
circles — and the next is an edge whose end sits at a *tangent junction*,
where the vertex's third edge has tangent faces, refused as
`TangentChain`. Step 1 separated that refusal on the fetched tier: of
2854 `TangentChain` edges 2479 are tangent dihedrals themselves, which
Open CASCADE refuses as well ("no suitable edges for chamfer or fillet")
and the battery no longer samples, and 375 are junctions, of which Open
CASCADE builds 299. Step 1's fixtures hold the anatomy with Open
CASCADE's oracle values: a D-shaped notch (an open arc ending on a plane
through its axis), a disc whose rim is two half circles and a stadium's
top outline (closed chains of arcs, and of lines and arcs), and a cap
edge that reaches the end arc of an earlier fillet.

Read in the reference trees. Open CASCADE's `ChFi3d_Builder_1.cxx`
(`PerformElement`) builds a *spine* from one named edge by walking it on
at each end vertex: an edge there joins the spine when it is not a
tangent dihedral itself, every *other* edge at the vertex is one
(`FaceTangency`, through `ChFi3d::IsTangentFaces`), and it does not turn
back; a spine that comes back to its first vertex is closed. A fillet of
one edge of the stadium therefore fillets the whole outline, and the
step-1 cap-edge fixture's oracle body is the cap edge, the earlier
fillet's end arc and the next cap edge blended as one spine of three.
Its counts (`expected.json`) say one blend face per edge of the spine,
even where two neighbours lie on one torus (the split rim: six faces,
two half tori). `ChFiKPart_ComputeData_FilPlnCyl.cxx` places the plane
against cylinder fillet along a circle as the torus coaxial with the
cylinder over the edge's own range, which is ADR-0007's ring without the
whole turn.

## Decision

**1. The selection is closed under tangent continuation.** At an end
vertex `v` of a blended edge `e`, `v` is a *tangent vertex* when it has
exactly three edges `e`, `e′` and `w`, the two faces of `w` are tangent
at `v`, `e′` is not a tangent dihedral, `e` and `e′` share exactly one
face, and their directions at `v` run on (the angle between the one
leaving `v` along `e′` and the one arriving along `e` is below a right
angle). Then `e′` is blended too, with the same kind and size, and the
walk goes on from its far vertex. The edges reached are the *chain* of
the named edges; it is open when both walks stop at a vertex that is not
tangent, closed when they meet. Naming any edge of a chain, or several,
or all, gives the same result with the same ids: the operation blends
the union of the chains in the body's iteration order, as ADR-0007 does
the named set. A walk that reaches an edge outside the table refuses
with that edge's own refusal, which names it.
*(Amended by ADR-0039: a vertex of exactly four edges is a tangent
vertex too where `e` and `e′` share no face and each of the two others is
tangent at `v` between one face of each; its junction runs between points
on those two edges, both cut.)*
*(Amended by ADR-0041: a vertex of exactly two edges that share both faces
and run on is a tangent vertex too, its junction cutting nothing.)*
*(Amended by ADR-0040: "tangent" here, and in §6, is tangent for the
blend: normals parallel to the angular precision, or the blend's size
times their sine within the faces' tolerance.)*

**2. One stripe per edge, still.** Each edge of a chain keeps its own
closed-form stripe and its own blend face, `Generated` from that edge
whether it was named or reached: the consumer's one-face-per-edge naming
(ADR-0007) holds over a chain, and a chain of one surface — the two
halves of a split rim — is two faces, as Open CASCADE builds it.

**3. Two stripes meet at a tangent vertex on the ball's cross-section.**
At a tangent vertex the ball that touches `e`'s two faces and the ball
that touches `e′`'s are one ball: the faces `e` and `e′` share is one,
and the two others are tangent at `v` with one normal. Each stripe
contains that ball's great circle in the plane through its centre square
to the edges' common direction — a fillet's cylinder as its cross-section
at `v`'s parameter, a fillet's torus as its meridian at `v`'s angle — and
a chamfer's plane or cone contains the chord of the same two contacts.
The *junction arc* is that circle (a fillet) or chord (a chamfer) from
the contact `q` on the shared face to the contact `p` on `w`; every
pcurve of it is exact: a line at constant `v` on a cylinder, at constant
`u` on a torus or a cone, a line on a plane. Its tolerance is the larger
of the two stripes'. The two blend faces meet along it at a tangent
dihedral; no face across takes it in its loop. On the shared face the two
contacts meet at `q`; `w` is shortened to `p` on its own curve; `v`
goes. The two balls being one is checked, not assumed: centres apart by
more than the tolerance, or one convex and one concave, is an internal
fault, which the tangent-vertex test above makes unreachable.

**4. An open arc between a plane and a cylinder blends to a section of
the ring's surface.** A circular edge where a plane meets a cylinder
square to its axis, open or closed, blends as ADR-0007's ring does — the
torus coaxial with the cylinder of major radius `R + sσr` and minor `r`,
or the 45° cone through the same two contact circles at `d` — over the
edge's own range rather than a whole turn: `u` runs along the edge from
its start vertex, the contacts are the edge's circle moved along the
axis and widened over the same range at constant `v`, and the face's
loop is a rectangle in (u, v), so an open arc needs no seam. Each end is
a junction (§3) or is trimmed by the face across, which then has to be a
plane containing the cylinder's axis: it holds the vertex, so it is the
half-plane at the vertex's angle, and the torus meets it in the meridian
circle there, the cone in its ruling, exact on the plane and a line at
constant `u` on the blend. The corner's two other edges lie on that
plane, so they are the radial line on the plane face and the ruling on
the cylinder, each shortened to its contact; the face across takes the
meridian between them as ADR-0007's end arc. The face across cannot be
square to the axis (it would hold a line of the edge's own plane), and
any other face across — a plane parallel to the axis off it, an oblique
plane, a curved face — meets the torus in a quartic: it is `Unsupported`
naming the blend's surface kind with the edge and the face across with
its kind. A closed edge keeps ADR-0007's ring with its seam.

**5. Provenance.** A junction arc and its two vertices are `Generated`
from both edges it joins, as a miter's are; `w` is `Modified` into its
shortened self and `v` is `Deleted`, as a corner edge and a corner vertex
are at a trimmed end. An open arc's blend face, its two contact arcs, its
meridians at trimmed ends and their four vertices are `Generated` from
the arc. No new `Role` and no new kind of record: the record of a chain
is the records of its edges, with the junctions shared like miters.

**6. What the refusals mean now.** `TangentChain` is an edge that is
itself a tangent dihedral, and an end at a vertex where a corner edge's
faces are tangent but the vertex is not a tangent vertex as §1 defines
it — the next edge turns back, or is itself a tangent dihedral.
`VertexBlend` keeps a vertex of other than three edges, a chain end at a
miter or a corner (a blended edge met by a chain at a vertex that is not
a tangent vertex), and the corners ADR-0007 lists. A junction whose
contact `p` leaves `w`, or whose `w` is shorter than the cut, is
`BlendTooLarge` naming the edge and `w`. A chain through a pair outside
the table is `Unsupported` naming that edge's pair. `Reason` gains no
variant.
*(Amended by ADR-0039: `VertexBlend` no longer keeps the vertex of four
edges where both faces turn tangentially; it keeps every other vertex of
more than three.)*

### Worked example: the stadium's outline

The stadium (`blend/stadium-outline-fillet`): two lines of length 2
and two half circles of radius 1, extruded 1, the four top edges
filleted at 0.25. The body has 8 vertices, 12 edges, 6 faces (top,
bottom, two planes, two half cylinders) and 6 loops. Every top vertex is
a tangent vertex: its third edge is the vertical line where a plane side
meets a half cylinder tangentially, and naming any one top edge reaches
the other three, a closed chain of four.

| | vertices | edges | faces | loops |
|---|---:|---:|---:|---:|
| the stadium | 8 | 12 | 6 | 6 |
| − the four top edges and vertices | −4 | −4 | | |
| + two contacts per edge | | +8 | | |
| + a junction arc and its two vertices per junction | +8 | +4 | | |
| + a blend face per edge, two half cylinders and two half tori | | | +4 | +4 |
| **the result** | **12** | **20** | **10** | **10** |

`V − E + F = 12 − 20 + 10 = 2` with one shell, genus 0; Open CASCADE's
counts are the same (`expected.json`). The record, per top edge: its
blend face, its two contacts, its two junction arcs and their four
vertices `Generated` (nine outputs, the single edge's nine; each arc and
each of its vertices also `Generated` from the neighbour across the
junction); the edge and, once each, the four top vertices `Deleted`; the
top face, the two plane sides, the two half-cylinder sides and the four
vertical edges `Modified`, the bottom and its four edges kept. 24
distinct outputs are generated: 4 faces, 8 contacts, 4 arcs, 8 vertices.

The D-notch (`blend/d-notch-rim-fillet`), an open arc trimmed at
both ends: the plate's 12 vertices, 18 edges and 8 faces lose the arc and
its two vertices and gain two contacts, two meridians and four vertices,
with one face: 14, 21, 9, Open CASCADE's counts.

## Consequences

- A fillet of one edge can now change more of the body than that edge's
  neighbourhood. Before this decision every such call was refused, so no
  result a consumer has changes; the changelog says the selection
  propagates.
- Every pcurve a chain adds is exact. The only fitted pcurves on a blend
  stay ADR-0007's oblique ends and miters.
- The junction is a miter whose third edge is tangent and whose curve is
  the ball's circle: the code builds both in one place, the junction a
  variant of the miter's end, and `blend.rs` grows rather than splits; a
  module split is a refactor step when one is wanted.
- The pairs with a cone, a sphere or a torus, and a chain through them,
  stay C6's later plans; a chain makes them reachable from a supported
  edge, so their refusal names the edge the walk reached, not the one the
  caller named.
- The torus against an oblique plane, the end the closed forms do not
  cover, stays refused rather than fitted (ADR-0007: nothing is fitted
  but a pcurve).

## Alternatives considered

- **The caller names every edge of a chain**, a tangent vertex between a
  named and an unnamed edge refused. Simpler — no walk — and every edge
  in the result is one the caller asked for. Rejected: Open CASCADE and
  every CAD the consumers come from propagate, the oracle's bodies are
  propagated ones, and a consumer naming a single rim edge of a part read
  from STEP cannot know where the file split the rim.
- **One blend face over a chain of one surface** (the split rim's two
  half tori as one torus face). Fewer faces, but the face would be
  `Generated` from two edges, the one-face-per-edge naming would break,
  and the oracle's counts would differ on every split rim.
- **The junction arc `Generated` from the vertex** it replaces. A vertex
  is an input entity the audit accepts as a root, but the consumer names
  blends from edges, and a miter — the same construction with a sharp
  third edge — is already recorded from both edges.
- **The open arc's end on a plane off the axis fitted** as a NURBS
  section of the torus. The quartic is the general torus section C3
  traced for the boolean (ADR-0019) and could be borrowed, but the end
  then needs a fitted 3D curve, which ADR-0007 rules out; it stays a
  refusal until a census asks for it.

## Amendment (2026-10-02, plan `c6-blend-network` step 4)

- **A tangent vertex also joins two edges of one sense: both convex or
  both concave.** §3 called a convex run meeting a concave one at a
  junction unreachable. It is not: NIST FTC-06 has a plate's bottom edge,
  rounded where the outline turns (convex), running tangentially into the
  rim of a boss hanging below the plate (concave). The wall above and the
  boss's wall below are tangent along the vertical edge between them, so
  §1 as written made the vertex a tangent vertex. The ball cannot cross
  there: the outline inflects, and the two contacts on the plate's bottom
  lie on opposite sides of it. Open CASCADE's walk (`PerformElement`)
  checks face tangency and turn-back only, so it does not settle the case.
  §1's test now also requires one sense. Such a vertex stops the walk, and
  the end there is §6's `TangentChain`, naming the edge, the tangent
  corner edge and the vertex. With that condition §3's consistency check
  stays an internal fault that cannot be reached.
- **An open arc reads its ends' corners before its contacts**, as a line's
  stripe does, so an end at a tangent corner edge is `TangentChain` rather
  than the `BlendTooLarge` of a contact that runs past it.
- **The horn torus stays refused.** The cap-edge chain at the step-1
  radii (first fillet 0.2, second 0.1) puts a torus of major radius equal
  to minor on the first blend's end arc. Open CASCADE builds it, and the
  ring refuses any torus that is not a ring torus (§4, ADR-0007) as
  `BlendTooLarge`. The fixture that builds is held at a first radius of
  0.3. The horn torus is `regression/tangent-chain-horn-torus` and a
  backlog line.
