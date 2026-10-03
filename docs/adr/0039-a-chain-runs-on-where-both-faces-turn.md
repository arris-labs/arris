# ADR-0039 — A chain runs on through a vertex of four edges where both of its faces turn tangentially

- Status: accepted (2026-10-03)
- Plan: `blend-corners` step 3 (the decision, the walk and its test); the
  junction built on lines in step 4, with arcs among its runs in step 5
- Amends: ADR-0035 §1 (a tangent vertex may have four edges), §3 (the
  junction's two points may both lie on cut edges) and §6 (what
  `VertexBlend` keeps); ADR-0007 §"Everything outside the table" (a
  vertex of four edges is no longer refused as such)

## Context

`blend-corners` step 1 counted `VertexBlend` by the site that refused it,
and its leading site was *a vertex of four edges whose one extra edge is a
sharp edge between two faces across*: 160 of the committed tier's 187
`VertexBlend` edges, 104 of the fetched tier's 227, Open CASCADE building
every one it was asked about on a plane against a plane or a cone against a
cylinder. The plan read that site as a *fan*: the face across an end split
in two, the end arc crossing both pieces.

Step 3 looked at the sites before deciding, reading each vertex's four
edges: which two are the corner edges (each sharing a face with the
blended edge), whether each corner edge's two faces are tangent at the
vertex, and how the extra edge leaves it. On the committed tier:

- **142 of the 160 are not a fan.** Both corner edges are tangent
  dihedrals, and the extra edge leaves the vertex exactly along the blended
  edge's own line: it is the blended edge's tangent continuation, its two
  faces each tangent to one of the blended edge's faces across one corner
  edge. CTC-04 holds 128 of them: a pin whose cylinder and conical tip are
  both split in half at the same vertex, the rim between them being two
  half circles (104), and a chamfered wall running into a rounded corner,
  the wall's line becoming the corner's circle where the plane wall turns
  into a cylinder and the plane chamfer into a cone (24). FTC-09 holds 8 of
  the first kind, CTC-01 4 and FTC-06 2. Every sampled edge of the
  battery's three refused solids (CTC-04 twice, FTC-09 once) is one of
  these.
- **18 are a fan,** all on CTC-01, a plane against a plane: both corner
  edges sharp and the extra edge between two faces across, leaving the
  vertex at 60° or 49° to the blended edge. No sampled edge is one.

The census named the extra edge *sharp* because it compared it with the
corner edges only, never with the blended edge. The census now names the
blended edge's continuation apart.

Open CASCADE builds the continuation because its fillet propagates a
selection along tangent-continuous edges: the half circle's fillet is the
whole circle's, and the wall's runs on round the corner. That is
ADR-0035's chain, at a vertex it did not admit: ADR-0035 §1 asks for
exactly three edges, the blended edge and the next sharing one face. Here
they share none, because both faces turn.

The shrunk case is a chamfered stadium whose chamfer's foot is filleted
(`regression/chamfered-stadium-foot-fillet`): the foot line, between the
side plane and the chamfer strip, runs on into the foot arc between the
half cylinder and the half cone, at a vertex whose two other edges, side
to half cylinder and strip to half cone, are tangent dihedrals. Open
CASCADE fillets the whole foot outline. No reference module was read for
this decision; the oracle's result is what was compared.

## Decision

**1. A vertex of four edges is a tangent vertex where both faces turn.**
At an end vertex `v` of a blended edge `e` with faces `F₀` and `F₁`, `v`
is also a tangent vertex when it has exactly four edges `e`, `e′`, `w₀`
and `w₁`, with:

- `e′` open, not a tangent dihedral, and of `e`'s convexity;
- `e` and `e′` sharing no face, `e′`'s faces being `G₀` and `G₁`;
- each `wₖ` tangent at `v` and between `Fₖ` and `Gₖ`, the two `w` using
  four different faces;
- the direction leaving `v` along `e′` within a right angle of the one
  arriving along `e`.

The selection follows it as ADR-0035 §1 follows a vertex of three edges:
`e′` is blended with the same kind and size, and the walk goes on from its
far vertex.

**2. The junction there is the one ball's, with a cut edge at each end.**
`Fₖ` and `Gₖ` have one normal at `v`, so the ball touching `e`'s faces
and the ball touching `e′`'s are one ball, as at ADR-0035 §3. The junction
arc is its great circle (a fillet) or the chord (a chamfer) square to the
edges' common direction. It runs between the point `p₀` where the
contacts on `F₀` and `G₀` meet on `w₀` and the point `p₁` where those on
`F₁` and `G₁` meet on `w₁`. Every pcurve of it is exact, as at a vertex of
three edges. Both `w₀` and `w₁` are cut at their points, as ADR-0035
cuts its one `w`. The vertex goes. No face across takes the arc: the two
blend faces meet along it at a tangent dihedral. Against ADR-0035's
junction, the point `q` on a shared face is replaced by a second cut edge.
A `w` shorter than its cut is `Reason::BlendTooLarge`, naming the edge and
that `w`.

**3. Provenance is ADR-0035 §5's.** The arc and its two vertices are
`Generated` from both edges. `w₀` and `w₁` are `Modified` into their cut
selves. `v` is `Deleted`. For the chamfered stadium's foot, the counts go
from the chamfered stadium's 12 vertices, 20 edges and 10 faces to 16, 28
and 14: each of the four foot edges is replaced by two contacts and a
blend face, and each of the four junctions adds its arc and its two
points in place of its vertex. These are Open CASCADE's counts.

**4. What `VertexBlend` keeps.** A vertex of four edges that is not a
tangent vertex by §1 is still `VertexBlend`, the fan among them: two
sharp corner edges with the face across split by a sharp edge. So is a
vertex of four edges where only one corner edge is tangent, and one whose
extra edge is smooth, a seam, or continues a corner edge (step 1's census:
Open CASCADE builds 2 of the 37 of those it was asked about). `Reason`
gains no variant. *(Amended by ADR-0043: the fan, and the vertex where the
face across is met twice, are built.)*

## Consequences

- The site that holds the battery's refused sample on CTC-04 and FTC-09
  is a chain junction, not a corner. Once §2 is built, those parts meet
  whatever stands behind it on the chain: on CTC-04 the battery's fillet
  stage already reaches a named tangent dihedral (`TangentChain`) once the
  walk passes the vertex.
- Walking further can reach a pair outside the table where the refusal
  used to be the vertex: a chain into a sphere, a NURBS face or a torus
  off its axis is refused naming that pair, as ADR-0035 §1 says of any
  chain.
- The walk judges each `w` tangent as ADR-0035 does, by the face
  normals at the model's angular precision (`1e-12`). The split rims pass:
  their two halves are one surface read twice. CTC-04's chamfered walls do
  not. As read, the plane wall meets its rounded corner and the chamfer
  strip meets its cone with normals `7e-11` to `9e-10` apart, which moves
  the ball by `r` times that, far inside the edges' `1e-7`. So the walk
  takes 104 of CTC-04's 128 committed edges and all 8 of FTC-09's. Judging
  tangency by the edge's own tolerance instead is a change to ADR-0035's
  test for every chain, and is the plan's open question, not this
  decision. *(Decided by ADR-0040: tangent for the blend, its size times
  the normals' sine within the faces' tolerance; the walls walk.)*
- The fan, the plan's opening hypothesis, is CTC-01's 18 edges and frees
  no part of either tier's column. It stays a backlog line, with
  `blend/five-edge-vertex`, a vertex of five edges where the face across
  appears twice.

## Alternatives considered

- **Build the fan first, as the plan opened.** It is a sixth of the site,
  on one part whose column it does not move. The census's label was what
  made it look like the lead.
- **Leave the continuation to tangent continuation's own line**
  (`TangentChain`, STC-09). That line is about an edge that is itself a
  tangent dihedral. This site is refused `VertexBlend`, its junction is
  ADR-0035's with one point moved onto a second cut edge, and it is what
  holds the parts.
- **Merge the two halves of a split rim into one edge before blending.**
  That covers CTC-04's pin and not its chamfered wall. It also changes the
  operand's topology behind the consumer's back, and the consumer names
  one blend face per edge (ADR-0035 §2).

*(ADR-0041 takes the vertex of two edges of a split rim; the rejection of merging halves above stands.)*
