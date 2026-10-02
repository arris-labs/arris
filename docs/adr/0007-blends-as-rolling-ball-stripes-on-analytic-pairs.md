# ADR-0007 — Blends are rolling-ball stripes on analytic face pairs, built in closed form and assembled with kept ids

- Status: accepted (2026-09-13)
- Plan: `fillet-and-chamfer` step 1 (one convex plane–plane edge, ends
  trimmed by the face across); the corner closed forms, the ruling and
  circle arms and the chamfer land in its later steps under this decision

## Context

C2's first line (`docs/ROADMAP.md` §C2) is a constant-radius fillet and
a flat chamfer of named edges, several in one call, on box edges, hole
rims and a boss's base, a second blend on a blended body, and two blends
meeting at a corner whose third edge stays sharp. The consumer's facade
names one blend face per edge and its probe corpus records where the
truck lineage fails: a second fillet on a filleted body, whose blends are
fitted surfaces the kernel cannot read back, and a vertical plus a cap
edge of a box, declined while two cap edges at the same kind of corner
blend. How a blend is built decides how much of C2's intersector and
checker lines the cycle has to take on: a general quadric intersector is
C3's, and every corpus result is held to `Full` with nothing unchecked.

Read in the reference trees. Open CASCADE's `ChFi3d` (`ModelingAlgorithms/
TKFillet`) builds one *stripe* per edge — a `ChFiDS_Stripe` of surface
data along a spine — and treats the corners between stripes separately:
`ChFi3d_Builder_C1.cxx` (`PerformOneCorner`, `PerformIntersectionAtEnd`)
ends one stripe at a vertex by intersecting it with the face across,
`ChFi3d_Builder_C2.cxx` (`PerformTwoCornerbyInter`) intersects two
stripes meeting at a vertex, and `ChFi3d_Builder_CnCrn.cxx`
(`PerformMoreThreeCorner`) fills a corner of three or more stripes with a
plate surface. Its analytic pairs are *known parts*, `ChFiKPart`:
`ChFiKPart_ComputeData_FilPlnPln.cxx` places the plane–plane fillet as a
cylinder whose axis lies where the two faces' offset planes meet, its
`X` at the first face's contact ruling and `u` running from `0` there to
the angle between the normals at the second, each contact a line on its
plane and a ruling on the cylinder — the construction this ADR takes,
reimplemented on Arris's frames; `FilPlnCyl` and `FilPlnCon` are the
plane–cylinder and plane–cone stripes the later steps read.
`monstertruck-fillet` (`ops.rs` `fillet`, `geometry.rs`) is the same
stripe per edge in Rust without exact surfaces: a rolling-ball surface
sampled into a NURBS and the two faces re-trimmed, which is what the
consumer's second-fillet probe records failing. The boolean's shared
paves, split in (u, v) and kept-id assembly are ADR-0004; a `Tangent`
face pair there contributes no section and a tangent contact interior to
two result faces is a refusal, which is what rules out the boolean-tool
option below.

## Decision

**One stripe per edge, built in closed form from its two faces.** A
blend is decided by the surface pair of its edge, by a table: two planes
blend to a cylinder of the radius on the line where the faces' offset
planes meet, and chamfer to a plane; a plane and a cylinder along a
ruling blend to a cylinder; along a circle they blend to a torus and
chamfer to a cone. Convex or concave is read from the dihedral — the
direction into one face from the edge against the other's outward
normal — and puts the ball's centre inside or outside the material. The
contact curves come from the construction, never from the intersector: a
plane's contact is the line at `r tan(φ/2)` from the edge, a `Line`
pcurve on the plane and a ruling at fixed `u` on the blend. The blend's
frame is the known part's: `X` at one contact so the contacts sit at
`u = 0` and `u = π − φ`, `Z` along the edge so `v` is the edge's own
parameter. Every surface is exact; nothing is fitted but a pcurve.

**Each end is trimmed by the face across the corner.** At a vertex of
three edges the stripe ends where the blend surface meets the third
face: a circle when that plane is perpendicular to the edge, an ellipse
when oblique, exact on the plane and a `Line` or a `Nurbs` fitted by
the oblique-section rule on the blend, at the arc's own tolerance. The
corner vertex goes; the corner's two other edges are shortened to the
arc's ends on their own curves; the face across takes the arc in its
loop between them. Corners of blended edges are closed forms too: two
equal-radius stripes at a vertex meet in the ellipse of their cylinders'
bisecting plane (a miter), three at a vertex of three planes in a sphere
through the ball's one centre. A closed edge has no ends.

**Everything outside the table is a typed refusal naming the edge, and
each is C6's.** A pair the table has no row for is `Unsupported` naming
the two kinds and faces; a tangent dihedral, or an edge that meets a
blend face, is `TangentChain`; a vertex of other than three edges, or a
corner the closed forms do not cover, is `VertexBlend`. There is no
marcher, no plate and no fallback.
*(Amended by ADR-0035: an end at a tangent vertex is a junction of two
stripes on the ball's cross-section and the selection follows the chain;
`TangentChain` keeps the tangent dihedral and a tangent end that is not
such a vertex. An open circle between a plane and a cylinder blends as
the ring does, its ends trimmed on a plane through the axis.)*

**The `BlendTooLarge` bound.** A contact curve or an end arc that would
leave its face through any edge but the corner's own is refused by name,
decided in the face's own (u, v) through `FaceDomain::side` at interior
samples, as is a corner edge shorter than the trim would cut from it. A
blend that meets a third face while its contacts stay inside their
faces — a hole nearer the edge than `r` — is not detected by the
operation in C2: S5 catches it in the corpus, and the fixture that shows
it is a `regression/` entry for C6.

**Assembled through `ops::rebuild` with untouched entities kept.**
`rebuild::rewrite` is the blend's entry: the operand's faces kept by id
unless their loops change, its edges and vertices kept unless shortened
or consumed, the blend faces added after the shell's own, the whole
proven by `Builder::assemble` as ADR-0004 assembles a boolean. Provenance
is rooted at the edge and needs no new `Role`: the blend face, its
contacts, its end arcs and its trim vertices `Generated` from the edge;
the edge's faces, the faces across its ends and the shortened corner
edges `Modified` into their new selves; the edge and its corner vertices
`Deleted`. A miter edge is `Generated` from both edges, a sphere corner
from its three. The consumer names one blend face per edge with no
matcher, and a second blend takes any body.

## Consequences

- The intersector and the checker's `Full` rows only ever meet a blend's
  surfaces in the positions the construction puts them — a plane
  perpendicular or oblique to a cylinder's axis, a torus coaxial with
  its hole, a sphere on a cylinder's axis — so C2's intersector and
  checker lines narrow to those arms (`docs/ROADMAP.md` §C2) and the
  general quadric pairs move to C3.
- The miter's equal-radius crossing-cylinder ellipse and the tangent
  parallel-axis arm are what S5 needs to check a blended corner and a
  ruling blend; until the cylinder–cylinder plan lands them, a miter
  result waits under `regression/` with exactly one S5 row unchecked.
- A fitted pcurve on the blend is the one non-exact item, held to the
  arc's tolerance by the fit and to E4 by the checker; a fit that misses
  the corpus is a finding with its fixture, never a raised tolerance.
- The `Reason` enum grows six variants, flat; grouping it by operation is
  a backlog line.
- Nothing here carries a variable radius, a tangent chain, a vertex of
  more than three edges or a blend over a blend; C6 extends the stripe
  model rather than replacing it, which is how `ChFi3d` grew too.

## Alternatives considered

- **The blend as a boolean tool**: per edge, an extrude or revolve of the
  corner-minus-quarter-disc profile, cut from a convex edge or fused onto
  a concave one, reusing the General Fuse and its provenance. The tool's
  blend face is tangent to the body's faces along exactly the edges that
  must survive — the `Tangent` pair ADR-0004 refuses as a contact or
  leaves unimprinted — the tool still has to stop at the edge's ends, two
  blends at a corner become a general cylinder–cylinder boolean, and the
  provenance roots at a private sweep's role. Nothing of it survives into
  C6.
- **A sampled rolling-ball surface fitted as a NURBS**, monstertruck's
  construction: every blend face becomes the free-form surface the
  consumer's second-fillet probe records the kernel unable to read back,
  and every S5 row against it is unchecked.
- **A general marcher for the ends and corners**, `ChFi3d`'s walking
  lines: the closed forms cover every corner C2 asks for, and a marcher
  is C4's NURBS intersector, not a blend's.
