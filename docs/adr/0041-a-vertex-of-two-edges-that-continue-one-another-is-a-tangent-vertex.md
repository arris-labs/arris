# ADR-0041 — A vertex of two edges that continue one another is a tangent vertex

- Status: accepted (2026-10-03)
- Plan: `split-rim-vertex` step 2
- Amends: ADR-0035 §1, §3, §5 and §6 (the tangent vertex, its junction, its
  provenance, what `VertexBlend` keeps). ADR-0039's rejection of merging the
  halves of a split rim into one edge stands.

## Context

The census after `blend-corners` (`census::vertex_blend_cause`) found the
site left in `VertexBlend` to be one case, with no exception: a circle of
a plane against a cylinder split into two half arcs, at the second vertex
of the split, where nothing but the two arcs meets. The first vertex has
the cylinder's seam, a vertex of three edges whose third the cylinder uses
twice; ADR-0035 §1 asks of a third edge that it lie between two faces the
blended edges do not share, so it does not walk through that vertex either
— the census never named it because each arc's far vertex refused first
(found at step 3). The second has no third edge, so ADR-0035 §1, which asks for exactly three,
does not take it and `corner_of` refuses the blended arc as `VertexBlend`.
Fetched, 67 of the 79 refused edges (CTC-01 4, CTC-04 34, STC-09 29);
committed, all 20 (CTC-01 4, FTC-06 10, FTC-10 6). Open CASCADE builds 26 of
27 asked on the fetched tier and 17 of 20 on the committed one. The shrunk
case is `regression/split-rim-two-edge-vertex-fillet`: FTC-06 read in place,
one half of a hole's rim filleted. Open CASCADE builds a half torus on each
arc, the faces 144 → 146, the edges 373 → 377. No reference module was read;
the oracle's result is what was compared.

The two arcs share both faces and one circle. The ball touching `e`'s two
faces and the ball touching `e′`'s are the one ball, as at ADR-0035 §3, and
there is no third edge for a junction to cut.

## Decision

**1. A vertex of exactly two edges is a tangent vertex where the edges
continue one another.** At an end vertex `v` of a blended edge `e`, `v` is
a tangent vertex when it has exactly two edges `e` and `e′`, or those two
and a seam of one of their faces (an edge that face uses twice, found at
step 3), where `e` and `e′` share both of their faces, `e′` is open and not a tangent dihedral, and the
direction leaving `v` along `e′` is within a right angle of the one
arriving along `e`. The selection follows it as ADR-0035 §1 follows a
vertex of three edges: `e′` is blended with the same kind and size, and
the walk goes on from its far vertex. A chain of arcs whose vertices are
all of one of the kinds (three edges with the seam, four, two) is closed
when the walks meet.

**2. The junction is the ball's cross-section, with nothing cut.** The
junction arc is the great circle (a fillet) or the chord (a chamfer) in
the plane square to the edges' common direction at `v`, between the two
contacts of each stripe: the contact on `F₀` meets that on `F₀` of the
other stripe at one end, the contact on `F₁` the other's at the other.
Its pcurves are exact, as ADR-0035 §3: a line at constant `u` on a torus
or a cone, at constant `v` on a cylinder. Its tolerance is the larger of
the two stripes'. The two blend faces meet along it at a tangent dihedral.
Each of `F₀` and `F₁` keeps its loop, with `v` replaced by the one contact
vertex of its own and the junction not in it. At a vertex of two edges no
edge is cut, because `v` has none but `e` and `e′`; at the seam's vertex
the seam is shortened to the contact vertex on its face, as a closed
edge's seam is (ADR-0036), and `p` is that one.

**3. Provenance is ADR-0035 §5's.** The junction arc and its two vertices
are `Generated` from both edges, `v` is `Deleted`, a seam shortened is
`Modified`, and the two halves stay two blend faces (ADR-0035 §2). Each
vertex of the rim trades itself for the junction's two, and each arc
itself for two contacts, so a rim of `n` arcs adds `n` vertices, `2n`
edges and `n` faces: Open CASCADE's counts on the plate and boss operands
its own writer splits (`blend/split-boss-base-*`,
`blend/three-arc-hole-rim-fillet`). On FTC-06 itself it builds two blend
faces that each span a whole turn, their contacts closed circles and
their two junction edges one tube circle twice, its genus one higher
than the part's (found at step 3); that fixture is held to Arris's counts
and to the closed forms (ADR-0015). The plan's question whether a vertex
of two edges needs a junction at all is answered yes, by the counts on
the operands Open CASCADE builds whole.

**4. What `VertexBlend` keeps.** A vertex of two edges whose edges share
fewer than both faces, turn back, or are a tangent dihedral stays
`VertexBlend` or `TangentChain` as ADR-0035 §6 says. `Reason` gains no
variant.

## Rejected alternatives

- **Merge the halves into one edge before blending.** Healing's job
  (ADR-0039), and the consumer names one blend face per edge.
- **Blend one half torus over both arcs.** One face where Open CASCADE
  builds two changes the consumer's naming and the oracle's counts on
  every split rim.

## Consequences

- A rim split into any number of arcs blends as the closed chain it is.
  A seam ends on the rim — the reader rebuilds a band's missing seam
  through a vertex of each loop — so a rim on a cylinder band has the seam
  at exactly one of its vertices, never at none (found at step 3).
- The census's `a vertex of 2 edges, the second: a continuation between the
  same two faces` should read zero after step 3 on both tiers, apart from
  the faces Open CASCADE refuses itself.
