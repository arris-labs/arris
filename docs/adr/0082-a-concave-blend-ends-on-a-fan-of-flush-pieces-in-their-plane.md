# ADR-0082 — A concave blend ends on a fan of flush pieces, in their plane

- Status: accepted (2026-11-01)
- Plan: `flush-fan-end` step 1 (the decision, against the oracle)
- Amends: ADR-0043 §1 and §2 (the smooth extra edges they leave to
  `VertexBlend`), ADR-0039 §4 (what `VertexBlend` keeps)
- Reference trees read: none; Open CASCADE run as the oracle

## Context

A fuse leaves faces on one plane unmerged (C11 Out; ADR-0043's
alternatives): the caps of two fused boxes, the front of a block standing on
a slab, the plate's end and a cylinder's cap. A concave fillet or chamfer
that ends on such a face meets it as several pieces, and at the end's
vertex `v` the seam between two pieces is a further, **smooth** edge.
ADR-0043 §1 refuses a smooth extra, so the end is `VertexBlend`, though the
pieces are one plane and the end is the closed form of a lone face across
(ADR-0007). The convex ends of the same bodies build.

Open CASCADE builds all five fixtures
(`regression/flush-l-inner-{fillet,chamfer}`,
`flush-step-concave-{fillet,chamfer}`, `knuckle-join-flush-end-fillet`).
Counts of the body before and after the blend, vertices / edges / faces:

| body | before | after | change |
|---|---|---|---|
| L of two fused boxes, fillet or chamfer | 14 / 23 / 11 | 18 / 28 / 12 | +4 / +5 / +1 |
| slab and block, fillet or chamfer | 14 / 23 / 11 | 18 / 28 / 12 | +4 / +5 / +1 |
| plate and half-buried cylinder, fillet | 14 / 23 / 11 | 18 / 28 / 12 | +4 / +5 / +1 |

A lone-face L (`blend/l-inner-edge`) goes 12 / 18 / 8 to 14 / 21 / 9. The
flush bodies gain the same +1 face and +3 edges at the blend, and no vertex
is lost at either end: `v` survives, and each end keeps one more edge.

## Decision

**1. The site (a *flush fan*).** The end of a blended edge `e` at a vertex
`v` of more than three edges, as ADR-0043 §1, except that the walk from
`c1` to `c2` (§2) meets only **smooth** extra edges: each lies between two
planar faces on one plane, the pieces. A sharp extra, a seam, or a piece
off the plane stays as ADR-0043 and ADR-0039 §4 have it. A flush fan is for
a concave `e` at a face end only.

At such an end the corner edges `c1` and `c2` are **convex**, the walls'
edges with the cap, as at a lone face across, and the extra edges are the
ones that are tangent (`tangent_at`) along their length. The walk's test
reads the corner edges as sharp and convex, the extras as smooth.

**2. The construction.** The end's section `a` lies on the pieces' plane,
in the wedge between `c1` and `c2` on the side of `v` that holds no piece
of the walk, so it crosses no extra. It is ADR-0007's arc or chord for one
lone face across. Let `P1` and `P2` be its ends on `c1` and `c2`.

- **The region** that `a`, `c1` from `P1` to `v` and `c2` from `v` to `P2`
  enclose goes to **`A1`**, the piece `c1` leads to. The counts above do
  not tell `A1` from `A2` apart (neither adds a face), so the pick is made
  for determinism: `A1` is the piece of `c1`, the corner edge in `F1`, where
  `F1` is the first face of `e`.
- **`c1` is cut at `P1`**: its part from `v` to `P1` is inside `A1` now and
  is dropped. **`c2` is cut at `P2`**: its part from `v` to `P2` stays, as
  an edge between `A1` and `A2`, the way ADR-0043 §4 cuts and keeps edges.
- **`v` survives** with the extras unchanged.

The counts follow: each end adds two vertices (`P1`, `P2`), the blend adds
the same edges as a lone face's, and each end adds the one kept edge from
`v` to `P2`: +4 vertices, +5 edges, +1 face, as Open CASCADE's.

**3. Provenance.** `v`: kept, no record. `P1`, `P2` and the arc: `Generated`
from `e`. `c1`, `c2`, `A1` and `A2`: `Modified`. Middle pieces: untouched.

**4. Refusals.** If `a` meets an extra's curve within the extra's range, the
end is `BlendTooLarge`, naming `e` and that extra. Pieces on a non-planar
surface (no fixture reaches the extras check; the arc cannot reach an
extra, so it is a guard and a corner edge's far vertex is what the fixture
holds), pieces not coincident within their tolerance, a mixed-sense extra
and a convex blend at a smooth extra stay `VertexBlend`. `Reason` gains no
variant.

## Consequences

- The hinge leaf's `knuckle_joins` and the two fused-box bodies build.
- Pieces on a cylinder or a sphere stay refused, by name, until something
  asks.
- The build reuses the lone face's end for the first piece (`face_end`,
  `finish_end`), the arc checked against the extras as `stays` are; the
  rewrite adds the kept part of `c2` as a new edge (`flush_tail`) with `c2`
  as its parent, and the two faces' loops take it at the junction.
- The coincidence test of the pieces is a step 2 decision; it uses the
  faces' tolerance and adds no literal.
