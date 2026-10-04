# ADR-0045 — A blended edge's own dihedral is read tangent within the edge's tolerance

- Status: accepted (2026-10-04)
- Plan: `blend-miters` step 6 (⚠ OPEN 3)
- Amends: ADR-0040 §3 (the faces' tolerance, not the edge's) for the one
  test it names that builds nothing

## Context

ADR-0040 reads two faces as tangent for a blend of size `s` when
`|n₁ × n₂| ≤` the angular precision or `s |n₁ × n₂| ≤ t`, with `t` the
larger of the model's default tolerance and the two faces' own, and §3
chose that `t` over the edge's on purpose: the stripes' contacts are built
and met at the faces' tolerance, a junction accepts two runs' contacts only
within it, so a tangency admitted at the edge's wider tolerance could put
them further apart than that, an internal fault.

The committed NIST CTC-04 and FTC-08 and the fetched FTC-10 hold 13 edges
that ADR-0040 leaves unread. Each is a circle written as a NURBS curve (or,
at FTC-10, a NURBS face) between a sphere and a cylinder of its radius
whose axis runs through its centre — tangent along the circle as designed.
The files write their coordinates to about `1e-6`, so the normals are
`7.5e-7` to `1.2e-5` apart, `s · sine` is `5.9e-7` or more at the census's
radii against the faces' `1e-7`, and the readers' healing set the edges'
tolerances to `8e-6` to `1.5e-4`, which is where the file's gap went. The
edges were refused as an unsupported cylinder against a sphere, and their
own twins with a smaller gap as `TangentChain`. `blend-miters` step 6
expected a refusal-order fault; there is none, the edges are read as
creases.

## Decision

**1. The blended edge's own dihedral** (a stripe's and a ring's
`TangentChain`, read at the edge's midpoint) is tangent when the ADR-0040
test holds with `t` the largest of the default, the faces' and the **edge's**
tolerance.

**2. Nothing else changes.** A corner edge at an end's vertex, the other
edges at a tangent vertex, the next edge the walk reads and the junction's
third edge keep the faces' tolerance (ADR-0040 §2–3). The refusal builds no
stripe, so ADR-0040 §3's reason does not reach it: an edge so read is
`TangentChain` naming itself and its two faces and nothing is constructed
from its contacts.

`Reason` gains no variant, and no public type or signature changes.

## Consequences

- On the committed tier, CTC-04's 7 edges and FTC-08's 4 (2 of them
  previously `BlendTooLarge`) and FTC-10's 2 are `TangentChain`, with the
  census naming "the edge itself, a tangent dihedral". Every edge that built
  before still builds (505, 326 and 141 for the three parts).
- A real edge whose faces are `θ` apart but whose tolerance is wider than
  `s · sin θ` is read tangent; the edge's tolerance is a model's own
  statement of how well its faces meet there.

## Alternatives considered

- **Widen every tangency read to the edge's tolerance.** Rejected: ADR-0040
  §3's reason holds for junctions and corner edges.
- **Leave them `Unsupported` and attribute them to the NURBS cycle.**
  Rejected: nothing is NURBS in the blend they would have needed; the
  geometry is tangent and Open CASCADE refuses it as well.
