# Idea: touching-operands-fuse-to-several-shells

- Status: Accepted (A). Curve and point touches and the remaining hinge
  operations are complete under
  [ADR-0088](../adr/0088-lumps-of-one-solid-may-touch.md) and
  [ADR-0089](../adr/0089-resample-a-piece-at-an-isolated-touch.md).
  Q2 remains to plan.
- Raised: 2026-11-13 from the training harness's C12 fuse refusals.

## Problem

Several booleans would produce two shells that share an edge or vertex.
Arris refuses these as `NonManifold`. Open CASCADE builds them as separate
solids in one compound. The fixtures are `edge-touching-fuse`,
`cross-cylinders-cut`, `pockets-touching-edge-cut-many` and
`fuse-frustum-box-rim-point` with its `-far-side` twin. These cases occur in
cuts as well as fuses.

## Constraints

Q1 and Q3 are settled in
[ADR-0088](../adr/0088-lumps-of-one-solid-may-touch.md). A `Solid` can
hold **touching lumps** with disjoint interiors. Each shell keeps its own
edges and vertices under ADR-0006. The result has to work as an operand
or be refused by name under ADR-0020.

## Decision

**Q2 (2026-11-17, delegated to the agent): A.** Give each lump its own copy
of the edge or vertex when the result can be grouped into manifold
shells. The checker already accepts duplicated edges where the faces
meet on their own boundaries. The boolean's grouping by shared edge has
to learn the split. The fixtures keep their `NonManifold` refusals until
that plan lands. Option B would keep those refusals as the contract.

The plan has to settle what `weld` does with the copies, since merging
their coincident positions can create a non-manifold mesh edge.
