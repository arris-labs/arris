# ADR-0077 — A vertex stands for the edge samples inside its tolerance

- Status: accepted (2026-10-30)
- Plan: `tolerant-tangency` step 7
- Amends: ADR-0052 (an edge's samples are a function of the edge: now of
  the edge and its end vertices' points and tolerances), ADR-0003
- Reference trees read: none

## Context

CTC-05 AP242's battery edge ends at a vanishing end: the skin's end row
(`1.97e-2` wide) is a degenerate edge at a vertex whose tolerance is
`1.99e-2`. The skin's mesh fans its pole to the contact edge's last
samples, and the torus beside it fans its own sliver from the same vertex
over the same samples: those spokes are in four triangles, the mesh is
not closed (`TriMesh::is_closed`), though the checker holds the body.

## Decision

An edge's sampler skips an interior sample that lies within an end
vertex's tolerance of that vertex: the model says the two are one point.
The edge's range ends and the loops' (u, v) read at the samples kept.
A vertex at `1e-7` skips nothing a chord of `1e-3` samples.

## Consequences

- The battery edge's mesh is closed, volume within the fixture's bound.
- A kept edge's samples now also depend on its vertices' tolerances: an
  operation that grows a kept vertex re-samples the edges at it, which
  ADR-0052's per-face cache already treats as a change of the edge.
  None of the operations here grows a kept vertex (ADR-0074).
