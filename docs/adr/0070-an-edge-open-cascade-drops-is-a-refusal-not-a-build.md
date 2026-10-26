# ADR-0070 — An edge Open CASCADE drops is a refusal, not a build

- Status: accepted (2026-10-26)
- Plan: `battery-honesty` step 1 (the evidence), steps 2–4 (the oracle,
  the corpus and the battery)
- Amends: ADR-0026 §5 and the amendment of step 9 (what blocks a part)
- Follows: ADR-0040 and ADR-0045 (the tangency tests)

## Context

C11 opened on a count: the blend network blocks 12 of 38 real parts at
`fillet`. Four of them (CTC-04, FTC-08, FTC-07 AP242, STC-07 AP242) are
refused by Arris at a sampled edge that is a tangent dihedral, and Open
CASCADE refuses that edge too when asked alone ("There are no suitable
edges for chamfer or fillet"). Yet the battery's stage with the same edge
among the others *builds* in Open CASCADE, so ADR-0026 counts it as Open
CASCADE building and Arris refusing.

`ChFi3d_FilBuilder::Add` (`ChFi3d/ChFi3d_FilBuilder.cxx`) calls
`PerformElement` (`ChFi3d_Builder_1.cxx`), which returns false, and so
adds no stripe, for an edge with one face, a degenerate edge, or an edge
whose faces `ChFi3d::IsTangentFaces` finds tangent (continuity above C0, or
the two faces' normals within `max(0.001, 1.5 × the edge's tolerance)`).
`Add` ignores the false: the edge is left out without a word, and
`BRepFilletAPI_MakeFillet` builds the rest. `Contour(edge)` is 0 for such
an edge.

## Evidence

Each part's battery sample of 4 edges at its radius, built in Open CASCADE
whole and with each edge left out in turn (volumes in the model's unit³; a
scratch script, not committed):

| part | edges with `Contour == 0` | volume, all 4 | volume without those edges |
|---|---|---|---|
| CTC-04 (#24) | 2 of 4 | 17518935.9074 | 17518935.9074 (identical) |
| FTC-08 (#27) | 2 of 4 | 503584.7437 | 503584.7437 (identical) |
| FTC-07 AP242 (#13272) | 1 of 4 | 1726280.2051 | 1726280.2051 (identical) |
| STC-07 AP242 (#210) | 2 of 4 | 1726242.0717 | 1726242.0717 (identical) |

A dropped edge changes nothing in the result, to the last digit, while
each contoured edge does. Open CASCADE did not blend the tangent edge on
any of the four parts, so none is the blend's; the step-1 stop did not
occur.

## Decision

1. **A dropped edge is a refusal.** An asked edge of a `fillet` or
   `chamfer` that no contour took (`Contour(edge) == 0` after `Build`)
   is recorded by the oracle beside the result (`dropped_edges` in
   `expected.json`, only when there is one), and the result stands. A
   battery stage whose oracle dropped an edge is `OracleRefuses`, or
   `BothRefuse` when Arris refuses: ADR-0026's "only a stage Open CASCADE
   builds" reads as "builds what was asked".
2. **The sample uses the blend's tangency test.** `is_tangent_dihedral`
   takes its tolerance as the largest of the model's default, the faces'
   and the edge's own (ADR-0045 §1), so the sample leaves out what the
   blend refuses.
3. **The oracle never raises for a drop.** An oracle refusal deletes
   `expected.json` and would strip the numbers from fixtures that hold
   them.
4. **The corpus lint** refuses a fixture whose oracle dropped an edge
   unless the fixture expects an error: a comparison against a body
   missing a blend is not a comparison.

## Consequences

- The refusal histogram counts only blends Open CASCADE really builds and
  Arris refuses; C11's Accept is restated against the corrected count.
- Cached oracle answers for `fillet` predate `dropped_edges`; the cache is
  cleared once by hand (not keyed by the oracle's source).
- Open CASCADE dropped two edges of CTC-04's sample, FTC-08's and STC-07
  AP242's, and one of FTC-07 AP242's; the census names Arris's refusing
  edge only for the first of each. Whether Arris blends the others is
  not settled here: the dropped-edge rule counts the stage as refused
  either way.
