# Idea: sew-step-crescent-vertices

- Status: Open
- Raised: 2026-11-18, a finding while enabling V4 (ADR-0090).

## Problem

Open CASCADE exports a crescent cap as an outer bound and a hole with
separate `VERTEX_POINT` instances at the contact. Its reader identifies
both instances with one vertex. Arris keeps both ids, so its read-back
measures correctly and counts extra vertices. The plain STEP read-backs
of `boolean/fuse-cylinder-tangent-inside` and
`boolean/fuse-arc-plate-tangent-inside-cylinder` hold this as
`occt_step_counts_differ`. The end-disc fixture also has a loop that
arrives at one instance and leaves at the other, which the reader
refuses as open.

The cylinder read-back also fails as a boolean operand. Fuse its STEP
solid `#15` with a box from `(100, 100, 100)` to `(101, 102, 103)`.
The box is disjoint, but the face split reports a hole inside no piece
on `f1` before result verification. The same refusal occurs before V4.
The sewn read-back should fuse to two lumps with additive volume.

## Constraints

Shells keep separate topology (ADR-0006, ADR-0090). The STEP reader
names every source in provenance (ADR-0025). A real short edge keeps
its two ends (ADR-0074). Sewing a vertex where two loops touch requires
updating the loops so the builder's Euler–Poincaré line still closes.
V4 runs on boolean results while these STEP representations are admitted
by the general checker.

## Options

### A — Sew the contact on the face
Identify the duplicate instances at a bound junction within the face
and splice the loops through their common vertex. Map both original ids
to the sewn vertex. Verify the Euler line, orientation and pcurves
with the two passing examples and the refused end-disc case. This
needs a reader plan with separate sewing and read-back steps.

### B — Keep the current representation
Preserve the file's two vertex ids and the affected counts exceptions.
The end-disc file continues to refuse and V4 remains scoped.

## Recommendation

A when the reader work is ranked. The known imports and their oracle
records can hold the measures, topology and source ids through sewing.

## Decision for the human

❓ **Q1 — Sew STEP crescent vertices**: Should the reader do the
face-local sewing in A when this reader work is picked up?

➡️ Yes, with an ADR covering the merge, loop repair and provenance.
