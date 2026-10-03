# Idea: blend-fan

- Status: Parked (2026-10-03). Split out of `tangent-continuation-and-fan`,
  whose other half (the S-bend walk) became `plans/s-bend-walk`.
- Raised: 2026-10-03
- Prompt (verbatim from the human): "/idea tangent-continuation-and-fan",
  then "I accept B, /plan it"

## Problem
A blend can end at a *fan*: a vertex of four edges where both corner edges
are sharp and the face across is split in two by a sharp edge `x`, so the
end arc crosses both pieces. CTC-01 has 12 plane × plane edges like this
on each tier, and Open CASCADE builds every one (`vertex-blend.md`,
2026-10-03). None of them is in CTC-01's battery sample. CTC-01 is held
in the column by its crossing cylinders, so building the fan moves no
part on either tier (ADR-0039 found the same). Beside it is
`blend/five-edge-vertex`, a vertex of five edges where the face across
appears twice.

## Constraints it runs into
- ADR-0007 and ADR-0039 §4: the fan is a new end construction. The end
  becomes two pieces, one on each face across, with a new vertex where it
  crosses `x`, and `x` is `Modified`. That needs its own ADR.
- ADR-0020: work is ranked by what it removes from the histogram, and the
  fan removes nothing today.

## Options
### A — Build the split end
About 4–5 steps: an ADR, the construction on plane × plane, fixtures from
CTC-01 with Open CASCADE's oracle, a property over random poses, and a
measurement.
### Do nothing (chosen for now)
CTC-01's 12 edges stay `VertexBlend` with a named cause. No part waits on
them.

## Recommendation
Stay parked. Reopen if a part's battery sample, a newly fetched part, or
the first consumer's side-by-side run hits a fan edge.

## Decision for the human
None until it is reopened. An ADR is needed when it is.
