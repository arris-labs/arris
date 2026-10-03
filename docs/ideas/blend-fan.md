# Idea: blend-fan

- Status: Accepted (2026-10-03): option B, planned as the second active
  plan once the crossing-cylinder plan opens; no backlog fallback.
  Reopened 2026-10-03 after being parked earlier that day. Split out of
  `tangent-continuation-and-fan`, whose other half (the tangent
  continuation, which turned out to be a cusp) became `plans/cusp-run-out`,
  now retired (ADR-0042).
- Raised: 2026-10-03
- Prompt (verbatim from the human): "/idea tangent-continuation-and-fan",
  then "I accept B, /plan it"; reopened with "/idea blend-fan", then
  "I agree with your recommendations"

## Problem
A blend can end at a *fan*: a vertex where both corner edges are sharp and
the face across is split by one or more sharp extra edges, so the end has to
cross several faces across instead of one. CTC-01 has 12 plane × plane edges
like this on each tier, all of the fetched tier's remaining `VertexBlend`
edges and 12 of the committed tier's 15 (`ftc-06` holds the other 3), and
Open CASCADE builds every one. `blend/five-edge-vertex` is the same site
with three pieces across, the lower box's top face twice and its side once,
and Open CASCADE builds it too.

What it does to the histogram: nothing. CTC-01 stays in the `fillet`
column because of its crossing cylinders on both tiers, and no fan edge is
in its battery sample (ADR-0039). What changed since it was parked is that
the cusp plan is retired. C6's own list ("the fan (CTC-01) and the miters
remain", `docs/ROADMAP.md` §C6) is now mostly the crossing cylinders, a
cylinder against a sphere, and the fan. The cycle can't close until each one
is either built or moved out by name.

## Constraints it runs into
- ADR-0007 / ADR-0039 §4: the fan is a new end construction and needs its
  own ADR. The end is one piece per face across, each cut the way
  `across_of` (`crates/arris-ops/src/blend.rs`) already cuts a single face
  across: a closed form or a trace (ADR-0035, ADR-0037). There is a new
  vertex where the end crosses each extra edge, and each extra edge is
  `Modified` (shortened to that vertex).
- ADR-0020: work is ranked by what it removes from the histogram. By that
  measure the crossing cylinders come first (CTC-01 on both tiers, 2
  parts), and the fan can only be justified as closure (AGENTS.md "closure
  before breadth"), not as a ranking win.
- ADR-0042 already cuts a stripe with a second wall between two trim points.
  The fan's piecewise cut is close to that, so this is mostly reuse rather
  than a new mechanism.

## Options
### A — The two-piece fan, plane × plane only
This covers CTC-01's 12 edges and nothing else. It takes 4–5 steps: the
ADR, the construction, a shrunk CTC-01 fixture with its oracle, a property
over random poses, and a measurement. It hard-codes "two pieces", which
`five-edge-vertex` breaks right away.
### B — A fan of k pieces across, each piece cut as a lone face across would be
The end walks the faces across in order around the vertex, between the two
corner edges. Each piece gets the end cut `across_of` gives it, and a vertex
goes on each extra edge. A piece `across_of` refuses keeps `VertexBlend`,
naming that face. This covers CTC-01 and `five-edge-vertex` (which moves
from its `expect_error` to Open CASCADE's counts, 19 / 28 / 12), for both
fillet and chamfer. It costs about one step more than A (5–6), and nothing
is foreclosed: a curved piece across gets the trace when `across_of` admits
it.
### Do nothing (move it out of C6)
At `/close-cycle` the fan becomes a backlog line, and the 12 edges stay
`VertexBlend` with a named cause. This costs nothing today. The cost is
that C6 closes with a plane × plane blend, the simplest pair there is,
still refused at a site Open CASCADE always builds, and that is the kind of
gap the first consumer's side-by-side run will find (ADR-0017).

## Recommendation
**B, planned after the crossing-cylinder plan opens, as the second active
plan.** The cylinders are the plan ADR-0020 ranks first, and they are the
larger of the two (a new pair in ADR-0007's table). The fan is small,
reuses the end machinery from ADR-0037 and ADR-0042, has a waiting
fixture, and once it is built the fetched tier has no `VertexBlend`. A
loses to B because its first counterexample is already committed. Doing
nothing loses because it leaves the simplest pair refused at C6's close
just to save five steps.

What would change my mind: if the shrunk CTC-01 fan shows the end can
leave a piece across before it reaches the extra edge (the end crossing a
third face, or the extra edge too short for the ball). That is a
`BlendTooLarge` case in the shape of ADR-0038, and if it is common the
cost goes up by a step or two.

## Decision for the human
1. Reopen the fan as option B (k pieces, `five-edge-vertex` included)?
   *Preferred: yes.* An ADR is needed, amending ADR-0039 §4.
2. Order: plan the crossing cylinders first and the fan beside it as the
   second active plan, or the fan first because it is smaller?
   *Preferred: the cylinders first, since ADR-0020 ranks them; the fan
   second.*
3. If not built in C6, does `/close-cycle` move it to the backlog by name?
   *Preferred: not applicable, build it; otherwise yes, as a `VertexBlend`
   line naming CTC-01's 12 edges.*
