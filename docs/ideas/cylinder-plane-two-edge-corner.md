# Idea: cylinder-plane-two-edge-corner

- Status: Open
- Raised: 2026-10-03
- Prompt (verbatim from the human): "/idea cylinder-plane-two-edge-corner"

## Problem
`VertexBlend` after `blend-corners` is mostly one site: a cylinder-against-plane
edge refused at a vertex of two edges (`corner edges that share no face
across, 2 edges`): 77 of the fetched tier's 79 refused edges (30 of 37 asked
build in Open CASCADE) and 20 of the committed tier's 35 (17 of 20 build).
`ctc_04` is the one fetched part it still holds in the `fillet` column.

The backlog line left the site unread. Read now (a probe over `ftc-06` and
`ctc-01`, both committed): it is a **split rim**. A hole's or boss's rim circle
is two half-circle edges between the *same* plane and the *same* cylinder, and
the vertex where one half meets the other carries nothing else: the cylinder's
seam runs from the rim's *other* vertex. Every printed vertex is
`e: v→w`, `e′: w→v`, both `Circle`, both uses `(plane, cylinder)`. It is not
a seam, a tangent junction or a face across. The kernel's walk meets the
seam-side vertex (three edges, ADR-0035 §1, built) and then this one, which
`corner_of` refuses because it is not a vertex of three edges
(`blend.rs` `three.len() != 3`).

## Constraints it runs into
- ADR-0035 §1 defines the tangent vertex by exactly three edges (amended by
  ADR-0039 for four). §2 keeps one stripe and one blend face per edge, so
  the two halves stay two faces, as Open CASCADE builds them.
- ADR-0039 rejected merging the halves into one edge (it changes the
  operand's topology and breaks the consumer's one-face-per-edge naming).
- ADR-0036 §4: a closed edge's ring has a seam; the open-arc ring is ADR-0007's
  ring without the whole turn.
- `.agents/rules/kernel.md`: provenance for every touched entity, ids
  deterministic, checker green at `Full`; `Reason` gains no variant, so
  `arris-py` is untouched.

## Options
### A — a vertex of two edges that continue one another is a tangent vertex
Amend ADR-0035 §1: a vertex of exactly two edges `e`, `e′` that share both
faces and run on is a continuation, with the same kind and size. The junction
is the ball's cross-section at the vertex (the plane square to the circle),
and, unlike every other junction, there is **no third edge to cut**: each
stripe's two contacts end on that section's two points and the vertex is
replaced by those two. The blend faces meet in one new edge `Generated`;
the two faces' loops each take the split vertex as two. Exact, closed form
(each pcurve a line at constant `u`/`v` as for the seam junction). The same
walk then closes a rim whose vertices all have two edges and a rim split in
more than two. Cost about four steps: the walk and selection with the
census's cause; the junction; a split-rim fixture with Open CASCADE's oracle
(the existing `blend/split-pin-foot-fillet` has the seam on one vertex only),
the property's split rim extended to a second vertex; docs. Also makes
a fillet and chamfer on a plane against a cone/sphere/torus split alike
wherever the table already builds the unsplit pair.
Foregoes nothing; it is ADR-0035's own case with the third edge absent.

### B — heal split rims first (unify same-domain edges)
Merge two edges that lie on one circle and share both faces into one before
blending. It would also help booleans and STEP reading generally, but it is
the healing cycle's remit (C4's histogram has healing second), and ADR-0039
rejected it for this very reason. Larger, more dangerous, frees the same
edges.

### C — leave `VertexBlend` and take another line
The fan (CTC-01's 18 plane × plane edges, no sampled edge, frees no part),
tangent continuation (`TangentChain`, STC-09, one fetched and one committed
part), or the crossing cylinders (`ctc_01`). Each frees one part at most;
none is as large in edges.

### Do nothing
97 edges across both tiers stay refused as `VertexBlend` though Open CASCADE
builds almost all of them, and any consumer part with a hole split into two
arcs and a fillet of its rim meets the refusal. NIST splits rims this way
routinely.

## Recommendation
**A.** It is the largest refusal site by edge count, the cause is now read,
the construction is ADR-0035's with a case removed (no cut), the closed forms
exist, and Open CASCADE's 30/37 and 17/20 give an oracle. B repeats the
rejected alternative. Be honest about the payoff: it frees `ctc_04` from the
`fillet` column only if nothing else refuses behind it, which the plan's first
step measures; the committed column does not move (no committed part's
battery fillet stops here). The fetched count is 4 now, 3 at most after.
What would change my mind: step 1 finding that the 7 refused of 37 asked
(`stc_07`, `ftc-06`'s 2 invalid) are the bulk of the sampled edges, or that
the two-edge vertices also include a genuine face-across corner.

## Decision for the human
1. Take A as the sixth C6 plan (`/plan cylinder-plane-two-edge-corner`)?
   Preferred: yes. This is the agent's call under the charter, so I will treat
   a silent "go" as yes.
2. ADR needed: yes, one, amending ADR-0035 §1 (the vertex of two edges) and
   noting ADR-0039's rejected merge stands.
3. Should the plan keep the census's other 2-edge case (any cause beyond a split
   rim found at step 1) in scope? Preferred: no, a backlog line.
