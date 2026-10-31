# Plan: parallel-cylinder-blend

- Started: 2026-11-01
- Milestone: M7 (cycle C11, docs/ROADMAP.md §C11, "the parallel-cylinder row")
- Idea: none; ADR-0036 §5 is the decision, settled 2026-10-02 and built
  here "when a census asks for it". Verbatim from the human: "/plan
  parallel-cylinder-blend"

## Goal
A fillet or a chamfer of a line edge where two cylinders with parallel
axes meet along a ruling builds in closed form (ADR-0036 §5). The fillet
is a cylinder of radius `r` about the line through the ball's centre. The
chamfer is the plane through the two rulings at chord `d`. Every contact
is a ruling with a line pcurve, and nothing is fitted. The ogive bar
(convex) and the figure eight's waist (concave) move from `regression/` to
`blend/` and pass the checker at `Full`. They are held to their closed
forms, because Open CASCADE walks this pair into a B-spline. The fetched
tier's `Unsupported pair: cylinder × cylinder` edges with parallel axes
(29 in 5 parts at C6) are recounted before the work and after it.

## Non-goals
- Two cylinders whose axes cross or are skew: a quartic, still `Unsupported` (ADR-0007).
- A cylinder against an elliptic cylinder, a cone or a NURBS face along a ruling.
- A miter or corner where such a stripe meets another blend: it stays
  `VertexBlend` (ADR-0036 §5, `miter.rs`/`corner.rs` `ruling` gate) until a
  census asks.
- The plane–cylinder ruling chamfer, which reaches the walked fallback
  today: unchanged; a backlog line if step 1's census shows it matters.
- The other C11 in-list line (fitted corner patches): a later plan.

## Design deltas
- **The ball in the section** (`blend/stripe.rs`). `ruling_ball` becomes
  the cross-section construction over two *traces* in the plane square to
  `d`. A trace is a line (a plane, normal `n`) or a circle (a cylinder
  about an axis ∥ `d`, radius `R`, side `σ`). Each trace is offset by
  `s r` toward the ball, and the centre is where the two offsets cross.
  For two circles that is the root on the edge's side of the line through
  the two axes, the one the edge's own point is the limit of as
  `r → 0`. Offsets that do not cross, or a circle offset to no radius,
  give `None` → `BlendTooLarge`. Plane–plane and plane–cylinder keep
  their arithmetic exactly, so no blessed dump moves.
- **`stripe()` dispatch.** A new arm for `(Cylinder, Cylinder)` whose axes
  are parallel to `d` within `tol.angular` and whose edge is a ruling of
  both. The arm sets `Section::Round` and `ruling: true`, and both
  contacts are rulings. The chamfer arm takes the chord of ADR-0036 §2 on
  each circle, the same in the plane–plane case, and builds the plane
  through the two contact rulings with `Section::Flat`. Any other
  cylinder pair stays `unsupported()`.
- **Ends, junctions, seams** need no new code. `section_between` already
  cuts a `Round` or `Flat` stripe by a face across: a plane gives a circle
  or an ellipse, exact, and a cylinder or a cone is traced (ADR-0037). A
  tangent vertex is a chain junction (ADR-0035 §3). Any gap this leaves
  is a step 5 finding.
- **Checker**: no change expected. The blend against each wall is the
  parallel-axes row of the cylinder pairs (`docs/DATA-MODEL.md` §Curves),
  touching along a ruling, inside (ogive) or outside (waist).
- **Provenance / errors / public API**: none change (ADR-0036 §7). No new
  `Reason`, no Python stub, no `Breaking`.
- ADR: an amendment to ADR-0036 §5 recording what was built, the root
  rule's form for two circles, and the ends ADR-0037 gave it. No new ADR
  unless step 2 finds the root choice ambiguous (a pose where both roots
  are on the edge's side).

## Steps
Grades: **[1]** routine; **[2]** careful (a geometric/numeric case in a
given design); **[3]** unproven (robustness or a bound to establish here).

- [x] Step 1 **[2]** — The census before. A test in `arris-debug`
  (`tests/parallel_cylinder_census.rs`, shaped like `horn_census.rs`)
  blends each fetched-tier edge whose faces are two parallel-axis
  cylinders along a ruling, at the battery's radius. It holds today's
  count by part, convex/concave and end kind. A clone without the files
  skips it.
- [x] Step 2 **[2]** — The section over two traces (deltas §1), with unit
  tests. The two fixtures' centres must match the closed form
  `(4 + √((2 − r)² − 1), 0)` and `(0.75, √((1 + r)² − 0.75²))`. A
  property puts random parallel pairs in random poses and checks that the
  centre lies `r` from both walls and that each contact is the foot of
  the centre on its wall. Plane–cylinder results must be bit-identical
  (the existing `blend/` dumps).
- [ ] Step 3 **[2]** — The fillet arm in `stripe()`. `ogive-bar-ruling-fillet`
  and `figure-eight-waist-fillet` move to `blend/` with blessed dumps and
  their `#[ignore]` lifted. Each gets `analytic.measure_differs`: the
  closed-form volume (section area × 1) and area, with the oracle's
  B-spline distance as ADR-0015's evidence.
- [ ] Step 4 **[2]** — The chamfer arm. New fixtures `blend/ogive-bar-ruling-chamfer`
  and `blend/figure-eight-waist-chamfer` with oracle values
  (`tools/oracle/`). Their contacts are at chord `d` (ADR-0036 §2), held
  to the closed form where Open CASCADE's plane is a B-spline.
- [ ] Step 5 **[2]** — Ends and chains. A `blend/` fixture each for an end on
  an oblique plane, an end on a face across that is a cylinder (traced,
  ADR-0037), and a chain through a tangent vertex into a plane–cylinder
  ruling stripe. A fixture where two such stripes meet at a sharp corner
  holds `VertexBlend` by name. Whatever does not build becomes a
  `regression/` fixture and a backlog line.
- [ ] Step 6 **[2]** — Property test in `blend_prop.rs`. Random
  parallel-cylinder unions and ogives in random poses, fillet and
  chamfer, each either `Ok` and checker-green, with its volume change
  equal to the closed form within the fixture tolerance, or refused by a
  named `BlendTooLarge`/`VertexBlend`.
- [ ] Step 7 **[1]** — The census after. Step 1's test holds the new counts
  and the residue by cause. Open CASCADE's verdicts on the stride sample
  (`ask_the_oracle`) go into the roadmap line. `tools/real-parts.sh`'s
  `fillet` row stays 0 of 38.

## Acceptance
- `cargo nextest run -p arris --test corpus -E 'test(/blend_(ogive|figure_eight)/)'`
  passes: four fixtures, checker green at `Full`, measures held to their
  closed forms.
- Step 5's fixtures pass or are `regression/` with a backlog line. Step
  6's property passes at 256 cases in the full gate.
- The census test shows every parallel-cylinder ruling edge of the fetched
  tier built or refused by a named cause other than `Unsupported`. The
  battery's `fillet` row is still 0 of 38.

## Docs to update on completion
- `docs/adr/0036-…md` — amendment to §5 (built, the two-circle root, ends by ADR-0037).
- `docs/ARCHITECTURE.md` §blend table — the parallel-cylinder row, present tense.
- `docs/ROADMAP.md` §C11 — the in-list line ticked with the census counts.
- `docs/BACKLOG.md` — the parallel-cylinder line removed; step 5/7 findings added.
- `CHANGELOG.md` Unreleased — "a fillet or chamfer where two cylinders with
  parallel axes meet along a straight edge now builds".
- `AGENTS.md` current state — M7 done.
- `GLOSSARY.md` — *trace* (a face's curve in a blend's cross-section) if
  the code keeps the word; check that no existing term covers it first.

## Open questions
- ⚠ OPEN: if step 1 counts far fewer than 29 edges (earlier plans may have
  absorbed some), does the plan still run? The agent decides at step 1:
  yes, since the roadmap lists the row and the two fixtures ask for it;
  the count goes into the ROADMAP line.
  *Settled* (step 1): the census counts 6 ruling edges in 3 parts (FTC-08,
  STC-08 convex between planes; STC-10 concave between tori), not 29 in 5:
  C6's count took every cylinder pair the battery met. The plan runs.
- ⚠ OPEN: the plane–cylinder ruling chamfer is walked today, not built in
  closed form. The agent decides at step 4 whether the shared chamfer arm
  covers it for free. If it does, it stays out anyway (dumps would move)
  and gets a backlog line.
