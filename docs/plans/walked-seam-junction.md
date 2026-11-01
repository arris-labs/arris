# Plan: walked-seam-junction

- Started: 2026-11-01
- Milestone: M9 (cycle C11, docs/ROADMAP.md §C11, "Corners the closed forms do not cover": a walked chain junction on a seam)
- Idea: none. The roadmap's in-list line and `docs/BACKLOG.md`'s line on the slanted seam (nurbs-face-blend step 9 finding), named as the next plan by `plans/flush-fan-end`'s non-goals

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal

A walked fillet or chamfer whose chain runs through a vertex on a seam
builds, even where the loop crosses the seam at a slant. The case is the ball
and the off-axis pin of `regression/sphere-cylinder-oblique-seam-fillet`. Today
`junction` refuses it as `VertexBlend` (`ends::on_corner_edge`): the
junction's arc lies in the vertex's normal plane, and there the contact on the
seamed face is `r·cot θ` off the seam. When this plan is done, the junction is
the ball **on the spine** whose contact on the seamed face lies on the seam.
The seam is cut there and the vertex is consumed. That matches Open CASCADE's
counts (8 faces, 15 edges, 9 vertices: no extra vertex beside the junction).
The fixture and its chamfer sibling move to `blend/`, meeting their oracle
measures. A property holds the blend to be independent of where the seam lies.

## Non-goals

- A seam vertex of a **table** chain (closed-form runs). Its behaviour is
  unchanged, and it stays refused where it is refused today. If something asks
  for it, it gets a backlog line.
- A closed edge's ring crossing a seam (`ring::cut_seams`). It is untouched.
- A seam vertex that also carries another blended edge, a non-seam extra, or a
  kink (ADR-0075). These stay `VertexBlend` or `TangentChain` by name.
- The cycle line's last corner, `cusp-corner-patch`: a cusp whose two edges are
  both blended, as a fitted corner patch (`blend/cusp-crescent-both-arcs-fillet`),
  with FTC-06's concave fitted cap beside it. It is the next milestone plan.

## Design deltas

- **The site (ADR-0083, step 1).** The site is a tangent vertex `v` of a walked
  chain whose two runs share both faces `F1` and `F2`. Every other edge at `v`
  is a seam of `F1` or of `F2`, used twice by its face. The two runs then trace
  one offsets' section. Their spines (each carried `REACH_RADII` past its
  edge's ends, ADR-0062 §1) hold the same balls to the tolerance.
- **The seam ball (ADR-0083).** For a seamed face `Fi`, the junction is the
  ball `B(t*)` on the spine whose contact on `Fi` lies on the seam. `t*` is
  the root of the contact's signed distance to the seam's isocurve in `Fi`'s
  `(u, v)`. It is bracketed within the reach on either side of `v`, then found
  by sampling for a change of sign and halving the bracket, as `reach_past` is.
  - A square crossing gives back the normal plane's ball, `t* = t(v)`.
  - When both faces are seamed at `v` (the square sibling has the ball's and
    the pin's seams in one plane), the two roots must agree within the
    tolerance. Otherwise the vertex is `VertexBlend`.
- **The construction.** It reuses ADR-0072's off-vertex junction:
  - Both runs end at `B(t*)`. One skin is cut short of `v`. The other is cut
    past `v`, on its spine's carried-on stretch (`skin_within`, `walked_cut`).
  - The arc is `B(t*)`'s, from contact to contact. Each seam is cut at its
    contact (`cut_corner`).
  - `v` is deleted, and so is the chain edge's piece between `v` and `t*`,
    which the extended run's blend takes over.
  - For a chamfer, the arc is the chord at `t*` (ADR-0062 step 7).
- **Refusals.** If no root lies within the reach, or the root lies past the
  other run's far end, the end is `BlendTooLarge`, naming the edge and the
  seam. A seam whose cut leaves no length is `BlendTooLarge`, as `cut_corner`
  already says.
- **Provenance.** `v` and the shortened chain edge's end piece are `Deleted`.
  The arc and its two vertices are `Generated` from both edges. Each seam is
  `Modified`, and so are `F1` and `F2`.
- **Types.** None public. `junction.rs` gains `seam_ball` beside
  `leaning_ball`, and the walked check at `on_corner_edge` routes to it.
  `Reason` gains no variant, so the binding is untouched.
- **Docs.** ADR-0062 step 9 gets one line, "superseded by ADR-0083". Its text
  stays, since ADRs are append-only. `docs/ARCHITECTURE.md` §blend, at the
  walked junction paragraph, gets one sentence. No new term: the *junction*
  entry in `GLOSSARY.md` gains "or on a seam, at the ball whose contact is on
  it (ADR-0083)".

## Steps

Grades are tiers (`.agents/tiers.md`). **[1]** routine; **[2]** careful,
a geometric or numeric case within the design; **[3]** unproven.

- [x] Step 1 **[2]** — Reproduce and decide. Use `inspect` on the regression
  fixture: which of Arris's seam vertices refuse (the ball's two, and the
  pin's if its seam crosses the loop), and at what slant. Add the `regression/`
  fixtures `sphere-cylinder-oblique-seam-chamfer` and
  `sphere-cylinder-pin-seam-slant-fillet` (the pin's seam alone at a slant).
  Each gets its Open CASCADE oracle and is `#[ignore]`d. Write ADR-0083 from
  their counts, which confirm or reverse "the junction is on the seam, `v`
  consumed".
- [x] Step 2 **[2]** — `seam_ball`: the root `t*` on a walked run's spine for
  a seamed face. Unit tests:
  - the square sibling's vertex gives `t(v)`;
  - an analytic sphere and cylinder give the contact on the seam to `1e-10`;
  - a run with no root in reach gives `None`.
- [x] Step 3 **[2]** — Build the slid junction for two walked fillet runs
  with one seamed face: skins cut at `t*`, the seam cut at its contact, `v`
  deleted, provenance as above.
  `regression/sphere-cylinder-oblique-seam-fillet` moves to `blend/` and is
  checker-green at `Full`, at the oracle's counts and measures (ADR-0062's
  walked allowance).
- [x] Step 3a **[3]** — The checker decides abutting pieces of one offset
  surface. *Settled* (2026-11-01): step 3's slide turned a `VertexBlend`
  into two walked skins tangent along the junction arc that S5 left
  undecided (`multi_tool_prop` shard_2); the same body unslid, at a square
  crossing, is undecided too, so the limit was the checker's. The NURBS
  march accounts such boxes on the arc (ADR-0068's 2026-11-01 amendment);
  test `two_skins_adjoining_on_a_slid_seam_junction_are_decided`.
- [x] Step 4 **[2]** — Two seams at one vertex: the roots are held to agree
  within the tolerance. `blend/sphere-cylinder-fillet` stays at its blessed
  dump, and `sphere-cylinder-pin-seam-slant-fillet` moves to `blend/`. A unit
  test of two disagreeing seams gives `VertexBlend`.
- [ ] Step 5 **[2]** — The walked chamfer's junction slides the same way, by
  the chord at `t*`. `sphere-cylinder-oblique-seam-chamfer` moves to
  `blend/`.
- [ ] Step 6 **[1]** — Refusals by name, each a `blend/` fixture holding its
  expected refusal:
  - a slant so shallow that `t*` lies past the reach, or past the other run's
    end: `BlendTooLarge`;
  - a seam vertex carrying a second blended edge: `VertexBlend`.

  `census::vertex_blend_cause` names "a seam junction" for the second.
- [ ] Step 7 **[2]** — Property: the seam does not matter. A ball revolved
  with its profile plane at a random angle `φ` about the axis, fused with a pin
  at a random offset and pose, filleted and chamfered at random `r`. Volume and
  area are held equal, at `1e-7` relative, to the same body with `φ` moved
  away from the loop, and the result is checker-green.
- [ ] Step 8 **[1]** — Census: re-run the real-parts battery (the blend row
  stays 0 of 38, no new refusal) and `tools/script-corpus.sh` (`fillet`
  refusals stay 0). Count the seam-junction `VertexBlend` edges of
  `tests/horn_census.rs`'s parts before and after. Commit the numbers to the
  roadmap's §C11 line.

## Acceptance

- `cargo nextest run -p arris-ops -p arris` passes at the fast profile.
  - The three new `blend/` fixtures meet their oracle counts, and their volume,
    area and centroid within their tolerances.
  - `blend/sphere-cylinder-fillet` is unchanged.
  - Each refusal fixture meets its named reason.
- The step-7 property passes at 256 cases in the full gate.
- The real-parts battery's blend row stays 0 of 38, the script corpus's
  `fillet` refusals stay 0, and there is no panic.

## Docs to update on completion

- `docs/ROADMAP.md` §C11: in "Corners the closed forms do not cover", mark the
  walked seam junction done, with step 8's census. The cusp stays, under its
  plan name.
- `docs/ARCHITECTURE.md` §blend, walked junction paragraph: the seam ball.
- `docs/adr/0062-…md` step 9: a "superseded by ADR-0083" line.
- `GLOSSARY.md` *Junction*: the seam clause.
- `docs/BACKLOG.md`: drop the slanted-seam line (line 104).
- `CHANGELOG.md` Unreleased: "A fillet or chamfer along a curve that crosses a
  seam of a curved face at a slant now builds".
- `AGENTS.md` current state: M9.

## Open questions

- ⚠ OPEN: does Open CASCADE consume `v` and put the junction on the seam? Its
  counts for the oblique fillet (no vertex beyond two per junction) say so. If
  step 1's sibling counts disagree, the alternative is a junction in the normal
  plane plus a new vertex where the contact crosses the seam. That would put
  ADR-0062 step 9's own reading into ADR-0083 instead. The agent decides at
  step 1.
- ⚠ OPEN: is `1e-7` relative the right bound for the property? Arris's walked
  balls hold to `1e-8`, but the two bodies' skins are fitted over different
  spans. The agent sets the bound at step 7 from the measured spread, under
  ADR-0062's `1e-5`.
