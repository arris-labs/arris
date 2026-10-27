# Plan: blend-runs-onto

- Started: 2026-10-27
- Milestone: M2 of `blend-residue-real-parts` (cycle C11, docs/ROADMAP.md),
  which the idea calls `blend-run-over`; renamed because C6's plan
  (ADR-0038, retired in `ab44c320`) holds that slug
- Idea: docs/ideas/blend-residue-real-parts.md (its M2 absorbed; decision
  round 1, 2026-10-26, settled: option A, M2 first)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
A fillet whose contact leaves its face through an edge that is not the
corner's own builds the way Open CASCADE builds it: between the two
crossings the ball rolls on that edge. This is a **run-over**. Its two shapes
are the script corpus's two refusals: the pi mount plate's standoff roots
(a concave ring whose contact on the plate's top runs over its front edge)
and the phone stand's lip (a convex blend whose contact on the lip's face
runs into the port's hole). Both scripts run to `ok` at every stage
against their Open CASCADE checkpoints. Each case is a fixture under
`blend/` with its oracle. A run-over the construction does not cover stays
`BlendTooLarge` naming the blended edge and the edge it crosses.

## Non-goals
- A blend that meets a third face while its contacts stay inside their
  faces (BACKLOG, ADR-0007's "third face"): detection is another test; it
  stays a backlog line.
- Run-over on a walked stripe (ADR-0062) or a walked chamfer
  (`blend/split-cylinder-rim-chamfer-past-a-hole` stays refused); to
  BACKLOG if a part or script asks.
- A crossing that falls within the trim of a corner end, or a contact
  that crosses more than one edge per side: refused by name.
- The horn and spindle torus (`BlendTooLarge` at the axis, ADR-0036 §3).
- M3 `three-edge-run-vertex` (CTC-02 AP242, FTC-07 AP242 `VertexBlend`).
- M4 `tolerant-tangency` (FTC-11, CTC-05 AP242; CTC-05's oracle chosen there).
- M5 `overhang-tip-cap` (no real part left; script corpus or a consumer only).

## Design deltas
- **ADR-0071, a blend that runs over an edge rolls on it** (step 2). It
  amends ADR-0007's `BlendTooLarge` bound the way ADR-0038 did. Step 1 read
  Open CASCADE's result for both fixtures, and that reading settles the
  working decision; the earlier guess, a blend cut by the face it runs
  onto, was wrong.
  - The contact on face `F` crosses `E`, a convex edge of `F` that is not
    the corner's own. `E` is an outer edge (pi) or a hole's rim (phone).
    Write `H` for the blend's other face and `G` for `E`'s other face.
  - Between the two crossings each section, in the stripe's own plane
    (square to its axis, a ring's meridian plane), is the circle of
    radius `r` centred on `H`'s offset through the point where `E`
    crosses that plane. *Settled* (step 4, ADR-0071 amended): not the
    ball touching `E`, which Open CASCADE matches at the middle only.
  - The **rolled face** is skinned through those sections, from the
    contact on `H` to `E`'s point, carried past each crossing (ADR-0071,
    amended). Open CASCADE fits it as a rational patch.
  - The closed-form blend ends at the section circle through each
    crossing, which it shares with the rolled face, tangent across it.
    The ring's torus becomes one face open at the rolled face; the open
    edge's cylinder becomes two faces.
  - `E` is split at the crossings. Its stretch between them now bounds
    the rolled face and `G`, and `G` is untouched. `F`'s loop runs its
    contact to each crossing and then along `E`'s outer pieces. Where `E`
    is a hole's rim, the hole's loop joins `F`'s outer loop.
  - Accepted only where `E` is convex, a line or a circle, and crossed
    exactly twice inside its range away from any corner end, and the
    spine is traced between the crossings. Anything else is
    `BlendTooLarge` naming the blended edge and `E`. `Reason` gains no
    variant, so `arris-py` is unchanged.
- **`crates/arris-ops/src/blend/run_over.rs`** (new, `pub(super)`). It
  finds the crossings where `on_side_of_face` says false today and pairs
  them into run-overs, and finds each section between them.
  **`rolled.rs`** (new, step 4) skins the rolled face and re-cuts `F`,
  `E` and the closed-form blend; `build.rs`'s `FaceEdit::split` cuts an
  edge into pieces on a face, a gap where `F`'s two loops join.
  `stripe.rs` and `ring.rs` call it before they refuse. No public type or
  signature changes.
- **Provenance** (DATA-MODEL §Provenance): the blend faces and the
  rolled face are `Generated`; `F`, `G` and `E`'s pieces are `Modified`;
  the crossing vertices are `Generated`. No new kind.
- **Checker**: S5 must decide the rolled face against `G`, `H` and the
  closed-form blend by the walked stripe's arms (ADR-0062). Arris's
  checker leaves Open CASCADE's own patch undecided against the torus and
  the hole's wall. If Arris's rolled face hits the same gap, the step
  that meets it adds the arm and says so. *Settled* (step 4): the
  rolled face's two rows on the stripe's one cylinder are traced a piece
  at a time (ADR-0061, amendment of 2026-10-27).
- **Glossary**: *Run-over*: a blend whose contact leaves its face through
  an edge that is not the corner's own; between the crossings the ball
  rolls on that edge. *Rolled face*: the face it sweeps there. _Avoid_:
  cliff edge, overflow.
- **Counts**: Open CASCADE splits the ring's torus in two (15/24/11) and
  the hole's rolled face at the rim's top (17/27/11). Where Arris keeps a
  face whole, the fixture says
  `counts_differ` with the reason, as in ADR-0038 §5.

## Steps
**[1]** routine; **[2]** careful, a geometric or numeric case within a
given design; **[3]** unproven, robustness or bound established here.

- [x] Step 1 **[2]**: both refusals shrunk to `regression/` fixtures with
  Open CASCADE's oracle and its topology read (`inspect`):
  `standoff-root-fillet-runs-off-the-plate`, `lip-fillet-runs-into-a-hole`.
- [x] Step 2 **[3]**: ADR-0071, decided on step 1's reading and a probe
  of every contact refusal on both fixtures and the four blocking parts'
  36 `BlendTooLarge` edges: none of the parts' is a two-crossing run-over.
- [x] Step 3 **[3]**: the rolled spine. `run_over.rs` finds and pairs
  the crossings and traces offset(`H`) ∩ pipe(`E`) between them. Unit
  tests hold it to closed forms. The crossings are at `x = 5 ± √(2.6² − 2.3²)`
  (pi) and `y = 10 ± √(3.5² − 2²)` (phone). The ball's centre at the
  midpoints is `(5, −0.3, 2 + √0.27)` and `(12 − √1.75, 10, 6)`. The
  refusal still names `E` where pairing or the trace fails.
  *Settled*: a traced run-over is refused naming `E` too until step 4
  builds it; ADR-0071 §3's feet on `H` are checked by the rolled face's
  walk there, which has them.
- [x] Step 4 **[3]**: the convex blend over a hole's rim (phone). The
  rolled face is walked, its feet on `H` checked inside `H`, the
  cylinder is cut in two, and the rim's loop
  joins `F`'s outer loop. The fixture moves to `blend/` with its dump;
  volume, area and inertia match the oracle. CHANGELOG bullet.
  *Settled*: step 3's traced spine is gone (ADR-0071, amended); the
  lip's 16/25/10 against 17/27/11 is `counts_differ`.
- [x] Step 5 **[3]**: the concave ring over an outer edge (pi). The
  torus is opened at the rolled face, and the plate's front edge is
  split. The fixture moves to `blend/`.
  *Settled*: a closed edge's torus only, and a stretch off `F` inside the
  contacts' range (not through the seam); the torus keeps no seam edge,
  the seam lying inside its face. 15/23/10 against 15/24/11 is
  `counts_differ`; volume, area and inertia hold to 1e-8 (1.1e-9 in
  volume, the patch's residue, as the lip's).
- [x] Step 6 **[2]**: the neighbours of these two cases: a convex blend
  over an outer edge, a concave one over a hole's rim, and an `E` that is
  an ellipse. Each becomes a fixture with its oracle, or an
  `expect_error` fixture where Open CASCADE refuses too.
- [x] Step 7 **[2]**: property tests in `blend_prop.rs`: a boss at random
  distance from the edge, at random radius and pose, and a slab with a
  cross hole at random height. Each builds checker-green or is refused by
  name. The rolled face's samples lie on a ball of radius `r` that
  touches `H` and `E`, within the walk's fit.
- [ ] Step 8 **[1]**: re-run the script corpus. The two `outcome.json`
  files go to `ok`, and `tools/py-test.sh tests/test_script_corpus.py`
  compares every checkpoint. Re-run the real parts' `fillet` row and
  print it (it must not rise above 4 of 38).

## Acceptance
- `blend/standoff-root-fillet-runs-off-the-plate`,
  `blend/lip-fillet-runs-into-a-hole` and step 6's fixtures pass in
  `crates/arris/tests/corpus.rs` at `Level::Full`. Volume, area, centroid
  and inertia are within each fixture's tolerance of Open CASCADE's.
- `pi-mount-plate` and `phone-stand` are `ok` at every stage, so the
  script corpus's `fillet` refusals drop from 3 to 1 (hinge leaf).
- The real parts' `fillet` row stays at or below 4 of 38. The property
  tests pass at 1000 cases in the full gate, with no panic.

## Docs to update on completion
- `docs/ROADMAP.md` §C11: the script corpus's refusals (3 → 1 at
  `fillet`), and the run-over item ticked.
- `docs/ARCHITECTURE.md` blend section: the run-over beside the mixed
  corner (ADR-0038).
- `docs/DATA-MODEL.md` §Provenance: a crossed edge's pieces, the rolled face.
- `docs/BACKLOG.md`: the "third face" line keeps only its own case; a
  walked stripe's run-over gets a line if a refusal names it.
- `docs/ideas/blend-residue-real-parts.md`: M2 marked done; the file
  keeps M3–M5.
- `CHANGELOG.md` `Unreleased`: a fillet that runs over its face's edge,
  or into a hole, builds.
- `AGENTS.md` current state: M2 done; next `/plan` from M3–M5's order.

## Open questions
- *Settled* (step 1): a run-over is neither clipped nor left
  overhanging; Open CASCADE rolls the ball on the crossed edge, and so
  does Arris (ADR-0071, step 2).
- ⚠ OPEN: the order of M3–M5 after this plan. The agent sets it at
  retirement from the re-run (idea, *Later*).
