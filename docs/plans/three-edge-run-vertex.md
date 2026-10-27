# Plan: three-edge-run-vertex

- Started: 2026-10-27
- Milestone: M3 of `blend-residue-real-parts` (cycle C11, docs/ROADMAP.md)
- Idea: docs/ideas/blend-residue-real-parts.md (its M3 absorbed; decision
  round 1, 2026-10-26, and M2's retirement set M3 next: settled)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
A chain whose wall turns from a plane into a cylinder tangent along a
ruling that leans out of the ball's normal plane at the vertex is filleted
the way Open CASCADE fillets it. The table stripe and the walked stripe
meet on one ball, off the vertex. A probe on 2026-10-27 put both blocked
parts' sampled refusals at this one site. CTC-02 AP242 meets it at v173
(top plane f43, wall plane f17, oblique cylinder f42, ruling e189).
FTC-07 AP242 meets it at v168 (f47, f276, f46, ruling e200). Backlog line
181 already names this case: the balls agree to 2.1e-4, but the contacts
lie 0.06 off the ruling. Both parts' `fillet` stages agree with Open
CASCADE, so the blend network row falls from 4 of 38 to 2. A junction
the construction does not cover stays `VertexBlend` naming both edges
and the vertex.

## Non-goals
- A tangent edge that is not a line (the annulus into a torus,
  `blend/chain-through-torus-unsupported`; backlog line 181 keeps that
  half), or a shared face that is not a plane: both stay `VertexBlend`.
- The same lean at a vertex of four edges where both faces turn (the
  census's 4 sites in CTC-02): to BACKLOG with its count.
- A walked chamfer's junction at a leaning ruling: it stays `VertexBlend`.
  The battery asks only for fillets.
- A walked stripe at a miter or a corner (`join_blends`), and the slanted
  seam (backlog line 97).
- M4 `tolerant-tangency` (FTC-11, CTC-05 AP242), M5 `overhang-tip-cap`.

## Design deltas
- **ADR-0072, a junction whose tangent edge leans meets on that edge's
  offset** (step 2). It amends ADR-0035 §3 and ADR-0062's junction.
  - Name the faces at the vertex: `P` the shared face (a plane), `W` and
    `C` the runs' far faces, and `w` the vertex's third edge, along which
    `W` and `C` are tangent.
  - When `w` is a line, `W` and `C` share their normal `n` all along it,
    so the ball's centres touching both lie on `w_r = w + r·n` (`n`
    signed by convexity). The shared ball's centre is
    `c* = w_r ∩ offset(P, r)`. It lies on both runs' spines exactly.
  - `q` is `c*`'s foot on `P`, and `p` is `c* − r·n`, a point of `w`.
    The arc is the great circle from `q` to `p`, square to `n_P × n`.
  - Both runs end at `c*`, not at the vertex. The table stripe's `t` is
    `c*`'s parameter on its axis and may lie past its edge's range. The
    walked stripe's end `v` is the spine's parameter at `c*`, which must
    lie within `REACH_RADII`. Otherwise the junction is `VertexBlend`.
  - `w` is cut at `p` (`cut_corner`). The vertex leaves the body.
  - The closed form is tried only where today's `meet` refuses a walked
    junction, so a junction that builds now keeps its result.
- **`crates/arris-ops/src/blend/junction.rs`**: `junction` computes `c*`
  before it refuses. `RunAt` carries the end it was read at, so that
  `Miter::t` holds `c*`'s parameters. `walked_blend.rs` gains the spine's
  parameter at a point. No public type or signature changes, and
  `Reason` gains no variant, so `arris-py` is unchanged.
- **Provenance**: no new kind. The blend faces are `Generated`; `P`, `W`,
  `C` and `w` are `Modified`; the vertex is `Deleted`; `q` and `p` are
  `Generated`, as at today's junction.
- **Glossary**: *Junction*: where two stripes of a chain meet on one
  ball's arc, at a tangent vertex or, where the tangent edge leans, on
  that edge's offset away from it (ADR-0035, ADR-0072).

## Steps
**[1]** routine; **[2]** careful, a geometric or numeric case within a
given design; **[3]** unproven, robustness or bound established here.

- [x] Step 1 **[2]**: write the shrunk cases as `regression/` fixtures,
  each with Open CASCADE's oracle and its topology read (`inspect`).
  `rounded-block-oblique-top-fillet` is a rounded rectangle extruded
  along `z`, its top cut by a box rotated 20° about `x`, with one top
  line filleted. Its chain has leaning rulings at four vertices and
  square ones at four. `rounded-pocket-oblique-floor-fillet` is the same
  as a pocket under a tilted floor (concave). Confirm that
  `regression/nist-ctc-02-ap242-ellipse-chain-fillet` refuses at this
  site.
- [ ] Step 2 **[3]**: write ADR-0072 from step 1's reading of Open
  CASCADE's faces at the junction (one face across it, or two). Add the
  `c*` closed form with unit tests on the convex fixture: `c*`, `q` and
  `p` against the hand formula, `p` on `w` within its tolerance, and
  `c*` at distance `r` from `P`, `W` and `C`.
- [ ] Step 3 **[3]**: re-end both runs at `c*` (the table's `t`, the
  walked spine's `v`). The convex fixture builds checker-green, its
  measures within tolerance of the oracle. It moves to `blend/` with
  its dump. CHANGELOG bullet.
- [ ] Step 4 **[2]**: the concave pocket moves to `blend/`. Add two
  neighbours: two walked runs at a leaning ruling (an S-bend of two
  tangent arcs under an oblique top, the census's cylinder × cylinder),
  and a lean that puts `c*` past the walked spine's reach. Each becomes a
  fixture with its oracle, or a `vertex-blend` `expect_error` fixture.
- [ ] Step 5 **[2]**: `regression/nist-ctc-02-ap242-ellipse-chain-fillet`
  passes (13 edges on f43, six junctions) and moves to `blend/`. Any count
  that differs is read and recorded as `counts_differ`, with its reason.
- [ ] Step 6 **[2]**: property tests in `blend_prop.rs`. Use a random
  tilt (5° to 40°), corner radius, fillet radius and pose, convex and
  concave. Each case builds checker-green or is refused by name. At each
  junction the arc's samples lie at `r` from `c*`, and `c*` lies at `r`
  from `P` and from `w`.
- [ ] Step 7 **[1]**: re-run the fetched tier (`tools/real-parts.sh`) and
  print the `fillet` row; CTC-02 AP242 and FTC-07 AP242 must agree.
  Re-run `--vertex-blend` on both parts, and give each site left over a
  backlog line with its count. Re-run the script corpus (its `fillet`
  refusals stay at 1).

## Acceptance
- Step 1's two fixtures, step 4's and the CTC-02 excerpt pass under
  `blend/` in `crates/arris/tests/corpus.rs` at `Level::Full`, each
  within its tolerance of Open CASCADE's volume, area, centroid and
  inertia.
- The real parts' blend network row is 2 of 38 (C11's Accept: below 4),
  and the script corpus's `fillet` refusals are at most 1.
- The property tests pass at 1000 cases in the full gate, with no panic.

## Docs to update on completion
- `docs/ROADMAP.md` §C11: the corrected count (2 of 38), and the
  `VertexBlend` part of "Corners the closed forms do not cover" split:
  the leaning ruling done, the torus and the seam left.
- `docs/ARCHITECTURE.md` blend section (the walked stripe's junction,
  around line 1109): the ball off the vertex on the ruling's offset.
- `docs/BACKLOG.md` line 181: only the torus half is kept. Add the
  four-edge lean and any site step 7 leaves.
- `docs/ideas/blend-residue-real-parts.md`: M3 done; the file keeps M4
  and M5.
- `GLOSSARY.md`: *Junction*.
- `CHANGELOG.md` `Unreleased`: a fillet along a top edge whose wall
  turns into a slanted cylinder builds.
- `AGENTS.md` current state: M3 done; next `/plan tolerant-tangency`.

## Open questions
- *Settled* (step 1): Open CASCADE does not split a face at the
  junction. The convex block is 18 faces, 40 edges and 24 vertices, one
  blend face per chain edge. Each leaning junction is one circle of
  radius `r` about `c*`, with its axis along `n_P × n`, and the ruling is
  cut at `p`. At (0, 0.5) that is (0.2, 0.5, 1.105179), the closed form's
  `w_r ∩ offset(P, r)`. CTC-02's excerpt refuses at v167, whose ruling
  e183 leans 2.5° from square to e184 on the plane z = 40.
- ⚠ OPEN: whether a table stripe whose `t` lies past its edge's end
  needs `ends.rs` to stretch its contact lines, or only to read `Miter::t`.
  The agent finds out at step 3. If it is more than a step, it gets a
  step of its own.
- ⚠ OPEN: what step 7 leaves of the census's 140 + 36 `VertexBlend`
  sites (a plane × plane or cylinder × cylinder edge of the same runs, or
  a vertex of four). The agent writes the lines; a new milestone is the
  human's only if it changes C11's scope.
