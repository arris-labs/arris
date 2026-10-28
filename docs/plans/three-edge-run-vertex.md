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
- **ADR-0073, a fixture's `measure_differs` may rest on a second
  oracle's record** (step 3). It amends ADR-0015 (Open questions, the
  oracle). `tools/oracle/second_oracle.py` imports a `second_oracle.json`;
  the corpus runner, its lint and `compare.py` read it beside `analytic`,
  and the read-back holds Open CASCADE's file to Open CASCADE's reading
  of it (`occt_step.py --measured`). Test-side only: `arris-debug`
  (`publish = false`) gains `Fixture::second_oracle`, `SecondOracle`,
  `SecondMeasured`, `Chain::variant`, `oracle::occt_step_measured` and
  `FileMeasure`; no published type changes.
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
  parameter at a point. `Miter` gains `leaning`, `Stripe` keeps a walked
  fillet's spine, and `walked_cut` refits a skin met there to end at
  `c*` (`Spine::skin_within`). No public type or signature changes, and
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
- [x] Step 2 **[3]**: write ADR-0072 from step 1's reading of Open
  CASCADE's faces at the junction (one face across it, or two). Add the
  `c*` closed form with unit tests on the convex fixture: `c*`, `q` and
  `p` against the hand formula, `p` on `w` within its tolerance, and
  `c*` at distance `r` from `P`, `W` and `C`.
- [x] Step 3 **[3]**: re-end both runs at `c*` (the table's `t`, the
  walked spine's `v`). The convex fixture builds checker-green, its
  measures within tolerance of the oracle. It moves to `blend/` with
  its dump. CHANGELOG bullet. Done in two commits: the runs end at
  `c*`, the walked skin refitted there (ADR-0072, amended), both
  fixtures and the split rims checker-green (ADR-0061, amended); then
  ADR-0073, and the block held to the second oracle within the default
  tolerances (its volume 5.3e-10 off, every inertia component 9e-10).
- [x] Step 4 **[2]**: the concave pocket, and the four split rims
  (steep, tilted, slab, column), each get a second oracle's run and
  record (ADR-0073) and move to `blend/`. Add two
  neighbours: two walked runs at a leaning ruling (an S-bend of two
  tangent arcs under an oblique top, the census's cylinder × cylinder),
  and a lean that puts `c*` past the walked spine's reach. Each becomes a
  fixture with its oracle, or a `vertex-blend` `expect_error` fixture.
  Done: the pocket, the slab and the column run on the second oracle's
  record, the two boxes on Open CASCADE at 1e-7, all moved to `blend/`; the S-bend is an ignored `regression/` fixture (S5); the
  reach case is two backlog lines (see *Settled*).
- [ ] Step 5 **[2]**: `regression/nist-ctc-02-ap242-ellipse-chain-fillet`
  passes (13 edges on f43, six junctions) and moves to `blend/`. Any count
  that differs is read and recorded as `counts_differ`, with its reason
  (Open CASCADE 664 faces, the second oracle 662 for the battery's edge
  set).
- [x] Step 6 **[2]**: property tests in `blend_prop.rs`. Use a random
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
- *Settled* (step 3): a table stripe whose `t` lies past its edge's end
  needs only `Miter::t`. `stripe::contacts` reads each end's `t` off the
  stripe's lines, which are unbounded, so `ends.rs` is unchanged.
- *Settled* (step 3, the checker): neither option the question named.
  The tracer found the shared arc and divided it out. What it could not
  resolve was the arc's foot on the walked face's contact edge. There
  both blends touch the top along contacts that meet tangent, so the
  quotient along that edge is its rounding over a stretch (`4e-14`
  across `8.8e-4` of `u`). A march there shrank its steps without end,
  and a candidate beside the foot was neither crossing nor touch.
  ADR-0061's amendment of 2026-10-28 runs the march aground and takes the
  candidate as the foot. The split property's rounded box then left one
  pair, its arc a fitted row `1.6e-4` of `u` off the exact one; the amendment's
  third rule gives the isoline the turning box beside it. All eight pairs of the block, and the pocket's,
  are decided at `Full`, square junctions included.
- *Settled* (step 3, the reach past `c*`, 2026-10-28): the skin ends at
  `c*`, its corner traced. `split_prop`'s
  `a_side_is_an_operand_of_every_operation` fillets a split rounded box's
  cap rim, every one `VertexBlend` before this step. Built, the walked
  skin's reach past `c*` rolled on along the corner cylinder's extension,
  tangent to the wall along the ruling, and the tracer found two singular
  points crowded at every tilt the property shrank to. A walked stripe
  met at a leaning junction is now refitted along its spine to end at
  `c*` (ADR-0072's amendment), its end row the exact arc. Its corner
  touches the wall at `p` and its end row lies on the table stripe's
  cylinder; ADR-0061's amendment decides both (the corner's touch found
  once, the section beside a touch or the arc within the tolerance
  theirs, an edge root the edge line joins to a touch the touch's).
  Rejected: a reach that leaves the extension has no ball to roll along,
  and resolving the crowded reach decides a surface no face keeps. The
  property passes at 16 cases. Its shrunk poses are
  `rounded-box-tilted-`, `rounded-slab-tilted-` and
  `rounded-column-split-cap-rim-fillet`, checker-green beside the steep
  box, all held to Open CASCADE's volume until ADR-0073 lands.
- *Settled* (step 3, the oracle, 2026-10-28): a second commercial
  kernel, run as a black box, is the evidence ADR-0015 asks for. It
  builds both cases. Its corner blend faces lie on the rolling ball to
  9e-14; Open CASCADE's miss it by up to 0.29 at ×100. Its block volume,
  11.5638251 at scale 1, is within the rounding of Arris's 3.8e-5 below
  Open CASCADE's, and its junction at (0, 0.5) is ADR-0072's circle. So
  the fixtures are held to it, not to closed forms: ADR-0073 (step 3)
  lets `measure_differs` rest on a `second_oracle.json` record, imported
  by a tool, never typed by hand, naming the source's version, its run's
  commit and the scale. It holds volume, area, centroid and inertia
  (per-face areas stay in the run's own record: the runner compares no
  per-face measure for any oracle); Open CASCADE's numbers stay in
  `expected.json` as the record. Arris-authored fixtures only; CTC-05
  AP242 and the fetched tier are M4's to decide on the same record.
- *Settled* (step 3, the read-back, 2026-10-28): Open CASCADE's file of
  the block reads back, in Open CASCADE itself, as 11.5639516, 2.7e-5
  off its in-memory 11.5642604: its corner faces' tolerances reach
  4.7e-4, and its pcurves lie that far from the edges the file keeps.
  Arris's reading of the file is 1.1e-9 from Open CASCADE's reading. So
  under a second oracle's record the read-back holds Arris's reading to
  Open CASCADE's reading of the same file (ADR-0073 §2a), not to its
  shape in memory. Rejected: widening the read-back to Open CASCADE's
  4.7e-4, which would pass nearly any reading.
- *Settled* (step 3, the inertia): the second oracle's mass properties
  carry no moments, so its run asks a lower API for the centroidal
  tensor at its accuracy mode. The relative-tolerance mode measured
  worse, 2.2e-7 off in volume.
- *Settled* (step 4, the records): ADR-0073's lint takes a record as a
  real difference only above 1e-6. The steep and tilted split boxes sit
  1.9e-8 and 5.1e-8 from Open CASCADE (its corner blends miss the ball,
  the second kernel's read agrees with Arris), so they carry no record
  and are held to Open CASCADE at 1e-7 (the tilted box's centroid at
  1e-6), the reason in their descriptions, as the sweep-thread fixtures
  are. The pocket, the slab and the column run on the second oracle's
  record. Its read of the unblended operand is itself 1.2e-9 to 6.6e-9
  off Open CASCADE's on the pocket, the tilted box, the slab and the
  column, exactly the gaps of their results, so the pocket, the slab and
  the column widen `volume_rel` to 1e-8 (the slab and column `area_rel`
  and `inertia_rel` too), each saying why in `measure_differs`.
- *Settled* (step 4, the neighbours): the S-bend builds but S5 leaves two
  pairs undecided (`regression/s-bend-oblique-top-fillet`, ignored); the
  tracer's rows are step 5's too. No tilt of the block's recipe from
  30° to 70° gives a `VertexBlend` at the reach (backlog).
- *Settled* (step 6, "checker-green"): at random poses nearly every
  built case leaves S5 pairs of a walked stripe against a tangent
  cylinder or plane undecided, the tracer's saddle cell (step 5's
  blocker), so `oblique_chains_meet_on_the_rulings_offset` demands no
  violation and allows only undecided S5 rows, naming the rest. It takes
  `OpError::Unsupported` ("no closed form for a cylinder against a
  plane", seen at 39° on a 1.5-wide block) as a refusal by name, with
  `Degenerate`. Rejecting the undecided cases instead would reject
  nearly all of them. When step 5's tracer work lands, the first
  allowance goes and `assert_checked` replaces it.
- *Settled* (step 5, 2026-10-28): not done. The CTC-02 excerpt builds
  but leaves 75 pairs undecided, not 25: 62 `CrowdedSingularity` from
  `torus_walk/singular.rs`'s saddle cell (`singular_cell`, no cell that
  parts the arms found for a walked blend tangent to a B-spline wall
  along its contact), the rest `UnresolvedTurning` in `march/walk.rs`.
  That is an ADR-0061 amendment of its own, as step 3's were, not a
  fixture move; the plan stays open at step 5 for it.
- ⚠ OPEN: what step 7 leaves of the census's 140 + 36 `VertexBlend`
  sites (a plane × plane or cylinder × cylinder edge of the same runs, or
  a vertex of four). The agent writes the lines; a new milestone is the
  human's only if it changes C11's scope.
