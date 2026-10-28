# Plan: walked-saddle-cell

- Started: 2026-10-28
- Milestone: between M3 and M4 of the blend residue (cycle C11, docs/ROADMAP.md)
- Idea: none; `docs/BACKLOG.md`'s line "the saddle cell of a walked blend
  beside a B-spline wall", which `plans/three-edge-run-vertex` step 5 moved
  here (that plan's *Settled*, step 5's move: closure before breadth puts
  it before M4).

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
Where a walked stripe meets the face across a leaning junction's ruling
or its own table stripe's cylinder, the checker's S5 decides the pair at
`Full`. `regression/s-bend-oblique-top-fillet` and
`regression/nist-ctc-02-ap242-ellipse-chain-fillet` pass under `blend/`,
held to their oracle. `blend_prop.rs`'s
`oblique_chains_meet_on_the_rulings_offset` and `split_prop.rs`'s fillet
of a split side no longer allow undecided S5 rows, and the oblique
property asserts a floor of built cases per shard. Each new rule of the
tracer is an ADR-0061 amendment with the measurement behind it, and a
geom-level test that fails without it.

## Non-goals
- The S-bend's f37/f38, two walked faces sharing the junction's arc
  (`UnresolvedTurning` in `march/walk.rs`): two NURBS faces sharing an
  edge, backlog's `twin_prop`/`loft_prop` line; the corpus already leaves
  a blend's NURBS × NURBS pairs.
- STC-07 AP242's four cylinder × cylinder pairs
  (`regression/nist-stc-07-ap242-fillet-checker-undecided`): no NURBS
  face, so another tracer; that is its own backlog line.
- The plane against a torus in a knife-edge pose (backlog, "S5 leaves a
  plane against a torus undecided"): another row, another line.
- M4 `tolerant-tangency` and M5 `overhang-tip-cap`; the `VertexBlend`
  sites left on CTC-02 AP242 e2.
- Any change to the blend's construction. The faces stay as
  ADR-0072 builds them; only the section tracer and the tests move.

## Design deltas
- **`arris-geom` `torus_walk/singular.rs`** (`singular_cell`, step 2):
  the cell is cut at an open edge of `u` or `v` also where the saddle is
  within its reach of it, not only within `EDGE_TOUCH`, when the edge's
  point is within `tol.linear` of the other surface (both: the excerpt's
  saddle is `1.6e-8` off its edge of `u`, step 2).
- **`arris-geom` `trace_torus.rs`** (`turns_at_a_kink`, step 3): a knot
  line counts as a kink only where the surface's normals either side
  part by more than the tolerance resolves. Otherwise the root there is
  a turning point on the knot line.
- **ADR-0061**: one amendment per rule (dated, as the 2026-10-14 to
  10-28 ones are), each with its measured stretch and a test that fails
  without it.
- **No public type or signature changes.** `SectionFault` keeps its
  variants; the Python stubs are untouched.
- **`arris-debug` `corpus::check_stage`** (step 5): a blend's result is
  held to every analytic × NURBS pair only where the blend made the
  NURBS face. A face the operand brought, read from a file, keeps the
  read operand's rule (`check_leaving_nurbs`), as `real/` holds it.
- **Fixtures**: the two `regression/` fixtures move to `blend/` with
  blessed dumps. A measure that differs from Open CASCADE's by more than
  ADR-0073's 1e-6 rests on a `second_oracle.json` record, imported by
  the tool; counts that differ are `counts_differ` with the reason (the
  excerpt: Open CASCADE 664 faces, the second oracle 662).
- **`GLOSSARY.md`**: *Saddle cell*, a crossing's certified cell where
  the section's two arms meet (ADR-0061), in the commit of step 2.

## Steps
Grades: **[1]** routine, the design says exactly what to write; **[2]**
careful, a geometric or numeric case to get right within a given design;
**[3]** unproven, robustness or bound established here.

- [x] Step 1 **[2]**: the diagnosis. Census the excerpt's undecided
  pairs, the S-bend's 2 and the properties' by fault, neighbour kind and
  place on the walked face's domain. One test per distinct place in
  `crates/arris-geom/tests/intersect_nurbs.rs`, `#[ignore]`d with the
  desired trace; the walked faces come from the corpus's recipes through
  `arris_debug::corpus` (a dev-dependency), not as copied control nets.
  Done: two places, one rule each (*Settled*, step 1).
- [x] Step 2 **[3]**: the saddle cell. A saddle at a corner of a walked
  skin, on one edge and inside its reach of the other, is on the corner
  and its cell cut both ways; the S-bend's f30/f38 and the excerpt's four
  walked × plane pairs decide. The ADR-0061 amendment, the glossary's
  *Saddle cell*, and
  `a_saddle_at_a_walked_skins_corner_within_its_reach_of_both_edges_is_on_the_corner`
  un-ignored. Done: the rule on both directions; the excerpt's pairs a
  second test.
- [x] Step 3 **[3]**: the arc knot. `turns_at_a_kink` takes a sign
  change of `∂f/∂v` across a knot line as a kink only where the surface
  is creased beyond what the tolerance resolves; across a tangent
  continuous knot it is a turning point on the knot line. The oblique
  property's pocket and block pairs decide. Its ADR-0061 amendment;
  `a_section_across_a_walked_skins_arc_knot_is_no_turn_at_a_kink`
  un-ignored. Done: the crease measured against `tol.linear` over the
  patch's `∂P/∂t`; the oblique property at 64 cases leaves no S5 row.
- [ ] Step 4 **[2]**: `regression/s-bend-oblique-top-fillet` passes at
  `Full` within its tolerance of its oracle and moves to `blend/`
  (`fixtures:`, its oracle named); its f37/f38 is the corpus's NURBS ×
  NURBS row, left.
- [ ] Step 5 **[2]**: `regression/nist-ctc-02-ap242-ellipse-chain-fillet`
  passes (13 edges on f43, six junctions) and moves to `blend/`. The
  corpus's `check_stage` holds a blend whose operand has NURBS faces the
  blend did not make to `check_leaving_nurbs` on pairs with one of
  those faces, and to `check_leaving_nurbs_pairs` on every other pair.
  The excerpt's 67 inherited pairs are the read part's own (*Settled*,
  step 1).
- [ ] Step 6 **[1]**: the allowances go. `oblique_holds` uses
  `assert_checked` and counts its builds, asserting a floor per shard
  (backlog line 311); `split_prop`'s `saddle_cell` predicate is deleted
  and its blend case takes `nothing`. Both pass at 256 cases.
- [ ] Step 7 **[1]**: re-run the fetched tier (`tools/real-parts.sh`)
  and the script corpus; the `fillet` row stays 2 of 38 and the scripts'
  refusals at 1, with no checker failure the tracer's rules added. Every
  walked fixture's blessed dump unchanged (a changed one says why).

## Acceptance
- Both fixtures pass under `blend/` in `crates/arris/tests/corpus.rs` at
  `Level::Full`, within tolerance of their oracle's volume, area,
  centroid and inertia.
- Step 1's geom-level tests pass, each failing with its rule taken out.
- `blend_prop.rs`'s oblique property and `split_prop.rs`'s
  `a_side_is_an_operand_of_every_operation` pass at 1000 cases in the
  full gate with no S5 allowance and the build floor met; no panic.
- The real parts' `fillet` row is 2 of 38; the script corpus's `fillet`
  refusals at most 1.

## Docs to update on completion
- `docs/adr/0061-…`: the amendments (written in steps 2 and 3).
- `docs/ARCHITECTURE.md` NURBS-section paragraph (around line 1595): one
  clause: a walked skin's corner saddle and its arc knot are decided at
  `Full`.
- `docs/ROADMAP.md` §C11: the excerpt and the S-bend built and checked;
  STC-07 still waiting.
- `docs/BACKLOG.md`: delete the saddle-cell line and line 311 (the build
  floor); add the S-bend's f37/f38 to the two-NURBS-faces line.
- `GLOSSARY.md`: *Saddle cell* (step 2).
- `CHANGELOG.md` `Unreleased`: a fillet round a pocket or block under a
  slanted face passes the checker at `Full` (it did not before).
- `AGENTS.md` current state: walked-saddle-cell done; next
  `/plan tolerant-tangency` (M4).

## Open questions
- *Settled* (step 1, 2026-10-28): one rule per fault, so the excerpt
  stays in the plan. Census (`trace_section` on each S5 row at the
  pair's tolerance):
  | where | pairs | fault | place |
  |---|---|---|---|
  | CTC-02 excerpt, the read operand's own | 67 of 71 | 48 crowded, 19 turning | read B-spline walls × cylinders, cones, planes and each other; 67 before the fillet, 3 renamed by trimming |
  | CTC-02 excerpt, walked × plane | 4 | crowded | skin corner, on one edge, 4.2e-7 inside the other; value 1e-12, reach 1.1e-6 to 3e-6 |
  | S-bend f30 × f38, walked × far arc's cylinder | 1 | crowded | the same corner saddle: 3.5e-7 inside, value 2e-14, reach 4.2e-7 |
  | S-bend f37 × f38, walked × walked | 1 | turning (march) | two NURBS faces sharing an edge: a non-goal |
  | oblique property, 64 cases | 14 in 7 of 56 built | turning | walked skin × its table cylinder: `turns_at_a_kink` at the arc's double knot, `∂f/∂v` ±5e-9 against floors 1e-11, normals equal to 3e-8 |
  | split property, 64 cases | 0 in 200 blends | — | the allowance is unused |

  The excerpt's 71 are not the 75 step 5 of the last plan counted. The
  plan's guess, a walked stripe tangent to its neighbour along the
  contact, was wrong. The saddle is at the corner ADR-0072's refit ends
  the skin at, against the face across the ruling, and the turn is the
  skin's own arc knot. A pocket at 30° (`R` 0.5, `r` 0.2) reproduces
  the knot; the committed 20° pocket does not.
- ⚠ OPEN: the oracle for each fixture, Open CASCADE or a second oracle's
  record. The agent decides at steps 4 and 5 by ADR-0073's 1e-6 lint.
- ⚠ OPEN: the floor of built cases per shard in step 6. The agent picks
  it from a 1000-case run's count (below half the measured count fails),
  and records the count here.
