# Plan: sweep-skeleton

- Started: 2026-10-25
- Start commit: `2d21f536` (the acceptance's `<start>`)
- Milestone: the refactor side slot beside C11 (cycle C11, docs/ROADMAP.md)
- Idea: docs/ideas/refactor-after-c10.md, its `sweep-skeleton` part absorbed
  (decision 2026-10-23: option B, the rest in the side slot; no ADR)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
`extrude`, `revolve`, `sweep` and `loft` build their solids through one
crate-private `sweep::skeleton`: the spec and role lists, the vertex and
edge rings, the rails, the four-use side cycle, the shells of a closed
sweep and the assemble → finish → verify → record tail are written once.
Each operation keeps only what is its own: extrude's shift, revolve's axis,
on-axis vertices, degenerate edges and chains, sweep's frames and
exact/fitted spans, loft's matching. Measured by `fn_lengths.py`: `extrude`
under 150 lines, the other three under 300 (now 305, 583, 505, 554). Every
blessed `dump.txt`, `fixture.json`, id, role and refusal string is
byte-identical to the plan's start commit.

## Non-goals
- No behaviour change. A defect found while moving code becomes a
  regression fixture and a backlog line, never a fix in the same commit.
- `planar_face`, `sweep/exact.rs`, `loft_exact.rs` and `refuse.rs` stay as
  they are: their inputs differ (idea, §sweep-skeleton).
- No public type or signature changes; no `CHANGELOG.md` entry.
- The other refactor plans: `one-bernstein`, then `long-functions` (side
  slot); the `arris-debug` split (its own idea and ADR).

## Design deltas
- **`crates/arris-ops/src/sweep/skeleton.rs`** (crate-private), beside the
  leaves `mod.rs` already shares (`cap_face`, `side_face`,
  `side_orientation`, `record`):
  - `Specs { vertices, vertex_roles, edges, edge_roles, faces, face_roles }`
    with `vertex(spec, role) -> usize`, `edge(…)`, `face(…)` pushes that
    return the slot, so a spec and its role cannot fall out of step.
  - `ring_vertices(specs, loops, point: Fn(&ProfileEdge) -> Option<Point3>,
    role) -> Vec<Vec<Option<usize>>>` — `None` where revolve's vertex on the
    axis has no copy; the other three map every vertex. `ring_edges(…)` the
    same over a ring's segments, `curve` from a closure, `None` for a
    segment that sweeps nothing (revolve's along the axis).
  - `rails(…)`: one edge per ring vertex between two rings, its curve and
    range from a closure (extrude's line, revolve's circle, sweep's rise,
    circle or skin column, loft's column).
  - `side_cycle(start, up, end, down, reversed) -> Vec<SideUse>`: the
    `Forward, Forward, Reversed, Reversed` cycle and its reversal, written
    four times now; revolve leaves out a rise at a vertex on the axis by
    passing `Option`s.
  - `shells(specs, key: Vec<Option<usize>>, order, role)`: faces bucketed
    by a key per side face (the loop for a closed sweep and loft, the
    chain for a full revolve), stored in `order`, roles re-zipped. A key
    of `None` is `Fault::Invariant`, as revolve's copy returns now.
  - `finish_solid(m, specs, shells, shell_roles, part) -> (Body,
    Provenance)`: `Builder::assemble`, `finish(Solid)`, `verify`, `record`.
- **The `Cavity` drift is settled, not a defect.** `along` names a closed
  sweep's void by `loops[li][0].loop_index`, `loft` by `li`.
  `Profile::edges` returns the loops in the consumer's order (outer, then
  each hole), so the two agree on every valid profile; `shells` takes the
  loop index from position. Step 1 holds it with a test before anything
  moves.
- `docs/ARCHITECTURE.md` §Operations, sweeps: "Both sweeps share the cap,
  side-face and provenance construction" becomes the four sharing one
  skeleton; nothing else in the design docs moves.

## Steps
Complexity grades: **[1]** routine — the design says exactly what to write
and the tests are mechanical; **[2]** careful — a geometric or numeric case
to get right within a given design; **[3]** unproven — an algorithm whose
robustness or bound has to be established here.

- [x] Step 1 **[1]** — Record the start commit in this plan. A test in
  `arris-ops/tests/sweep.rs`: a closed-path sweep and a closed loft of a
  plate with two holes name their voids `Cavity { loop_index: 1 }` and
  `{ 2 }`, matched against `Profile::edges`' loop order.
- [ ] Step 2 **[1]** — `skeleton.rs` with `Specs` and `finish_solid`; all
  four sweeps build through them. Blessed dumps unchanged.
- [ ] Step 3 **[2]** — `shells`: `along` and `loft` by loop, `revolve` by
  chain in its spanning-first order. The step-1 test and every
  `revolve-notch-to-axis`/`revolve-hollow-ring`/`sweep-ring` dump unchanged.
- [ ] Step 4 **[2]** — `ring_vertices` and `ring_edges`, designed against
  revolve first (on-axis vertex, segment along the axis), then extrude,
  sweep and loft. `revolve-onto-axis` and `probe-revolve-onto-axis-m` ids
  unchanged. `GLOSSARY.md` gains *Ring* and *Rise*.
- [ ] Step 5 **[1]** — `rails` in all four.
- [ ] Step 6 **[2]** — `side_cycle` in all four; revolve's annulus `split`
  and its rise left out at the axis pass through unchanged.
- [ ] Step 7 **[1]** — `fn_lengths.py` holds the Goal's bounds; whatever
  of a sweep's own part still runs over moves into a named helper in its
  file. `docs/ARCHITECTURE.md` sentence and `sweep/mod.rs` crate doc updated.

## Acceptance
- `git diff <start> -- tests/fixtures` is empty: every blessed dump,
  `fixture.json` and tolerance band of `sweep/`, `loft/`, `boolean/`,
  `blend/` and `real/` byte-identical.
- `cargo nextest run -p arris-ops` (the `sweep`, `extrude`, `revolve`,
  `loft`, `sweep_prop`, `loft_prop` suites) and `tools/py-test.sh
  tests/test_build.py` green; the full gate (`tools/full-gate.sh`) green
  at `/retire-plan`.
- `fn_lengths.py 150 | grep sweep/` shows no sweep past the Goal's bounds.

## Docs to update on completion
- `docs/ARCHITECTURE.md` §Operations, sweeps — the shared-construction
  sentence (step 7).
- `docs/ideas/refactor-after-c10.md` — `Status` names `sweep-skeleton`
  done and retired.
- `AGENTS.md` current state — the refactor sequence line.
- `GLOSSARY.md` — *Ring* (a profile's vertices and edges carried to one
  station of a sweep) and *Rise* (a vertex's rail in extrude and revolve)
  are used in the architecture but not listed; added in step 4. *Rail* is
  listed.
- `docs/BACKLOG.md` — revolve's *chain* (a run of a loop's segments off the
  axis) collides with the glossary's blend *Chain*; one line to rename it
  in the architecture and code, not done here.

## Open questions
- ⚠ OPEN: whether `rails` and `ring_edges` share one closure shape or
  `rails` takes the sweep's rail law directly — the agent decides at
  step 4, after revolve's rings, by which one keeps revolve's own part
  shorter.
- ⚠ OPEN: if `revolve` cannot reach 300 lines without moving its chain
  analysis, the agent moves `Chain` and the spanning pick into
  `revolve/chains.rs` at step 7 and records the bound reached.
