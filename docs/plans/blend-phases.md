# Plan: blend-phases

- Started: 2026-11-10
- Milestone: side slot beside C12 (docs/ROADMAP.md §C12); the refactor review's third plan, after the rewrite seams
- Idea: docs/ideas/refactor-after-c11.md, its `blend-phases` part absorbed (decision B, 2026-11-04, round 1)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
The blend's assembly reads as its phases. `blend/build.rs` (2479 lines)
is the module `blend/build/`, one file per phase: classify and joins,
make, entities, faces, with `build` itself, the shared types and `record`
in `mod.rs`. The five functions C11 grew — `junction::junction` (376
lines), `stripe::stripe` (279), `build::add_stripes` (252),
`ring::place_contacts` (205), `build::rewrite_faces` (174) — are each a
short driver over named parts, none past 150 lines, and
`add_ring_faces` writes a ring's four-use loop once. Behaviour does not
change: every blessed `dump.txt`, `fixture.json`, `cancel_counts.txt` and
`tolerance_band.txt` is byte-identical to the plan's start, and ids and
refusal strings are unchanged.

## Non-goals
- No public type or signature change: everything here is `pub(super)` or
  private, so the binding, `_arris.pyi` and `CHANGELOG.md` are untouched.
- `census.rs`, `fixtures::lint`, `blend_prop` and the loft section types:
  `census-and-tests`, the idea's last plan.
- The blend's other functions past 150 lines that the review did not
  name, visible since the parser fix (`walked::spine` 296,
  `ends::face_end` 244, `ring::face_end_of` 214, `corner::corner_sphere`
  190, `ends::fan_end` 189, `ring::blend_surface` 177,
  `miter::read_miter` 175, `miter::miter_curve` 154): one backlog line
  for the next review. `rolled_of` and `add_run_over` stay, as the
  review said.
- A defect found while moving code becomes a regression fixture and a
  backlog line, never a fix in the same commit.

## Design deltas
- **`blend/build.rs` → `blend/build/`** (as `boolean/pave/` was split):
  - `mod.rs`: the shared types (`Insertion` … `CornerMade`, `BlendCtx`,
    `Env`, `Classified`, `Joins`, `Draft`, `Drafted` and the aliases),
    `cut_once`, `build`, `record`;
  - `classify.rs`: `classify`, `through_kinks`, `end_meets_nothing`,
    `vertex_joins`, `cusp_of`;
  - `make.rs`: `make_stripes`, `join_blends`, `make_blends`,
    `add_miters`, `add_corners`;
  - `entities.rs`: `add_stripes` … `shorten_edges`;
  - `faces.rs`: `rewrite_faces` … `add_ring_faces`.

  `mod.rs` keeps every item the sibling modules import
  (`super::build::{Env, Made, FaceEdit, SplitPiece}`) at its path, so no
  file outside `build/` changes its `use`. A phase file's items are
  `pub(super)` to `build/` only where `build` or another phase calls them.
- **`junction::junction`** becomes a driver over a private context struct
  holding the two runs, the vertex, the edges and the tolerances, with one
  function per phase the review read: pair the contacts, exits across a
  kink, the lean, the meeting, the slide onto a seam, the trims — as
  `BlendCtx` serves `build`.
- **`stripe::stripe`** reads the edge, its two uses and the dihedral once,
  then calls one function per row of the surface-pair table: two planes, a
  plane and a cylinder along a ruling, two cylinders with parallel axes.
- **`build::add_stripes`** becomes a loop calling end vertices, the
  run-over, contact edges, end arcs and remnants, each its own function
  in `entities.rs`.
- **`ring::place_contacts`** and **`build::rewrite_faces`** are split
  along the phases their own comments mark; the step names them.
- **`add_ring_faces`**: the walked ring and the closed form each produce
  the four `(EdgeKey, forward, &Curve2)` of their loop, and one
  `StoredUse::new` map and one `rw.face` push follow.
- No ADR: structure under ADR-0013 and the blend ADRs it moves. No new
  term.

## Steps
Complexity grades; a step's grade is its tier, the model that executes it
(`.agents/tiers.md`), and `**[1, tier 3]**` overrides the tier: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

Every step: the `blend` area's fixtures and `blend_prop` pass unchanged,
and `git diff <plan start> -- tests/fixtures 'crates/**/cancel_counts.txt'
'crates/**/tolerance_band.txt'` is empty.

- [x] Step 1 **[2]** — `junction::junction` as a driver over its context
  and six phases. The slot chains (`blend/slot-chain-posed-pcurve-end`,
  `blend/nist-ftc-11-ap242-slot-chain-fillet`) and the `*-seam-*` and
  `cusp-*` fixtures are the ones to watch.
- [x] Step 2 **[2]** — `stripe::stripe` read once, one function per
  table row; the `ogive-bar-ruling-*` fixtures are the ones to watch.
- [x] Step 3 **[2]** — `ring::place_contacts` split along its phases;
  the rim and `cusp-*` fixtures are the ones to watch.
- [x] Step 4 **[1]** — `git mv` `build.rs` to `build/mod.rs`; move the
  classify-and-joins phase to `build/classify.rs`.
- [x] Step 5 **[1]** — Move the make phase to `build/make.rs`.
- [x] Step 6 **[1]** — Move the entities phase to `build/entities.rs`.
- [ ] Step 7 **[1]** — Move the faces phase to `build/faces.rs`; `mod.rs`
  holds what the deltas list and no phase's function.
- [ ] Step 8 **[2]** — `add_stripes` as a loop over its five parts; the
  `flush-*` fixtures and `blend_prop`'s run-over section are the ones to
  watch.
- [ ] Step 9 **[2]** — `rewrite_faces` split along its phases;
  `add_ring_faces` writes one loop. `blend/split-torus-ring-fillet` and
  the rim fixtures are the ones to watch.

## Acceptance
- `tools/full-gate.sh` is green on the plan's last commit.
- `git diff <plan start>.. -- tests/fixtures 'crates/**/cancel_counts.txt'
  'crates/**/tolerance_band.txt'` is empty.
- `crates/arris-ops/src/blend/build.rs` does not exist; no file under
  `blend/build/` is past 800 lines.
- `python3 .agents/skills/refactor-review/scripts/fn_lengths.py 400 |
  grep blend/` lists none of `junction`, `stripe`, `add_stripes`,
  `place_contacts`, `rewrite_faces` and no function under `blend/build/`
  past 150 lines.
- `grep -rn 'super::build::' crates/arris-ops/src/blend --include='*.rs'`
  outside `build/` reads as at the plan's start.

## Docs to update on completion
- `docs/ideas/refactor-after-c11.md`: drop this plan's section, already
  cut at planning; the status names `census-and-tests` as the last plan.
- `docs/ARCHITECTURE.md`: no blend section names `build.rs` today;
  one clause where the blend's assembly is described, naming
  `blend/build/`'s four phases, if the retirement review finds the place.
- `docs/BACKLOG.md`: the non-goals' line of the blend functions past 150
  lines the review did not name.
- `CHANGELOG.md`, `GLOSSARY.md`: nothing.

## Open questions
- None for the human. The agent names `junction`'s context struct and
  the phase functions at their steps, and decides at step 7 whether
  `record` (143 lines) earns its own `record.rs`; each choice goes in the
  commit body.
