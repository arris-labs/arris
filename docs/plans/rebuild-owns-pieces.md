# Plan: rebuild-owns-pieces

- Started: 2026-11-15
- Milestone: side slot beside C12 (docs/ROADMAP.md §C12); the last small
  module cycle the `map` skill reports
- Idea: none, verbatim from the human (handoff of 2026-11-15): "break the
  module cycle `ops::boolean ⇄ ops::rebuild` … rebuild owns its piece
  vocabulary (VRef, ERef, SubEdge, PieceUse) and takes section vertices as
  data, not &Interferences — see ARCHITECTURE.md §rebuild, ADR-0004"

## Goal
`ops::rebuild` names nothing in `ops::boolean`. The vocabulary a producer
hands `rebuild` its pieces in (`VRef`, `ERef`, `SubEdge`, `PieceUse`) lives
in `rebuild`, and `rebuild::assembly` takes the section vertices and
section edges as plain data the boolean prepares, not `&Interferences`.
The dependency runs boolean → rebuild only, as ARCHITECTURE.md's `rebuild`
paragraph and ADR-0004 already say it should. Behaviour does not change:
every blessed dump, `fixture.json`, `cancel_counts.txt` and
`tolerance_band.txt` is byte-identical to the plan's start; ids, refusal
strings and the public API are unchanged.

## Non-goals
- No public type or path change: `arris_ops::boolean::Interferences`,
  `SectionVertex` and `SectionEdge` stay where they are. Option 2 of the
  handoff (the pave model moved out of `boolean`, re-exported) is not
  taken: it is larger, and it fixes nothing this cycle needs.
- The other four tangles the map observes (geom, ops::blend, debug, py)
  are intra-module and stay.
- `write_provenance`'s signature beyond the moved types' import path, and
  any split of `boolean/result.rs` (1963 lines).
- A defect found while moving code becomes a regression fixture and a
  backlog line; it is never fixed here.

## Design deltas
- **`ops::rebuild` becomes a directory**: `rebuild.rs` → `rebuild/mod.rs`
  (`git mv`, so the rename is tracked), and a new `rebuild/piece.rs`
  holding `VRef`, `ERef`, `SubEdge` and `PieceUse` moved verbatim from
  `boolean/pieces.rs`, all still `pub(crate)`, re-exported as
  `crate::rebuild::{VRef, ERef, SubEdge, PieceUse}`. `boolean/pieces.rs`
  keeps `EdgeOnFace`, `Alias`, `FacePiece`, `SplitFace` and the
  arrangement, importing the four from `rebuild`. Their doc comments stop
  naming `Interferences`: `VRef::Section(k)` is "the `k`-th section vertex
  the producer hands `assembly`", `ERef::Section(k)` likewise for section
  edges.
- **`rebuild::assembly` loses `i: &Interferences`, `curve_ids` and
  `vref_of`** and gains two slices of new `pub(crate)` rebuild types, in
  `rebuild/piece.rs`:
  - `NewVertex { point: Point3, tolerance: f64 }`, indexed by
    `VRef::Section`;
  - `NewSection { curve: CurveId, range: Interval, start: VRef, end: VRef,
    tolerance: f64 }`, indexed by `ERef::Section`, its curve already the
    model's id and its ends already `VRef`s.

  The boolean's `Build::assembly` (`boolean/result.rs`) builds both from
  `self.i`, `self.curve_ids` and `self.vref_of` in index order. The order
  `assembly` emits specs in is unchanged (section vertices ascending, then
  operand vertices, then sub-edges, then sections ascending), so ids are.
  `write_provenance` keeps `vref_of: &[VRef]` — a slice of rebuild's own
  type, no boolean name.
- No ADR: structure under ADR-0004 (rebuild is `arris-ops`'s, not the
  boolean's) and ADR-0013; this restores what they already say.
- No new glossary term: the types are crate-private.

## Steps
Complexity grades; a step's grade is its tier, the model that executes it
(`.agents/tiers.md`), and `**[1, tier 3]**` overrides the tier: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[1]** — `git mv` `rebuild.rs` to `rebuild/mod.rs`; move the
  four types into `rebuild/piece.rs` and re-export them; point
  `boolean/pieces.rs`, `boolean/result.rs` and `rebuild/mod.rs` at the new
  path; reword the two doc comments. `cargo test -p arris-ops` passes
  unchanged; `grep -n 'boolean::pieces' crates/arris-ops/src/rebuild` is
  empty.
- [x] Step 2 **[1]** — Add `NewVertex` and `NewSection`; change
  `rebuild::assembly` to take them; build them in `Build::assembly`.
  `grep -rn 'boolean' crates/arris-ops/src/rebuild --include=*.rs` hits
  only comments; `archmap.py ops` no longer lists the cycle; the boolean,
  blend and offset tests and their corpus areas pass with no fixture diff.
  The commit body records the map's observation count before and after
  (5 → 4).

## Acceptance
- `python3 .agents/skills/map/scripts/archmap.py ops`, then
  `grep -c 'boolean ⇄ rebuild' target/archmap/report.md` prints 0, and the
  report still has 0 findings.
- `grep -rn 'use crate::boolean' crates/arris-ops/src/rebuild` is empty
  (run at planning: today it hits `rebuild.rs:22-23`, both in step 1's and
  step 2's files).
- `tools/full-gate.sh` is green on the plan's last commit.
- `git diff <plan start>.. -- 'tests/fixtures/**'
  'crates/**/cancel_counts.txt' 'crates/**/tolerance_band.txt'` is empty.
- `tools/semver-gate.sh` (CI's semver gate) passes with nothing under
  `### Breaking`: no public path moved.

## Docs to update on completion
- `docs/ARCHITECTURE.md` §Operations, the `rebuild` paragraph (~line 787):
  one sentence that `rebuild` owns the piece vocabulary a producer speaks
  (`VRef`, `ERef`, `SubEdge`, `PieceUse`) and takes a boolean's section
  geometry as new vertices and sections, so it names nothing of the
  boolean's.
- `docs/ARCHITECTURE.md` and `docs/BACKLOG.md`: any path naming
  `boolean/pieces.rs` for the moved types, or `rebuild.rs` as a file
  (BACKLOG line 200 names `boolean/pieces.rs` for an orientation flag that
  stays there; leave it). The map's design-doc path check catches a stale
  `rebuild.rs`.
- `CHANGELOG.md`: nothing, since no consumer can see the change.
- `GLOSSARY.md`: no new term.
- `AGENTS.md` current state: nothing; a side-slot refactor is no milestone.

## Open questions
- None for the human. Decided by the agent at planning: option 1 (the
  vocabulary into `rebuild`, section geometry as data) over option 2 (the
  pave model out of `boolean`), since the design docs already place the
  vocabulary in `rebuild` and option 2 would move public types for no
  gain; and `rebuild/piece.rs` over a neutral `ops::piece` module, since
  `rebuild` is the only consumer of the vocabulary besides the boolean
  that produces it, and over appending to `rebuild.rs`, which would pass
  1000 lines.
