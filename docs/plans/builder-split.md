# Plan: builder-split

- Started: 2026-10-24
- Milestone: the refactor sequence before C11's first plan (cycle C11, docs/ROADMAP.md)
- Idea: docs/ideas/refactor-after-c10.md, its `builder-split` part absorbed
  (decision 2026-10-23: option B, Q2 yes — nest `BuildError`, type
  `DegenerateReason`; no ADR)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
`topo/builder.rs` (2977 lines) is a `builder/` module of one concern per
file, every public path under `arris_topo::builder::` unchanged. Edge-use
validation is written once, for `finish` and `assemble` both.
`BuildError`'s 40 variants are four groups plus `NotFound`. `GeomError::Degenerate`
carries a typed `DegenerateReason` instead of free text. No test matches
on a substring of a reason. A multi-shell boolean reads its lump order
without cloning the model. Every blessed dump, fixture, id and refusal
string is byte-identical to `4a3b0df3`. The binding and `CHANGELOG.md`
change once, in the steps that break them.

## Non-goals
- No behaviour change. A defect found while moving code becomes a
  regression fixture and a backlog line, never a fix in the same commit.
- No new Euler operator or assembly feature; C11's blends come after.
- `OpError`/`Reason`, `FitError::Degenerate(String)` and the reader's
  `Refusal` stay as they are (a backlog line if `FitError` wants the same).
- The other refactor plans: `sweep-skeleton`, `one-bernstein`,
  `long-functions` (side slot, in that order), the `arris-debug` split
  (its own idea and ADR).

## Design deltas
- **`crates/arris-topo/src/builder/`** replaces `builder.rs`: `mod.rs`
  (the crate doc, `Builder`'s fields, accessors, `dump`, re-exports),
  `refs.rs` (the `refs!` macro, `Position`, `Use`), `staged.rs`
  (`Staged*`, `Seed`/`Strut`/`Split`/`Join`, `Counts`, `Built`, `Slots`),
  `error.rs`, `euler.rs` (the ten operators, `find_position`,
  `vertex_at`), `finish.rs`, `spec.rs` (`Assembly`, the specs,
  `GeometryRemap`, `of_body`), `assemble.rs` (`assemble`, `assemble_sheet`
  and their checks), `validate.rs`. Moves
  only: items keep their names, docs and order of evaluation. Doc links
  in `docs/DATA-MODEL.md` §Euler operators name `arris_topo::builder::…`
  paths, which hold.
- **`validate.rs`**: one `edge_uses(edges, open) -> Result<(), BuildError>`
  replacing the copy in `finish` and in `assembled_genus`, checking edges
  in the same slot order, so the first error reported is the same one.
- **`BuildError` nested (breaking)**, grouped by concern, as `Reason` is in
  ops; every group `#[error(transparent)]` so each message is unchanged:
  - `Slot(SlotError)`: `NoVertex`, `NoEdge`, `NoFace`, `NoLoop`,
    `BadPosition`, `NoSpec`;
  - `Euler(EulerError)`: `NotEmpty`, `NotOneLoop`, `NotTwoLoops`,
    `Ambiguous`, `NotInLoop`, `NotASeed`, `NotAStrut`, `VertexInUse`,
    `VertexSeeds`, `NotSeparating`, `NotInOneLoop`, `Rings`, `NoRing`,
    `OneFace`, `SurfaceMismatch`, `SameOrientation`, `NoHandle`,
    `DegenerateStrut`, `DegenerateEnds`;
  - `Closure(ClosureError)`: `Empty`, `Kind`, `EmptyLoop`,
    `MissingPcurve`, `EdgeUses`, `SameDirection`, `Duplicate`,
    `LoopOpen`, `Disconnected`, `NotClosed`, `EmptyShell`, `SharedEdge`,
    `SharedVertex` — what `finish` and `assemble` refuse;
  - `NotFound(NotFound)` and `Invariant { face }` stay top-level.
  `From<SlotError | EulerError | ClosureError> for BuildError`, so `?`
  inside the builder is unchanged. Callers that match (`topo` tests,
  `debug::sample`, `ops::primitive`) match the nested path.
- **`primitive::line`/`rise`**: the impossible bad interval stops
  reporting `BuildError::Empty` and returns `OpError::Internal(Fault::
  Invariant { what })` (the existing bookkeeping fault). Unreachable after
  validation, so no observable refusal changes; `line`'s return type
  becomes `OpError`.
- **`DegenerateReason` (breaking)**, in `arris-geom/src/error.rs`: one
  variant per distinct reason written today (~45 sites, ~30 reasons),
  grouped where a family exists (frames, NURBS skin/rows, crossings,
  projection). A reason that formats a nested error carries it typed
  (`Box<GeomError>`, `FrameError`); one that formats numbers carries
  them. `Display` reproduces each current string byte for byte. The
  closures `degenerate = |reason: String| …` become constructors taking
  the variant. `GeomKind` stays beside it.
- **Binding**: both errors reach Python as text — `BuildError` through
  `Fault`/`Rejection`'s `detail`, `GeomDegenerateError.reason` through
  `Display` — so with the messages unchanged, no Python class,
  attribute or `_arris.pyi` line changes. `kernel_error.rs`'s test
  constructors change with the Rust types.
- **`lump_order`** (`ops/boolean/result.rs`): finish the scratch assembly
  inside a `Model::transaction` on the real model that always rolls back,
  instead of finishing into `self.m.clone()`. Needs `&mut Model` where
  `Build` holds `&Model`; if that borrow does not fit, the clone stays and
  the step records why. A single assembly (reordering the staged shells)
  is taken only if its dump is byte-identical; else backlog.
- **`CHANGELOG.md` `### Breaking`**: `BuildError` nested (one-line fix:
  match `BuildError::Euler(EulerError::NotAStrut { .. })`…) and
  `GeomError::Degenerate.reason` typed (fix: match the variant, or
  `to_string()` for the old text).
- **Glossary**: no new term (groups are type names, not words).

## Steps
Complexity: **[1]** routine — the design says exactly what to write and
the tests are mechanical; **[2]** careful — a geometric or numeric case to
get right within a given design; **[3]** unproven — an algorithm whose
robustness or bound has to be established here.

- [ ] Step 1 **[1]** — Record the baseline: every blessed `dump.txt`,
  `fixture.json`, `cancel_counts.txt`, `tolerance_band.txt` hashed at
  `4a3b0df3` into the scratchpad, plus a script that diffs against it;
  each later step runs it. No commit.
- [x] Step 2 **[1]** — Split `builder.rs` into `builder/` as the deltas
  say (eight files; `spec.rs` split off `assemble.rs`, which held 1015
  lines against the ~900 acceptance bound), moves only, `pub use` keeping every path. `topo` tests and doctests
  pass unchanged; baseline identical.
- [x] Step 3 **[2]** — `validate::edge_uses` replaces the two copies
  (edge by edge inside each caller's loop, so the first fault is the same
  one); a unit test in `validate.rs` pins the rule for both callers' list
  types, the existing `tests/assemble.rs` and `tests/builder.rs` cases
  holding each caller.
- [ ] Step 4 **[2]** — Nest `BuildError`; update matches in topo tests,
  `debug::sample`, `ops::primitive` (the `Empty` misuse → `Fault::Invariant`),
  `kernel_error.rs` tests; a test asserting every group's `Display` equals
  the flat message it replaced. `### Breaking` bullet.
- [ ] Step 5 **[2]** — Inventory every `GeomError::Degenerate` reason
  (geom, `ops::query`, `ops::blend::walked`) into `DegenerateReason` with
  byte-identical `Display`; replace the `String` and the closures. A unit
  test per variant pins its message. `### Breaking` bullet.
- [ ] Step 6 **[1]** — Tests that match on a reason substring
  (`geom/tests/nurbs_offset.rs` and the others the grep finds) match the
  variant instead.
- [ ] Step 7 **[2]** — `lump_order` without the model clone, per the
  delta; the multi-shell boolean fixtures and `prop` booleans byte-identical.
- [ ] Step 8 **[1]** — `tools/py-test.sh` green with no stub change;
  `tools/semver-gate.sh` sees both breaks listed.

## Acceptance
`tools/full-gate.sh` green; the step 1 baseline diff empty (every blessed
dump, fixture, cancel count and tolerance band byte-identical, ids and
refusal strings unchanged); `tools/py-test.sh` green with `_arris.pyi`
untouched; `grep -rn 'reason.contains\|reason: String' crates/arris-geom`
empty; no file under `crates/arris-topo/src/builder/` past ~900 lines.

## Docs to update on completion
- `docs/DATA-MODEL.md` §Euler operators — `finish`'s signature line names
  the grouped `BuildError`; one sentence on the four groups.
- `docs/ARCHITECTURE.md` — the `Internal`/`Rejection` table row's
  `BuildError` wording, if it names a variant.
- `docs/ideas/refactor-after-c10.md` — the `builder-split` findings removed,
  `Status:` names this plan done.
- `CHANGELOG.md` — the two `### Breaking` bullets (written in steps 4, 5).
- `AGENTS.md` current state — `builder-split` done; next C11's first item.

## Open questions
- ⚠ OPEN: group boundaries of `BuildError` — `NoSpec` under `Slot` or
  `Closure`, `Empty` under `Closure`. Agent decides at step 4; picked as
  above (a key that names nothing is a slot fault; an empty builder is a
  body that does not close).
- ⚠ OPEN: does `lump_order` get `&mut Model`? Agent decides at step 7;
  the clone stays if the borrow would thread `&mut` through `Build`.
