# Arris

A B-Rep geometric kernel in pure Rust: analytic and NURBS geometry,
tolerance-carrying topology in an arena, booleans with provenance,
tessellation, STEP. A library: any CAD, CAM or simulation tool can build on it; the first
consumer swaps it in behind a kernel facade once it passes that
application's probe corpus.

Read in this order: `SEED.md` (charter, competition, stack, the decisions
taken at kickoff), `docs/ARCHITECTURE.md`, `docs/DATA-MODEL.md`,
`docs/ROADMAP.md`, `docs/adr/`, then the rules in `.agents/rules/*.md`
(git, docs lifecycle, kernel) — Claude Code loads them automatically via
`.claude/rules`; any other agent reads them here. `GLOSSARY.md` holds the
project's words, one meaning each: use them as written. Skills live in
`.agents/skills/` (same symlink arrangement): the idea → plan → work →
retire → close-cycle → release pipeline, `inspect` for seeing geometry,
and for the human `/next`, `/handoff`, `/retro` and `/wait-what`.

## Setup (once per clone)

```sh
git config core.hooksPath .githooks   # fmt, clippy -D warnings, a path-scoped fast test gate before every commit
cargo install cargo-nextest --locked  # the hook runs a fast profile under nextest
cargo install cargo-sweep --locked    # /retire-plan sweeps stale build output
# Optional, only for the fuzz targets in fuzz/ (outside the workspace):
rustup toolchain install nightly --profile minimal
cargo install cargo-fuzz --locked
```

## Current state

**C1–C9 done (2026-09-12 to 10-12)**, each compressed to its status line in
`docs/ROADMAP.md`: the vertical slice, the application gate, every quadric
pair, the STEP reader and real-part corpus, the consumer's API, the blend
network, prismatic features, the query cycle, the healing cycle (healing
blocks 0 of 38 parts at `read`). ADR-0001 to ADR-0068; PyPI `arris` beside
them (ADR-0034).
**C10, the NURBS cycle** (opened 2026-10-12): NURBS against the analytic
kinds (ADR-0061) and the blend on a NURBS face as a walked stripe
(ADR-0062) landed; sweep along a path landed 2026-10-16 (ADR-0063), and beside it every
transcendental goes through one `libm`, so wasm32 bodies match native
(ADR-0064); loft landed 2026-10-17 (ADR-0065). Beside the cycles: audience
is developers first (ADR-0067); the script corpus landed 2026-10-18 (18 parts,
67 built, 4 refused: blend network 3, SurfaceCollapses 1); NURBS against
NURBS landed 2026-10-19 to 10-20: the marched section (ADR-0068), then the
booleans, S5, B1 and the walked stripe on two NURBS faces; NURBS blocks 0
of 38. Next: `/close-cycle` for C10; draft and thicken beside it.

## Rules that are not derivable from the code

- Lower crates never name upper crates' types: `math` ← `geom` ← `topo` ←
  `check` ← `ops`/`mesh` ← `io` ← `debug` ← `arris` (ADR-0013).
- No panics on geometry; typed errors naming entities. The checker runs
  after every operation in debug builds. Every operation returns
  provenance. Deterministic ids and iteration. Details:
  `.agents/rules/kernel.md`.
- Every failure becomes a fixture with an oracle value; acceptance is a
  corpus run, never a picture. The agent looks at geometry itself with
  the `inspect` skill; the human has no better view of a shape.
- Decisions go in `docs/adr/`; `⚠ OPEN:` in a doc marks a deferred one.
  Kickoff decisions are in `SEED.md` §9 and are not re-litigated.
- Closure before breadth: a cycle keeps its own output operable before the
  kernel takes new input. An unopened cycle carries a name, never a
  number — `/close-cycle` assigns that — and the cycle after the reader's
  is picked from its refusal histogram and the first consumer's
  regressions, not from a list (ADR-0020).
- Update this file's "Current state" when a milestone lands; keep it under
  ~15 lines — the roadmap holds the detail.
- Backlog line → `/idea` (brainstorm, `docs/ideas/`) → `/plan` (todo,
  `docs/plans/`) → `/work` (one step, one commit) → `/retire-plan` (the
  diff reviewed against the plan, docs updated, plan deleted);
  `/close-cycle` at a roadmap boundary, then `/release`. Not every idea becomes a plan. Details:
  `.agents/rules/docs-lifecycle.md`.
- Trunk-based git, `main` always green, commit per plan step, never push
  or publish unasked: `.agents/rules/git.md`. `main` carries the next
  version with `-dev`; a `v*` tag the human pushes is what publishes the
  workspace to crates.io. A change a consumer would notice writes its
  `CHANGELOG.md` bullet under `Unreleased` in the same commit, a break
  under `Breaking` — CI's semver gate holds it to that (ADR-0027).
- Reference trees (truck, monstertruck, Fornjot, Open CASCADE, FreeCAD,
  Rerun, and the application-side projects the requirements come from) are
  read-only and never copied, never `path =` deps. Where they live on this
  machine and what each is good for: `docs/notes/reference-trees.md` —
  gitignored and machine-local, so a fresh clone has to ask the human for
  it. Tracked docs say "the reference trees", never a path.
