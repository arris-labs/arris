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
`.claude/rules`; any other agent reads them here. Skills for the idea →
plan → work → retire → close-cycle → release pipeline, and `inspect` for
seeing geometry, live in `.agents/skills/` (same symlink arrangement).

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

**C1–C5 done (2026-09-12 to 10-02):** the vertical slice, the application
gate, every quadric pair, the STEP reader with its real-part corpus
(healing then blocks 14 of 38 parts), the consumer's API (roles, body
bytes, cancellation, mirror, products). ADR-0001 to 0033. Beside them the
Python binding, PyPI `arris` from the `v*` tag (ADR-0034).
**C6 done (2026-10-05): the blend network.** Chains, the coaxial pairs, traced
ends, steps, tangent vertices, cusps, fans and unequal miters blend; the
fetched tier's `fillet` column is 2 parts of 27, both the NURBS cycle's;
the residue is on the backlog. ADR-0035 to 0046.
**C7 done (2026-10-07): prismatic features.** Offset faces, shell, the
multi-tool boolean, split by a plane and per-face tessellation; sweep and
loft went to the NURBS cycle (ADR-0047 to 0052).
**Next: C8, the query cycle** (opened 2026-10-08): distance, ray fire,
interference, planar section. The DIY part corpus (an `/idea`) picks the
cycle after it.

## Rules that are not derivable from the code

- Lower crates never name upper crates' types: `math` ← `geom` ← `topo` ←
  `check` ← `ops`/`mesh` ← `io` ← `debug` ← `arris` (ADR-0013).
- No panics on geometry; typed errors naming entities. The checker runs
  after every operation in debug builds. Every operation returns
  provenance. Deterministic ids and iteration. Details:
  `.agents/rules/kernel.md`.
- Every failure becomes a fixture with an oracle value; acceptance is a
  corpus run, never a picture. The agent looks at geometry itself —
  `inspect` skill. Never ask the human to describe a shape.
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
  `docs/plans/`) → `/work` (one step, one commit) → `/retire-plan` (docs
  updated, plan deleted); `/close-cycle` at a roadmap boundary, ending in
  `/release`. Not every idea becomes a plan. Details:
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
