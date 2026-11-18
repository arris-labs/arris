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
retire → close-cycle → release pipeline, `refactor-review` (structural
debt, measured and read, as one idea), `inspect` for seeing geometry,
`map` for seeing the architecture (charts and checks derived from the code),
and for the human `/next`, `/handoff`, `/retro` and `/wait-what`.

## Setup (once per clone)

```sh
git config core.hooksPath .githooks   # fmt, clippy -D warnings before every commit; the path-scoped fast tests after it (verified before a push, ADR-0078)
cargo install cargo-nextest --locked  # the verification runs the fast profile under nextest
cargo install cargo-sweep --locked    # /retire-plan sweeps stale build output
# Optional, only for the fuzz targets in fuzz/ (outside the workspace):
rustup toolchain install nightly --profile minimal
cargo install cargo-fuzz --locked
```

## Current state

**C1–C10 done and released (2026-09-12 to 10-22)**, with their status
lines in `docs/ROADMAP.md`. **C11, the blend residue, done and released
as 0.10.0 (2026-11-04)**. The blend row blocked 0 of 38 real parts,
the scripts had 0 fillet refusals, and five nightly findings stayed open.
**C12, the training harness's refusals**, opened 2026-11-04 and re-scoped
2026-11-16 (ADR-0020). The cycle's exact batched classification, flat-edge
classification and tangency fixes are done. Per-face surface properties
shipped beside it in 0.11.0, followed by 0.11.1, the first release on PyPI.
Solids that touch along a curve or at an isolated point fuse to one solid
of touching lumps. The torus-link, ball-knob and hinge pairs cut to their
targets and have empty commons (ADR-0088, ADR-0089). Draft, thicken and
`region2` wait for the next named cycle.
**Next:** `/plan touching-lumps-with-shared-topology` (`docs/ROADMAP.md` §C12 **In**).

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
- `/work <slug>` with no step **conducts** the plan: each step a fresh
  subagent on its tier (`.agents/tiers.md` maps tiers to models), then the
  conducting session follows `retire-plan`'s SKILL.md (the Skill tool refuses it). Waits (gate, CI) run in the
  background, never in a foreground loop.
- Trunk-based git, `main` always green (a pushed tip is a verified commit,
  ADR-0078), commit per plan step, never push or publish unasked: `.agents/rules/git.md`. `main` carries the next
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
