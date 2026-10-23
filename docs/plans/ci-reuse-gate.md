# Plan: ci-reuse-gate

- Started: 2026-10-23
- Milestone: beside C11 (side slot, docs/ROADMAP.md §Beside the cycles)
- Idea: verbatim from the human: "we just completed 24 jobs in b46a8e1 (run for many hours), changed few docs and run again the same jobs + 2 publish jobs. That is massive duplication of work and waste of time" — then "plan both: the reuse gate and sharding parallel"
- Runs on `plan/ci-reuse-gate`: steps 5 and 7 are **[CI]** and need a push.

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
A push whose tree differs from the nearest green commit of `main` only in
version strings, a changelog and prose no test reads does not rerun the
suite: `ci.yml` verifies that difference, skips the hours-long jobs, and
runs the cheap ones that the difference can break. 0.9.1 cost three
full runs (the retired plan's tip, the release commit, the tag) of about
22 runner-hours and 5 hours wall each, for two commits that changed no
code. Beside it, the `parallel` job, one 5-hour job that sets every run's
wall time, is sharded like `test`. Any other difference, any doubt (no
green ancestor, a lookup failure), and a pull request run everything as
now.

## Non-goals
- **No change to what the full run tests**, or to the release's own wheel
  build (`release.yml`'s `pypi-build`): the wheel must carry the release
  version, so it is rebuilt and tested on every platform.
- **`release.yml` does not depend on CI.** The reviewer still reads the tag's
  run; it is now short and says which green commit it rests on.
- The nightly workflow and the hook are unchanged.
- Faster individual tests (`sweep_sweep_thread_chamfer` alone takes 22
  minutes locally): a backlog line if the shard times show it dominates.

## Design deltas
- **`tools/ci-reuse.sh <head>`** (new): finds the base, the nearest of the
  first 20 first-parent ancestors of `<head>` that has a successful `ci.yml`
  push run on `main` (`gh api`, `actions: read`), then holds
  `git diff base head` to an allow-list and prints `skip=true|false` and the
  base. Allowed: `CHANGELOG.md`, `AGENTS.md`, the files `ci.yml`'s path
  filter already ignores (`docs/**` bar `ROADMAP.md`, `DATA-MODEL.md` and
  `plans/**`, `.agents/**`, `SEED.md`), `Cargo.lock` changed lines that are
  all `version = "…"`, and `Cargo.toml` equal to the base's once every
  `version = "…"` / `version = "=…"` string is normalised. Everything else,
  `.github/**` and `tools/**` included, is `false`. `--self-test` runs it
  over a scratch repository per case, as `semver-gate.sh` does.
- **`ci.yml`**: a `reuse` job first (seconds, fetch-depth 0), on every event;
  it is `false` for a pull request. `test`, `parallel`, `python`, `oracle`,
  `wasm`, `rerun` and `wheels` take `if: needs.reuse.outputs.skip != 'true'`.
  `fmt`, `clippy`, `layers`, `doc`, `no-default-features`, `semver` stay
  unconditional. A job `reused`, on `skip == 'true'` only, runs
  `cargo package --workspace --exclude arris-py --locked` and the
  `docs_refs` test (the changelog heading format the release reads), and
  writes the base to the step summary. `workflow_dispatch` is added so a
  branch can be run alone (`gh workflow run ci.yml --ref <branch>`).
- **`parallel`** becomes a matrix `partition: [1, 2, 3, 4]` (count settled at
  step 5), each `cargo nextest run -p <crate> --features parallel
  --partition hash:N/4` over the same three crates, `fail-fast: false`.
- **ADR-0069**: a push rests on the nearest green commit when the
  difference is version-only; the allow-list and why it fails closed.
  Listed in `docs/adr/README.md`.

## Steps
Complexity grades: **[1]** routine — the design says exactly what to write
and the tests are mechanical; **[2]** careful; **[3]** unproven.

- [x] Step 1 **[2]** — `tools/ci-reuse.sh` with `--self-test`: a version-only
  bump and a changelog/AGENTS edit give `true`; a source, test, `tools/`,
  `.github/`, `ROADMAP.md` or non-version `Cargo.toml`/`Cargo.lock` change
  gives `false`; no green ancestor and a failed lookup give `false`. The
  green lookup is one function, stubbed by an environment variable in the
  self-test. `<head>` itself counts as a base candidate (a tag on a commit
  already green on `main` has an empty diff).
- [x] Step 2 **[1]** — `ci.yml`: `workflow_dispatch`, the `reuse` job, the
  `if:` on the seven jobs, the header comment. Dispatch first as its own
  commit if GitHub refuses a trigger absent from `main` (see Open).
- [ ] Step 3 **[1]** — the `reused` job: `cargo package`, `docs_refs`, the
  summary line naming the base commit.
- [ ] Step 4 **[1]** — `parallel` sharded 4 ways, same commands.
- [ ] Step 5 **[CI]** **[1]** — push the branch, dispatch `ci.yml`: `reuse`
  prints `false` (the branch changes `tools/` and `.github/`), every job
  runs, the four `parallel` shards are green; record each shard's time
  and set the count so none is more than ~1.5× the median.
- [ ] Step 6 **[1]** — ADR-0069 and its README line; `.agents/rules/git.md`
  (what CI runs on a release); `/release` skill steps 1 and 9 (the tag's
  run is short and names its base; the duplicate-run check goes if the
  `main` pushes no longer run); `docs/ROADMAP.md` lean-gate paragraph.
- [ ] Step 7 **[CI]** **[1]** — after merge, on the green `main` tip: a throwaway
  branch with one version-only commit, dispatched. `reuse` prints `true`,
  the seven jobs are skipped, `reused` and the cheap jobs pass in minutes;
  then a branch with one source-comment commit prints `false`. Both
  branches deleted. Runs in this plan's retirement, not in its commits.

## Acceptance
`tools/ci-reuse.sh --self-test` green (`gate.sh` classifies `tools/` as
"anything else" and the hook runs no tool self-test, as for `check-layers.sh`:
it runs in the `reuse` job, step 2); step 5's dispatch green with four
`parallel` shards; step 7's probe: `true` on the version-only commit,
`false` on the source one. The next release's tag run is under 30 minutes.

## Docs to update on completion
- `docs/adr/0069-….md` and `docs/adr/README.md` — the decision.
- `.agents/rules/git.md` §The full gate — CI on a release rests on the
  green commit it was cut from.
- `.claude/skills/release/SKILL.md` (`.agents/skills/release`) — steps 1, 9.
- `docs/ROADMAP.md` §Beside the cycles — the lean gate, the reuse gate.
- `docs/ARCHITECTURE.md` §Formats and tools — `tools/ci-reuse.sh` if the
  tools are listed there.
- `AGENTS.md` current state — nothing unless the side-slot sentence needs it.
- `GLOSSARY.md` — no new term. `CHANGELOG.md` — nothing (no consumer sees it).

## Open questions
- ⚠ OPEN: does GitHub dispatch `ci.yml` on a branch while `main`'s copy has
  no `workflow_dispatch`? Settled at step 2 (agent): the trigger lands in
  step 2's commit with the rest, and step 5's dispatch is the test; if
  GitHub refuses, that one line lands on `main` first as its own commit.
- ⚠ OPEN: is any `ci.yml` job a required status check in the branch rules?
  A skipped job satisfies one, so nothing changes; the human confirms
  before step 7.
- Decided at planning by the agent: the gate covers pushes to `main` as well
  as tags, since the release commit and the `-dev` commit are version-only
  pushes that each cost a full run; pull requests always run everything.
  The default is the full run, so a mistaken `false` costs hours and a
  mistaken `true` is what the allow-list exists to prevent.
