# Plan: verify-after-commit

- Started: 2026-10-31
- Milestone: none (tooling beside C11, docs/ROADMAP.md)
- Idea: docs/ideas/development-throughput.md (absorbed: option B here, the
  rest by the retired `development-throughput`; its *Later* is in the
  backlog)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal

A code commit returns once its static half has passed (layers, tolerance
literals, fmt, clippy), and its test half (the `fast` slice at 16 cases,
the doctests, `cargo doc` when due) runs after it, in the background, on
the commit's own tree; the agent starts the next step at once. A commit is
**verified** (`GLOSSARY.md`) when that run passed. A red one is met at the
next commit, whose hook then runs the test half inline over everything
since the last verified commit; a push waits for a verified tip. `main`
at every pushed tip is as green as today, and the 3.4 min the hook blocks
a code commit for (44 h over 784 commits, the idea's measure) is gone.

## Non-goals

- Option D's cheaper gate (the relink, `opt-level = 3` for tests,
  `real_nist_ftc_07`): step 1 measures their order, the backlog keeps them.
- Prose-only and version-bump commits: their hook stays as it is (seconds).
- Two plans in two worktrees at once: after this plan has run a cycle.
- `/retire-plan`'s full gate and CI: unchanged.

## Design deltas

- **ADR-0078, verify after the commit** (amends ADR-0032's three tiers,
  ADR-0066's "the hook runs 16 cases", which now run at verification, and
  `git.md`'s first rule). It records step 1's numbers.
- **The state**, in the common git dir (`git rev-parse --git-common-dir`)
  under `arris-verify/`: `<sha>` holding `running`, `pass` or `fail`,
  `<sha>.log`, and the lock `arris-verify.lock`. The verification's tree is
  one linked worktree, `arris-verify/tree`, detached at each sha in turn so
  cargo rebuilds only what changed; its build output is `target/verify/`
  (gitignored with `target/`, swept with it).
- **`tools/gate.sh since <base> [<sha>]`**: the gate for every path, and
  the `doc` key for every Rust line, changed from `<base>` to `<sha>`
  (the index when no `<sha>`). The staged form stays the default.
- **`tools/verify.sh <sha>`**: takes the lock, checks the tree out at
  `<sha>`, runs the test half of `gate.sh since <sha>^ <sha>` through
  `tools/gate-scope.sh` with `CARGO_TARGET_DIR=target/verify`, writes the
  state. `tools/verify.sh --status [<sha>]` prints it, waiting on the lock
  while it runs (default `HEAD`). `ARRIS_VERIFY_CMD` replaces the test half
  in its test.
- **The hooks.** `pre-commit` keeps its prose and version paths. For a
  code commit it runs the static half, then reads `HEAD`'s state (waiting
  on a running one): `pass` defers the test half; `fail` or none runs it
  inline over `gate.sh since <the last verified commit>`, today's gate on a
  wider slice. It leaves `arris-verify/next` = `deferred` or `pass`; the
  new **`post-commit`** records `pass` for `HEAD`, or starts `verify.sh
  HEAD` detached (`setsid`, its output to the log), so a background
  `git commit` returns. A prose or version commit takes its parent's
  state. **`pre-push`** refuses a tip that is not `pass`, waiting on a
  running one. `ARRIS_VERIFY=inline` keeps the blocking hook.
- **One heavy build at a time**: `tools/full-gate.sh` holds the verify
  lock for its run, so a verification never overlaps it.
- **The skills.** `/work`'s *Do*: a commit's hook that runs the test half
  inline is meeting an earlier step's red; a fix of it carries that step's
  suffix (`git.md`, attribution). The conductor, at the last hand-back,
  reads `verify.sh --status` before `/retire-plan`.

## Steps

Complexity grades; a step's grade is its tier, the model that executes it
(`.agents/tiers.md`), and `**[1, tier 3]**` overrides the tier: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[1]** — The hook prints one line of stage times (static,
  nextest, doctest, doc, in seconds). Replay the hook on five recent code
  commits in a scratch worktree and write the split, and D's items in
  their measured order, into the backlog's D lines. Test: the line appears
  on a commit; the numbers are in the commit body.
- [x] Step 2 **[2]** — ADR-0078 from the deltas and step 1's numbers;
  ADR-0032 and ADR-0066 name it as amending them. Test: the docs tests.
- [x] Step 3 **[2]** — `tools/gate.sh since <base> [<sha>]`. Test:
  `tools/gate-test.sh` cases for a range of two commits, a range whose only
  doc-comment change is in its first commit, and the index form.
- [ ] Step 4 **[2]** — `tools/verify.sh` and its `--status`, and
  `tools/full-gate.sh` taking the verify lock. Test: `tools/verify-test.sh`
  on a scratch repository with `ARRIS_VERIFY_CMD` stubs: a pass, a fail, a
  status read while one runs, a second run waiting on the first.
- [ ] Step 5 **[2]** — `pre-commit` split, `post-commit`, `pre-push`, and
  `ARRIS_VERIFY=inline`. Test: `tools/hook-flow-test.sh` on a scratch
  repository with stub halves: a deferred commit returns before its test
  half ends; a red parent makes the next commit run inline over both;
  a prose commit takes its parent's state; a push of an unverified tip is
  refused. The step's own commit is the first to verify after itself.
- [ ] Step 6 **[1]** — Rules and skills: `git.md` (the first rule, the
  waits), `/work` *Do* and *Conduct*, `/retire-plan` (the sweep covers
  `target/verify`), `AGENTS.md`'s setup and rules lines, `CONTRIBUTING.md`'s
  hook sentence, the hook's header. Test: the docs tests.

## Acceptance

- `tools/gate-test.sh`, `tools/verify-test.sh`, `tools/hook-flow-test.sh`
  pass.
- Step 6's commit (and the review's) return after the static half, and
  `tools/verify.sh --status` reads `pass` for each; the hook's time per
  code commit, step 1's line against step 5's onward, is in the retirement
  body.
- The full gate is green (the plan changes the hooks and the gate's tools).

## Docs to update on completion

- `.agents/rules/git.md` — the first rule, "The gate's scope", the waits
  (step 6; checked here).
- `AGENTS.md` — Setup (the hooks now three), the git rule line.
- `CONTRIBUTING.md` — the hook sentence.
- `GLOSSARY.md` — *Verified* stands; *Gate* names the static and test
  halves.

## Open questions

- ⚠ OPEN 1: memory while a verification overlaps the agent's own builds or
  the next hook's clippy: per-scope bounds (each 48G on a 61G machine), or
  the verification's scope lowered. The agent decides at step 4 from step
  1's peak, and records it in ADR-0078.
- ⚠ OPEN 2: a verification that is still running when the next commit
  comes: the hook waits for it (the design), or defers again and leaves
  two queued. The agent decides at step 5; the wait is the default, as it
  keeps one verification at a time.
