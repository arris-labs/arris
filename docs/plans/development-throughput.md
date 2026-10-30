# Plan: development-throughput

- Started: 2026-10-30
- Milestone: none (tooling beside C11, docs/ROADMAP.md)
- Idea: docs/ideas/development-throughput.md (options A, E and F absorbed;
  option B left there for the next plan)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
The agent stops blocking on its own waits, and a plan runs from its first
unchecked step to its retirement without the human between steps. A run
that takes over a minute (a gate, a test run, CI) runs in the background
while the agent goes on with what does not depend on it. `/work` with no
range **conducts** the plan: each step is a fresh subagent whose model is
set by the step's grade, read through a per-harness tier table. After the
last step the conductor runs `/retire-plan` itself. The gate's builds run in
their own memory-limited scope, so memory pressure kills the gate and not
the session. A command prints, per tier, how many steps needed a later
commit to repair them, so the grades can be re-weighted on evidence.

## Non-goals
- Verifying after the commit (the idea's option B): the next milestone
  plan, with its ADR amending ADR-0032, ADR-0066 and `git.md`.
- A cheaper gate (option D: the relink, `opt-level = 3`, `real_nist_ftc_07`):
  ordered by B's first-step measurement; they stay backlog lines.
- Tier tables and a conductor for other harnesses: written when one is
  first used here (idea, *Later*).
- Parallel plans in two worktrees: after B has run a cycle.

## Design deltas
- **Tiers** (`.agents/tiers.md`, new, harness-neutral). A step's grade is
  its tier: **[1]** routine, **[2]** careful, **[3]** unproven (the plan
  template's scale, unchanged). A step overrides its tier only by naming
  one, `**[1, tier 3]**`, never a model. One table per harness. Claude
  Code's: tier 1 Haiku 5.5, tier 2 Sonnet 5.5, tier 3 Opus 5.5, at the
  session's effort. A model update is a one-line edit of that table; plans,
  skills and ADRs name tiers only.
- **The conductor** (`/work`, `/retire-plan`). `/work <slug>` with no range
  runs every remaining step. For each step it starts a fresh subagent on
  the step's tier with the one-step `/work` procedure. On its hand-back the
  conductor checks that the step's commit landed with its box ticked, then
  starts the next step. A subagent that stops for the human (a push, a
  publish, a branch, a change to the plan's acceptance, a product
  question) stops the conductor with its question. After the last box,
  the conductor runs `/retire-plan`. `/work <slug> step N` keeps today's
  single step, in the current session. Claude Code's mechanism (the
  subagent tool's model override) is one paragraph of `tiers.md`. A
  harness without a per-subagent model runs the steps one at a time in
  fresh sessions.
- **Attribution** (`git.md`, `/work`). A commit that repairs an earlier
  step's work, of any type, names that step, `(plans/<slug> step N)`, and
  never a later one. A fix found by the retirement review keeps
  `(plans/<slug> review)`.
- **The rework measure** (`tools/step-rework.sh`, new). For each step
  commit since a date, it reads the step's tier from the plan file as it
  stood in that commit, honouring a `tier N` override. It counts the later
  commits naming the same step, then prints per tier the steps, the steps
  reworked, the share and the rework commits, plus the review fixes per
  plan, which no step claims. `tools/step-rework-test.sh` holds it on a
  scratch repository. `/retro` and `/close-cycle` print it.
- **No foreground waits** (`git.md`, `/work`, `/retire-plan`,
  `/close-cycle`).
  - A gate, a test run expected past a minute, or a CI watch runs with
    `run_in_background`, and a foreground `until`/`while`/`sleep` loop
    waiting on one is never written.
  - `/retire-plan` starts the full gate first and runs the review, the
    *Docs to update* and the drift check while it runs; nothing commits
    until it is green.
  - `/close-cycle` reads the last finished CI run on `main`; a run still
    going is read again at the end, never watched.
- **The gate's scope** (`tools/gate-scope.sh`, new; `.githooks/pre-commit`,
  `tools/full-gate.sh`). Where `systemd-run --user` exists, the gate runs in
  its own transient scope with a memory bound, so the bound or oomd ends
  the gate and not the terminal's scope. A gate ended this way fails,
  naming the bound. Elsewhere the gate runs unchanged. The bound is the
  agent's to set (⚠ OPEN 1).
- `GLOSSARY.md`: **Conductor** (the session that runs a plan's steps as
  subagents and retires it) and **Tier** (a step's grade read as the model
  class that executes it). *Verified* is already in, for the next plan.

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here. **[CI]** after the grade: the step's
only test is a CI run, which needs a push, so the plan runs on
`plan/<slug>` (say so in the header).

- [x] Step 1 **[1]** — `.agents/tiers.md` with the Claude Code table and
  the override syntax; the plan template and `/plan` point at it. Test: the
  docs tests (`docs_refs`) read the new paths.
- [x] Step 2 **[2]** — The conductor in `/work` and the self-run
  `/retire-plan`, as in the deltas. Test: a tier-1 subagent started
  through the table reports its model (`claude-haiku-5-5`); the conductor's
  run of steps 3–7 is the acceptance's (its first hand-back is checked by
  hand before the second starts).
- [ ] Step 3 **[2]** — `tools/step-rework.sh` and its test on a scratch
  repository: a reworked step, an override, a review fix, a step with no
  grade. The attribution rule in `git.md` and `/work`. Baseline: its table
  over `main` since 2026-09-13, in the commit body.
- [ ] Step 4 **[1]** — No foreground waits: `git.md` and `/work`. Test:
  the gate rules still read as `.agents/rules/git.md` cites them
  (`docs_refs`).
- [ ] Step 5 **[1]** — `/retire-plan` overlaps its full gate with the
  review, the docs and the drift check; `/close-cycle` stops watching CI;
  `/retro` and `/close-cycle` print `step-rework.sh`.
- [ ] Step 6 **[2]** — `tools/gate-scope.sh`, wired into the hook and
  `full-gate.sh`. Test: a command over the bound in a scope fails naming
  it, the terminal survives, and the hook passes inside a scope; without
  `systemd-run` (`ARRIS_GATE_SCOPE=off`) it runs as before.
- [ ] Step 7 **[1]** — `GLOSSARY.md`: *Conductor* and *Tier*; `docs/README.md`
  "Document lifecycle" names the conductor.

## Acceptance
- Steps 3–7 ran under one conductor invocation, with no human prompt
  between the first and the last, and each step's commit landed with its box
  ticked. The conductor's session transcript shows each subagent started
  on its tier's model, and no foreground wait loop.
- `tools/step-rework-test.sh` passes. `tools/step-rework.sh --since
  2026-09-13` prints the per-tier table, which goes into the retirement
  commit as the baseline the next plans are compared with.
- The hook and `tools/full-gate.sh` run green inside the gate's scope;
  `tools/gate-test.sh` passes.
- `/retire-plan` for this plan is started by the conductor, and its full
  gate runs while the review subagent reads the commits.

## Docs to update on completion
- `.agents/rules/git.md` — the background rule, the attribution rule, the
  gate's scope beside the lock paragraph.
- `docs/README.md` "Document lifecycle" — `/work` conducts a plan; the
  agent retires it.
- `AGENTS.md` — "Rules that are not derivable": `/work` with no range
  conducts the plan and retires it; tiers in `.agents/tiers.md`.
- `GLOSSARY.md` — *Conductor*, *Tier*.
- `docs/ideas/development-throughput.md` — left holding option B only.
- `docs/BACKLOG.md` — a line for a time breakdown per plan (agent, gate,
  CI, human) in `/retro`'s session digest, from the transcripts.

## Open questions
- ⚠ OPEN 1: the gate scope's bound: `MemoryMax`, `MemoryHigh` or both, as
  a share of RAM or a figure, and whether the swap is bounded. The agent
  decides by step 6, from one full gate's peak (`systemd-cgtop` or the
  scope's `MemoryPeak`), and records it in the step's commit.
- *Settled* (step 2), was OPEN 2: the retirement review subagent runs on
  tier 3 always, not the plan's heaviest step's tier: it reads every
  step's commits, the heaviest included, against the plan. Revisit if the
  rework table shows review fixes are rare.
- *Settled* (step 2): a step that does not land is retried once by a fresh
  subagent one tier up, given the first one's report; a second miss stops
  the conductor. A [CI] step stops it before starting (its push is the
  human's).
