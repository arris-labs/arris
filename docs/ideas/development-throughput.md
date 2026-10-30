# Idea: development-throughput

- Status: Accepted (A, E, F and B); A, E and F absorbed by plans/development-throughput, B is the next plan
- Raised: 2026-10-30
- Prompt (verbatim from the human): "I am concerned about how slow we are
  moving. […] review all sessions and collate statistics about what took
  what portion of time during development: work (agent doing job), running
  tests during work and idle waiting for human to respond" — then, on the
  levers found: "yes, /idea all that"

## Problem

Measured from every session transcript of this repository, 2026-09-13 to
2026-10-30 (439 sessions with time in them, 633 h recorded, 848 commits).
Sessions almost never overlapped (two at once for 1.6 h), so these hours
are close to calendar hours.

| Of 421 h active (working, testing, CI, a wait on the human under 30 min) | Share | Hours |
|---|---|---|
| The gate and other tests | 55% | 231 |
| The agent working (model, edits, builds, probes) | 24% | 99 |
| Waiting on the human | 14% | 60 |
| Watching CI | 7% | 31 |

Outside active time: 129 h in 58 human gaps over 30 min (mostly nights),
and 112 h of dead sessions (an oomd kill or a closed terminal, resumed
hours later; mostly nights too). A commit costs 30 min of active time, 7
of it agent work.

Where the test time goes:

- **The hook blocks every commit**: 784 `git commit` calls, 3.4 min each
  on average, 44 h. The agent can do nothing while the hook runs.
- **The agent's own wait loops**: 44 h of foreground `until`/`flock`/
  `sleep` on the gate lock, a turn blocked for no reason the harness needs:
  a background run notifies when it ends.
- **`/retire-plan` waits on the full gate**: 59 h of tests against 4.8 h
  of agent work under that skill. Its step 1 runs the full gate (about an
  hour; `real_nist_ftc_07` alone is 16 min, a backlog line) and stops on
  red before the review, the docs and the drift check begin, though none
  of them depends on the result.
- **`/close-cycle` watches CI**: 14 h of `gh run watch`, though its step 1
  asks only that the *last* run on `main` was green.
- `/work` is 59% tests; the human answers fast (median 34 s, 625 of 796
  replies inside 5 min), so the human's share is the long gaps, not the
  replies.

Nothing here is the kernel's: it is the loop's. The kernel moves at the
rate of one session's wait on its own gate.

## Constraints it runs into

- `.agents/rules/git.md`: every commit passes `.githooks/pre-commit`,
  never `--no-verify`; `main` always green; one gate at a time (the hook's
  `flock`); branching asked first.
- **ADR-0032** (the lean gate) and **ADR-0066** (16 cases, the slow set):
  the hook's slice, the full gate once per plan. ADR-0032 already accepts
  that a commit can land on `main` with a failure the hook did not look
  for, because `main` is green at every *pushed* tip: the human reads CI
  before a push, `/retire-plan` runs the full gate, a release waits on CI.
- `SEED.md` §1: development is fully agentic; the human sets direction and
  judges results. `/work` stops after one step unless given a range, and
  "the human runs `/retire-plan`": two checkpoints for a human who has
  delegated the technical decisions.
- The hook's lock is `git rev-parse --git-path arris-gate.lock`, which a
  linked worktree resolves to its own git dir. Two worktrees do not queue
  on one lock; they compete for memory, and memory pressure is what oomd
  kills sessions for.
- Backlog lines already on the gate's cost: the 116 s relink of the test
  binaries after a lower-crate change, `opt-level = 3` for tests (1.35×),
  `real_nist_ftc_07` at 16 min, the `arris-debug` split.

## Options

Options A (never wait in the foreground), E (use the nights, through the
conductor) and F (keep oomd off the session) are absorbed by
`plans/development-throughput`; what remains here is the commit hook's
block.

### B — Verify after the commit

The hook keeps what is cheap and static (layers, tolerance literals, fmt,
clippy, the docs tests for prose) and blocks on those only. The test half
(the doctests, the `fast` slice, `cargo doc`) runs after the commit, in
the background, on that commit's own tree checked out in a scratch
worktree with its own `target/`: the agent starts step N+1 at once. A
commit is **verified** when its run passed; the result is kept by commit
id under the git dir.

- The next commit is refused while the previous one's verification is
  red: a red is fixed first, in a `fix(scope): … (plans/<slug> step N)`
  commit, never an amend.
- A `pre-push` hook refuses a tip that is not verified, so every pushed tip
  is as green as today's.
- `/retire-plan` and the full gate are unchanged.

This is ADR-0032's own trade taken one step further: an unpushed local
commit may be red for minutes, found before anything is built on it or
pushed. It hides the whole 3.4 min per commit (44 h) and the relink with
it. Costs: a second `target/` (disk, which has filled before), two
builds at once (memory; the half-cores rule sizes both), and a red now
arrives while the agent is in the next step. About 5 steps and an ADR
amending ADR-0032, ADR-0066 and `git.md`'s first rule.

### C — A worktree branch per step

Each step on its own branch and worktree, fast-forwarded onto `main` when
its gate is green; the agent starts the next step in the next worktree.
Same gain as B, but every step is a branch (`git.md`: branching is asked
first), a rebase per step when steps touch the same files, and a fresh
`target/` per worktree unless they share one. Loses to B on every count
but keeping today's "the commit is the gate's".

### D — A cheaper gate

The backlog's own lines: the relink (the `arris-debug` split), `opt-level
= 3` for tests, `real_nist_ftc_07` moved to the nightly set or made
faster. Each is worth doing on its merits, but none removes the wait: the
agent still blocks, on a shorter gate. Their order should come from
measuring one average hook run (compile, link, test run) with `cargo
--timings` and nextest's own times, not from a guess.

### Do nothing

Calendar speed stays at one session's wait: about a quarter of active time
is the agent working. Every cycle pays the same shares.

## Recommendation

**A, E and F now, as one plan; B next, as its own milestone plan with its
ADR; D's measurement as B's first step; C rejected; parallel plans in two
worktrees parked until B has run a cycle.**

A, E and F are rule and script changes that take nothing from the gate
and recover the waits that need no new mechanism. B is the only option
that removes the commit hook's block, and it keeps every guarantee ADR-0032
relies on (the pushed tip is green); it deserves its own plan because it
changes `git.md`'s first rule. D follows from B's measurement: once the
gate no longer blocks, its length matters only through memory and through
how late a red arrives. Two plans in parallel worktrees would double the
memory load that already kills sessions; that question comes back with B's
numbers.

What would change my mind: the measurement showing the hook's average is
mostly one cheap fix (say the relink, if the `arris-debug` split is near),
which would make D before B; or a red found after the next step had begun
proving costly in practice, which would send B back to A only.

## Decision for the human

Round 1:

❓ **Q1 — Foreground waits**: adopt option A (background gates and CI,
no foreground wait loops, `/retire-plan` overlapping its full gate with the
review and docs, `/close-cycle` not watching CI) as a plan now? (A: yes;
B: no, keep today's skills)
➡️ A: it is rules only and recovers about an hour per plan plus the CI
watching.

❓ **Q2 — The nights**: may `/work` with no range run the whole plan and
the agent run `/retire-plan` itself after the last step? (A: both; B: only
the whole plan, the human still runs `/retire-plan`; C: keep both
checkpoints)
➡️ A: the decisions at those checkpoints are already the agent's
(`SEED.md` §1, the skills' "take the recommendation and go on"); what still
needs the human (push, publish, a product question) still stops.

❓ **Q3 — The commit hook**: verify after the commit (option B), as a
milestone plan of its own after Q1's, with an ADR amending ADR-0032,
ADR-0066 and `git.md`? (A: yes, B; B: a worktree branch per step, option
C; C: keep the blocking hook and only make it cheaper, option D)
➡️ A: the only option that removes the 44 h block, and the pushed tip
stays as green as today.

❓ **Q4 — oomd**: run the hook's and the full gate's builds in their own
memory-limited scope where `systemd-run` exists (option F), in Q1's plan?
(A: yes; B: no)
➡️ A: one step, and most of the 112 h of dead sessions were the terminal
being killed for the build's memory.

An ADR: none for A, E, F (skills and a script); one for B.

*Later*: whether two plans run at once in two worktrees (after Q3's plan
has run a cycle, with its memory numbers); the order of option D's items
(from Q3's first-step measurement).

**The words.** The glossary's *gate* is "the checks a change passes: the
hook's slice … and the full gate"; B splits the hook's slice, so it needs
one more entry, proposed with Q3:

> **Verified**: a commit whose gate tests passed on its own tree after it
> was committed; the next commit and any push wait on it. _Avoid_: checked
> (the checker's word), green (a CI run's).

## Decision

Round 1 (2026-10-30), the human: "yes".

- **Q1** — A: option A becomes a plan.
- **Q2** — A, with the human's condition: the step's weight keeps choosing
  the model, so a plan does not run as one compacting session. Settled as
  option E's conductor, each step a fresh subagent; the weight-to-model
  table is Q5.
- **Q3** — A: verify after the commit (option B), its own milestone plan
  after Q1's, with an ADR amending ADR-0032, ADR-0066 and `git.md`.
  *Verified* is in `GLOSSARY.md`.
- **Q4** — A: option F in Q1's plan.

Round 2:

❓ **Q5 — Weight to model**: which model (and effort) does each step
weight get when the conductor starts it? (A: `[1]` Sonnet, `[2]` Opus at
medium effort, `[3]` Opus at high effort; B: `[1]` Sonnet, `[2]` and `[3]`
Opus at the session's effort; C: a table the human writes)
➡️ A: matches how the weights were used by hand, and effort is the second
lever on tokens beside the model.

❓ **Q6 — Where the table lives**: in the `/work` skill, one table for
every plan, a plan's step may override it with a model named after its
weight (`**[2: opus]**`)? (A: yes; B: the table in each plan's header)
➡️ A: one place to change it; an override is rare and stays visible on
the step.

Round 2 (2026-10-30), the human: "yes, but I wish this system to work not
only with Claude Code, but also with other harnesses (e.g. codex or Pi
with open models). Lets start from claude only, but keep in mind that
different models can be used (even claude updates their models each few
months)".

- **Q5** — A, as **tiers**, not model names: a weight is a tier (`[1]`
  light, `[2]` standard, `[3]` deep), and what each tier runs is one table
  per harness. Claude Code's table is the only one written now, as the
  human set it afterwards ("lets use haiku 5.5 for 1, sonnet 5.5 for 2 and
  opus 5.5 for 3"): `[1]` Haiku 5.5, `[2]` Sonnet 5.5, `[3]` Opus 5.5, each
  at the session's effort. A model update is a one-line edit of that table;
  plans, skills and ADRs name only tiers.
- **Q6** — A, adjusted to the same rule: the table lives once, beside the
  skills in `.agents/` (where every harness reading `AGENTS.md` finds it),
  not in `/work`'s Claude-specific wording; a step's override names a
  tier (`**[1, tier 3]**`), never a model.

Settled by the agent from these answers: the plans' text, the skills and
the ADR speak of tiers and of "a fresh subagent", never of a product; the
conductor's Claude mechanism (the subagent tool's model override) is the
one harness-specific paragraph, and a harness with no subagent model choice
falls back to running the plan step by step in fresh sessions.

*Later*: the other harnesses' tables and their conductor (a driver script
that starts the harness's command line once per step, with the tier's
model, would serve any of them), when one is first used here; parallel
plans in two worktrees, after option B's plan has run a cycle; the order of
option D's items, from B's first-step measurement.

