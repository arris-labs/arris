# Arris docs

`ARCHITECTURE.md`, `DATA-MODEL.md` and `ROADMAP.md` are the living design;
ADRs record decisions and their reasons at the moment they were taken and are
never edited after acceptance
(supersede with a new one). `SEED.md` at the repo root is the charter:
problem, competition, differentiators, chosen stack, the decisions taken at
kickoff.

| Doc | What it holds |
|---|---|
| [architecture](ARCHITECTURE.md) | Crate layout, layer rule, arena and handles, operation signature and error model, the checker, threading, how a consumer's kernel facade maps on |
| [data-model](DATA-MODEL.md) | Geometry enums and parametrisations, topology entities, pcurves, tolerances, the invariant list, provenance, native format |
| [roadmap](ROADMAP.md) | Cycles and milestones with acceptance corpora; the work that stands beside them, and the named cycles not yet opened |
| [adr/](adr/README.md) | Architecture decision records |

Conventions used in these docs: `⚠ OPEN:` marks a question deliberately left
for implementation time; a decision that closes it gets an ADR.

## Document lifecycle

Three tiers, three lifetimes. Which tier a sentence belongs to is decided by
how long it should stay true.

| Tier | Files | Lifetime | Rule |
|---|---|---|---|
| Charter + decisions | `SEED.md`, `adr/` | Append-only | `SEED.md` is frozen at kickoff. A change of mind is a new ADR that supersedes an old one; the old one is never edited. |
| Design | `ARCHITECTURE.md`, `DATA-MODEL.md`, `ROADMAP.md` | Living | Present tense; describes the system as it is *now*. The commit that changes behaviour updates the doc. No "as of M2" prose — git blame is the history. Milestone progress is one status line per milestone in `ROADMAP.md`, nothing more. |
| Ideas | `ideas/<slug>.md` | Until decided | A **brainstorm**, not a todo: problem, options with trade-offs, cost, conflicts, recommendation, the decision for the human. From `ideas/TEMPLATE.md`. Accepted → absorbed by its plan and deleted; rejected → one line under "Rejected" in `BACKLOG.md` with the reason, file deleted; parked → kept with `Status: Parked`. |
| Finding | `BACKLOG.md` §Findings | Until fixed | A defect a nightly, a fuzz target or the differential **measured**, in the fixed shape there: what, where, the run and commit and seed it was seen on, how to replay it, the measured numbers, and its state. Written down the moment it is seen — `Seen` and `Reproduce` are what keep it replayable after the run's artifacts age out. Shrinking it to a `regression/` fixture changes its state, not its home. Reconciled at every `/close-cycle` (step 2). |
| Changelog | `../CHANGELOG.md` | Append-only per release | What a consumer can now do, the refusals they will hit, and each break with its fix. The commit that makes a change a consumer would notice writes its bullet under `Unreleased` (a plan's, at `/retire-plan`); `/release` alone turns `Unreleased` into a version (ADR-0027). |
| Plans | `plans/<slug>.md` | Ephemeral | Created from `plans/TEMPLATE.md` when an idea is picked up; edited together; executed with checkboxes ticked and commits referencing it; on completion the durable parts move to tier 1/2 and **the plan is deleted**. Deletion is the "done" signal; git keeps it. At most two plans active. |

Raw ideas go in `BACKLOG.md`, one line each. A line that needs thinking
becomes an idea; one that is obvious goes straight to a plan; not every idea
becomes a plan. A defect someone *measured* — a red nightly job, a fuzz
crash, a differential disagreement — is not a raw idea and does not go on
that list: it goes under `BACKLOG.md` §Findings, in its own shape, carrying
the seed and the commit that replay it, and `/close-cycle` reconciles them
before a cycle closes. The pipeline is walked by the shared skills `/idea`,
`/plan`, `/work`, `/retire-plan` and, at a cycle boundary, `/close-cycle`
(`.agents/skills/`, symlinked into `.claude/skills/`), under the rules in
`.agents/rules/`. `inspect`, beside them, is how the agent sees a shape it is
changing without a GUI.

`AGENTS.md`'s "Current state" is capped at ~15 lines and speaks at milestone
granularity only; progress narrative goes into the roadmap's status lines
and the plan being executed.

**Drift review** at every milestone boundary: the agent reads each design doc
against the code and lists discrepancies; the milestone is not done until the
list is empty. It is step 3 of `/close-cycle`.

A finished cycle **compresses in place**: `ROADMAP.md` keeps one section per
cycle — goal, status line, in/out/accept — and the next cycle is appended below
it, taking its number at that moment (ADR-0020; until then it is one of the
named cycles at the foot of the file). There is never a second roadmap file;
these docs are topics, not versions.

## What is different from an application's docs

Arris is a library, so three things an application's process leaves implicit
are written down here:

- **Acceptance is a number, never a picture.** A milestone closes on a
  corpus run: checker green, oracle values matched, counts and volumes
  asserted. `inspect` renders pictures for the *agent's* eyes during work;
  no picture is ever a test.
- **The public API is a design doc.** A change to a public type or
  signature is a design delta in the plan, and rustdoc on every public item
  is part of the step, not a follow-up. Pre-1.0, breaking changes are
  allowed but named in the commit body.
- **Every failure becomes a fixture.** A geometry bug is shrunk to a
  minimal case and committed with the desired assertion before it is fixed
  (`.agents/rules/kernel.md`).
