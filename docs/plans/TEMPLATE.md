# Plan: <slug>

- Started: YYYY-MM-DD
- Milestone: M? (cycle C?, docs/ROADMAP.md)
- Idea: docs/ideas/<slug>.md (absorbed, or its <part> absorbed), or verbatim from the human: "…"

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
One paragraph: what is true when this plan is done.

## Non-goals
What this plan deliberately does not touch, so scope cannot creep silently.

## Design deltas
Which design docs / types / crate boundaries change, and how. If a decision
here is non-obvious, it becomes an ADR (list it).

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here. **[CI]** after the grade: the step's
only test is a CI run, which needs a push, so the plan runs on
`plan/<slug>` (say so in the header).

- [ ] Step 1 **[1]** — …
- [ ] Step 2 **[1]** — …
Each step is one commit-sized unit with its own test, fixture or oracle comparison.

## Acceptance
The executable check (test, corpus run, oracle comparison) that closes the plan.

## Docs to update on completion
- `docs/0x-….md` §… — …
- `AGENTS.md` current state — …
- `GLOSSARY.md` — a term this plan introduces or sharpens
Written at planning time; executed before the plan is deleted.

## Open questions
`⚠ OPEN:` items, each with who decides and by which step: the agent by
default; the human only for a cycle's scope, positioning, or what publishes
or costs money.
