---
name: plan
description: Write an executable plan under docs/plans/ from the template — commit-sized checkbox steps, acceptance corpus, design deltas including public-API changes, docs-to-update list — for a feature, refactor, or milestone. Use when the human says "plan", "let's do", "implement", names a milestone ("plan M0"), or accepts an idea's recommendation. Reads the idea file if one exists and absorbs it. Never starts executing.
argument-hint: <slug | milestone | accepted idea slug>
---

# /plan — from decision to todo

A plan is a todo the agent can execute step by step with a commit per step,
and the human can read in two minutes: under ~150 lines and ~10 steps.

1. **Check the limit.** `ls docs/plans/` — two active plans (excluding
   `TEMPLATE.md`) means stop and ask which to retire first.
2. **Gather.** Read `docs/ROADMAP.md` for the milestone's in/out/accept
   lists, the design docs the work touches, the relevant ADRs,
   `.agents/rules/kernel.md`, `GLOSSARY.md`, and `docs/ideas/<slug>.md` if
   the plan comes from an idea.
3. **Size it.** Work that will not fit ~10 steps is split into milestone
   plans, each with an acceptance of its own, riskiest first. Write only
   the first now, and list the others in its *Non-goals* as one line each.
4. **Absorb the idea.** Its decision becomes the plan's *Goal* and *Design
   deltas*, and the header cites `Idea: docs/ideas/<slug>.md (absorbed)`.
   The whole idea absorbed: delete the file in the same change (git keeps
   it). Only part of it: cut the file down to the rest and set `Status:
   Accepted (<option>); <part> absorbed by plans/<slug>`.
5. **Write `docs/plans/<slug>.md`** from `docs/plans/TEMPLATE.md`:
   - *Goal* is one paragraph of what is true when done; *Non-goals* fence the
     scope; overflow past the milestone's "out" list goes to
     `docs/BACKLOG.md`.
   - *Design deltas* name each design doc section, public type and
     signature, and crate boundary that changes. A non-obvious decision
     becomes an ADR — list it as a step. A `⚠ OPEN:` in a design doc that
     this plan closes is listed here with the ADR that closes it. A new
     term is named with its glossary entry.
   - *Steps* are commit-sized: each has an observable result and its own
     test, fixture or oracle comparison, could be reverted alone, and is a
     few lines long (its detail lives in *Design deltas* or its ADR). Order
     them so the riskiest unknown retires first — for kernel work that is
     almost always the geometric case nobody has proven yet, not the API
     around it. A step that adds a geometric case brings its fixture with
     an oracle value. Every step carries a complexity grade the human uses
     to pick the agent; state the scale once, above the list: **[1]**
     routine, the design says exactly what to write and the tests are
     mechanical; **[2]** careful, a geometric or numeric case to get right
     within a given design; **[3]** unproven, an algorithm whose robustness
     or bound has to be established here.
   - *Acceptance* is an executable corpus run, ideally the milestone's own:
     which fixtures, which oracle values, which property tests.
   - *Docs to update* is written now, while the deltas are fresh — it is
     what `/retire-plan` executes; `GLOSSARY.md` is on it when a term is
     new. Rustdoc on new public items is part of the steps.
   - *Open questions* carry `⚠ OPEN:` items, each with who decides and by
     which step.
6. Count the lines (`wc -l`); past ~150, move step detail into the deltas
   or split as in 3.
7. Reply with the step list and the open questions, then **stop**. The
   human edits the plan before `/work` starts it. The plan links the idea's
   decision (and its ADR) as settled; code and branches wait for `/work`.

`$ARGUMENTS`
