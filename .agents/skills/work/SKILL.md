---
name: work
description: Execute the next unchecked step(s) of an active plan in docs/plans/ — implement, test, run the pre-commit checks, commit with the plan-step suffix, tick the box. Use when the human says "work on <plan>", "next step", "continue the plan", or "do steps 3-5". Stops at the step boundary or when a step's open question blocks it.
argument-hint: <plan slug> [step number or range]
---

# /work — one step, one commit

## Do

1. Open `docs/plans/<slug>.md`; target the first unchecked step or the given
   range. Re-read its *Design deltas* and any `⚠ OPEN:` naming it — an
   unanswered question that is the human's means stop and ask.
2. Implement the step with its test, fixture or oracle comparison, the docs
   it changes and rustdoc on every new public item.
3. Run only the step's own tests (`cargo nextest run -p <crate> --test
   <file>` or `-E 'test(/^<area>_/)'`) and `cargo fmt --all`. The hook runs
   clippy, doctests and the `fast` slice; `/retire-plan` the full gate — a
   step needing it sooner says why in its commit. For a geometric change,
   look at the result (`inspect` skill) before trusting a green test.
4. Commit `type(scope): summary (plans/<slug> step N)` with the box ticked
   in the same commit; the body says why, cites docs/ADRs, names any public
   type or signature changed.
5. A step that shows the plan is wrong edits the plan in that commit and
   says so. A geometry failure outside the step's scope becomes an ignored
   `regression/` fixture in the same step (`inspect` skill); its fix is a
   backlog line.
6. Reply: what landed, the commit, anything surprising. Then list every
   remaining unchecked step, one line each — number, `[weight]`, short
   title, what it depends on or may rewrite — mark the next, and end with
   `/work <slug> step N` (or `N-M`). Stop unless a range was requested.

## Don't

- Don't fold two steps into one commit, or tick a box whose test didn't run.
- Don't bypass the hook; fix the failure or stop and report.
- Don't widen a tolerance, add a wildcard `match` arm or a fallback path to
  make a case pass — that is a design change for the plan's open questions.
- Don't retire the plan; the human triggers `/retire-plan`.

`$ARGUMENTS`
