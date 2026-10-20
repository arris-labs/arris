---
name: work
description: Execute the next unchecked step(s) of an active plan, a commit per step, then list what is left.
disable-model-invocation: true
argument-hint: <plan slug> [step number or range]
---

# /work — one step, one commit

## Do

1. Open `docs/plans/<slug>.md`; target the first unchecked step or the given
   range. No slug: one active plan is it; two means ask which, naming each
   one's next step. Re-read its *Design deltas* and any `⚠ OPEN:` naming it — an
   unanswered question that is the human's means stop and ask.
2. Implement the step with its test, fixture or oracle comparison, the docs
   it changes and rustdoc on every new public item.
3. Run only the step's own tests (`cargo nextest run -p <crate> --test
   <file>` or `-E 'test(/^<area>_/)'`) and `cargo fmt --all`. The hook runs
   clippy, doctests and the `fast` slice; `/retire-plan` the full gate — a
   step needing it sooner says why in its commit. For a geometric change,
   look at the result (`inspect` skill) before trusting a green test.
4. Read your own `git diff --cached` against `.agents/review.md` and
   `.agents/rules/kernel.md` before committing: an `unwrap` on geometry, a
   public item nothing calls or tests, a test whose input or assertion got
   weaker while its name and doc comment keep the old claim. A weakened
   test is a plan open question, never a quiet edit.
5. Commit `type(scope): summary (plans/<slug> step N)` with the box ticked
   in the same commit; the body says why, cites docs/ADRs, names any public
   type or signature changed.
6. A step that shows the plan is wrong edits the plan in that commit and
   says so; a rescope inside the plan is the agent's to decide, written as
   *Settled* with its reason, and the work goes on. A failure that resists a first fix gets the `inspect` skill's
   diagnosis loop (a red loop, then ranked hypotheses) before any theory. A geometry failure outside the step's scope becomes an ignored
   `regression/` fixture in the same step (`inspect` skill); its fix is a
   backlog line.
7. Reply: what landed, the commit, anything surprising. Then list every
   remaining unchecked step, one line each — number, `[weight]`, short
   title, what it depends on or may rewrite — mark the next, and end with
   `/work <slug> step N` (or `N-M`). Stop unless a range was requested.

## Keep to

- One step per commit, its box ticked only when its test ran.
- A failing hook is fixed, or the step stops and says why; `--no-verify`
  is never the way past it.
- A case that will not pass with the design as written is an open question
  in the plan, never a widened tolerance, a wildcard `match` arm or a
  fallback path.
- The last ticked box leaves the plan open: the human runs `/retire-plan`.

`$ARGUMENTS`
