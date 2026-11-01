---
name: work
description: Execute an active plan, a commit per step — given steps in this session, or with none the whole plan via subagents, then its retirement.
disable-model-invocation: true
argument-hint: <plan slug> [step number or range]
---

# /work — one step, one commit

`/work <slug> step N` (or `N-M`) works those steps in this session, by
*Do*. `/work` with no step, slug or not (no slug: the one active plan, or
ask which if two), **conducts** the whole rest of the plan by *Conduct*,
then retires it: each step a fresh subagent that follows *Do* for that one
step. The conductor's turn does not end between steps: a checked hand-back
is followed at once by the next step's subagent. It stops early only for
what *Conduct* names (a **[CI]** step, a human-only question, a second
miss); reporting after one step and waiting for another `/work` is a bug.

## Do

1. Open `docs/plans/<slug>.md`; target the first unchecked step or the given
   range. No slug: one active plan is it; two means ask which, naming each
   one's next step. Re-read its *Design deltas* and any `⚠ OPEN:` naming it. A
   **[CI]** step is worked on `plan/<slug>`, and the push its test needs is
   asked for before writing it, not after.
2. Implement the step with its test, fixture or oracle comparison, the docs
   it changes and rustdoc on every new public item.
3. Run only the step's own tests (`cargo nextest run -p <crate> --test
   <file>` or `-E 'test(/^<area>_/)'`) and `cargo fmt --all`. The hook runs
   fmt and clippy at the commit; the doctests and the `fast` slice run after
   it, verified on the commit's own tree (ADR-0078, `tools/verify.sh
   --status`). `/retire-plan` the full gate — a step needing it sooner says
   why in its commit. A step that makes a
   property's cases dearer names its slowest shard's time at 16 cases in
   the commit body; the full gate pays sixteen times that, and `fast`
   stops a test at 8 min. A run expected past a minute goes with
   `run_in_background`, and the step goes on with what does not depend on
   it (`.agents/rules/git.md`, no foreground waits). For a geometric change,
   look at the result (`inspect` skill) before trusting a green test.
4. Read your own `git diff --cached` against `.agents/review.md` and
   `.agents/rules/kernel.md` before committing: an `unwrap` on geometry, a
   public item nothing calls or tests, a test whose input or assertion got
   weaker while its name and doc comment keep the old claim. A weakened
   test is a plan open question, never a quiet edit.
5. Commit `type(scope): summary (plans/<slug> step N)` with the box ticked
   in the same commit; the body says why, cites docs/ADRs, names any public
   type or signature changed.
   A commit that repairs an earlier step's work names that step, never the
   step in hand (`.agents/rules/git.md`, attribution). A commit whose hook
   runs the test half inline is meeting an earlier commit's red (the
   previous verification failed); the fix of that red carries the earlier
   step's suffix, and the step in hand commits after it.
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
- No foreground wait: no `until`, `while` or `sleep` loop on a gate, a test
  run or CI. The completion notification is the wait.
- A failing hook is fixed, or the step stops and says why; `--no-verify`
  is never the way past it.
- A case that will not pass with the design as written is an open question
  in the plan, never a widened tolerance, a wildcard `match` arm or a
  fallback path.
- An open question, a fork between designs or a rescope is the agent's,
  whoever the plan names: take the recommended option, write it as
  *Settled* with its reason, and go on through the requested range. Only a
  push, publish, branch, a change to the plan's acceptance or a product
  question (what the kernel offers its callers) waits for the human.
- The last ticked box of a step or range leaves the plan open for
  `/retire-plan`.

## Conduct

1. Read the plan; for each unchecked step note its tier: its grade, or
   the tier it names (`**[1, tier 3]**`). A **[CI]** step stops the
   conductor before it starts: its push is the human's.
2. For each step in order, start one fresh subagent on the step's tier
   (`.agents/tiers.md` says how in this harness), and wait for its
   hand-back: the harness delivers it, never a poll of its output. Its prompt: "Work step N of `docs/plans/<slug>.md` in this
   checkout by *Do* in `.agents/skills/work/SKILL.md`, that one step only,
   and commit it. A run whose result you need goes in the foreground
   under `timeout`, never backgrounded and polled; the commit's test half
   is not yours to wait for. You answer to the conductor, never the human:
   hand back the commit id and anything surprising, or the question that
   stopped you." The conductor writes no code itself; it alone talks to
   the human.
3. On each hand-back, check what landed: the last commit names `(plans/<slug>
   step N)`, the plan has step N ticked, `git status` is clean. Its test
   half may still be running: that is not a failure, and the next hook or
   the push waits on it.
   - A question only the human may answer (*Keep to*: a push, publish,
     branch, change to the plan's acceptance or product question): stop and
     relay it.
   - Not landed for any other reason: one fresh subagent one tier up (3
     stays 3) gets the step and the first one's report; a second miss
     stops the conductor with both reports.
4. After the last box, read `tools/verify.sh --status` for HEAD, waiting on
   a run in flight. `pass` is the plan's tip verified. `fail` is an earlier
   step's red: a fresh subagent on that step's tier fixes it in a commit
   carrying that step's suffix, before retirement. Then retire the plan in
   this session by
   `.agents/skills/retire-plan/SKILL.md`; its review subagent runs on
   tier 3. A hard finding goes to a fresh subagent on the tier of the step
   it repairs, the finding as its brief; the conductor commits only the
   retirement.
5. Reply as *Do*'s step 7 does, one line per step conducted (its commit,
   its tier, anything surprising), then the retirement's reply.

`$ARGUMENTS`
