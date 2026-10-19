---
name: retire-plan
description: Close a completed plan: verify, review its commits against the plan, update the docs, delete the plan, sweep the build cache.
disable-model-invocation: true
argument-hint: <plan slug>
---

# /retire-plan — move the durable parts, delete the rest

Deletion is the "done" signal; anything worth keeping moves first.

## Do

1. **Verify.** Every step of `docs/plans/<slug>.md` ticked (never tick one
   here), and the acceptance run green — run it now. This is where the
   plan's **full gate** runs (`.agents/rules/git.md`), plus
   `ARRIS_ORACLE_CACHE=off` for the oracle comparisons the plan touched,
   and `cargo nextest run -p <crate> --features parallel` for each of
   `arris-mesh`, `arris-ops`, `arris` whose source changed. The nextest
   run takes about half an hour: give it `--no-fail-fast`, send the whole
   output to a scratchpad file, with no pipe filter and no timeout, run it
   in the background, then grep the file for `FAIL`, `SIGSEGV` and
   `TIMEOUT`. A single run then names every red test. Anything red: stop
   and report.
2. **Review the commits against the plan**, in a subagent so it reads with
   fresh eyes. Give it the plan file, `.agents/review.md` (its brief and
   standards), `.agents/rules/kernel.md`, `GLOSSARY.md`, and the plan's
   commits: `git log --format='%h %s' --grep='plans/<slug>'` (another
   plan's commits may sit between them, so it reads each with `git show`,
   not one range diff). Report its findings by axis, unmerged. Fix a hard
   finding in its own commit (`fix(scope): … (plans/<slug> review)`)
   before going on. A judgement finding the agent decides on the
   charter's terms (`SEED.md`), stating the decision and its reason in the
   reply; only a scope or product question waits for the human. One left
   for later is a backlog line.
3. Execute *Docs to update* line by line, present tense, no "as of this
   plan". A closed `⚠ OPEN:` gets its ADR now, listed in
   `docs/adr/README.md`.
4. **Drift check** every design doc the plan touched against the code; list
   the fixes in the reply.
5. `docs/ROADMAP.md`: update the status line; if the plan completes a
   milestone, say so and remind the human to tag it.
6. `AGENTS.md` "Current state": one milestone-level sentence, under ~15
   lines.
7. `CHANGELOG.md` `## Unreleased`: the plan's bullets if a consumer would
   notice (`.agents/rules/git.md` §The changelog). Every public type or
   signature the commit bodies name goes under `### Breaking` with its fix;
   merge with bullets commits already added. Never the plan's history.
8. Deferred work: one line each in `docs/BACKLOG.md`.
9. `git rm docs/plans/<slug>.md`; commit all as `docs: retire plan <slug>`,
   the body listing the docs updated.
10. **Sweep the build cache** (`target/` grows past 100 GB otherwise; build
   output only, never `target/oracle-cache`):

   ```sh
   cargo sweep --stamp
   cargo nextest run --workspace --no-run && cargo clippy --workspace --all-targets
   cargo sweep --file
   rm -rf target/debug/incremental
   ```

   Report how much it freed.
11. End with **one recommended next step** from the roadmap's open lines,
    the other plan and the backlog: the skill and its argument, and one
    sentence on why it beats the alternatives. Suggest `/retro <slug>`
    first if the plan's sessions had friction worth mining (long stalls,
    repeated corrections, the human asking where things stand).

Durable facts move to the design docs and ADRs; the plan itself is
deleted, not summarised or kept for reference — git has it.

`$ARGUMENTS`
