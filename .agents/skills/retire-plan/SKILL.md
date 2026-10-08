---
name: retire-plan
description: Close a completed plan — verify every step is ticked and the acceptance test passes, execute its docs-to-update list, run a drift check on the design docs it touched, update the roadmap status line and AGENTS.md current state, delete the plan file, commit. Use when the human says "retire", "close the plan", "plan is done", or after the acceptance test of a plan's last step passed and the human confirmed.
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
   `arris-mesh`, `arris-ops`, `arris` whose source changed. Anything red:
   stop and report.
2. Execute *Docs to update* line by line, present tense, no "as of this
   plan". A closed `⚠ OPEN:` gets its ADR now, listed in
   `docs/adr/README.md`.
3. **Drift check** every design doc the plan touched against the code; list
   the fixes in the reply.
4. `docs/ROADMAP.md`: update the status line; if the plan completes a
   milestone, say so and remind the human to tag it.
5. `AGENTS.md` "Current state": one milestone-level sentence, under ~15
   lines.
6. `CHANGELOG.md` `## Unreleased`: the plan's bullets if a consumer would
   notice (`.agents/rules/git.md` §The changelog). Every public type or
   signature the commit bodies name goes under `### Breaking` with its fix;
   merge with bullets commits already added. Never the plan's history.
7. Deferred work: one line each in `docs/BACKLOG.md`.
8. `git rm docs/plans/<slug>.md`; commit all as `docs: retire plan <slug>`,
   the body listing the docs updated.
9. **Sweep the build cache** (`target/` grows past 100 GB otherwise; build
   output only, never `target/oracle-cache`):

   ```sh
   cargo sweep --stamp
   cargo nextest run --workspace --no-run && cargo clippy --workspace --all-targets
   cargo sweep --file
   rm -rf target/debug/incremental
   ```

   Report how much it freed.
10. End with **one recommended next step** from the roadmap's open lines,
    the other plan and the backlog: the skill and its argument, and one
    sentence on why it beats the alternatives.

## Don't

- Don't summarise the plan into a design doc or keep the file "for
  reference" — git has it.

`$ARGUMENTS`
