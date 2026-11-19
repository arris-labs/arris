---
name: retire-plan
description: Close a completed plan: verify, review its commits against the plan, update the docs, delete the plan, sweep the build cache.
disable-model-invocation: true
argument-hint: <plan slug>
---

# /retire-plan — move the durable parts, delete the rest

Run by the human or a fresh coordinator after `/work`'s last step.
The handoff carries the plan slug, verified SHA, acceptance commands and
docs checklist. Deletion is the done signal.

## Do

1. **Check the handoff and review the commits.** Every step of
   `docs/plans/<slug>.md` is ticked. Check that HEAD matches the verified
   SHA, or verify HEAD with `tools/verify.sh --status HEAD` if starting
   directly. Use the harness-specific wait in `.agents/rules/git.md`.
   Start the review in a tier 3 subagent (`.agents/tiers.md`). Give it the
   plan file, `.agents/review.md`, `.agents/rules/kernel.md`, `GLOSSARY.md`
   and the plan commits from `git log --format='%h %s' --grep='plans/<slug>'`.
   It reads each commit with `git show`, since other plans may interleave.
   Report findings by axis. A fresh subagent on the tier of the affected
   step fixes each hard finding in its own commit
   (`fix(scope): … (plans/<slug> review)`). Resolve judgement findings on
   the charter's terms (`SEED.md`) and state the decision in the reply.
   A product question waits for the human. Deferred findings go in the
   backlog.
2. **Verify the reviewed tip and start acceptance.** Once review and fixes
   have handed back, pin HEAD and run `tools/verify.sh --status <sha>`.
   Fix any red verification before launching acceptance on that SHA's
   tree. Run the plan's acceptance via
   `tools/at-commit.sh <sha> sh -c '<runs>'` using the harness-specific
   wait rule. For a tooling-only diff limited to shell scripts under
   `tools/`, GitHub workflow YAML and prose, run the affected checks named
   in the plan and exercise any changed routing. Use the full gate if the
   diff changes kernel or test behavior, including how the full gate runs
   tests. For other diffs that reach code, include
   `tools/full-gate.sh <absolute-log>`. Use
   `ARRIS_ORACLE_CACHE=off` for the oracle comparisons the plan touched.
   Run `cargo nextest run -p <crate> --features parallel` for each of
   `arris-mesh`, `arris-ops` and `arris` whose changed files intersect
   `git grep -l 'feature = "parallel"' crates`. Include
   `tools/py-test.sh` if the plan touched `crates/arris-py`.
   As job owner, report the pinned SHA, expected finish (about 55 minutes
   for the full gate), exit-status path or harness job ID, and log path.
   Inspect the log on failure or at the first check after a missed expected
   finish. Give the conductor the result in the hand-back. Steps 3–8 can proceed during acceptance. A red
   result stops retirement until fixed and verified on the new tip.
   A prose-only plan uses the commits' hooks as its gate.
3. Execute *Docs to update* line by line, present tense, no "as of this
   plan". A closed `⚠ OPEN:` gets its ADR now, listed in
   `docs/adr/README.md`.
4. **Drift check** every design doc the plan touched against the code; list
   the fixes in the reply.
5. `docs/ROADMAP.md`: update the status line; if the plan completes a
   milestone, say so and remind the human to tag it.
6. `AGENTS.md` "Current state": one milestone-level sentence, under ~15
   lines. Its `**Next:**` line names
   `/plan <slug>` while the slug has no file under `docs/plans/`, and
   `/work <slug>` only once it does.
7. `CHANGELOG.md` `## Unreleased`: the plan's bullets if a consumer would
   notice (`.agents/rules/git.md` §The changelog). Every public type or
   signature the commit bodies name goes under `### Breaking` with its fix;
   merge with bullets commits already added. Never the plan's history.
8. Deferred work: one line each in `docs/BACKLOG.md`.
9. Once acceptance is green and HEAD is verified, `git rm docs/plans/<slug>.md`; commit all as `docs: retire plan <slug>`,
   the body listing the docs updated. `git grep -n <slug> -- ':!docs/plans'`
   comes back empty first: the docs-refs test refuses a file citing it. An
   idea's `absorbed by plans/<slug>` status and the backlog's mentions are
   reworded to name the part done ("the rewrite seams part is done"), not
   deleted. An idea whose last part this plan absorbed is `git rm`'d in the
   same commit (`docs/README.md`, the idea lifecycle), and `git status
   --short` shows no staged or tracked change left behind.
10. **Sweep the build cache** (`target/` grows past 100 GB otherwise; build
   output only, never `target/oracle-cache`), in the background under the
   verification's lock, so it waits out a verification in flight and none
   starts while it deletes their binaries:

   ```sh
   flock "$(git rev-parse --git-common-dir)/arris-verify/lock" sh -c '
     cargo sweep --stamp
     cargo nextest run --workspace --no-run && cargo clippy --workspace --all-targets
     cargo sweep --file
     rm -rf target/debug/incremental target/verify/debug/incremental
     CARGO_TARGET_DIR=target/verify cargo sweep --time 2'
   ```

   `target/verify` is the verification's build output (ADR-0078); its last
   line drops what no verification has used in two days (a week left it at
   175 GB beside `target/debug`'s 93 GB on 2026-11-07), and the next one
   rebuilds the rest.

   Report how much it freed.
11. End with **one recommended next step** from the roadmap's open lines,
    the other plan and the backlog: the skill and its argument, and one
    sentence on why it beats the alternatives. Suggest `/retro <slug>`
    first if the plan's sessions had friction worth mining (long stalls,
    repeated corrections, the human asking where things stand).

Durable facts move to the design docs and ADRs; the plan itself is
deleted, not summarised or kept for reference — git has it.

`$ARGUMENTS`
