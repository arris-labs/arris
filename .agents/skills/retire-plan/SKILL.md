---
name: retire-plan
description: Close a completed plan: verify, review its commits against the plan, update the docs, delete the plan, sweep the build cache.
disable-model-invocation: true
argument-hint: <plan slug>
---

# /retire-plan — move the durable parts, delete the rest

Run by the human, or by `/work`'s conductor after a plan's last step.

Deletion is the "done" signal; anything worth keeping moves first.

## Do

1. **Verify, starting the gate.** Every step of `docs/plans/<slug>.md`
   ticked (never tick one here). First `tools/verify.sh --status HEAD`, in
   the background (it waits on a run in flight): a `fail` is fixed before
   the gate starts — the verification takes minutes, the gate an hour.
   Then start the acceptance's runs, in the
   background (`run_in_background`; no wait loop, `.agents/rules/git.md`),
   on the plan's tip's own tree so the checkout moves on (ADR-0081):
   `tools/at-commit.sh HEAD sh -c '<runs>'`, the runs being the plan's
   **full gate**, `tools/full-gate.sh <scratchpad>/gate.log` (absolute;
   about an hour; its last lines name every red test), then
   `ARRIS_ORACLE_CACHE=off` for the oracle comparisons the plan touched,
   `cargo nextest run -p <crate> --features parallel` for each of
   `arris-mesh`, `arris-ops`, `arris`, only when the plan's diff touches a
   file `git grep -l 'feature = "parallel"' crates` lists (CI's `parallel`
   job covers the rest; the three runs cost 60 minutes on rewrite-seams for
   a plan that touched none), and
   `tools/py-test.sh` when the plan touched `crates/arris-py`. The reply
   names each run's expected finish (the gate: 55 minutes) when it starts.
   Steps 2–4 run while they do, the review's fixes committed as they come. The
   retirement commit waits for the gate green and HEAD verified `pass`;
   anything red: stop and report (a conductor fixes it by `/work`'s
   *Conduct* step 4). A plan whose diff (`git diff --name-only`
   over its commits) reaches no code path (`tools/gate-reuse.sh
   --reaches-code <the plan's first commit's parent> HEAD` exits 1, the
   one list ADR-0085 §3 names) runs no full gate: its commits' hooks were
   its gate, and the reply says so.
2. **Review the commits against the plan**, in a subagent on tier 3
   (`.agents/tiers.md`) so it reads with fresh eyes, started while the gate
   runs. Give it the plan file, `.agents/review.md` (its brief and
   standards), `.agents/rules/kernel.md`, `GLOSSARY.md`, and the plan's
   commits: `git log --format='%h %s' --grep='plans/<slug>'` (another
   plan's commits may sit between them, so it reads each with `git show`,
   not one range diff). Report its findings by axis, unmerged. Fix a hard
   finding in its own commit (`fix(scope): … (plans/<slug> review)`),
   made at once: the gate reads the tip's tree, not the checkout. A judgement finding the agent decides on the
   charter's terms (`SEED.md`), stating the decision and its reason in the
   reply — a choice between fixes included; only a product question (what
   the kernel offers its callers) waits for the human. One left
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
   the body listing the docs updated. `git grep -n <slug> -- ':!docs/plans'`
   comes back empty first: the docs-refs test refuses a file citing it. An
   idea's `absorbed by plans/<slug>` status and the backlog's mentions are
   reworded to name the part done ("the rewrite seams part is done"), not
   deleted.
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
