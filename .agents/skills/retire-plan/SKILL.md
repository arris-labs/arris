---
name: retire-plan
description: Close a completed plan — verify every step is ticked and the acceptance test passes, execute its docs-to-update list, run a drift check on the design docs it touched, update the roadmap status line and AGENTS.md current state, delete the plan file, commit. Use when the human says "retire", "close the plan", "plan is done", or after the acceptance test of a plan's last step passed and the human confirmed.
argument-hint: <plan slug>
---

# /retire-plan — move the durable parts, delete the rest

Deletion is the "done" signal. Anything worth keeping was moved first.

## Do

1. Open `docs/plans/<slug>.md`. Every step ticked? Acceptance run and green
   (run it now)? If not, stop and report which. **This is where the full
   profile runs, once per plan** (ADR-0032): `cargo nextest run
   --workspace` (the default profile: every test, `real_*` included, 256
   cases), `cargo test --workspace --doc`, `RUSTDOCFLAGS="-D warnings"
   cargo doc --workspace --no-deps`, and `ARRIS_ORACLE_CACHE=off` for the
   oracle comparisons the plan touched. If the plan changed `arris-mesh`,
   `arris-ops` or `arris` source, also `cargo nextest run -p <crate>
   --features parallel` for each. The per-step hook never ran any of it.
2. Execute *Docs to update* line by line. Design docs stay present tense —
   describe the system as it now is; no "as of this plan" narrative.
   If an `⚠ OPEN:` was closed by a decision, write the ADR now and add it to
   `docs/adr/README.md`.
3. **Drift check** on every design doc the plan touched: read it against the
   code and fix every sentence that is no longer true. List what you fixed
   in the reply.
4. `docs/ROADMAP.md`: update the milestone's status line. If this plan
   completes a milestone, say so and remind the human to tag `mN`.
5. `AGENTS.md` "Current state": one milestone-level sentence, keep the block
   under ~15 lines.
6. **`CHANGELOG.md`**: the plan's bullets under `## Unreleased`, if a
   consumer would notice it (ADR-0027). Write them for someone who has
   read only `README.md`. Say what they can now do, not what was built:
   "a sketch with an elliptic arc extrudes into a solid" is a change,
   `Surface::EllipticCylinder` is not. Add the refusals they will hit. Put
   every public type or signature the plan's commit bodies name as changed
   under `### Breaking`, with the one-line fix. Leave out ADR numbers, plan
   slugs and fixture names. Some commits may already have added their own
   `Breaking` bullets: merge them, don't duplicate them. A plan no
   consumer can see (process, tooling, tests) adds nothing.
7. Anything deferred from the plan goes to `docs/BACKLOG.md` as one line.
8. `git rm docs/plans/<slug>.md` and commit everything as
   `docs: retire plan <slug>` with a body listing the docs updated.
9. **Sweep the build cache.** A plan's commits leave a copy of every
   crate and test binary per build in `target/`, and nothing removes
   them: past a hundred gigabytes the disk fills and builds fail. After
   the commit (step 1's full run already built the tests; the commands
   below rebuild only what it did not), keep what the workspace builds now and drop the rest
   (`cargo-sweep`, installed per the setup in `AGENTS.md`):

   ```sh
   cargo sweep --stamp
   cargo nextest run --workspace --no-run && cargo clippy --workspace --all-targets
   cargo sweep --file
   rm -rf target/debug/incremental
   ```

   Say in the reply how much it freed (`cargo sweep` prints it). It
   touches build output only, never tracked files, fixtures or
   `target/oracle-cache`.
10. End the reply with **one recommended next step**, read from the
   roadmap's open lines, the other active plan and the backlog: the
   skill to run and its argument (`/plan <slug>`, `/idea <topic>`,
   `/work <plan>`, `/close-cycle`) and one sentence on why it comes
   before the alternatives.

## Don't

- Don't summarise the plan into a design doc — design docs hold the design,
  not the history of how it got there.
- Don't keep the plan file "for reference"; git has it.
- Don't write the plan's history into `CHANGELOG.md`. It holds what
  changed for a consumer, not the steps that got there.
- Don't retire with unticked boxes by editing them to ticked.

`$ARGUMENTS`
