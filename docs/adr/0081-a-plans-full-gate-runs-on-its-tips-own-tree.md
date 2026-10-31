# ADR-0081 — A plan's full gate runs on its tip's own tree

- Status: accepted (2026-10-31)
- Plan: none (`/retro horn-torus`)
- Amends: ADR-0078 §5 (the full gate also runs on the verification's
  tree), `/retire-plan` step 1 ("nothing commits until the gate is green")
- Reference trees read: none

## Context

`/retire-plan` starts the full gate in the checkout and its review beside
it. A gate in the checkout reads the working tree, so nothing may change
under it: the review's hard findings wait for the gate to go green before a
fix is written. In the horn-torus retirement (2026-10-31) the review came
back in 4 min with four hard findings; the conductor then sat idle for
about 43 min of the 47 min gate (nextest 2780 s), and the fixes, made after
it, were held only by their own verifications. The full gate was the
largest serial block left in a conducted plan.

ADR-0078 already runs a commit's test half on the commit's own tree, a
linked worktree (`arris-verify/tree`) with its own `target/verify`, under a
lock that keeps one heavy build at a time.

## Decision

**1. A command runs on a commit's own tree.** `tools/at-commit.sh <sha>
<command>` takes the verification's lock, detaches the verification's tree
at `<sha>` (`tools/verify-tree.sh`, shared with `tools/verify.sh`) and runs
the command there with `CARGO_TARGET_DIR=target/verify`. It does not take
the hook's lock: commits go on in the checkout.

**2. `/retire-plan` gates the plan's tip there.** `tools/at-commit.sh HEAD
sh -c 'tools/full-gate.sh <log>; …'`, the `parallel` and Python runs after
the gate in the same command. The review's hard fixes are made and
committed while it runs; the retirement commit waits for the gate green and
HEAD verified `pass`. The fixes after the gated tip are held by their
verifications, as every commit is (ADR-0078); a plan whose fixes reach far
reruns the gate on the new tip.

**3. Memory and disk as a verification.** The run is beside the agent's
builds, so its scope is the verification's (MemoryHigh 20G, MemoryMax 24G,
ADR-0078 §6), and it shares `target/verify` rather than adding a third
build directory to a disk that reached 97% the same day. A gate killed by
the bound prints it; `ARRIS_VERIFY_MEMORY_MAX` raises it.

**4. One heavy build at a time still holds.** The lock is the
verification's, so a commit's verification queues behind the gate; a second
code commit's hook waits on the first's verdict (ADR-0078 §7). The overlap
won is the fix work, not two builds at once.

## Consequences

- A retirement's review fixes overlap its gate: about 40 min saved on a
  plan like horn-torus.
- The gate tests the tip it was started on, never a tree mid-edit.
- A verification may wait up to the gate's length; the hook of the next
  code commit after it waits too.
- `tools/full-gate.sh` in the checkout is unchanged for `/close-cycle` and
  `/release`, which have nothing to overlap.
