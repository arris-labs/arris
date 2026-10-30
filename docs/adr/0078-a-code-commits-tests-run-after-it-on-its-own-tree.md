# ADR-0078 — A code commit's tests run after it, on its own tree

- Status: accepted (2026-10-31)
- Plan: `verify-after-commit` steps 1–6
- Amends: ADR-0032 (the hook row of the three tiers: the static half stays
  at the commit, the test half moves after it), ADR-0066 ("the hook runs 16
  cases": the 16 cases now run at verification), `.agents/rules/git.md`
  (its first rule)
- Follows: ADR-0024 §4 (time is never a gate; a test is not weakened to be
  fast)
- Reference trees read: none

## Context

The hook blocks a code commit for the whole of its gate. The idea's measure
(`development-throughput`): 3.4 min per code commit, 44 h over 784 commits,
the agent idle for all of it. Step 1 of this plan made the hook print its
stage times and replayed it on the five most recent code commits (2026-10-31,
each staged alone in a scratch worktree):

| commit   | staged | static | nextest | doctest | doc  | wall  |
|----------|--------|--------|---------|---------|------|-------|
| 4e493c28 | 4      | 91.8   | 572.8   | 6.8     | 42.7 | 714.1 |
| acfb40f0 | 4      | 1.7    | 19.5    | 6.7     | 0.0  | 28.0  |
| 5e4e7023 | 7      | 1.6    | 234.4   | 6.7     | 34.3 | 277.0 |
| 5c8a2a57 | 15     | 1.7    | 484.6   | 6.7     | 4.7  | 497.8 |
| f0c7bf30 | 32     | 1.9    | 484.2   | 6.7     | 0.0  | 492.8 |

The static half (layers, tolerance literals, fmt, clippy) is under 2 s on a
warm tree; a changed crate set costs a clippy check (4e493c28: 92 s). The
test half is the rest: nextest 20–573 s (compile, link and run together),
doctests 7 s, `cargo doc` 0–43 s. A cheaper test half (the relink,
`opt-level = 3` for tests; backlog item D) acts inside the nextest stage and
is measured separately; `real_nist_ftc_07` is not in the hook (`fast`
excludes `real_*`). The cheaper gate shortens the wait, this decision removes
it, and the two compose.

Two things the replay found that bind the design:

- **A path baked into a binary.** The scratch worktree's path went into the
  `arris-debug` and `arris` test binaries through `env!("CARGO_MANIFEST_DIR")`
  in the shared `target/`; cargo reused them once the worktree was gone and
  the next nextest failed on a missing fixture path (`cargo clean -p
  arris-debug -p arris` cleared it). A tree other than the checkout must not
  share the checkout's build output.
- **Memory.** The gate runs in a scope bounded by `MemoryHigh` 40G and
  `MemoryMax` 48G on a 61G machine (`tools/gate-scope.sh`). Two heavy builds
  at once can exceed the machine though each is within its bound.

## Decision

**1. Split the gate.** The *static half* (layer rule, tolerance literals,
fmt, clippy) runs in the hook, as today. The *test half* (the `fast` profile
at 16 cases over the reached crates and corpus areas, the doctests, `cargo
doc` when a doc comment changed) runs after the commit, in the background, on
the commit's own tree. A commit whose test half passed is **verified**
(`GLOSSARY.md`). Prose and version-bump commits keep their hook, which is
seconds, and take their parent's state.

**2. The verification's tree.** `tools/verify.sh <sha>` takes a lock, checks
`<sha>` out in one linked worktree, `arris-verify/tree` in the common git
dir, detached at each sha in turn so cargo rebuilds only what changed, and
runs the test half of `tools/gate.sh since <sha>^ <sha>` through
`tools/gate-scope.sh`. **Its build output is `CARGO_TARGET_DIR=target/verify`,
never the checkout's `target/`**, for the reason in the Context: the path of
the tree is baked into its binaries, and cargo reuses a binary whose inputs
it believes unchanged. `target/verify` is gitignored with `target/` and swept
with it. The state is a file per sha under `arris-verify/` (`running`, `pass`
or `fail`) with a log; `verify.sh --status` prints it and waits on the lock
while a run is in flight.

**3. A red is met at the next commit.** The next code commit's hook reads
`HEAD`'s state: `pass` defers its test half; `fail` or none runs it inline
over `gate.sh since <the last verified commit>`, today's gate on a wider
slice. A fix of the red carries the red step's suffix (`git.md`,
attribution). A `post-commit` hook records `pass` or starts the verification
detached (`setsid`), so a background `git commit` returns at once.

**4. A push waits for a verified tip.** `pre-push` refuses a tip that is not
`pass`, waiting on a running one. `main` at every pushed tip is as green as
under the blocking hook; only an unpushed tip may be red. `ARRIS_VERIFY=inline`
keeps the blocking hook.

**5. One heavy build at a time.** `tools/full-gate.sh` holds the verify lock
for its run (a verification in flight finishes first; a new one queues
behind the full gate), so the two never overlap. The lock is
`arris-verify/lock`.

**6. Memory: the verification's scope is lowered.** A verification runs in
its own `tools/gate-scope.sh` scope at `MemoryHigh` 20G and `MemoryMax` 24G
(`ARRIS_VERIFY_MEMORY_HIGH` / `_MAX` override), against the hook's 40G and
48G. Two scopes each bounded at 48G on the 61 GiB machine could together
exceed it, and the kernel's pick under pressure is the biggest process, which
may be the agent's clippy or the terminal's scope; a verification is the
one that can wait, so it is the one bounded low, and a verification killed by
its bound reads `fail` with the bound printed in its log. Step 1 recorded
stage times, not a peak (the gate's scope has no unit name to read a
`MemoryPeak` from, a backlog line), so 24G is a bound chosen with room, not
a measured fit: a verification that dies at it is the evidence to raise it.

⚠ OPEN: a verification still running when the next commit comes: the hook
waits for it (the default, one verification at a time) or defers again and
leaves two queued (plan open question 2, step 5).

## Consequences

- A code commit costs its static half, seconds on a warm tree, instead of
  20–715 s; the agent starts the next step at once. The test half's cost is
  unchanged and now overlaps the agent's work.
- A red commit can sit unpushed under further commits until the next hook
  or the push meets it; the bisect range is bounded by the last verified
  commit.
- `target/verify` is a second build of the workspace on disk; `/retire-plan`'s
  sweep covers it.
- The three halves of the gate (static, test, full) are named in `GLOSSARY.md`
  (*Gate*).
