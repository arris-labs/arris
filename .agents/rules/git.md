# Git: trunk-based, always green

- **Every commit** passes `.githooks/pre-commit`; never `--no-verify`. The
  hook sizes its gate to what is staged (`tools/gate.sh`, ADR-0032, split by
  ADR-0078). Prose runs the docs tests; a version bump runs `cargo check`;
  anything else runs the layer rule, new tolerance literals, fmt and
  `clippy -D warnings` at the commit, and returns. Its **test half** (the
  doctests and the `fast` nextest profile at 16 cases over the crates and
  corpus areas the paths reach, plus `cargo doc` when a doc comment changed)
  runs after the commit, in the background, on the commit's own tree
  (`tools/verify.sh`). A commit whose test half passed is **verified**
  (`GLOSSARY.md`). A code commit never waits for an earlier commit's
  verification: its own is queued, and one still waiting is **covered** by it
  (ADR-0085), tested over the range since the last verified commit. A red one
  is met at the next code commit, whose hook runs the test half inline over
  everything since the last verified commit (`tools/verify.sh --base`); the
  fix of it carries the red step's suffix. A prose or version commit takes
  its parent's state. `pre-push` refuses a tip that is not verified, so
  `main` at every pushed tip is as green as before. The state is read with
  `tools/verify.sh --status [<sha>]`, which waits on a run in flight
  (`--in-flight` does not); a waiter is a background job or `pre-push`, never a poll loop. A
  commit or gate runs
  with `run_in_background`; wait for its completion notification, never
  `sleep` or a watch on its output for a string. One gate runs at a time:
  the hook refuses a second, and a verification waits behind the full gate
  (`tools/full-gate.sh` holds the verify lock). A gate an earlier session left running is
  waited on with `flock .git/arris-gate.lock true` in the background; any
  other run it left is rerun, never waited on (no notification comes).
  Never use a `pgrep -f` loop, which matches its own shell and never ends. A
  stuck one is stopped with `tools/stop-gate.sh`, never a bare `pkill -f`.
  `ARRIS_VERIFY=inline` keeps the blocking hook.
- **The gate's scope.** The hook and `tools/full-gate.sh` run themselves
  through `tools/gate-scope.sh`: where `systemd-run --user` works, in a
  transient scope with `MemoryHigh` 40G, `MemoryMax` 48G and no swap, so
  memory pressure ends the gate and not the terminal's scope. A verification
  runs in a lower bound, `MemoryHigh` 20G and `MemoryMax` 24G
  (`ARRIS_VERIFY_MEMORY_HIGH` / `_MAX`), since it is the one that can wait
  (ADR-0078 §6); a verification killed by it reads `fail`. A gate killed
  this way prints the bound; raise it with `ARRIS_GATE_MEMORY_MAX`. Without
  `systemd-run`, or with `ARRIS_GATE_SCOPE=off`, the gate runs unchanged.
- **No foreground waits.** A test run expected to take over a minute, and
  a CI watch, runs with `run_in_background`, and the agent goes on with
  what does not depend on its result. A foreground `until`, `while` or
  `sleep` loop, or a `tail --pid`/`tail -f`, waiting on one is never
  written: the completion notification is the wait. In Claude Code
  `tools/bash-guard.sh` refuses it (a PreToolUse hook).
- **The full gate** is `tools/full-gate.sh <log>`, run with
  `run_in_background`: the layer rule, fmt, clippy `-D warnings`, nextest
  over the workspace (256 cases, `real_*` included), the doctests, `cargo
  doc` with warnings denied. Every stage runs; its last lines name each red
  stage and test. It runs once per plan that reaches the code (`/retire-plan`,
  on the tip's own tree by `tools/at-commit.sh` so fixes go on beside it,
  ADR-0081), at
  `/close-cycle` and `/release`. CI runs it at 1000 cases plus the
  `oracle`, `wasm` (which runs the determinism corpus on `wasm32-wasip1`),
  `parallel` and `python` jobs no local gate runs; the
  human reads CI before a push that matters. A push whose diff from the
  nearest green commit of `main` is version strings, the changelog and
  prose no test reads (`tools/ci-reuse.sh`, ADR-0069) skips the
  hours-long jobs: a release's CI run rests on the commit it was cut from
  and is short.
- Commit **directly to `main`**, one plan step per commit. Branch
  (`plan/<slug>`, rebased, `merge --ff-only`, deleted after) only when a
  plan may be thrown away or the human asked to review first.
- Never rewrite `main` (no amend of a pushed commit, no force-push, no
  rebase). Never push unless asked.
- Stage deliberately: read `git status` and `git diff`, no blind `git add
  -A`. No `git stash`: a pop can bring back another session's work
  (`tools/bash-guard.sh` refuses it). A changed `tests/fixtures/**/*.json` expectation is staged only
  with an intentional geometry change, and the message says `fixtures:`
  and why. Oracle values do not drift; if they changed, the body says
  whether the fixture or the oracle script did.

## Message format

```
type(scope): imperative summary, lower case, no period  (plans/<slug> step N)

Why, not what. Cite the ADRs and docs that justify it or that it updated.
Name any public type or signature it changes.
```

- `type`: `feat`, `fix`, `refactor`, `perf`, `test`, `docs`, `chore`,
  `build`, `ci`, `fixtures`. `scope`: crate short names (`math` … `arris`,
  several as `geom,ops`) or `docs`, `plan`, `adr`, `tools`.
- The `(plans/<slug> step N)` suffix marks every plan-step commit;
  retirement is `docs: retire plan <slug>`.
- **Attribution**: a commit that repairs an earlier step's work, of any
  type, carries that step's suffix, never a later one's; a fix found by the
  retirement review carries `(plans/<slug> review)`. `tools/step-rework.sh`
  reads these to count, per tier, the steps that needed a later commit.
- Docs change in the **same commit** as the behaviour; a docs-only catch-up
  is `docs(sync): …`.

## Tags

- `m0`, `m1`, … mark milestones; `vX.Y.Z` releases, on `main`, created and
  pushed by the human. The pushed tag publishes the crates to crates.io
  and the Python package `arris` to PyPI in lockstep (ADR-0034), each
  behind its own environment reviewer (`crates-io`, `pypi`). `arris-debug`
  and `arris-py` are `publish = false`. Mechanics: the `/release` skill.

### The version

- `main` carries the next version with `-dev`: `[workspace.package].version
  = "0.N.0-dev"`, and the seven internal crates pinned to exactly it in
  `[workspace.dependencies]` (`version = "=0.N.0-dev"`). The exact pin is
  the guard: a bump that misses one fails `cargo check`, and `release.yml`
  refuses a pre-release. PyPI reads it as PEP 440 (`0.5.0.dev0`).
- Pre-1.0, `0.y.z` is `y` breaking. **Closing a cycle bumps the minor**
  (Cn → `0.n.0` while that lines up); **a release between cycles bumps the
  patch**, unless `CHANGELOG.md`'s `Unreleased` has a `### Breaking`
  bullet — a lookup, not a judgement. `tools/semver-gate.sh` fails CI on a
  break `Breaking` doesn't list.
- The version lives only in `Cargo.toml` and `CHANGELOG.md`'s headings —
  no doc, README or rustdoc states it.

### The changelog

`CHANGELOG.md` is what a consumer reads (ADR-0027).

- An entry is written in the commit that makes the change; a plan's
  bullets by its `/retire-plan` commit. Changes no consumer can see
  (process, tooling, tests, docs) write nothing.
- Bullets go under `## Unreleased`, for a reader who has read only
  `README.md`: what they can now do (not what was built), the refusals
  they will hit, no ADR numbers, plan slugs or fixture names. A changed
  public type or signature goes under `### Breaking` with its one-line
  fix, in the same commit.
- Only `/release` turns `Unreleased` into a version.

## Without asking

While executing a plan: run checks, commit each step, tick boxes, edit the
plan. Branching, tagging, pushing, resetting, rewriting and publishing are
asked first.
