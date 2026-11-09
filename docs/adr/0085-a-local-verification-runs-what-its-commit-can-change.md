# ADR-0085 — A local verification runs what its commit can change

- Status: accepted (2026-11-09)
- Plan: `local-gate-diet` steps 1–9
- Amends: ADR-0032 (the unplaced default stays `whole`; the paths it could not
  place are now placed, and the blend rule narrows a `src/blend/*` change's
  corpus), ADR-0078 §7 (the hook no longer waits for a running verification;
  a queued commit is covered by the newer one). ADR-0078 §3's deferral and
  §4–§6 stand.
- Follows: ADR-0066 (the slow set is a command's output; its rule is applied
  again here), ADR-0069 (a diff from a green commit made only of paths no test
  reads keeps that commit's verdict: the precedent for reuse), ADR-0081 (a
  plan's full gate runs on its tip's own tree; its record is what reuse reads)
- Reference trees read: none

## Context

Measured from `.git/arris-verify/*.log`, 2026-10-31 to 11-09 (152 commits, 93
of them code commits with a verification): 64 whole `fast` runs took 10.2 h
(about 10 min each); two thirds of the verifications ran the whole workspace,
and some of them could not catch anything.

- **Paths no local test reads ran whole.** Any path `tools/gate.sh` could not
  place set `whole=yes`. `.github/workflows` (in 7 whole runs), `tools/oracle`
  (11) and `.config/nextest.toml` (3) were such paths; the regression fixtures
  fell to the same default. A YAML commit ran 2910 tests for 10 min.
- **`arris-ops` reaches almost everything.** Its source reaches its reverse
  dependencies, and most module rules add `corpus=all`. A `src/blend/*` change
  therefore ran every corpus row, although no module of `arris-ops` outside
  `src/blend/` calls into it (only `lib.rs` re-exports).
- **The full gate ran again on a tree it had passed.** `/close-cycle` and
  `/release` each ran the full gate "now" after a `/retire-plan` gate with only
  docs in between: up to two more 47-minute runs on the same code.
- **The hook waited.** ADR-0078 §7 made a code commit that came while `HEAD`'s
  verification ran wait for its verdict: 19 of 93 code commits did.

Corrections found when the plan checked the idea against the code:

1. Blending fixtures are read from the whole `fixture.json`: 13 part fixtures
   hold their blends under `battery` (a `steps` reader finds 225 of 238 files).
2. A corpus row's name comes from `corpus.rs` by its `"area/dir"` string; six
   directories have several rows.
3. `tests/fixtures/regression/` needs more than its test by name: all its rows
   are `#[ignore]`, so the term runs nothing, and the readers are the tests
   that name each slug.
4. `tools/oracle/*` reaches beyond `package(arris)`: `arris-debug`, eight
   `arris-io` binaries and three `arris-ops` binaries call the oracle.
5. A `src/blend/*` change's code paths gain `tools/oracle/`, whose sources are
   in the verification's cache key.

The corpus's `blend` area is 2,841 s of a whole run's 9,236 s of test time; the
oracle's rule saves about the properties' share (2,400 s), not the 10 minutes
the idea first counted.

## Decision

**1. Placed paths (ADR-0032's unplaced default stays).** `tools/gate.sh` places:

- `.github/*`: no nextest; `tool_tests` gains `tools/workflows-test.sh`
  (`actionlint` when installed, else a line saying so).
- `.config/nextest.toml`: no nextest; `tool_tests` gains
  `tools/profile-test.sh`, which holds the `fast` filter to the slow set.
- `tests/fixtures/regression/<slug>/*`: `binary_id(arris::corpus_lint)`, the
  test that every fixture directory has a corpus test, and each file naming
  `regression/<slug>` (read by `git grep -F` under `crates/`): a
  `tests/<f>.rs` gives its binary, anything else its package.
- `tools/oracle/*`: `package(arris)` with the corpus, `package(arris-debug)`,
  and the binaries whose non-comment lines call `oracle::`, `compare_stl`,
  `corpus::run`, `part::run` or `query::run`. `tool_tests` gains
  `tools/oracle-test.sh` (the oracle's `selftest.py` over `primitive/`; the
  whole selftest stays CI's).

Every rule has a `gate-test.sh` case. A test that reads a placed path outside
its rule is not run locally; CI and the full gate still run it.

**2. Blend reach (D2).** A `src/blend/*` change reaches the area `blend`, and
the rows of every fixture outside `blend/`, `real/` and `regression/` whose
file has a `fillet` or `chamfer` op (read from the whole `fixture.json`, so a
new fixture is placed with no list to edit). Regression rows are ignored and
`real_*` is outside `fast`, so neither is named. The reach is a `test(...)`
filter beside the prefix regex.

The rule's guard runs at classification. Any `blend::`, `crate::fillet` or
`crate::chamfer` under `crates/arris-ops/src` outside `blend/`, or a `lib.rs`
re-export other than `pub use blend::{chamfer, fillet};`, sets `corpus=all`.
A caller outside the directory is the only way a blend change can alter a
non-blending recipe, so the rule holds for as long as the guard does.

**3. A full gate's verdict is reused (ADR-0069's reasoning, ADR-0081's
record).** `tools/full-gate.sh` calls `tools/gate-reuse.sh --record` when no
stage failed; it writes `arris-verify/gated/<HEAD>`, and refuses when a code
path is dirty. Without an argument, `gate-reuse.sh` names the nearest recorded
ancestor of `HEAD` whose diff to `HEAD` reaches no code path, and exits 1 when
there is none. A swept record costs one more gate, never a wrong reuse.

The code paths are one list, `gate-reuse.sh --reaches-code <a> <b>`: the list
`/retire-plan` already uses, plus `tools/oracle/`, `tools/check-*` and the
non-`.sh` files of `tools/` (`real-parts.waits` is read by tests). `/retire-plan`
cites the same call, so the two rules cannot disagree.

**4. A queued commit is covered (supersedes ADR-0078 §7's wait).** The hook
does not wait for `HEAD`'s verification. A code commit's verification queues,
and when the commit before it is still `queued` and is its ancestor, the newer
run writes `covered <new-sha>` to that commit's state and takes its place.
A commit of another line is left alone. `queued` → `running` happens under a
short `queue.lock`, so one verification still runs at a time (ADR-0078 §6's
memory bound holds). `verify.sh --status` follows `covered` and still prints one
word; `verify.sh --in-flight <sha>` exits 0 while `queued` or `running`, without
waiting. A red covering several commits names the range, not one commit (the
glossary's *Covered*); a red `HEAD` is still met at the next commit's hook, as
ADR-0078 §3 has it.

`pre-push` still refuses a tip that is not `pass`, and a covered commit takes
its run's verdict. `ARRIS_VERIFY=inline` still waits, and a prose or version
commit still takes its parent's state.

**5. The slow set is refreshed by ADR-0066's rule (D1).**
`tools/test-timings.sh --slow-set` prints the single tests at 30 s or more
under the hook's counts; `.config/nextest.toml` and `tools/profile-test.sh`
take that list. The rule was last applied on 10-20, before the C11 tests
existed. The run's seconds are a record in the step's commit body, not a gate
(ADR-0024 §4).

## Consequences

- A YAML, `.config/`, regression-fixture or oracle commit no longer runs the
  whole `fast` profile locally.
- A blend commit's local run drops the non-blending fixtures of the other
  areas, about 3,000 s of test time. Those fixtures still run at the full gate
  and in CI, at 1000 cases.
- `/close-cycle` and `/release` reuse a full gate on the same code: up to
  94 minutes per boundary on a tree already gated. The reuse names the sha it
  reuses in its reply.
- A commit's test half no longer blocks the agent's next commit; a run covering
  several commits can name only the range when it is red.
- Two files of state are added under `arris-verify/` (`gated/<sha>`, the queue
  line); `/retire-plan`'s sweep covers them with the rest.

## Alternatives considered

- **A module map over the test binaries** (the idea's first reading of D).
  Rejected: the corpus is 65% of test time (`arris::corpus`), and the
  properties are a fifth; a map over binaries cuts little.
- **Keep `corpus=all` for `src/blend/*`** (Q6 b). Rejected: the recipe reader
  is exact for the blend case and its guard covers the one way it could fail.
- **Keep ADR-0078 §7's wait.** Rejected for the queue: the wait was the rest
  of a run the agent did not wait for at the previous commit. Coalescing keeps
  one verification at a time and drops the superseded runs.
- **Moving the full gate of `/retire-plan` to CI.** Out of scope: the human
  left it out. The full gate's contents are unchanged.
