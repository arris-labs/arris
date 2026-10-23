# ADR-0069 — A version-only push rests on the nearest green commit

- Status: accepted (2026-10-23)
- Plan: `ci-reuse-gate` steps 1–4 (the script, the `reuse` and `reused`
  jobs, the sharded `parallel` job); the first measured run is step 5
- Follows: ADR-0032 (the lean gate), ADR-0034 (the release publishes the
  workspace and the wheel in lockstep)

## Context

The suite is about 22 runner-hours and 5 hours of wall clock per CI run.
Releasing 0.9.1 took three such runs for two commits that changed no code:
the retired plan's tip, the release commit (version strings and the
changelog heading) and the tag on it. The `-dev` commit after a release is
the same again. No test can change its verdict across such a difference, so
the runs buy nothing.

## Decision

1. **A push rests on a base.** `tools/ci-reuse.sh <head>` takes as base
   `<head>` itself or the nearest of its first 20 first-parent ancestors
   that has a successful `ci.yml` push run on `main`. It prints `skip=true`
   only when `git diff base head` lies inside an allow-list: `CHANGELOG.md`,
   `AGENTS.md`, `SEED.md`, `.agents/**`, `docs/**` except `ROADMAP.md`,
   `DATA-MODEL.md` and `plans/**` (the set `ci.yml`'s path filter already
   ignores, because tests read the three it keeps), `Cargo.lock` lines that
   are all `version = "…"`, and a root `Cargo.toml` equal to the base's once
   the workspace version and the internal crates' exact pins are normalised.
2. **It fails closed.** Anything else, `.github/**` and `tools/**` included,
   no green ancestor, a lookup that errors, and every pull request are
   `skip=false`: the full run, as before. A mistaken `false` costs hours; a
   mistaken `true` would be a missed test, so the list names what is safe
   and nothing is inferred.
3. **What still runs.** `fmt`, `clippy`, `layers`, `doc`,
   `no-default-features` and `semver` always. `reused`, on a skip, runs
   `cargo package --workspace --exclude arris-py --locked` and the
   `docs_refs` test, which are what a version bump can break, and writes the
   base commit to the step summary. `test`, `parallel`, `python`, `oracle`,
   `wasm`, `rerun` and `wheels` are skipped.
4. **The release is unchanged.** `release.yml` does not depend on CI; its
   own wheel build still runs on every platform, since the wheel carries the
   release version. The reviewer reads the tag's run, which is now short and
   names the green commit it rests on.
5. **`parallel` is sharded** `hash:N/4` like `test`, so the longest job no
   longer sets the run's wall time. The count is settled by the first
   measured run (step 5): no shard above about 1.5× the median.

## Consequences

- A release costs one full run (the plan's tip) and short runs after it.
- `tools/ci-reuse.sh --self-test` runs in the `reuse` job; the hook does not
  run tool self-tests.
- A new file kind that is safe to skip is an allow-list edit with its
  self-test case, in `tools/`, which itself forces a full run.
