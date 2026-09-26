# Idea: publish-survives-index-timeout

- Status: Open
- Raised: 2026-09-27
- Prompt (verbatim from the human): "yes, write that as idea and run that
  manual republish for not published lib so we get v0.3.0 for all crates"
  (following: "Why there is timeout overall?")

## Problem

`release.yml` runs one `cargo publish --workspace --locked`. Cargo uploads
a crate, then waits for it to appear in the crates.io index before it
uploads anything that depends on it. On stable that wait is fixed at 60 s.
`publish.timeout` exists, but only behind `-Z publish-timeout`.

v0.3.0's run (attempt 2, 2026-09-26) uploaded arris-math, geom and topo,
each of which was indexed within about a second. It then uploaded
arris-check, which took more than 60 s to reach the index. Cargo gave up
with "timed out waiting … the registry may have a backlog" and exited 101.
That left ops, mesh, io and arris unpublished and skipped the GitHub
Release step. The upload itself had succeeded; only the wait failed.

Rerunning the job doesn't help: `cargo publish --workspace` refuses when
a version already exists. The release was finished by hand with
`cargo publish -p` from the tag, using a personal token. That puts a
maintainer's token and machine back into a path that trusted publishing
had just removed from it.

It will happen again. The index's lag is crates.io's, and a
publish of eight crates in a chain has seven waits.

## Constraints it runs into

- `.agents/rules/git.md` §Tags: pushing the tag publishes, and the
  `crates-io` environment's reviewer approves the run. A fix must keep
  both and must not add a manual step.
- Trusted publishing: `rust-lang/crates-io-auth-action` mints a
  short-lived token (about 30 minutes per crates.io's docs) and revokes it
  in its post step. Retries with long sleeps have to fit inside that
  window or mint a new token.
- A tag runs the workflow file of the tagged commit. A fix helps only
  releases tagged after it lands. v0.3.0's run stays red.
- ADR-0027: the Release body is the version's `CHANGELOG.md` section.
  Recovery must still reach that step.

## Options

### A — Idempotent publish with a retry loop

Before each attempt, read the sparse index (`index.crates.io`) to list
the workspace's publishable crates that don't yet have the tag's version.
Stop if none are missing. Otherwise run `cargo publish --workspace
--locked --exclude <each one already up>`. On failure, wait and repeat,
up to three attempts.

A rerun of a partial run then becomes safe for free, because it starts by
skipping what's already published. Cargo still decides the order. Cost:
one plan step, a small script under `tools/` that the workflow calls,
plus a dry mode so it can be exercised without uploading. A genuine error
such as a 403 or a build failure also gets retried, but three attempts
bound it and the log shows each one.

### B — Own the order and the wait

A script walks the crates in dependency order. For each one it skips an
existing version, runs `cargo publish -p`, and polls the index itself for
up to about 10 minutes. This gives full control over the wait. It also
duplicates the ordering and the waiting that cargo already does, and a
new crate would have to be added to its order. Cost: one to two steps. It
does more than A for no gain A lacks.

### C — Nightly cargo for the publish step

Run `cargo +nightly -Z publish-timeout publish` with `publish.timeout =
600`. Cost: one step. It publishes the release with a nightly toolchain,
and a partial run still can't be rerun. It fixes this one symptom.

### Do nothing

Each time the index lags, a maintainer finishes the release by hand with
their own token and creates the GitHub Release themselves. That is the
manual, token-bearing path the workflow exists to remove.

## Recommendation

A. It turns the rerun button into the fix, needs no new toolchain, and
leaves ordering to cargo. B wins only if cargo's order ever proves wrong,
and nothing suggests it will. C trades a stable toolchain for a timeout
knob and still leaves reruns broken.

The GitHub Release step should tolerate an existing release, so a rerun
after a late failure also completes. `softprops/action-gh-release`
updates an existing release, so this should need only a check.

## Decision for the human

1. Take A, the idempotent retry loop in a `tools/` script? (Preferred:
   yes. Alternatives: B, C, do nothing.)
2. How many attempts and how long a wait? (Preferred: 3 attempts with
   2 minutes between them, well inside the token's lifetime.)
3. ADR needed? (Preferred: no. It's release tooling, and git.md §Tags
   gets a sentence saying a rerun is safe.)
