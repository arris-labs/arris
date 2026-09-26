# Git: trunk-based, always green

- `main` is the trunk and is always green: `cargo fmt --check`, `cargo clippy
  --workspace --all-targets -- -D warnings`, `cargo test --workspace` and
  `cargo doc --workspace --no-deps` with `-D warnings` pass on every commit.
  The `.githooks/pre-commit` hook enforces it; never bypass it with
  `--no-verify`. It sizes the gate to what is staged: prose alone runs the
  tests that read docs, a version bump alone runs `cargo check`, and
  anything else runs all of it. A commit that could change what the suite
  tests always gets the whole suite.
- Commit **directly to `main`**. A plan step is the unit of work and the unit
  of commit: finish the step, run the checks, tick the box, commit.
- Branch only when a plan is experimental enough that throwing it away is a
  real outcome, or the human asked to review before it lands. Then:
  `plan/<slug>`, rebased onto `main`, fast-forwarded in (`git merge --ff-only`),
  deleted after. No merge commits, no long-lived branches.
- Never rewrite `main`: no `--amend` of a pushed commit, no force-push, no
  rebase of anything already on `main`.
- Never push unless asked. Commits are the agent's; pushes are the human's.
- Stage deliberately: read `git status` and `git diff`; no blind `git add -A`.
  A changed fixture expectation (`tests/fixtures/**/*.json`) is only staged
  with a matching intentional geometry change, and the commit message says
  `fixtures:` and why. Oracle values do not drift; if they changed, the
  fixture or the oracle script changed, and the body says which.

## Message format

```
type(scope): imperative summary, lower case, no period  (plans/<slug> step N)

Why this change, not what — the diff already says what. Reference ADRs
(ADR-0004) and docs (docs/DATA-MODEL.md §Tolerances) that justify or were
updated by it. A change to a public type or signature names it here.
```

- `type`: `feat`, `fix`, `refactor`, `perf`, `test`, `docs`, `chore`, `build`,
  `ci`, `fixtures`.
- `scope`: crate short name(s) — `math`, `geom`, `topo`, `check`, `ops`,
  `mesh`, `io`, `debug`, `arris` — or `docs`, `plan`, `adr`, `tools`.
  Several: `feat(geom,ops): …`.
- The `(plans/<slug> step N)` suffix is present on every commit that executes
  a plan step. Plan retirement commits are `docs: retire plan <slug>`.
- Docs change in the **same commit** as the code that changes behaviour.
  A commit that only updates docs to match existing code is `docs(sync): …`.
- A commit that changes a public type or signature adds its bullet under
  `CHANGELOG.md`'s `### Breaking` in the same commit, as well as naming
  the change in its body (§The changelog).
## Tags

- Milestones: `m0`, `m1`, … on the commit that retires the milestone's last
  plan and passes its acceptance corpus.
- Releases: `vX.Y.Z` SemVer, on `main`, created by the human. Pushing the
  tag is what publishes: `.github/workflows/release.yml` runs `cargo
  publish --workspace` and opens a GitHub Release. CI runs on the tag as
  well, and the publish waits for the `crates-io` environment's reviewer,
  so the human approves it with that run's result in front of them. Every
  crate but `arris-debug` goes up; that one is `publish = false`.

### The version

- **`main` between releases carries the next version with `-dev`**:
  `[workspace.package].version = "0.2.0-dev"`, and each internal crate in
  `[workspace.dependencies]` pinned to exactly it, `version =
  "=0.2.0-dev"`. The eight crates are published in lockstep and only ever
  make sense as a set, so the pin is exact — and it is also the guard: a
  `[workspace.package]` bump that misses the requirements fails the next
  `cargo check`, and `release.yml` refuses to publish a pre-release
  version at all. A release is therefore impossible without the deliberate
  commit that drops `-dev` from all eight places at once — the workspace
  version and the seven requirements. `arris-debug` carries no version
  because it is never published.
- **Pre-1.0, Cargo reads `0.y.z` as `y` breaking, `z` compatible**, so:
  - **closing a roadmap cycle bumps the minor.** A cycle here always
    breaks the API — a new surface or curve kind makes every exhaustive
    `match` fail to compile, which is the point (`.agents/rules/kernel.md`
    §API). Cycle Cn releases `0.n.0` as long as that lines up; it is a
    convention, not a law, and the cycle's status line names the tag it
    actually got.
  - **a release between cycles bumps the patch** — the case that exists
    because a consumer is waiting on a fix. Unless `CHANGELOG.md`'s
    `Unreleased` has a `### Breaking` bullet: then it is a minor. That is
    a lookup, not a judgement. Every such change also names itself in its
    commit body, and `tools/semver-gate.sh` fails CI on a break that
    `Breaking` doesn't list, so the three sources check one another.
- The version lives in `Cargo.toml`, and the released ones in
  `CHANGELOG.md`'s headings. No other doc, README or rustdoc line states
  one, so nothing can go stale.
- `/release` cuts one. It picks the number from `CHANGELOG.md`, turns
  `Unreleased` into the version's section, bumps, proves the workspace
  still packages, and hands the human the exact tag command.
  `/close-cycle` ends by calling it.

### The changelog

`CHANGELOG.md` is what a consumer reads to learn what changed and what
broke (ADR-0027). Each published crate ships it through a symlink.

- **An entry is written in the commit that makes the change.** A plan's
  bullets are written by its `/retire-plan` commit. A standalone commit a
  consumer would notice (a fix, a public API change) writes its own. A
  commit no consumer can see (process, tooling, tests, docs) writes
  nothing.
- **Bullets go under `## Unreleased`.** They say what a consumer can now
  do and which refusals they will hit, for a reader who has read only
  `README.md`: no ADR numbers, plan slugs or fixture names. `### Breaking`
  names each broken type or signature with its one-line fix.
- **Only `/release` turns `Unreleased` into a version.** The GitHub Release
  body is that section, taken by `release.yml`.

## What the agent does without asking

While executing a plan: run checks, commit each step, tick boxes, update the
plan file. Anything else that touches history — branching, tagging, pushing,
resetting, rewriting, publishing to crates.io — is asked first.
