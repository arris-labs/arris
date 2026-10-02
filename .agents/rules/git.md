# Git: trunk-based, always green

- `main` is always green at a pushed tip, in tiers (ADR-0032). **Every
  commit** passes the `.githooks/pre-commit` hook; never bypass it with
  `--no-verify`. It sizes the gate to what is staged (`tools/gate.sh`):
  prose alone runs the tests that read docs, a version bump alone runs
  `cargo check`, and anything else runs the layer rule, `cargo fmt --check`,
  `cargo clippy -D warnings`, the doctests and the `fast` nextest profile
  at 32 cases over the crates and corpus areas the paths reach — the whole
  profile for a path it cannot place — and `cargo doc` only when a doc
  comment changed. `ARRIS_GATE=full` runs every test at 256 cases. **Every
  plan** runs the full profile once, at `/retire-plan`; **CI** runs it at
  1000 cases with `real_*`, nightly deeper, and a release waits on CI's run
  of its tag. The human reads CI before a push that matters.
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
  crate but `arris-debug` and `arris-py` goes up; those two are
  `publish = false`.
- **The same tag publishes the Python package** `arris` to PyPI, in
  lockstep (ADR-0034 §3): `release.yml`'s `pypi-build` builds the wheel and
  the sdist and checks them with `twine`, and `pypi` uploads them behind its
  own `pypi` environment reviewer, by trusted publishing. The package's
  version is the workspace's in PEP 440 (`0.5.0-dev` is `0.5.0.dev0`, a
  release is `0.5.0`), so the bump that drops `-dev` is the one commit for
  both, and the build refuses a pre-release as the crates job does. A
  release therefore has two approvals, `crates-io` and `pypi`, and the
  human gives each with the tag's CI run, its `python` job included, in
  front of them. The platforms of the wheel are the human's call.

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
