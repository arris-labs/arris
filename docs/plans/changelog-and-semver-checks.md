# Plan: changelog-and-semver-checks

- Started: 2026-09-26
- Milestone: none — process work beside the cycles (docs/ROADMAP.md §Beside the cycles)
- Idea: docs/ideas/branch-per-plan-and-changelog.md (absorbed; option E)
- Idea (verbatim from the human): "One option that I applied in other projects: we have CHANGELOG.md with changes in all versions and all new features from implemented plans go to Unreleased section in that file and human in arbitrary moment can command AI agent to release - then agent looks at all unreleased changes and derives bump version values and does release commit directly to main without branching."

## Goal

The repository has a `CHANGELOG.md`, and each published crate's tarball
ships a copy. A change a consumer would notice adds its bullets under
`## Unreleased` in the same commit that makes the change: the
`/retire-plan` commit for a plan, or the fix commit for a standalone fix.
Every bullet is written for a reader who has read only `README.md`.
`/release` turns that section into `## X.Y.Z — date`. It derives the
number from the section (a non-empty `### Breaking`, or a closed cycle,
means a minor bump) and cross-checks it against the commit-body lookup it
already runs. The GitHub Release body is that same section.
`cargo-semver-checks` runs in CI against the last `v*` tag. It fails when
it finds a breaking change and `Unreleased` lists none under `Breaking`,
so a break can't slip into a patch release unannounced. The trunk
workflow is unchanged: commits to `main`, and releases pushed straight to
it.

## Non-goals

- Branch per plan, PRs, a ruleset on `main`, `CONTRIBUTING.md`: parked
  until a second writer commits (backlog line).
- Changelog fragments (`changes/*.md`): only if concurrent branches
  arrive and conflicts in `Unreleased` start to hurt.
- release-plz or any tool that writes the changelog from commits.
- Changing the version scheme, the `-dev` guard, the environment
  reviewer or who tags (`.agents/rules/git.md` §Tags).
- A changelog for `arris-debug`, which is never published.

## Design deltas

- **ADR-0027** (new): the changelog lives in the repository. It has an
  `Unreleased` section. Every entry has an owner: the commit that makes
  the consumer-visible change. `/release` is the only thing that turns
  `Unreleased` into a version, and the number comes from `Breaking`,
  cross-checked by the commit-body lookup and by `cargo-semver-checks`.
  This overturns `/release`'s "no `CHANGELOG.md`" rule and `/close-cycle`'s
  "Don't create … a CHANGELOG". The ADR names the rejected options A–D
  from the absorbed idea in one line each.
- **`CHANGELOG.md`** (new, root): `## Unreleased` with `### Breaking` and
  `### Added / Changed / Fixed` as needed. The released sections are
  backfilled for `0.1.0`, `0.1.1`, `0.2.0` and `0.3.0` from the release
  commits' bodies and the roadmap status lines.
- **A `CHANGELOG.md` symlink** in each of the eight published crates, the
  same way the licence files get into the tarballs.
- **`.agents/rules/git.md`**:
  - a new §The changelog, stating who writes what and when;
  - §The version: the "lookup, not a judgement" becomes the `Breaking`
    subsection plus the commit bodies;
  - §Message format: a consumer-visible commit touches `CHANGELOG.md`.
- **Skills**:
  - `/release`: step 2 derives the number from `Unreleased`, step 3 moves
    the notes into the file, and the Don't list drops the ban.
  - `/retire-plan`: a new step writes the plan's bullets.
  - `/close-cycle`: its Don't list drops "a CHANGELOG".
- **`.github/workflows/release.yml`**: the GitHub Release body comes from
  the version's section of `CHANGELOG.md` (`body_path`), not from
  `generate_release_notes`.
- **`.github/workflows/ci.yml`**:
  - a new `semver` job running `tools/semver-gate.sh`;
  - `CHANGELOG.md` joins the prose-only path filter.
- **`tools/semver-gate.sh`** (new): runs `cargo semver-checks` over the
  published crates, with the baseline set to the last `v*` tag and the
  release type set to patch. On failure, it fails only if `Unreleased`'s
  `### Breaking` is empty. CI and `/release` step 2 share it.
- No public Rust type or signature changes.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — `tools/semver-gate.sh` and the `semver` CI job.
  This comes first because it is the unknown: whether `cargo-semver-checks`
  runs cleanly on this workspace (rustdoc JSON of eight crates, the
  `publish = false` crate skipped, `-dev` versions against a published
  baseline), and how long it takes. Prove it three ways, on scratch
  changes that are not committed:
  - `main` passes;
  - removing a public item fails the check while `Breaking` is empty;
  - the same change passes once `Breaking` has a bullet.

  Record the job's runtime in the commit body. (The backlog line for
  `cargo-semver-checks` was removed when this plan was written.)
- [x] Step 2 **[1]** — ADR-0027 and its line in `docs/adr/README.md`.
  Write `CHANGELOG.md` with an empty `Unreleased` (nothing
  consumer-visible has landed since `v0.3.0`) and the backfilled sections
  for 0.1.0–0.3.0, in `/release` step 3's reader-facing style. Add the
  eight crate symlinks. A test in `crates/arris/tests/docs_refs.rs`
  checks:
  - the file opens with `## Unreleased`;
  - every released heading names a version that has a `v*` tag in the
    format `## X.Y.Z — YYYY-MM-DD`;
  - every published crate has the symlink.

  `cargo package --workspace --list` shows `CHANGELOG.md` in each of the
  eight crates.
- [x] Step 3 **[1]** — The process. Update `.agents/rules/git.md`
  (§The changelog, §The version, §Message format) and the `/release`,
  `/retire-plan` and `/close-cycle` skills as the design deltas say. In
  `release.yml`, extract the tag's section into a file and pass it as
  `body_path`, with a step that fails if the section is missing. Add
  `CHANGELOG.md` to `ci.yml`'s prose path filter.

## Acceptance

- `tools/semver-gate.sh` passes on `main`. On a scratch change that
  removes a public item it fails while `Breaking` is empty, and passes
  once `Breaking` has a bullet. Both results are recorded in step 1's
  commit body.
- The CI run on `main` after step 3 is green, `semver` job included.
- The `docs_refs` test holds `CHANGELOG.md` to its format and checks the
  eight symlinks.
- `cargo package --workspace --list` lists `CHANGELOG.md` for every
  published crate.
- A dry read of `/release` against the current log derives `0.3.1`, or
  says it has nothing to release, from an empty `Unreleased`, and agrees
  with the commit-body lookup.

## Docs to update on completion

- `docs/ROADMAP.md` §Beside the cycles: no line. This is process work,
  and the ADR and git.md hold it.
- `AGENTS.md` "Rules that are not derivable from the code": the git
  bullet adds "a consumer-visible change adds its `CHANGELOG.md` bullet in
  the same commit".
- `docs/README.md` document lifecycle table: a row for `CHANGELOG.md`
  (owner: the commit that makes the change; `/release` versions it).
- `SEED.md` §9, if it says anything about release notes or a changelog:
  check it, and point it at ADR-0027 rather than editing a kickoff
  decision.

## Open questions

None. The idea's decisions (2026-09-26) settled the process. Anything in
step 1 that depends on measurement is the agent's call, recorded in the
commit body: the job's runtime, and whether `semver` runs on every push or
only where Rust sources changed.
