# Idea: branch-per-plan-and-changelog

- Status: Accepted 2026-09-26 — E: a `CHANGELOG.md` with an `Unreleased`
  section and `cargo-semver-checks` in CI go to a plan; branch per plan is
  parked until a second writer commits, and becomes a backlog line with
  that trigger when the plan absorbs and deletes this file
- Raised: 2026-09-26
- Prompt (verbatim from the human, said in the plugin CAD's repository for both): "Until now we worked without branching at all, just committing and pushing to main in both [the plugin CAD] and arris, which is bad practice. I suggest change this workflow to trunk-based where each plan is separate feature branch, where retire plan skil finishes by creating mr to main. The only thing we need to think through is versioning: how to connect plans with versions properly. One option that I applied in other projects: we have CHANGELOG.md with changes in all versions and all new features from implemented plans go to Unreleased section in that file and human in arbitrary moment can command AI agent to release - then agent looks at all unreleased changes and derives bump version values and does release commit directly to main without branching."
- Follow-up (the human, same day): branching was meant for future
  contributors who also write code with agents; for the near future there
  is one contributor, so the process stays lean and fast.

## Problem

Two separate things.

**Review and gating.** Every plan step lands on `main` as it is written
(`.agents/rules/git.md`). CI's `oracle`, `wasm` and `parallel` jobs don't
run in the pre-commit hook, so they find a break only after it has landed.
With one writer, that is the whole cost. The problem the human cares
about, contributors whose agents write code, doesn't exist yet. When it
does, it doesn't depend on how the maintainer commits: an outside
contributor can't push to `main` and comes through a fork and a PR, which
`ci.yml` already runs on (`pull_request`).

**Release notes.** `/release` writes the notes at release time, from
commit bodies, into the reply and the GitHub Release. The agent that
writes them isn't the one that did the work, and the notes live only on
GitHub. Four releases (`v0.1.0` to `v0.3.0`) have gone out with no
changelog in the repository, and the plugin CAD has to read GitHub
Releases to learn what broke.

## Constraints it runs into

- `.agents/rules/git.md`: "Commit directly to `main`", branches only for
  experimental plans or on request. Option E leaves this as it is.
- git.md §Tags, §The version: unchanged. Lockstep workspace, `main` on
  `X.Y.Z-dev`, a release is two commits (`release X.Y.Z` and
  `open …-dev`) pushed straight to `main`, the tag on the first, and the
  human tags and approves the `crates-io` environment.
- `/release` step 3 and its Don't list: "no `CHANGELOG.md`: a
  hand-maintained fifth place is the one with no owner". E overturns this,
  so it needs an ADR. The answer to the argument is that every entry has
  an owner (the plan that retires, or the fix that lands), and `/release`
  is the only thing that turns `Unreleased` into a version.
- Cargo packages only files under each crate's directory. `README.md`
  reaches the tarballs through `readme.workspace = true`, and the licence
  files are copied into each crate. A root `CHANGELOG.md` reaches no
  tarball unless it is copied the same way or added through `include`.
- CI takes about three hours (`/release` step 9). Anything that gates a
  merge on it adds three hours of latency per merge.
- Backlog: "`cargo-semver-checks` in CI once the first non-placeholder
  version is published". That condition is met, and the tool gives a
  mechanical check of the semver effect `CHANGELOG.md` declares.

## Options

### A — Branch per plan, PR at retire

Each plan runs on `plan/<slug>` with a draft PR from step 1, pushed after
every step so CI's extra jobs see each step. A ruleset on `main` requires
the CI jobs and linear history, but not up-to-date branches, which would
restart the other open branch's three-hour run on every merge. A
`changes` job lets prose-only PRs skip the heavy jobs, because a required
check that a path filter never starts leaves the PR pending forever. The
agent merges with `gh pr merge --auto --rebase`, and the human can stop
it. Release commits go straight to `main` through an admin bypass, as the
prompt asks: they touch only `Cargo.toml`, `Cargo.lock` and
`CHANGELOG.md`, and the tag's CI run is already the release gate.

Work outside plans (fixes, CI, ideas, backlog) needs short-lived branches
too. A plan file that lives only on its branch breaks the two-active-plans
count, `/work`'s lookup of the active plan and the `docs_refs` test on
`main`, so those need a new rule as well. Cost: about 3 plan steps, and
three hours of CI per merge. With one writer, the only thing it buys is
earlier `oracle`, `wasm` and `parallel` runs.

### B — A, plus per-change changelog fragments

Each PR that a consumer would notice adds `changes/<slug>.md`, with its
semver effect. `/release` merges the fragments into `CHANGELOG.md` and
deletes them. Fragments exist to spare parallel branches a conflict in a
shared `Unreleased` list. Without parallel branches they are a directory
and an assembly step that buy nothing.

### C — Notes written at retire, published only in the GitHub Release

Fragments as in B, pasted into the GitHub Release and deleted. This keeps
`/release`'s no-changelog rule. The history stays only on GitHub, and the
plugin CAD's complaint stands.

### D — release-plz

It writes one changelog line per conventional commit, which is noisy with
one commit per step and can't produce the reader-facing notes `/release`
step 3 asks for. It doesn't know about the `-dev` guard or the
environment reviewer.

### E — Trunk as today, `CHANGELOG.md` with an `Unreleased` section, `cargo-semver-checks`

The human's own versioning scheme, without the branching.
`/retire-plan`, and any `fix` commit a consumer would notice, adds
bullets under `## Unreleased` in `/release` step 3's reader-facing style:
what a consumer can now do, the refusals they will hit, and a
`### Breaking` subsection with the type or signature and the one-line fix.
`/release` renames the section to the version and the date, and derives
the number from it: a non-empty `Breaking` means a minor, anything else a
patch, and a cycle close is always a minor. It cross-checks that against
the commit-body lookup it already runs and against `cargo-semver-checks`,
and stops and asks if they disagree. It also pastes the section into the
GitHub Release. Each crate ships the file the same way it ships the
licences. `cargo-semver-checks` joins CI against the last published
version and retires its backlog line.

Moving to A or B later loses nothing. `CHANGELOG.md`'s format stays, and
only the staging of new entries changes. Cost: about 2 plan steps (ADR and
skills, then CI).

### Do nothing

The notes keep being written late, from commit bodies, and only on
GitHub.

## Recommendation

**E.** It follows the human's versioning scheme, keeps the one-writer
workflow fast, and costs nothing that A or B would later have to undo.
A and B pay three hours of CI per merge, and rules for plan files,
non-plan work and fragments, to prepare for contributors who don't exist
yet. When they arrive they use forks and PRs, which work today. C leaves
the history off the repository. D fights the `-dev` and reviewer design.

What would change my mind: a second writer, whether an outside
contributor or two agents committing at once. Then A's ruleset (with a
bypass for the maintainer), a `CONTRIBUTING.md`, one line in git.md
telling contributors' agents to fork, branch per plan and end
`/retire-plan` in a PR, and the `changes` job make about one plan step,
written against that contributor's actual needs. Fragments (B) come only
if conflicts in `Unreleased` start to hurt.

## Decision for the human

Decided 2026-09-26:

1. Branch per plan with a PR at retire? *Parked* until a second writer
   commits. It becomes a backlog line with that trigger.
2. `CHANGELOG.md` with an `Unreleased` section, written by `/retire-plan`
   and by consumer-visible fixes, released by `/release`? *Yes*, with an
   ADR that overturns `/release`'s "no `CHANGELOG.md`".
3. Fragments? *No*, not while there is one writer.
4. `cargo-semver-checks` in CI now, as a cross-check on the declared
   effect? *Yes*. This retires its backlog line.
5. Does `CHANGELOG.md` ship in each crate's tarball? *Yes*, copied the way
   the licence files are.
