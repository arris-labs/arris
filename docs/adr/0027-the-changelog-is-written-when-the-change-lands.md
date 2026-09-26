# ADR-0027 — The changelog is written when the change lands, and a break is announced before it can be released

- Status: accepted (2026-09-26)
- Plan: `changelog-and-semver-checks`
- Follows: `.agents/rules/git.md` §Tags, §The version; the `/release`
  skill's "no `CHANGELOG.md`" rule, which this overturns

## Context

Four versions have gone out (0.1.0 to 0.3.0). The notes for each were
written by `/release` at release time, from commit bodies, into the reply,
for the human to paste into the GitHub Release. None was pasted: every
GitHub Release body is GitHub's generated "Full Changelog" link and
nothing else. What broke in a release is recorded only in the body of its
`chore(arris): release X.Y.Z` commit, which a consumer reading crates.io
never sees. The first consumer learns about a break by failing to compile.

`/release` banned a `CHANGELOG.md` because "a hand-maintained fifth place
is the one with no owner". That argument holds for a file written at
release time. It fails once each entry is written by the commit that makes
the change, which is the one moment someone knows what the change means to
a consumer.

The version number already comes from a lookup: every change to a public
type or signature is named in its commit body (`.agents/rules/git.md`).
Nothing checks that lookup, so a missing line in a body would release a
break as a patch.

## Decision

1. **`CHANGELOG.md` at the root**, newest first. At the top is
   `## Unreleased`, then one `## X.Y.Z — YYYY-MM-DD` section per release.
   A section has bullets for a reader who has read only `README.md`: what
   they can now do, the refusals they will hit, and a `### Breaking` list
   naming each broken type or signature with its one-line fix. No ADR
   numbers, plan slugs or fixture names.
2. **An entry is written in the commit that makes the change.** For a
   plan, that is its `/retire-plan` commit, which writes the plan's
   bullets. For a standalone commit a consumer would notice (a fix, a
   public API change), it is that commit. A commit that changes a public
   type or signature adds its `Breaking` bullet at the same time as it
   names the change in its body.
3. **`/release` is the only thing that turns `Unreleased` into a
   version.** The number follows from the file: a cycle's close, or a
   non-empty `Breaking`, means a minor bump, and anything else a patch.
   `/release` cross-checks it against the commit-body lookup and
   `tools/semver-gate.sh`, and stops and asks when they disagree. The
   GitHub Release body is the version's section, extracted by
   `release.yml`. Nobody pastes anything.
4. **A break can't reach a release unannounced.** CI's `semver` job runs
   `cargo-semver-checks` on the published crates against the last `v*`
   tag, as if the next release were a patch. A break fails the job unless
   `Unreleased` has a `Breaking` bullet. A "minor" verdict (a deprecation,
   say) breaks nothing, and pre-1.0 a patch carries it.
5. **The file ships with the crates.** Each published crate holds a
   `CHANGELOG.md` symlink to the root file, as it does for the licences,
   so `cargo package` puts the file in every tarball.

## Consequences

- The notes are written by whoever knows the change, when they know it,
  and a consumer reads them in the crate, on GitHub or in the repository.
- A release no longer writes notes. It renames a heading, derives the
  number from the section and publishes the section.
- The semver check turns a missing `Breaking` bullet from a mistake a
  consumer finds into a red CI job.
- The trunk workflow is unchanged. With one writer, the `Unreleased` list
  can't conflict. When a second writer arrives, branch per plan (a backlog
  line) comes first, and changelog fragments only if conflicts in
  `Unreleased` start to hurt. The file's format survives both.

## Alternatives considered

- **Notes only in the reply and the GitHub Release** (the rule until now).
  They were never pasted, and that is the evidence against it.
- **Branch per plan with a PR at retire.** It gates a merge on three hours
  of CI and needs new rules for plan files and non-plan work, all to
  prepare for contributors who don't exist yet. Outside contributors come
  through forks and PRs, which work today. Parked as a backlog line.
- **Per-change fragments (`changes/*.md`) merged at release.** They spare
  parallel branches a conflict in `Unreleased`, and there are no parallel
  branches.
- **Fragments published only in the GitHub Release.** The history would
  stay out of the crate and the repository.
- **release-plz.** It writes one line per commit, which is noisy with one
  commit per plan step, and it can't write for a consumer. It doesn't know
  about the `-dev` guard or the environment reviewer.
