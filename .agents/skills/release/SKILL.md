---
name: release
description: Cut a release of the workspace to crates.io — pick the version from the log rather than by feel, write the user-facing release notes, bump the version and its pins, prove the workspace still packages, and hand the human the exact tag command. Use when the human says "release", "cut a version", "publish", "ship 0.2", or at the end of /close-cycle. Never tags, never pushes, never runs cargo publish.
argument-hint: <nothing, or a version to force, e.g. 0.2.0>
---

# /release — the tag is the release

Nothing is published from a laptop. The human tags a commit on `main` and
pushes the tag; `.github/workflows/release.yml` runs `cargo publish
--workspace` behind the `crates-io` environment's reviewer and opens a
GitHub Release. This skill prepares the commit that tag will point at.

The version scheme, the `-dev` convention and who does what are
`.agents/rules/git.md` §Tags. Read it first; this skill executes it.

## Do

1. **Check the tree.** On `main`, clean, and the hook's gate green: `cargo
   fmt --check`, `cargo clippy --workspace --all-targets -- -D warnings`,
   `cargo nextest run --workspace`, `cargo test --workspace --doc`,
   `RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps`,
   `tools/check-layers.sh`, and the last CI run on `main` green in every
   job — the hook runs none of CI's `oracle`, `wasm` or `parallel` jobs.
   A release is not the place to discover a red suite. An active plan in `docs/plans/` does not block a patch release —
   say in the reply which plans are open, so the human knows what is
   half-landed.
2. **Pick the version from the log, and show your evidence.** `git log
   $(git describe --tags --match 'v*' --abbrev=0)..HEAD` — or the whole
   log before the first release. Read the *bodies*: every change to a
   public type or signature is named in one (`.agents/rules/git.md`).
   - closing a cycle → minor;
   - otherwise patch, unless a body names a public API change → minor.

   State the decision as a derivation in the reply — the commits that
   forced it, by hash — never as an opinion. An argument to this skill
   overrides the number, and then say in the reply what the log would have
   picked instead. If the log is ambiguous, stop and ask.
3. **Write the release notes into the reply.** Three to five bullets, and
   a **Breaking** list under them when the minor moved:
   - what a consumer can now *do* that they could not before. Not what was
     built — `Surface::EllipticCylinder` is not a change, "a sketch with
     an elliptic arc extrudes into a solid" is;
   - the refusals a consumer will hit, because that is what an issue gets
     filed about;
   - the breaking list names the type or signature and the one-line fix,
     because pre-1.0 the minor is the only warning a consumer gets;
   - no ADR numbers, no plan slugs, no fixture names. Someone who has read
     nothing but `README.md` is the reader.

   In the reply and in no file. There is no `CHANGELOG.md`: what landed
   already lives in the git log, the roadmap status lines, the ADRs and
   the GitHub Release, and a hand-maintained fifth place is the one with
   no owner.
4. **Bump.** `[workspace.package].version` to the number, `-dev` dropped,
   and the seven internal `version = "=X.Y.Z-dev"` requirements in
   `[workspace.dependencies]` with it — eight places, one edit, and a
   `cargo check` that fails if one was missed. `arris-debug` has no
   version to bump: it is path-only and never published.
5. **Prove it publishes.** `cargo package --workspace` on the clean tree:
   every crate packages and its verifying build passes. A crate that fails
   here fails in the workflow after some of the others are already on
   crates.io, where a version can be yanked but never replaced.
6. **Commit** as `chore(arris): release X.Y.Z`, body giving the derivation
   from step 2 — the commits that set the number — and the breaking list.
   When `/close-cycle` called this skill, that skill's `docs: close
   <cycle>` commit comes first and this one after it.
7. **Open the next `-dev` in its own commit**, right away: `chore(arris):
   open X.(Y+1).0-dev`, same eight places. `main` never sits on a released
   version, and the human's tag goes on the commit from step 6, not this
   one. Tagging this one by mistake is safe — `release.yml` refuses a
   pre-release version before it uploads anything.
8. **Hand the human the tag**, with the exact commands and the release
   notes to paste:

   ```sh
   git tag vX.Y.Z <the sha from step 6>
   git push origin main
   git push origin vX.Y.Z
   ```

   Then: they approve the `crates-io` environment when GitHub asks, with
   the tag's CI run in front of them. Remind them of that — the approval
   is the last checkpoint before a version exists forever.
9. **Once they have pushed, cancel `main`'s duplicate CI run.** The two
   pushes start two full runs, about three hours each: the tag's, which
   is the release gate, and `main`'s tip, which is the same code plus
   the `-dev` bump. Find them with `gh run list --workflow CI --limit 4`
   (the branch column says `vX.Y.Z` or `main`). Cancel `main`'s with
   `gh run cancel <id>` **only if** `git diff --name-only vX.Y.Z main`
   lists nothing but `Cargo.toml`, `Cargo.lock` (the `-dev` bump),
   `*.md`, `.agents/` and `.githooks/`, none of which CI runs.
   If anything else changed, leave the run alone: that is code the tag's
   run never tested. Never cancel the tag's run. Say in the reply which run
   was cancelled and why.

## When a publish fails partway

crates.io rate-limits **new** crates: a burst of five, then one per ten
minutes. (A new version of a crate that already exists is one a minute,
burst thirty — so only a release that introduces crates hits this.) The
first release of a workspace this size therefore stops with a `429` after
five of them, with some crates published and some not.

That is recoverable and nothing is at risk: `cargo publish --workspace`
warns `already exists on crates.io index` for each crate that went up and
uploads only the rest. Wait for the window the error names, then re-run
the failed job — `gh run rerun <id> --failed` — and repeat until it is
through. Each re-run asks the environment's reviewer again.

The workflow that runs is the one **at the tag**, so fixing `release.yml`
on `main` does not change a re-run, and the tag never moves to pick it up.
A release blocked by something the workflow itself gets wrong is a fix on
`main` and a new version, never a moved tag.

## Don't

- Don't tag, don't push, don't run `cargo publish`. All three are the
  human's, and the last one would skip the gate and the reviewer both.
- Don't write a `CHANGELOG.md`, and don't put the version in a doc, a
  README or a rustdoc line. `Cargo.toml` is its only home.
- Don't bump the minor "to be safe". Pre-1.0 the minor is a consumer's
  signal that its code will not compile; spending it on a release that
  breaks nothing makes the signal worthless.
- Don't release with a red suite, a dirty tree, or an unreviewed fixture
  expectation. A yanked version is still a version.

`$ARGUMENTS`
