---
name: review-pr
description: Review an outside pull request against Arris's rules or land it through the maintainer's verified rebase.
disable-model-invocation: true
argument-hint: <pull request number> [--land]
---

# /review-pr

Use `arris-labs/arris` for GitHub commands. Read the PR description and
comments, confirm it is open against `main`, and record its head SHA, fork,
and branch. Follow [the git rules](../../rules/git.md#pull-requests) and
[ADR-0091](../../../docs/adr/0091-an-outside-pull-request-lands-through-the-verified-rebase.md).

## Review

1. Fetch canonical `main` and save its SHA. Fetch the PR head with
   `git fetch --no-tags https://github.com/arris-labs/arris.git "+refs/pull/$n/head:refs/heads/pr/$n"`
   (where `n` is the PR number). Keep `pr/<n>` as a local snapshot and
   confirm it matches the PR head you recorded. Compute the merge base of
   canonical `main` and that head. Read their three-dot diff, the commits
   after the merge base, and the contributor's check output.
2. Read the [kernel rules](../../rules/kernel.md),
   [docs lifecycle](../../rules/docs-lifecycle.md), [review standards](../../review.md),
   and the ADRs the change reaches. Inspect the workflow and code it runs
   before using **Approve and run** on any CI run awaiting first-time
   contributor approval. Approving a run leaves the PR review for later.
3. Run [pr-check](../../../tools/pr-check.sh) from a clean, detached
   worktree at the PR head, using the script from the maintainer checkout
   and passing `--base` with the fetched canonical `main` SHA. The check
   reports a stale base and still checks the PR's own range. Record the
   findings and CI results for this head. Follow the git rules for long
   checks and CI waits.
4. Run the new regression tests on the PR head and carry just the tests
   and their fixtures to the merge base. Prepare an absolute-path binary
   patch from the PR diff with only the test hunks and data, leaving the
   fix out even when the tests share a source file with it. Run the same
   targeted test command through
   [tools/at-commit.sh](../../../tools/at-commit.sh) at the merge base. In
   that command, apply the patch with `git apply --index` and reverse it
   with `git apply --index --reverse` in an exit trap so the shared
   verification worktree is clean. Confirm the tests pass with the fix
   and fail for the reported bug without it. If the tests cannot run
   against the earlier API, record the obstacle and use a reproduction
   that exercises the bug on both revisions when possible. Review each
   geometry fixture's oracle value and the source of its expectation.
5. Draft the review under these four headings. Give file and line
   references, the evidence, and the change needed for each finding.

   - **What I checked** — head and merge base, commands and results,
     including the regression's failure on the base and its pass on the PR.
   - **Blockers** — correctness defects, missing proof of the fix, and
     violations of project rules that block landing.
   - **Should fix** — improvements for the contributor to consider.
   - **Project conventions** — the rules relevant to this change, with
     references the contributor can follow.

   Show the draft to the maintainer. When they authorize posting, confirm
   the reviewed head is still current and use `gh pr review` with
   `--body-file` and the agreed review disposition.

## Land with `--land`

1. Confirm the reviewed PR head is still current and its blockers are
   resolved. For a code PR, confirm the `full-ci` label is present and
   the full CI run for that head finished with all blocking jobs green,
   including the jobs enabled by the label. Read the advisory semver
   job's verdict and use it when preparing the changelog. Wait for an
   in-flight full run by the git rules before proceeding.
2. Fetch canonical `main` again. Use a clean checkout, bring local `main`
   to that tip by fast-forward if needed, and pause if local `main` has
   unpublished commits. Create a temporary landing branch at the
   reviewed head and rebase onto the fetched tip. Review the rebased diff
   and repeat the relevant checks if conflict resolution changes it.
   Amend the landing branch's last commit with any required documentation
   and ADRs and, for a consumer-facing change, the `CHANGELOG.md` bullet.
   Put an API break under `Unreleased` → `Breaking` with its migration.
   Follow the repo's normal hooks for the amendment.
3. Pin the final landing SHA. Run [tools/verify.sh](../../../tools/verify.sh)
   on it and require `tools/verify.sh --status <sha>` to report `pass`.
   Check that the canonical base and PR head still match the pinned ones
   before pushing. If either moved, reconcile the change and verify the
   resulting tip before attempting the landing again.
4. With maintainer edit rights, push that exact verified SHA to the PR's
   source branch on its fork first. Use an explicit
   `--force-with-lease=refs/heads/<branch>:<reviewed-sha>` with the refspec
   `<verified-sha>:refs/heads/<branch>` so a concurrent contributor update
   stops the push. Then fast-forward local `main` to the verified SHA
   (`git merge --ff-only <verified-sha>`) and push `main` to canonical with
   a normal fast-forward push. Confirm GitHub recorded the PR as merged.
5. If the maintainer lacks permission to update the source branch,
   fast-forward and push `main` in the same way, then close the PR with a
   comment identifying its landed commit IDs. For a rejected lease or an
   unclear push failure, diagnose it before proceeding; lack of edit
   rights is the fallback condition. Report the landed SHA, verification
   result, and PR state. The `--land` invocation authorizes these landing
   pushes and the fallback close.

`$ARGUMENTS`
