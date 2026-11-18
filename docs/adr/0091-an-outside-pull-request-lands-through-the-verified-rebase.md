# ADR-0091 — An outside pull request lands through the verified rebase

- Status: accepted (2026-11-19)
- Plan: outside contributor workflow (retired)
- Follows: ADR-0078 and ADR-0085 for verification, ADR-0027 for the changelog
- Reference trees read: none

## Context

PR #2 exposed mechanical issues that the contributor could have fixed
before review. It also conflicted on `CHANGELOG.md`. The maintainer's
local gate verifies a specific commit before it reaches `main`. The
GitHub merge button updates `main` without that local verification.

## Decision

1. A contributor works on a branch from a fork, runs
   `tools/pr-check.sh` against upstream `main`, and opens a pull request
   using the template. The PR description proposes the changelog bullet.
   The maintainer owns edits to `CHANGELOG.md` and `docs/adr/` at landing.
2. PR CI runs the cheap jobs and `pr-fast` by default. The `full-ci`
   label enables the full suite, including the jobs that the local gate
   does not run. Before landing a code PR, the maintainer checks that
   full CI passed on GitHub's merge commit with the pinned canonical
   `main` tip and the reviewed PR head as its parents. The PR semver job
   reports its verdict without blocking because any required `Breaking`
   bullet is written at landing.
3. The maintainer reviews with `/review-pr <n>`, checks the PR against
   the project rules and runs its regression tests on the merge base to
   confirm they fail without the fix. The maintainer reads the semver
   verdict when preparing the changelog bullet.
4. To land, the maintainer uses `/review-pr <n> --land` to rebase onto
   the pinned `main`, include the changelog bullet and any required docs,
   run the static half on the final rebased tree against that base, and
   verify the same tip with `tools/verify.sh`. With edit
   rights, the maintainer pushes the verified commits to the PR head,
   then fast-forwards and pushes `main`, so GitHub records the PR as
   merged. Without edit rights, the maintainer pushes the verified tip
   to `main` and closes the PR with the landed commit ids.

## Consequences

- Maintainer-authored work continues to commit directly to `main`.
  For both paths, every pushed tip of `main` is verified.
- Contributors can check the same mechanical rules before opening a PR
  that the maintainer will use in review. Keeping changelog and ADR edits
  at landing avoids conflicts on those paths in long-lived branches.
- The maintainer chooses when to spend full CI on an outside code change.
  GitHub settings own the label and protection against force pushes and
  deletion of `main`.

## Alternatives considered

- **Use GitHub's merge button.** It cannot ensure that the tip that
  arrives on `main` has passed the local verification in ADR-0078.
- **Run full CI on every PR update.** The mechanical check and reached
  tests give the contributor feedback sooner, with the full suite
  required before a code PR lands.
- **Have contributors edit the changelog or add fragments.** Direct
  edits repeat the conflict from PR #2. Fragments add a file format and
  a collection step where the maintainer can use the proposed bullet in
  the PR description.
