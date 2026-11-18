---
name: contribute
description: Prepare a contribution from a fork, check the committed topic branch, and draft a pull request against canonical main.
argument-hint: <change to contribute>
---

# /contribute

1. Read [Contributing from a fork](../../../AGENTS.md#contributing-from-a-fork),
   the setup and project rules it points to, and [CONTRIBUTING.md](../../../CONTRIBUTING.md).
   Confirm that the push remote points to the contributor's fork. Fetch
   canonical `main` with
   `git fetch --no-tags https://github.com/arris-labs/arris.git refs/heads/main`
   and start a topic branch from `FETCH_HEAD`. For an existing topic branch,
   rebase it onto the freshly fetched tip. Keep the work on that branch and
   follow this contributor workflow in place of the maintainer's `/plan`
   and `/work` pipeline.
2. Make focused commits with a dotted author email domain and subjects in
   the form `type(scope): lower-case summary` without a trailing period.
   Keep the history linear and omit `Co-Authored-By`, `Claude-Session`,
   and `Generated with` trailers. For a geometry fix, include a regression
   fixture and its oracle value. Put new STEP files under `tests/fixtures/`.
   Describe the proposed changelog wording and any decision that needs an
   ADR in the PR body. The maintainer writes the changelog and ADR edits
   at landing.
3. Run [`tools/pr-check.sh`](../../../tools/pr-check.sh) from the committed
   topic branch tip. Fix its findings and rerun it on the resulting tip.
4. Draft the body from the [pull request template](../../../.github/pull_request_template.md).
   Explain what changed and why, give the fixture and its expected oracle
   value or the checks for a non-geometric change, propose the changelog
   bullet if applicable, and include the final check output. With the
   contributor's authorization, push only the topic branch to their fork
   and open the PR against `arris-labs/arris:main`.

`$ARGUMENTS`
