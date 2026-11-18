# Plan: contributor-flow

- Started: 2026-11-19
- Milestone: none (tooling beside C12, docs/ROADMAP.md)
- Idea: verbatim from the human, accepting the recommendation given after
  PR #2's review: "I agree with all, apply it" — the maintainer keeps
  trunk-based pushes to `main`; outside pull requests get a contributor
  path, a mechanical check, a review skill, a landing through the local
  verification, and a lighter PR CI with the full suite on a label.

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
A contributor's agent, reading `AGENTS.md` from a fork, works on a branch,
runs one script that catches every mechanical blocker PR #2's review listed
(hooks, a stale base, commit format, trailers, author, test data placement,
fmt, clippy, the tests the change reaches) and opens a pull request from a
template. A pull request runs a light CI by default and the whole suite
when the maintainer labels it `full-ci`. The maintainer reviews with
`/review-pr <n>` (the fix's tests shown failing on the merge base, the
rules read, the review in one format) and lands with `/review-pr <n>
--land`: rebased locally, the changelog bullet and docs written by the
maintainer, verified by `tools/verify.sh`, pushed so GitHub reads the pull
request as merged. `main` still only ever receives verified tips.

## Non-goals
- Pull requests for the maintainer's own work: plans keep committing to
  `main` (`.agents/rules/git.md`).
- Required reviews, CODEOWNERS, a merge queue, a CLA.
- Branch protection and creating the `full-ci` label: GitHub settings the
  human changes (the reply says which).
- Changelog fragments: a contributor leaves `CHANGELOG.md` to the landing.

## Design deltas
- **ADR-0091, an outside pull request lands through the maintainer's
  verified rebase.** The merge button is never used: it puts an unverified
  commit on `main` (ADR-0078). Contributors do not edit `CHANGELOG.md` or
  `docs/adr/`; the PR description carries the changelog bullet and the
  landing writes both, so a long-lived branch does not conflict on them.
  The PR's `semver` job reports a break without failing (the bullet is not
  in the branch yet); the landing reads its verdict. CI on a pull request
  runs the cheap jobs plus `pr-fast` by default, everything with the
  `full-ci` label, which a code PR carries before it lands (the `python`,
  `wasm` and `parallel` jobs run nowhere locally).
- **`.agents/rules/git.md` §Pull requests** (new): the rule above in five
  lines, and the landing's push (to the PR head, then `main`
  fast-forwarded) named as the one push a `--land` is the ask for.
- **`tools/pr-check.sh [--ci] [--base <rev>]`** (new): the base is
  `upstream` main fetched from the canonical repository (`--base` names
  another; `--ci` takes it as given and skips the local-only checks). Fails,
  each finding on one line: hooks path not `.githooks` (local); the branch
  not containing the base's tip (local); a merge commit; a subject not
  `type(scope): lower-case summary` without a period; a `Co-Authored-By`,
  `Claude-Session` or `Generated with` trailer; an author email without a
  domain dot or at `localhost`; a change to `CHANGELOG.md` or `docs/adr/`;
  an added `*.step`/`*.stp` outside `tests/fixtures/`. Then layers,
  tolerance literals (`--diff <base>`), fmt, clippy (local), and
  `tools/test-half.sh <base> HEAD`. `tools/pr-check-test.sh` holds the
  commit and file rules on scratch repositories; `tools/gate.sh` runs it
  when the script changes.
- **CI** (`ci.yml`, `wheels.yml`): `pull_request` takes `labeled` too;
  `reuse` outputs `light` (a PR without `full-ci`); the eight heavy jobs
  skip when light; a `pr-fast` job runs `tools/pr-check.sh --ci` with the
  oracle environment and `NEXTEST_TEST_THREADS=num-cpus`; `semver` is
  `continue-on-error` on a pull request; `wheels.yml` drops its own PR
  trigger and `ci.yml`'s `wheels` job also runs on a `full-ci` PR.
- **Skills**: `contribute` (a contributor's: branch, check, PR body from
  the template, never `/plan`, `/work` or a push to `main`); `review-pr`
  (the maintainer's, `disable-model-invocation`), the review and the
  landing. `.github/pull_request_template.md` (new): what and why, the
  fixture and its oracle value, the changelog bullet, `pr-check` output.
- **GLOSSARY.md**: *Land* (a pull request) — taking it onto `main` through
  the maintainer's rebase and verification; _Avoid_: merge (the button).

## Steps
Complexity grades; a step's grade is its tier (`.agents/tiers.md`):
**[1]** routine; **[2]** careful; **[3]** unproven. No step needs a push:
the workflow step is held by actionlint here and by the next pull request's
run after the human pushes.

- [x] Step 1 **[1]** — ADR-0091 and its index row; `.agents/rules/git.md`
  §Pull requests; *Land* in `GLOSSARY.md`. Test: the docs tests.
- [x] Step 2 **[2]** — `tools/pr-check.sh` and `tools/pr-check-test.sh`
  (each commit and file rule shown failing and passing on a scratch repo),
  wired into `tools/gate.sh`'s `tool_tests`. Test: the self-test, and the
  script run on PR #2's head names its clippy, trailer, author and
  test-data findings.
- [x] Step 3 **[1]** — the contributor path: a *Contributing from a fork*
  section in `AGENTS.md` (read before the maintainer's pipeline),
  `CONTRIBUTING.md` pointing to it and the script,
  `.github/pull_request_template.md`. Test: the docs tests.
- [x] Step 4 **[1]** — `.agents/skills/contribute/SKILL.md`. Test: the
  docs tests; the skill names only scripts that exist.
- [ ] Step 5 **[2]** — `.agents/skills/review-pr/SKILL.md`: review (the PR
  fetched to `pr/<n>`, CI approved for a first-time contributor, `pr-check`
  over it, the new tests run on the merge base without the fix through
  `tools/at-commit.sh`, the rules and `.agents/review.md` read, the review
  drafted as *What I checked / Blockers / Should fix / Project
  conventions* and posted on the human's word) and `--land` (`full-ci`
  green, rebased, bullet and docs amended in, `tools/verify.sh`, pushed to
  the head then `main` fast-forwarded; without maintainer edit rights,
  `main` pushed and the PR closed naming the commits). Test: docs tests.
- [ ] Step 6 **[2]** — the CI tiers of the design delta. Test:
  `tools/workflows-test.sh` (actionlint) and `tools/ci-reuse.sh
  --self-test`.

## Acceptance
- `tools/pr-check-test.sh` passes; `tools/pr-check.sh --base <PR #2's merge
  base>` on PR #2's head fails naming the clippy lint, the trailers, the
  `localhost` author and `crates/arris-io/tests/data/*.step`, and passes on
  `main`'s tip against its parent.
- actionlint is clean over `.github/workflows/`.
- After the human pushes: PR #2's next run shows `pr-fast` and the cheap
  jobs, no `test`/`parallel`/`wheels` shards, until it is labelled
  `full-ci`.

## Docs to update on completion
- `AGENTS.md` — the skills sentence names `contribute` and `review-pr`.
- `docs/README.md` §skills — the two skills beside the pipeline's.
- `.agents/rules/git.md` — written in step 1; re-read against the landed
  scripts.
- `GLOSSARY.md` — *Land*, step 1.

## Open questions
- ⚠ OPEN: the `full-ci` label and protecting `main` against force pushes
  and deletion (no required review) — the human, in GitHub settings, after
  retirement (`gh label create full-ci`; Settings → Rules).
- Decided (agent): contributors leave `CHANGELOG.md` alone and `semver` is
  advisory on a PR — the conflict PR #2 hit sat in the changelog, and the
  landing that writes the bullet runs before `main`'s CI holds the gate.
