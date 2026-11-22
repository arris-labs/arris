# Plan: publish-survives-index-timeout

- Started: 2026-11-22
- Milestone: release tooling beside C12 (`docs/ROADMAP.md`)
- Idea: `docs/ideas/publish-survives-index-timeout.md` (absorbed, option A)

## Goal

The crates.io job can resume a tagged release after an upload succeeds but
Cargo times out waiting for the sparse index. Each attempt checks which
workspace versions are indexed, then lets stable Cargo publish the missing
crates in dependency order. Once all crates are published, the job can
create or update the GitHub Release with the tagged changelog section.

## Non-goals

PyPI recovery, changes to registry permissions or the release version, and
repairing past tags. Acceptance uses local test doubles for uploads and
GitHub Release creation.

## Design deltas

- The crates.io job calls `tools/publish-workspace.sh` after the tag,
  release notes and authentication steps. The GitHub Release action runs
  on success and uses the existing notes file (ADR-0027). Its `body_path`
  input replaces the notes on a rerun. The Rust and Python packages still
  ship from the same tag through independent jobs (ADR-0034).
- Derive the crate names and exact versions from locked Cargo metadata
  for crates.io-publishable workspace members. Use the [sparse index file
  layout and format](https://doc.rust-lang.org/cargo/reference/registry-index.html)
  to look up each version. Treat a 404 or a valid file without the exact
  version as missing. Treat lookup errors, invalid records, and a yanked
  target version as explicit failures. Bound the time for index requests.
- Use `cargo publish --workspace --locked` with an `--exclude` for each
  version already indexed. When every version is present, finish without a
  Cargo publish command. Use at most three attempts, with a two-minute
  wait after a Cargo failure when another attempt remains, and refresh
  the index before that attempt. After a final Cargo failure, check once
  more for completion.
  If any crate is still missing or its status cannot be established, fail
  the job with that context. Keep the upload token in the Cargo step's
  environment, with index lookups unauthenticated.
- Provide a read-only `--dry-run` that reports the selected Cargo command
  or that all crates are already indexed. The command and retry limits
  belong to release tooling. Public APIs and crate boundaries are outside
  this plan's scope. The recovery procedure is recorded in the release
  runbook and Git rules.

## Steps

**[1]** routine. **[2]** careful work within a given design. **[3]** an
unproven algorithm or bound. The grade selects the tier in `.agents/tiers.md`.

- [x] Step 1 **[1]** — Add the resume helper and its offline regression
  check in `tools/publish-workspace-test.sh`. Route the crates.io job
  through the helper. Run the acceptance commands and check the tagged-job
  routing with stubbed index, Cargo, and GitHub Release commands.

## Acceptance

- Run `bash -n tools/publish-workspace.sh`,
  `bash -n tools/publish-workspace-test.sh`, `tools/publish-workspace-test.sh`,
  and `tools/workflows-test.sh` with `actionlint` on `PATH`.
- The offline test exercises the helper's normal entry point. Cover a fresh
  release, delayed visibility after upload, a fresh run resuming a partial
  release, and an already-complete release. Assert the Cargo arguments at
  each attempt, including exact exclusions and workspace selection. Cover
  a crate with other indexed versions, a missing index file, and metadata
  for private workspace members. Dry run records zero upload calls.
- Inject unreadable or malformed index data, a yanked version, and a
  persistent Cargo failure. Assert that unknown state never selects an
  upload, that at most three attempts and two waits occur, and that failure
  reaches the workflow if the final lookup still shows an incomplete
  release. Cover a final Cargo error followed by an all-present lookup.
- Review `release.yml` against its base diff and exercise the changed
  routing locally. A branch or a mismatched tag cannot reach publishing.
  Successful recovery and the already-complete case reach the existing
  GitHub Release action with the same notes file. Failed recovery stops
  before that action, and PyPI is independent. Confirm the action's
  existing-release behavior against its [documented `body_path` and
  `append_body` inputs](https://github.com/softprops/action-gh-release#inputs).

## Docs to update on completion

- `.agents/rules/git.md` §Tags and `.agents/skills/release/SKILL.md`
  §When a publish fails partway describe bounded retries and rerunning the
  failed job at its original tag after the index catches up.
- `docs/ROADMAP.md` C12 side slot records the publish retry as landed.
- `AGENTS.md` Current state advances to `/close-cycle`.

## Open questions

None.
