---
name: release
description: Cut a release of the workspace to crates.io and its Python package to PyPI — pick the version from CHANGELOG.md's Unreleased section, cross-checked against the log and the semver gate, turn that section into the version's notes, bump the version and its pins, prove the workspace still packages, and hand the human the exact tag command. Use when the human says "release", "cut a version", "publish", "ship 0.2", or at the end of /close-cycle. Never tags, never pushes, never runs cargo publish.
argument-hint: <nothing, or a version to force, e.g. 0.2.0>
---

# /release — the tag is the release

Nothing is published from a laptop. The human pushes a `v*` tag;
`.github/workflows/release.yml` publishes the crates behind the `crates-io`
reviewer, the wheel and sdist behind the `pypi` reviewer, and opens a
GitHub Release from the version's `CHANGELOG.md` section. This skill
prepares the commit the tag points at. The scheme is
`.agents/rules/git.md` §Tags — read it first.

## Do

1. **Check the tree.** On `main`, clean, the full gate green
   (`.agents/rules/git.md`), and the last CI run on `main` green in every
   job, `python` included. Name any open plan in the reply — it does not
   block a patch release, but the human should know what is half-landed.
2. **Pick the version from `## Unreleased`**: closing a cycle → minor;
   otherwise patch, unless `### Breaking` has a bullet → minor; empty →
   nothing to release, stop. Cross-check:
   - the commit bodies in `git log $(git describe --tags --match 'v*'
     --abbrev=0)..HEAD` — every public type or signature change they name
     has its `Breaking` bullet;
   - `tools/semver-gate.sh` passes.

   A body names a break the section lacks → add the bullet and say so. Any
   other disagreement → stop and ask. Give the decision as a derivation
   (bullets, commit hashes). An argument overrides the number; then say
   what the derivation would have picked.
3. **Turn `Unreleased` into the version**: heading `## X.Y.Z — YYYY-MM-DD`,
   a fresh empty `## Unreleased` above. Tighten it for a reader of only
   `README.md` (`.agents/rules/git.md` §The changelog) — it is the release
   notes. `docs_refs` holds the heading format.
4. **Bump** `[workspace.package].version` (drop `-dev`) and the seven
   `=X.Y.Z-dev` pins in `[workspace.dependencies]` — eight places, one
   edit; `cargo check` catches a miss. `arris-debug` has no version.
5. **Prove it publishes.** `cargo package --workspace --exclude arris-py`
   — every crate packages and verifies (a failure in the workflow comes
   after some crates are already up, and a version can't be replaced).
   Then the wheel, in a venv with `maturin` and `twine`:
   `maturin develop --manifest-path crates/arris-py/Cargo.toml --extras
   test`, `pytest crates/arris-py`, `maturin build --release`, `maturin
   sdist`, `twine check --strict` on both; the wheel's version is `X.Y.Z`,
   no `.dev0`.
6. **Commit** the bump and changelog as `chore(arris): release X.Y.Z`, the
   body giving step 2's derivation and the breaking list. After
   `/close-cycle`, its `docs: close <cycle>` commit comes first.
7. **Open the next `-dev`** in its own commit: `chore(arris): open
   X.(Y+1).0-dev`, same eight places. (Tagging this one by mistake is safe:
   `release.yml` refuses a pre-release.)
8. **Hand over the tag**:

   ```sh
   git tag vX.Y.Z <sha of step 6>
   git push origin main
   git push origin vX.Y.Z
   ```

   Remind them to approve `crates-io` and `pypi` with the tag's CI run in
   front of them — each is the last checkpoint before a version exists
   forever. The first PyPI upload also needs the pending publisher
   registered (`release.yml`'s header).
9. **After they push, cancel `main`'s duplicate CI run** (`gh run list
   --workflow CI --limit 4`; ~3 h each) **only if** `git diff --name-only
   vX.Y.Z main` lists nothing but `Cargo.toml`, `Cargo.lock`, `*.md`,
   `.agents/`, `.githooks/`. Never cancel the tag's run. Say which run was
   cancelled and why.

## When a publish fails partway

crates.io rate-limits new crates (burst 5, then one per 10 min), so a
release introducing crates can stop on a `429` half-published. Recoverable:
wait the window, `gh run rerun <id> --failed`; already-published crates are
skipped. Each re-run asks the reviewer again. The workflow that runs is the
one **at the tag**: a broken workflow is fixed on `main` and released as a
new version, never by moving the tag.

## Don't

- Don't tag, push, `cargo publish` or `twine upload`.
- Don't state the version anywhere but `Cargo.toml` and the changelog.
- Don't write `Unreleased` from the log now — the log only cross-checks it.
- Don't bump the minor "to be safe": pre-1.0 it is the consumer's only
  breakage signal.
- Don't release with a red suite, a dirty tree or an unreviewed fixture
  expectation.

`$ARGUMENTS`
