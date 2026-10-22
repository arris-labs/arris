# Plan: python-wheels

- Started: 2026-10-22
- Milestone: beside C11 (docs/ROADMAP.md §Beside the cycles, the first-party binding)
- Idea: none; docs/BACKLOG.md's line "Python wheels beyond Linux x86_64 …" and ADR-0067 §5 (the launch gate), verbatim from the human: "both 2 and 3"

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal

`pip install arris` installs a prebuilt wheel, with no Rust toolchain, on
Linux x86_64 and aarch64, macOS arm64 and x86_64, and Windows x86_64. Each
wheel CI builds is installed in a clean virtual environment on its own
platform and passes the binding's pytest. The bodies it builds match
Linux's byte for byte: the determinism corpus runs natively on each of
those platforms. CI builds and tests the matrix on every push to `main`.
`release.yml` builds the same matrix from the same definition and uploads
it under the `pypi` environment beside the sdist. A pyo3 upgrade has a
written rule that re-runs ADR-0034's `forbid(unsafe_code)` check.

## Non-goals

- musllinux, Windows arm64, 32-bit targets, Python builds without abi3: each
  is one line in the matrix later, if someone asks. Backlog line at
  retirement.
- Any change to the binding's API, or the wheel's contents beyond the
  extension module.
- conda-forge, Homebrew, or any channel other than PyPI.
- Making the full Rust gate (`test`, `parallel`, `oracle`) run on macOS or
  Windows. Only the determinism corpus and the Python suite run there.

## Design deltas

- **One matrix, two callers.** A reusable workflow,
  `.github/workflows/wheels.yml` (`on: workflow_call`), builds the abi3 wheel
  per target with `PyO3/maturin-action`. It then installs the wheel into a
  fresh venv *without* `maturin develop` and runs `pytest crates/arris-py`.
  `ci.yml` calls it on push to `main` and on pull requests that touch
  `crates/**`. `release.yml`'s `pypi-build` calls it after its version
  guard, builds the sdist once, and merges the artifacts. The `pypi` job is
  unchanged: it publishes `dist/*`.
- **Targets** (runner → target): `ubuntu-latest` → x86_64 manylinux2014;
  `ubuntu-24.04-arm` → aarch64 manylinux2014; `macos-14` → aarch64-apple-darwin;
  x86_64-apple-darwin is built on the arm runner and tested under Rosetta.
  Step 1 checks that Rosetta is available; if it isn't, the wheel is tested
  on an Intel macOS image. `windows-latest` → x86_64-pc-windows-msvc. The
  macOS deployment target is 11.0, set as `MACOSX_DEPLOYMENT_TARGET` in the
  workflow.
- **Determinism off Linux.** ADR-0064 holds wasm32 to the native digests
  with one `libm`. This plan adds `cargo test -p arris --test determinism`
  on the macOS and Windows runners. A digest that differs is a fault fixed
  in the kernel, never a per-platform digest.
- **The Python suite off Linux** reads oracle values from committed JSON,
  so it needs no Open CASCADE. Paths go through `pathlib` and line endings
  are normalised where a test compares text (STEP, OBJ). Each fix lands in
  the step that finds it.
- **pyo3 upgrade rule:** a paragraph in ADR-0034's Consequences, as an
  amendment dated at this plan. A pyo3 bump re-runs the expansion check
  that ADR-0034 recorded for `forbid(unsafe_code)`, and the commit body
  names its result. `docs/ARCHITECTURE.md` §the binding points to it.
- *Settled:* `determinism-platforms` lives in `wheels.yml`, not `ci.yml`,
  and the workflow takes `workflow_dispatch`: everything that runs off
  Linux x86_64 is one workflow a branch can run alone, without the hours
  of `ci.yml`'s `test` job. The release therefore runs it too.
- No public Rust or Python type changes.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [ ] Step 1 **[2]** — The riskiest unknown first: a CI job,
  `determinism-platforms`, runs the determinism corpus natively on
  `macos-14` and `windows-latest` (and `ubuntu-24.04-arm`). A digest that
  differs is shrunk to a regression fixture and fixed in this step, or split
  into its own step if the fix is large. Record whether Rosetta is available
  on `macos-14`.
- [ ] Step 2 **[2]** — `wheels.yml`: the build-and-test matrix above, called
  from `ci.yml`. Each target's wheel is installed from the built file and
  passes `pytest crates/arris-py` (3.13). Path and line-ending faults it
  finds are fixed here.
- [ ] Step 3 **[1]** — The `python` job's 3.10 run is kept on Linux. Its
  wheel and sdist step and artifact are removed, because `wheels.yml` now
  builds them. `twine check --strict` runs over the merged artifacts.
- [ ] Step 4 **[1]** — `release.yml`: `pypi-build` keeps its version guard,
  calls `wheels.yml`, builds the sdist, merges every wheel and the sdist
  into `python-dist`, and runs `twine check`. The comment about the
  platform matrix being a release-time decision is replaced with the list.
- [x] Step 5 **[1]** — A smoke test for an installed wheel only
  (`tests/test_wheel.py`, skipped under `maturin develop`). It imports
  `arris` with no compiler on `PATH`, builds the showcase bracket
  (`examples/bracket.py`), and holds it to `bracket.oracle.json`.
- [ ] Step 6 **[1]** — The pyo3 upgrade rule: the ADR-0034 amendment and
  the `docs/ARCHITECTURE.md` pointer, plus a `CHANGELOG.md` bullet under
  `Unreleased` saying `pip install arris` now has prebuilt wheels for
  macOS, Windows and Linux on ARM.

## Acceptance

On a push to `main`: the `wheels` workflow is green for all five targets,
each wheel installed from its file and passing the binding's pytest and
`test_wheel.py`, and `determinism-platforms` is green on macOS, Windows and
Linux aarch64 with the committed digests. `release.yml` is checked by
`actionlint` and by one `workflow_dispatch` dry run on a branch. That run
stops at the version guard's pre-release refusal, after the matrix has
built, because nothing may publish. Its `python-dist` artifact holds five
wheels and one sdist, and `twine check` accepts them all.

## Docs to update on completion

- `docs/ROADMAP.md` §Beside the cycles, the first-party binding: "the
  platforms of the wheel beyond Linux x86_64" is removed from what it waits
  on, and the platforms are named.
- `docs/ARCHITECTURE.md` §the binding: the wheel matrix and the pyo3 rule.
- `docs/adr/0034-*.md`: the amendment (step 6).
- `README.md` install section: a line saying that prebuilt wheels cover
  Linux, macOS and Windows.
- `docs/BACKLOG.md`: the non-goal targets as one line.
- `AGENTS.md` current state: not needed (not a milestone).

## Open questions

- ⚠ OPEN: **CI minutes.** The repository is public, so the macOS and Windows
  runners are free. If that changes, the matrix moves to release-only. The
  agent decides, and only if it changes.
- ⚠ OPEN: **x86_64 macOS testing.** Rosetta on the arm runner, or an Intel
  image. The agent decides at step 1 and records it in `wheels.yml`.
- ⚠ OPEN: **When the first multi-platform upload happens** is decided by
  the human: the next `v*` tag after this plan retires. Publishing stays
  theirs.
