# Git: trunk-based, always green

- **Every commit** passes `.githooks/pre-commit`; never `--no-verify`. The
  hook sizes its gate to what is staged (`tools/gate.sh`, ADR-0032): prose
  runs the docs tests, a version bump `cargo check`, anything else the
  layer rule, fmt, `clippy -D warnings`, the doctests and the `fast`
  nextest profile at 32 cases over the crates and corpus areas the paths
  reach, and `cargo doc` when a doc comment changed.
- **The full gate** is `ARRIS_GATE=full`'s list: `cargo fmt --check`,
  `cargo clippy --workspace --all-targets -- -D warnings`, `cargo nextest
  run --workspace` (256 cases, `real_*` included), `cargo test --workspace
  --doc`, `RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps`,
  `tools/check-layers.sh`. It runs once per plan (`/retire-plan`), at
  `/close-cycle` and `/release`. CI runs it at 1000 cases plus the
  `oracle`, `wasm`, `parallel` and `python` jobs no local gate runs; the
  human reads CI before a push that matters.
- Commit **directly to `main`**, one plan step per commit. Branch
  (`plan/<slug>`, rebased, `merge --ff-only`, deleted after) only when a
  plan may be thrown away or the human asked to review first.
- Never rewrite `main` (no amend of a pushed commit, no force-push, no
  rebase). Never push unless asked.
- Stage deliberately: read `git status` and `git diff`, no blind `git add
  -A`. A changed `tests/fixtures/**/*.json` expectation is staged only
  with an intentional geometry change, and the message says `fixtures:`
  and why. Oracle values do not drift; if they changed, the body says
  whether the fixture or the oracle script did.

## Message format

```
type(scope): imperative summary, lower case, no period  (plans/<slug> step N)

Why, not what. Cite the ADRs and docs that justify it or that it updated.
Name any public type or signature it changes.
```

- `type`: `feat`, `fix`, `refactor`, `perf`, `test`, `docs`, `chore`,
  `build`, `ci`, `fixtures`. `scope`: crate short names (`math` … `arris`,
  several as `geom,ops`) or `docs`, `plan`, `adr`, `tools`.
- The `(plans/<slug> step N)` suffix marks every plan-step commit;
  retirement is `docs: retire plan <slug>`.
- Docs change in the **same commit** as the behaviour; a docs-only catch-up
  is `docs(sync): …`.

## Tags

- `m0`, `m1`, … mark milestones; `vX.Y.Z` releases, on `main`, created and
  pushed by the human. The pushed tag publishes the crates to crates.io
  and the Python package `arris` to PyPI in lockstep (ADR-0034), each
  behind its own environment reviewer (`crates-io`, `pypi`). `arris-debug`
  and `arris-py` are `publish = false`. Mechanics: the `/release` skill.

### The version

- `main` carries the next version with `-dev`: `[workspace.package].version
  = "0.N.0-dev"`, and the seven internal crates pinned to exactly it in
  `[workspace.dependencies]` (`version = "=0.N.0-dev"`). The exact pin is
  the guard: a bump that misses one fails `cargo check`, and `release.yml`
  refuses a pre-release. PyPI reads it as PEP 440 (`0.5.0.dev0`).
- Pre-1.0, `0.y.z` is `y` breaking. **Closing a cycle bumps the minor**
  (Cn → `0.n.0` while that lines up); **a release between cycles bumps the
  patch**, unless `CHANGELOG.md`'s `Unreleased` has a `### Breaking`
  bullet — a lookup, not a judgement. `tools/semver-gate.sh` fails CI on a
  break `Breaking` doesn't list.
- The version lives only in `Cargo.toml` and `CHANGELOG.md`'s headings —
  no doc, README or rustdoc states it.

### The changelog

`CHANGELOG.md` is what a consumer reads (ADR-0027).

- An entry is written in the commit that makes the change; a plan's
  bullets by its `/retire-plan` commit. Changes no consumer can see
  (process, tooling, tests, docs) write nothing.
- Bullets go under `## Unreleased`, for a reader who has read only
  `README.md`: what they can now do (not what was built), the refusals
  they will hit, no ADR numbers, plan slugs or fixture names. A changed
  public type or signature goes under `### Breaking` with its one-line
  fix, in the same commit.
- Only `/release` turns `Unreleased` into a version.

## Without asking

While executing a plan: run checks, commit each step, tick boxes, edit the
plan. Branching, tagging, pushing, resetting, rewriting and publishing are
asked first.
