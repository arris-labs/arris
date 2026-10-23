---
name: close-cycle
description: Close a finished cycle: verify, reconcile findings, drift-review the docs and glossary, compress its roadmap section, open the next, then hand over to /release.
disable-model-invocation: true
argument-hint: <cycle or milestone, e.g. v0.2 — and optionally the next cycle's theme>
---

# /close-cycle — one roadmap, one section per cycle

`docs/ROADMAP.md` is a living design doc, not a log: a finished cycle
shrinks to its status line and the next is appended below it. Never a
second roadmap file, never an `ARCHIVE.md` — git holds the history.

## Do

1. **Check it is finished.** Every line of the section done or moved to
   `docs/BACKLOG.md`; `docs/plans/` holds only `TEMPLATE.md`; the full gate
   (`.agents/rules/git.md`) green — run it now, don't wait for CI; and the
   last CI run on `main` green in every job (`gh run list --branch main`,
   `gh run view <id>`), since `oracle`, `wasm`, `parallel` and `python` run
   only there. Anything open: stop and report it; never close around it.
2. **Reconcile the findings** (`docs/BACKLOG.md` §Findings). Every failing
   job of the scheduled nightlies since the cycle opened (`gh run list
   --workflow=nightly.yml`) is accounted for by a block, a regression
   fixture or a named exclusion — otherwise it is open work. Walk every
   block: `State` current (`raw` with numbers → `measured`; shrunk → names
   its `regression/<slug>`), `Reproduce` still replays, fixed blocks
   deleted with their fixture lifted, stale ones rewritten or deleted.
3. **Drift review** — the only place it happens (`docs/README.md`): read
   `ARCHITECTURE.md`, `DATA-MODEL.md`, `ROADMAP.md` and `GLOSSARY.md`
   against the code and fix every sentence no longer true; a term the docs
   now use and the glossary lacks gets its entry. List the fixes in the reply, or say
   why nothing drifted.
4. **Public docs** — `README.md` and `docs/COMPARISON.md` are read against
   the code and `CHANGELOG.md`, not the roadmap, and fixed where they
   disagree:
   - `README.md`: every capability in its Features list is in the
     changelog or the code, and nothing the changelog says is in is missing;
     the "Not yet" list matches the refusals the code returns; every link
     resolves.
   - `docs/COMPARISON.md`: each Arris cell is true at the version being
     cut; the Open CASCADE column matches the `cadquery-ocp` pin in
     `tools/oracle/pyproject.toml`; every fixture path it cites exists
     (`ls`); every measured number is re-measured or still cited to its
     fixture; "What is not claimed" still holds.
   - Neither names a cycle or a plan: those belong to the roadmap.
   List the fixes in the reply, or say why nothing drifted.
5. **Compress the finished section** to: goal, one `**Status: done <date>,
   tag `vN`.**` line naming the risk retired and the ADRs taken, then the
   in / out / accept lists. Before deleting a sentence, grep `docs/`,
   `crates/` and `README.md` for its fact: a measurement or rationale that
   lives only here moves to the doc or code comment that wants it.
6. **Open the next section and number it** (ADR-0020): `## Cn — <name>`
   with goal, in, out, accept, from the roadmap's outline and the backlog.
   Choose the name by ADR-0020's rule — the refusal histogram over the
   real-part corpus and the first consumer's regressions, the consumer's
   ranking first while one waits on a swap — and record the numbers it was
   chosen on in the goal. Theme and in/out split are the **human's call**:
   propose; if the rule and backlog don't make it obvious, ask.
7. **`Spine:`** gains the new cycle by number; its name leaves "Named
   cycles, unordered"; the standing sections keep their numbers.
8. **`AGENTS.md` "Current state"**: one sentence for the closed cycle, a new
   `**Next:**`, still under ~15 lines.
9. Commit `docs: close <cycle>`, the body listing the docs updated and the
   drift fixed.
10. **Hand over to `/release`**: tell the human to run it next — a closed
   cycle bumps the minor. Tags and pushes stay the human's. Then suggest
   `/refactor-review`, unless a `docs/ideas/refactor-*.md` is still open:
   its milestone plans run in the new cycle's side slot.

## Keep to

- The finished section shrinks to its five parts, however useful the rest
  reads; its facts move to the doc that owns them.
- The next cycle's scope is the human's: propose it, and leave its plans to
  `/idea` and `/plan`.
- The version moves only through `/release`.

`$ARGUMENTS`
