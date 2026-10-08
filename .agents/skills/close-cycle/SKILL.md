---
name: close-cycle
description: Close a finished release cycle or milestone — verify nothing is still open, run the mandated drift review of every design doc against the code, compress the finished section of docs/ROADMAP.md to its status line, open the next cycle's section, update the spine and AGENTS.md, and hand the tag to the human. Use when the human says "close the cycle", "v0.2 is done", "milestone is done", "what's next after this release", or when the last line of a roadmap cycle has been retired. Never tags and never pushes.
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
   `ARCHITECTURE.md`, `DATA-MODEL.md` and `ROADMAP.md` against the code and
   fix every sentence no longer true. List the fixes in the reply, or say
   why nothing drifted.
4. **Compress the finished section** to: goal, one `**Status: done <date>,
   tag `vN`.**` line naming the risk retired and the ADRs taken, then the
   in / out / accept lists. Before deleting a sentence, grep `docs/`,
   `crates/` and `README.md` for its fact: a measurement or rationale that
   lives only here moves to the doc or code comment that wants it.
5. **Open the next section and number it** (ADR-0020): `## Cn — <name>`
   with goal, in, out, accept, from the roadmap's outline and the backlog.
   Choose the name by ADR-0020's rule — the refusal histogram over the
   real-part corpus and the first consumer's regressions, the consumer's
   ranking first while one waits on a swap — and record the numbers it was
   chosen on in the goal. Theme and in/out split are the **human's call**:
   propose; if the rule and backlog don't make it obvious, ask.
6. **`Spine:`** gains the new cycle by number; its name leaves "Named
   cycles, unordered"; the standing sections keep their numbers.
7. **`AGENTS.md` "Current state"**: one sentence for the closed cycle, a new
   `**Next:**`, still under ~15 lines.
8. Commit `docs: close <cycle>`, the body listing the docs updated and the
   drift fixed.
9. **Run `/release`** — a closed cycle bumps the minor. Tags and pushes
   stay the human's.

## Don't

- Don't keep the finished section long "because it is useful".
- Don't invent the next cycle's scope, or open plans for it here — `/idea`
  and `/plan` come after.
- Don't bump the version by hand or skip `/release`.

`$ARGUMENTS`
