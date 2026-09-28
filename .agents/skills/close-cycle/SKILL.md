---
name: close-cycle
description: Close a finished release cycle or milestone — verify nothing is still open, run the mandated drift review of every design doc against the code, compress the finished section of docs/ROADMAP.md to its status line, open the next cycle's section, update the spine and AGENTS.md, and hand the tag to the human. Use when the human says "close the cycle", "v0.2 is done", "milestone is done", "what's next after this release", or when the last line of a roadmap cycle has been retired. Never tags and never pushes.
argument-hint: <cycle or milestone, e.g. v0.2 — and optionally the next cycle's theme>
---

# /close-cycle — one roadmap, one section per cycle

`docs/ROADMAP.md` is a **living design doc**, not a log. It is never
forked into a second roadmap file and never accumulates plans: a finished
cycle shrinks to its status line, and the next cycle is appended below it.
The design docs are *topics* — architecture, data model, roadmap — so a
second roadmap file would only ever raise "which one is current?".

## Do

1. **Check it is actually finished.** Every line of the cycle's section
   either done or explicitly moved to `docs/BACKLOG.md`; `docs/plans/`
   holding nothing but `TEMPLATE.md`; `cargo fmt --check`, `cargo clippy
   --workspace --all-targets -- -D warnings` and `cargo test --workspace`
   green; and the last CI run on `main` green in every job (`gh run list
   --branch main`, then `gh run view <id>` for its jobs) — the hook does
   not run CI's `oracle`, `wasm` or `parallel` jobs, so a job red on
   `main` is only visible there. If not, stop and report exactly what is open — do not close
   around it.
2. **Reconcile the findings** (`docs/BACKLOG.md` §Findings). List the
   scheduled `Nightly` runs since the cycle opened (`gh run list
   --workflow=nightly.yml`); every failing job is a defect that tier
   measured, and each must be accounted for by a block there, by a
   regression fixture or by a named exclusion — an unaccounted one is open
   work, and the cycle does not close around it. Then walk every block:
   its `State` is current (a `raw` that now has its numbers is
   `measured`; a `measured` that has been shrunk names its
   `regression/<slug>`), its `Reproduce` still replays, and any block the
   cycle fixed is deleted with its fixture lifted. A block whose seed or
   commit has gone stale is rewritten or deleted, never left to age.
3. **Drift review** (`docs/README.md` mandates it at every boundary, and
   this is the only place it happens): read **every** design doc —
   `ARCHITECTURE.md`, `DATA-MODEL.md`, `ROADMAP.md` — against the code and
   fix each sentence that is no longer true. The cycle is not closed until the list is empty. List
   what you fixed in the reply; if you fixed nothing, say why you believe
   nothing had drifted.
4. **Compress the finished section** to the standard shape:
   goal, one `**Status: done <date>, tag `vN`.**` line naming the risk that
   was retired and the ADRs that were taken, then the in / out / accept
   lists. Narrative goes; `/plan` reads in/out/accept and `/retire-plan`
   writes the status line, so those four survive. Before deleting any
   sentence, grep for the fact in `docs/`, `crates/` and `README.md` — a
   measurement or a rationale that lives *only* here is relocated to the
   doc or the code comment that wants it, never dropped.
5. **Open the next section, and give it its number.** An unopened cycle
   carries a **name** in `docs/ROADMAP.md` and gets its number here
   (ADR-0020) — which is what keeps "cycle Cn releases `0.n.0`"
   (`.agents/rules/git.md` §Tags) lining up with the order cycles are
   actually opened in. `## Cn — <name>` with goal, in, out and accept,
   drawn from the roadmap's outline for that cycle and `docs/BACKLOG.md`.
   **Which** name comes next: the reader cycle is next while it is
   unopened; after it, apply ADR-0020's selection rule — the refusal
   histogram over the real-part corpus and the first consumer's
   side-by-side regressions, the consumer's ranking first while one waits
   on a swap — and record in the new section's goal the numbers it was
   chosen on, so the choice can be re-checked. The theme and the in/out
   split are the **human's call**: propose, do not decide. If the answer
   is not obvious from the rule and the backlog, stop and ask.
6. **`Spine:`** at the top of the roadmap gains the new cycle, by number;
   the name leaves the "Named cycles, unordered" list at the same time,
   and the standing sections beside the cycles (the measuring harness,
   the binding) keep their current numbers.
7. **`AGENTS.md` "Current state"**: one sentence for the closed cycle, a
   new `**Next:**`, block still under ~15 lines.
8. Commit as `docs: close <cycle>` with a body listing the docs updated and
   the drift fixed.
9. **Run `/release`.** A closed cycle is a release: it bumps the minor
   (`.agents/rules/git.md` §Tags), and that skill picks the number, turns
   `CHANGELOG.md`'s `Unreleased` into the version's section, bumps the
   version and its pins, proves the workspace still packages, and hands
   the human the tag. Tags and pushes are theirs
   (`.agents/rules/git.md`), never yours.

## Don't

- Don't create a second roadmap file or an `ARCHIVE.md`. Git holds the
  history; that is the whole reason the section compresses. `CHANGELOG.md`
  is not one: it is what a consumer reads, written change by change
  (ADR-0027), not the cycle's record.
- Don't leave the finished section at full length "because it is useful" —
  a cycle that keeps 40 lines is what makes the file look unmaintainable
  after three of them.
- Don't invent the next cycle's scope. A roadmap section is a commitment
  the human makes, not one the agent proposes into existence.
- Don't tag, push, or open plans for the new cycle here. `/idea` and
  `/plan` come after, one line at a time.
- Don't bump the version by hand or skip `/release` "because it is only a
  version number": it is eight places in `Cargo.toml`, a derivation from
  the changelog and the log, and the version's changelog section.

`$ARGUMENTS`
