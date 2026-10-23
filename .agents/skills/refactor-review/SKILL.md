---
name: refactor-review
description: Review the whole workspace for structural debt — duplicated code, oversized files and functions, seams later features will hit, what earlier refactor reviews left open — and write the result as one idea under docs/ideas/ whose milestone plans the side slot works through. Use when the human says "refactor", "review the codebase", "we added features without refactoring", or after /close-cycle hands over. Analysis only; never edits code.
argument-hint: [focus, e.g. a crate or "before <cycle>"]
---

# /refactor-review — measure, read, then one idea

A review finds structure to change with **no behaviour change**. What it
writes is an idea (`docs/ideas/refactor-<when>.md`, e.g.
`refactor-after-c10`); plans absorb it one milestone at a time, cutting
the file down (`.agents/rules/docs-lifecycle.md`), and the last one
deletes it. One file per review, never one per finding.

1. **Find the base.** The previous review is a `docs/ideas/refactor-*.md`
   still open, or the last one in git (`git log --diff-filter=D
   --oneline -- 'docs/ideas/refactor-*'`) with the retirements of the
   refactor plans that absorbed it. Its base commit is where this review
   measures from; its unfinished items are this review's first rows.
2. **Measure** — run, don't read, the scripts beside this file, from the
   repository root:
   - `sh scripts/sizes.sh <base>`: lines per crate, files past 900 lines, commits since the base;
   - `python3 scripts/fn_lengths.py`: the longest library functions;
   - `python3 scripts/duplicates.py`: identical bodies and repeated constants across files;
   - `python3 scripts/rule_patterns.py`: what `.agents/rules/kernel.md` forbids, counted.
   Also `ls` each crate's module tree with sizes; new sibling modules
   beside an old one (a second tracer, a fourth sweep) are where copies hide.
3. **Re-check the open items** from step 1 against the code: each is
   *fixed*, *open* (with its size now) or *no longer applies*. Backlog lines
   that name a split or a duplicate (`grep -niE 'split|duplicat|twice'
   docs/BACKLOG.md`) are open items too.
4. **Read the hotspots.** A script hit is a lead, not a finding: open the
   code, compare the copies, say how they differ. Every finding is marked
   **read** (compared in the source) or **measured** (a count or a hash).
   A drift between copies that could be a wrong answer is a regression
   fixture first (`inspect`), named in the idea, never fixed here.
5. **Write the idea** from `docs/ideas/TEMPLATE.md`, with a
   `## Findings` section after *Problem*:
   - findings grouped by the milestone plan that would take them, each
     naming files and functions — no line numbers (they rot; state the
     commit instead);
   - a *leave these alone* list: what looks big and is right;
   - *Options* are orderings against the current cycle (all first,
     interleaved, only when a file is next touched, do nothing), not the
     milestones themselves; each milestone is a plan of ≤10 steps with
     its acceptance: every blessed dump, `fixture.json`,
     `cancel_counts.txt` and `tolerance_band.txt` byte-identical, ids and
     refusal strings unchanged;
   - a public type or signature change is called out as a break needing
     the binding, `CHANGELOG.md` `### Breaking`, and the human's yes;
   - the backlog lines the idea absorbs are removed from `docs/BACKLOG.md`.
6. **Reply** with the summary, the recommendation and decision round 1,
   as `/idea` does; answers go into the file's *Decision*.

Keep to: tracked docs only (no `docs/notes/` path, no consumer names);
the glossary's words; one commit, `docs: refactor review <when>`.
Refactor plans run in the side slot beside the cycle's own plan, within
the two-plan limit.

`$ARGUMENTS`
