---
name: next
description: Where things stand (cycle, plans, findings, uncommitted and unpushed work, CI, release due) and the one command to run next.
disable-model-invocation: true
---

# /next — where we are, and what to run

Read-only: it changes nothing, it orients the human. Gather in parallel:

- `AGENTS.md` "Current state" and the open cycle's section of
  `docs/ROADMAP.md` (its in-list against what landed);
- `docs/plans/*.md` but `TEMPLATE.md`: each plan's unchecked steps with
  their grades;
- `docs/ideas/*.md`: each idea's `Status` and any unanswered round;
- `docs/BACKLOG.md` §Findings: how many, and any naming the open cycle;
- `CHANGELOG.md` `## Unreleased`: whether it has bullets, and a
  `### Breaking`;
- `git status --short`, `git log --oneline origin/main..main | wc -l`
  (unpushed), `git describe --tags --match 'v*' --abbrev=0` and the
  commits since it;
- `gh run list --branch main --limit 3` and `gh run list --workflow
  nightly.yml --limit 3` (skip them quietly if `gh` fails).

Reply in one screen, in this shape:

```
Cycle    C10, the NURBS cycle: 2 of 4 in-list lines landed
Plans    sweep: step 4 next ([2] the refusals), 6 left
Ideas    sweep-and-loft: loft half waits for the next plan
Findings 3 open, 1 on this cycle (the differential's case 222)
Tree     clean | 4 files staged by another session
Git      9 unpushed; last release v0.9.0, 41 commits since; CI green
Due      a patch release (Unreleased has 5 bullets, no break); push

→ /work sweep step 4-5   both [2]; Sonnet fits
  or: /release   (one line on why it waits or wins)
```

The `→` line is one command with its arguments and a reason, and the
model its grade suggests ([1]/[2] Sonnet, [3] Opus). A step that waits on
an `⚠ OPEN:` is named as blocked, with what unblocks it. A release is due
by `.agents/rules/git.md` §The version: say which bump the lookup gives.
Staged or modified files you did not make are another session's: say so,
and recommend nothing that would commit them.
