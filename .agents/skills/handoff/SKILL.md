---
name: handoff
description: Write a handoff note so a fresh session continues this one's work, and give the prompt to paste.
disable-model-invocation: true
argument-hint: <what the next session is for, optional>
---

# /handoff — a note a fresh session starts from

Write `/tmp/handoff-arris-<slug or topic>-<HHMM>.md`, outside the
repository. It holds only what the repository does not: the plan, ADRs,
commits and diffs are pointed at by path, hash or step number, never
restated.

- **Goal**: the one thing the next session is for (the argument, if given).
- **Where it stands**: the plan and step, the last commit (`git log -1
  --oneline`), and what `git status` shows, saying which changes are this
  session's and which are not.
- **Done here but not in a commit**: findings, measurements, a half-made
  change and why it is half-made.
- **Dead ends**: what was tried and why it failed, so it is not tried again.
- **Next action**: the first command or edit, concretely.
- **Open questions**: each with who decides.
- **Skills**: which to run first (`/work <slug> step N`, `inspect`, …).

Leave out secrets and machine-local paths a tracked doc would not name.
Then reply with the path and the line to paste into the fresh session:

```
Read /tmp/handoff-arris-<…>.md and continue from its Next action.
```

`$ARGUMENTS`
