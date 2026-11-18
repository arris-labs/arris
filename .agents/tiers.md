# Tiers: which model executes a plan step

A plan step's complexity grade (`docs/plans/TEMPLATE.md`) is its **tier**:
the class of model that executes it.

| Tier | Grade | The step |
| --- | --- | --- |
| 1 | **[1]** routine | the design says exactly what to write; the tests are mechanical |
| 2 | **[2]** careful | a geometric or numeric case to get right within a given design |
| 3 | **[3]** unproven | an algorithm whose robustness or bound is established here |

A step runs on its grade's tier. A step that needs another tier than its
grade names it after the grade, `**[1, tier 3]**`: a small step that needs
the strongest reasoning. Plans, skills and ADRs name tiers, never a model;
which model a tier is belongs to the tables below, one per harness, and a
model update is an edit of one row.

## Claude Code

| Tier | Model | Effort |
| --- | --- | --- |
| 1 | `haiku 5.5` | `medium` |
| 2 | `sonnet 5.5` | `medium` |
| 3 | `opus 5.5` | `medium` |

The conductor (`/work` with no range) starts a step with the Agent tool,
`model` set to the tier's name in parentheses; the subagent's model follows
the harness's current release of that family.

The conductor needs no tier of its own: it starts steps, checks what
landed and runs `/retire-plan`, whose review and hard fixes are
subagents. Tier 2 is enough; it conducted development-throughput.

## Codex

| Tier | Model | Reasoning effort |
| --- | --- | --- |
| 1 | `gpt-6.2-sol` | `low` |
| 2 | `gpt-6.2-sol` | `medium` |
| 3 | `gpt-6.2-sol` | `high` |

Effort is pinned here rather than inherited: a tier-3 step is the one that
needs the deepest reasoning, and the session's setting may be lower. The
conductor for this harness is not built yet (`docs/BACKLOG.md`); until it
is, a step runs in a fresh session on its tier's model and effort.

## Other harnesses

A harness gets its table here when it first runs a plan. One whose
subagents cannot be given a model runs a plan's steps one at a time, each
in a fresh session started on the tier's model.
