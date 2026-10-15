---
name: idea
description: Turn a raw feature idea or design question from the human into a brainstorm document under docs/ideas/ — problem, options with trade-offs, cost, conflicts with existing decisions, a recommendation, and the decision the human has to make. Use when the human says "idea:", "what if", "should we", "think about", or hands over a backlog line; also when a plan request is too vague or contested to plan directly. Produces analysis only — never code, never a plan, never checkboxes.
argument-hint: <idea in one sentence, or a backlog line>
---

# /idea — brainstorm, don't plan

An idea is where thinking happens so a plan doesn't have to. Not every idea
becomes a plan; a clear "no, because…" is a successful outcome.

1. Pick a slug (`kebab-case`, noun phrase). If `docs/ideas/<slug>.md` exists,
   extend it rather than duplicating.
2. Read what constrains the idea before forming an opinion: `SEED.md`
   (non-goals, differentiators), `GLOSSARY.md`, the design doc(s) it
   touches, every ADR whose decision it might bend, `docs/ROADMAP.md` for
   which cycle it would belong to, and the code if it exists. Cite paths
   and ADR numbers.
3. Write `docs/ideas/<slug>.md` from `docs/ideas/TEMPLATE.md`:
   - options are real alternatives with honest trade-offs, "do nothing"
     among them;
   - cost in plan steps, not hours; past ~10 steps, the milestone plans it
     would split into;
   - what it *conflicts* with, by name: a non-goal in `SEED.md`, an ADR, a
     layer rule, the consumer's API;
   - **the words**: a term the human used that `GLOSSARY.md` defines
     differently, or a new concept it needs, with the entry you propose.
     Say it in the reply too: "the glossary's *refusal* is X; you seem to
     mean Y".
   Two pages is long; write what the decision needs.
4. End with one recommendation and the decisions, in rounds. A round holds
   every question whose prerequisites are already settled; a question that
   hangs on another's answer waits for the next round, listed under
   *Later*. Number them and word each so "yes" accepts your preferred
   answer:

   ```
   ❓ **Q1 — <title>**: <the question, the options by letter>
   ➡️ <preferred answer, one line on why>
   ```

5. If the idea came from `docs/BACKLOG.md`, remove that line.
6. Reply with the recommendation and the round; the human answers "yes",
   or per question (`Q2: B`). Write the answers into the file's *Decision*
   section, add the agreed glossary entries to `GLOSSARY.md`, and ask the
   next round until *Later* is empty. The file is the record, the reply is
   the conversation.

The idea stays analysis: steps, checkboxes and file lists belong to
`/plan`; an ADR is proposed here and written by the plan; code and design
docs stay as they are.

`$ARGUMENTS`
