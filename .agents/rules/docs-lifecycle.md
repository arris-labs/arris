# Docs lifecycle: backlog → idea → plan → docs

Full rules in `docs/README.md` ("Document lifecycle"). The short version:

| Artefact | Where | Is | Lifetime |
|---|---|---|---|
| Backlog line | `docs/BACKLOG.md` | one sentence | until picked up or rejected |
| Idea | `docs/ideas/<slug>.md` | a **brainstorm**: problem, options, trade-offs, recommendation, decision for the human. No checkboxes. | until decided: accepted → absorbed by a plan and deleted; rejected → one line under "Rejected" in the backlog with the reason, file deleted; parked → kept with `Status: Parked` |
| Plan | `docs/plans/<slug>.md` | a **todo**: commit-sized steps with checkboxes, acceptance test, docs-to-update list | until retired: durable parts moved to design docs / ADRs, then deleted |
| Design doc | `docs/ARCHITECTURE.md`, `docs/DATA-MODEL.md`, `docs/ROADMAP.md` | the system as it is now, present tense | living |
| ADR | `docs/adr/` | a decision and its reasons | append-only |

- Not every backlog line becomes an idea, and not every idea becomes a plan.
  A small, obvious task skips the idea and goes straight to a plan; a large
  or contested one gets an idea first.
- At most **two active plans**. `docs/plans/` holding a third means retire
  one first.
- **A plan is read in two minutes**: under ~150 lines and ~10 steps, each
  step a few lines (its detail lives in *Design deltas* or an ADR). Larger
  work is split into milestone plans, each with its own acceptance,
  planned one after the other.
- **An idea one plan covers only in part** is cut down when that plan
  absorbs its part: the file keeps only what is still unplanned, with
  `Status: Accepted (<option>); <part> absorbed by plans/<slug>`, and the
  next plan absorbs the rest and deletes it.
- **`GLOSSARY.md` holds the project's words.** A new or sharpened term is
  written there in the commit that first uses it in a design doc, ADR or
  public API; a word the glossary lists under _Avoid_ is replaced by its
  term.
- An idea holds options and a decision, its steps left to the plan; a plan
  links its idea's decision (and the ADR if one was needed) as settled.
- Skills: `/idea`, `/plan`, `/work`, `/retire-plan` (which reviews the
  plan's diff against it and `.agents/review.md`) walk this pipeline;
  `/close-cycle` closes a roadmap cycle once its last plan is retired —
  drift review, the finished section compressed to its status line, the
  next section opened. One roadmap file, always; never a second one.
