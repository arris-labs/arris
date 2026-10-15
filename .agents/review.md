# Review: a plan's diff, read against the plan and the standards

Read by the reviewer subagent `/retire-plan` starts, never by the agent
implementing a step: the reviewer gets the commits and has no exploration
to do, so the judgement calls live here instead of in every session's
context. What the compiler, clippy, the hook, `tools/check-layers.sh`, the
corpus lint, the docs tests and CI's semver gate already enforce is left
to them.

## The brief

Two axes, reported apart so one cannot mask the other.

**Plan.** Quote the plan line for each finding:

1. a *Goal*, *Design deltas* or *Acceptance* item the commits leave missing
   or partial;
2. behaviour the commits add that the plan did not ask for, or that its
   *Non-goals* fence out;
3. an item that looks done but looks wrong;
4. a box ticked whose commit carries no fixture, property or oracle
   comparison for it;
5. a *Docs to update* line or a design doc the commits make untrue, and a
   public type or signature change missing from a commit body or from
   `CHANGELOG.md` §Unreleased.

**Standards.** Cite the rule (`.agents/rules/kernel.md`, this file, an
ADR) for each finding, and quote the hunk. Mark each **hard** (a stated
rule broken) or **judgement** (a smell worth a look).

Under 500 words in all; nothing found on an axis is reported as such.

## The standards

`.agents/rules/kernel.md` is the hard list: no panics on geometry,
provenance from every operation, determinism, tolerances from the model,
every failure a fixture with an oracle, exhaustive geometry enums, the
binding updated with a break, references read and never copied. Beside it:

### Geometry
- A tolerance widened, a fallback path or a special case added to make a
  fixture pass is a hard finding unless the plan's open questions decided
  it.
- A numeric constant is named, commented and lives in `arris-math`, with
  the reason it is not a tolerance (`POLYNOMIAL_ROUNDING` is the model).
- An iteration (Newton, a march, a fit) states its stopping rule and what
  it returns when it does not converge: a typed refusal, never the last
  iterate.
- A refusal names the entities and the reason, so a consumer can act on it.

### Tests
- A fixture's expected values come from the oracle or a closed form, never
  from Arris's own output; `measure_differs` carries ADR-0015's evidence.
- A changed `expected.json` or blessed dump says in its commit why the
  geometry changed.
- `prop_shards!` shards are seeded from the base seed and their index, and
  together run at least the configured case count.
- A test asserts through the public operation and the checker, not a
  private helper, wherever the operation reaches the case.

### API and docs
- Rustdoc says what an item guarantees and refuses, not how it works; an
  operation has a runnable example.
- A new public item that a consumer would use is bound in `arris-py` or
  named in the plan as deliberately unbound.
- Docs change in the commit that changes the behaviour, in present tense,
  without the plan's history; a `CHANGELOG.md` bullet speaks to a reader of
  `README.md` only.
- A term `GLOSSARY.md` defines is used as written; one it lists under
  _Avoid_ is a judgement finding. A new concept named in public API or a
  design doc has its glossary entry.

### Code smells (always judgement)
A name that does not say what it holds; the same logic twice in the diff
(two dispatch tables over the same pair kinds); a function reaching into
another module's data more than its own; parameters that always travel
together; an `f64` or `usize` standing for a concept that deserves a type;
abstraction or hooks for a case no fixture has.
