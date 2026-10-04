# Idea: failure-capsule-and-triage

- Status: Parked — the time to build it is when ArriX users start sending reports
- Raised: 2026-10-04
- Prompt (verbatim from the human): "write that review to new file in notes and /idea 1-2" — then "/plan option B"; on where triage output goes: "Not sure yet, it seems like I need to spin up some 24/7 server that would collect reports from different sources (Github CI, local runners, Arrix users) to db and we request those findings from here on-demand"; on scheduling: "Manually for now"; on versioning the recipe grammar: "As you wish". Parked 2026-10-05: "It's hardening infrastructure, and the time to build it is when ArriX users start sending reports."

## Problem

Every failure the harness finds (a property test, the differential, a fuzz
target, the real-part survey) is triaged by hand: read the nightly's log,
shrink, group by cause, write a `Findings` block. That works at the volume one
night produces. It stops working when failures arrive from many sources at
once (CI, local runs, a consumer's users), because there is no common,
replayable unit of "a failure" to group, de-duplicate or promote.

Nothing hurts yet beyond the hand-triage cost. The hurt arrives with the
consumer's reports.

## Constraints it runs into

- ADR-0024 §2 (classes and exclusions) and §6 (a red night posts nothing):
  both kept.
- ADR-0029: body bytes are the wire for a payload that no recipe rebuilds.
- ADR-0013 layering: everything lives in `arris-debug`, the top crate, which
  is `publish = false`, so no public API of a published crate changes.
- `.agents/rules/kernel.md` §Testing: every failure becomes a fixture with an
  oracle; a capsule is the step before that, never a replacement.

## Options

### A — Do nothing
Hand triage stays. Costs one agent session per red night and loses
de-duplication across sources. Fine until reports come from outside.

### B — Capsules, replay, triage, promote (the written plan)
A **capsule** is one versioned, self-contained JSON file: commit, source, a
**signature** and a payload that replays it.

- Signature: (source, operation, class, key). Operation is the last recipe
  step's op, the property's path or the fuzz target; class is the
  differential's `Outcome::class` or its analogue; key is the error variant
  path, the checker row, the disagreement's stage or a panic's `file:line`,
  never a number, an id or a message's free text.
- Payload kinds: recipe, body (body bytes plus operations), property (path,
  seed, shard, case count), fuzz (bytes plus target), part (sha256, solid id,
  battery stage).
- `arris-debug` gains `capsule` and the examples `replay` (verdicts: still
  fails with the same signature, fixed, fails differently), `triage`
  (read directories, group by signature, set aside what a finding, exclusion
  or regression fixture covers, replay the rest, draft `Findings` blocks) and
  `promote` (recipe or body capsule to a `regression/` fixture).
- The recipe grammar gets a `RECIPE_VERSION`, mirrored in
  `tools/oracle/oracle/recipe.py` and held equal by the corpus lint; replay
  refuses another version by name.
- `ARRIS_CAPSULE_DIR` makes `prop::check`, the differential and the survey
  write capsules on failure; `nightly.yml` uploads `capsules-<job>`.
- `docs/BACKLOG.md` §Findings gains `Signature:` and `Capsule:` lines.
- Cost: about nine commit-sized steps, grades [1] to [2], no [3]. The
  riskiest is the body payload: promotion writes the body as STEP for the
  `step` operand, and the reader's tolerances may heal or move the failure.
  Measure on the shard-13 drawn body and a body of each `regression/nist-*`
  fixture; a body that loses its signature stays a body capsule with its
  finding `measured`.
- Dry run proposed: the 10-03 nightly seed (`56c7709c…`) on the tree it was
  triaged on, compared with the hand-triaged list.
- Forecloses nothing; triage is run by hand, nothing is scheduled.

### C — A collecting server
A 24/7 service that gathers reports from CI, local runners and ArriX users
into a database, queried on demand. Right when consumer reports exist, wrong
before: it is one more directory source of the same capsule files, so B does
not foreclose it, and B is its prerequisite.

## Recommendation

Park. B is correct infrastructure and is the shape C would consume, but it is
hardening with no current consumer: the hand-triage cost is low until users
report. Build B (then C if volume asks) when ArriX users start sending
reports, folding in the later `field-reports` question of consumer-side
witness extraction.

What would change this: a nightly whose hand triage costs more than a session,
or the first consumer report arriving in a form that cannot be replayed.

## Decision for the human

1. Parked until ArriX users send reports. Preferred: yes (decided 2026-10-05).
2. When it is picked up: directories only, no server until consumer reports
   exist (agent's recommendation of 2026-10-04). Open until then.
3. ADR-0046 would record the capsule format, signature definition, recipe
   grammar version, replay verdicts and directories as the only source.

Open design questions carried over:

- If most bodies lose their signature through STEP, does a `regression/`
  fixture accept body bytes directly (a new recipe operand, Arris-only, the
  oracle fed the body's STEP)? Decided with the measured table; amends the ADR.
- Whether the nextest thread name is a reliable test path in every harness the
  workspace uses; fall back to `prop_shards!` passing the path explicitly.
- Non-goals if revived: migrations for old capsules (a fixture is migrated
  in-tree by the commit that changes the grammar), fixing the failures triage
  finds, scheduling the triage.
