# Plan: script-corpus

- Started: 2026-10-18
- Milestone: C10's corpus line (docs/ROADMAP.md §C10, "In"; the DIY part corpus renamed)
- Idea: docs/ideas/developer-first-positioning.md (its script corpus absorbed; Q3 and Q8 settled, ADR-0067)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
About 15 hobby parts and 3 CAE parts are Python scripts on the raw binding,
written the way a script author or an agent writes them, and committed with
what each stage did: built, or the typed refusal it met. Every stage a script
reaches is checker-green and matches Open CASCADE's build of the same part at
that stage; every refusal is on record. `tools/script-corpus.sh` prints the
corpus's refusal histogram beside the real-part one, by operation and by the
cycle each refusal waits on, and that histogram is what C10's close ranks C11
with. Where the raw API made a script clumsy, the friction is written down as
the evidence for the modelling layer's scope.

## Non-goals
- The modelling layer (`arris.build`): its own plan, after this one. The
  scripts are ported to it then.
- Lifting any refusal the corpus meets: each is a histogram row, and a
  kernel fault is a regression fixture, not a fix here.
- The MCP server, the JS binding, any render of the parts.
- New oracle operations: a part that needs an op `oracle/recipe.py` lacks
  is replaced by one that does not, and the op is a backlog line.

## Design deltas
- **Layout.** `crates/arris-py/corpus/` (outside `python/`, so not in the
  wheel): `harness.py`, and per part `<slug>/part.py` (the script),
  `<slug>/oracle.json` (the same part as an oracle recipe, `tests/fixtures/README.md`
  grammar), `<slug>/expected.json` (written by `expected.py`, never by hand)
  and `<slug>/outcome.json` (what each stage did, blessed). Slugs are
  kebab-case nouns (`spool-holder`), never numbered.
- **The script's contract.** `part.py` defines `build(run)` and calls each
  operation as `run.stage("<name>", lambda: model.<op>(...))`; the stage
  name is the recipe step's name it is matched against. A refused stage
  ends the part, except one marked `finish=True` (a fillet, a chamfer, a
  cosmetic hole), whose refusal is recorded and the part goes on with the
  body before it. Selection is by provenance and geometry queries, as a
  script author would; never by index into `faces()`.
- **`outcome.json`**: per stage, `"ok"` or the refusal as the binding raises
  it: the exception class, and `reason` (`OpDegenerateError`), the two kinds
  (`*UnsupportedError`) or `detail`. A stage the part never reached is
  `"not reached"`. `OpInternalError`, `ModelPoisonedError`, any non-Arris
  exception and a checker-red stage are **faults**, never recorded as
  outcomes: the test fails until the fault is a `regression/` fixture and
  the part is listed in `crates/arris-py/corpus/waits` (`tools/real-parts.waits`'s
  format and rules).
- **The oracle.** `expected.py` learns `"checkpoints": [step names]` on a
  recipe: it measures each named step as it measures `result` and records
  them under `checkpoints` in `expected.json`. A checkpoint Open CASCADE
  cannot build records `{"refused": why}` as a battery case does. Edge and
  face picks in the recipe are by point, as now; the script picks by
  provenance, so the two meeting is itself checked.
- **Per-part tolerances.** `oracle.json` may carry `tolerances` in the
  fixtures' grammar, over the `conftest` defaults, where Open CASCADE's own
  approximation is the limit (a blend's torus or a helical sweep, as
  `sweep-thread`'s fixture sets): the knob 2e-8, the bottle cap 1e-6. The
  harness leaves undecided only the face pairs of two NURBS faces, as the
  corpus runner does (`NURBS_PAIR`), and faults on any other row.
- **The vase is a ruled loft**: its smooth loft (four circles of unequal
  radius and spacing) differs from Open CASCADE's by 8e-4 of the volume,
  a finding kept as a backlog line, not a tolerance.
- **A recipe's `result` is a step Open CASCADE's STEP round trip keeps**
  (`selftest.py` holds it to that): the vase's `result` is `base_round`,
  its `hollow` (valid in Open CASCADE, yet written to STEP as two shells)
  a checkpoint only.
- **The test.** `crates/arris-py/tests/test_script_corpus.py`, one case per
  part: run it, compare against `outcome.json` (a changed outcome fails;
  `ARRIS_BLESS=1` rewrites it, and a lifted refusal is a `fixtures:` commit),
  `check(..., "full")` every reached stage, hold each checkpoint to
  `expected.json` at the fixture defaults (`conftest`), and run the part
  twice for identical STEP bytes. It runs in CI's `python` job; the oracle
  does not run in CI's python job, `expected.json` is committed.
- **The histogram.** `arris_debug::histogram` gains `add_script(part,
  stage_op, outcome)`, mapping a recorded refusal to its cycle by name
  through the table `blocks_reason` already holds (the mapping by name is a
  new `pub fn` beside it, so both tables stay one). An `arris-debug` example
  `script_corpus` reads every `outcome.json`; `tools/script-corpus.sh` runs
  it and writes `target/script-corpus/histogram.md`, the first refusal per
  part by operation and by cycle, and every refusal beside it.
- **CAE parts** tag faces through provenance (the showcase's `Labelled`
  pattern lifted into `harness.py`) and assert their tagged sets survive
  every cut and reach `tessellate`'s per-face ranges.
- **The parts** (agent's pick; swapped freely while each stays a part a
  hobbyist prints or a CAE user meshes): prismatic — wall hook, Pi mount
  plate, enclosure with screw bosses (`shell`), hinge leaf, phone stand;
  revolved — knob, spool holder, hose adapter, bottle cap with a helical
  thread (`sweep`); swept and lofted — drawer handle, cable clip, vase
  (`loft`), funnel; patterned — spur-gear blank, GoPro-style mount. CAE —
  a duct with an obstacle (inlet, outlet, walls, obstacle tagged), a
  finned heat sink (base and fins tagged), a cantilever (fixed face, load
  face tagged).
- No public type or signature of the binding or the kernel changes.
  `GLOSSARY.md` already holds *Script corpus*.

## Steps
Complexity grades: **[1]** routine — the design says exactly what to write
and the tests are mechanical; **[2]** careful — a geometric or numeric case
to get right within a given design; **[3]** unproven — an algorithm whose
robustness or bound has to be established here.

- [x] Step 1 **[2]** — `expected.py`'s `checkpoints` (with its selftest
  case), `harness.py`, `test_script_corpus.py`, and the first part end to
  end: the enclosure, whose `shell` and boss fillets are the likeliest to
  show script and recipe disagreeing on what was picked.
- [x] Step 2 **[2]** — The four other prismatic parts.
- [x] Step 3 **[2]** — The four revolved parts, the threaded cap's helix
  sweep among them.
- [x] Step 4 **[2]** — The four swept and lofted parts.
- [x] Step 5 **[2]** — The two patterned parts.
- [ ] Step 6 **[2]** — The three CAE parts, tags lifted into `harness.py`
  and asserted through every cut and the mesh.
- [ ] Step 7 **[2]** — `add_script` and its by-name cycle mapping (unit
  tests over every recorded class), the `script_corpus` example and
  `tools/script-corpus.sh`; the histogram printed.
- [ ] Step 8 **[1]** — The friction list: per part, each place the raw
  binding made the script longer than a layer would, written into the
  idea's remaining part as the layer plan's input.

A fault found in steps 1–6 is shrunk into `tests/fixtures/regression/` and
listed in `waits` in the step that met it (the `inspect` skill), never fixed
there.

## Acceptance
`pytest crates/arris-py/tests/test_script_corpus.py` passes over 18 parts:
every reached stage checker-green at `Full` and within the fixture defaults
of its `expected.json` checkpoint, every outcome equal to the committed one,
two runs byte-identical, every fault a listed wait with its `regression/`
fixture. `uv run --project tools/oracle tools/oracle/selftest.py
crates/arris-py/corpus/*/` reproduces every `expected.json`.
`tools/script-corpus.sh` prints the histogram, and the plan's retirement
copies its table into C10's section.

## Docs to update on completion
- `docs/ROADMAP.md` §C10 — the corpus line says *script corpus*, marked
  done with its histogram; the Accept's "DIY part corpus" likewise.
- `docs/ARCHITECTURE.md` — the crate table's `arris-debug` row
  (`script_corpus`), and the corpus section: the script corpus, its
  layout, its oracle and its waits.
- `tests/fixtures/README.md` and `tools/oracle/README.md` — `checkpoints`.
- `docs/ideas/developer-first-positioning.md` — status: the script corpus
  done; the friction list kept for the layer plan.
- `AGENTS.md` current state — the corpus landed, its first refusal row.
- `CHANGELOG.md` — nothing: no consumer-visible change.

## Open questions
None blocking. The parts list is the agent's pick (above); the human may
swap any part before step 2.
