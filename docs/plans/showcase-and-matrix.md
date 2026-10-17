# Plan: showcase-and-matrix

- Started: 2026-10-17
- Milestone: beside C10, in the side slot (docs/ROADMAP.md §Beside the cycles)
- Idea: docs/ideas/developer-first-positioning.md (its credibility plan
  and Q5's ADR absorbed; the script corpus, the modelling layer and the
  MCP server stay there)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
A developer who opens the repository sees within ten seconds a real part,
the short Python script that built it, and its faces coloured by the
operation that made them. They also find an honest, test-held table of
what each operation does and refuses for each face kind. The README and
COMPARISON.md say what `main` does, sweep, loft and NURBS-against-analytic
included. The table is a test's output, never a hand-kept claim. The
showcase is a script CI runs, with its STEP read back by Open CASCADE and
held to its values. ADR-0067 records the developer-first audience
(developer-first-positioning, Q1 and Q2).

## Non-goals
- The script corpus, the modelling layer, the MCP server, the JS binding:
  later plans (the idea, in order).
- C10's roadmap edits (the corpus renamed, draft and thicken moved out):
  the script-corpus plan's first step.
- A browser playground (rejected, `docs/BACKLOG.md`); any launch post (the
  human's).
- New kernel operations. A refusal the showcase part hits becomes a
  `regression/` fixture and a backlog line, and the part changes; the
  kernel is not fixed here.
- Rendering as a product: the renderer stays in `arris-debug`
  (`publish = false`), and ADR-0034 §5 (no PNG in the wheel) stands.

## Design deltas
- **ADR-0067, the audience.** Developers first: code-CAD script authors
  and agents, then CAE pipelines. The plugin CAD is one consumer: its
  regressions rank first (ADR-0020 §2), its feature asks do not. The
  modelling layer ships in the wheel as pure Python (`arris.build`),
  amending ADR-0034 §1. It selects by provenance and role first, and its
  values are immutable and chained. It also records the launch gate and the
  claims never made.
- **The capability matrix** (`crates/arris/tests/capability.rs`). One
  representative case per cell builds through the facade:
  - rows: cut, fuse, common, split, section, fillet, chamfer, offset,
    shell, sweep operand, loft operand, ray, distance, STEP round trip;
  - columns: the face kind met (plane, cylinder, elliptic cylinder, cone,
    sphere, torus, NURBS);
  - a pairwise row meets the kind with a plane-faced box, or with a
    cylinder in the plane column.

  Each cell prints `ok` (checker `Full` green) or the refusal's kind.
  The table is a blessed text file (`capability.txt`, as
  `tolerance_band.txt` is) and the test fails on any drift.
  COMPARISON.md embeds it, and a docs test holds the copy equal to the file.
- **A labelled render** in `arris-debug`: `render_labelled(mesh, labels,
  palette, view, path)` colours triangles by a per-face label, not a face
  id. It uses Lambert shading and 2× supersampling, so a reader sees form.
  The flat agent render stays as it is. A small example binary
  (`cargo run -p arris-debug --example render_labelled`) reads the mesh
  bytes the binding writes, plus a labels JSON. `arris-debug` is
  unpublished, so no consumer sees a signature change.
- **The showcase part** (`crates/arris-py/examples/bracket.py`, ~20 lines
  of modelling): a motor-mount bracket.
  - Built as: an extruded L profile, a boss fused on, a bolt pattern and
    bore cut in one multi-tool call, counterbores, fillets on the inside
    bend and hole rims, a chamfer on the boss.
  - Each final face is labelled by the operation that generated it,
    followed through `modified` records.
  - It writes STEP, mesh bytes and the labels.
  - `tests/test_showcase.py` runs it in CI's `python` job: checker `Full`
    green; volume, area and centroid matched to the oracle's reading of
    its STEP, committed beside it (`bracket.oracle.json`, written by a
    `tools/oracle/` script).
- **`tools/showcase.sh`** regenerates `docs/assets/showcase.png` from the
  script and the example. It is deterministic, so a rerun diffs empty.
- **COMPARISON.md**: its Arris column brought to `main` (version `0.9.0`
  heading, ADR-0061 to 0065), the matrix embedded, and three claims added
  under "What is not claimed":
  - "solves topological naming": provenance is what a naming scheme
    needs, and the scheme is the application's (ADR-0009);
  - any wasm size compared with another kernel: never measured;
  - any share of mechanical parts the analytic kinds cover: no source.
- **README.md**: the image and the script at the top; one sentence saying
  who it is for (code-CAD scripts, agents, CAE pipelines); Features and
  "Not yet" synced to `main`; a link to the matrix.

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — The showcase part. `bracket.py`, its oracle script
  (`tools/oracle/showcase.py`) and values, and `test_showcase.py`. This goes
  first because the part may hit a refusal. It hit none: the bracket stands
  as designed. Any refusal becomes a `regression/` fixture and a
  backlog line, and the part changes.
- [ ] Step 2 **[2]** — The capability matrix: `capability.rs` and its
  blessed `capability.txt`. Each `ok` cell is checker-green, and each
  refusal is named. A cell that faults, rather than refusing, becomes a
  regression fixture.
- [ ] Step 3 **[2]** — `render_labelled` with shading and supersampling,
  plus the example binary. Its test: each label's colour is present in
  the pixels, and the buffer's digest is stable across two runs.
- [ ] Step 4 **[1]** — `tools/showcase.sh` and `docs/assets/showcase.png`.
  Read the PNG: every label is visible from the chosen view.
- [ ] Step 5 **[1]** — ADR-0067. Also: a note on ADR-0034 §1 that ADR-0067
  amends it, and the modelling layer's line under ROADMAP "Beside the
  cycles".
- [ ] Step 6 **[1]** — COMPARISON.md synced: the matrix embedded with its
  docs test, the three non-claims, and the version heading.
- [ ] Step 7 **[1]** — README: the image and script, the audience
  sentence, Features and "Not yet" synced, the matrix linked.

## Acceptance
- CI's `python` job runs `test_showcase.py` green: the bracket is
  checker `Full` green, and its volume, area and centroid match
  `bracket.oracle.json` within the part's tolerance.
- `cargo nextest run -p arris --test capability` matches `capability.txt`
  cell for cell, and every `ok` cell is checker-green.
- The docs tests hold COMPARISON.md's matrix equal to `capability.txt`.
- `tools/showcase.sh` run twice leaves `git status` clean.
- The full gate passes at retirement.

## Docs to update on completion
- `docs/ideas/developer-first-positioning.md`: cut down to the script
  corpus, the layer and the MCP server; its status names this plan.
- `docs/ROADMAP.md` §Beside the cycles: the showcase and the matrix, one
  sentence each.
- `docs/ARCHITECTURE.md` §Formats and tools: `render_labelled` and
  `tools/showcase.sh` beside the inspect renderer.
- `AGENTS.md` current state: the audience (ADR-0067), one line.
- `.agents/skills/inspect`: `render_labelled`, one line.
- `CHANGELOG.md`: nothing; no consumer-visible change.

## Open questions
- Settled at step 1: the part is the bracket above; nothing refused.
- ⚠ OPEN: the image's view and palette. The agent decides at step 4.
  The human may veto the image before step 7 commits it to the README.
