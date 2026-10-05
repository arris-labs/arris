# Plan: multi-tool-boolean

- Started: 2026-10-05
- Milestone: C7, prismatic features (docs/ROADMAP.md §C7)
- Idea: none of its own — the ask is A8 of
  `docs/ideas/plugin-cad-consumer-asks.md` ("`cut`/`fuse` with N tools in
  one General Fuse"), scheduled by ADR-0047. The decisions are taken here
  and in ADR-0050.
- Idea (verbatim from the human): "/plan multi-tool-boolean"

## Goal

`arris_ops::cut_many(m, target, tools, control)` and
`arris_ops::fuse_many(m, bodies, control)` take one body and N tools (or N
bodies) through **one** General Fuse decomposition (ADR-0004) over all of
them: every face pair of two different operands whose boxes overlap is
intersected once, tool against tool included, every section vertex made
once and shared — the point where three faces of three operands meet among
them — every face split once by every section on it, each piece classified
against the other operands and kept by the N-ary selection table. The
result is the same solid the chained two-operand booleans give (volume,
area, counts), passes the checker, keeps the id of every entity no tool
touched, and carries one provenance record naming each tool: a tool's
surviving pieces `Generated` from that tool's entities, the target's from
the target's. `cut` and `fuse` become the one-tool case of the same code,
their results and dumps bit for bit what they are today. A plate with a
pattern of 100 holes is one call, one decomposition, measurably cheaper than
100 chained cuts. Bound in the facade and in Python, matched against Open
CASCADE's multi-tool `BRepAlgoAPI_Cut`/`Fuse` on a fixture corpus.

## Non-goals

- `common` of N operands, and the general "cells" output of a General Fuse
  (every region of the arrangement kept, labelled by the operands it lies
  in): a backlog line each, for the consumer that asks.
- Split by a plane: its own plan, after this one (it is a cut whose tool is
  a half-space and whose both sides are kept, and will reuse this plan's
  operand-indexed decomposition).
- NURBS operands: refused as today (`OpError::Unsupported`), the NURBS
  cycle's.
- A tool that is a sheet or a wire body: the healing cycle's sheet bodies.
- A spatial index over faces beyond what step 7's measurement asks for; the
  query cycle builds the general one.
- Per-face tessellation (A9): the side plan.

## Design deltas

- **`arris-ops`, public**: `pub fn cut_many(m: &mut Model, target: Body,
  tools: &[Body], control: &Control<'_>) -> Result<(Body, Provenance),
  OpError>` and `pub fn fuse_many(m: &mut Model, bodies: &[Body], control:
  &Control<'_>) -> Result<(Body, Provenance), OpError>`, re-exported beside
  `cut`/`fuse`. `cut(m, t, tool)` is `cut_many(m, t, &[tool])` and `fuse(m,
  a, b)` is `fuse_many(m, &[a, b])`; `common` stays two-operand over the
  same decomposition. Refusals: `BooleanReason::NoTools` (an empty `tools`,
  fewer than two `bodies`) and `BooleanReason::RepeatedOperand` (one body
  twice, the target among its tools), naming the body. Each a
  `CHANGELOG.md` `### Breaking` bullet (`BooleanReason` grows, an
  exhaustive `match` adds its arm) and the binding's arm in the commit that
  adds it (kernel.md §API).
- **`arris-ops`, public, the decomposition**: `Interferences { a, b, .. }`
  becomes `Interferences { operands: Vec<Body>, .. }`; `FacePair` gains the
  two operand indices; `EdgeImage::side: usize` becomes `operand: usize`
  with `on: usize`, the operand it lies on; `interferences(m, a, b)` keeps
  its signature and `interferences_many(m, &[Body])` is added. Closes the
  backlog line "The boolean's `side: usize` and `[_; 2]` operand pairs" in
  the form it asked for: operand-indexed, not `enum Operand { A, B }`, and
  the `Option<bool>` selections become `enum Selection { Drop, Keep,
  KeepReversed }`. Named in each commit body and a `### Breaking` bullet.
- **`arris-ops`, crate-private**: `pave::Build`'s `faces`/`edges: [_; 2]`
  become `Vec` by operand; the face-pair search runs over every pair of
  operands with overlapping boxes, operand order outer; `result::Op` is
  read through an N-ary table — a piece of operand `k` is classified
  against every other operand whose box holds its point, and kept by
  (`k` is the target or a tool, the set of operands it is inside). The
  traced-section region (`pave::region`) becomes per operand pair, so a
  pair's curve stays bit for bit what the two-operand build gives it
  (ADR-0018).
- **The triple point** (new geometry): on a face of operand `i`, section
  curves of two pairs `(i, j)` and `(i, k)` with `j ≠ k` cross where no edge
  of any operand pierces — two overlapping holes' circles on a plate's top.
  Found as the section curve of `(i, j)` against the face of `k`, in the
  arm `hits` already has for an edge's curve against a surface, kept when
  inside all three faces; merged with the section vertices by the
  existing tolerance components, so the one vertex paves all three section
  curves (`(i, j)`, `(i, k)`, `(j, k)`). A new `VertexSource::TriplePoint`.
- **Topology and provenance** (docs/DATA-MODEL.md §Provenance): unchanged in
  kind. A cut's body `Modified` from the target's; a fuse's from every
  operand's, each result shell from the operand shells it holds pieces of;
  each tool's entities `Deleted`, its surviving pieces `Generated` from it;
  a section edge `Generated` from both faces of its pair, a triple point
  from the three faces. Split order (ADR-0009): operands in the caller's
  order, then as today — so the ids are a function of the tool order, and
  a property holds the geometry independent of it.
- **Recipe grammar** (both interpreters, `arris_debug::fixtures` and
  `tools/oracle/oracle/recipe.py`): `cut target <name>, tools [<name>, ...]`
  beside `tool <name>`, and `fuse bodies [<name>, ...]` beside `a`/`b`;
  and a `pattern` op — `pattern of <name>, step [x,y,z], count n` and
  a second `step`/`count` for a grid — which yields the n copies as a list
  a `tools` or `bodies` field takes, so a 100-hole fixture is three lines.
  Open CASCADE is driven by `BRepAlgoAPI_Cut`/`Fuse` with `SetArguments` and
  `SetTools` over lists.
- **Facade and binding**: `arris::ops::{cut_many, fuse_many}` by the
  re-export; `Model.cut_many(target, tools)` and `Model.fuse_many(bodies)`
  in `arris-py` with cancel and budget, their stubs and docstring examples;
  `NoTools` and `RepeatedOperand` as Python classes.
- **ADR-0050** — the multi-tool boolean: one decomposition over N operands
  rather than a fused tool or a chain (and why: the tools' own overlaps are
  sections of the one arrangement, provenance per tool, cost), the operand
  index as the public shape, the N-ary selection table, the triple point,
  the per-pair traced region, ids as a function of the tool order (step 1).

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — The oracle and the grammar. `tools`/`bodies` lists and
  the `pattern` op in `recipe.py` (multi-tool `BRepAlgoAPI_Cut`/`Fuse`) and
  in `arris_debug::fixtures`, the Rust runner refusing a list of more than
  one until step 3. Fixtures under `regression/` with `expected.json` and
  their closed forms, each with the chained-cut result's numbers recorded
  beside Open CASCADE's multi-tool answer so a disagreement between the two
  on Open CASCADE's side shows here: a plate with a 2×2 and a 10×10 grid
  of disjoint holes, a plate with bosses fused in one call, a slot of three
  overlapping holes in a row, two crossing rectangular pockets, three holes
  through one common point. ADR-0050 written here.
- [x] Step 2 **[2]** — Operand-indexed decomposition, no behaviour change:
  `[_; 2]` and `side: usize` to `Vec` by operand and operand indices in
  `Build`, `Interferences`, `FacePair`, `EdgeImage` and `result`; `enum
  Selection`; the per-pair traced region; `interferences_many`. Every
  existing corpus fixture, dump, `interferences` doctest and property
  unchanged bit for bit; the arris-debug interference dump and draw take the
  new shape. Closes the backlog line.
- [x] Step 3 **[2]** — Disjoint tools: `cut_many` and `fuse_many` where no
  two tools' faces have overlapping boxes, so every pair is target × tool;
  the N-ary selection with the box prefilter on classification; `cut` and
  `fuse` routed through them; `NoTools`, `RepeatedOperand` and the binding's
  arms. Fixtures move to `boolean/` and are blessed: the 2×2 and 10×10 hole
  grids, the fused bosses, a pattern on a cylinder wall (radial holes), a
  tool that misses the target (`Deleted` whole, the target's ids all kept).
  Landed as found: the cylinder-wall fixture is two square pockets
  (`radial-pockets-cut-many`), since round radial holes meet the wall in
  saddle curves whose B-spline read-back stage of the corpus run takes 35 s
  for two (the round case is a test against the chain, `multi_tool.rs`);
  the 10×10 grid's corpus run is 170 s, about 1.7 s a hole of that same
  read-back, so `boolean_plate_10x10_holes_cut_many` joins the slow set
  (ADR-0032); tools meeting each other are refused as `Unsupported` until
  step 4; the `NoTools`/`RepeatedOperand` refusals need no binding arm
  (`arris-py` carries a boolean reason as its text).
- [x] Step 4 **[3]** — Overlapping tools: tool × tool pairs, the triple
  point on a face of a third operand, and the classification of a tool's
  piece against the target and every other tool. Fixtures: the slot of three
  overlapping holes, the two crossing pockets, the three holes through one
  point, a counterbore cut as two coaxial cylinders of different radii in
  one call (a tool's cap inside another tool), fuse of three mutually
  overlapping cylinders (a tripod).
  Landed as found: the crossing pockets' floors are coincident with each
  other, so the coincident row between two tools of a cut (the fuse's,
  kept once from the earlier tool) landed here rather than in step 5; the
  three bores and the tripod carry Arris's counts under `counts_differ`,
  Open CASCADE cutting each pair of equal cylinders' ellipse at its
  parameter origin as in `cross-cylinders-common` (Arris's one call equals
  its own chain).
- [ ] Step 5 **[3]** — Coincident and tangent between tools: two tools flush
  on a face (stacked boxes), two tools both flush with the target's face
  (two pockets opening on one top face that touch along an edge), a tool
  repeated by value — a second body of the same geometry, every face
  coincident — tangent tools (two cylinders touching along a ruling inside
  the target: `TangentContact` where both pieces survive, as the
  two-operand rule says), and coincident tool faces inside the target.
  Fixtures for each, the refusals among them named.
- [ ] Step 6 **[1]** — The facade and the binding: `Model.cut_many` and
  `Model.fuse_many` with cancel and budget, stubs, docstring examples, a
  pytest per refusal; the rustdoc example on both functions; the cancel
  step counts of the two-operand fixtures unchanged, the multi-tool ones
  recorded.
- [ ] Step 7 **[2]** — Cost: the corpus benchmark gains the 10×10 grid as
  `cut_many` and as 100 chained `cut`s, timed apart; `cut_many` must be the
  faster, and the measured ratio is recorded in ADR-0050. If the face-pair
  search or the classifier's per-piece ray against every tool dominates,
  a sort-and-sweep over face boxes (and the box prefilter of step 3 for
  the classifier) is added here, its result bit for bit the quadratic
  search's.
- [ ] Step 8 **[2]** — Properties, sharded and seeded: `cut_many` against
  the chained `cut`s and `fuse_many` against the chained `fuse`s (volume,
  area, counts) over random tools in random poses on boxes, cylinders and
  rounded boxes, overlapping and not; the result's geometry (volume, area,
  counts) independent of the tool order; `cut_many` commutes with
  `transform`; the result is an operand of fillet, chamfer, `offset_faces`,
  `shell` and every boolean, checker green; `prop::recipe` and the
  differential draw `cut`/`fuse` with two to four tools.

## Acceptance

`ARRIS_GATE=full` green; every two-operand `boolean/` fixture's dump
unchanged; the new `boolean/` fixtures (the fifteen-odd named above)
checker-green at `Full`, matched to Open CASCADE's volume, area, centroid,
inertia, counts and probes, or to their closed forms where ADR-0015's
evidence is recorded; every refusal fixture fails with its named reason;
step 8's properties at 256 cases; the differential at 1000 recipes with the
multi-tool booleans drawn and no new unexplained disagreement; step 7's
benchmark showing `cut_many` faster than the chain on the 10×10 grid; the
`python` job's pytest, docstring examples and `mypy.stubtest` green.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Operations — the boolean over N operands: the
  operand-indexed decomposition, the N-ary selection table beside the
  two-operand one, the triple point, the per-pair traced region; §Errors —
  `NoTools`, `RepeatedOperand`; the capability table (a "Cut or fuse many
  tools at once" row); §Formats and tools — the benchmark's new row.
- `docs/DATA-MODEL.md` §Provenance — what a multi-tool boolean records (per
  tool, the triple point, split order by operand).
- `docs/adr/0004-…` — no edit (append-only); ADR-0050 cites it.
- `tests/fixtures/README.md` — `tools`/`bodies` and `pattern` in the
  grammar.
- `tools/oracle/README.md` — how the multi-tool boolean is driven.
- `docs/ROADMAP.md` §C7 — status line: the multi-tool boolean landed.
- `docs/BACKLOG.md` — remove the operand-indexing line (closed by step 2);
  add N-ary `common` and the General Fuse's cells output; whatever refusal
  the corpus shows is common.
- `CHANGELOG.md` `Unreleased` — what a consumer can now do (a pattern cut
  or fused in one call, provenance per tool) and the refusals they will
  hit; `### Breaking` holds the bullets the steps wrote.
- `AGENTS.md` current state — the multi-tool boolean done, split by a plane
  next.

## Open questions

- ⚠ OPEN: Does Open CASCADE's multi-tool `BRepAlgoAPI_Cut` agree with its own
  chained cuts on overlapping tools and on the flush cases of step 5, or
  does it differ (fused coplanar faces, a refusal)? Agent decides at step 1
  from its runs; where it differs the fixture is held to the chained result
  or its closed forms, recorded in ADR-0050.
- Resolved at step 2: the per-pair traced region keeps every two-operand
  dump bit for bit (the corpus, provenance and boolean tests ran unchanged
  with the region computed per operand pair), so ADR-0050 §6 stands as
  written.
- Resolved at step 4: a triple point at a tool's edge needs no case of its
  own — a blind bore whose rim runs through four triple points of three
  orthogonal bores merges each with the rim's hits by the tolerance
  components, the result its chain's (ADR-0050, landed with step 4).
