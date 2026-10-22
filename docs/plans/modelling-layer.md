# Plan: modelling-layer

- Started: 2026-10-22
- Milestone: the modelling layer's first milestone, beside C11 (docs/ROADMAP.md §Beside the cycles)
- Idea: docs/ideas/developer-first-positioning.md (its modelling-layer part absorbed; Q6–Q8 settled, ADR-0067). The script corpus's friction list is at commit 76d42508.

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal

`arris.build` ships in the wheel as pure Python over `arris._arris`
(ADR-0067 §2–4). A script author builds a solid from immutable values with
chained methods, on workplanes that are frames. They select faces and
edges by provenance, role and tag first, then by geometric filters that
return sets in a deterministic order. No selector picks by index. Patterns
and through-holes need no hand-written trigonometry or oversized tools.
Tags put on faces survive every operation, for CAE boundaries. The
showcase bracket and four script-corpus parts are rewritten on the layer
and held to the same Open CASCADE checkpoints as their raw versions. The
README's first snippet uses the layer. This is the last item ADR-0067 §5
gates the launch on.

## Non-goals

- A sketch constraint solver (`SEED.md` §4); sheet and wire values,
  assemblies, text, draft and thicken; the MCP server (its own plan next).
- The other 14 corpus parts (a backlog line), any Rust crate, any kernel
  change beyond step 1's read-only queries, builder contexts or positional
  selectors (ADR-0067 §3–4).

## Design deltas

- **Binding queries (step 1, public Python API)**, replacing the corpus's
  tessellate-to-select (`harness.edge_points`): `Model.point(vertex)`;
  `Model.curve(edge)` → `CurveInfo` (`kind`, end points, the closed form's
  parameters where one exists); `Model.surface(face)` → `SurfaceInfo`
  (`kind`, plane origin and normal, quadric and torus axis, centre and
  radii); `Model.bounds(entity)`, a containing box, loose for NURBS. Each
  has its stub line, docstring example and `CHANGELOG.md` bullet.
- **Values.** `arris.build.Solid` is frozen: its `model`, `body` handle,
  `provenance` (the chain by `Provenance.then`, from the constructor's own
  record), `last` record and `tags`. Constructors: `Solid.of((body,
  record))` over any binding result, `box` and `cylinder`, which with no
  `model=` use `default_model()`; mixing models raises the binding's
  `ForeignHandleError`. `apply((body, record), tools)` follows an operation
  the layer does not wrap, and every wrapped one goes through it.
  `cut(*tools)` (`cut_many` past one), `fuse(*others)`, `common`, `fillet`, `chamfer`,
  `shell`, `moved` and `mirrored` each return a new `Solid`. A kernel
  refusal reaches the caller as the binding's typed exception, unchanged.
  Queries: `measure`, `check`, `to_step`/`export_step`, `mesh`.
- **Selections.** `solid.faces()` and `solid.edges()` return a
  `Selection`: a frozen set in ascending handle-index order (deterministic
  by the kernel's ids). Provenance first: `created_by(tool)`,
  `modified_by(tool)` through the history, `from_role(Role)`,
  `tagged(name)`, `last()`. Then geometry: `kind`, `normal`, `parallel`,
  `perpendicular`, `extreme(dir)` (the set at the maximum, ties within
  tolerance) and `within(box)`. Set algebra `|`, `&` and `-`; adjacency
  `edges()` and `faces()`. No `__getitem__`: `one()` returns the single
  member or raises `SelectionError` (an `ArrisError`) naming the chain and
  the count.
- **Workplanes.** `Workplane` is a frame (`arris.Frame` semantics): the
  constructors `XY()`, `YZ()` and `XZ()` with an offset, `face.workplane()`
  on a planar face (one-element selection), `offset(d)`, and `rotated`.
  2D builders return a `Sketch` in the plane's coordinates: `rect`,
  `circle`, `polygon`, `slot`, and a path of `line_to`, `arc_to` (via a
  tangent, with the via-point computed) and `close`. The sketch then makes
  a solid with `extrude(d, both=False)`, `revolve(axis, angle)`,
  `sweep(path, law)` or `loft(sketches)`. A sketch is turned into a
  `Profile` on its way to an operation, never earlier.
- **Placements and features.** `Workplane.at(points)`, `polar(n, radius,
  start)` and `grid(nx, ny, dx, dy)` give located workplanes. Then:
  - `solid.hole(locs, diameter, depth=…|"through")` and
    `solid.pocket(sketch, depth=…|"through")` size a through tool from
    `Model.bounds` along the plane's normal, with a margin of the model's
    tolerance times a named constant, never a literal.
  - `solid.boss(sketch, height)` fuses.
  - A pattern of tools goes through one `cut_many` / `fuse_many`.
- **Tags** generalise the bracket's `Labelled` (`examples/bracket.py`).
  `solid.tag(name, faces)` returns a solid whose tag set follows each
  operation's `modified`, then `generated`, records. Faces cut by the
  operation keep the tag, and a face the operation deleted drops it.
- **Glossary and types.** *Workplane* and *Selection* are new entries; a
  *Tag* is the layer's, in Python only, and is distinct from the attribute
  cycle's attributes (ADR-0009). Typed inline (`py.typed` already ships):
  `mypy --strict` and `test_docstrings.py` cover `arris/build`.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — The binding queries `point`, `curve`, `surface` and
  `bounds`, with stubs, docstrings and the changelog. Tests: on the
  primitives and on a swept NURBS face, each query is checked against the
  construction's parameters, and `bounds` contains `tessellate`'s
  positions.
- [x] Step 2 **[2]** — `Solid`, the history and tags. Test: the bracket's
  labels, rebuilt through `Solid.tag`, equal `examples/bracket.py`'s
  per-triangle labels exactly. Tags survive the property-test cut/fuse
  sequence on random boxes, and a deleted face drops its tag.
- [x] Step 3 **[2]** — `Selection`, its filters and `SelectionError`, plus
  the glossary's *Selection*. Tests: each filter on the primitives, order
  stable across two runs, `created_by` after a `cut_many` returning exactly
  the tool walls, and `extreme` with coplanar ties returning the whole set.
- [x] Step 4 **[1]** — `Workplane` and `Sketch`, with `extrude`, `revolve`,
  `sweep` and `loft`, plus the glossary's *Workplane*. Tests: each
  construction matches the raw call's body (mass properties equal) on a
  workplane rotated off the axes.
- [ ] Step 5 **[2]** — Placements, `hole`, `pocket`, `boss` and patterns.
  Tests: a through-hole in a plate tilted 30° has the oracle's volume, and
  a 24-tooth polar pattern equals the raw corpus gear's stage body.
- [ ] Step 6 **[1]** — `fillet`, `chamfer`, `shell`, `moved`, `mirrored`,
  `measure`, `check`, `to_step` and `export_step` on `Solid`, each with a
  docstring example. `mypy --strict` and the docstring collection are added
  to CI's `python` job.
- [ ] Step 7 **[1]** — The showcase on the layer: `examples/bracket.py`
  rewritten, held to `bracket.oracle.json`, with `tools/showcase.sh`'s PNG
  byte-identical. The README's snippet is swapped for the layer's, and the
  raw-binding snippet moves under "The binding beneath".
- [ ] Step 8 **[2]** — Four corpus parts ported as `corpus/<slug>/build.py`
  (enclosure: selection and shell; spur-gear-blank: polar pattern;
  cantilever: CAE tags; duct-obstacle: through-cut and tags). The harness
  runs both scripts against the same `expected.json` checkpoints and the
  same `outcome.json`. The line counts before and after are recorded in
  the corpus README.

## Acceptance

`pytest crates/arris-py` is green in CI's `python` job (3.10 and 3.13).
That covers the showcase and the four ported parts at every stage
checkpoint, held to Open CASCADE at the fixture defaults, with their
outcomes equal to the raw scripts' (refusals included). It also covers the
tag property test and every `arris.build` docstring example. `mypy --strict`
is clean over `arris/build` and `mypy.stubtest` over the new binding
queries. `tools/showcase.sh` reproduces `docs/assets/showcase.png`
byte-identical from the layer's bracket.

## Docs to update on completion

- `docs/ROADMAP.md`: §Beside the cycles, the modelling layer, as landed;
  C11's "In" line no longer mentions a parked plan.
- `docs/ARCHITECTURE.md` §the binding: `arris.build` above `_arris`, the
  default model, the selection order rule.
- `GLOSSARY.md` (*Workplane*, *Selection*, possibly *Tag*: steps 3–4);
  `CHANGELOG.md` `Unreleased` (the layer); `README.md` (step 7); the
  corpus harness docstring (two scripts per ported part).
- `docs/BACKLOG.md`: the other 14 corpus parts; sheet values.
- `AGENTS.md` current state: the layer landed, and ADR-0067 §5's launch
  gate met once `python-wheels` has retired too.

## Open questions

- *Settled (step 2)*: **the value's name** is `Solid`: `Body` would shadow
  the handle `arris.Body`, which a `Solid` holds as `.body`. Its chain is
  `provenance`, not "history" (the glossary's _Avoid_).
- *Settled (step 2)*: **tags follow `modified`, then `generated`**: a face
  with a face origin in `modified` takes the tags of those origins only,
  else of its `generated` face origins; a face neither names keeps its own.
  This reproduces the bracket's single labels exactly, so the layer keeps
  tag sets (a face may carry several). `faces()` returns a list until
  step 3 makes it a `Selection`.
- ⚠ OPEN: **The default model's growth**: `arris.build.collect()` over
  `Model.retain`, or nothing in M1. The agent decides at step 6 by
  measuring the four ported parts.
- *Settled (step 3)*: **`extreme`'s tie tolerance** is the model's own,
  read from the new `Model.tolerance` (with `Model.angular_tolerance` for
  `normal`, `parallel` and `perpendicular`): two read-only getters added to
  step 1's queries, since Python could not see the model's precision and a
  literal is the rule's bug. A member's centre is its box's centre moved
  onto its plane, line or circle's plane, so coplanar faces tie exactly
  even turned off the axes. `within` grows its box by four model
  tolerances (`_BOX_SLACK`): `Model.bounds` grows a face's box by its own
  and its edges' tolerances, two at the default precision.
- *Settled (step 3)*: **`created_by` and `modified_by` read each
  operation's record**, kept on `Solid.steps` with its tools' bodies, and
  carry the outputs forward through later records. A record names no tool
  per output, so `modified_by` with several tools in one `cut_many` is what
  any of them modified; `created_by` is exact per tool. `faces()` and
  `edges()` now return a `Selection`.
- *Settled (step 4)*: **workplane conventions.** `XZ`'s normal is the
  world `-y` (`x` × `y` with `y` = `z`), so every plane is right-handed and
  `offset` moves along the normal. `extrude(d, both=True)` goes `d` to each
  side (total `2d`) and a negative `d` goes against the normal. `revolve`'s
  axis is `"x"`, `"y"` or two points in the plane's coordinates.
  `Sketch.cut(*inner)` makes holes; `Sketch.loft` is a static method over
  sketches. A sketch's `arc_to(to, tangent=None)` leaves by the previous
  segment's end tangent, the first one needing `tangent=`. New shared
  `SketchError`, `PathSketch` and `Selection.workplane()`.
- ⚠ OPEN: **Launch timing and venue**: the human's (ADR-0067 §5).
