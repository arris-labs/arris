# Plan: loft

- Started: 2026-10-16
- Milestone: C10, the NURBS cycle, second item's second half (docs/ROADMAP.md §C10)
- Idea: `docs/ideas/sweep-and-loft.md` (absorbed: its loft half, option A,
  decided 2026-10-15 with Q5 and Q6; the sweep half is ADR-0063)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal

`ops::loft` builds a solid through two or more placed `Profile`s, its
*loft sections*, with matching loop counts: **ruled** (each span between
consecutive sections straight) or **smooth** (one surface through all of
them), open with two caps or **closed** (no caps). A section with fewer
segments is split by arc length (not as `ThruSections` does, by
projection: step 1); each loop's start point fixes the correspondence. A ruled span is exact: coplanar lines give
planes, coaxial circles cylinders and cones, any other pair a degree-1
NURBS with no fit. A smooth loft interpolates the control nets. Fixtures
match `BRepOffsetAPI_ThruSections`, bad sections are refused by name, and a
loft is an operand of every boolean and blend the cycle allows so far.

## Non-goals

- **A point as an end section**, guide rails, end tangency: backlog lines.
- **Unequal loop counts** (a hole appearing midway): refused.
- **NURBS × NURBS**: a boolean or blend between two NURBS faces stays
  `Unsupported` until the marcher; a loft's own NURBS faces meet only on
  columns it builds.
- **Self-intersection** of a twisted loft or crossing sections: left to the
  checker, as for the sweep. **A NURBS profile segment**: its own backlog line.

## Design deltas

- **ADR-0065 (step 2): a loft matches sections by arc length and is exact
  where a ruled span's kind holds it.** The matching rule and a split
  section's segment numbering, the ruled span table, the smooth
  interpolation (degree, parameters from step 1), the closed loft, roles,
  the oracle's limits. Alternatives: `fit_skin` (needs a continuous family);
  matching by vertex count only (refuses a circle to a square).
- **`arris-geom`, public, new (not a break):**
  - `LoopNurbs`: a profile loop as one closed rational quadratic NURBS,
    split at its vertices by double knots — lifted from
    `arris-ops::sweep::along`'s private `Composite`, which then uses it.
  - `match_sections(&[Profile], closed, tol) -> Result<Vec<MatchedSection>,
    LoftError>`: every loop cut at every vertex's arc-length fraction when
    counts differ and made compatible (one degree, one knot vector) across
    sections; `MatchedLoop { nurbs, sources }`.
  - `interpolate_skin(&[NurbsCurve], params, degree, closed) ->
    Result<NurbsSurface, GeomError>`: a surface through compatible
    sections, its isolines at `params` the sections exactly.
  - `LoftError::{TooFewSections, LoopCount { section }, Degenerate { section,
    loop_index }, ...}`; the final list is ADR-0065's.
- **`arris-topo::provenance`, a break:** `Role::Loft(PathPart)` appended
  after `Sweep`, `PathPart`'s doc widened: span `k` is a path's segment `k`
  or the span from loft section `k` to `k + 1`; a smooth loft has one span.
  Binding `role.rs`, `_arris.pyi` and `### Breaking` in that commit.
- **`arris-ops`, public:** `loft(m, sections: &[Profile], kind: LoftKind,
  closed: bool, control) -> Result<(Body, Provenance), OpError>` with
  `LoftKind::{Ruled, Smooth}`, re-exported by the facade;
  `OpError::Loft(LoftError)` (a break, Python `OpLoftError` with `detail`).
- **Fixture grammar:** a `loft` op in `tools/oracle/oracle/recipe.py` and
  `arris-debug`'s recipe types; Open CASCADE runs `ThruSections(solid,
  ruled)`, a closed loft repeating the first section. `prop::recipe`
  draws it.
- **DATA-MODEL** §Profiles gains loft sections, §Provenance `Role::Loft`.
  **GLOSSARY:** *loft section* (never bare "section", the boolean's word),
  *ruled span*.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[3]** — **The oracle, the fixtures, and the two
  measurements.** Add `loft` to both interpreters (Rust refuses
  `Unsupported`). Commit under `regression/`, `#[ignore]`d with Open
  CASCADE's values: `loft-square-frustum` (ruled, all planes),
  `loft-cone-frustum` (coaxial circles), `loft-circle-to-square` (ruled,
  the circle split), `loft-twisted-square` (ruled, start rotated 45°),
  `loft-ring-to-ring` (a hole in every section), `loft-duct` (smooth,
  circle → rounded rectangle → circle), `loft-closed-torus-like` (smooth,
  closed, four circles round an axis). Measure and record here: (a) the
  split rule `ThruSections` applies to unequal segment counts, with
  `CheckCompatibility` off so the consumer's start point holds; (b) how far
  a cubic interpolation of the control nets under chord-length and
  centripetal parameters sits from Open CASCADE's smooth loft on
  `loft-duct` and the closed one (volume, area). Answers open questions 1–2.
- [x] Step 2 **[2]** — **ADR-0065, `LoopNurbs` and `match_sections`.**
  Write the ADR from step 1. Lift the composite loop into `geom` (sweep's
  fixtures unchanged, bit for bit). Add matching and `LoftError`. Tests:
  split points at the measured arc-length ratios, every matched section
  reproduces its profile exactly, compatibility across sections, each
  refusal by name. Rustdoc examples.
- [x] Step 3 **[2]** — **The ruled loft, every span a degree-1 NURBS.**
  `ops::loft` for `Ruled`, open and closed; `Role::Loft` (the break, with
  the binding); planar caps from the end sections. Adjacent faces share
  their column, as the sweep's do. `loft-circle-to-square`,
  `loft-twisted-square` and `loft-ring-to-ring` pass against the oracle
  (twisted and ring to 1e-9; the circle to the square only to the
  pairing's 1.2e-3 volume, ADR-0065 §2, which its `tolerances` record) and
  move to a new `tests/fixtures/loft/` area. `OpError::Loft` and
  `OpLoftError` land with the op (the binding cannot compile without them);
  `LoftKind` has `Ruled` only until step 5.
- [x] Step 4 **[2]** — **Exact ruled spans.** Classify by check, not by
  segment kind: two line segments coplanar within the tolerance give a
  `Plane`; two circles coaxial in parallel planes a `Cylinder` or `Cone`;
  joints between exact spans are the analytic curves. `loft-square-frustum`
  and `loft-cone-frustum` pass, also held to the frustum's closed-form
  volume `h/3·(A₁ + A₂ + √(A₁A₂))`.
- [x] Step 5 **[3]** — **The smooth loft.** `interpolate_skin` in `geom`
  at the degree and parameters ADR-0065 records (two sections: the ruled
  surface; Open CASCADE's are degree n − 1 at chord length, a closed loft
  repeating its first section, open question 2); one span per loft. `loft-duct`
  and `loft-closed-torus-like` pass within the tolerances step 1 measured.
  *Done as:* the sections' points interpolated, not their nets (one
  weight function per column first), the chord of the control nets; the
  duct differs by §2's pairing (8.5e-5) and carries it, the ring holds
  1e-8 (ADR-0065 §7).
- [x] Step 6 **[2]** — **Refusals.** Too few sections (two, three closed),
  unequal loop counts, two consecutive sections coincident, a loop that
  collapses; each a `regression/` refusal fixture moved to `loft/` passing.
  *Done as:* the first three are `loft/` fixtures (`ExpectError::
  LoftTooFewSections | LoftLoopCount | LoftCoincident`; the oracle records
  the empty result where it is not asked or reports not done), passing at
  once, so none passed through `regression/`; a collapsed loop is
  `ProfileError` before it reaches matching (`LoftError::Loop` guards the
  rest), so it is an `arris-ops` test, not a fixture.
- [x] Step 7 **[2]** — **Operands and properties.** Operand fixtures: a
  cone frustum drilled by a cylinder, a duct cut by a plane, a fillet on an
  exact ruled edge, a ruled loft fused with a box. `loft_prop`: translation
  commutes; reversed order keeps volume and area; two equal parallel
  sections are their extrude; similar coaxial sections meet the frustum
  formula. `prop::recipe` draws `loft`.
- [x] Step 8 **[1]** — **The Python binding.** `loft`, `LoftKind` and
  `OpLoftError` in `arris-py`, the stub and a docstring example; the facade's
  doc example.
  *Done as:* `Model.loft(sections, kind="ruled", closed=False)`, `kind` a
  string as `shell`'s `side` is, so no `LoftKind` class; `OpLoftError` had
  landed in step 3.

## Acceptance

`cargo nextest run --workspace` with every `loft/` fixture passing against
its `ThruSections` values (closed forms where `measure_differs` holds
them), the refusal fixtures refusing by name, `loft_prop` at 256 cases, the
differential at 1000 draws with `loft` drawn and no new failure; pytest and
`mypy.stubtest` in CI's `python` job.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Operations, the sweeps — loft, its span table,
  refusals; the crate table's `arris-geom` row (`match_sections`,
  `interpolate_skin`).
- `docs/ROADMAP.md` §C10 — loft done; differential tally with loft drawn.
- `docs/BACKLOG.md` — point end sections, guide rails, end tangency.
- `GLOSSARY.md` — loft section, ruled span.
- `CHANGELOG.md` — the loft bullet.
- `AGENTS.md` current state; `docs/ideas/plugin-cad-consumer-asks.md`'s
  status (A7's loft landed); `tests/fixtures/README.md` the loft row.

## Open questions

- ~~OPEN 1~~ (answered, step 1): `ThruSections` does **not** keep the
  consumer's start points, and does not split by arc length. With
  `CheckCompatibility(false)` it splits nothing and lofts only the first
  edge of each section (a circle against a square: one face). With it on,
  `BRepFill_CompatibleWires` (a) re-starts each wire at the vertex nearest
  the previous section's start, whatever the flag `Perform(WithRotation)`
  says, and (b) cuts the wire with fewer edges where the other wire's
  vertices **project onto it** (a circle against a 12 × 4 rectangle is cut
  at 18.4°, the rectangle corner's direction from the centre, not at an
  arc-length fraction); a start point lying on the other wire's edge adds a
  vertex there. So a section pair of different segment counts matches
  arc-length fractions only where the sections are similar and centred, as
  the circle and the square fixture are; ADR-0065 chooses its own rule (arc
  length, the start point fixing the correspondence) and the fixtures place
  start points where Open CASCADE keeps them: `loft-twisted-square` turns
  30°, not 45° (at 45° the nearest vertex is a tie), `loft-circle-to-square`
  starts the circle at the square's start corner, and `loft-duct` gives the
  circles eight arcs so no split is needed (a split circle there would
  differ between the two rules).
- ~~OPEN 2~~ (measured, step 1; `loft-duct` and `loft-closed-torus-like`):
  Open CASCADE's smooth loft is **one polynomial of degree n − 1** through
  the n sections (capped by `MaxDegree` 8), points matched by the fraction
  of each edge, spans parametrised by **chord length**; `SetParType` changes
  nothing; a closed loft repeats the first section and is *not* periodic.
  Checked by interpolating the sections' sampled points (400 per edge):
  the duct (sections at 0, 7, 20) has chord-length parameters 0, 0.3518, 1
  and a quadratic through them gives volume 722.2725 and area 511.6934
  against Open CASCADE's 722.2564 and 511.6833 (2e-5 and 2e-5); centripetal
  gives 710.96 and 504.99 (1.6% and 1.3% off), uniform 707.33. The closed
  one is a degree-4 polynomial through the four circles and the first again
  at uniform parameters: 382.9646 and 504.3118 against 382.9751 and
  504.3182 (2.7e-5, 1.3e-5); a periodic cubic spline gives 344.94 and
  463.53 (10% off), a natural cubic 318.28. The residual 2e-5 is the
  sampling and the chord taken between sampled points (between vertices it
  is 4e-5 the other way): Open CASCADE takes it on its own nets, so step 5
  tries the chord of the control nets first and, if it cannot reach 1e-5,
  holds the smooth fixtures to 1e-4 through ADR-0065 rather than closed
  forms. **The plan's cubic and periodic closed loft are wrong for this
  oracle:** ADR-0065 §7 takes degree n − 1 as Open CASCADE does and refuses more
  than nine sections. Settled in step 5: the chord of the control nets;
  points interpolated over one weight function, since the nets'
  homogeneous interpolation was 4.1e-3 off; the duct's residual is §2's
  pairing, the ring's Open CASCADE's approximation (ADR-0065 §7).
- ~~OPEN 3~~ (decided, step 2, ADR-0065 §4): a loft's `Side { segment }`
  numbers the matched segments in the loop's walk; `MatchedLoop::sources`
  gives the consumer's segment each came from.
