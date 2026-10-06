# ADR-0051 — Split by a plane: one decomposition, two selections, a scratch box for the plane

- Status: accepted (2026-10-06)
- Plan: `split-by-plane` step 1 (⚠ OPEN: how the plane enters the
  decomposition)
- Idea: the roadmap's C7 line "Split by a plane", scheduled by ADR-0047;
  builds on ADR-0004 (the General Fuse), ADR-0009 (split order), ADR-0010
  (`retain`, sparse slots), ADR-0028 (roles), ADR-0050 (the operand-indexed
  decomposition)

## Context

A consumer's split feature cuts a solid by a plane and keeps both sides: a
part divided for two operations, a body halved for symmetry, a mould's two
halves. Today that is `common` and `cut` against a half-space the consumer
builds as a box: two decompositions of the same pair, each re-splitting the
body's faces, two cap faces that share nothing, and a box whose size the
consumer has to guess and whose ids appear in both provenance records.

## Decision

**1. One decomposition, two selections.** `split(m, body, plane, control)`
runs ADR-0004's General Fuse once over the body and the plane's operand, and
reads two selections from it: the body's pieces and the plane's near-face
pieces that a *common* keeps (the positive side) and that a *cut* keeps (the
negative). The body's faces are split once. Not a `cut` and a `common` run
apart: that is the same arrangement built twice, and two chances for the
sides' section edges to disagree in the last digit.

**2. Self-contained sides.** Each side owns its section edges, its section
vertices and its cap faces, one copy per side with opposite orientation; the
two bodies share no entity but the geometry (curves and surfaces are values
in arenas, not owners). Sides that shared an edge would make `retain` or a
later boolean on one side reach into the other. A body entity the plane does
not touch keeps its id on the side it lies on.

**3. The plane is a scratch box, not a half-space operand in `pave`.** The
plane's operand is a box in the plane's frame: its near face on the plane,
its other five faces past the body's extent in the frame's axes by a margin
no tolerance reaches, so they have no face pair with the body and add no
section. It is built in the model, runs through the decomposition unchanged,
and is freed afterwards. The criteria the plan set, and why this meets them:

- *No plane entity survives in the model.* `Model::discard(body, keep)`
  (`arris-topo`, new) frees the slots of the entities of `body`'s closure
  that no body of `keep` reaches, as `retain` frees what no kept body reaches
  (ADR-0010); the scratch box's body, shell, faces, edges and vertices go,
  the surface a cap carries stays because the cap reaches it. A transaction
  cannot do this: it undoes appends on `Err` and the results are appended
  after the box. A scratch model cannot either: the results would be copies,
  and the body's untouched entities would lose their ids (point 2).
- *Output ids depend only on the input.* The box is a function of the body's
  bounding box and the frame, built first, and the freed slots are refilled
  in arena order by the next operation, as after any `retain`.
- *Classification is exact.* A half-space operand in `pave` would classify
  by signed distance, but it would need an unbounded face in every stage that
  reads `FaceId`s from the model (hits, sections, pieces, result): a second
  kind of face through 7 000 lines, and every case C2–C5 settled for a plane
  against a face — coincident, through an edge, through a vertex, tangent,
  through a cone's apex — written a second time. The box's near face is a
  plane, and the plane-against-body cases are the booleans' own, tested
  against Open CASCADE by the corpus. A split inherits their rulings and their
  refusals, `TangentContact` and `BesideSingularity` included, and a fixed
  one improves both.

The cost is a face of the box in the arrangement whose parameter domain is
the body's extent plus a margin, not the plane's infinity, so no section is
traced over a span far beyond the body.

**4. The plane is a `Frame`.** As `query::project_to_plane` takes it: the
origin and `z` place the plane and say which side is positive, `x` fixes the
cap's (u, v), so a consumer's sketch plane gives caps whose parameters are
the sketch's. The box is built in the frame, so the cap's surface is the
plane `Frame` itself, not a re-derived one.

**5. The return shape.** `Split { positive, negative, provenance }`: the
first operation whose output is not `(Body, Provenance)`, because it has two
bodies and one account of them. A side may hold several lumps (a U-bracket
split across both arms), and either side is never empty when `split` returns
`Ok`.

**6. Provenance.** The plane is no entity, so a cap face is `Generated` from
its side's role, `Role::Split(SplitPart::Cap(PlaneSide))` (appended last,
ADR-0028), and the scratch box's own ids never appear in a record: the
`Generated`/`Deleted` entries that name them are rewritten to the role. The
body's faces, edges, shell and body are `Modified` into their pieces on each
side; a section edge is `Generated` from the body face it lies on and from
the cap role of its side; a section vertex from the edge it splits and that
role. The input body is `Modified` into both sides. Split order (ADR-0009):
the positive side first, then the negative.

**7. Refusal.** `SplitReason::NoCrossing { body }`: the plane misses the body
or only touches it along a face, an edge or a vertex, so one side would be
empty. A touch with a crossing elsewhere follows the booleans' rule
(`TangentContact`).

**8. The oracle is Open CASCADE's splitter.** `BRepAlgoAPI_Splitter` with the
body as the argument and a planar face past the body's box as the tool; its
solids go to the side their centroid's signed distance says. The recipe
grammar's `split` step yields the side a `side` expression picks as the
step's own name and both sides as `<name>.positive` and `<name>.negative`
(`tests/fixtures/README.md`), and each side is also built as Open CASCADE's
common or cut with a half-space box and recorded as `half_space` in
`expected.json`.

## Evidence from step 1

Open CASCADE's splitter against its own half-space booleans on the twelve
step-1 fixtures that have a side to build (`tests/fixtures/regression/*split*`
less the two refusals, a side each way): a box split mid-height and across a
corner, a cylinder across and along its axis (through its seam and off it), a
tube, a sphere, a plate with a row of holes, a U-bracket split across both
arms, a box with filleted verticals, a cylinder with a filleted rim, a
shelled box. In all 24 sides the splitter's counts (vertices, edges, faces,
loops, shells, solids) are **equal** to the common's or cut's, and its volume
and area agree to 3e-15 relative; the `half_space` entry in each
`expected.json` keeps the record. Both agree with the closed forms of the
fixtures to the lint's 1e-6. So on these cases the splitter's answer is the
booleans', and a split that is the booleans' two selections is held to it.

One place where the sides' counts differ from each other is Open CASCADE's
convention, not a disagreement: an axial split off the seam imprints the seam
on one side, a half-cylinder of 6 vertices, 9 edges and 5 faces against 4, 6
and 4 on the other, in the splitter and the boolean alike. The fixtures hold
Arris to the oracle's counts per variant; step 4 decides whether Arris's
convention for a seam its plane does not touch is the oracle's or a stated
`counts_differ`.
