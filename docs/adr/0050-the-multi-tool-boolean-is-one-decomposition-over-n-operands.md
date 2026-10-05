# ADR-0050 — The multi-tool boolean: one decomposition over N operands

- Status: accepted (2026-10-05)
- Plan: `multi-tool-boolean` step 1 (⚠ OPEN: does Open CASCADE's multi-tool
  cut agree with its own chain)
- Idea: A8 of `docs/ideas/plugin-cad-consumer-asks.md`, scheduled by ADR-0047;
  builds on ADR-0004 (the General Fuse), ADR-0009 (split order), ADR-0018
  (the traced region)

## Context

A pattern of holes, a set of bosses, a pocket cut with several profiles are
one feature to the caller and N booleans to the kernel today: N
decompositions, each re-splitting the target's faces the one before split,
each result an operand of the next, and provenance that names the previous
intermediate body, not the tools. The consumer asked for `cut` and `fuse`
with N tools in one call (A8).

## Decision

**1. One decomposition over all operands, not a fused tool, not a chain.**
`cut_many(m, target, tools, control)` and `fuse_many(m, bodies, control)` run
ADR-0004's General Fuse once over the target and every tool: every face pair
of two different operands whose boxes overlap is intersected once — a tool
against a tool included — every section vertex is made once and shared, every
face is split once by every section on it, and each piece is classified
against the other operands and kept by the N-ary selection. Fusing the tools
first would hide each tool's provenance behind one merged body and cost a
boolean among the tools; a chain pays N times the splitting and leaves an
intermediate body's ids in the provenance. The tools' own overlaps are
sections of the one arrangement, which is why this form can name each tool in
the result.

**2. `cut` and `fuse` are the one-tool case.** `cut(m, t, tool)` is
`cut_many(m, t, &[tool])` and `fuse(m, a, b)` is `fuse_many(m, &[a, b])`,
their results and dumps unchanged bit for bit. `common` stays two-operand over
the same decomposition; N-ary `common` and the General Fuse's cells output are
backlog lines for the consumer that asks.

**3. The operand index is the public shape.** `Interferences` carries
`operands: Vec<Body>`; a face pair, an edge image and a section carry the
operand indices they lie on, in place of `side: usize` and `[_; 2]`; the
`Option<bool>` selections become `enum Selection { Drop, Keep, KeepReversed }`.
Not `enum Operand { A, B }`: a third operand has no name.

**4. The N-ary selection.** A piece of operand `k` is classified against every
other operand whose box holds its point, and kept by the pair (`k` is the
target or a tool; the set of operands it lies inside). A cut keeps the
target's pieces outside every tool and every tool's pieces inside the target
and outside every other tool, reversed; a fuse keeps each operand's pieces
outside every other.

**5. The triple point.** On a face of operand `i` the section curves of two
pairs `(i, j)` and `(i, k)` cross where no edge of any operand pierces — two
overlapping holes' circles on a plate's top. Found as the section curve of
`(i, j)` against the face of `k`, kept when inside all three faces, and merged
with the section vertices by the existing tolerance components, so one vertex
paves all three section curves. `VertexSource::TriplePoint`.

**6. The traced region is per operand pair**, so a pair's curve stays what the
two-operand build gives it (ADR-0018).

**7. Provenance.** A cut's body is `Modified` from the target's; a fuse's from
every operand's. Each tool's entities are `Deleted` and its surviving pieces
`Generated` from it; a section edge is `Generated` from both faces of its
pair, a triple point from the three faces. Split order (ADR-0009) is operands
in the caller's order, so the ids are a function of the tool order; the
geometry — volume, area, counts — is not, and a property holds that.

**8. Refusals.** `BooleanReason::NoTools` (no tools, fewer than two bodies)
and `RepeatedOperand` (one body twice, the target among its tools), naming the
body.

**9. The oracle is Open CASCADE's multi-tool boolean.** `BRepAlgoAPI_Cut` and
`Fuse` driven by `SetArguments` and `SetTools` over lists (a cut: the target
the argument, every tool a tool; a fuse: the first body the argument, the
rest its tools), the recipe grammar's `tools`/`bodies` lists and `pattern` op
(`tests/fixtures/README.md`).

## Evidence from step 1

Open CASCADE's multi-tool boolean against its own chain of two-operand ones,
on the six step-1 fixtures (`tests/fixtures/regression/*-many`,
`*-cut-many`, `*-fuse-many`): a 2×2 and a 10×10 grid of disjoint holes, three
bosses fused, three overlapping holes in a row, two crossing pockets, three
orthogonal bores through one point. Counts (vertices, edges, faces, loops),
shells and genus are **equal** in all six, volume and area agree to 1e-11
relative (the bores' chain differs from the multi-tool result in the 12th
digit, from the order of the splits). Both agree with the closed forms of the
fixtures to the corpus lint's 1e-6. So on the overlapping and the
through-one-point cases Open CASCADE's multi-tool answer is its chain's, and
the fixtures are held to it. The flush and tangent cases of step 5 are
measured there.

## Landed with step 3

- The classification of a piece reads every other operand whose box holds
  its interior point, and no other: `Interferences::bounds` (each operand's
  box, the union of its faces' boxes grown by their tolerances) is what a
  point outside is outside of with no ray cast. A piece on two other
  operands at once is `Unsupported` until step 5.
- Tools that meet one another are refused as `OpError::Unsupported` naming
  the two faces — a face pair of two operands other than the first that
  meets in anything — until step 4 decomposes them. A fuse's operands are
  symmetric in the table but not in this guard: the body the others are
  fused onto goes first.
- `cut` and `fuse` are `cut_many` with one tool and `fuse_many` with two
  bodies; the corpus, provenance and boolean tests, every dump and every
  two-operand cancel step count run unchanged.
- The corpus run of a pattern of N holes is dominated by the B-spline
  read-back, about 1.7 s a hole, so the 10×10 grid (170 s) is in the slow
  set (ADR-0032) and the round radial holes of a curved wall are a test
  against the chain, the corpus holding the same wall pierced by square
  pockets.

## Landed with step 4

- The guard on tools that meet one another is gone: tool × tool pairs are
  sections of the one decomposition, and a piece is kept by the N-ary row
  of §4 against every operand it lies in.
- The triple point is found once per point, as the crossing curve of the
  pair of the two lower operands `i < j` against each face of an operand
  `k > j` whose box reaches both faces; a crossing on all three faces is
  kept, a touch and a curve lying in the third surface are not. Its vertex
  paves the three section curves through it like any other, the curve it
  was found on at its own parameter.
- A triple point on an operand edge — a blind bore whose cap's rim runs
  through four of the eight triple points of three orthogonal bores —
  needs no rule of its own: it lies within the tolerance of the rim's hits
  on the other two walls, and the tolerance components that merge every
  section vertex make the two one (`multi_tool.rs`, the bore ending at
  the triple points).
- The coincident row between two tools of a cut is the fuse's: the tools
  are taken away as their union, so two tool faces coincident with each
  other and agreeing are kept once, from the earlier tool, reversed —
  two crossing pockets' floors. Between the target and a tool, and in a
  fuse or a common, the row is as before, kept from the lower operand.
- Where Open CASCADE's counts differ it is the convention the
  two-operand corpus already records for equal crossing cylinders: it
  also cuts a section ellipse at its parameter origin, one vertex and edge
  more for each pair of equal bores (three bores through one point, the
  tripod), and Arris's one call gives its own chain's counts; those two
  fixtures carry Arris's counts under `counts_differ`.

## Landed with step 5

- **A section edge two pairs share.** Two tools' walls cutting one face of
  a third operand along one curve — a tool repeated by value, two pockets'
  walls in one plane — give two pairs one section block each, on that face
  side by side, which no arrangement orders (the tie is a
  `TangentContact`). A block of a later pair that runs between the same two
  section vertices as an earlier pair's section edge on a face both pairs
  hold, within the edge's tolerance at every check point and with the
  edge's midpoint on it, is that edge: `SectionEdge::shared` records the
  later pair and its other face with the edge's pcurve there, fitted,
  placed and ended as the edge's own. The result splits each face by the
  one edge, and the edge is generated from all three faces.
- **An image held once.** Two tools' edges along one line of a third
  operand's face, both flush with it — two pockets side by side on a
  plate's top — are each imaged on that face, and are already one common
  block between the tools' own coincident faces; the image of the block's
  `b` piece is dropped where its `a` piece is imaged on the same face.
- **A piece flush with several operands.** A piece lying on faces of two
  other operands or more at once is read on the face's two sides: ahead of
  it (along its normal) it is outside its own operand and inside each flush
  operand whose normal opposes, behind it the reverse, every other operand
  the same on both sides as classified. It is a boundary of the result
  exactly when the result holds one side and not the other, and is kept
  once, from the lowest flush operand, facing out of the material,
  standing for every flush face in the provenance. With one flush operand
  this is the coincident row §4 already reads, which stays the path for
  that case, so no two-operand result moves.
- **A contact between two tools** is read against every other operand,
  along the contact at the model's check points, a point on a third
  operand's boundary deciding nothing: two tools touching inside a cut's
  target both survive there and are `TangentContact`, as the two-operand
  rule says. A touch that runs along a tool's seam is an image of the seam,
  not a contact, and the result pinches there: `NonManifold`, as two
  pockets touching along an edge are.
- **Open CASCADE on the flush and tangent cases** (the ⚠ OPEN of step 1):
  on the stacked pockets and bosses, the pockets side by side and
  overlapping on a top face, the overlapping bosses, the adjoining
  cavities and the repeated hole, its multi-tool answer equals its own
  chain in counts, volume and area, and Arris's equals both. On the two
  pockets touching along an edge and the two holes touching along a
  ruling it builds a non-manifold shape from the one call, which its own
  chain's measure rejects as an open surface; those fixtures carry the
  refusal Arris gives (`expect_error`).
