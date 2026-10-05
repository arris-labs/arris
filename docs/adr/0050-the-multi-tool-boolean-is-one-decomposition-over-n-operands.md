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
