# ADR-0040 — Faces are tangent for a blend where its ball moves within their tolerance

- Status: accepted (2026-10-03)
- Plan: `blend-corners` step 5 (the plan's open question on tangency as
  read)
- Amends: ADR-0035 §1 and §6 (the tangency the walk and `TangentChain`
  read), ADR-0039 §1 (the same test at a vertex of four edges)

## Context

A blend asks whether two faces meet tangentially in three places: along
the blended edge itself (a tangent dihedral has no corner to roll a ball
into, `TangentChain`), at a corner edge where an end meets a vertex (a
tangent corner edge stops a chain, or is a junction's third edge), and at
the next edge the walk might take (ADR-0035 §1, ADR-0039 §1). ADR-0035
read all three by the outward normals' cross product against the model's
angular precision, `1e-12`.

That is a test of exact geometry, and a file is not exact. ADR-0039 found
CTC-04's chamfered walls, a plane wall turning into a cylinder round a
rounded corner and its plane chamfer strip turning into a cone, written
tangent but read with normals `7e-11` to `9e-10` apart. The walk did not
take them, and the 24 edges along them stayed `VertexBlend`, though the
same corner built from a recipe walks
(`blend/chamfered-stadium-foot-fillet`). Behind the corners it is the
same misreading: on the committed tier, FTC-08 refused 139 edges alone as
`BlendTooLarge` and FTC-10 24 as an unsupported cylinder against a
sphere, where a face meeting its neighbour tangentially as written was
read as a sharp edge.

Open CASCADE's fillet decides the same question with
`ChFi3d::IsTangentFaces`, read in the reference trees: a continuity the
edge carries, or else samples along the edge held to G1 at an angle of
`0.1` and a distance of the larger of `0.001` and 1.5 times the edge's
tolerance. Those are fixed numbers, wide enough to call a 5° edge tangent;
the reason for a wide test is the one above.

What decides whether a near-tangency matters to a blend is the blend.
Two faces with normals `θ` apart at a point of their edge are touched at
that point by balls of radius `r` whose centres are `2r sin(θ/2)` apart,
and a chamfer's contacts at distance `d` move by the same order. Where that
is within the tolerance the blend is built to, the two faces are one face
to this blend: the stripes on either side meet as one ball's, and a
blend of the edge itself would be a stripe narrower than its tolerance.

## Decision

**1. Tangent for a blend of size `s`.** Two faces with outward normals
`n₁` and `n₂` at a point of their edge are tangent for a fillet of
radius `s` or a chamfer of distance `s` when `|n₁ × n₂| ≤` the angular
precision, or `s |n₁ × n₂| ≤ t`, where `t` is the larger of the model's
default tolerance and the two faces' own. The first clause keeps every
case the old test took; the second is the ball's move.

**2. Every tangency a blend reads is this one.** The blended edge's own
dihedral at its midpoint (a stripe's and a ring's `TangentChain`), each
corner edge at an end's vertex, the other edges at a tangent vertex of
three or four edges and the next edge's dihedral the walk reads all use
it, with the operation's one radius or distance. A vertex is a tangent
vertex, a junction's third edge is tangent and a corner edge refuses
`TangentChain` by the same answer, so no vertex is both and none is
neither.

**3. The faces' tolerance, not the edge's.** The stripes are built and
their contacts met at the faces' tolerance (the default where theirs is
smaller): the junction's points are the two runs' contacts, and they must
agree within it (ADR-0035 §3). An edge's tolerance may be wider than its
faces'; admitting a tangency at that width would put the two runs'
contacts further apart than the junction accepts, an internal fault the
test must keep unreachable.

`Reason` gains no variant, and no public type or signature changes.

## Consequences

- CTC-04's chamfered walls walk: the 24 edges refused `VertexBlend` at
  ADR-0039 build alone, checker green at `Full`, and the four
  cylinder-against-plane edges beside them, each a tangent dihedral as
  written, are refused `TangentChain`, as Open CASCADE refuses them
  (`blend/nist-ctc-04-chamfered-wall-fillet`, the part read in place).
  The committed tier's `VertexBlend` edges go from the 63 that ADR-0039's
  junction left to 35, none of them on CTC-04.
- On FTC-08, 113 more edges build alone (139 `BlendTooLarge` become 17),
  and 47 are refused `TangentChain` as the tangent dihedrals they are
  written as; on FTC-10, the 24 cylinder-against-sphere refusals are a
  tangent cap's dihedral, `TangentChain`. Every edge that newly builds on
  the committed tier passes the checker (the census run with `paranoid`);
  the three that fail, all on FTC-08, failed the same way before
  (`regression/nist-ftc-08-fillet-pcurve-jump-checker-fault`). FTC-08's battery fillet now meets a
  sampled tangent dihedral first where it ran over before.
- The battery's own filter of tangent dihedrals (`arris_debug::battery`)
  still reads the angular precision alone, so it samples edges the kernel
  now names tangent dihedrals. Aligning it changes every part's sample and
  is measured where the plan measures the tiers again.
- A test that depends on the size is a test of the operation, not of the
  shape: the same edge can be tangent for a fillet of 0.25 and sharp for
  one of 300. That is the intent. A chain's extent changes with the
  radius only across a dihedral the radius cannot resolve.

## Alternatives considered

- **Keep the angular precision.** The kernel then refuses every
  tangency a file writes to `1e-10`, which is every file's.
- **A fixed wider angle**, Open CASCADE's `0.1` or a smaller one. It is a
  literal standing in for a tolerance (`.agents/rules/kernel.md`), wrong at
  both ends: too wide for a large blend, whose ball it moves past the
  tolerance, and arbitrary for a small one.
- **The edge's own tolerance**, as the plan's question put it. §3: wider
  than what the junction meets its contacts at.
- **Snap the faces tangent on reading.** The reader would change the
  operand's geometry for one operation's sake, and every other operation
  would read a shape the file did not write.
