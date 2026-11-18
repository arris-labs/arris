# ADR-0089 — Resample a piece at an isolated touch

- Status: accepted (2026-11-18)
- Follows: ADR-0004 (classify trimmed pieces), ADR-0030 (metering) and
  ADR-0088 (touching lumps)
- Extends: ADR-0088's boolean rule for surviving curve contacts to
  isolated point contacts

## Context

A ball of radius 2 centred at the origin touches a cylinder's end disc at
the south pole. The pave model records a `Touch` point for the plane and
sphere and a tangent seam hit at the pole. There are no crossing section
edges on the trimmed faces. In the centred pose, the disc's first
interior sample lies in the sphere's tolerance band. The classifier
answers `On`, and the selection has neither a curvature across a
contact curve nor a section edge with which to decide the side. The
offset disc's centre is 0.3 from the pole and about 0.02237 from the
sphere. That sample is already clear at the default face tolerance of
`1e-7`.

## Decision

Use the original coincidence, curve-curvature and transversal rules
when they establish the answer. If the entity named by `On` is a face
with an isolated surface touch or on its boundary, try further points
on the same trimmed piece against the implicated operand. Read the
piece's own pcurve loops in their stored translate. In order, offset the
first sample by 1/4, 1/8, 1/16, 1/32 and 1/64 of the respective box
width in each of +u, −u, +v and −v. A candidate must be inside the loops
with clearance greater than their polygonal chord deviation plus the
classifier's parameter band. Evaluate it on the original surface and
charge a meter step for each classification. The first `Inside` or
`Outside` enters the selection table. If every candidate is rejected,
`On` or undecided, keep the original named `Unsupported`. The original
sample still orders provenance.

The centred disc clears on the first quarter-width step. A shift by
0.5 in its plane separates the new sample from the sphere by about
0.06155. A 0.01 gap gives two disjoint lumps, and a 0.01 overlap cuts the
disc and sphere along a crossing circle with radius about 0.19975. The
gap and overlap use the ordinary classification and section paths.

The result uses the surviving face trims for isolated contacts. A
touch retained on both faces of one lump away from their shared
topology is `TangentContact`, naming the two source faces. The same
point between distinct lumps is allowed by ADR-0088. For example, the
outside point contact of the ball and disc builds with two independent
shells, while a spherical cavity touching its outer plane is refused.
A trim that removes the touch leaves no contact to refuse.
The same test applies to touches inherited from two shells of one
operand, whose faces the pave does not compare. If those shells contribute
pieces to one result lump, the checker intersects the selected faces and
the boolean maps a surviving contact back to the source faces. Face
pairs and their intersections use the boolean's meter (ADR-0030). A bridge
can fuse the ball and disc when it encloses the pole. A bridge joining
them off-centre leaves the pole exposed and returns `TangentContact`.

## Consequences

Both ball and disc poses pass `Full` with the existing B1 placement
rule. The cuts keep their target and the commons return `Empty`.
Open CASCADE agrees on the measures, counts and probe classes for
the two fuses, including the contact point. Its own STEP can use a
lone `VERTEX_LOOP` on the sphere, an existing reader refusal held
by the fixtures. Open CASCADE's reading of Arris's STEP is compared
with the oracle's measures and probes.

## Alternatives considered

A normal offset from the first point can enter material through the
piece itself. A second sample on the surface keeps the classification
about the same piece. A global change to the classifier's `On` band
would also affect ray and point queries that correctly identify the
contact.
