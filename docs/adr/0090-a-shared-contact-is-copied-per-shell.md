# ADR-0090 — A shared contact is copied per shell

- Status: accepted (2026-11-18)
- Follows: ADR-0088 §6 and ADR-0089
- Preserves: ADR-0006's rule that shells share no topology, and
  ADR-0052's edge identity for tessellation

## Context

ADR-0088 allows the boundaries of distinct lumps to touch. A boolean
can leave two lumps that meet at the same edge piece or vertex in its
decomposition. For example, two cuboids touching along an edge have
four surviving face uses of that piece, where a closed shell needs two.
If the boolean kept the common topology, the resulting shells would
share an edge or vertex. The same contact can also occur within one
lump, for example when two pockets meet along an edge or when a void
touches its outer shell.

## Decision

1. **Pair edge uses across material.** On an edge with more than two
   uses, take the tangent at the middle of the range. In the plane
   normal to it, sort the uses by their direction into their face
   (`n × t`, with the effective face normal and oriented edge
   tangent for each use). Pair adjacent uses across material and
   connect faces only within those pairs when finding shells. If two
   directions coincide within angular tolerance, refuse the edge
   with `NonManifold` because the radial order cannot decide the pair.
2. **Give each shell its own topology at the contact.** Key an edge
   piece or vertex reached by several shells by its reference and
   shell. The first shell in deterministic shell order keeps the
   original key and any kept id. Later shells get new ids with the same
   curve and range or point, and the same tolerance. Edge copies take
   their shell's vertices. Each coedge already has its own pcurve.
3. **Refuse a shared contact inside one lump.** If two pairs of an edge
   are in one shell, or shells sharing an edge or vertex belong to one
   lump, return `NonManifold` naming the contact. A later boolean also
   refuses when previously copied contact entities survive in one
   lump, extending ADR-0089's inherited-contact check. A bridge that
   covers the entire contact can join the lumps because it removes the
   contact from the boundary.
4. **Record every copy in provenance.** A source edge, vertex or
   section maps to all its result copies through its `modified` or
   `generated` record.
5. **Weld by edge id.** Each lump's mesh is closed. `TriMesh::weld`
   joins parts at matching edge ids (ADR-0052), so the copies have
   separate mesh indices even where their positions coincide.
6. **Check vertex coincidence within a shell.** The Fast row V4
   reports distinct vertices of one shell whose distance is at most
   the larger vertex tolerance. A sweep sorted along x limits the
   pairs that need a distance test. The scope is one shell so
   coincident vertices in distinct touching lumps are allowed.

## Consequences

- The solid stores ordinary closed shells and `lumps` derives their
  grouping. The volumes of touching lumps add. The checker and mesher
  accept hand-built edge and point contacts with separate topology.
- Open CASCADE can return a compound with an edge shared by two
  solids. The copied topology can give Arris higher entity counts
  while the volume, area and centroid agree. A consumer that merges
  the mesh's contact indices by position can recover a non-manifold
  edge.
- `Violation::CoincidentVertices { a, b }` adds an exhaustive Rust
  enum variant. `InputReason::NonManifold` describes a shell or lump
  whose boundary would touch itself at a shared edge or vertex.

## Alternatives considered

- **Share topology across shells.** This would change the shell
  contract in ADR-0006 and would let welding join the four face uses
  along one mesh edge.
- **Keep refusing these contacts.** The kernel already represents the
  same geometries with independent shell topology, and the training
  harness needs the resulting solids for subsequent operations.
- **Return a `General` body.** This could represent self-contact
  within one shell, but the kernel's solid operations, measures and
  tessellation do not take it. It remains a separate design.

## Amendment of 2026-11-18 — Short edges keep both ends

The CTC-05 elbow fillets in the corpus leave an edge from the narrowest
blend arc to the original vertex, as ADR-0074 requires. In the posed
AP203 fixture, the ends of that edge are about 0.00657 apart and both
have tolerance 0.00774. Removing either end would remove the edge and
change the fillet's topology.

V4 allows a pair of vertices at different points when both are ends
of one non-degenerate edge in their shell. The other rows still check
the edge and its ends. V4 reports exact coincidences even when an
edge joins the pair, and all unjoined pairs within the larger vertex
tolerance.

Before the Fast guard, booleans use V4 to find coincident vertex pairs.
They can preserve a pair that already qualifies in one operand shell
when both vertices keep their source positions. A newly coincident
pair returns `NonManifold` naming source entities. This includes a
pair from distinct shells of one operand that end up in the same shell.
The general checker continues
to accept the two vertex instances Open CASCADE writes where a STEP
crescent's outer bound and hole touch. Sewing those vertices needs to
splice the two loops as well. Merging ids alone changes the builder's
Euler–Poincaré line by one per contact. See
`docs/ideas/sew-step-crescent-vertices.md`.

## Amendment of 2026-11-18 — Radial order reads the angular tolerance

The pcurve gives each use's surface normal at the same midpoint
parameter as the edge tangent. Increasing angle about the forward
tangent crosses into material at a reversed use and out at a forward
use, after composing each use with its result face's orientation.
Every adjacent pair of directions in the cyclic order must alternate
orientations and be farther apart than `Precision::angular_tolerance`.
The directions come from local unit vectors, so changing an edge's trim
length does not change the threshold.
