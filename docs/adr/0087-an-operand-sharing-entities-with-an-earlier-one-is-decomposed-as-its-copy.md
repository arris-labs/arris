# ADR-0087 — An operand sharing entities with an earlier one is decomposed as its copy

- Status: accepted (2026-11-17)
- Plan: `flat-edge-read-as-tangent-contact` step 9 (retired), written at its
  retirement review
- Amends: ADR-0004 (the Consequences' "the untouched faces, edges and
  vertices are shared with the input by id": not for an operand copied
  here)
- Follows: ADR-0050 (the operand-indexed decomposition), ADR-0051
  (`Model::discard` of a scratch body), ADR-0002 (provenance origins)

## Context

Entities are immutable and bodies share them by id. A body split by a plane
and its sides fused back keeps the faces the split left untouched, with
their edges and vertices, by id: the fused body and the body it was split
from share them. ADR-0050's pave model keys every vertex, edge and face by
its id and reads each as one operand's (`FacePair::operands`,
`EdgeImage::operand`), so a boolean of the two read a shared face as both
operands' at once. `interference` of a box against its split-and-fused
sides faulted in the common's builder (an edge used by no coedge), and
`fuse`, `cut` and `common` of the two faulted alike, where the same box
fused from two boxes answered (`query/interference-box-against-its-split-and-fused-sides`,
found by `coplanar_prop`).

## Decision

1. **The later operand is copied.** Before the decomposition,
   `boolean/shared.rs` walks the operands in the caller's order. An operand
   that shares any vertex, edge, face or shell with an earlier one is
   replaced by its copy, `transform`'s `copy_body` with `KeepGeometry`
   (entity for entity, the same curves and surfaces), appended inside the
   boolean's transaction. Operands that share nothing are read as given, at
   the cost of one closure each, so every other boolean's ids are unchanged.
   `fuse`, `cut`, `common`, their n-ary forms and the `interference` query
   read operands so.
2. **The record is written against the caller's ids.** Every origin in the
   copy is its original. An entity of the copy the result keeps is a new id,
   `Modified` from its original, which the result no longer holds by id:
   for the later operand, an untouched entity does not keep its id. A
   shared entity is one input read twice. When the result keeps it by id
   (the earlier operand's, untouched), it is kept with no record
   (`is_kept`), as ADR-0004 keeps any untouched entity: the copy's reading
   of it stood in for it as the flush case's one edge of the result, and
   is dropped; a piece of the copy's reading other than the entity itself
   would cover what the kept entity covers, an invariant fault. Otherwise
   the pieces of both readings are its pieces, `Deleted` only when neither
   left one.
3. **A refusal names the caller's entities.** An entity of the copy in an
   `Unsupported`, `Degenerate`, `Tolerance` or `InvalidInput` is its
   original, since the copy is rolled back with the failed boolean. A
   kernel fault keeps the ids it was raised on.
4. **The copy is discarded on success.** Once the boolean has succeeded,
   the copy's entities that neither the result nor an operand reaches are
   freed (`Model::discard`, ADR-0051's precedent), so the caller's model
   holds no body it did not ask for. `interference` runs on a scratch clone
   it drops, so it discards nothing.

## Consequences

- A body and one split from it and fused back are operands of every
  boolean; the result and its record are checker-clean and audited against
  both operands' ids.
- The exception to ADR-0004's "an untouched entity keeps its id": an
  untouched entity of a copied operand that the result keeps is a new id,
  `Modified` from the caller's. `docs/DATA-MODEL.md` §Provenance and
  `docs/ARCHITECTURE.md` §Operations state it.
- The copy costs a `copy_body` of the later operand, only when operands
  share an entity. It meters on its own `Meter`, not the boolean's budget
  (`docs/BACKLOG.md`).
- `interferences` and `interferences_many` read operands as given: two
  that share a face fault there (`docs/BACKLOG.md`).

## Alternatives considered

- **Key the pave model by (operand, id).** Every map of the decomposition,
  the coincident pairs and images and the builder's kept-by-id assembly
  would carry the operand index, and a shared entity would be two keys
  the assembly could keep twice. A change across `pave/` and `rebuild`
  for one input shape; the copy confines it to one module.
- **Copy the earlier operand instead.** The target of a `cut` and the first
  operand of a `fuse` are the ones whose untouched entities a caller most
  expects to keep their ids; copying the later one keeps them.
- **Refuse operands that share entities.** The shape is ordinary (a split
  and a fuse back), and Open CASCADE answers it.
- **Fuse the shared faces away before the boolean.** That is same-domain
  face merging, out of C12, and would change the operands' ids too.
