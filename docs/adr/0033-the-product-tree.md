# ADR-0033 — The product tree: occurrences beside the flattened bodies, read from the walk the flattening makes, written back from the same value

- Status: accepted (2026-10-01)
- Plan: `step-product-structure` steps 1–7
- Amends: ADR-0025 §5 "Instances" (the bodies stay flattened and baked; a
  tree is returned beside them)
- Follows: ADR-0030 (cancellation), ADR-0013 (`io` owns the format's
  vocabulary)

## Context

ADR-0025 flattened an assembly to one body per placed solid and said Arris
has no product structure. The first consumer's ask A4
(`docs/ideas/plugin-cad-consumer-asks.md`) is the product structure itself:
products, instances, placements, names and colours beside the bodies, on
read and on write. Bodies stay what they are — a consumer that wants the
unplaced prototype inverts the occurrence's composed placement.

## Decision

### 1. The tree is an `arris_io::step` value, not topology

`Read` gains `products: ProductTree`. `ProductTree { roots, faces }`;
`Occurrence { product: Option<u64>, name, placement:
Result<Isometry, Refusal>, colour: Option<Rgb>, solids: Vec<usize>,
children }`; `Rgb([f64; 3])`; `FaceColour { solid, face, colour }`. `Body`,
`Model` and `arris-topo` do not change. `solids` are indices into
`Read::solids`.

### 2. One walk, so one numbering

The tree is the walk `reader/assembly.rs` already makes over the
placements: each site of it — a node at the end of a path of placement ids
from a root — is an occurrence, and a solid's instance number at a site is
its rank among the solid's paths, which is how the flattening numbers it.
The tree and the bodies therefore agree on what "instance k" is by
construction, and a cycle of usages is cut where the flattening cuts it.
A node's product is found from the `SHAPE_DEFINITION_REPRESENTATION` of any
representation the node joins; its name is the `PRODUCT` name, the id where
the name is empty, and empty for a node no product defines. Children are
ascending by the id of the placement that puts them, which is the
flattening's path order.

An occurrence is kept if it has a product, a solid or a kept child, so a
representation the file attaches to nothing (a wireframe, annotation
geometry) is no root.

### 3. Refused placements stay in the tree

`placement` is a `Result`: an occurrence under a `MAPPED_ITEM` or
`CARTESIAN_TRANSFORMATION_OPERATOR_3D` Arris does not read carries the
refusal, its solids refused as before, so the consumer sees the part exists
and why it has no pose. Dropping it would hide the part. The refusal
histogram is unchanged: nothing is counted twice.

### 4. The writer takes the same value, with the bodies in their products' frames

`step::write_products(&Model, &[Body], &ProductTree, &Control)`: a consumer
builds the same type it reads. `Occurrence::solids` index the `bodies` slice
on write as they index `Read::solids` on read, but what a body *is*
differs: a read body is baked at its placement (§Context), a written body
is its product's own geometry, and the placements move it. A consumer that
writes back what it read moves each body by the inverse of its occurrence's
composed placement first (`ops::transform`); the writer never moves
geometry, and never writes a placement twice.

Occurrences with the same `Some(product)` are one product, written once
and placed many times (they must agree in name, colour, solids and
children: `TreeError::ProductMismatch`); `None` is a product of its own. A
body held by two products is `TreeError::SolidShared`, since its faces and
edges would be written twice. A placement that is `Err` or not finite
cannot be written (`TreeError::Placement`); a root has no parent and its
placement is not written. `StepError` gains `Tree(TreeError)` and
`Interrupted` (the writer ticks per occurrence and per body); `write`
stays the one-product case, byte for byte.

### 5. Names and colours

A name is the `PRODUCT` name (Open CASCADE writes the same in `id` and
`name`; the `NEXT_ASSEMBLY_USAGE_OCCURRENCE`'s own names are generated
noise, `=>[0:1:1:2]`, and not read). A colour is plain RGB through the
`STYLED_ITEM` chain; anything else is skipped, never a reason to lose a
body. An occurrence's colour is that of its own solids: one with no solid has none on read, and the writer refuses one given (`TreeError::ColourWithoutSolids`) rather than drop it. A face colour is part of the ask — Open CASCADE writes it as an
`OVER_RIDING_STYLED_ITEM` on the `ADVANCED_FACE` over the solid's own
`STYLED_ITEM` — and is read into `ProductTree::faces`.

## Consequences

- `Read` (pub fields) and `StepError` (exhaustive) change: `### Breaking`.
- What was read of Open CASCADE's files is its XCAF output, observed in the
  file the oracle writes (`tools/oracle/occt_assembly.py`); no source of the
  reference trees informed this.
- Shared prototypes stay a non-goal: bodies are baked per instance.
