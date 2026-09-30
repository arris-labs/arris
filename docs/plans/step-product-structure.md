# Plan: step-product-structure

- Started: 2026-10-01
- Milestone: C5 — the consumer's API, ask A4 (docs/ROADMAP.md)
- Idea (verbatim from the human): "step-product-structure" — A4 of
  `docs/ideas/plugin-cad-consumer-asks.md`: "the reader returns products,
  instances, placements, names and colours beside the flattened bodies; the
  writer writes instances, names and colours." The idea file stays: its
  other asks are C5's done items or backlog lines.

## Goal

`step::read` returns, beside the flattened bodies it returns today, a
`ProductTree`: the roots of the file's assemblies, each an *occurrence* of a
product with its name, its placement in its parent, its children and the
bodies (indices into `Read::solids`) it holds, plus the colour a solid or
face carries. `step::write_products` writes such a tree over a slice of
bodies — each product once, each placement as a
`NEXT_ASSEMBLY_USAGE_OCCURRENCE`, names and colours as
`PRODUCT` names and `STYLED_ITEM`s — and Arris's own written assembly reads
back to the same tree, while Open CASCADE's XCAF STEP reads to the names,
placements and colours it wrote. `write` and the flattened `Read::solids`
are unchanged in behaviour: the histogram and every existing consumer still
see one body per placed solid.

## Non-goals

- Shared prototypes: bodies stay baked at their placement, one per
  instance (ADR-0025 §5). A consumer that wants the unplaced prototype
  inverts the occurrence's composed placement; instancing in topology is
  not asked for.
- Layers, PMI, materials, properties, part numbers beyond the name, and
  edition-3 sections (still not read).
- `CARTESIAN_TRANSFORMATION_OPERATOR_3D` placements: still refused, and a
  tree occurrence under one carries the refusal (backlog line stands).
- Colour models other than plain RGB (`COLOUR_RGB`, and the
  `DRAUGHTING_PRE_DEFINED_COLOUR` names that are RGB in disguise);
  transparency, textures, per-curve colours.
- Any change to `Body`, `Model` or `arris-topo`: the tree is an
  `arris-io` value, not topology (asks idea §Constraints).

## Design deltas

- **ADR-0033 (new; amends ADR-0025 §5 "Instances")**: the product tree is
  an `arris_io::step` value beside the bodies; bodies stay flattened and
  baked; occurrences are the paths the flattener already numbers
  (`FileEntity::instance`), so the tree and the bodies agree on what
  "instance k" is. Names the reference modules read in Open CASCADE
  (`STEPCAFControl`, `StepToTopoDS` product handling).
- **Public types in `arris_io::step` (new)**: `ProductTree { roots:
  Vec<Occurrence> }`, `Occurrence { product: Option<u64> (the
  `PRODUCT_DEFINITION` file id; `None` for a representation no product
  defines), name: String, placement: Result<Isometry,
  Refusal> (in the parent, in `ReadOptions::length_unit`; identity at a
  root), colour: Option<Rgb>, solids: Vec<usize>, children:
  Vec<Occurrence> }`, `Rgb([f64; 3])`, and `FaceColour { solid: usize,
  face: FaceId, colour: Rgb }` held in `ProductTree::faces`.
- **`Read` gains `products: ProductTree`** — a pub field added to a pub
  struct: `### Breaking` in `CHANGELOG.md`, fix "add `products: _` to a
  struct pattern / ignore with `..`". Named in the commit body.
- **`step::write_products(&Model, &[Body], &ProductTree, &Control) ->
  Result<String, StepError>`** (new; `write` keeps its signature and is
  the one-product case). `StepError` gains variants for a tree that
  references a body out of range, a body placed under two occurrences as
  prototypes without a placement, a non-finite placement, a cycle —
  exhaustive enum, so also `### Breaking`.
- **Reader layering**: `step/reader/assembly.rs` keeps flattening;
  a new `step/reader/products.rs` builds the tree from
  `PRODUCT_DEFINITION`, `NEXT_ASSEMBLY_USAGE_OCCURRENCE` (names, the
  `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` placement already read) and
  `MAPPED_ITEM` assemblies, and `colours.rs` resolves
  `STYLED_ITEM → PRESENTATION_STYLE_ASSIGNMENT → SURFACE_STYLE_USAGE →
  SURFACE_STYLE_FILL_AREA → FILL_AREA_STYLE_COLOUR → COLOUR_RGB`. The
  presentation instances `assembly.rs` skips as placements are exactly
  these.
- **DATA-MODEL / ARCHITECTURE**: §Formats and tools, "The STEP reader"
  ("What it does not read: product names, colours, …") and the writer
  paragraph change; no data-model section changes (no topology change).
- **Cancellation (ADR-0030)**: the tree walk ticks once per occurrence;
  the writer ticks per occurrence and per body as it does today.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — The oracle first. Extend
  `tools/oracle/occt_assembly.py` to name every part and the assembly,
  colour the parts (distinct RGB each, one face of one part coloured
  apart), nest one sub-assembly (an assembly inside the assembly, placed
  once, so paths are two deep), and print the expected tree as JSON beside
  the volumes and centroids; `arris_debug::oracle::occt_assembly` parses
  the tree. Test: the oracle's selftest and the existing
  `an_assembly_reads_to_a_body_per_placed_solid` still pass unchanged
  (the flattened expectation does not move).
- [x] Step 2 **[2]** — ADR-0033, and `ProductTree`/`Occurrence`/`Rgb` with
  the tree read from structure and names alone (`reader/products.rs`):
  roots, children, names, placements (composed the way `assembly.rs`
  composes them, one edge at a time, in the parent's unit), `solids`
  indices agreeing with `FileEntity::instance`. `Read::products` added
  (`Breaking` bullet). Test: Open CASCADE's XCAF file from step 1 reads to
  the oracle's tree — names, nesting, every placement to the solid's own
  tolerance, each leaf's `solids` pointing at the body whose volume and
  centroid the oracle measured. A plain single-product file (Arris's and
  the C4 corpus's) reads to one root with one occurrence.
- [x] Step 3 **[2]** — Shapes the corpus really has: a part with no
  `NEXT_ASSEMBLY_USAGE_OCCURRENCE` (lone part, one root), a file with two
  roots, a `MAPPED_ITEM` assembly, an occurrence under a refused
  placement (`placement: Err`, its solids refused as today), an empty
  product name, a cycle of usages (not followed, as in `assembly.rs`).
  Test: a unit fixture of Part 21 text for each, under
  `crates/arris-io/tests/`; then the 38 real parts read with the tree
  built and the C4 histogram unchanged (the refusal table and the
  battery's counts print identically).
- [ ] Step 4 **[2]** — Colours read (`reader/colours.rs`): a solid's
  colour on its occurrence, a face's in `ProductTree::faces`, both resolved
  from `STYLED_ITEM` through the style chain; an item coloured twice takes
  the lowest `STYLED_ITEM` id (deterministic); anything but plain RGB is
  skipped, not refused — a colour is never a reason to lose a body.
  Test: the XCAF file's colours, including the one face, match the
  oracle's to the last digit the file spells.
- [ ] Step 5 **[2]** — The writer: `write_products` and its `StepError`
  variants. One `PRODUCT` per distinct product, one
  `NEXT_ASSEMBLY_USAGE_OCCURRENCE` + `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`
  + `ITEM_DEFINED_TRANSFORMATION` per placed child, names written, colours
  as `STYLED_ITEM`s; bodies in the tree's order so ids are deterministic;
  a non-rigid or non-finite placement is a typed error. Test: Open
  CASCADE's XCAF reader reads Arris's assembly to the same names,
  placements and volumes (the oracle script gains a read side), and
  `write` of one body is byte-identical to before (existing
  `step_round_trip` goldens).
- [ ] Step 6 **[1]** — Round trip as a property. A random tree over random
  corpus bodies (depth ≤ 3, fan-out ≤ 3, random rigid placements, random
  names from a fixed alphabet with the quote and backslash cases, random
  colours) written by Arris reads back to an equal `ProductTree` and, per
  instance, the same volume and centroid as the body composed with its
  path. Seeded, sharded (`prop_shards!`), in the `fast` profile at 32
  cases. Also the cancellation property covers the new entry points, and
  the reader fuzz target's seed corpus gains an assembly.
- [ ] Step 7 **[1]** — `arris` facade and `arris-debug`: re-export the
  tree types, the recipe/corpus runner records the tree size per real part
  in the battery line (printed, not asserted beyond "reads"), rustdoc with
  an example on `ProductTree`, `write_products`; `CHANGELOG.md` bullets
  under `Unreleased` (a consumer can now read and write assemblies) and
  `### Breaking` (the two changes above).

## Acceptance

`cargo nextest run --profile full -p arris-io -p arris-debug` with the
oracle available: (1) Open CASCADE's XCAF assembly — nested, named, placed,
two parts coloured and one face — reads to the oracle's tree and
unchanged flattened bodies; (2) Arris's own written assembly reads back to
an equal tree, and Open CASCADE's XCAF reader reads it to the same names
and placements; (3) the round-trip property at 256 cases, and the
interruption property, green; (4) the 38 real parts read with the tree
built and the refusal histogram identical to ADR-0026's; (5) the
semver gate passes on the `Breaking` bullets.

## Docs to update on completion

- `docs/ROADMAP.md` §C5 — A4 done with the date and ADR-0033; status line;
  the **Accept** sentence for the product tree is met.
- `docs/ARCHITECTURE.md` §Formats and tools — "The STEP reader" (the tree,
  names and colours no longer in "what it does not read") and the writer
  (`write_products`).
- `docs/adr/README.md` — ADR-0033 row; ADR-0025 gets the amendment note
  the ADR process asks for (append-only: a line under its §5 pointing to
  0033, not an edit of its text).
- `docs/ideas/plugin-cad-consumer-asks.md` — A4 marked done; if every ask
  is now done or a backlog line, the idea is retired per the lifecycle.
- `docs/BACKLOG.md` — colours other than RGB, layers/PMI, a reader for
  `CARTESIAN_TRANSFORMATION_OPERATOR_3D` stays as is; add "prototype
  sharing: un-baked bodies per product" if the agent judged it worth a line.
- `AGENTS.md` current state — C5's "STEP product structure to go" becomes
  done; the **Next** line names the blend network and the binding.
- `CHANGELOG.md` — written by step 7 and checked here.

## Open questions

- ⚠ OPEN: is a face colour part of A4 or a stretch? Plan includes it
  (`ProductTree::faces`) because XCAF colours faces as readily as solids.
  Decides: agent, by step 4; drop to solid-only if the chain proves
  exporter-specific and say so in the ADR.
- ⚠ OPEN: does the writer take `ProductTree` directly (indices into the
  `bodies` slice) or a separate input type with `Body` handles? Plan
  assumes the former so a read tree can be written back unchanged.
  Decides: agent, by step 2 (the ADR records it).
  **Decided (step 2, ADR-0033 §4): `ProductTree` directly.**
- ⚠ OPEN: `Occurrence::placement` as `Result<Isometry, Refusal>` versus
  dropping an occurrence under a refused placement. Plan keeps it with its
  refusal so the consumer sees the part exists. Decides: agent, by step 3.
  **Decided (step 2, ADR-0033 §3): kept with its refusal.**
