# Plan: consumer-roles

- Started: 2026-09-27
- Milestone: C5 — the consumer's API, its first line (docs/ROADMAP.md §C5, "Consumer roles (A1)")
- Idea: `docs/ideas/plugin-cad-consumer-asks.md` A1 (accepted 2026-09-26;
  the file stays until C5's last plan absorbs it — it holds A2–A4 and A11 too)
- Idea (verbatim from the human): "/plan consumer-roles"

## Goal

A consumer that builds topology itself roots its provenance chains at keys
of its own. `Role::Consumer(ConsumerKey { namespace, key })` exists,
opaque to the kernel. `ops::build` takes a `Builder` the consumer
filled — by Euler operators or `Builder::assemble` — with a key for
every slot, and returns a checker-green body whose every entity is
`Generated` from its key, or a typed refusal that leaves the model as it
was. A record from any other operation that creates from nothing (a
primitive, a sweep, the STEP reader) is re-rooted at the consumer's
keys by `Provenance::rerooted`, so every chain a plugin feature makes
can end in the plugin's own words. A body built this way is a corpus
operand: a `polyhedron` recipe op builds it in both kernels, its
fixtures match Open CASCADE, and a rebuild with moved vertices keeps
every chain ending at the same keys in the same split order.

## Non-goals

- A name grammar or a lineage value (ADR-0009 stands; the backlog line
  on `Provenance::lineage` stays a backlog line).
- Sheet, wire or general bodies from `ops::build`: `Builder::finish`
  refuses every kind but `Solid`, and sheets are the healing cycle's.
- Body bytes (A2): serialising a consumer-rooted record is that plan's.
  `Role` derives serde already; the new variant is appended last, so an
  encoding of the existing variants does not move.
- Changing the signature of `primitive_box`, `primitive_cylinder`,
  `extrude`, `revolve` or `step::read` (see the design delta on
  re-rooting).
- `polyhedron` in the measuring harness's random recipes (`prop::recipe`
  and the differential): a backlog line if a later cycle wants it.
- Faces that are not planar in `polyhedron`: a curved face is what a
  consumer builds through `ops::build` directly, and the op exists to
  give the corpus an oracle for it, not to be a modelling feature.

## Design deltas

- **`arris-topo` `provenance`** (breaking, `CHANGELOG.md` `### Breaking`
  in step 1): `Role::Consumer(ConsumerKey)` and `pub struct ConsumerKey
  { pub namespace: u32, pub key: u64 }`, Copy/Ord/Hash/serde like
  `FileEntity`, `Display` as `consumer:{namespace}/{key}`. A tuple
  variant over a named struct, as `Role::File(FileEntity)` is
  (ADR-0025), rather than the roadmap's struct-variant spelling — the
  same fields, one idiom in the enum. The kernel never reads either
  field; ordering is by `(namespace, key)`, which is what fixes the
  record's iteration order.
- **`arris-topo` `provenance`** (additive): `Provenance::rerooted(&self,
  f: impl Fn(Role) -> Role) -> Provenance` — every `Origin::Role(r)`
  replaced by `Origin::Role(f(r))`, entity origins and `deleted`
  untouched. Two roles mapped to one concatenate their outputs in the
  old roles' order, deduplicated, so the result is deterministic even
  for a map that is not injective; the rustdoc says an injective map is
  what keeps `Split(k)` meaningful.
- **`arris-ops`** (additive): `ops::build(model: &mut Model, builder:
  Builder, keys: &BuildKeys) -> Result<(Body, Provenance), OpError>`
  with `pub struct BuildKeys { pub namespace: u32, pub vertices:
  BTreeMap<VertexRef, u64>, pub edges: BTreeMap<EdgeRef, u64>, pub
  faces: BTreeMap<FaceRef, u64>, pub shells: Vec<u64>, pub body: u64 }`.
  Keys are not required to be unique: two slots under one key are two
  outputs of one role, in slot order, as a box's shell and body are
  of one part kind. Refusals, each with the model untouched: a live
  slot with no key (a new `OpError::Unkeyed { slot }` naming it — a
  record with an unrooted output would fail `audit`), the builder's
  own `BuildError`, and a body the checker rejects. The last is the
  consumer's input, not a kernel bug, so `ops::build` runs the checker
  in **every** build profile, not only debug, and returns a new
  `OpError::Rejected { report }` instead of `verify`'s panic. Two new
  `OpError` variants are breaking (`OpError` is exhaustive to its
  callers' matches) — named in step 2's `### Breaking` bullet.
- **Where "taken by every operation that creates from nothing" lands.**
  The roadmap line reads as each such operation taking a key. This plan
  gives them `Provenance::rerooted` instead, and records why in
  ADR-0028: a primitive's, sweep's or file's role already names the
  part of that call exactly, so the consumer's key only has to be
  *prefixed* to it, and a map `Role → Role` does that for all five
  operations and every future one without a signature change on any.
  `ops::build` is the one operation where nothing names the parts but
  the consumer, so it is the one that takes keys.
- **The recipe grammar** (`arris_debug::fixtures`, `tools/oracle/oracle/
  recipe.py`, `tests/fixtures/README.md`): a `polyhedron` op — `points`
  (expressions), `faces` (each a list of loops, each a list of point
  indices, counter-clockwise seen from outside for the outer loop,
  clockwise for a hole), `namespace`. Arris: an `Assembly` of new
  vertices, line edges keyed by their sorted point-index pair, planar
  faces with line pcurves, through `Builder::assemble` and `ops::build`
  with keys point `i` → `i`, face `j` → `j`, edge `{a, b}` → `a <<
  32 | b`, shell and body `0` — the recipe's own convention, not the
  kernel's. Open CASCADE: polygon wires to planar faces, sewn, a solid.
- **Corpus lint**: a new area `build/` held to passing, blessed fixtures
  like the others.
- **ADR-0028** (new): the consumer's key — the opaque key, `ops::build`
  checking its input in every profile, re-rooting over per-operation
  key parameters.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[1]** — `Role::Consumer(ConsumerKey)` in `arris-topo`,
  with `Display`, serde and the doc example; every exhaustive `match`
  on `Role` in the workspace given its arm (the reader's, `arris-debug`'s
  test helpers, the dump). ADR-0028 written. Tests: the key round-trips
  through serde JSON and postcard with the existing variants' encodings
  unchanged (a byte-exact check against a record of each old variant);
  `Display`; ordering by `(namespace, key)`. `CHANGELOG.md`
  `### Breaking`: the new variant, fix "add an arm".
- [ ] Step 2 **[2]** — `ops::build` and `BuildKeys`, with
  `OpError::Unkeyed` and `OpError::Rejected`. The record is built the
  way the primitives build theirs (the private `roles` helper in
  `primitive.rs` generalised and shared). Tests in
  `crates/arris-ops/tests/build.rs`: a tetrahedron by Euler operators and
  a square frame (genus 1) by `Builder::assemble`, each checker-green at
  `Full`, `audit` clean, every entity `Generated` from exactly its key;
  a missing face key → `Unkeyed` naming it; a tetrahedron with one face
  flipped → `Rejected` in a release-profile test as well
  (`cargo test --release -p arris-ops --test build` in the step's
  checks); on every refusal the model equals its clone from before.
  Rustdoc example: the tetrahedron. `CHANGELOG.md` `### Breaking`: the
  two `OpError` variants.
- [ ] Step 3 **[2]** — the `polyhedron` recipe op in both interpreters
  and the `build/` area in the lint. Fixtures with oracle values:
  `build/tetrahedron` (and a variant in a skewed pose),
  `build/l-prism` (a concave edge, eight vertices), `build/frame` (a
  square prism with a square through-hole: faces with an inner loop,
  genus 1). Each passes the whole runner — checker at `Full`, measures,
  probes, STEP both ways, NURBS read-back — and has its dump blessed.
  The oracle's self-test reproduces the three `expected.json`.
- [ ] Step 4 **[1]** — `Provenance::rerooted`. Tests: the identity map
  returns an equal record; an injective map then its inverse returns the
  original; a non-injective map concatenates in the old roles' order;
  `audit` holds on a re-rooted primitive's, sweep's and read solid's
  record; `rerooted` commutes with `then` over a primitive's record
  followed by a cut (re-root then compose equals compose then re-root).
  Rustdoc example: a box's top re-rooted at a consumer key.
- [ ] Step 5 **[2]** — the stability claim, on a consumer's body. A
  fixture `provenance/consumer-rebuild`: `build/l-prism`'s polyhedron
  cut by a cylinder through its concave corner and filleted along one
  kept edge, in three variants that move vertices without changing
  which faces bound which piece; every output's chain through `then`
  ends at the same `Role::Consumer` keys in the same split order in all
  three (the check `bolt-pattern-rebuild` makes, with consumer roots).
  A property test in `crates/arris/tests/provenance.rs`: a `build/`
  fixture's body in a random rigid pose, cut by a random box, then its
  record composed — `audit` holds and every chain of every output ends
  at a `Role::Consumer` or at the box's roles, never nowhere; seeded,
  `prop_shards!` like its neighbours.

## Acceptance

`cargo test --workspace` green with the corpus: the three `build/`
fixtures and `provenance/consumer-rebuild` passing against their
`expected.json`, blessed, and the lint holding `build/` to zero ignored
fixtures; `uv run --project tools/oracle tools/oracle/selftest.py`
reproducing the four new `expected.json`; the step-5 property test green
at the hook's and CI's case counts; `ops::build`'s refusal test green in
the release profile; `tools/semver-gate.sh` passing with the two
`### Breaking` bullets present.

## Docs to update on completion

- `docs/DATA-MODEL.md` §Provenance — `Role::Consumer(ConsumerKey)` in the
  enum sketch; a paragraph on `ops::build`'s record (every entity
  `Generated` from its key, keys not unique) and on `rerooted`, its
  merge order; the `Stability` paragraph naming consumer roots as a
  chain's end.
- `docs/ARCHITECTURE.md` — `ops::build` in the operations list and the
  error table (`Unkeyed`, `Rejected`, and that `build` checks its result
  in every profile where other operations check only in debug).
- `docs/ROADMAP.md` §C5 — the "Consumer roles (A1)" line becomes done,
  reworded to the landed shape (`ops::build` takes keys; the other
  creating operations are re-rooted), citing ADR-0028.
- `tests/fixtures/README.md` — the `polyhedron` op in the recipe grammar
  and `build/` in the lint's list of areas.
- `CHANGELOG.md` `## Unreleased` — the feature bullets: a consumer builds
  a solid from its own topology with its own names on every entity, and
  re-roots any other operation's record at its own names; the refusals
  (`Unkeyed`, `Rejected`).
- `docs/ideas/plugin-cad-consumer-asks.md` — Status line gains "A1 landed
  (plans/consumer-roles)"; the file is deleted by C5's last plan, not
  this one.
- `AGENTS.md` current state — nothing until C5 closes.

## Open questions

- ⚠ OPEN: does `ops::build` check at `Level::Fast` or `Level::Full`?
  ADR-0028 stays `proposed` until this is answered.
  `Full` catches self-intersecting input the consumer is likely to hand
  it, at a cost on large bodies. Agent decides by step 2, on the
  frame's and the tetrahedron's timings and on what `Fast` misses of a
  hand-built bad body; the ADR records it.
- Decided (step 1, ADR-0028 §4): `BuildKeys` does not refuse a key
  reused across kinds. Refusing it would make the kernel decide a key
  names one kind of entity; each output keeps its kind in its `Shape`,
  so a mixed `generated_from` can still be read.
- Found (step 1): a `Provenance` has no JSON form (its maps are keyed by
  `Origin`, which JSON can't use as an object key), so the step's
  round-trip of a record runs through postcard only; the `Role` itself
  round-trips through both.
