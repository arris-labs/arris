# Plan: body-bytes

- Started: 2026-09-27
- Milestone: C5 — the consumer's API, its second line (docs/ROADMAP.md §C5, "Body bytes (A2)")
- Idea: `docs/ideas/body-bytes.md` (absorbed); the ask is
  `docs/ideas/plugin-cad-consumer-asks.md` A2
- Idea (verbatim from the human): "body-bytes"

## Goal

One body and its `Provenance` leave a model as bytes and enter another,
in the same process or another, today or releases later.
`arris_io::body::write` imports the body into a fresh model under the
writer's `Precision` (dense ids from zero, ADR-0010) and encodes that
model, the record in the writer's ids and the writer → dense map, under a magic and
`BODY_VERSION` — `postcard` for storage, JSON beside it for diffs, both
deterministic byte for byte. `arris_io::body::read` decodes, migrates an
earlier version through the chain in `body::compat`, refuses a newer one,
refuses a tolerance outside the target's `[min_tolerance,
max_tolerance]`, imports into the caller's model inside a transaction,
runs the checker at `Full` in every build profile, and returns the new
body, its record, and the writer → caller map — or a typed error with
the model left as it was. Entities of the record outside the body (a
boolean's inputs) are reported as foreign; `IdMap::inverse` lets the
consumer that sent the inputs translate them back, and
`Imported::translated` puts the whole record in its ids in one pass. One blessed
bytes file per released version, covering every curve, surface and
pcurve kind, is read in the suite forever, so a model change that
alters the decoding of an old file fails a test instead of shipping.

## Non-goals

- The whole-model native format keeps its `NativeError::Version`
  refusal (idea decision 6); it may adopt the compat machinery later.
- No dedicated wire schema (the idea's option B) and no self-describing
  encoding (option C). A second consumer writing bodies in another
  language is what reopens B.
- No rescaling of a body written under another `Precision`, and no
  units: those stay the consumer's.
- No forward compatibility: an older kernel refuses a newer file.
- No multi-body files, no assembly structure (that is A4), no Python
  API (the binding waits on this plan, it is not in it).

## Design deltas

- **ADR-0029, body bytes** (step 1): model types as the wire with
  per-version frozen fixtures as the guard and compat modules only when a
  type changes (option A); every earlier version reads by chained
  migrations, dropping one is a `Breaking` line and an ADR amendment;
  newer versions refused; foreign origins written as they are, in the
  writer's ids; `Full` check on every read in every profile (as
  ADR-0028 decision 3); precision mismatch refused, not rescaled. It
  amends `docs/DATA-MODEL.md` §Native format's refusal for bodies only.
- **Decided here, recorded in the ADR:** the chain's machinery is
  proven before any second version exists by a `#[cfg(test)]` version 0
  (a unit test inside `body::compat`), read through the public `read`
  end to end. No format is shipped that no release ever wrote, and the
  roadmap's "the previous version's bytes read and migrate" is held by
  that test until v2 exists and by the v1 guard file from then on.
- **Decided here, corrected in step 4 (ADR-0029's amendment):** the
  record travels in the writer's ids beside the writer → dense `IdMap`.
  A record mapped to dense ids cannot tell a foreign origin from a body
  entity that has the same id, and neither a stored flag nor a computed
  one fixes that. `read` returns the writer → caller map, `foreign()`,
  and `translated(&IdMap)` to put everything in the caller's ids in one
  pass.
- **Decided here:** a magic precedes the version, so body bytes and
  native model bytes (both starting with the varint `1`) can never be
  mistaken for each other.
- **New public API, `arris-io`** (behind the existing `serde` feature):
  - `pub mod body`, `pub const BODY_VERSION: u32 = 1`,
    `pub const BODY_MAGIC: [u8; 8]`.
  - `pub fn write(model: &Model, body: Body, record: &Provenance) -> Result<Vec<u8>, BodyError>`,
    `pub fn to_json(…) -> Result<String, BodyError>` with the same inputs.
  - `pub fn read(model: &mut Model, bytes: &[u8]) -> Result<Imported, BodyError>`,
    `pub fn from_json(model: &mut Model, text: &str) -> Result<Imported, BodyError>`.
  - `pub struct Imported { pub body: Body, pub provenance: Provenance, pub map: IdMap, pub version: u32 }`
    — the record as written and the writer → caller map — with
    `fn foreign(&self) -> Vec<Shape>` (writer's ids, ascending) and
    `fn translated(&self, foreign: &IdMap) -> Provenance`.
  - `pub enum BodyError { Magic, Version { found, newest }, Encode(String), Decode(String), Precision { entity: EntityId, tolerance: f64, min: f64, max: f64 }, Rejected(Report), Topo(TopoError) }`
    — the exact variant set is fixed by step 2 and 3's tests.
  - `pub mod compat` (doc: the procedure for a bump), holding no
    version module until v2.
- **New public API, `arris-topo`:** `IdMap::inverse(&self) -> IdMap`
  (the map is injective by construction of `import`).
- **Found in step 2, `arris-topo`:** `Provenance`'s `serde` form writes
  each relation's map as a sequence of `(origin, outputs)` pairs, since
  JSON has no map keyed by an `Origin`. `postcard` encodes a map entry and
  a pair alike, so the native format's bytes are unchanged (a test holds
  it). Reading refuses origins out of order and an output listed twice.
- **Crate boundary:** unchanged. `body` lives in `arris-io`, above
  `check` (ADR-0013), so it runs the checker on read.
- **Fixture tree:** unchanged. The guard is `arris-io`'s own test data,
  `crates/arris-io/tests/body/v<N>/`, beside the test that reads it and
  packaged with the crate (step 5); the corpus lint never sees it.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[1]** — ADR-0029, body bytes: the six decisions of the
  idea and the three decided here, the idea's options as the
  alternatives, and its amendment of DATA-MODEL §Native format; the ADR
  index updated. Docs only.
- [x] Step 2 **[2]** — `body::write`, `read`, `to_json`, `from_json`
  at v1 for the happy path: write through `import` into a fresh model
  with the writer's `Precision` and `Provenance::mapped`; read decodes
  into a scratch model and imports into the caller's inside a
  transaction. Tests in `crates/arris-io/tests/body.rs`: the box,
  cylinder, frame and NURBS box round-trip through both encodings into
  a fresh model to the same dump, measures bit for bit and the record
  equal to the written one mapped; into a model that already holds
  bodies, the same up to the returned map; the same body written from a
  model full of holes and from a dense copy gives the same dense model
  (the same bytes until step 4 put the record in the writer's ids); two
  writes are identical; native model bytes are `BodyError::Magic`.
- [x] Step 3 **[2]** — trust and precision: every refusal typed, the
  target model unchanged on every `Err` (its native bytes equal before
  and after). The checker at `Full` in every profile, `Rejected` with its
  report; a tolerance outside the target's range is `Precision` naming
  the entity in the writer's ids; hand-crafted JSON with a dangling
  reference (`Topo`), a body turned inside out and a vertex moved off its
  edges (`Rejected`; a face crossing itself is not reachable by editing
  JSON, and ADR-0028's crossing frame is `ops::build`'s test), and a
  truncated stream (`Decode`) each fail typed, never panic. `from_json`
  reads a tree first, so truncated JSON is `Decode`, not `Magic`.
- [x] Step 4 **[2]** — foreign origins: `IdMap::inverse` with its
  doctest; `Imported::foreign` and `Imported::translated`, the record
  carried in the writer's ids with the writer → dense map (ADR-0029's
  amendment), a map not one to one onto the body refused. The plugin round trip as a test:
  model A holds the frame and a cutter, both imported into model B, B
  cuts, the result and its record are written, read back into A; every
  foreign origin, sent through the inverse of B's import maps, is an
  entity of A's operands, and the record's chains from A's operand
  faces reach the read body's faces as `provenance/split-frame-cut`
  reaches them in-process.
- [x] Step 5 **[2]** — the v1 guard: one blessed `.bin` and `.json`
  per body under the guard directory, for a set that covers every
  `Curve`, `Surface` and `Curve2` kind (the traced and fitted sections,
  NURBS, the elliptic cylinder, the torus), a consumer-keyed record from
  the `build/` recipe and a boolean's record with foreign origins; each
  is read in the suite, checker-green, its dump equal to the committed
  one. A coverage test matches exhaustively on every geometry kind, so a
  new kind fails to compile until the guard has a body with it. Blessing
  (`ARRIS_BLESS=1`) writes a version's files once and refuses to
  overwrite an existing one.
- [x] Step 6 **[2]** — the chain: `read` dispatches on the version
  through `body::compat`, migrating one version at a time to the
  newest; `found > BODY_VERSION` is `Version`. The `#[cfg(test)]`
  version 0 — a v1 body with one field shaped differently — is written
  as bytes and read through `read` to the same body as its v1 twin. The
  module doc states the procedure for a bump (freeze the touched types'
  old shape into `compat::vN`, bump, write `vN → vN+1`, bless the new
  guard beside the old).
- [x] Step 7 **[2]** — the round-trip property: bodies drawn by
  `prop::recipe`, written and read into a fresh model and into a
  populated one — checker green, counts, mass properties bit for bit,
  record equal to the written one mapped, a re-write the same geometry
  and topology as the first and the same bytes from the second on (the
  first carries the writer's ids in its record and map, found in the
  step) — sharded with `prop_shards!` at the configured case count.
- [ ] Step 8 **[1]** — a `body_read` fuzz target in `fuzz/`, seeded
  from the guard's bytes: any input is `Ok` with a checker-green body or
  a typed error, the target model unchanged on `Err`; `nightly.yml` runs
  it 30 minutes beside the other four.

## Acceptance

`cargo nextest run -p arris-io --test body` green: a body built under a
consumer role (`build/tetrahedron`), written as body bytes and imported
into a fresh model, is checker-green with the same counts, measures and
record (step 2 and 5); the test-only previous version reads and migrates
through `read` (step 6), and every blessed v1 guard file reads to its
committed dump (step 5); the round-trip property green at CI's case
count on the fixed seed (step 7). The body half of C5's accept line.

## Docs to update on completion

- `docs/DATA-MODEL.md` §Native format — a paragraph on body bytes: what
  is written, the magic and `BODY_VERSION`, the compatibility policy and
  the guard, the foreign-origin rule; the "a file of another version is
  `NativeError::Version`" sentence scoped to the whole-model format.
- `docs/DATA-MODEL.md` §Provenance — a record crossing models:
  `mapped`, foreign origins, `IdMap::inverse`, `Imported::translated`;
  and the overlap `mapped` has when a record names entities outside the
  imported body and the target is not fresh (ADR-0029's amendment).
- `docs/ARCHITECTURE.md` — the `arris-io` row of the crate table (`body`
  beside `native`); §Formats and tools, the body bytes entry beside the
  native one; the measuring harness's fuzz-target count (five).
- `docs/ROADMAP.md` §C5 — the status line (body bytes done, ADR-0029)
  and the "Body bytes (A2)" bullet compressed to what landed; "Beside the
  cycles", the binding's body-bytes interop no longer waiting; the fuzz
  paragraph's fourth-to-fifth target.
- `docs/ideas/plugin-cad-consumer-asks.md` — A2 marked absorbed.
- `CHANGELOG.md` `Unreleased` — what a consumer can now do (write and
  read a body with its record, across releases) and the refusals it
  will hit; `IdMap::inverse`.
- `AGENTS.md` current state — only if C5 closes with it (it does not).

## Open questions

- Decided in step 5: the guard lives in `crates/arris-io/tests/body/v<N>/`
  (`<name>.bin`, `.json`, `.dump.txt`), read by `tests/body/guard.rs`.
  v1's set is six bodies, five of them corpus results
  (`boolean/revolve-extrude-fuse-cut-rounding-knots` for every curve and
  pcurve kind and the cone, sphere and torus; `boolean/ring-pin-cut` for
  the torus walker's traced section; `boolean/elliptic-operand-cut`;
  `build/tetrahedron`; `provenance/split-frame-cut` for foreign origins)
  and `sample::cuboid_nurbs` for the NURBS surface, which no recipe op
  builds. The dump is the body's text dump, the record as written and the
  foreign list.
