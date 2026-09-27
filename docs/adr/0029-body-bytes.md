# ADR-0029 — Body bytes: the model's types as the wire, a frozen file per version as the guard, every earlier version migrated

- Status: accepted (2026-09-27)
- Plan: `body-bytes` steps 2–8
- Follows: ADR-0010 (sparse slots; dense ids come from `import`),
  ADR-0009 (the split order a record's outputs keep), ADR-0013 (the
  layer order that puts this in `arris-io`), ADR-0028 (a consumer-rooted
  record, and `Full` on a consumer's input in every profile)
- Amends: `docs/DATA-MODEL.md` §Native format, for bodies only

## Context

The plugin CAD (`docs/ideas/plugin-cad-consumer-asks.md` A2) freezes a
feature's result into the user's file, and its Python plugins run out of
process. One body and its `Provenance` therefore have to leave a model as
bytes and enter another model, in another process or under a later
release. The only format today is `arris_io::native`: the whole model,
freed slots included, `serde` of the in-memory types, refusing any
`NATIVE_VERSION` but its own. A user's file outlives the release that
wrote it, so a refusal there loses the user's work.

`Model::import` already deep-copies a body's closure into another model
inside a transaction and returns an `IdMap`. `Provenance::mapped`
translates a record through that map, and importing into a fresh model
gives dense ids from zero. A file needs four things an in-process copy
does not: a schema that can migrate, a meaning for origins outside the
body, distrust of its input, and a rule for a `Precision` that differs.

## Decision

1. **The model's types are the wire.** `arris_io::body::write` imports
   the body into a fresh model under the writer's `Precision` and encodes
   that model and the record, mapped through the import, as `serde` under
   a magic and `BODY_VERSION`. The encodings are `postcard` for storage
   and JSON for diffs, as native uses. Both are deterministic byte for
   byte, and the same body gives the same bytes whatever holes its model
   had. No separate wire schema exists until a type changes. **The
   commit that changes a type the bytes carry** copies the old shape of
   only the types it touched into `body::compat::vN`, bumps
   `BODY_VERSION` and writes the `vN → vN+1` migration.
2. **The guard is a frozen file per version.** Each version has one
   blessed bytes file per guard body, and together those bodies cover
   every `Curve`, `Surface` and `Curve2` kind, a consumer-keyed record
   and a boolean's record. The suite reads every version's files forever
   and checks each against its committed dump. A change to the model that
   alters how an old file decodes therefore fails a test and cannot ship
   by accident. Blessing writes a version's files once and never
   overwrites them. An exhaustive `match` over the geometry kinds keeps
   the coverage complete: a new kind fails to compile until a guard body
   carries it.
3. **Every earlier version reads**, by migrations chained one version at
   a time up to the newest, because a user's file has no expiry date.
   Dropping a version is a `Breaking` changelog line and an amendment of
   this record. **A newer version is refused** as
   `BodyError::Version`, because an older kernel cannot know what a newer
   field means.
4. **The chain is proven before a second version exists.** A
   `#[cfg(test)]` version 0 (a v1 body with one field shaped differently)
   is written as bytes and read through the public `read`. The migration
   path therefore runs in the suite from the day it is written, and no
   format ships that no release ever wrote. Until v2 exists, that test
   holds the roadmap's line that "the previous version's bytes read and
   migrate". After that, the v1 guard files hold it.
5. **Foreign origins are written as they are.** A record's origins
   outside the body (a boolean's inputs, its `deleted`) stay in the
   writer's ids. `mapped` already leaves them untouched. On read, an
   origin that the import's map does not hold is foreign
   (`Imported::foreign_origins`). This is computed, not stored, because
   the bytes carry nothing that can be derived. A consumer that sent the
   inputs across translates them back with `IdMap::inverse` of the map
   its own import returned.
6. **Every read is checked at `Full`, in every build profile.** `read`
   decodes into a scratch model, then imports into the caller's model
   inside a transaction and runs the checker at `Level::Full`. A
   violation is `BodyError::Rejected(report)`, and the caller's model is
   left as it was, on this and on every other error. The bytes are input
   from outside the kernel, exactly as a consumer's topology is under
   ADR-0028 decision 3.
7. **A precision mismatch is refused, never rescaled.** The writer's
   `Precision` travels in the bytes. A vertex, edge or face whose
   tolerance lies outside the target's `[min_tolerance, max_tolerance]`
   is `BodyError::Precision`, naming the entity in the writer's ids. The
   other fields of `Precision` are the target's. Units are the
   consumer's concern, as they already are.
8. **A magic precedes the version.** Body bytes and native model bytes
   both begin with the varint `1`, so without it either could be
   mistaken for the other. `BodyError::Magic` refuses anything that is
   not body bytes.
9. **The whole-model native format keeps its refusal.**
   `NativeError::Version` stands for the whole model. It can adopt this
   machinery later if a consumer comes to store whole models.

## Consequences

- A consumer can freeze a feature's body into its own file and read it
  under any later release. A plugin process can hand a body and its
  record back to the application.
- Every commit that changes the encoding of a geometry or topology type
  now pays for a compat module, a migration and a new guard version.
  Decision 2 makes that cost impossible to miss, and until such a commit
  lands the cost is nothing.
- `read` costs a `Full` check in release builds, which no in-process
  `import` pays.
- `arris-io` gains `body` and `arris-topo` gains `IdMap::inverse`. Both
  are additions, so they are not `Breaking`.
- The bytes are schema-less `postcard`: only Arris reads them. A second
  consumer writing bodies in another language would reopen option B
  below, and the compat modules would become that schema's first
  versions.

## Alternatives considered

- **A dedicated wire schema from day one** (flat tables of points,
  curves, surfaces, pcurves and entities as their own `serde` structs).
  Internal refactors would never touch it, but every geometry kind would
  need a wire twin, the NURBS and the traced and fitted sections
  included, and every new kind would have a second place to update
  forever. It would buy a stability that decision 2's guard already
  gives.
- **A self-describing encoding** (CBOR or JSON values, with old versions
  rewritten as untyped trees before the typed decode). It needs no frozen
  types, but the migrations are untyped, the bytes are several times
  larger than `postcard`, and a wrong rewrite fails deep inside the
  decode.
- **Reading only version N−1**, as the roadmap first worded it. A user's
  file skips releases, so N−1 is the tested minimum, not the policy.
- **Refusing foreign origins, or requiring a record to be re-rooted
  before it leaves.** This would force every boolean's record through a
  rewrite before it could cross a process boundary, and the consumer
  that sent the inputs is the one party able to translate them back
  anyway.
- **Rescaling a body into the target's `Precision`.** A tolerance is a
  statement about how the geometry was built. Changing it silently
  claims an accuracy the body never had.
- **Storing whole native models per frozen feature.** They are full of
  holes unless imported first, much larger, and lost at the first
  version bump.

## Amendment (2026-09-27, plan `body-bytes` step 4)

**Decision 5 was wrong as worded, and decision 1 changes with it: the
record travels unmapped, in the writer's ids, beside the map from the
writer's ids to the dense ones.**

Once a record has been mapped to dense ids, the body's entities carry
dense ids and the foreign ones keep the writer's, and the two ranges
overlap. The writer's operand face `f2` and the body's own dense face `f2`
are then the same value in the record, and neither a stored flag nor a
computed one can tell them apart. In-process `Provenance::mapped` into a
model that is not fresh has the same overlap. It was harmless there only
because nobody had needed to separate the two.

- `write` encodes the dense model and the body as before. It also
  encodes the record as the writer holds it and the `IdMap` its own
  import into the fresh model returned (writer → dense). The geometry and
  topology written are still identical whatever holes the writer's model
  had. The record and the map are in the writer's ids, so they are not.
- `read` refuses a map that is not one to one onto the body's closure as
  `Decode`. It composes the map with its own import into the caller's
  model and returns the writer → caller map as `Imported::map`, with the
  record as written. A record entity that the map does not hold is
  foreign (`Imported::foreign`). This is exact, because membership is
  decided in the writer's ids, where no two entities share an id.
- `Imported::translated(foreign)` puts the record into the caller's ids
  in one pass over a single map: the body's entities through
  `Imported::map`, the foreign ones through the map the caller supplies
  (the inverse of the import that sent them). Because it is one pass, no
  id is translated twice.
- `IdMap` gains a JSON form in the same way `Provenance` did: a sequence
  of pairs in key order, with `postcard` bytes unchanged.
- `BodyError::Precision` names the entity in the writer's ids, through
  the inverse of the written map.
