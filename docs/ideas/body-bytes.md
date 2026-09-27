# Idea: body-bytes

- Status: Open
- Raised: 2026-09-27
- Prompt (verbatim from the human): "body-bytes"

## Problem

C5's A2 (`docs/ROADMAP.md` §C5, `docs/ideas/plugin-cad-consumer-asks.md`):
the plugin CAD freezes a feature's result into the user's file, and its
Python plugins run out of process, so one body and its `Provenance` have
to leave a model as bytes and enter another. Today the only format is
`arris_io::native`, which is the *whole model*, carries every freed slot,
and refuses any other `NATIVE_VERSION` (data-model §Native format). A
user's file outlives the kernel release that wrote it; a refusal there is
lost work. The first-party binding (`docs/ideas/python-binding.md`) waits
on this too.

Half of it exists. `Model::import` deep-copies a body's closure into
another model inside a transaction and returns an `IdMap`;
`Provenance::mapped` translates a record through it; importing into a
fresh model gives dense ids from zero. So writing a body is "import into
an empty model, serialise it with the record". What is missing is
everything a *file* needs that an in-process copy does not:

1. **A schema that can migrate.** Native is `serde` of the in-memory
   types. Any field added to an entity or geometry type, or a variant
   inserted rather than appended, silently changes the bytes, and nothing
   today notices.
2. **Foreign origins.** A boolean's record names its input entities
   (`Origin::Entity`, `deleted`). Those live in the writer's model, not in
   the body; `mapped` leaves them as they are, which is right in-process
   and meaningless in another process unless the consumer can translate
   them back.
3. **Trust.** Bytes from a plugin process or an old file are untrusted
   input. Native validates values as it decodes but leaves a dangling
   reference for the checker; an import into the consumer's live model
   cannot.
4. **Precision.** `import` does not compare the two models' `Precision`;
   a body written under a finer `min_tolerance` than the target's lands
   with tolerances the target says cannot exist.

## Constraints it runs into

- DATA-MODEL §Native format: "a file of another version is
  `NativeError::Version`". Body bytes amend that for bodies only; the
  whole-model format may keep its refusal.
- ADR-0028: a consumer-rooted record serialises with no special case.
- ADR-0009 / ADR-0010: split order and sparse slots — the record's output
  order must survive the round trip; dense ids come from `import`, not
  `retain`.
- ADR-0013 layer order: this lives in `arris-io` (above `check`, so it can
  run the checker on read).
- `.agents/rules/kernel.md`: determinism (same body → same bytes), typed
  errors, and "an operation that returns `Ok` returns a checker-green
  shape". ADR-0028 decision 3 already ran `Full` in every profile for
  consumer-supplied topology.
- Exhaustive enums are a feature (kernel rules §API): a new surface kind
  is a breaking change in code; in *bytes* it is compatible only if
  appended.

## Options

### A — native reused, frozen fixtures as the guard, lazy compat modules
`arris_io::body::{write, read}` (bytes; JSON beside it for diffs, as
native has). Write = `import` into a fresh model with the writer's
`Precision`, then serde of that model plus the mapped record under a
magic and a `BODY_VERSION`. The wire types *are* the model types until
the day one changes; that commit copies the old shape of only the types
it touched into `body::compat::vN`, bumps the version and writes the
`vN → vN+1` migration. The guard: one blessed bytes file per released
version under the fixtures, read, checked and dumped in the suite, so a
model change that alters decoding of an old file fails a test instead of
shipping. Cost: 4 steps + ADR. Cheap now, pays per bump, and the bump
cannot be forgotten.

### B — a dedicated wire schema from day one
Flat tables (points, curves, surfaces, pcurves, vertices, edges, faces,
shells) as their own `serde` structs, designed as a format and converted
to/from the model. Internal refactors never touch it; migrations are
typed `wire::vN → vN+1`. Cost: 6–7 steps + ADR — every geometry kind gets
a wire twin, including NURBS and the traced/fitted sections — and a
second place to update for every new surface kind forever.

### C — a self-describing encoding, migrations as tree rewrites
CBOR or JSON values; old versions are rewritten as untyped trees before
the typed decode. No frozen types, but migrations are untyped, the bytes
are several times larger than `postcard`, and a wrong rewrite fails deep
in the decode.

### Do nothing
The consumer stores whole native models per frozen feature (wasteful,
full of holes unless it imports first) and loses every user file at the
first version bump. Blocks C5's acceptance and the binding.

## Recommendation

**A.** It reuses what exists (`import`, `mapped`, native's
deterministic encoding and decode-time validation), and the fixture guard
turns "the schema is the model" from a hazard into a test. B's cost is
paid on every new geometry kind for a stability A already gets from the
fixtures; C trades types for size and late errors. What would change my
mind: a second consumer writing bodies in another language, which wants
a documented schema — that is B, and A's compat modules become its first
two versions.

The four gaps, as A answers them:

- **Foreign origins** are written in the writer's ids, flagged as not in
  the body, and left as they are on read; `IdMap::inverse` lets a
  consumer that sent the inputs across translate them back. Refusing them
  would force every boolean's record to be re-rooted before it can leave.
- **Trust**: `read` imports into the caller's model inside a transaction
  and runs the checker at `Full` in every profile, as `ops::build` does;
  a violation is a typed error and nothing is appended.
- **Precision**: the writer's `Precision` travels; `read` refuses (typed,
  naming the entity) a tolerance outside the target's `[min, max]`,
  never rescales. Units are the consumer's concern, as they are now.
- **Compatibility**: every earlier body version reads, by chained
  migrations, since a user's file has no expiry date; dropping one is a
  `Breaking` changelog line and an ADR amendment. A newer version than
  the reader's is a refusal (no forward compatibility).

## Decision for the human

1. Option A — model types as the wire, per-version fixtures as the
   guard, compat modules only when a type changes? *Preferred: yes.*
2. Read every earlier version (chain), or only N−1 as the roadmap
   words it? *Preferred: every earlier version; N−1 is the tested
   minimum, the chain is what the fixtures hold.*
3. Foreign origins written as-is with `IdMap::inverse`, rather than
   refused? *Preferred: as-is.*
4. `Full` check on every read in every profile? *Preferred: yes — same
   reasoning as ADR-0028 decision 3.*
5. Precision mismatch refused, not rescaled? *Preferred: refused.*
6. The whole-model native format keeps its refusal? *Preferred: yes, for
   now; it can adopt the same compat machinery later if a consumer stores
   models.*

An ADR is needed (roadmap §C5 says so): the compatibility policy and the
foreign-origin rule, amending DATA-MODEL §Native format's refusal for
bodies.
