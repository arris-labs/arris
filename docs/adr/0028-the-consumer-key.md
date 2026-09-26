# ADR-0028 — The consumer's key: an opaque `Role::Consumer`, taken by `ops::build`, reached by re-rooting from every other operation

- Status: proposed (2026-09-27) — accepted when `plans/consumer-roles`
  step 2 fixes the checker level of decision 3
- Plan: `consumer-roles` steps 1, 2 and 4
- Follows: ADR-0002 (provenance rooted in roles), ADR-0009 (no name
  grammar; the words are the consumer's), ADR-0025 (`Role::File`, the
  last variant appended the same way)

## Context

Every chain of provenance records ends at a `Role`: what an entity is to
the operation that made it from nothing. The roles are the kernel's
operations — `Box`, `Cylinder`, `Extrude`, `Revolve`, `File` — and a
consumer's persistent name is a function of the role a chain ends at.

The plugin CAD (`docs/ideas/plugin-cad-consumer-asks.md` A1) has plugin
features that build topology themselves, through `Builder`. Nothing names
those entities: `Builder::finish` returns a body and no record, so a
chain through a later boolean ends nowhere, and the feature has no stable
name for anything it made. A plugin feature that calls a primitive or a
sweep gets roles, but they are the kernel's words, and two features that
each make a box root their chains at the same `Box(Face(Z, Max))`.

The roadmap line (§C5) had every operation that creates from nothing take
a key. That is a new parameter on `primitive_box`, `primitive_cylinder`,
`extrude`, `revolve` and `step::read`, and on each one added later.

Two questions the plan left to this record: whether a key reused across
entity kinds is refused, and at which level `ops::build` checks its
result.

## Decision

1. **`Role::Consumer(ConsumerKey { namespace: u32, key: u64 })`**,
   appended last so every earlier variant keeps its serialised index. A
   tuple variant over a named struct, as `Role::File(FileEntity)` is. The
   kernel never reads either field: it carries the pair, orders it by
   `(namespace, key)` and prints it as `consumer:{namespace}/{key}`. What
   a key means is the consumer's (ADR-0009).
2. **`ops::build` takes the keys; the other creating operations are
   re-rooted.** `ops::build(model, builder, &BuildKeys)` finishes a
   builder the consumer filled into a solid and records every entity
   `Generated` from `Role::Consumer` of the key the consumer gave its
   slot. Nothing but the consumer names those parts, so it is the one
   operation that takes keys. A primitive's, sweep's or file's role
   already names the part of that call exactly, so the consumer only
   has to *prefix* its key to it: `Provenance::rerooted(f)` replaces
   every `Origin::Role(r)` by `Origin::Role(f(r))`, and one map does that
   for all five operations and every later one, with no signature
   change on any.
3. **`ops::build` checks its result in every build profile**, and
   returns `OpError::Rejected { report }` rather than `verify`'s
   debug-build panic. The body is the consumer's input, not the kernel's
   output, so a failure is a refusal of that input and not a kernel bug;
   a release build that skipped the check would hand an invalid body to
   the next operation. The level is recorded by step 2.
4. **Keys are not required to be unique, within a kind or across
   kinds.** Two slots under one key are two outputs of one role, in slot
   order, as a box's shell and body are two outputs of one kind of
   part. A vertex and a face under one key is allowed too: refusing it
   would make the kernel decide that a key names one kind of entity,
   which is the consumer's call. A consumer that reuses a key by mistake
   sees both entities in `generated_from(key)`, and each keeps its kind
   in its `Shape`, so the mix can be read and nothing is lost.
5. **A record from `ops::build` is audit-clean by construction:** a live
   slot with no key is `OpError::Unkeyed { slot }`, never an output with
   no origin.

## Consequences

- Every chain a plugin feature makes can end in the plugin's own words:
  at a key from `ops::build`, or at a key `rerooted` put in place of a
  primitive's, sweep's or file's role.
- `Role` gains a variant: a `match` over it needs an arm. `OpError`
  gains two (`Unkeyed`, `Rejected`), with the same fix. Both are listed
  under the changelog's `Breaking`.
- `ops::build` costs a checker run in release builds, which no other
  operation pays.
- `rerooted` with a map that is not injective merges two roles' outputs
  into one list, in the old roles' order. That is deterministic, but it
  loses `Split(k)`'s meaning across the merged roles, and the rustdoc
  says so.
- Body bytes (A2) serialise a consumer-rooted record with no special
  case: the variant is plain serde.

## Alternatives considered

- **A key parameter on every creating operation** (the roadmap's
  wording). This changes five signatures now and one per future
  operation, and it duplicates, in the consumer's key, what the
  operation's role already says. The consumer would also have to supply
  a key per part (a box has 26), or the kernel would have to invent the
  part suffix, which is a name grammar (ADR-0009).
- **A struct variant `Role::Consumer { namespace, key }`.** Same fields,
  but `Role` would then carry two idioms. `File(FileEntity)` set the
  tuple-over-struct one.
- **A string key.** It is not `Copy`, and `Role` is `Copy` everywhere.
  It also makes the kernel carry the consumer's vocabulary as text,
  which ADR-0009 keeps out. A `u64` indexes whatever table the consumer
  keeps.
- **Refusing a key reused across kinds.** See decision 4. The check
  would cost a set per build and protect only a consumer from its own
  table.
- **`ops::build` checked only in debug builds, like every other
  operation.** See decision 3.
