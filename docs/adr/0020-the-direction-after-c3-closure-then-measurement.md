# ADR-0020 — The direction after C3: closure before breadth, measured then chosen, cycles named not numbered

- Status: accepted (2026-09-20)
- Plan: `direction-rework` step 1
- Amends: `SEED.md` §6 "Toward Parasolid grade — later cycles, roughly in
  order"
- Follows: ADR-0008 and ADR-0018 (the quadric guard and the sections that
  lift it), ADR-0017 (the gate closes on the corpus; the swap and the
  regressions it finds are the consumer's)

## Context

`SEED.md` §6 lists what comes after the application gate as one line of
seven items "roughly in order": cone/sphere/torus pairs · coincident and
tangent faces · NURBS–NURBS · sweep, loft, shell, offset · fillet
networks · STEP reader and healing · sheet and wire bodies · IGES. The
order ranks by **geometric generality**: each item takes a wider class of
surface than the one before it. That is the order a kernel's table of
contents has, and at kickoff there was nothing else to rank by.

Two cycles of corpus runs later there is. What the generality order costs
shows up as four facts a reader can re-check in the tree:

**The kernel refuses its own output.** `ops::fillet`, `ops::chamfer` and
`ops::revolve` return bodies carrying tori, spheres and cones — the
surface is named in each fixture's `dump.txt`: `blend/boss-base-fillet`
and `blend/hole-rim-fillet` (torus), `blend/box-all-edges-fillet` and
`blend/box-corner-three-fillets` (sphere), `blend/hole-rim-chamfer`
(cone), `sweep/revolve-ring` (torus), `sweep/revolve-barrel` (sphere),
`sweep/revolve-frustum` (cone). The boolean's quadric guard
(`crates/arris-ops/src/boolean/pave.rs`, `fn quadric`) refuses exactly
those kinds, at two call sites: `intersect_pairs`, before any face pair
reaches the intersector, and `hit_edge_face`, before any edge is met
against a face. So a body Arris built a moment ago is not an operand
Arris accepts: fillet-then-cut is `OpError::Unsupported`. A consumer's
feature tree meets this on its second feature, and no amount of new
surface kinds elsewhere makes it go away.

**Nothing measures speed.** The workspace has no benchmark. The oracle
re-runs Open CASCADE on every test run although `recipe_hash` already
identifies an unchanged recipe. A robustness fix that costs 10× is
invisible here until a consumer reports it, at which point the cause is
several cycles back.

**The kernel has never met a part it did not design.** Every fixture is a
recipe the agent wrote, out of the operations Arris has, in poses Arris
handles. There is no reader, so there is no way to take a part from
elsewhere and find out what Arris refuses. The refusals are therefore
unranked: `SEED.md` §6's order is a guess at which one a real part hits
first, and two cycles of evidence never touched it.

**The apparatus for measuring is nearly free.** The corpus recipe grammar
already carries the same eleven operations on both kernels — `Step` in
`crates/arris-debug/src/fixtures/mod.rs` and the `op` dispatch in
`tools/oracle/oracle/recipe.py`, box, cylinder, profile, extrude,
revolve, transform, fuse, common, cut, fillet, chamfer. A generator of
random *recipes* therefore runs both kernels with no new interpreter on
either side, and every fixture already carries an oracle answer keyed by
`recipe_hash`.

One prerequisite is worth naming because it is buried: `Surface::project`
is `GeomError::Unsupported` for a NURBS surface
(`crates/arris-geom/src/project.rs`), by cycle-1 design. A reader has to
rebuild a pcurve for every edge of every face it reads, including NURBS
faces, so the reader needs that projection before it needs anything else
on this list.

Finally, the numbering itself is load-bearing in a way a reorder breaks.
`C4`, `C6` and `C7` are cited inside **accepted** ADRs — ADR-0006
(`C7`'s `General` body), ADR-0007 (blend networks are `C6`'s, the NURBS
intersector `C4`'s), ADR-0014 (`C5`'s sweep along a path), ADR-0018
(`C4`'s marcher) — and ADRs are never edited after acceptance
(`docs/README.md`). They are also in rustdoc, in test doc comments and in
two fixture descriptions. Rebinding `C6` to a different cycle makes a
document that cannot be corrected say something false.

## Decision

### 1. Closure before breadth

**Every body an Arris operation can return is a legal operand of every
other Arris operation before the kernel takes new kinds of input.**

C3 is that closure, and it stays whole: the torus pairs, the conic hits
on curved quadrics and the quadric guard lifted are the rest of the
cycle, not a nice-to-have trimmed when one consumer's corpus is green.
`fillet`, `chamfer` and `revolve` build the tori, spheres and cones the
guard refuses, so the guard is the closure gap, and it comes down before
anything widens the input side.

The rule outlives C3. A cycle that adds a surface kind, an operation or a
body kind carries the work that keeps its own output operable: the new
kind in the intersector, in the checker's face-pair rows and as a boolean
operand. "The operation builds it but nothing downstream takes it" is an
unfinished cycle, the same way an operation without provenance is an
unfinished operation (`.agents/rules/kernel.md`).

### 2. Measured, then chosen

**A measuring harness stands beside the cycles, and the cycle after the
next one is chosen from what it measures.**

The harness is benchmarks over tessellation and the boolean corpus, the
oracle cached by `recipe_hash` so an unchanged recipe is not re-run, a
nightly property tier above CI's case count, random *recipes* run through
both kernels, and fuzz targets seeded from the corpus. It is **not a
cycle**: it changes no public type or signature, so it earns no minor
version (`.agents/rules/git.md` §Tags). It is a plan beside C3's work and
then a standing line in `docs/ROADMAP.md` carrying its current numbers.

**The next cycle is the STEP reader and a real-part corpus**, because it
is the one that turns refusals into a ranking. It is chosen here, not
measured into, precisely because nothing can be measured until it exists.
Its staged scope is in `docs/ROADMAP.md`; what matters for this decision
is its output: a **refusal histogram** — every typed refusal Arris
returned over a public corpus of real parts, counted.

**The cycle after that is picked from two numbers, not from a list.** The
refusal histogram says what real parts hit most; the first consumer's
side-by-side run of its suite against both backends (ADR-0017) says what
a waiting consumer regresses on. Where they disagree: **the consumer's
regressions rank first while there is a consumer waiting on a swap, the
histogram after.** A consumer mid-swap is a concrete blocked user; the
histogram is a population estimate, and it keeps its weight once nobody
is blocked on it.

**A first-party binding for code-first and agent-driven modelling is in
scope and lives in this repository**, beside the cycles rather than as
one, and it starts after C3 closes. `SEED.md` §1 names code-first
modelling tools among the consumers Arris is built for, and a binding is
the only way that consumer exists before an application is written on
top. It waits for C3 because a script that fillets and then cuts meets
the quadric guard, and because a second crate under every C3 API break
pays for the break twice. Everything else about it — the layer position,
the `#![forbid(unsafe_code)]` exception a binding needs, `publish`, the
wasm job, PyPI beside crates.io — is its own idea and its own ADR.

### 3. Unopened cycles carry names, not numbers

A cycle gets its **number** when `/close-cycle` opens its section, and
not before. Until then it is a name. This keeps "cycle Cn releases
`0.n.0`" (`.agents/rules/git.md` §Tags) lining up with the order cycles
are actually opened in, and it makes a reorder free.

The names, and what each older citation meant by its number:

| Cited as | Name | What it holds |
|---|---|---|
| `C4` | **the NURBS cycle** | NURBS–NURBS surface intersection (a marcher with explicit seam handling); NURBS operands in booleans |
| `C5` | **the sweep cycle** | sweep along a path, loft, shell, offset |
| `C6` | **the blend-network cycle** | edge chains, vertex blends, variable radius, blends over blends, blends on quadric face pairs |
| `C7`, the reader half | **the reader cycle** | the STEP reader and the real-part corpus — opened next, so it is the first to take a number |
| `C7`, the bodies half | **the healing cycle** | healing; sheet and wire bodies in every operation |
| `C8` | **the breadth-and-speed cycle** | IGES; same-domain face merging; the performance pass |
| — | **the query cycle** | distance, clash, ray fire, selection |
| — | **the attribute cycle** | attributes a consumer attaches to entities, propagated by declared rules over the split order (ADR-0009) |

`C7` covered two things that now separate: its reader is the next cycle,
its sheet and wire bodies are a named one. An accepted ADR that says
`C7` is read through this table and is never edited.

The named cycles are **unordered**. `docs/ROADMAP.md` lists them under
the selection rule above; none is scheduled, and the order they are
listed in carries no meaning.

## Consequences

- `SEED.md` §6's "roughly in order" is superseded by this ADR for
  everything after C3. The seed's *content* stands — every item on its
  list is on this one — and its ordering does not.
- `docs/ROADMAP.md` gains a "Beside the cycles" section (the harness, the
  binding), the reader cycle in outline, and the named cycles under the
  selection rule, in place of "later cycles, one line each".
- No living doc, backlog line, rustdoc line or fixture description names
  an unopened cycle by number. Accepted ADRs keep theirs and are read
  through the table.
- Three follow-ups, within the two-plan cap and in this order: a plan for
  the measuring harness beside C3's remaining work; an idea for the
  binding once C3 closes; an idea for the reader cycle's scope as C3's
  last plan nears its end.
- The choice of the cycle after the reader is deferred *on purpose*.
  Anyone looking for it in the roadmap finds the rule and the two inputs
  instead of an answer, and that is the decision, not an omission.
- No public type, signature or crate boundary changes.

## Alternatives considered

- **Keep the kickoff order.** It is written, cited and needs no work. It
  also schedules NURBS–NURBS surface intersection — the hardest single
  algorithm on the list — ahead of the closure gap that breaks
  fillet-then-cut today, and ahead of any way to find out which refusal a
  real part meets first. The seed froze an order chosen with no evidence;
  keeping it now would be choosing it a second time, with evidence
  against.
- **Stop C3 at what the first consumer needs.** The probe corpus is green
  (ADR-0017), so the quadric pairs that remain are not blocking that
  consumer. Rejected: the guard refuses bodies Arris's own operations
  return, which blocks every consumer including that one as soon as a
  feature tree fillets before it cuts; a real-part corpus is mostly
  filleted parts, so the reader's refusal histogram would measure the
  guard instead of the reader; and torus–torus in general pose, the one
  rare case, passed its own go/no-go gate and rides the tracer the rest
  of C3 already needs.
- **The reader before C3 closes.** It is the fastest route to real parts.
  Rejected: nearly every real part carries a blend, so the corpus would
  read parts Arris cannot then operate on, and the histogram would be
  dominated by one refusal already scheduled to go away.
- **A binding in a separate repository.** It keeps the kernel workspace
  pure Rust with no `unsafe` exception. Rejected: a binding outside the
  workspace lags every pre-1.0 API break by a release, and a consumer who
  is also the kernel's fastest feedback loop should break in the same CI
  run as the kernel. Whether the exception is granted, and how, is that
  binding's own ADR.
- **Renumber the cycles to the new order.** The numbers are cited in four
  accepted ADRs that may never be edited, and `Cn → 0.n.0` would then be
  wrong for every cycle after the renumber. Names cost one translation
  table, once.

## Amendment (2026-09-26, closing C4)

§2 ranks a consumer first only "while there is a consumer waiting on a
swap". C4 closed with a second consumer on record: a plugin-based CAD
seeded on Arris, whose asks (`docs/ideas/plugin-cad-consumer-asks.md`)
are not regressions of a swap but API it cannot start without. Consumer
roles, body bytes, cancellation and mirror block its first cycle, and
STEP product structure its second. The histogram ranks the blend network
first (17 of 38 parts), and that consumer is blocked on none of it.

**A consumer blocked on missing API ranks first too, as a consumer
waiting on a swap does.** The reason is §2's own: a blocked consumer is
a concrete user, and the histogram is a population estimate. The same
limits apply to both. The ranking covers only what blocks that
consumer's next cycle, not everything it will ever want. The asks it
ranks for later (the query and sweep cycles) are recorded as input and
not applied. And the histogram keeps its weight for the cycle after.

So C5 is the consumer's API (A1–A4 and A11 of that idea), and the blend
network, first by the histogram, stays a named cycle next in line
(§3: it takes its number when it opens).

## Amendment (2026-11-16, re-scoping C12)

C12 opened at C11's close as the plugin CAD's remaining asks (draft,
thicken, `region2`), picked by the amendment above because every kernel
row of the histogram was 0 and no consumer had reported a regression.
Since then a second consumer has: the **training harness**, which runs
generated code-CAD programs through the binding and scores each body
against its target by exact measures. Over its benchmark corpus it filed
eleven refusals against 0.10.0, each a program Open CASCADE builds, each
held on its side as a fixture with a closed form, together reaching about
1,100 of the corpus's programs. Three kinds are worse than a refusal: an
`OpInternalError` ("kernel bug"), and an `Ok` body the checker refuses,
which `.agents/rules/kernel.md` forbids outright. The plugin CAD has
reported nothing, and its asks block none of its own cycles.

**Between consumers, measured refusals of programs a consumer runs today
rank before API a consumer has not called yet.** The reason is §2's: a
refusal of what Open CASCADE builds is a regression of what the kernel
claims, counted over a corpus as the histogram is, while an ask is a
design input. **A cycle opened under one ranking may be re-scoped under a
later one before its first plan starts**; it keeps its number (so
`Cn → 0.n.0` holds), and its section's goal says it was re-scoped and why.

So C12 becomes the training harness's refusals, and the plugin CAD's
remaining asks return to the named cycles, next in line by the amendment
above unless the harness's next refusals outrank them at C12's close.

Rejected: the harness's fixes in the second plan slot beside the original
C12. They are a cycle's worth (six ideas, about four plans), and the
second slot belongs to the nightly's findings.
