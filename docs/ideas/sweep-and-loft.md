# Idea: sweep-and-loft

- Status: Accepted 2026-10-15 (option A, every preferred answer). The
  sweep half is absorbed by `docs/plans/sweep.md`; the loft half waits
  for its own plan, which deletes this file.
- Raised: 2026-10-15
- Prompt (verbatim from the human): "sweep and loft"

## Problem

The plugin CAD's largest open ask is A7's sweep along a path and loft
(`docs/ideas/plugin-cad-consumer-asks.md`). ADR-0047 gave both to the
NURBS cycle, and C10's roadmap section names them as its next item now that
a NURBS face is a boolean and blend operand against every analytic kind
(ADR-0061, ADR-0062). The DIY audience reaches for sweep to make pipes,
bent handles, rails, springs and threads, and for loft to make a transition
duct, a nozzle, or a grip from a circle to a rounded square. Today the only
sweeps are `extrude` and `revolve` (`crates/arris-ops/src/sweep.rs`).
`extrude` also refuses an oblique direction (`SweepReason::DirectionNotNormal`,
backlog), and the roadmap files that refusal here.

What exists already: `fit_skin` (`crates/arris-geom/src/nurbs/skin.rs`)
skins a surface through compatible sections. Its doc names loft as its next
consumer. There are exact NURBS circles, ellipses, extrusions and
revolutions (`nurbs/exact.rs`), and `Builder::assemble` already takes a
sweep's faces in a fixed order. What does not exist: a 3D path value, a
frame law, a section-matching rule, provenance roles for either operation,
and an oracle script for `BRepOffsetAPI_MakePipe` and `ThruSections`
(`tools/oracle/` has neither).

## Constraints it runs into

- **ADR-0020 §1, closure.** A swept or lofted body whose adjacent side
  faces are both NURBS has edges that a blend can only walk as a
  NURBS × NURBS offset section, and two such bodies meet in a boolean as
  NURBS × NURBS. `intersect.rs` refuses that pair as `Unsupported` until
  C10's third item, the marcher, lands. The roadmap's wording accepts this:
  "an operand of every boolean and blend that the first item allows". The
  idea should still keep the refused set as small as the geometry allows.
- **Exhaustive enums (kernel rules).** New `Role` variants (or a widened
  `SweepPart`) and new `SweepReason` variants are breaking changes. They go
  under `### Breaking` and need the Python binding updated in the same
  commit. C10 releases a minor version anyway.
- **ADR-0013, layers.** The path value belongs in `geom` beside `Profile`,
  and the operation belongs in `ops`.
- **Tolerances.** A fitted face holds the model's tolerance under ADR-0062's
  budget, with no literals.
- **Oracle.** The roadmap's acceptance is "match `MakePipe` and
  `ThruSections`". The volume of a profile swept perpendicular to its path
  does not depend on twist (Pappus), but the area does. So the frame law
  has to be one that Open CASCADE can also be asked for, or the area cannot
  be held to the oracle.

## Options

### A — Exact where a kind holds it, fitted NURBS elsewhere (recommended)

**Sweep** takes a `Profile` placed by a frame, plus a `geom::Path`: a G1
chain of 3D lines, arcs, helices and NURBS curves. A corner in the path is
refused by name; the consumer rounds it in the sketch. There are two frame
laws:
- **rotation-minimising** (double reflection), the default. On a planar
  path it agrees with Open CASCADE's `CorrectedFrenet`.
- **fixed binormal** (a direction). This is what a thread or spring needs,
  so the profile stays in the helix axis's plane. It matches OCC's
  binormal mode.

Each segment then produces its own faces:
- **Line segment:** an extrude. An oblique profile gives the exact
  `EllipticCylinder` for an arc, which retires the backlog line.
- **Arc segment, rotation-minimising law:** a revolve about the arc's
  axis, giving planes, cylinders, cones and tori.
- **Helix or NURBS segment:** `fit_skin` through the profile carried by
  the frame, one fitted face per profile segment, held to ADR-0062's budget.

As a result, a pipe or handle bent from lines and arcs is all-analytic. It
is an operand of every quadric boolean and blend today, and only springs,
threads and spline paths carry NURBS faces. A local self-intersection, where
the profile reaches past the path's radius of curvature, is refused by name.

**Loft** takes two or more placed `Profile`s with matching loop counts. It
can be ruled or smooth, and it can be closed. A section with fewer segments
is split by arc length to match the others, as `ThruSections` does. The
profile's start point fixes the correspondence between sections. Ruled
spans between coplanar edges are planes, coaxial circles give cones and
cylinders, and everything else is skinned by `fit_skin`.

Cost: sweep about 9 plan steps (path value, frame laws, exact segments,
fitted segments, self-intersection refusal, roles and binding, oracle
script, corpus and properties). Loft about 6 steps. That is two plans run
in sequence, sweep first. It forecloses nothing. The NURBS × NURBS refusals
lift on their own when the marcher lands.

### B — Everything fitted

One code path: every side face is `fit_skin` of the carried profile, even
along a line or an arc. This is simpler, about 3 steps fewer, but a bent
pipe becomes adjacent NURBS faces. A fillet on its rails, or a second pipe
fused to it, is then refused until the marcher, and the faces are heavier
to tessellate and intersect. It widens the ADR-0020 gap exactly where the
DIY parts live.

### C — Marcher first, then sweep and loft

Reorder C10 so NURBS × NURBS lands first, and every swept body is closed on
arrival. This is correct by ADR-0020's letter, but the plugin CAD waits a
whole item longer for its largest ask. The marcher is the cycle's hardest
item, and nothing it builds ships to the consumer on its own.

### Do nothing

A7 stays open. The DIY corpus, built in this cycle, records "no sweep" and
"no loft" as refusals in parts that would otherwise model. `fit_skin` keeps
one consumer.

## Recommendation

**A**, sweep plan first and then loft. The exact segments are a small cost,
and they keep the common DIY sweep (lines and arcs) inside what C3 and C6
already close. The fitted path is reused machinery: `fit_skin` and
ADR-0062's budget. B loses on closure where it matters most, and C loses on
the consumer ranking ADR-0020's amendment set.

What would change my mind: the plugin CAD saying its users sweep mostly
along splines, which makes A's exact path rarely used, so B's simplicity
wins. Or the self-intersection refusal turning out to need the marcher
anyway.

## Decision for the human

1. Sweep along a G1 path of lines, arcs, helices and NURBS curves, exact
   per segment where a kind holds it? *Preferred: yes (A).*
2. Frame laws: rotation-minimising by default, plus a fixed binormal for
   helices, and nothing else for now (no Frenet, no guide rail)? *Preferred:
   yes.*
3. A path corner refused, rather than mitered or rounded by the kernel?
   *Preferred: refused. The consumer rounds the sketch.*
4. Fold the oblique extrude into the sweep (a line path with a tilted
   profile) and retire its backlog line? *Preferred: yes.*
5. Two plans in sequence, sweep then loft, both in the main slot, leaving
   the second slot for draft and thicken? *Preferred: yes.*
6. ADRs: one for the sweep (path value, frame laws, exact-segment table,
   roles) and one for the loft (section matching, ruled versus smooth).
   Each is written by its plan.
