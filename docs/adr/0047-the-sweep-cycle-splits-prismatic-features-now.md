# ADR-0047 — The sweep cycle splits: shell and offset are the prismatic-features cycle now, sweep and loft go with NURBS

- Status: accepted (2026-10-05)
- Plan: none — taken at `/close-cycle` of C6, by the human
- Amends: `docs/ROADMAP.md` §Named cycles ("the sweep cycle — sweep along
  a path, loft, shell, offset"); applies ADR-0020's amendment of
  2026-09-26 (a consumer blocked on missing API ranks first)

## Context

`docs/ROADMAP.md` names one *sweep cycle* holding four operations: sweep
along a path, loft, shell and offset. They were grouped as "the operations
that move or grow a surface", which is how a kernel's table of contents
groups them. They do not share the machinery that decides their cost.

- **Shell and offset on analytic faces stay analytic.** The offset of a
  plane is a plane, of a cylinder or a cone a coaxial cylinder or cone, of
  a sphere a concentric sphere, of a torus a torus of the same major
  radius — every kind Arris builds today, including the tori, spheres and
  cones a fillet or a revolve returns. A shell is the offset faces met by
  the intersector the booleans already use (every quadric pair, C3) and
  closed against the removed faces' rims. No new surface kind, no new
  intersector; what is new is the topology (which faces offset, which
  open, how a vertex of three offset faces closes) and the refusals where
  an offset collapses (a radius driven through zero, a face that vanishes).
- **Sweep along a path and loft need free-form surfaces.** A profile swept
  along a curve that is not a line or a circle, or lofted between sections
  that are not coaxial circles, has no exact analytic kind: its surface is
  a fitted NURBS. Such a body is an operand of the next boolean only once
  NURBS faces are boolean operands, which is the NURBS cycle's (the
  marcher). ADR-0020 §1 (closure before breadth) forbids building a body
  the kernel then refuses as an operand, so a sweep cycle ahead of the
  NURBS cycle would either break that rule or carry the NURBS cycle inside
  it.

The ranking. At C6's close (2026-10-05) the real-part histogram's
`fillet` column holds 2 of the fetched tier's 27 parts (`ctc_01`'s crossing
cylinders and `stc_07`'s NURBS surface, both the NURBS cycle's) and 5 of
the committed tier's 11; healing still blocks 14 of 38 at `read`, the
NURBS cycle 3 at `box_cut`, the sweep row 0. By the histogram alone healing
is first. But the first consumer, the plugin-based CAD, has no users
because it cannot yet model an ordinary prismatic part: its audience is
the hobbyist mechanical-part user, and the operations that audience reaches
for after extrude, revolve, boolean and fillet are shell, offset face
(press-pull), a pattern cut in one go, and split. Those are its recorded
asks A7 (shell, offset) and A8 (the multi-tool boolean) in
`docs/ideas/plugin-cad-consumer-asks.md`, beside A9 (per-face
tessellation) that its viewer caches meshes by. ADR-0020's amendment of
2026-09-26 ranks a consumer blocked on missing API first, as one waiting on
a swap is.

## Decision

1. **The sweep cycle is split.** Shell and offset leave it and join the
   multi-tool boolean and split by a plane as the **prismatic-features
   cycle**, opened now (C7). Its operands and results are the analytic
   kinds `Surface` has; an offset or shell that would need a surface with
   no exact kind (an offset of a NURBS face, a variable thickness) is
   refused by name and is the NURBS cycle's.
2. **Sweep along a path and loft join the NURBS cycle**, which becomes
   "NURBS–NURBS intersection, NURBS operands in booleans, and the
   operations that build free-form faces: sweep along a path, loft". The
   name *sweep cycle* is retired; nothing else is renamed. Oblique
   extrusion and the elliptic revolve, which the backlog filed with the
   sweep cycle, go with it to the NURBS cycle where they need a free-form
   meridian, and are prismatic only where an exact kind holds them
   (`Surface::EllipticCylinder` for an oblique extrusion of an arc, a
   backlog line either way).
3. **Per-face tessellation (A9) is a side plan beside the cycle**, not one
   of its lines: it changes how `arris-mesh` discretises an edge, not what
   a model can hold, and it can land in any order with the cycle's plans.
4. **The histogram's attribution follows** when a plan next touches
   `arris_debug::histogram`: its `Cycle::Sweep` row splits into the
   prismatic-features cycle (a STEP offset surface or curve on an analytic
   basis) and the NURBS cycle (composite curves and surfaces, the oblique
   and elliptic sweeps). Until then the row reads 0 and its doc comment
   names this ADR.

## Consequences

- C7 adds operations, not surface kinds, so it is the first cycle since C2
  whose `Surface` and `Curve` enums need not change. It still breaks the
  API (new `Reason` variants for the collapsing offsets, new operation
  signatures), so it releases a minor as every cycle does.
- The closure rule binds the new operations: a shelled or offset body is a
  legal operand of every operation, blends and booleans included, and the
  cycle's acceptance holds it to that.
- The NURBS cycle grows. When it is ranked it carries sweep and loft, and
  the histogram's sweep row moves into its count.
- Healing stays first by the histogram and is not scheduled by this ADR;
  it is ranked again when C7 closes, by the same two numbers.

## Alternatives considered

- **Keep one sweep cycle and open it now.** Shell and offset would wait on
  the sweep and loft work, which waits on the marcher, or sweep and loft
  would ship bodies no boolean takes, against ADR-0020 §1.
- **Open the healing cycle, first by the histogram.** It lets more real
  parts read, but the consumer that is blocked is blocked on modelling, not
  on reading, and ADR-0020's amendment ranks it first.
- **Take shell and offset as plans beside C6's residue, with no cycle.**
  They change public signatures and add refusals, which is a minor version
  and a cycle's acceptance corpus, not upkeep (ADR-0020 §2).
