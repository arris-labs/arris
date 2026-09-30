# ADR-0025 — The STEP reader: what it converts, what it refuses by name, and what it flattens

- Status: accepted (2026-09-25)
- Plan: `step-reader` step 1
- Follows: ADR-0020 (the reader's output is a refusal histogram that picks
  the cycle after it), ADR-0023 (`step_differs`), ADR-0024 (the oracle
  cache, the fuzz targets, the refusal `Reason` table)

## Context

ADR-0020 opened the reader cycle for its **output**. Its refusal histogram,
beside the first consumer's regressions, picks the cycle after it. The
roadmap's C4 section lists what the reader builds, but it does not say
enough to produce that histogram:

- **What gets counted.** Over parts that are only *read*, "every typed
  refusal Arris returned" counts the reader's own: unsupported entities,
  gaps, open shells. Those point at one named cycle, the healing cycle. The
  NURBS, blend-network and sweep cycles show up only when an *operation*
  runs on a part that was read. If nothing runs on the read parts, the
  histogram cannot rank the cycles it exists to rank.
- **Entities Arris has no variant for.** Real files carry
  `SURFACE_OF_REVOLUTION` and `SURFACE_OF_LINEAR_EXTRUSION` of splines,
  `OFFSET_SURFACE`, `TRIMMED_CURVE`, `COMPOSITE_CURVE`, assemblies with
  placements and units other than millimetres. A new `Surface` variant is a
  breaking change (`.agents/rules/kernel.md` §API).
- **Partial reads.** A file of ten solids, one of which refuses.
- **Provenance.** "`Generated` from the file's entity" names a STEP `#id`,
  which is not an Arris entity.
- **Open CASCADE's own files.** ADR-0023 records that its STEP of
  `ball-offset-drill-cut`, `ball-corner-cut` and `ball-beside-pole-slice-cut`
  does not read back as itself. "Open CASCADE's STEP of every corpus
  fixture read back" would hold Arris to a file that is wrong.
- **Prerequisites.** The oracle cache and the fuzz harness, which ADR-0020
  named as the cycle's first follow-up, exist since ADR-0024.

The idea `reader-cycle-scope` weighed four options (the reader as listed;
the reader plus a probe battery on every part read; that plus healing; the
roadmap section as it stands) and recommended the second. The charter
delegates the five decisions it left, so they are taken here, each argued
once.

## Decision

### 1. The subset, and the exact conversion of everything Arris holds

The reader maps the AP203/214/242 B-Rep subset onto Arris's own variants.
No entity becomes an approximation of itself: it is exactly a variant Arris
has, or it is refused (§2).

- **Kept as themselves:** `PLANE`, `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`,
  `SPHERICAL_SURFACE`, `TOROIDAL_SURFACE`, `LINE`, `CIRCLE`, `ELLIPSE`, and
  the elliptic cylinder that a section of `SURFACE_OF_LINEAR_EXTRUSION`
  gives (below).
- **Every B-spline** subtype, curve and surface (`UNIFORM`, `QUASI_UNIFORM`,
  `BEZIER`, `PIECEWISE_BEZIER`, `_WITH_KNOTS`, and the rational complex
  instances), becomes `Nurbs`, knot multiplicities expanded.
- **`SURFACE_OF_LINEAR_EXTRUSION`.** Of a line it is a `Plane`. Of a
  circle or an ellipse, in any direction, it is a `Cylinder` or an
  `EllipticCylinder` with the section recomputed perpendicular to the
  direction. Of a spline it is a `Nurbs`, exact.
- **`SURFACE_OF_REVOLUTION`.** Of a line it is a `Plane`, a `Cylinder` or a
  `Cone`. Of a circle in a plane through the axis it is a `Sphere` or a
  `Torus`. Anything else is a rational `Nurbs`, exact, in rational
  quadratic arcs of at most 90°.
- **`PARABOLA`, `HYPERBOLA`, `POLYLINE`** become `Nurbs`, exact, over the
  range the edge uses.
- **`TRIMMED_CURVE` and `RECTANGULAR_TRIMMED_SURFACE`** become their basis.
  The edges and faces trim them anyway, and a trim a bound does not agree
  with is a gap (§4), not a second source of truth.
- **Pcurves are never read.** A file's are optional, approximate or absent,
  and the data model wants one on every edge, exact on the analytic
  surfaces. Every pcurve is rebuilt from the 3D curve and the surface
  (`pcurve_on`), which is why `Surface::project` and `pcurve_on` onto
  `Nurbs` are built first (plan steps 3 and 4). A degenerate edge the file
  left out is rebuilt at the surface's singularity.

The exactness is the reason no variant is added: extrusions and
revolutions are representable, so a variant of their own would buy the
reader nothing and cost every exhaustive `match` in the workspace.

### 2. What is refused, by name

Each is a `Refusal` whose `RefusalKind` names the entity type or condition,
so the histogram can count it:

- `OFFSET_SURFACE` and `OFFSET_CURVE_3D`;
- `COMPOSITE_CURVE`, `COMPOSITE_CURVE_ON_SURFACE` and the composite
  surfaces;
- the curve-bounded surface family;
- a degenerate torus, and a torus Arris does not hold (`R ≤ r`,
  data-model §Surfaces);
- an open shell, a `SHELL_BASED_SURFACE_MODEL`, a `GEOMETRIC_SET`: the
  healing cycle's;
- a faceted B-rep (`FACETED_BREP` and its tessellated relatives);
- a Part 21 edition-3 anchor or reference section;
- a length context with no unit;
- a gap past the cap (§4), a loop that does not close outside a singular
  row (`OpenLoop`), and a body the checker rejects (`Invalid`, §5).

Refusing costs one line in the histogram. Approximating an offset as a
fitted `Nurbs` would return a solid that looks right and is not the
consumer's part, which is the "no wrong solid" half of the cycle's accept
line. If offsets rank high, a `Surface` variant is the next cycle's
decision, made on that evidence. Nothing here adds one.

### 3. Partial reads

`step::read` returns `Result<Read, ReadError>`. A parse error, or an
unusable header, fails the file: nothing after it can be trusted to mean
what the grammar says. Past parsing, `Read` holds one result per solid
instance, and each is `Ok(ReadSolid)` or its own `Refusal`. One refused
solid does not hide nine good ones. A file that holds no solid at all is
an empty `Read`, not an error.

### 4. Tolerances come from measured gaps

The per-entity tolerance model (`SEED.md` §9) survives import only if an
entity's tolerance is its own. The file's global
`UNCERTAINTY_MEASURE_WITH_UNIT` is read as the file's *claim* and kept
beside the result. It is never an entity's tolerance.

- A **vertex** takes the largest distance from its point to the end of each
  edge curve that meets it.
- An **edge** takes the largest distance from its curve to each face's
  surface at the checker's samples, and its pcurves' deviation.
- A **face** takes the floor.
- Values are raised to keep vertex ≥ edge ≥ face, floored at the model's
  minimum tolerance.
- The cap is **relative to the part's size**, a named constant in
  `arris-math` whose value is set from measured gaps and commented with the
  evidence. Past the cap the solid is refused as a gap. It is never grown
  into a wrong solid, and never repaired: closing a gap wider than the cap
  is sewing, which is healing.

### 5. Units, instances and the checker

- **Units.** Lengths and angles are read from the file's context
  (`SI_UNIT` with its prefix, `CONVERSION_BASED_UNIT` for inch and foot,
  radians or degrees) and converted to `ReadOptions::length_unit`,
  millimetres by default to match the writer. Precision is chosen for that
  unit, as the metre fixtures already do.
- **Instances.** Arris has no product structure. An assembly is flattened
  to one body per solid *instance*, with the placement path composed and
  baked into the geometry, numbered in the deterministic order of the
  paths. Refusing assemblies would fill the histogram with a trivial
  refusal and hide everything behind it.
  *(Amended by ADR-0033: the bodies stay flattened and baked, and a
  product tree is returned beside them.)*
- **Provenance.** Every entity of a read body is `Generated` from a
  `Role::File(FileEntity { id, instance })`, naming the `#id` and which
  placement of it. `Role` is exhaustive, so this is a breaking change to a
  public type of `arris-topo`, pre-1.0 and named in its commit. An entity
  the file never had is `Generated` from the nearest entity it does: a
  rebuilt degenerate edge from its face, an edge split at a seam from its
  `EDGE_CURVE`.
- **The checker.** The kernel rule runs the checker after every operation
  *in debug builds*, because a violation there is the kernel's fault. A
  violation in a body read from a file is the file's fault, and it must not
  reach a consumer in a release build either. So the reader runs the
  checker at `Level::Fast` in **every** build, and a violation is a
  `Refusal::Invalid` naming the checker's row, never a panic.

### 6. The battery and the histogram (built by the next plan)

Counting only the reader's refusals would rank the healing cycle against
nothing. So the cycle also runs a **fixed, deterministic battery** on every
checker-green part it reads, and counts refusals **per step of the
battery**:

1. tessellate and measure, held to the oracle;
2. write and read back;
3. cut by a box through the centroid, and by a cylinder drilled along each
   principal axis;
4. fillet a deterministic sample of its edges.

Every refusal — the reader's `RefusalKind` and the operations' `Reason`
(ADR-0024 §2) — maps through a declared table to the named cycle it blocks,
so the histogram reads out directly as *cycle → parts blocked*:

| Refusal | Blocks |
|---|---|
| open shell, shell-based model, gap past the cap, `OpenLoop`, sheet or wire body | the healing cycle |
| a boolean or blend on a `Nurbs` face | the NURBS cycle |
| a blend on a face pair outside ADR-0007's table, a blend over a blend | the blend-network cycle |
| an offset, a composite curve or surface, a sweep the operations lack | the sweep cycle |
| a faceted B-rep, an entity outside the subset | counted as itself; no cycle until it ranks |

The table is code with a test that every `RefusalKind` and every `Reason`
has a row, and it is the next plan's (`real-part-corpus`).

The corpus has two tiers: a small committed tier of public-domain parts
(the NIST MBE PMI test models, US-government works) as `source: step`
fixtures run by `cargo test`; and a fetched tier, a pinned, hash-listed
sample of a large public dataset run by a script into a gitignored cache,
its licence confirmed before the plan names it. A regression shrunk from a
fetched part is re-expressed as a recipe, or committed as an excerpt only
if the licence allows.

### 7. How the cycle is cut

C4 is **two plans**: `step-reader` builds the reader and the refusal type
the histogram counts, and `real-part-corpus` builds the corpus, the battery
and the table. The reader is held three ways (Arris's own STEP of random
shapes in random poses; Open CASCADE's STEP of every corpus fixture; the
same converted to NURBS) and by a fuzz target over the parser and the
reader. The accept line's "Open CASCADE's STEP of every corpus fixture
read back" **skips `step_differs` fixtures** (ADR-0023), which name the
files that are wrong.

## Consequences

- No new `Surface` or `Curve` variant. `arris-geom` gains exact NURBS forms
  (conic arcs, extrusion, revolution), `Surface::to_nurbs`, and
  `Surface::project` and `pcurve_on` onto `Nurbs`.
- `arris-topo` gains `Role::File(FileEntity)`, a breaking change. Data-model
  §Provenance gains the rule that a read body is `Generated` from its file
  entities.
- A closed NURBS curve that is not periodic, met by a surface where its
  ends join, is one hit. Data-model §Curves says two today.
- A NURBS face the reader returns is not yet a boolean operand, which is
  the NURBS cycle's; its S5 face pairs stay `unchecked`, as the checker
  already reports. This cycle **measures** that gap and does not close it.
- The reader runs the checker in release builds, at a cost per solid the
  plan measures.
- Backlog gains a line per refused entity family until the histogram ranks
  them.

## Alternatives considered

- **The reader as listed, with a read-only histogram** (idea's option A).
  About six steps cheaper. Its histogram measures one cycle, and the cycle
  after this one would again be chosen by guess, which ADR-0020 set out to
  end.
- **The reader, the battery, and healing** (option C). It takes over the
  healing cycle before the histogram has said healing comes next: the
  "chosen, not measured" mistake again.
- **A `Surface` variant for offsets, extrusions and revolutions.** A
  breaking change to every exhaustive `match`, for entities that are
  representable exactly (extrusions, revolutions) or that no operation
  could then use (offsets). Deferred to the evidence.
- **Approximating what has no variant** (an offset as a fitted `Nurbs`).
  Returns a solid that passes the checker and is not the part. Refused.
- **Refusing assemblies**, or reading only the first solid. Fills the
  histogram with a trivial refusal, or hides the rest of the file.
- **Failing the whole file on the first refused solid.** One unsupported
  entity would hide every solid that reads.
- **The file's uncertainty as the tolerance.** It is one global value for a
  model the kernel measures per entity; and vendors write it from a
  default, not from a measure.
- **The checker in debug builds only, as the kernel's other operations.**
  A file's fault would return an invalid `Ok` body in release.
- **Reading the file's pcurves.** They are optional, approximate or absent,
  so a second path would be needed anyway, and the two would disagree.

## Amendment (2026-09-26, plan `step-reader` step 12)

- **The floor is `default_tolerance`, not `min_tolerance`.** §4 said the
  values are "floored at the model's minimum tolerance". Every algorithm
  of the kernel works at `default_tolerance` — the pcurves the reader
  rebuilds are fitted to it, a boolean's sections and a checker's S5
  band are sized by the faces' tolerances — so an entity read with a
  measured gap of `1e-15` would claim more than any fit on it delivers,
  and a later operation would be asked to hold it. The default is what a
  primitive gets, Open CASCADE's `Precision::Confusion`, below which the
  oracle merges points too. A face takes the default; an edge and a
  vertex take their gap where it is above it.
- **A gap past the cap found by the loop's walk is a gap, not an open
  loop.** Two uses whose ends are farther apart in 3D than the cap are
  `Refusal::Gap` naming the vertex; `OpenLoop` is kept for uses that meet
  in 3D but not on the face, off a singular row. A seam line moved off
  its vertices is the first, where step 11 reported the second.
- **Pcurves ending apart are ended on one point.** Two exact pcurves of
  curves ending a tolerance apart meet in (u, v) no closer than the
  curves do, which L2 refuses past its band. The reader ends them where a
  boolean ends a section edge: on the seam's end when one of the two is a
  seam (whose uses must stay a period apart), else on the vertex's own
  (u, v). The move is measured into the edge's tolerance. The boolean's
  helper moved to `arris-geom` as `pcurve_ending_on` so both use one.
- **The cap's constant** is `arris_math::READ_GAP_FRACTION`, `1e-4` of the
  diameter of the ball holding the part's vertices and edges, at most
  `max_tolerance`; its comment carries the evidence.

## Amendment (2026-09-26, plan `step-reader` steps 14 and 15)

- **The cap is `2e-4` of the part's size.** Open CASCADE's own STEP of
  the corpus, plain and converted to B-splines, leaves gaps of at most
  `2.5e-6` of the part but for one fixture whose boolean it gets wrong,
  where its converted file measures `1.13e-4` — exactly the tolerance its
  own shape carries there and its checker passes. A file its exporter
  calls valid by its own tolerances is read; `2e-4` admits it with `1.8×`
  to spare. `READ_GAP_FRACTION`'s comment carries the numbers.
- **A B-spline's padding knot is clamped.** Open CASCADE writes a
  periodic B-spline unclamped — `(1, 2, …, 2, 1)` of degree two round a
  closed circle — whose end knots lie outside the domain and enter no
  basis function on it. The reader moves them onto the domain's end, an
  exact change, so a closed direction is recognised as closed.
- **A file that describes no solid is refused, not healed.** Open
  CASCADE's writer keeps the bounds' sense of a face reversed on a
  left-handed torus frame while writing the frame right-handed with
  `same_sense` turned, so the face walks an edge the way its neighbour
  does; its own cut can leave a face whose walk cannot close. Both are
  refusals — `Topology` and `OpenLoop` — and a fixture that meets one
  says so (`analytic.occt_step_refused`). Turning a face's loops to agree
  with its neighbours is the healing cycle's.
- **The measures on a NURBS face split at its knots.** The face integral
  split its inner quadrature on a grid of the smallest knot span anchored
  at zero, which put a converted quadric's knots, where its derivatives
  jump, inside an interval: `1e-5` of a volume. `integrate::Grid` splits
  at the knots themselves, in `u` inside and in both parameters along the
  boundary.

