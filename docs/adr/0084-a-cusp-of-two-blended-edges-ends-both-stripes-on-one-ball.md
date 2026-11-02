# ADR-0084 — A cusp of two blended edges ends both stripes on one ball: a fillet fills the rest with a height graph, a chamfer meets on its section

- Status: accepted (2026-11-02); §2's stripe ends and §4–§7 superseded the
  same day by the amendment below (the mutual trim, the human's decision)
- Plan: `cusp-corner-patch` step 1 (the law, against the oracle); step 2
  records the fit budget below, steps 3–5 build it
- Amends: ADR-0042 §6 and ADR-0079 (a cusp with both edges blended is no
  longer refused at the site of §1)
- Reference trees read: Open CASCADE's `ChFi3d_Builder_CnCrn` and
  `ChFi3d_FilBuilder_C2`, for the structure of its corner (a plate solve
  then an approximation); nothing taken. Open CASCADE run as the oracle.

## Context

ADR-0042 cuts a stripe that ends at a cusp with the next wall, and refuses
a cusp whose two edges `e` and `n` are both blended: "that takes a corner
patch". Six recipes ask for it (`regression/cusp-corner-*`): the crescent
prism, its pocket and a line–arc spandrel, each filleted and chamfered.
Open CASCADE builds all six at 0.02, 0.1 and 0.3, valid, no edge dropped.
`tools/oracle/cusp_corner.py` records the boundaries
(`measurements.json`) and `tools/oracle/cusp_corner_law.py` holds them to
the laws below (`law.json`, beside it in `cusp-corner-crescent-fillet`).

What the record shows:

- **One ball ends both stripes.** The walls' offsets by `r` on the
  blend's side meet, in the shared face `T`'s plane, at `Q`. The ball at
  `(Q, h − σr)` (`h` the plane's height, `σ` +1 where the blend lies under
  `T`, −1 over it) touches `T` at `T* = (Q, h)` and wall `i` at `A_i`. The
  fillet patch's four corners are `A_1`, `T*`, `A_2` and the cusp vertex
  `v` itself, within 5.4·10⁻¹³ in all nine fillet runs; its two edges on
  the stripes lie on that ball within 3.6·10⁻¹⁶.
- **The wall curves are straight in the wall's (u, v)**, from `A_i` to
  `v`: a helix on a cylinder, a line on a plane. The oracle's lie within
  7.2·10⁻⁵ of them; its edge tolerance on the patch is 1.1·10⁻³ to
  2.1·10⁻² (5.5 % to 7 % of `r`).
- **Tangency is not met.** The oracle's patch leaves the stripes at up to
  20.4° and crosses the walls at 70.9° to 151.5°. Tangency to both walls is
  impossible at the cusp, where they are tangent with opposite normals.
- **The patch is a height graph over `T`'s plane.** Its outward normal's
  `z` keeps one sign at every interior sample, `|n_z|` from 0.11 to 1.0.
- **A chamfer adds no patch.** The two chamfer faces meet on their exact
  section (an ellipse on the crescent and pocket, a parabola on the
  spandrel, edge tolerance ≤ 2·10⁻⁷) from `(Q_d, h)` to `(cusp, h − σd)`;
  the spine is cut there and `v` is gone: 9 / 14 / 7 (the pocket 17 / 26
  / 12) against the fillet's 11 / 17 / 8 (19 / 29 / 13).
- **The oracle's own tolerance bounds its measures.** Over the patch, area
  times the patch's tolerance is 2.5·10⁻⁷ to 3.9·10⁻³ of the body's volume,
  boundary length times it 1.2·10⁻⁵ to 5.7·10⁻³ of its area; at the
  acceptance fixture (crescent, 0.1) 1.1·10⁻⁴ and 6.4·10⁻⁴. There its
  faces' cone volumes from the origin sum to 2.5·10⁻⁵ more than its
  solid's volume: the boundary is not closed at the default tolerances.

## Decision

**1. The site.** ADR-0042 §1's vertex `v` of three edges `e`, `n`, `w`, with
both `e` and `n` blended, of one sense, by one fillet radius or one
symmetric chamfer distance; `T` planar; each wall a plane or a cylinder
whose rulings are normal to `T`. Anything else at such a vertex keeps its
refusal: mixed senses (ADR-0079), unequal sizes, another shared face or
wall kind are `TangentChain` naming `e`, `n` and `v`, as ADR-0042 §6.

**2. The cut-back is the ball at `Q`.** `Q` is the meeting of the two
offset traces nearest `v`, between `v` and both edges' far ends; none
there is `BlendTooLarge` naming `e` and `n`. Both stripes end on that
ball's cross-sections: the arc `A_i → T*` for a fillet, the chord for a
chamfer. `T*` is a vertex of both stripes and of `T`.

**3. A chamfer meets on its section.** No patch. The two chamfer faces are
cut by each other along their exact section from `T*` (at `Q_d`) to
`(cusp, h − σd)`, the walls' contacts at `h − σd` meet there, and `w` is
cut at that point. `v` is deleted.

_§4–§7 are superseded by the amendment below._

**4. A fillet ends on a corner patch** (`GLOSSARY.md`), bounded by the two
arcs, and on each wall the curve straight in its (u, v) from `A_i` to `v`.
`v` and `w` survive unchanged. The patch is **G1 to the ball** along the
open arcs and **G0 on the walls**.

**5. The patch is the least-bending height graph.** Over `T`'s plane it is
`z = f(x, y)` on `D`, the region its boundary projects to: two wall traces
meeting tangent at the cusp and two segments from `A_i` to `Q`. `f` takes
the boundary's heights, has zero slope across the two segments (the
ball's gradient is radial from `Q`), and among such functions minimises
`∫_D (f_xx² + 2 f_xy² + f_yy²)`. The problem is convex: one `f`, the same
on every run, no fold, and the cusp a point of `D`'s boundary like any
other, not a corner of a chart. It is carried by a B-spline graph over a
rectangle of `T`'s plane covering `D`, trimmed by the four boundary
curves. Its agreement with the oracle is step 2's to measure: within the
oracle's own bound above, and at refinement converging.

**6. Fixtures.** The `cusp-corner-*` fillets are held to the oracle at the
oracle's own bound of the patch (§Context's area-times-tolerance rule, from
`law.json`), never wider; a chamfer at the default tolerances, its
geometry being exact. Counts are the oracle's.

**7. Provenance.** The patch, its four edges, `A_1`, `T*` and `A_2` are
`Generated` from both `e` and `n`; for a chamfer the section edge and its
lower vertex. Each stripe's other entities keep its edge's origin alone.
`e` and `n` are `Deleted`; for a chamfer `v` too, and `w` is `Modified`.
The walls and `T` are `Modified` once. In `generated_from` each edge lists
its stripe's own records first, then the shared ones: the patch, the arc
on `e`'s stripe, the arc on `n`'s, the wall curve on `W_e`, on `W_n`,
then `A_e`, `T*`, `A_n`. `generated_pair(e, n)` is exactly the shared ones,
in that order.

## Consequences

- `blend/cusp-crescent-both-arcs-fillet` and the six `regression/cusp-
  corner-*` fixtures become buildable: step 3 the crescent fillet, step 4
  the pocket and spandrel fillets, step 5 the chamfers.
- The patch is the first blend face that is a trimmed graph over a plane.
  Step 2 records its fit budget here (the surface's, the boundary curves'
  and the pcurves' shares of the entity tolerance), with the size domain
  where `D` stays a graph's domain.
- `Reason` gains no variant; the binding does not change.

## Amendments

- **The mutual trim (2026-11-02, plans/cusp-corner-patch step 2; the
  human's decision).** §5's graph was held to Open CASCADE's patch and
  missed its own bound in all nine runs: 1.9–4.5 bounds below on volume
  and 0.86–2.0 on area (`graph.json`). A second oracle's runs of the three
  fillets at the three sizes (ADR-0073; `second_oracle_runs.json`, held to
  every candidate by `tools/oracle/cusp_corner_second.py` in `second.json`)
  build all nine and **no corner patch**: the two stripes run on past the
  ball at `Q` and meet on their own intersection, from `T*` down to the
  spine at `h − σr`; `v` is gone and `w` is cut there, 9 / 14 / 7 (the
  pocket 12 faces and 26 edges) against Open CASCADE's 11 / 17 / 8 (13 and
  29). Each kernel's change `result − operand` against the second
  oracle's, in units of Open CASCADE's own bound on its patch:
  - the mutual trim (each edge filleted alone by Open CASCADE, an exact
    rolling-ball stripe cut by the next wall, ADR-0042; the two results
    combined by `common`, or `fuse` on the pocket) agrees to 1.3·10⁻⁹ of
    the body at the acceptance size, 2.9·10⁻⁶ at worst, 0.24 bounds at worst;
  - Open CASCADE's patch is 0.03–0.55 bounds off on volume, 2.6–4.2 on area;
  - the graph is 1.9–5.1 bounds off on volume and 1.5–3.2 on area.

  So Open CASCADE's patch is its own construction, not the shape: it misses
  its neighbours by 5.5–7 % of `r` and its boundary does not close
  (§Context), and the second kernel's result is the exact mutual trim,
  which ADR-0073 §4 asks for as local evidence. The rejected miter below is
  that shape; its "outside the oracle's bound" was a bound on Open
  CASCADE's patch. The human chose the mutual trim over keeping §5 or
  matching Open CASCADE's counts. In place of §2's stripe ends and §4–§7:

  **2′. `Q` locates the section, not the stripes' ends.** `Q`, `T* =
  (Q, h)` and `BlendTooLarge` stand as §2 has them. Neither stripe ends on
  the ball's cross-section: the arcs and chords `A_i → T*` are no edges,
  and `A_i` no vertices.

  **4′. A fillet is the stripes' mutual trim.** Each stripe is its edge's
  rolling-ball blend (ADR-0007's table), run on toward the cusp; each is
  cut by the other along their section, from `T*` to `S = (cusp, h − σr)`
  where the two contacts on the walls meet the spine. Both trim points are
  closed forms, as ADR-0042 §2's are, with the other stripe in place of
  `W_n`. The section is the intersector's closed form where one exists,
  else traced and fitted between the trim points under ADR-0042 §3 (two
  tori on parallel axes on the crescent and pocket, a torus against a
  ruling stripe's cylinder on the spandrel). The two stripes are tangent
  at both trim points (at `T*` both touch `T`; at `S` both touch walls
  tangent along `w`), and ADR-0042 §4 already admits a singular point at a
  trim point. No face is added and no fitted surface: each stripe keeps its
  exact surface. `w` is cut at `S`; `v` is deleted. §3's chamfer is this
  law with its faces' exact section: one construction for both.

  **5′. Nothing is fitted.** §5's graph is rejected (above); the patch's
  fit budget and graph domain lapse. The safe size domain is where `Q`
  lies between `v` and both far ends and the section's branch runs from
  `T*` to `S` inside both stripes' bands; plan step 7 derives it.

  **6′. Fixtures.** A fillet is held to the second oracle's record
  (`second_oracle.json`, ADR-0073) with `analytic.measure_differs`, and to
  its counts with `analytic.counts_differ`; Open CASCADE's patch stays in
  `expected.json` as the record. The first is the crescent's at 0.1
  (`regression/cusp-corner-crescent-fillet`, its recipe the acceptance
  fixture's). A chamfer is held to Open CASCADE at the default tolerances,
  §3 unchanged.

  **7′. Provenance.** Each stripe's face, contacts and far end are
  `Generated` from its own edge. The section edge, `T*` and `S` are
  `Generated` from both `e` and `n`, in that order after each edge's own
  records, and `generated_pair(e, n)` is exactly those three. `e`, `n` and
  `v` are `Deleted`; `w` is `Modified` (cut at `S`), and the walls and `T`
  are `Modified` once. The same for a chamfer.

## Alternatives considered

- **§5's least-bending height graph** (the decision until the amendment):
  convex and fold-free, but 1.9–5.1 of Open CASCADE's bounds off the
  second oracle on volume and 1.5–3.2 on area in all nine runs, and off
  Open CASCADE's patch too. A fitted face nobody's kernel builds.

- **A square fill on the corners `v`, `A_1`, `T*`, `A_2`**: the bilinear
  Coons patch, corrected G1 to the ball along the arcs. Its volume and area
  stay inside the oracle's bound in all nine runs, so the measures do not
  rule it out; its regularity does. `v` is a corner of zero angle, the
  square's two directions meet parallel there, and the fill folds near it:
  neighbouring normals reach −0.998 on the crescent and pocket at every
  size. A one-sided Hermite variant folded on the crescent at 0.02 as well.
- **The miter**: each stripe run on until it meets the other, the chamfer's
  law for a fillet. Exact, with no fitted face. But it changes the counts
  (seven faces, not eight) and its area differs from the oracle's beyond
  the oracle's own bound in all nine runs (1.7·10⁻⁴ to 1.5·10⁻² of the
  area, against 1.2·10⁻⁵ to 5.7·10⁻³), so it is a different shape, not a
  representation of the same one. A second oracle's record of the crescent
  (ADR-0073) could still argue for it; none exists yet.
- **Matching the oracle's surface**: its patch is an approximation that
  misses its own neighbours by up to 7 % of `r`. Fitting to it would carry
  that error in. Its boundary is taken; its interior is the target only
  within its own bound.
