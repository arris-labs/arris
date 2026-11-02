# Cusp corner measurements

Observed 2026-11-02, Open CASCADE 8.0.1.0.0. The evidence ADR-0084 decides
the corner law from: the cut-back on one ball, the patch boundaries, the
chamfer's section, the patch as a height graph, and the oracle's own
bound on its measures.

Reproduce the 18 observations with:

```sh
uv run --project tools/oracle tools/oracle/cusp_corner.py
```

The record is `tests/fixtures/regression/cusp-corner-crescent-fillet/measurements.json`.
The laws are held to the oracle by

```sh
uv run --project tools/oracle tools/oracle/cusp_corner_law.py
```

whose record is `law.json` beside it: per run, the patch's corners and
boundaries against ADR-0084's, its tolerance and angles to each
neighbour, its normal's `z` range (one sign: a graph), the oracle's own
bound on the measures, the rejected square fill's measures and folds, the
miter's measures, and each chamfer's section edge.
ADR-0084 §5's interior is held to the oracle's own bound by

```sh
uv run --project tools/oracle tools/oracle/cusp_corner_graph.py
```

whose record is `graph.json` beside it: per fillet run, the oracle
patch's measures and bound, the least-bending height graph solved in
plain numpy at two resolutions, and a G1 Hermite fill over a ruled chart
of the domain that does not fold, each one's volume and area less the
oracle's in units of that bound. The graph converges 1.9 to 4.5 bounds
below on volume and 0.86 to 2.0 on area, 0.14 to 0.31 r under the
oracle's interior on average; the Hermite fill lies 0.65 to 5.9 and 2.7 to
4.7 bounds above. Neither is inside: the plan's open question.

A second oracle's runs of the three fillets at the three sizes (ADR-0073;
each recipe scaled ×100, both edges in one feature, accuracy 1.0, its
operand measured by the same kernel to 5·10⁻¹⁶ of Open CASCADE's) are
`second_oracle_runs.json`. Open CASCADE measuring each exported result
agrees with the run's own measure within its export tolerance. They are
held to the candidates by

```sh
uv run --project tools/oracle tools/oracle/cusp_corner_second.py
```

whose record is `second.json` beside it. The second oracle builds all
nine, and builds **no corner patch**: 7 faces and 14 edges (the pocket 12
and 26), against Open CASCADE's 8 and 17 (13 and 29). Its two stripes run
past the ball at `Q` to the cusp and meet on their own intersection, from
`T*` down to the spine at `h − σr`; the cusp vertex is gone and the spine
is cut there, as ADR-0084 §3's chamfer and its rejected miter. Each
kernel's change against the second oracle's:

- the mutual trim (each edge filleted alone by Open CASCADE, the results
  combined by `common`, or `fuse` on the pocket) agrees to 1.3·10⁻⁹ of the
  body at the acceptance size, 2.9·10⁻⁶ at worst, 0.24 bounds at worst;
- Open CASCADE's patch is 0.03–0.55 bounds off on volume but 2.6–4.2 on
  area;
- ADR-0084 §5's graph is 1.9–5.1 bounds off on volume and 1.5–3.2 on area.

The second oracle sides with neither the graph nor Open CASCADE's patch.
The human decided for the mutual trim (ADR-0084 as amended). This
fixture's `second_oracle.json` is the run at 0.1, imported by
`tools/oracle/second_oracle.py` (the run's record multiplied back to its
scale of 100, which the tool divides out); its `analytic` holds Arris to
it and to the record's 9 / 14 / 7. The recipe is the acceptance
fixture's, `blend/cusp-crescent-both-arcs-fillet`, which takes both when
it builds.

Six desired-result fixtures under `regression/cusp-corner-*` cover crescent,
pocket and spandrel, each filleted and chamfered. Their `expected.json`
files come from `expected.py`, not Arris. At sizes 0.02, 0.1 and 0.3 all
18 oracle bodies are valid and neither selected edge is dropped.

Fillets add one B-spline face; chamfers add none. The crescent fillet's
patch is degree 8 by 8. At radius 0.1 its boundary has four curves:

- A stripe cross-section from (-1.7894736842, -0.8931875131, 0.9)
  to the common top contact (-1.7, -0.8485281374, 1).
- The other cross-section from that contact to
  (-1.6363636364, -0.7713892158, 0.9).
- One curve on each cylinder from those lower contacts back to the
  original cusp (-2, 0, 1). The original cusp survives.

The top contact is the intersection of the two offset contact circles:
centres (0,0), (-1,0), radii 2-r and 1+r. The stripe cross-sections there
supply the two lower contacts. The wall curves are straight in each
wall's (u, v), within 7.2e-5 on all three families (`law.json`).
Sampling a B-spline's entire rectangular carrier is insufficient: much
lies outside the trimmed face.

Reference trees read: `ChFi3d_Builder_CnCrn` and `ChFi3d_FilBuilder_C2`.
The former contains a third-order plate solve followed by degree-at-most-8
approximation; boundary orders vary between G0 and G1. No implementation
or identifiers were copied. ADR-0084 settles Arris's own interior law (the
least-bending height graph); a square Coons fill was measured and rejected
for folding at the cusp corner.

The oracle STEP cannot yet serve as an Arris-rendered inspection:

```sh
uv run --project tools/oracle tools/oracle/occt_step.py \
  tests/fixtures/regression/cusp-corner-crescent-fillet target/inspect/cusp-corner.step
cargo run -p arris-debug --example inspect_step -- \
  target/inspect/cusp-corner.step cusp-corner
```

The reader reports #608's gap 0.005603629779155641 past the cap
0.00229128784747792. This is a read observation, not evidence by itself
that the oracle's body or its mass properties are wrong. Diagnose the
boundary/pcurve discrepancy before choosing an oracle-fit allowance;
no fixture tolerance or acceptance condition has been changed.
