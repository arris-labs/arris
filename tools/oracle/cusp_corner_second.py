#!/usr/bin/env python3
"""Hold the cusp corner's candidates to a second oracle's runs (ADR-0073).

Run with `uv run --project tools/oracle tools/oracle/cusp_corner_second.py`.
The input is `tests/fixtures/regression/cusp-corner-crescent-fillet/second_oracle_runs.json`:
the second oracle's runs of the three `regression/cusp-corner-*-fillet`
fixtures at three sizes (ADR-0073's neutral record, unscaled into the
fixtures' units, each run's operand measured by the same kernel, and its
face and edge counts; a refused run keeps its verdict and message). The record written beside it is `second.json`
(plan `cusp-corner-patch` step 2's open question). Per run, each kernel's
own change, `result − operand`:

- the second oracle's;
- Open CASCADE's both-edges fillet (its corner patch);
- ADR-0084 §5's least-bending graph, read from `graph.json`: Open
  CASCADE's change plus the graph's patch less Open CASCADE's patch (its
  finer resolution);
- the mutual trim: each selected edge filleted alone by Open CASCADE (a
  single cusp stripe runs to the cusp, cut by the other wall, ADR-0042),
  the two results combined by a boolean, `common` where the blends remove
  material (convex edges) and `fuse` where they add it (concave). No
  patch: the two stripes meet on their own intersection.

each against the second oracle's in units of Open CASCADE's own bound on
its patch (patch area times tolerance on volume, boundary length times
tolerance on area, ADR-0084 §Context), and relative to the body.
"""
import copy
import json
from pathlib import Path

from oracle import occt_version, require_ocp

require_ocp()
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Fuse  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402
from OCP.BRepGProp import BRepGProp  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from oracle.fixture import corpus_root, load_fixture  # noqa: E402
from oracle.recipe import build  # noqa: E402

HERE = corpus_root() / "regression" / "cusp-corner-crescent-fillet"
# Where the blends add material (a pocket's concave edges) the mutual trim
# is the union of the single-edge results, elsewhere their intersection.
CONCAVE = ("cusp-corner-pocket-fillet",)


def count(shape, kind):
    seen, e = [], TopExp_Explorer(shape, kind)
    while e.More():
        if not any(e.Current().IsSame(s) for s in seen):
            seen.append(e.Current())
        e.Next()
    return len(seen)


def measures(shape):
    v, a = GProp_GProps(), GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, v)
    BRepGProp.SurfaceProperties_s(shape, a)
    return {"volume": v.Mass(), "area": a.Mass(), "faces": count(shape, TopAbs_FACE),
            "edges": count(shape, TopAbs_EDGE), "valid": BRepCheck_Analyzer(shape).IsValid()}


def bodies(name, r):
    """Open CASCADE's operand, both-edges fillet and mutual trim at `r`."""
    fixture = load_fixture(corpus_root() / "regression" / name)
    fixture["params"]["r"] = r
    fillet = next(s for s in fixture["steps"] if s["op"] == "fillet")
    operand = copy.deepcopy(fixture)
    operand["steps"] = [s for s in fixture["steps"] if s["op"] != "fillet"]
    operand["result"] = fillet["of"]
    singles = []
    for edge in fillet["edges"]:
        alone = copy.deepcopy(fixture)
        next(s for s in alone["steps"] if s["op"] == "fillet")["edges"] = [edge]
        singles.append(build(alone, "default", [])[0])
    combine = BRepAlgoAPI_Fuse if name in CONCAVE else BRepAlgoAPI_Common
    trim = combine(singles[0], singles[1])
    trim.Build()
    return (build(operand, "default", [])[0], build(fixture, "default", [])[0], trim.Shape())


def record(run, graph):
    operand, both, trim = bodies(run["fixture"], run["size"])
    base = measures(operand)
    o = graph["oracle"]
    bound = {"volume": o["patch_area"] * o["patch_tolerance"], "area": o["boundary_length"] * o["patch_tolerance"]}
    second = {k: run[k] - run["operand"][k] for k in ("volume", "area")}
    fine = graph["least_bending_graph"][-1]

    def change(m):
        return {k: m[k] - base[k] for k in ("volume", "area")}

    occt = change(measures(both))
    candidates = {
        "occt_patch": {**occt, "faces": measures(both)["faces"], "edges": measures(both)["edges"]},
        "least_bending_graph": {k: occt[k] + fine[f"{k}_over_bound"] * bound[k] for k in ("volume", "area")},
        "mutual_trim": {**change(measures(trim)), **{k: measures(trim)[k] for k in ("faces", "edges", "valid")}},
    }
    for c in candidates.values():
        for k in ("volume", "area"):
            c[f"{k}_off_over_bound"] = (c[k] - second[k]) / bound[k]
            c[f"{k}_off_relative"] = (c[k] - second[k]) / run[k]
    return {"fixture": run["fixture"], "size": run["size"], "bound": bound,
            "operand_off_relative": {k: (run["operand"][k] - base[k]) / base[k] for k in ("volume", "area")},
            "second_oracle": {**second, "faces": run["faces"], "edges": run["edges"]}, **candidates}


def rounded(x):
    if isinstance(x, float):
        return float(f"{x:.4g}")
    if isinstance(x, dict):
        return {k: rounded(v) for k, v in x.items()}
    if isinstance(x, list):
        return [rounded(v) for v in x]
    return x


def main():
    runs = json.loads((HERE / "second_oracle_runs.json").read_text())["runs"]
    graphs = {(g["fixture"], g["size"]): g for g in json.loads((HERE / "graph.json").read_text())["records"]}
    built = [r for r in runs if r["verdict"] == "builds"]
    out = {"occt": occt_version(), "records": [record(r, graphs[(r["fixture"], r["size"])]) for r in built],
           "refused": [{k: r[k] for k in ("fixture", "size", "verdict", "why")} for r in runs if r not in built]}
    (HERE / "second.json").write_text(json.dumps(rounded(out), indent=2) + "\n")
    print(HERE / "second.json")


if __name__ == "__main__":
    main()
