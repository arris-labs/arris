#!/usr/bin/env python3
"""Sample the mutual trim's section edge of each cusp-corner fillet.

Run with `uv run --project tools/oracle tools/oracle/cusp_corner_section.py`.
The record is `tests/fixtures/regression/cusp-corner-crescent-fillet/section.json`
(plan `cusp-corner-patch` step 2, ADR-0084 §4′). For each of the three
`regression/cusp-corner-*-fillet` recipes at three sizes: each selected
edge filleted alone by Open CASCADE (a single cusp stripe, cut by the other
wall, ADR-0042), the two results combined by `common` where the blends
remove material and by `fuse` where they add it (the pocket), and the edge
of that body the two stripes share: the section of the two stripe
surfaces, from the top's contact `T*` down to the spine at `S`. Per run,
the edge's tolerance, its two vertices with theirs, and points at equal
steps of its own parameter, ends included, which `blend::mutual`'s test holds Arris's section to.

A stripe face is a torus, or a cylinder of the blend's radius (a line
edge's stripe). Observations, not an implementation.
"""
import copy
import json

from oracle import occt_version, require_ocp

require_ocp()
from OCP.BRep import BRep_Tool  # noqa: E402
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface  # noqa: E402
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Fuse  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Torus  # noqa: E402
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE  # noqa: E402
from OCP.TopExp import TopExp, TopExp_Explorer  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402
from oracle.fixture import corpus_root, load_fixture  # noqa: E402
from oracle.recipe import build  # noqa: E402

HERE = corpus_root() / "regression" / "cusp-corner-crescent-fillet"
FIXTURES = ("cusp-corner-crescent-fillet", "cusp-corner-pocket-fillet", "cusp-corner-spandrel-fillet")
CONCAVE = ("cusp-corner-pocket-fillet",)
SIZES = (0.02, 0.1, 0.3)
# Points on the edge, at equal steps of its parameter, ends included.
SAMPLES = 33


def explore(shape, kind, cast):
    explorer = TopExp_Explorer(shape, kind)
    while explorer.More():
        yield cast(explorer.Current())
        explorer.Next()


def stripe(face, r):
    surface = BRepAdaptor_Surface(face)
    kind = surface.GetType()
    if kind == GeomAbs_Torus:
        return abs(surface.Torus().MinorRadius() - r) < 1e-9
    if kind == GeomAbs_Cylinder:
        return abs(surface.Cylinder().Radius() - r) < 1e-9
    return False


def trim(name, r):
    fixture = load_fixture(corpus_root() / "regression" / name)
    fixture["params"]["r"] = r
    fillet = next(s for s in fixture["steps"] if s["op"] == "fillet")
    singles = []
    for edge in fillet["edges"]:
        alone = copy.deepcopy(fixture)
        next(s for s in alone["steps"] if s["op"] == "fillet")["edges"] = [edge]
        singles.append(build(alone, "default", [])[0])
    combine = BRepAlgoAPI_Fuse if name in CONCAVE else BRepAlgoAPI_Common
    op = combine(singles[0], singles[1])
    op.Build()
    shape = op.Shape()
    if not BRepCheck_Analyzer(shape).IsValid():
        raise RuntimeError(f"{name} at {r}: the mutual trim is not valid")
    return shape


def record(name, r):
    shape = trim(name, r)
    stripes = [f for f in explore(shape, TopAbs_FACE, TopoDS.Face) if stripe(f, r)]
    shared = []
    for i, f in enumerate(stripes):
        for g in stripes[i + 1:]:
            for edge in explore(f, TopAbs_EDGE, TopoDS.Edge):
                if any(edge.IsSame(x) for x in explore(g, TopAbs_EDGE, TopoDS.Edge)):
                    if not any(edge.IsSame(s) for s in shared):
                        shared.append(edge)
    if len(shared) != 1:
        raise RuntimeError(f"{name} at {r}: {len(shared)} edges shared by two stripes, not one")
    edge = shared[0]
    curve = BRepAdaptor_Curve(edge)
    lo, hi = curve.FirstParameter(), curve.LastParameter()
    points = [list(curve.Value(lo + (hi - lo) * k / (SAMPLES - 1)).Coord()) for k in range(SAMPLES)]
    first, last = TopExp.FirstVertex_s(edge, True), TopExp.LastVertex_s(edge, True)
    return {"fixture": name, "size": r, "curve": str(curve.GetType()).split("_")[-1],
            "tolerance": BRep_Tool.Tolerance_s(edge),
            "vertices": [{"point": list(BRep_Tool.Pnt_s(v).Coord()), "tolerance": BRep_Tool.Tolerance_s(v)}
                         for v in (first, last)],
            "points": points}


def main():
    out = {"occt": occt_version(), "samples": SAMPLES,
           "records": [record(name, r) for name in FIXTURES for r in SIZES]}
    (HERE / "section.json").write_text(json.dumps(out, indent=1) + "\n")
    print(HERE / "section.json")


if __name__ == "__main__":
    main()
