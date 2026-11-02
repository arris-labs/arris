#!/usr/bin/env python3
"""Print cusp-corner oracle boundaries at three sizes as JSON.

Run with `uv run --project tools/oracle tools/oracle/cusp_corner.py`.
Samples are observations, not an independent construction or an error bound.
"""
import json

from oracle import occt_version, require_ocp

require_ocp()
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.GeomAbs import GeomAbs_BSplineSurface
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from oracle.fixture import corpus_root, load_fixture
from oracle.recipe import build


def explore(shape, kind, cast):
    explorer = TopExp_Explorer(shape, kind)
    while explorer.More():
        yield cast(explorer.Current())
        explorer.Next()


def measurements():
    records = []
    for directory in sorted((corpus_root() / "regression").glob("cusp-corner-*")):
        for size in (0.02, 0.1, 0.3):
            fixture = load_fixture(directory)
            fixture["params"]["r"] = size
            dropped = []
            shape, _ = build(fixture, "default", dropped)
            valid = BRepCheck_Analyzer(shape).IsValid()
            if not valid or dropped:
                raise RuntimeError(f"{directory.name} at {size}: invalid or dropped edges")
            faces = []
            for face in explore(shape, TopAbs_FACE, TopoDS.Face):
                surface = BRepAdaptor_Surface(face)
                record = {"kind": str(surface.GetType()).split(".")[-1]}
                if surface.GetType() == GeomAbs_BSplineSurface:
                    spline = surface.BSpline()
                    record["degree"] = [spline.UDegree(), spline.VDegree()]
                    record["boundaries"] = []
                    for edge in explore(face, TopAbs_EDGE, TopoDS.Edge):
                        curve = BRepAdaptor_Curve(edge)
                        lo, hi = curve.FirstParameter(), curve.LastParameter()
                        record["boundaries"].append([
                            list(curve.Value(lo + (hi - lo) * t).Coord())
                            for t in (0, 0.25, 0.5, 0.75, 1)
                        ])
                faces.append(record)
            records.append({"fixture": directory.name, "size": size,
                            "valid": valid, "dropped_edges": dropped, "faces": faces})
    if len(records) != 18:
        raise RuntimeError(f"expected 18 observations, got {len(records)}")
    return {"occt": occt_version(), "records": records}


if __name__ == "__main__":
    print(json.dumps(measurements(), indent=2))
