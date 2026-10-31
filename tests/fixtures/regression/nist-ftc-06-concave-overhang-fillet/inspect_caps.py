"""Reproduce this fixture's cap evidence with Open CASCADE alone.

Run from the repository root with:
  uv run --project tools/oracle tests/fixtures/regression/nist-ftc-06-concave-overhang-fillet/inspect_caps.py

Samples are a 41 by 41 grid in each cap's UV bounds, retaining only points
classified IN or ON the trimmed face. Distance is to the nearest torus in
the result, so a positive distance also rules out the stripe's torus.
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[3] / "tools" / "oracle"))

from oracle import require_ocp  # noqa: E402

require_ocp()

from OCP.BRep import BRep_Tool  # noqa: E402
from OCP.BRepAdaptor import BRepAdaptor_Surface  # noqa: E402
from OCP.BRepClass import BRepClass_FaceClassifier  # noqa: E402
from OCP.BRepTools import BRepTools  # noqa: E402
from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf  # noqa: E402
from OCP.GeomAbs import GeomAbs_SurfaceType  # noqa: E402
from OCP.TopAbs import TopAbs_FACE, TopAbs_IN, TopAbs_ON  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402
from OCP.gp import gp_Pnt2d  # noqa: E402

from oracle.fixture import load_fixture  # noqa: E402
from oracle.measure import measure  # noqa: E402
from oracle.recipe import build  # noqa: E402


def evidence():
    """The oracle's volume change and cap distances, independent of Arris."""
    result, steps = build(load_fixture(HERE))
    faces = []
    explorer = TopExp_Explorer(result, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face(explorer.Current())
        faces.append((face, BRepAdaptor_Surface(face)))
        explorer.Next()
    tori = [BRep_Tool.Surface_s(f) for f, a in faces if a.GetType() == GeomAbs_SurfaceType.GeomAbs_Torus]
    caps = []
    for face, surface in faces:
        if surface.GetType() != GeomAbs_SurfaceType.GeomAbs_BSplineSurface:
            continue
        u0, u1, v0, v1 = BRepTools.UVBounds_s(face)
        distances = []
        for i in range(41):
            for j in range(41):
                u, v = u0 + (u1 - u0) * i / 40, v0 + (v1 - v0) * j / 40
                if BRepClass_FaceClassifier(face, gp_Pnt2d(u, v), 1e-9).State() not in (TopAbs_IN, TopAbs_ON):
                    continue
                point = surface.Value(u, v)
                distances.append(min(GeomAPI_ProjectPointOnSurf(point, torus).LowerDistance() for torus in tori))
        caps.append({"degrees": [surface.UDegree(), surface.VDegree()], "trimmed_samples": len(distances), "max_distance_to_any_torus": max(distances)})
    operand = measure(steps["part"], [], 1e-7)
    output = measure(result, [], 1e-7)
    return {"operand_volume": operand["volume"], "result_volume": output["volume"], "volume_change": output["volume"] - operand["volume"], "caps": caps}


if __name__ == "__main__":
    print(json.dumps(evidence(), indent=2))
