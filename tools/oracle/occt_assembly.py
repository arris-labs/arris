#!/usr/bin/env python3
"""occt_assembly.py <fixture-a> <fixture-b> <out.step>  — build both
fixtures' results in Open CASCADE, make a named XCAF assembly "assembly" of
the first ("part-a", placed once) and a sub-assembly "sub-assembly" placed
once, which holds the second ("part-b") placed twice, so the second's paths
are two deep. The parts are coloured, and one face of the first apart. Write it with
`STEPCAFControl_Writer` — product structure, `NEXT_ASSEMBLY_USAGE_OCCURRENCE`
and `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`, names, styled items — and
print, as JSON, `instances` (each placed instance's volume and centroid
measured on the placed shape, in the order box, block, block), `tree` (the
expected product tree: name, placement in the parent as a 3x4 row-major
matrix, colour, `solids` as indices into `instances`, children) and `faces`
(the coloured face: its instance, centroid, area and colour): what Arris's
reader of the file is held to (ADR-0025 §5, ADR-0033). Exits 2 with `occt_assembly: ERROR <why>` on stderr when a
recipe does not build or the file is not written. Run as
`uv run --project tools/oracle tools/oracle/occt_assembly.py`.
"""

import json
import math
import sys
from pathlib import Path

from oracle import OracleError, require_ocp

require_ocp()

from OCP.BRepGProp import BRepGProp  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.IFSelect import IFSelect_RetDone  # noqa: E402
from OCP.STEPCAFControl import STEPCAFControl_Writer  # noqa: E402
from OCP.STEPControl import STEPControl_AsIs  # noqa: E402
from OCP.TCollection import TCollection_ExtendedString  # noqa: E402
from OCP.Quantity import Quantity_Color, Quantity_TOC_sRGB  # noqa: E402
from OCP.TDataStd import TDataStd_Name  # noqa: E402
from OCP.TDocStd import TDocStd_Document  # noqa: E402
from OCP.TopLoc import TopLoc_Location  # noqa: E402
from OCP.XCAFApp import XCAFApp_Application  # noqa: E402
from OCP.XCAFDoc import XCAFDoc_ColorSurf, XCAFDoc_DocumentTool  # noqa: E402
from OCP.TopAbs import TopAbs_FACE  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec  # noqa: E402

from oracle import step  # noqa: E402
from oracle.fixture import load_fixture  # noqa: E402
from oracle.recipe import build  # noqa: E402


def placement(axis: tuple[float, float, float], degrees: float, by: tuple[float, float, float]) -> gp_Trsf:
    """A turn about `axis` through the origin, then a translation."""
    turn = gp_Trsf()
    turn.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(*axis)), math.radians(degrees))
    shift = gp_Trsf()
    shift.SetTranslation(gp_Vec(*by))
    return shift.Multiplied(turn)


# The first fixture once at the top, a sub-assembly once, and the second
# twice inside it: a turn and a shift each, none the identity, so a
# placement read the wrong way round moves a centroid.
A_IN_ROOT = placement((0, 0, 1), 30.0, (10.0, 0.0, 0.0))
SUB_IN_ROOT = placement((0, 1, 0), 20.0, (0.0, 0.0, 30.0))
B_IN_SUB = [
    placement((1, 0, 0), 90.0, (0.0, 20.0, 0.0)),
    placement((0, 1, 1), -45.0, (-7.0, -20.0, 5.0)),
]
# Colours that spell exactly in decimal, distinct per part and one face.
COLOUR_A = (0.75, 0.25, 0.125)
COLOUR_B = (0.0625, 0.5, 0.75)
COLOUR_FACE = (0.9375, 0.875, 0.0625)


def name_of(label, text: str) -> None:
    TDataStd_Name.Set_s(label, TCollection_ExtendedString(text))


def matrix(trsf: gp_Trsf) -> list[float]:
    return [trsf.Value(r, c) for r in (1, 2, 3) for c in (1, 2, 3, 4)]


def measure(shape) -> dict:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    c = props.CentreOfMass()
    return {"volume": props.Mass(), "centroid": [c.X(), c.Y(), c.Z()]}


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(__doc__)
        return 2
    try:
        shapes = [build(load_fixture(Path(d)), "default")[0] for d in argv[:2]]
        app = XCAFApp_Application.GetApplication_s()
        doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
        app.InitDocument(doc)
        tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
        colours = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
        rgb = lambda c: Quantity_Color(c[0], c[1], c[2], Quantity_TOC_sRGB)  # noqa: E731
        parts = [tool.AddShape(s, False) for s in shapes]
        for label, text, colour in ((parts[0], "part-a", COLOUR_A), (parts[1], "part-b", COLOUR_B)):
            name_of(label, text)
            colours.SetColor(label, rgb(colour), XCAFDoc_ColorSurf)
        # One face of the first part, coloured apart: its first.
        faces = TopExp_Explorer(shapes[0], TopAbs_FACE)
        face = faces.Current()
        colours.SetColor(face, rgb(COLOUR_FACE), XCAFDoc_ColorSurf)
        face_props = GProp_GProps()
        BRepGProp.SurfaceProperties_s(face, face_props)
        face_centre = face_props.CentreOfMass()
        face_at = {"centroid": [face_centre.X(), face_centre.Y(), face_centre.Z()], "area": face_props.Mass()}

        sub = tool.NewShape()
        name_of(sub, "sub-assembly")
        assembly = tool.NewShape()
        name_of(assembly, "assembly")
        tool.AddComponent(assembly, parts[0], TopLoc_Location(A_IN_ROOT))
        tool.AddComponent(assembly, sub, TopLoc_Location(SUB_IN_ROOT))
        for trsf in B_IN_SUB:
            tool.AddComponent(sub, parts[1], TopLoc_Location(trsf))
        tool.UpdateAssemblies()

        # A part's placed shape: the parent's placement applied after the
        # child's, `outer.Multiplied(inner)`.
        instances = [measure(shapes[0].Moved(TopLoc_Location(A_IN_ROOT)))]
        for trsf in B_IN_SUB:
            instances.append(measure(shapes[1].Moved(TopLoc_Location(SUB_IN_ROOT.Multiplied(trsf)))))
        # The coloured face as it stands placed.
        placed_face = face.Moved(TopLoc_Location(A_IN_ROOT))
        placed_props = GProp_GProps()
        BRepGProp.SurfaceProperties_s(placed_face, placed_props)
        pc = placed_props.CentreOfMass()
        face_at["centroid"] = [pc.X(), pc.Y(), pc.Z()]
        face_at["instance"] = 0
        face_at["colour"] = list(COLOUR_FACE)
        tree = {
            "name": "assembly",
            "placement": None,
            "colour": None,
            "solids": [],
            "children": [
                {
                    "name": "part-a",
                    "placement": matrix(A_IN_ROOT),
                    "colour": list(COLOUR_A),
                    "solids": [0],
                    "children": [],
                },
                {
                    "name": "sub-assembly",
                    "placement": matrix(SUB_IN_ROOT),
                    "colour": None,
                    "solids": [],
                    "children": [
                        {
                            "name": "part-b",
                            "placement": matrix(trsf),
                            "colour": list(COLOUR_B),
                            "solids": [1 + i],
                            "children": [],
                        }
                        for i, trsf in enumerate(B_IN_SUB)
                    ],
                },
            ],
        }
        step.quiet()
        writer = STEPCAFControl_Writer()
        if not writer.Transfer(doc, STEPControl_AsIs):
            raise OracleError("the XCAF transfer failed")
        out = Path(argv[2])
        out.parent.mkdir(parents=True, exist_ok=True)
        if writer.Write(str(out)) != IFSelect_RetDone:
            raise OracleError(f"STEP write failed for {out}")
    except Exception as e:  # OracleError, or Open CASCADE's own
        message = " ".join(str(e).split()) or type(e).__name__
        print(f"occt_assembly: ERROR {message}", file=sys.stderr)
        return 2
    print(json.dumps({"instances": instances, "tree": tree, "faces": [face_at]}))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
