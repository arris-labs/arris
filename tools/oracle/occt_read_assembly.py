#!/usr/bin/env python3
"""occt_read_assembly.py <in.step>  — read a STEP file with
`STEPCAFControl_Reader` (names, colours and product structure on) and print,
as JSON, `roots`: each free shape of the XCAF document as an occurrence —
`name`, `placement` in its parent as a 3x4 row-major matrix (`null` at a
root), the part's `colour` (`null` when it has none), `children` — and
`instances`: the volume and centroid of every leaf part at the placement its
path composes, in depth-first order. What Arris's writer of an assembly is
held to (ADR-0033). Exits 2 with `occt_read_assembly: ERROR <why>` on stderr
when the file does not read. Run as
`uv run --project tools/oracle tools/oracle/occt_read_assembly.py`.
"""

import json
import sys
from pathlib import Path

from oracle import OracleError, require_ocp

require_ocp()

from OCP.BRepGProp import BRepGProp  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.IFSelect import IFSelect_RetDone  # noqa: E402
from OCP.Quantity import Quantity_Color, Quantity_TOC_sRGB  # noqa: E402
from OCP.STEPCAFControl import STEPCAFControl_Reader  # noqa: E402
from OCP.TCollection import TCollection_ExtendedString  # noqa: E402
from OCP.collections import Sequence_TDF_Label  # noqa: E402
from OCP.TDF import TDF_Label  # noqa: E402
from OCP.TDataStd import TDataStd_Name  # noqa: E402
from OCP.TDocStd import TDocStd_Document  # noqa: E402
from OCP.TopLoc import TopLoc_Location  # noqa: E402
from OCP.XCAFApp import XCAFApp_Application  # noqa: E402
from OCP.XCAFDoc import XCAFDoc_ColorSurf, XCAFDoc_ColorTool, XCAFDoc_DocumentTool  # noqa: E402
from OCP.gp import gp_Trsf  # noqa: E402

from oracle import step  # noqa: E402


def name_of(label) -> str:
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""


def matrix(trsf: gp_Trsf) -> list[float]:
    return [trsf.Value(r, c) for r in (1, 2, 3) for c in (1, 2, 3, 4)]


def main(argv: list[str]) -> int:
    if len(argv) != 1:
        print(__doc__)
        return 2
    try:
        app = XCAFApp_Application.GetApplication_s()
        doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
        app.InitDocument(doc)
        step.quiet()
        reader = STEPCAFControl_Reader()
        reader.SetColorMode(True)
        reader.SetNameMode(True)
        if reader.ReadFile(argv[0]) != IFSelect_RetDone:
            raise OracleError(f"STEP read failed for {argv[0]}")
        if not reader.Transfer(doc):
            raise OracleError("the XCAF transfer failed")
        tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
        colours = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
        instances = []

        def visit(label, placement, world):
            colour = Quantity_Color()
            has = XCAFDoc_ColorTool.GetColor_s(label, XCAFDoc_ColorSurf, colour)
            node = {
                "name": name_of(label),
                "placement": None if placement is None else matrix(placement),
                "colour": [colour.Red(), colour.Green(), colour.Blue()] if has else None,
                "children": [],
            }
            if has:
                # Red, Green, Blue are linear; the file spells sRGB.
                r, g, b = colour.Values(Quantity_TOC_sRGB)
                node["colour"] = [r, g, b]
            if tool.IsAssembly_s(label):
                comps = Sequence_TDF_Label()
                tool.GetComponents_s(label, comps)
                for i in range(1, comps.Length() + 1):
                    comp = comps.Value(i)
                    ref = TDF_Label()
                    if not tool.GetReferredShape_s(comp, ref):
                        continue
                    loc = tool.GetLocation_s(comp)
                    trsf = loc.Transformation()
                    node["children"].append(visit(ref, trsf, world.Multiplied(trsf)))
            else:
                shape = tool.GetShape_s(label).Moved(TopLoc_Location(world))
                props = GProp_GProps()
                BRepGProp.VolumeProperties_s(shape, props)
                c = props.CentreOfMass()
                instances.append({"volume": props.Mass(), "centroid": [c.X(), c.Y(), c.Z()]})
            return node

        roots = Sequence_TDF_Label()
        tool.GetFreeShapes(roots)
        tree = [visit(roots.Value(i), None, gp_Trsf()) for i in range(1, roots.Length() + 1)]
    except Exception as e:  # OracleError, or Open CASCADE's own
        message = " ".join(str(e).split()) or type(e).__name__
        print(f"occt_read_assembly: ERROR {message}", file=sys.stderr)
        return 2
    print(json.dumps({"roots": tree, "instances": instances}))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
