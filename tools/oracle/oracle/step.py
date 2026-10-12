"""STEP AP214 in and out through Open CASCADE."""

import re
from pathlib import Path

from OCP.BRepBuilderAPI import BRepBuilderAPI_NurbsConvert
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.Message import Message, Message_Gravity
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Reader, STEPControl_Writer
from OCP.TopAbs import TopAbs_SHELL, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TCollection import TCollection_AsciiString
from OCP.TopoDS import TopoDS_Shape
from OCP.collections import DataMap_TCollection_AsciiString_TCollection_AsciiString

from . import OracleError


def quiet() -> None:
    """Keep only failures and alarms from OCCT's messenger: the STEP transfer
    otherwise prints a statistics banner per file."""
    for printer in list(Message.DefaultMessenger_s().Printers()):
        printer.SetTraceLevel(Message_Gravity.Message_Fail)


def write(shape: TopoDS_Shape, path: Path) -> None:
    quiet()
    Interface_Static.SetCVal_s("write.step.schema", "AP214")
    w = STEPControl_Writer()
    if w.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone:
        raise OracleError(f"STEP transfer failed for {path}")
    if w.Write(str(path)) != IFSelect_RetDone:
        raise OracleError(f"STEP write failed for {path}")


def read(path: Path) -> TopoDS_Shape:
    if not path.exists():
        raise OracleError(f"no such STEP file: {path}")
    quiet()
    r = STEPControl_Reader()
    if r.ReadFile(str(path)) != IFSelect_RetDone:
        raise OracleError(f"STEP read failed for {path}")
    r.TransferRoots()
    if r.NbShapes() == 0:
        raise OracleError(f"STEP file has no shapes: {path}")
    return r.OneShape()


def nurbs(shape: TopoDS_Shape) -> TopoDS_Shape:
    """`shape` with every surface and curve converted to B-splines by
    `BRepBuilderAPI_NurbsConvert`: the free-form faces with seams and poles
    the STEP reader is held to (ADR-0025)."""
    converter = BRepBuilderAPI_NurbsConvert(shape, True)
    if not converter.IsDone():
        raise OracleError("BRepBuilderAPI_NurbsConvert failed")
    return converter.Shape()



# A statement's start, past blanks and comments: `ENDSEC`, or an
# instance's `#id =`.
_STATEMENT = re.compile(r"(?:\s|/\*.*?\*/)*(?:(ENDSEC)|#(\d+)\s*=)?", re.S)


def instances(text: str) -> dict[int, str]:
    """Every instance of the DATA sections of a Part 21 file: its `#id` and
    the text after its `=`, up to its `;`. Strings (`'…'`, a quote doubled
    inside) and comments are skipped over, so a `;` or a `#12=` in either
    neither ends nor starts an instance."""
    out: dict[int, str] = {}
    for section in re.finditer(r"\bDATA\b[^;]*;", text):
        at = section.end()
        while at < len(text):
            m = _STATEMENT.match(text, at)
            if m.group(1) == "ENDSEC":
                break
            body_start = m.end()
            at = _statement_end(text, body_start)
            if m.group(2):
                out[int(m.group(2))] = text[body_start:at]
            at += 1
    return out


def _statement_end(text: str, at: int) -> int:
    """The index of the `;` that ends the statement from `at`."""
    while at < len(text):
        c = text[at]
        if c == "'":
            end = at + 1
            while True:
                end = text.find("'", end)
                if end < 0:
                    raise OracleError("a string in the DATA section does not end")
                if text.startswith("''", end):
                    end += 2
                    continue
                break
            at = end + 1
        elif text.startswith("/*", at):
            end = text.find("*/", at + 2)
            if end < 0:
                raise OracleError("a comment in the DATA section does not end")
            at = end + 2
        elif c == ";":
            return at
        else:
            at += 1
    raise OracleError("an instance in the DATA section does not end")


_TOKEN = re.compile(r"\s*(?:(\()|(\))|(,)|#(\d+)|('(?:[^']|'')*')|([^\s(),']+))")


def _params(body: str) -> tuple[str, list]:
    """A simple instance's type name and its parameters: a reference as an
    `int`, a list as a `list`, anything else as its text."""
    m = re.match(r"\s*([A-Z_0-9]+)\s*\(", body)
    if not m:
        raise OracleError(f"not a simple instance: {body[:60]!r}")
    stack: list[list] = [[]]
    at = m.end()
    while stack:
        t = _TOKEN.match(body, at)
        if not t or t.end() == at:
            raise OracleError(f"cannot read the parameters of {body[:60]!r}")
        at = t.end()
        if t.group(1):
            stack.append([])
        elif t.group(2):
            done = stack.pop()
            if not stack:
                return m.group(1), done
            stack[-1].append(done)
        elif t.group(4):
            stack[-1].append(int(t.group(4)))
        elif t.group(5) or t.group(6):
            stack[-1].append(t.group(5) or t.group(6))
    raise OracleError(f"unbalanced parameters in {body[:60]!r}")


# The solids a file defines, as the reader looks for them (ADR-0025).
SOLID_TYPES = ("MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS")


def _first_point(bodies: dict[int, str], shell: int) -> tuple[int, tuple[float, ...]]:
    """A shell's face count and the start point of the first edge of the
    first bound of its first face: what names its solid on both sides of
    Open CASCADE's model, which keeps no `#id`."""
    _, (_, faces) = _params(bodies[shell])
    _, face = _params(bodies[faces[0]])
    _, bound = _params(bodies[face[1][0]])
    _, loop = _params(bodies[bound[1]])
    _, oriented = _params(bodies[loop[1][0]])
    _, edge = _params(bodies[oriented[3]])
    _, vertex = _params(bodies[edge[1]])
    _, point = _params(bodies[vertex[1]])
    return len(faces), tuple(float(x) for x in point[1])


def _entity_point(entity) -> tuple[int, tuple[float, ...]]:
    """`_first_point` of an entity of Open CASCADE's model."""
    shell = entity.Outer()
    point = (
        shell.CfsFacesValue(1).BoundsValue(1).Bound().EdgeListValue(1).EdgeElement().EdgeStart().VertexGeometry()
    )
    return shell.NbCfsFaces(), tuple(point.CoordinatesValue(i) for i in (1, 2, 3))


# Every switch of the transfer's `FixShape` (Open CASCADE's
# `XSAlgo_ShapeProcessor`, `FixShape.<name>`): all `0` is its healing off.
FIX_MODES = (
    "AutoCorrectPrecisionMode", "ClosedWireMode", "CreateOpenSolidMode", "FixAddCurve3dMode",
    "FixAddNaturalBoundMode", "FixAddPCurveMode", "FixConnectedMode", "FixDegeneratedMode",
    "FixEdgeCurvesMode", "FixEdgeSameParameterMode", "FixFaceMode", "FixFaceOrientationMode",
    "FixFreeFaceMode", "FixFreeShellMode", "FixFreeWireMode", "FixIntersectingEdgesMode",
    "FixIntersectingWiresMode", "FixLackingMode", "FixLoopWiresMode", "FixMissingSeamMode",
    "FixNonAdjacentIntersectingEdgesMode", "FixNotchedEdgesMode", "FixOrientationMode",
    "FixRemoveCurve3dMode", "FixRemovePCurveMode", "FixReorderMode", "FixReversed2dMode",
    "FixSameParameterMode", "FixSeamMode", "FixSelfIntersectingEdgeMode",
    "FixSelfIntersectionMode", "FixShellMode", "FixShellOrientationMode", "FixShiftedMode",
    "FixSmallAreaWireMode", "FixSmallMode", "FixSolidMode", "FixSplitFaceMode", "FixTailMode",
    "FixVertexPositionMode", "FixVertexToleranceMode", "FixWireMode", "ModifyGeometryMode",
    "ModifyTopologyMode", "PreferencePCurveMode", "RemoveLoopMode", "RemoveSmallAreaFaceMode",
)


def solids(path: Path, heal: bool = True) -> list[tuple[int, TopoDS_Shape]]:
    """Every solid Open CASCADE's reader transfers from `path`, placed where
    the product structure puts it and healed as its reader heals by default
    (ADR-0026 §3) — or, with `heal` false, with every `FixShape` switch
    off, which only takes once `ReadFile` has made the transfer's actor —
    each with the `#id` of the `MANIFOLD_SOLID_BREP` or
    `BREP_WITH_VOIDS` it came from, in the order the transfer gives them.

    The entity behind a solid is `EntityFromShapeResult`'s for the solid
    unplaced (placed, it is the occurrence that places it). Open CASCADE's
    model keeps no `#id` (its `Number` and `IdentLabel` answer 0 in this
    build) and drops some instances nothing refers to, so the `#id` is
    found by content instead: the face count and first vertex of the
    solid's outer shell, read from the file's own text, which must name
    one solid of the file alone."""
    if not path.exists():
        raise OracleError(f"no such STEP file: {path}")
    bodies = instances(path.read_bytes().decode("latin-1"))
    by_point: dict[tuple, list[int]] = {}
    for label, body in bodies.items():
        name = re.match(r"\s*([A-Z_0-9]+)", body)
        if name and name.group(1) in SOLID_TYPES:
            _, params = _params(body)
            by_point.setdefault(_first_point(bodies, params[1]), []).append(label)
    quiet()
    r = STEPControl_Reader()
    if r.ReadFile(str(path)) != IFSelect_RetDone:
        raise OracleError(f"STEP read failed for {path}")
    if not heal:
        switches = DataMap_TCollection_AsciiString_TCollection_AsciiString()
        for name in FIX_MODES:
            switches.Bind(TCollection_AsciiString(f"FixShape.{name}"), TCollection_AsciiString("0"))
        r.SetShapeFixParameters(switches)
    r.TransferRoots()
    transfer = r.WS().TransferReader()
    out = []
    for k in range(1, r.NbShapes() + 1):
        explorer = TopExp_Explorer(r.Shape(k), TopAbs_SOLID)
        while explorer.More():
            solid = explorer.Current()
            # Located, a placed solid's result is the assembly occurrence
            # that placed it; unlocated, the solid entity itself.
            entity = transfer.EntityFromShapeResult(solid.Located(TopLoc_Location()), 1)
            if entity is None or not hasattr(entity, "Outer"):
                raise OracleError(f"{path}: a solid Open CASCADE read has no solid entity of the file behind it")
            labels = by_point.get(_entity_point(entity), [])
            if len(labels) != 1:
                raise OracleError(f"{path}: a solid Open CASCADE read matches {len(labels)} solids of the file by its first vertex")
            out.append((labels[0], solid))
            explorer.Next()
    return out


def sheets(path: Path) -> list[TopoDS_Shape]:
    """Every shell Open CASCADE's reader transfers from `path` that no
    solid holds -- a `SHELL_BASED_SURFACE_MODEL`'s open or closed shells, a
    `FACE_BASED_SURFACE_MODEL`'s connected face sets -- placed where the
    product structure puts them and healed as its reader heals by default
    (ADR-0057), in the order the transfer gives them. Open CASCADE keeps no
    `#id` of the entity behind a shape (`solids`), so a sheet is matched to
    Arris's by its face count and centroid."""
    if not path.exists():
        raise OracleError(f"no such STEP file: {path}")
    quiet()
    r = STEPControl_Reader()
    if r.ReadFile(str(path)) != IFSelect_RetDone:
        raise OracleError(f"STEP read failed for {path}")
    r.TransferRoots()
    out = []
    for k in range(1, r.NbShapes() + 1):
        # Shells outside every solid, then faces outside every shell.
        explorer = TopExp_Explorer(r.Shape(k), TopAbs_SHELL, TopAbs_SOLID)
        while explorer.More():
            out.append(explorer.Current())
            explorer.Next()
    return out
