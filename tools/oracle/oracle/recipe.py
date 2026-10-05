"""Build a shape from a fixture recipe.

The recipe grammar (also in `tests/fixtures/README.md`):

    {
      "params":   {"t": 10, "r": 3},                 # optional numbers
      "variants": {"thicker": {"t": 12}},           # optional param overrides
      "steps": [  {"name": "...", "op": "...", ...}, ... ],
      "result":   "<name of the step whose shape is the fixture's result>",
      "probes":   [{"point": [x, y, z], "label": "inside"}, ...],
      "tolerances": {"volume_rel": 1e-9, "area_rel": 1e-9,
                     "centroid_abs": 1e-7, "probe": 1e-7}
    }

Every number in a step may be a string expression over the params
(`"50 + R * cos(radians(45))"`); a plain JSON number is used as is.

Operations, by `op`:

    box        min [x,y,z], max [x,y,z]
    cylinder   base [x,y,z], axis [x,y,z], radius, height
    profile    plane {origin, x, y}, outer <loop>, holes [<loop>, ...]
               <loop> = {"circle": {"center": [u,v], "radius": r}}
                      | {"ellipse": {"center": [u,v], "major": [du,dv],
                                     "minor_radius": b}}
                      | {"start": [u,v], "segments": [
                            {"line_to": [u,v]},
                            {"arc_to": [u,v], "via": [u,v]},
                            {"ellipse_to": [u,v], "center": [u,v],
                             "major": [du,dv], "minor_radius": b,
                             "ccw": true}, ...]}
               (the last segment ends at `start`; loop orientation is
               irrelevant, holes are oriented by the interpreter; an
               ellipse's `major` runs from the centre to a major vertex,
               a `minor_radius` longer than it swaps the axes as
               `gp_Elips` needs, and `ccw` is the turn about the plane's
               normal — ADR-0014)
    extrude    profile <name>, direction [x,y,z], length
    revolve    profile <name>, axis {origin, direction}, angle_deg
    transform  of <name>, translate [x,y,z] (optional),
               rotate {axis [x,y,z], origin [x,y,z], angle_deg} (optional)
    mirror     of <name>, plane {origin [x,y,z], normal [x,y,z]}
    fuse       a <name>, b <name>
    common     a <name>, b <name>
    cut        target <name>, tool <name>
    fillet     of <name>, edges [[x,y,z], ...], radius
               (each point names the edge nearest to it, which must be the
               only edge within the fixture's `probe` tolerance)
    chamfer    of <name>, edges [[x,y,z], ...], distance
               (edges named as a fillet's; one distance, measured on both
               faces from the edge)
    offset     of <name>, faces [[x,y,z], ...], distance
               (the faces moved along their outward normals by the signed
               distance, positive adding material, each named by a point
               that lies on that face alone, within the fixture's `probe`;
               Open CASCADE's `BRepOffset_MakeOffset` with offset 0 on the
               fixed faces and `SetOffsetOnFace` on the moved ones, the
               intersection join, a closed shell it returns made the
               solid it bounds; the named faces closed over tangent
               edges first, each face tangent to a moved one moved with
               it, as Arris drags the chain; an offset Open CASCADE
               refuses or crashes on, tried in a forked child, is the
               empty compound — ADR-0048)
    polyhedron points [[x,y,z], ...], faces [[[i, j, k, ...], <hole>...], ...],
               namespace <u32>
               (each face its loops of point indices, the outer one
               counter-clockwise seen from outside, a hole's clockwise;
               built as polygon wires on planar faces, sewn, made a solid.
               `namespace` is Arris's key space and unread here)
    step       file <path beside fixture.json>, sha256 <of the file>,
               id <#id of its MANIFOLD_SOLID_BREP or BREP_WITH_VOIDS>,
               near [x,y,z] (where the file places it more than once:
               the placement whose centroid is nearest)
               (read by Open CASCADE's reader, healed — ADR-0026 §3)

Conventions are Open CASCADE's: a full revolve (360°) has seam edges, a
fuse of flush boxes does not merge coplanar faces, a common with no volume
is an empty compound. Arris mirrors these; the fixtures' `analytic` values
catch a mismatch on either side.
"""

import ast
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any

from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Curve2d, BRepAdaptor_Surface
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakePolygon,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_Sewing,
    BRepBuilderAPI_MakeVertex,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Transform,
)
from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepFilletAPI import BRepFilletAPI_MakeChamfer, BRepFilletAPI_MakeFillet
from OCP.BRepGProp import BRepGProp
from OCP.BRepOffset import BRepOffset_MakeOffset, BRepOffset_Skin
from OCP.BRepLib import BRepLib
from OCP.BRepLProp import BRepLProp_SLProps
from OCP.BRepPrimAPI import (
    BRepPrimAPI_MakeBox,
    BRepPrimAPI_MakeCylinder,
    BRepPrimAPI_MakePrism,
    BRepPrimAPI_MakeRevol,
)
from OCP.GeomAbs import GeomAbs_Intersection
from OCP.GC import GC_MakeArcOfCircle, GC_MakeArcOfEllipse
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax1, gp_Ax2, gp_Ax3, gp_Circ, gp_Dir, gp_Elips, gp_Pln, gp_Pnt, gp_Trsf, gp_Vec
from OCP.ShapeFix import ShapeFix_Face
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED, TopAbs_SHELL
from OCP.TopExp import TopExp
from OCP.TopoDS import TopoDS, TopoDS_Compound, TopoDS_Shape
from OCP.collections import IndexedDataMap_TopoDS_Shape_List_TopoDS_Shape_TopTools_ShapeMapHasher as IndexedDataMapOfShapeListOfShape
from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as IndexedMapOfShape

from . import OracleError, step as step_file

# --- parameters and expressions -------------------------------------------

_FUNCS = {
    "sin": math.sin,
    "cos": math.cos,
    "tan": math.tan,
    "atan": math.atan,
    "sqrt": math.sqrt,
    "radians": math.radians,
    "degrees": math.degrees,
    "abs": abs,
    "pi": math.pi,
}


def _eval(node: ast.AST, params: dict[str, float]) -> float:
    if isinstance(node, ast.Expression):
        return _eval(node.body, params)
    if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
        return float(node.value)
    if isinstance(node, ast.Name):
        if node.id in params:
            return float(params[node.id])
        if node.id in _FUNCS and not callable(_FUNCS[node.id]):
            return float(_FUNCS[node.id])
        raise OracleError(f"unknown name {node.id!r} in expression")
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
        v = _eval(node.operand, params)
        return -v if isinstance(node.op, ast.USub) else v
    if isinstance(node, ast.BinOp):
        a, b = _eval(node.left, params), _eval(node.right, params)
        if isinstance(node.op, ast.Add):
            return a + b
        if isinstance(node.op, ast.Sub):
            return a - b
        if isinstance(node.op, ast.Mult):
            return a * b
        if isinstance(node.op, ast.Div):
            return a / b
        if isinstance(node.op, ast.Pow):
            return a**b
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
        f = _FUNCS.get(node.func.id)
        if callable(f) and not node.keywords:
            return float(f(*[_eval(a, params) for a in node.args]))
    raise OracleError(f"unsupported expression element {ast.dump(node)}")


def number(value: Any, params: dict[str, float]) -> float:
    """A JSON number, or a string expression over `params`."""
    if isinstance(value, bool):
        raise OracleError(f"expected a number, got {value!r}")
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        # `^` is power, binding tighter than `* /` and right-associative with
        # a signed exponent, as `arris_debug::fixtures::expr` parses it:
        # Python's `**`. Python's own `^` is XOR, below `+ -`, so it is
        # rewritten before parsing; a `**` written in the recipe is refused.
        if "**" in value:
            raise OracleError(f"bad expression {value!r}: `**` is not power here, `^` is")
        try:
            tree = ast.parse(value.replace("^", "**"), mode="eval")
        except SyntaxError as e:
            raise OracleError(f"bad expression {value!r}: {e}") from e
        return _eval(tree, params)
    raise OracleError(f"expected a number or expression, got {value!r}")


def vector(value: Any, params: dict[str, float], n: int = 3) -> list[float]:
    if not isinstance(value, list) or len(value) != n:
        raise OracleError(f"expected a {n}-vector, got {value!r}")
    return [number(v, params) for v in value]


def resolve_params(fixture: dict, variant: str) -> dict[str, float]:
    """The params of `variant` ("default" is the base set)."""
    params = {k: float(v) for k, v in fixture.get("params", {}).items()}
    if variant != "default":
        variants = fixture.get("variants", {})
        if variant not in variants:
            raise OracleError(f"no variant {variant!r} in the recipe")
        params.update({k: float(v) for k, v in variants[variant].items()})
    return params


def variant_names(fixture: dict) -> list[str]:
    return ["default"] + sorted(fixture.get("variants", {}).keys())


def probes(fixture: dict, variant: str = "default") -> list[dict]:
    """The recipe's probes with their points evaluated under the variant's
    params: a probe's coordinates may be expressions like a step's
    (`tests/fixtures/README.md`)."""
    params = resolve_params(fixture, variant)
    return [{**p, "point": vector(p["point"], params)} for p in fixture.get("probes", [])]


SOLID_KEYS = ("params", "variants", "steps", "result", "probes")
PART_KEYS = ("kind", "file", "sha256", "battery")

# Where `fixture.load_fixture` records a recipe's directory, which a `step`
# operand's file is relative to: no recipe writes it, and no hash reads it.
DIR_KEY = "__dir__"
GEOMETRY_KEYS = ("kind", "params", "surfaces", "curves", "samples", "pairs")


def fixture_kind(fixture: dict) -> str:
    """`solid` (the default), `geometry` or `part`; anything else is an
    error."""
    kind = fixture.get("kind", "solid")
    if kind not in ("solid", "geometry", "part"):
        raise OracleError(f"unknown fixture kind {kind!r}")
    return kind


def recipe_hash(fixture: dict) -> str:
    """SHA-256 of the parts of the recipe the oracle evaluates, canonically
    encoded, so an edit to `analytic` or `description` does not stale the
    expected.json and an edit to a step does. The keys depend on the kind
    (`fixture_kind`); the Rust side hashes the same ones."""
    keys = {"geometry": GEOMETRY_KEYS, "part": PART_KEYS}.get(fixture_kind(fixture), SOLID_KEYS)
    evaluated = {k: fixture.get(k) for k in keys}
    return hashlib.sha256(canonical_json(evaluated).encode()).hexdigest()


def canonical_json(value: Any) -> str:
    """The encoding the Rust side (`arris_debug::fixtures`) hashes too:
    sorted keys, no whitespace, non-ASCII kept, and floats in the shortest
    round-trip form with the exponent's leading zeros dropped (`1e-9`,
    `1e+16`), which is what serde_json prints; Python's repr writes
    `1e-09`, and `5e-05` where serde_json writes `0.00005`."""
    if isinstance(value, bool) or value is None:
        return json.dumps(value)
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        text = repr(value)
        if "e" in text:
            mantissa, exp = text.split("e")
            if int(exp) == -5:
                # serde_json writes a decimal point down to 1e-5, where
                # repr has switched to an exponent below 1e-4.
                sign = "-" if mantissa.startswith("-") else ""
                return f"{sign}0.0000{mantissa.lstrip('-').replace('.', '')}"
            sign = "-" if exp.startswith("-") else "+"
            exp = exp.lstrip("+-").lstrip("0") or "0"
            text = f"{mantissa}e{sign}{exp}"
        return text
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False)
    if isinstance(value, list):
        return "[" + ",".join(canonical_json(v) for v in value) + "]"
    if isinstance(value, dict):
        return "{" + ",".join(f"{json.dumps(k, ensure_ascii=False)}:{canonical_json(v)}" for k, v in sorted(value.items())) + "}"
    raise OracleError(f"cannot canonicalise {value!r}")


# --- building ---------------------------------------------------------------


def _pnt(v: list[float]) -> gp_Pnt:
    return gp_Pnt(v[0], v[1], v[2])


def _dir(v: list[float]) -> gp_Dir:
    if math.hypot(*v) == 0.0:
        raise OracleError("zero direction vector")
    return gp_Dir(v[0], v[1], v[2])


def _checked(builder, what: str) -> TopoDS_Shape:
    # The MakeXxx builders are lazy: IsDone is meaningful only after Build.
    if hasattr(builder, "Build"):
        builder.Build()
    if not builder.IsDone():
        raise OracleError(f"{what}: Open CASCADE reports not done")
    return builder.Shape()


class _Plane:
    def __init__(self, spec: dict, params: dict[str, float]):
        self.origin = vector(spec["origin"], params)
        x = vector(spec["x"], params)
        y = vector(spec["y"], params)
        nx, ny = math.hypot(*x), math.hypot(*y)
        if nx == 0.0 or ny == 0.0:
            raise OracleError("profile plane axes must be non-zero")
        self.x = [c / nx for c in x]
        self.y = [c / ny for c in y]
        # Mirrors arris_math::Precision::DEFAULT.angular_tolerance
        # (Open CASCADE's Precision::Angular), the Rust side's named
        # tolerance for the same check (arris-debug's fixtures::geom).
        if abs(sum(a * b for a, b in zip(self.x, self.y))) > 1e-12:
            raise OracleError("profile plane axes must be orthogonal")
        self.z = [
            self.x[1] * self.y[2] - self.x[2] * self.y[1],
            self.x[2] * self.y[0] - self.x[0] * self.y[2],
            self.x[0] * self.y[1] - self.x[1] * self.y[0],
        ]

    def to3d(self, uv: list[float]) -> gp_Pnt:
        u, v = uv
        return gp_Pnt(*[self.origin[i] + u * self.x[i] + v * self.y[i] for i in range(3)])

    def ax3(self) -> gp_Ax3:
        return gp_Ax3(_pnt(self.origin), _dir(self.z), _dir(self.x))


def _elips(spec: dict, plane: _Plane, params: dict[str, float], what: str) -> gp_Elips:
    """The `gp_Elips` of an ellipse spec on the plane's normal: `major`
    from the centre to a major vertex, and when `minor_radius` is the
    longer one the axes swapped a quarter turn, as Arris's
    `Profile::edges` normalises it (ADR-0014)."""
    center = vector(spec["center"], params, 2)
    major = vector(spec["major"], params, 2)
    b = number(spec["minor_radius"], params)
    a = math.hypot(*major)
    if a <= 0.0 or b <= 0.0:
        raise OracleError(f"{what}: ellipse radii must be positive")
    ex = [major[0] / a, major[1] / a]
    if b > a:
        ex, a, b = [-ex[1], ex[0]], b, a
    x3 = [ex[0] * plane.x[i] + ex[1] * plane.y[i] for i in range(3)]
    return gp_Elips(gp_Ax2(plane.to3d(center), _dir(plane.z), _dir(x3)), a, b)


def _wire(loop: dict, plane: _Plane, params: dict[str, float]):
    if "ellipse" in loop:
        elips = _elips(loop["ellipse"], plane, params, "ellipse loop")
        edge = BRepBuilderAPI_MakeEdge(elips)
        wire = BRepBuilderAPI_MakeWire(TopoDS.Edge(_checked(edge, "ellipse edge")))
        return TopoDS.Wire(_checked(wire, "ellipse wire"))
    if "circle" in loop:
        c = loop["circle"]
        center = plane.to3d(vector(c["center"], params, 2))
        radius = number(c["radius"], params)
        if radius <= 0.0:
            raise OracleError("circle radius must be positive")
        circ = gp_Circ(gp_Ax2(center, _dir(plane.z), _dir(plane.x)), radius)
        edge = BRepBuilderAPI_MakeEdge(circ)
        wire = BRepBuilderAPI_MakeWire(TopoDS.Edge(_checked(edge, "circle edge")))
        return TopoDS.Wire(_checked(wire, "circle wire"))
    start = vector(loop["start"], params, 2)
    segments = loop.get("segments", [])
    if len(segments) < 2:
        raise OracleError("a loop needs at least two segments")
    wire = BRepBuilderAPI_MakeWire()
    current = start
    for i, seg in enumerate(segments):
        if "line_to" in seg:
            to = vector(seg["line_to"], params, 2)
            edge = BRepBuilderAPI_MakeEdge(plane.to3d(current), plane.to3d(to))
        elif "arc_to" in seg:
            to = vector(seg["arc_to"], params, 2)
            via = vector(seg["via"], params, 2)
            arc = GC_MakeArcOfCircle(plane.to3d(current), plane.to3d(via), plane.to3d(to))
            if not arc.IsDone():
                raise OracleError(f"segment {i}: three-point arc is degenerate")
            edge = BRepBuilderAPI_MakeEdge(arc.Value())
        elif "ellipse_to" in seg:
            to = vector(seg["ellipse_to"], params, 2)
            elips = _elips(seg, plane, params, f"segment {i}")
            # `sense` true runs with the ellipse's parametrisation, which
            # is counter-clockwise about the plane's normal.
            arc = GC_MakeArcOfEllipse(elips, plane.to3d(current), plane.to3d(to), bool(seg["ccw"]))
            if not arc.IsDone():
                raise OracleError(f"segment {i}: elliptic arc is degenerate")
            edge = BRepBuilderAPI_MakeEdge(arc.Value())
        else:
            raise OracleError(f"segment {i}: expected line_to, arc_to or ellipse_to")
        wire.Add(TopoDS.Edge(_checked(edge, f"segment {i}")))
        current = to
    if any(abs(a - b) > 1e-12 for a, b in zip(current, start)):
        raise OracleError(f"loop does not close: ends at {current}, started at {start}")
    return TopoDS.Wire(_checked(wire, "loop wire"))


def _profile(step: dict, params: dict[str, float]) -> TopoDS_Shape:
    plane = _Plane(step["plane"], params)
    outer = _wire(step["outer"], plane, params)
    face = BRepBuilderAPI_MakeFace(gp_Pln(plane.ax3()), outer, True)
    for hole in step.get("holes", []):
        face.Add(_wire(hole, plane, params))
    shape = _checked(face, "profile face")
    fix = ShapeFix_Face(TopoDS.Face(shape))
    fix.FixOrientation()
    return fix.Face()


def _polyhedron(step: dict, params: dict[str, float]) -> TopoDS_Shape:
    points = [vector(p, params) for p in step["points"]]
    faces = step["faces"]
    if not isinstance(faces, list) or not faces:
        raise OracleError("polyhedron needs a list of faces")
    sewing = BRepBuilderAPI_Sewing()
    for j, loops in enumerate(faces):
        if not loops or any(len(loop) < 3 for loop in loops):
            raise OracleError(f"polyhedron face {j}: a loop of fewer than three points, or none")
        wires = []
        for loop in loops:
            polygon = BRepBuilderAPI_MakePolygon()
            for i in loop:
                if not 0 <= i < len(points):
                    raise OracleError(f"polyhedron face {j} names point {i}, past the {len(points)} points")
                polygon.Add(_pnt(points[i]))
            polygon.Close()
            wires.append(TopoDS.Wire(_checked(polygon, f"polyhedron face {j} polygon")))
        # The plane from the outer loop, its normal the loop's turn; a
        # hole, clockwise, is already the way round the face wants it.
        face = BRepBuilderAPI_MakeFace(wires[0], True)
        for hole in wires[1:]:
            face.Add(hole)
        sewing.Add(_checked(face, f"polyhedron face {j}"))
    sewing.Perform()
    sewn = sewing.SewedShape()
    if sewn.ShapeType() != TopAbs_SHELL:
        raise OracleError("polyhedron faces do not sew into one shell")
    solid = _checked(BRepBuilderAPI_MakeSolid(TopoDS.Shell(sewn)), "polyhedron solid")
    solid = TopoDS.Solid(solid)
    if not BRepLib.OrientClosedSolid_s(solid):
        raise OracleError("polyhedron shell does not close")
    return solid


def build(fixture: dict, variant: str = "default") -> tuple[TopoDS_Shape, dict[str, TopoDS_Shape]]:
    """The result shape of `fixture` for `variant`, and every named step."""
    params = resolve_params(fixture, variant)
    probe = float(fixture.get("tolerances", {}).get("probe", 1e-7))
    base = fixture.get(DIR_KEY)
    shapes: dict[str, TopoDS_Shape] = {}

    def ref(name: Any) -> TopoDS_Shape:
        if name not in shapes:
            raise OracleError(f"step refers to {name!r}, which is not built yet")
        return shapes[name]

    for i, step in enumerate(fixture.get("steps", [])):
        name = step.get("name")
        op = step.get("op")
        if not name or not op:
            raise OracleError(f"step {i}: needs a name and an op")
        if name in shapes:
            raise OracleError(f"step {i}: name {name!r} is already used")
        try:
            shapes[name] = _build_step(step, op, params, ref, probe, base)
        except KeyError as e:
            raise OracleError(f"step {name!r} ({op}): missing field {e}") from e
    result = fixture.get("result")
    if result not in shapes:
        raise OracleError(f"result {result!r} is not a step")
    return shapes[result], shapes


def _edge_at(shape: TopoDS_Shape, point: list[float], probe: float):
    """The edge of `shape` nearest `point` by `BRepExtrema`, which must be
    the only edge within `probe` of it — the rule Arris's runner keeps on
    its side with `classify_point` (tests/fixtures/README.md)."""
    vertex = BRepBuilderAPI_MakeVertex(_pnt(point)).Vertex()
    edges = IndexedMapOfShape()
    TopExp.MapShapes_s(shape, TopAbs_EDGE, edges)
    near = []
    for i in range(1, edges.Extent() + 1):
        edge = TopoDS.Edge(edges.FindKey(i))
        d = BRepExtrema_DistShapeShape(edge, vertex)
        if d.IsDone() and d.Value() <= probe:
            near.append(edge)
    if len(near) != 1:
        raise OracleError(f"edge point {point} is within {probe} of {len(near)} edges, not one")
    return near[0]


def _face_at(shape: TopoDS_Shape, point: list[float], probe: float):
    """The face of `shape` that `point` lies on: the only face within
    `probe` of it by `BRepExtrema`, so a point on an edge (two faces) or
    off the body (none) is refused — Arris's runner keeps the rule with
    `classify_point` (tests/fixtures/README.md)."""
    vertex = BRepBuilderAPI_MakeVertex(_pnt(point)).Vertex()
    faces = IndexedMapOfShape()
    TopExp.MapShapes_s(shape, TopAbs_FACE, faces)
    near = []
    for i in range(1, faces.Extent() + 1):
        face = TopoDS.Face(faces.FindKey(i))
        d = BRepExtrema_DistShapeShape(face, vertex)
        if d.IsDone() and d.Value() <= probe:
            near.append(face)
    if len(near) != 1:
        raise OracleError(f"face point {point} is within {probe} of {len(near)} faces, not one")
    return near[0]


# Two faces whose outward normals at their edge's midpoint are within this
# sine of parallel are tangent there: a fillet's contact is exact to the
# fitting tolerance, a sharp edge is degrees away.
_TANGENT_SINE = 1e-6


def _outward(face: TopoDS_Shape, edge: TopoDS_Shape, t: float) -> gp_Vec:
    """The outward unit normal of `face` at parameter `t` of its `edge`."""
    uv = BRepAdaptor_Curve2d(edge, face).Value(t)
    props = BRepLProp_SLProps(BRepAdaptor_Surface(face), uv.X(), uv.Y(), 1, 1e-9)
    n = gp_Vec(props.Normal())
    return n.Reversed() if face.Orientation() == TopAbs_REVERSED else n


def _tangent_chain(shape: TopoDS_Shape, faces: list[TopoDS_Shape]) -> list[TopoDS_Shape]:
    """`faces` closed over tangent edges: every face that meets a listed
    one tangentially along an edge, and so on along the chain (ADR-0048
    §6)."""
    ancestors = IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(shape, TopAbs_EDGE, TopAbs_FACE, ancestors)
    across: list[tuple[TopoDS_Shape, TopoDS_Shape]] = []
    for i in range(1, ancestors.Extent() + 1):
        edge = TopoDS.Edge(ancestors.FindKey(i))
        pair = list(ancestors.FindFromIndex(i))
        if len(pair) != 2 or pair[0].IsSame(pair[1]) or BRep_Tool.Degenerated_s(edge):
            continue
        curve = BRepAdaptor_Curve(edge)
        t = 0.5 * (curve.FirstParameter() + curve.LastParameter())
        a, b = (TopoDS.Face(f) for f in pair)
        if _outward(a, edge, t).Crossed(_outward(b, edge, t)).Magnitude() <= _TANGENT_SINE:
            across.append((a, b))
    chain = list(faces)
    grown = True
    while grown:
        grown = False
        for a, b in across:
            has_a = any(a.IsSame(f) for f in chain)
            has_b = any(b.IsSame(f) for f in chain)
            if has_a != has_b:
                chain.append(b if has_a else a)
                grown = True
    return chain


def _make_offset(shape: TopoDS_Shape, faces: list[TopoDS_Shape], distance: float) -> BRepOffset_MakeOffset:
    mo = BRepOffset_MakeOffset()
    mo.Initialize(shape, 0.0, 1e-7, BRepOffset_Skin, True, False, GeomAbs_Intersection, False)
    for f in faces:
        mo.SetOffsetOnFace(f, distance)
    mo.MakeOffsetShape()
    return mo


def _offset_survives(shape: TopoDS_Shape, faces: list[TopoDS_Shape], distance: float) -> bool:
    """Whether Open CASCADE's offset finishes with a shape, tried in a
    forked child first: where a moved chain pulls clear of a fixed face
    its intersection join has nothing to meet and the process dies
    (ADR-0048 §6), which would take the whole batch with it."""
    pid = os.fork()
    if pid == 0:
        try:
            mo = _make_offset(shape, faces, distance)
            os._exit(0 if mo.IsDone() and not mo.Shape().IsNull() else 1)
        except BaseException:
            os._exit(1)
    _, status = os.waitpid(pid, 0)
    return os.WIFEXITED(status) and os.WEXITSTATUS(status) == 0


def _offset_faces(shape: TopoDS_Shape, faces: list[TopoDS_Shape], distance: float) -> TopoDS_Shape:
    """`shape` with `faces` moved by `distance` along their outward
    normals, with every face tangent to a moved one, and every other face
    held (offset 0), joined by intersection. Where Open CASCADE refuses or
    crashes it makes nothing, recorded as the empty compound: a result
    with no solid, which only a fixture that expects Arris's refusal
    accepts."""
    chain = _tangent_chain(shape, faces)
    if not _offset_survives(shape, chain, distance):
        empty = TopoDS_Compound()
        BRep_Builder().MakeCompound(empty)
        return empty
    mo = _make_offset(shape, chain, distance)
    result = mo.Shape()
    # Skin mode hands back the bare shell where the moved face sits among
    # concave neighbours (a pocket's floor): closed, it bounds the solid.
    if result.ShapeType() == TopAbs_SHELL:
        shell = TopoDS.Shell(result)
        if not BRep_Tool.IsClosed_s(shell):
            raise OracleError("offset: Open CASCADE's result is an open shell")
        result = _checked(BRepBuilderAPI_MakeSolid(shell), "offset solid")
    return result


def _read_solid(step: dict, params: dict[str, float], probe: float, base: str | None) -> TopoDS_Shape:
    """The solid a `step` operand names: of the file's solids from `#id`,
    the only one, or the one whose centroid is nearest `near`, a tie
    within `probe` refused — the rule the Rust side keeps with Arris's
    reader."""
    if base is None:
        raise OracleError("a step operand needs its fixture's directory")
    path = Path(base) / step["file"]
    if not path.exists():
        raise OracleError(f"no such STEP file: {path}")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != step["sha256"]:
        raise OracleError(f"{path} hashes to {digest}, not the recipe's {step['sha256']}")
    wanted = int(step["id"])
    candidates = [s for label, s in step_file.solids(path) if label == wanted]
    if not candidates:
        raise OracleError(f"{step['file']} has no solid #{wanted} that Open CASCADE reads")
    if len(candidates) == 1:
        return candidates[0]
    if "near" not in step:
        raise OracleError(f"{step['file']} places #{wanted} {len(candidates)} times: name one by `near`")
    near = vector(step["near"], params)

    def distance(solid) -> float:
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(solid, props)
        c = props.CentreOfMass()
        return math.dist((c.X(), c.Y(), c.Z()), near)

    ranked = sorted((distance(s), k) for k, s in enumerate(candidates))
    if ranked[1][0] - ranked[0][0] <= probe:
        raise OracleError(f"two placements of #{wanted} are as near {near}")
    return candidates[ranked[0][1]]


def _build_step(
    step: dict, op: str, params: dict[str, float], ref, probe: float = 1e-7, base: str | None = None
) -> TopoDS_Shape:
    if op == "box":
        lo, hi = vector(step["min"], params), vector(step["max"], params)
        if any(h <= l for l, h in zip(lo, hi)):
            raise OracleError("box max must exceed min on every axis")
        return _checked(BRepPrimAPI_MakeBox(_pnt(lo), _pnt(hi)), "box")
    if op == "cylinder":
        base, axis = vector(step["base"], params), vector(step["axis"], params)
        r, h = number(step["radius"], params), number(step["height"], params)
        if r <= 0.0 or h <= 0.0:
            raise OracleError("cylinder radius and height must be positive")
        return _checked(BRepPrimAPI_MakeCylinder(gp_Ax2(_pnt(base), _dir(axis)), r, h), "cylinder")
    if op == "profile":
        return _profile(step, params)
    if op == "extrude":
        d = vector(step["direction"], params)
        length = number(step["length"], params)
        n = math.hypot(*d)
        if n == 0.0 or length <= 0.0:
            raise OracleError("extrude needs a direction and a positive length")
        vec = gp_Vec(*[c / n * length for c in d])
        return _checked(BRepPrimAPI_MakePrism(ref(step["profile"]), vec), "extrude")
    if op == "revolve":
        ax = step["axis"]
        axis = gp_Ax1(_pnt(vector(ax["origin"], params)), _dir(vector(ax["direction"], params)))
        angle = math.radians(number(step["angle_deg"], params))
        if angle <= 0.0:
            raise OracleError("revolve angle must be positive")
        return _checked(BRepPrimAPI_MakeRevol(ref(step["profile"]), axis, angle), "revolve")
    if op == "transform":
        trsf = gp_Trsf()
        if "rotate" in step:
            rot = step["rotate"]
            r = gp_Trsf()
            r.SetRotation(
                gp_Ax1(
                    _pnt(vector(rot.get("origin", [0, 0, 0]), params)),
                    _dir(vector(rot["axis"], params)),
                ),
                math.radians(number(rot["angle_deg"], params)),
            )
            trsf = r
        if "translate" in step:
            t = gp_Trsf()
            t.SetTranslation(gp_Vec(*vector(step["translate"], params)))
            trsf = t.Multiplied(trsf)
        return _checked(BRepBuilderAPI_Transform(ref(step["of"]), trsf, True), "transform")
    if op == "mirror":
        plane = step["plane"]
        trsf = gp_Trsf()
        trsf.SetMirror(
            gp_Ax2(
                _pnt(vector(plane["origin"], params)),
                _dir(vector(plane["normal"], params)),
            )
        )
        return _checked(BRepBuilderAPI_Transform(ref(step["of"]), trsf, True), "mirror")
    if op == "fuse":
        return _checked(BRepAlgoAPI_Fuse(ref(step["a"]), ref(step["b"])), "fuse")
    if op == "common":
        return _checked(BRepAlgoAPI_Common(ref(step["a"]), ref(step["b"])), "common")
    if op == "cut":
        return _checked(BRepAlgoAPI_Cut(ref(step["target"]), ref(step["tool"])), "cut")
    if op == "fillet":
        shape = ref(step["of"])
        radius = number(step["radius"], params)
        if radius <= 0.0:
            raise OracleError("fillet radius must be positive")
        points = step["edges"]
        if not isinstance(points, list) or not points:
            raise OracleError("fillet needs a list of edge points")
        mf = BRepFilletAPI_MakeFillet(shape)
        for p in points:
            mf.Add(radius, _edge_at(shape, vector(p, params), probe))
        return _checked(mf, "fillet")
    if op == "chamfer":
        shape = ref(step["of"])
        distance = number(step["distance"], params)
        if distance <= 0.0:
            raise OracleError("chamfer distance must be positive")
        points = step["edges"]
        if not isinstance(points, list) or not points:
            raise OracleError("chamfer needs a list of edge points")
        mc = BRepFilletAPI_MakeChamfer(shape)
        for p in points:
            mc.Add(distance, _edge_at(shape, vector(p, params), probe))
        return _checked(mc, "chamfer")
    if op == "offset":
        shape = ref(step["of"])
        distance = number(step["distance"], params)
        if not math.isfinite(distance) or distance == 0.0:
            raise OracleError("offset distance must be finite and nonzero")
        points = step["faces"]
        if not isinstance(points, list) or not points:
            raise OracleError("offset needs a list of face points")
        faces = [_face_at(shape, vector(p, params), probe) for p in points]
        return _offset_faces(shape, faces, distance)
    if op == "polyhedron":
        return _polyhedron(step, params)
    if op == "step":
        return _read_solid(step, params, probe, base)
    raise OracleError(f"unknown op {op!r}")
