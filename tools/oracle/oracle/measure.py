"""Measure a shape: the numbers expected.json holds and compare.py checks.

    volume, area, centroid       GProp mass properties
    inertia                      the 3x3 inertia tensor about the centroid,
                                 unit density, physical convention (the
                                 products of inertia carried negated)
    counts                       unique vertices, edges (a seam once, a
                                 degenerate or an INTERNAL edge not at
                                 all), faces, loops (wires), shells, solids
    euler_characteristic         V − E + 2F − L: 2(S − G) for closed shells
    genus                        S − χ/2, what the fixture's analytic genus
                                 is checked against (the "Euler line");
                                 None for a non-manifold result, whose
                                 shared edge or vertex, or tangent contact
                                 carried as an edge of four faces, makes
                                 χ odd
    degenerate                   no solid in the result
    probes                       in / out / on for each probe point
"""

import math

import numpy

from OCP.BRep import BRep_Tool
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Curve2d, BRepAdaptor_Surface
from OCP.BRepTools import BRepTools_WireExplorer
from OCP.GCPnts import GCPnts_AbscissaPoint
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepGProp import BRepGProp
from OCP.GeomAbs import GeomAbs_CurveType, GeomAbs_SurfaceType
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt, gp_Pnt2d, gp_Vec, gp_Vec2d
from OCP.TopAbs import (
    TopAbs_EDGE,
    TopAbs_FACE,
    TopAbs_FORWARD,
    TopAbs_IN,
    TopAbs_ON,
    TopAbs_OUT,
    TopAbs_REVERSED,
    TopAbs_SHELL,
    TopAbs_SOLID,
    TopAbs_VERTEX,
    TopAbs_WIRE,
)
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape
from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as IndexedMapOfShape

from . import OracleError

# The tolerance the length of an extrusion face's basis arc is asked to:
# three orders below the corpus's `area_rel` (1e-9).
LENGTH_TOL = 1e-13

# How far a pcurve's `u` may move along an edge of a face on a surface of
# linear extrusion and still be a ruling: the pcurves of this corpus's
# extrusion faces are exactly axis-parallel, so this only absorbs the
# rounding of a boolean's own recomputation.
RULING_TOL = 1e-9

# The points a pcurve of an extrusion face is read at to see whether it is
# a ruling or an arc.
RULING_SAMPLES = 16

# The relative error the adaptive integration is asked for on a shape
# bounded by a spline edge (`_spline_bounded`): three orders below the
# corpus's `volume_rel` and `area_rel` (1e-9), as LENGTH_TOL is.
SPLINE_EPS = 1e-12

# 3D edge curves the fixed-order integration is exact beside.
_CONIC_EDGES = (
    GeomAbs_CurveType.GeomAbs_Line,
    GeomAbs_CurveType.GeomAbs_Circle,
    GeomAbs_CurveType.GeomAbs_Ellipse,
)

# Surfaces on which a conic has no exact 2D form, so a section that is not
# a circle of revolution or a ruling carries a fitted pcurve (ADR-0021).
_NO_EXACT_PCURVE = (
    GeomAbs_SurfaceType.GeomAbs_Cone,
    GeomAbs_SurfaceType.GeomAbs_Sphere,
    GeomAbs_SurfaceType.GeomAbs_Torus,
)

_ELEMENTARY = (
    GeomAbs_SurfaceType.GeomAbs_Plane,
    GeomAbs_SurfaceType.GeomAbs_Cylinder,
    GeomAbs_SurfaceType.GeomAbs_Cone,
    GeomAbs_SurfaceType.GeomAbs_Sphere,
    GeomAbs_SurfaceType.GeomAbs_Torus,
)


def _faces(shape: TopoDS_Shape):
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        yield TopoDS.Face(explorer.Current())
        explorer.Next()


def _spline_bounded(shape: TopoDS_Shape) -> bool:
    """Whether the shape is trimmed by a B-spline pcurve of many spans,
    over which the fixed-order integration is 1e-6 off in volume and
    area where the shape itself is right to 1e-10 — the adaptive
    overloads, asked for SPLINE_EPS, agree with a quadrature of the
    closed form to 1e-10. Such a shape is measured by those; every other
    keeps the fixed-order integration and its committed numbers.

    Three ways in. An edge whose 3D curve is no line and no conic: a
    B-spline, the approximation of a section two quadrics meet in (Open
    CASCADE's walked line, Arris's fitted one, ADR-0018). A cone, a
    sphere or a torus face trimmed by a B-spline pcurve under an exact
    conic edge, which is what a section of one carries where it is
    neither a circle of revolution nor a ruling (ADR-0021). And a shape
    with a surface-of-extrusion face, whose every face the fixed order
    is 2e-7 off on (`boolean/elliptic-operand-cut`'s planes, against
    their closed form)."""
    if _has_extrusion(shape):
        return True
    for face in _faces(shape):
        fitted = BRepAdaptor_Surface(face).GetType() in _NO_EXACT_PCURVE
        explorer = TopExp_Explorer(face, TopAbs_EDGE)
        while explorer.More():
            edge = TopoDS.Edge(explorer.Current())
            explorer.Next()
            if BRep_Tool.Degenerated_s(edge):
                continue
            if BRepAdaptor_Curve(edge).GetType() not in _CONIC_EDGES:
                return True
            if fitted and BRepAdaptor_Curve2d(edge, face).GetType() not in _CONIC_EDGES:
                return True
    return False


def _has_extrusion(shape: TopoDS_Shape) -> bool:
    """Whether a face of the shape lies on a `Geom_SurfaceOfLinearExtrusion`
    — an extruded elliptic profile segment, ADR-0014. Over such a face the
    area element is no polynomial in the parameters, and the plain
    adaptive integration is 1e-6 off in the inertia tensor of a shape
    whose closed forms are exact (`boolean/elliptic-operand-cut`); the
    Gauss–Kronrod overload with the span option agrees with them to
    1e-11, and is what such a shape's volume properties are taken by."""
    return any(
        BRepAdaptor_Surface(f).GetType() == GeomAbs_SurfaceType.GeomAbs_SurfaceOfExtrusion
        for f in _faces(shape)
    )


def _area(shape: TopoDS_Shape) -> float:
    """The total face area. Every face on a plane, cylinder, cone, sphere
    or torus: `BRepGProp::SurfaceProperties` over the whole shape, whose
    fixed-order integration is exact there, so the committed numbers
    stay bit for bit. A face on a `Geom_SurfaceOfLinearExtrusion` — an
    extruded elliptic profile segment, ADR-0014 — has an area element no
    polynomial in the parameters, which that integration is 2e-5 off on
    and the adaptive overload worse, and takes `_extrusion_area`. Any
    other face — a B-spline patch of a sample — takes the integration
    the whole-shape call would have given it, adaptive where the shape
    is spline-bounded."""
    faces = list(_faces(shape))
    spline = _spline_bounded(shape)
    if all(BRepAdaptor_Surface(f).GetType() in _ELEMENTARY for f in faces):
        sp = GProp_GProps()
        if spline:
            BRepGProp.SurfaceProperties_s(shape, sp, SPLINE_EPS)
        else:
            BRepGProp.SurfaceProperties_s(shape, sp)
        return sp.Mass()
    total = 0.0
    for face in faces:
        adaptor = BRepAdaptor_Surface(face)
        if adaptor.GetType() == GeomAbs_SurfaceType.GeomAbs_SurfaceOfExtrusion:
            total += _extrusion_area(face, adaptor)
        else:
            sp = GProp_GProps()
            if spline:
                BRepGProp.SurfaceProperties_s(face, sp, SPLINE_EPS)
            else:
                BRepGProp.SurfaceProperties_s(face, sp)
            total += sp.Mass()
    return total


def _arc_length(basis, u0: float, u: float) -> float:
    """The basis curve's arc length from `u0` to `u`, signed by the
    direction — `GCPnts_AbscissaPoint` gives the same positive length
    either way, and a face whose `u` range straddles the basis curve's
    own origin needs the two sides to tell apart."""
    if u == u0:
        return 0.0
    if u > u0:
        return GCPnts_AbscissaPoint.Length_s(basis, u0, u, LENGTH_TOL)
    return -GCPnts_AbscissaPoint.Length_s(basis, u, u0, LENGTH_TOL)


# Gauss-Legendre nodes and weights on [-1, 1] of the order the contour
# of a curved pcurve is integrated with, per panel.
_GAUSS_NODES, _GAUSS_WEIGHTS = (
    [float(x) for x in numpy.polynomial.legendre.leggauss(8)[0]],
    [float(w) for w in numpy.polynomial.legendre.leggauss(8)[1]],
)

# The agreement two successive halvings of the panels of a curved
# contour must reach, relative to a contour of at least one: one order
# below `area_rel` (1e-9). Open CASCADE's pcurve of a section is a
# B-spline whose own pieces meet with a jump in the second derivative,
# which the rule sees as a floor near 1e-11, so a tighter ask would
# refuse every such edge.
CONTOUR_TOL = 1e-10

# The most times the panels are halved before the contour is refused.
CONTOUR_MAX_HALVINGS = 12


def _smooth_length(basis, u0: float, u: float, panels: int = 8) -> float:
    """`_arc_length` by the same Gauss-Legendre rule as the contour it
    feeds, so that it is a smooth function of `u` where
    `GCPnts_AbscissaPoint`'s adaptive result carries its own tolerance as
    noise, which the contour integral's halvings would see as a
    disagreement. Exact to rounding for a conic over `panels` panels."""
    h = (u - u0) / panels
    total = 0.0
    for k in range(panels):
        mid = u0 + (k + 0.5) * h
        for x, w in zip(_GAUSS_NODES, _GAUSS_WEIGHTS):
            point, tangent = gp_Pnt(), gp_Vec()
            basis.D1(mid + 0.5 * h * x, point, tangent)
            total += 0.5 * h * w * tangent.Magnitude()
    return total


def _curved_contour(basis, u0: float, pcurve, reversed_edge: bool) -> float:
    """One edge's part of the contour integral of L(u) dv, where the
    pcurve is neither a ruling nor an arc: the composite Gauss-Legendre
    rule over the edge's own parameter, L the basis arc length from `u0`
    (`_arc_length`), dv the pcurve's `v'(t) dt`, the panels halved until
    two successive halvings agree to CONTOUR_TOL. An edge whose
    integral does not settle is refused, as it was before there was a
    quadrature: a wrong number is worse than none."""
    t0, t1 = pcurve.FirstParameter(), pcurve.LastParameter()

    def rule(panels: int) -> float:
        total = 0.0
        h = (t1 - t0) / panels
        for k in range(panels):
            mid = t0 + (k + 0.5) * h
            for x, w in zip(_GAUSS_NODES, _GAUSS_WEIGHTS):
                point, tangent = gp_Pnt2d(), gp_Vec2d()
                pcurve.D1(mid + 0.5 * h * x, point, tangent)
                total += 0.5 * h * w * _smooth_length(basis, u0, point.X()) * tangent.Y()
        return total

    panels, previous = 4, rule(4)
    for _ in range(CONTOUR_MAX_HALVINGS):
        panels *= 2
        current = rule(panels)
        if abs(current - previous) <= CONTOUR_TOL * max(1.0, abs(current)):
            return -current if reversed_edge else current
        previous = current
    raise OracleError(
        "a pcurve on a surface of linear extrusion whose contour integral "
        f"does not settle in {panels} panels"
    )


def _extrusion_area(face: TopoDS_Shape, adaptor: BRepAdaptor_Surface) -> float:
    """The area of one face on a `Geom_SurfaceOfLinearExtrusion`, whose
    (u, v) region a boolean need no longer leave a rectangle. The area
    element is the basis curve's speed alone — |C'(u) x d| = |C'(u)|,
    since every extrude in the grammar runs along the profile plane's
    normal — so the area is the integral of |C'(u)| over the region,
    which by Green's theorem is the contour integral of the basis arc
    length L(u) against dv. Every pcurve trimming such a face in this
    corpus is a straight line in (u, v): the other operand's planes are
    parallel to the extrusion (a ruling, u constant, contributing
    L(u)·Δv) or perpendicular to it (an arc, v constant, contributing
    nothing), so the integral is exact and a rectangle gives back
    (L(u2) − L(u1))·Δv, the arc-length rule itself. A curved pcurve
    there is `_curved_contour`'s quadrature."""
    basis = adaptor.BasisCurve()
    u0 = basis.FirstParameter()
    signed = 0.0
    explorer = TopExp_Explorer(face, TopAbs_WIRE)
    while explorer.More():
        wire = TopoDS.Wire(explorer.Current())
        explorer.Next()
        walker = BRepTools_WireExplorer(wire, TopoDS.Face(face))
        while walker.More():
            edge = walker.Current()
            walker.Next()
            pcurve = BRepAdaptor_Curve2d(edge, TopoDS.Face(face))
            t0, t1 = pcurve.FirstParameter(), pcurve.LastParameter()
            samples = [
                pcurve.Value(t0 + (t1 - t0) * i / RULING_SAMPLES)
                for i in range(RULING_SAMPLES + 1)
            ]
            us = [p.X() for p in samples]
            vs = [p.Y() for p in samples]
            if max(vs) - min(vs) <= RULING_TOL:
                continue
            if max(us) - min(us) > RULING_TOL:
                signed += _curved_contour(basis, u0, pcurve, edge.Orientation() == TopAbs_REVERSED)
                continue
            first, last = samples[0], samples[-1]
            if edge.Orientation() == TopAbs_REVERSED:
                first, last = last, first
            signed += _arc_length(basis, u0, first.X()) * (last.Y() - first.Y())
    return abs(signed)


DEFAULT_TOLERANCES = {
    "volume_rel": 1e-9,
    "area_rel": 1e-9,
    "centroid_abs": 1e-7,
    "probe": 1e-7,
    "inertia_rel": 1e-9,
}


def _count(shape: TopoDS_Shape, kind) -> int:
    m = IndexedMapOfShape()
    TopExp.MapShapes_s(shape, kind, m)
    return m.Extent()


def _count_boundary(shape: TopoDS_Shape) -> tuple[int, int, int]:
    """Unique vertices, edges and wires of the faces' boundaries. An edge
    oriented INTERNAL or EXTERNAL bounds nothing — Open CASCADE leaves a
    tangent contact it imprints as one, in a wire of its own, and its STEP
    writer drops it — so it, a wire of nothing else and a vertex on nothing
    else are left out, as a round trip would. A degenerate edge (a cone's
    apex, a sphere's pole) is a singular point of its surface, not a
    boundary between faces, so the Euler line leaves it out, as Arris's
    `Report::euler` does (docs/DATA-MODEL.md §Euler–Poincaré); its vertex
    is counted."""
    vertices, edges, wires = IndexedMapOfShape(), IndexedMapOfShape(), IndexedMapOfShape()
    wx = TopExp_Explorer(shape, TopAbs_WIRE)
    while wx.More():
        wire = wx.Current()
        bounds = False
        ex = TopExp_Explorer(wire, TopAbs_EDGE)
        while ex.More():
            edge = ex.Current()
            if edge.Orientation() in (TopAbs_FORWARD, TopAbs_REVERSED):
                bounds = True
                TopExp.MapShapes_s(edge, TopAbs_VERTEX, vertices)
                if not BRep_Tool.Degenerated_s(TopoDS.Edge(edge)):
                    edges.Add(edge)
            ex.Next()
        if bounds:
            wires.Add(wire)
        wx.Next()
    return vertices.Extent(), edges.Extent(), wires.Extent()


def solids(shape: TopoDS_Shape) -> list:
    out = []
    ex = TopExp_Explorer(shape, TopAbs_SOLID)
    while ex.More():
        out.append(TopoDS.Solid(ex.Current()))
        ex.Next()
    return out


def classify(shape: TopoDS_Shape, point: list[float], tolerance: float) -> str:
    """"in", "out" or "on" against the solids of `shape`: on if on any, in
    if in any, else out."""
    states = []
    for solid in solids(shape):
        c = BRepClass3d_SolidClassifier(solid)
        c.Perform(gp_Pnt(*point), tolerance)
        states.append(c.State())
    if TopAbs_ON in states:
        return "on"
    if TopAbs_IN in states:
        return "in"
    if all(s == TopAbs_OUT for s in states):
        return "out"
    raise OracleError(f"probe {point}: classifier returned UNKNOWN")


def inertia(props: GProp_GProps) -> list[list[float]]:
    """The 3x3 inertia tensor of `props` about the centre of mass, unit
    density, as rows. OCCT's `MatrixOfInertia` is already about the centre
    of mass and already in the physical convention -- the diagonal holds
    the moments and the off-diagonal the negated products -- which is the
    convention `arris_ops::measure::MassProperties` states."""
    m = props.MatrixOfInertia()
    return [[m.Value(i, j) for j in range(1, 4)] for i in range(1, 4)]


def measure(
    shape: TopoDS_Shape,
    probes: list[dict],
    probe_tolerance: float,
    manifold: bool = True,
    read: bool = False,
) -> dict:
    """Everything expected.json records for one result. `manifold` false
    admits an odd Euler characteristic — solids of a compound sharing an
    edge or a vertex, what a recipe expecting `non-manifold` builds, or a
    tangent contact carried as an edge of four faces, what one expecting
    `tangent-contact` may build — and records no genus for it; otherwise
    an odd one is an error.

    `read` true is a solid read from a part's file (ADR-0026). Where it is
    spline-bounded — as healing leaves every seamless face, its added
    seams' pcurves B-splines — its volume properties are taken by the
    Gauss–Kronrod overload, as an extrusion's are: the plain adaptive one
    is 1e-6 off in the inertia tensor of NIST's FTC-11, a solid of
    revolution whose Ixx and Iyy it splits by 3.3 in 1.6e6 and whose Ixy
    it makes −1.1, where the fixed-order and Gauss–Kronrod integrations
    both keep the symmetry to 1e-13. A recipe's spline-bounded result
    keeps the plain adaptive integration and its committed numbers."""
    vertices, edges, loops = _count_boundary(shape)
    counts = {
        "vertices": vertices,
        "edges": edges,
        "faces": _count(shape, TopAbs_FACE),
        "loops": loops,
        "shells": _count(shape, TopAbs_SHELL),
        "solids": _count(shape, TopAbs_SOLID),
    }
    degenerate = counts["solids"] == 0
    out: dict = {"degenerate": degenerate, "counts": counts}
    if degenerate:
        return out

    vp = GProp_GProps()
    if _has_extrusion(shape) or (read and _spline_bounded(shape)):
        BRepGProp.VolumePropertiesGK_s(shape, vp, SPLINE_EPS, False, True, True, True)
    elif _spline_bounded(shape):
        BRepGProp.VolumeProperties_s(shape, vp, SPLINE_EPS)
    else:
        BRepGProp.VolumeProperties_s(shape, vp)
    area = _area(shape)
    c = vp.CentreOfMass()
    chi = counts["vertices"] - counts["edges"] + 2 * counts["faces"] - counts["loops"]
    if chi % 2 != 0 and manifold:
        raise OracleError(f"Euler characteristic {chi} is odd: the shape is not a closed surface")
    out.update(
        {
            "volume": vp.Mass(),
            "area": area,
            "centroid": [c.X(), c.Y(), c.Z()],
            "inertia": inertia(vp),
            "euler_characteristic": chi,
            "genus": counts["shells"] - chi // 2 if chi % 2 == 0 else None,
            "probes": [
                {
                    "label": p.get("label", str(i)),
                    "point": [float(x) for x in p["point"]],
                    "class": classify(shape, [float(x) for x in p["point"]], probe_tolerance),
                }
                for i, p in enumerate(probes)
            ],
        }
    )
    for k in ("volume", "area"):
        if not math.isfinite(out[k]):
            raise OracleError(f"{k} is not finite")
    return out


def compare(expected: dict, actual: dict, tolerances: dict) -> list[tuple[str, str, str, bool]]:
    """Rows of (quantity, expected, actual, ok). Counts and classes exact,
    volume, area and each inertia component relative, centroid absolute."""
    tol = {**DEFAULT_TOLERANCES, **tolerances}
    rows: list[tuple[str, str, str, bool]] = []

    def row(name, e, a, ok):
        rows.append((name, str(e), str(a), bool(ok)))

    row("degenerate", expected["degenerate"], actual["degenerate"], expected["degenerate"] == actual["degenerate"])
    for k in ("vertices", "edges", "faces", "loops", "shells", "solids"):
        e, a = expected["counts"][k], actual["counts"][k]
        row(f"counts.{k}", e, a, e == a)
    if expected["degenerate"] or actual["degenerate"]:
        return rows
    for k, t in (("volume", tol["volume_rel"]), ("area", tol["area_rel"])):
        e, a = expected[k], actual[k]
        row(k, repr(e), repr(a), abs(a - e) <= t * max(abs(e), abs(a), 1e-300))
    e, a = expected["centroid"], actual["centroid"]
    dist = math.dist(e, a)
    row("centroid", [f"{x:.9g}" for x in e], [f"{x:.9g}" for x in a], dist <= tol["centroid_abs"])
    ei, ai = expected.get("inertia"), actual.get("inertia")
    if ei is not None and ai is not None:
        # Relative to the largest component of the tensor: a product of
        # inertia that cancels to zero is not compared against itself.
        scale = max(abs(x) for row_ in ei for x in row_) or 1.0
        for i in range(3):
            for j in range(3):
                row(
                    f"inertia[{i}][{j}]",
                    repr(ei[i][j]),
                    repr(ai[i][j]),
                    abs(ai[i][j] - ei[i][j]) <= tol["inertia_rel"] * scale,
                )
    row("genus", expected["genus"], actual["genus"], expected["genus"] == actual["genus"])
    ea = {p["label"]: p["class"] for p in expected.get("probes", [])}
    aa = {p["label"]: p["class"] for p in actual.get("probes", [])}
    for label in sorted(set(ea) | set(aa)):
        row(f"probe.{label}", ea.get(label), aa.get(label), ea.get(label) == aa.get(label))
    return rows


def format_table(rows: list[tuple[str, str, str, bool]]) -> str:
    w0 = max(len(r[0]) for r in rows) if rows else 8
    w1 = max(len(r[1]) for r in rows) if rows else 8
    w2 = max(len(r[2]) for r in rows) if rows else 8
    lines = [f"{'quantity':<{w0}}  {'expected':<{w1}}  {'actual':<{w2}}  ok"]
    for name, e, a, ok in rows:
        lines.append(f"{name:<{w0}}  {e:<{w1}}  {a:<{w2}}  {'ok' if ok else 'MISMATCH'}")
    return "\n".join(lines)


def tolerance_of(shape: TopoDS_Shape) -> float:
    """The largest vertex tolerance: what "on" means for this shape."""
    t = 0.0
    ex = TopExp_Explorer(shape, TopAbs_VERTEX)
    while ex.More():
        t = max(t, BRep_Tool.Tolerance_s(TopoDS.Vertex(ex.Current())))
        ex.Next()
    return t


def own_measures(shape: TopoDS_Shape, measured: dict) -> dict | None:
    """What the first-order bound of `within_own_tolerance` is taken
    over: the shape's largest vertex tolerance `tolerance`, the total
    length of its edges `edge_length`, and `reach`, the farthest corner of
    its bounding box from the centroid; and its `removable_vertices`. None
    for a result with no solid or no tolerance. The differential (`expected.py --own`) records these
    so that Arris can add its own tolerance to the oracle's before it
    compares the two (ADR-0024 §2)."""
    t = tolerance_of(shape)
    if measured["degenerate"] or t == 0.0:
        return None
    props = GProp_GProps()
    BRepGProp.LinearProperties_s(shape, props)
    c = measured["centroid"]
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    lo, hi = box.CornerMin(), box.CornerMax()
    reach = max(math.dist(c, (x, y, z)) for x in (lo.X(), hi.X()) for y in (lo.Y(), hi.Y()) for z in (lo.Z(), hi.Z()))
    return {"tolerance": t, "edge_length": props.Mass(), "reach": reach, "removable_vertices": removable_vertices(shape)}


def removable_vertices(shape: TopoDS_Shape) -> int:
    """The vertices that split one edge between the same two faces into
    two: exactly two distinct edges meet there, neither closed nor
    degenerate, and both lie on the same two faces. Removing one merges
    two edges and changes no face, loop or shell, so two B-reps of one
    solid may differ by such vertices alone. Open CASCADE keeps one where
    a section crossed an operand's seam; Arris merges it. The differential
    compares counts net of them (ADR-0024 §2)."""
    vm = IndexedMapOfShape()
    em = IndexedMapOfShape()
    fm = IndexedMapOfShape()
    TopExp.MapShapes_s(shape, TopAbs_VERTEX, vm)
    TopExp.MapShapes_s(shape, TopAbs_EDGE, em)
    TopExp.MapShapes_s(shape, TopAbs_FACE, fm)
    faces_of: dict[int, set[int]] = {}
    for fi in range(1, fm.Extent() + 1):
        explorer = TopExp_Explorer(fm.FindKey(fi), TopAbs_EDGE)
        while explorer.More():
            faces_of.setdefault(em.FindIndex(explorer.Current()), set()).add(fi)
            explorer.Next()
    edges_at: dict[int, list[int]] = {}
    for ei in range(1, em.Extent() + 1):
        edge = TopoDS.Edge(em.FindKey(ei))
        if BRep_Tool.Degenerated_s(edge):
            continue
        for v in (TopExp.FirstVertex_s(edge), TopExp.LastVertex_s(edge)):
            edges_at.setdefault(vm.FindIndex(v), []).append(ei)
    return sum(
        1
        for es in edges_at.values()
        if len(es) == 2 and es[0] != es[1] and len(faces_of.get(es[0], ())) == 2 and faces_of.get(es[0]) == faces_of.get(es[1])
    )


def within_own_tolerance(tolerances: dict, shape: TopoDS_Shape, measured: dict) -> dict:
    """`tolerances` widened to what `shape` itself declares: a reader may
    move any point of the boundary by up to its largest vertex tolerance
    `t` and still hand back the same shape, so a STEP round trip of it is
    held to the first-order change such a move makes and no closer. Over a
    boundary of area `A` and volume `V`, edges of total length `L`, and
    the corners of its bounding box within `R` of the centroid
    (`own_measures`), that is `A·t` of volume, `L·t` of area (the edges
    moved across their faces), `A·t·R / V` of centroid and `A·t·R²` of
    each inertia component. Counts, genus and probe classes are not
    widened: a move within the tolerance changes none of them."""
    m = own_measures(shape, measured)
    if m is None:
        return dict(tolerances)
    t, length, reach = m["tolerance"], m["edge_length"], m["reach"]
    volume, area = abs(measured["volume"]), measured["area"]
    scale = max((abs(x) for row in measured.get("inertia") or [[0.0]] for x in row), default=0.0) or 1.0
    own = {
        "volume_rel": area * t / volume,
        "area_rel": length * t / area,
        "centroid_abs": area * t * reach / volume,
        "inertia_rel": area * t * reach**2 / scale,
    }
    return {k: max(v, own.get(k, 0.0)) for k, v in tolerances.items()}
