#!/usr/bin/env python3
"""Hold the cusp corner's laws against Open CASCADE and print the record.

Run with `uv run --project tools/oracle tools/oracle/cusp_corner_law.py`.
The record is `tests/fixtures/regression/cusp-corner-crescent-fillet/law.json`
(ADR-0084). For each `regression/cusp-corner-*` fixture at three sizes:

- a fillet's patch: the oracle's corners against ADR-0084's (the end
  ball's arcs, the walls' (u, v)-straight curves to the cusp), how far its
  boundary lies from those curves, its own edge tolerance, the angle it
  makes with each neighbour along each edge, the range of its outward
  normal's `z` over interior samples (one sign: a height graph over the
  shared plane), and the bound its tolerance puts on the measures (patch
  area times tolerance of volume, boundary length times tolerance of
  area);
- the rejected square fill (ADR-0084 §Alternatives: the bilinear Coons
  patch on the corners C, A1, T, A2, corrected G1 to the ball along the
  arcs) against the oracle's patch: normal distance at interior samples,
  cone volume from the origin and area, and the worst cosine between
  neighbouring normals on a grid graded into the cusp corner (negative:
  the fill folds);
- a chamfer: the two chamfer faces' shared edge, against the section law's
  ends `(Q_d, h)` and `(cusp, h - σd)`, and whether the cusp vertex at the
  shared face survives;
- the miter (each stripe run on to the other's: the lone blends' common,
  or fuse where they add material): its volume and area beside the
  oracle's two-edge result.

The square fill is a reference evaluator in plain numpy, independent of
the oracle's surface. Observations, not an implementation.
"""
import copy
import json
import math

import numpy as np

from oracle import occt_version, require_ocp

require_ocp()
from OCP.BRep import BRep_Tool  # noqa: E402
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface  # noqa: E402
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Fuse  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402
from OCP.BRepClass import BRepClass_FaceClassifier  # noqa: E402
from OCP.BRepGProp import BRepGProp, BRepGProp_Domain, BRepGProp_Face, BRepGProp_Vinert  # noqa: E402
from OCP.GeomAbs import GeomAbs_BSplineSurface  # noqa: E402
from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf  # noqa: E402
from OCP.GeomLProp import GeomLProp_SLProps  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_IN, TopAbs_REVERSED, TopAbs_VERTEX  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402
from OCP.gp import gp_Pnt, gp_Pnt2d  # noqa: E402
from oracle.fixture import corpus_root, load_fixture  # noqa: E402
from oracle.recipe import build  # noqa: E402

SIZES = (0.02, 0.1, 0.3)
# Interior samples: an N×N grid over the oracle patch's (u, v) rectangle,
# kept where the face classifier says inside the trimmed face.
GRID = 30
# Gauss–Legendre points per direction for the law's volume and area.
GAUSS = 48
# Finite-difference step in the law's (u, v); the law is smooth inside.
STEP = 1e-6


def explore(shape, kind, cast):
    e = TopExp_Explorer(shape, kind)
    while e.More():
        yield cast(e.Current())
        e.Next()


def setting(name: str, r: float):
    """The fixture's two walls in the shared plane's (x, y), the cusp, the
    end ball's centre `q` (where the walls' offsets by `r` meet), the shared
    plane's height `h` and `sigma`, +1 where the blend is below it (convex,
    material under the plane) and -1 above (a pocket floor)."""
    if name.startswith(("crescent", "pocket")):
        x = ((1 + r) ** 2 - (2 - r) ** 2 - 1) / 2
        q = np.array([x, -math.sqrt((2 - r) ** 2 - x * x)])
        walls = [("circle", np.array([0.0, 0.0]), 2.0), ("circle", np.array([-1.0, 0.0]), 1.0)]
        return walls, np.array([-2.0, 0.0]), q, 1.0, 1 if name.startswith("crescent") else -1
    q = np.array([4 * math.sqrt(r), r])
    walls = [("line", np.array([0.0, 0.0]), np.array([1.0, 0.0])), ("circle", np.array([0.0, 4.0]), 4.0)]
    return walls, np.array([0.0, 0.0]), q, 1.0, 1


def foot(wall, p):
    if wall[0] == "circle":
        d = p - wall[1]
        return wall[1] + wall[2] * d / np.linalg.norm(d)
    return wall[1] + wall[2] * ((p - wall[1]) @ wall[2])


def along(wall, a, c, s):
    """The wall's point a fraction `s` of the way from `a` to `c`, linear in
    the wall's own parameter (angle on a circle, length on a line)."""
    if wall[0] == "circle":
        ta = math.atan2(a[1] - wall[1][1], a[0] - wall[1][0])
        tc = math.atan2(c[1] - wall[1][1], c[0] - wall[1][0])
        t = ta + ((tc - ta + math.pi) % (2 * math.pi) - math.pi) * s
        return wall[1] + wall[2] * np.array([math.cos(t), math.sin(t)])
    return a + (c - a) * s


class Law:
    """The rejected square fill: corners C (the cusp at h), A1, T, A2 of the
    unit square at (0,0), (1,0), (1,1), (0,1). Sides: v = 0 wall 1 from C to
    A1, u = 1 the end ball's arc A1 → T, v = 1 its arc A2 → T, u = 0 wall 2
    from C to A2. The bilinear Coons patch of the four sides, corrected along
    each arc by h(u) = u²(u − 1) times the gap between its cross derivative
    and that derivative's projection onto the ball's tangent plane, less the
    Gregory twist at T: G1 to the ball along both arcs, the walls' cross
    derivatives the bilinear patch's own."""

    def __init__(self, name: str, r: float):
        walls, cusp, q, h, sigma = setting(name, r)
        self.r, self.h, self.sigma, self.walls, self.cusp2 = r, h, sigma, walls, cusp
        self.centre = np.array([*q, h - sigma * r])
        self.a = [np.array([*foot(w, q), h - sigma * r]) for w in walls]
        self.t = np.array([*q, h])
        self.c = np.array([*cusp, h])
        e = 1e-5
        self.gu1 = (self.gap_u(1.0) - self.gap_u(1.0 - e)) / e
        self.gv1 = (self.gap_v(1.0) - self.gap_v(1.0 - e)) / e

    def wall(self, i: int, s: float):
        """Wall i's boundary from A_i (s = 0) to the cusp C (s = 1): straight
        in the wall's (u, v), its height h − σr(1 − s)."""
        p = along(self.walls[i], self.a[i][:2], self.cusp2, s)
        return np.array([*p, self.h - self.sigma * self.r * (1 - s)])

    def arc(self, i: int, t: float):
        """The end ball's great arc from A_i (t = 0) to T (t = 1)."""
        a, b = self.a[i] - self.centre, self.t - self.centre
        om = math.acos(float(np.clip(a @ b / (self.r * self.r), -1, 1)))
        return self.centre + (math.sin((1 - t) * om) * a + math.sin(t * om) * b) / math.sin(om)

    def coons(self, u: float, v: float):
        b, tp = self.wall(0, 1 - u), self.arc(1, u)
        lf, rt = self.wall(1, 1 - v), self.arc(0, v)
        corners = (1 - u) * (1 - v) * self.c + u * (1 - v) * self.a[0] + u * v * self.t + (1 - u) * v * self.a[1]
        return (1 - v) * b + v * tp + (1 - u) * lf + u * rt - corners

    def tangent(self, d, p):
        n = (p - self.centre) / np.linalg.norm(p - self.centre)
        return d - (d @ n) * n

    def gap_u(self, v: float):
        """Along arc 1 (u = 1): the bilinear patch's cross derivative less
        its projection onto the ball's tangent plane. Zero at both ends."""
        d = (self.coons(1, v) - self.coons(1 - STEP, v)) / STEP
        return self.tangent(d, self.arc(0, v)) - d

    def gap_v(self, u: float):
        d = (self.coons(u, 1) - self.coons(u, 1 - STEP)) / STEP
        return self.tangent(d, self.arc(1, u)) - d

    def __call__(self, u: float, v: float):
        h = lambda x: x * x * (x - 1)  # noqa: E731  h(0) = h(1) = h'(0) = 0, h'(1) = 1
        w = (1 - u) + (1 - v)
        twist = ((1 - u) * self.gu1 + (1 - v) * self.gv1) / w if w > 0 else (self.gu1 + self.gv1) / 2
        return (self.coons(u, v) + h(u) * self.gap_u(v) + h(v) * self.gap_v(u)
                - h(u) * h(v) * twist)

    def frame(self, u: float, v: float):
        uu, vv = min(max(u, STEP), 1 - STEP), min(max(v, STEP), 1 - STEP)
        su = (self(uu + STEP, vv) - self(uu - STEP, vv)) / (2 * STEP)
        sv = (self(uu, vv + STEP) - self(uu, vv - STEP)) / (2 * STEP)
        return su, sv

    def measures(self):
        """Cone volume from the origin and area, by Gauss–Legendre."""
        x, w = np.polynomial.legendre.leggauss(GAUSS)
        x, w = (x + 1) / 2, w / 2
        vol = area = 0.0
        for i in range(GAUSS):
            for j in range(GAUSS):
                su, sv = self.frame(x[i], x[j])
                n = np.cross(su, sv)
                vol += w[i] * w[j] * (self(x[i], x[j]) @ n) / 3
                area += w[i] * w[j] * np.linalg.norm(n)
        return vol, area

    def folds(self):
        """The least cosine between neighbouring unit normals on a grid
        graded geometrically into the cusp corner (0, 0)."""
        ticks = np.concatenate([np.geomspace(1e-4, 0.05, 12), np.linspace(0.06, 1, 30)])
        normals = {}
        for i, u in enumerate(ticks):
            for j, v in enumerate(ticks):
                n = np.cross(*self.frame(u, v))
                normals[(i, j)] = n / np.linalg.norm(n)
        return min(float(n @ normals[k]) for (i, j), n in normals.items()
                   for k in ((i + 1, j), (i, j + 1)) if k in normals)

    def project(self, p, grid):
        pts, uvs = grid
        u, v = uvs[int(np.argmin(np.linalg.norm(pts - p, axis=1)))]
        for _ in range(40):
            su, sv = self.frame(u, v)
            step = np.linalg.lstsq(np.array([su, sv]).T, p - self(u, v), rcond=None)[0]
            u, v = min(max(u + step[0], 0.0), 1.0), min(max(v + step[1], 0.0), 1.0)
            if np.linalg.norm(step) < 1e-13:
                break
        su, sv = self.frame(u, v)
        n = np.cross(su, sv)
        return (p - self(u, v)) @ (n / np.linalg.norm(n)), n


def point(p):
    return np.array([p.X(), p.Y(), p.Z()])


def face_normal(face, p):
    surface = BRep_Tool.Surface_s(face)
    u, v = GeomAPI_ProjectPointOnSurf(gp_Pnt(*p), surface).LowerDistanceParameters()
    props = GeomLProp_SLProps(surface, u, v, 1, 1e-12)
    if not props.IsNormalDefined():
        return None
    n = point(props.Normal())
    return -n if face.Orientation() == TopAbs_REVERSED else n


def volume_area(shape):
    v, a = GProp_GProps(), GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, v)
    BRepGProp.SurfaceProperties_s(shape, a)
    return v.Mass(), a.Mass()


def miter(fixture, both_volume, both_area, concave):
    """The lone blends of each edge, met: their common where the blend
    removes material, their fuse where it adds."""
    lone = []
    for k in (0, 1):
        g = copy.deepcopy(fixture)
        g["steps"][-1]["edges"] = [fixture["steps"][-1]["edges"][k]]
        lone.append(build(g, "default", [])[0])
    met = (BRepAlgoAPI_Fuse if concave else BRepAlgoAPI_Common)(lone[0], lone[1]).Shape()
    v, a = volume_area(met)
    faces = sum(1 for _ in explore(met, TopAbs_FACE, TopoDS.Face))
    return {"valid": BRepCheck_Analyzer(met).IsValid(), "faces": faces,
            "volume_rel": (v - both_volume) / both_volume, "area_rel": (a - both_area) / both_area}


def fillet_record(name, r, shape, fixture):
    law = Law(name, r)
    patch = next(f for f in explore(shape, TopAbs_FACE, TopoDS.Face)
                 if BRepAdaptor_Surface(f).GetType() == GeomAbs_BSplineSurface)
    faces = list(explore(shape, TopAbs_FACE, TopoDS.Face))
    corners = {tuple(np.round(point(BRep_Tool.Pnt_s(v)), 12)) for v in explore(patch, TopAbs_VERTEX, TopoDS.Vertex)}
    expected = [law.c, law.a[0], law.t, law.a[1]]
    corner_error = max(min(np.linalg.norm(np.array(c) - e) for c in corners) for e in expected)
    edges, tolerance, length = [], 0.0, 0.0
    for edge in explore(patch, TopAbs_EDGE, TopoDS.Edge):
        other = next(f for f in faces if not f.IsSame(patch)
                     and any(x.IsSame(edge) for x in explore(f, TopAbs_EDGE, TopoDS.Edge)))
        curve = BRepAdaptor_Curve(edge)
        lo, hi = curve.FirstParameter(), curve.LastParameter()
        props = GProp_GProps()
        BRepGProp.LinearProperties_s(edge, props)
        length += props.Mass()
        tolerance = max(tolerance, BRep_Tool.Tolerance_s(edge))
        kind = str(BRepAdaptor_Surface(other).GetType()).split("_")[-1]
        angles, off_law = [], 0.0
        ends = [point(curve.Value(lo)), point(curve.Value(hi))]
        stripe = min(np.linalg.norm(e - law.t) for e in ends) < 1e-9
        samples = [point(curve.Value(lo + (hi - lo) * t)) for t in np.linspace(0.02, 0.98, 9)]
        for p in samples:
            n1, n2 = face_normal(patch, p), face_normal(other, p)
            angles.append(math.degrees(math.acos(float(np.clip(n1 @ n2, -1, 1)))))
            if stripe:
                off_law = max(off_law, abs(np.linalg.norm(p - law.centre) - r))
        if not stripe:
            i = int(np.argmin([np.linalg.norm(samples[4][:2] - foot(w, samples[4][:2])) for w in law.walls]))
            for p in samples:
                best = min(np.linalg.norm(p - law.wall(i, s)) for s in np.linspace(0, 1, 2001))
                off_law = max(off_law, best)
        edges.append({"neighbour": ("stripe " if stripe else "wall ") + kind,
                      "max_angle_deg": max(angles), "off_law": off_law})
    bs = BRepAdaptor_Surface(patch)
    u0, u1, v0, v1 = bs.FirstUParameter(), bs.LastUParameter(), bs.FirstVParameter(), bs.LastVParameter()
    surface = BRep_Tool.Surface_s(patch)
    uvs = [(i / 60, j / 60) for i in range(61) for j in range(61)]
    grid = (np.array([law(*q) for q in uvs]), uvs)
    devs, sign, nz = [], None, []
    for i in range(1, GRID):
        for j in range(1, GRID):
            u, v = u0 + (u1 - u0) * i / GRID, v0 + (v1 - v0) * j / GRID
            if BRepClass_FaceClassifier(patch, gp_Pnt2d(u, v), 1e-9).State() != TopAbs_IN:
                continue
            p = point(bs.Value(u, v))
            n = point(GeomLProp_SLProps(surface, u, v, 1, 1e-12).Normal())
            n = -n if patch.Orientation() == TopAbs_REVERSED else n
            nz.append(float(n[2]))
            d, ln = law.project(p, grid)
            s = 1 if ln @ n > 0 else -1
            sign = s if sign is None else sign
            devs.append(d * sign)
    devs = np.array(devs)
    vol, area = law.measures()
    vol *= sign
    cone = BRepGProp_Vinert(BRepGProp_Face(patch), BRepGProp_Domain(patch), gp_Pnt(0, 0, 0), gp_Pnt(0, 0, 0), 1e-12)
    patch_area = GProp_GProps()
    BRepGProp.SurfaceProperties_s(patch, patch_area)
    body_volume, body_area = volume_area(shape)
    return {
        "corner_error": corner_error,
        "patch_tolerance": tolerance,
        "edges": edges,
        "interior": {"samples": len(devs), "max_dev_over_r": float(np.abs(devs).max() / r),
                     "mean_dev_over_r": float(devs.mean() / r)},
        "outward_nz": [min(nz), max(nz)],
        "square_fill": {"volume_rel": (vol - cone.Mass()) / body_volume,
                        "area_rel": (area - patch_area.Mass()) / body_area,
                        "fold_min_cos": law.folds()},
        "own_bound": {"volume_rel": patch_area.Mass() * tolerance / body_volume,
                      "area_rel": length * tolerance / body_area},
        "miter": miter(fixture, body_volume, body_area, law.sigma < 0),
    }


def chamfer_record(name, d, shape):
    # q: where the walls' offsets by d meet on the shared plane, the top contacts' meeting.
    _, cusp, q, h, sigma = setting(name, d)
    blends = [f for f in explore(shape, TopAbs_FACE, TopoDS.Face)
              if str(BRepAdaptor_Surface(f).GetType()).split("_")[-1] in ("Cone", "Plane")]
    shared = []
    for i, f in enumerate(blends):
        for g in blends[i + 1:]:
            for edge in explore(f, TopAbs_EDGE, TopoDS.Edge):
                if any(edge.IsSame(x) for x in explore(g, TopAbs_EDGE, TopoDS.Edge)):
                    curve = BRepAdaptor_Curve(edge)
                    ends = sorted([point(curve.Value(curve.FirstParameter())), point(curve.Value(curve.LastParameter()))],
                                  key=lambda p: p[2] * sigma)
                    shared.append((ends, BRep_Tool.Tolerance_s(edge), str(curve.GetType()).split("_")[-1]))
    want = [np.array([*cusp, h - sigma * d]), np.array([*q, h])]
    hits = [(s, t, k) for s, t, k in shared if np.linalg.norm(s[0] - want[0]) < 1e-6 and np.linalg.norm(s[1] - want[1]) < 1e-6]
    if len(hits) != 1:
        raise RuntimeError(f"{name} at {d}: no shared chamfer edge from the cusp at h - σd to q_d")
    _, tol, kind = hits[0]
    top_cusp = any(np.linalg.norm(point(BRep_Tool.Pnt_s(v)) - np.array([*cusp, h])) < 1e-9
                   for v in explore(shape, TopAbs_VERTEX, TopoDS.Vertex))
    return {"section_edge": kind, "section_tolerance": tol, "cusp_vertex_at_h": top_cusp}


def records():
    out = []
    for directory in sorted((corpus_root() / "regression").glob("cusp-corner-*")):
        name = directory.name.removeprefix("cusp-corner-")
        for size in SIZES:
            fixture = load_fixture(directory)
            fixture["params"]["r"] = size
            shape, _ = build(fixture, "default", [])
            record = {"fixture": directory.name, "size": size}
            if name.endswith("fillet"):
                record.update(fillet_record(name, size, shape, fixture))
            else:
                record.update(chamfer_record(name, size, shape))
            out.append(record)
    return {"occt": occt_version(), "records": out}


def rounded(x):
    if isinstance(x, float):
        return float(f"{x:.4g}")
    if isinstance(x, dict):
        return {k: rounded(v) for k, v in x.items()}
    if isinstance(x, list):
        return [rounded(v) for v in x]
    return x


if __name__ == "__main__":
    print(json.dumps(rounded(records()), indent=2))
