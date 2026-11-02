#!/usr/bin/env python3
"""Hold candidate interiors of the cusp corner patch to the oracle's bound.

Run with `uv run --project tools/oracle tools/oracle/cusp_corner_graph.py`.
The record is `tests/fixtures/regression/cusp-corner-crescent-fillet/graph.json`
(ADR-0084 §5 and §6, plan `cusp-corner-patch` step 2). For each
`regression/cusp-corner-*-fillet` fixture at three sizes:

- the oracle's patch as the body sees it: its area, its cone volume from
  the origin under the body's outward normal, its edge tolerance and
  boundary length, the body's volume and area, and the oracle's own bound
  on the measures (patch area times tolerance of volume, boundary length
  times it of area, ADR-0084 §Context);
- ADR-0084 §5's least-bending height graph over the shared plane, solved
  here in plain numpy at two resolutions (a cubic tensor B-spline over a
  rectangle covering `D`, knots graded into the end ball's feet and the
  cusp; bending energy over `D` by Gauss points per knot span kept inside it; the boundary
  heights and the zero slope across the two segments by penalty): its
  cone volume and area against the oracle's in units of that bound, its
  largest miss of the boundary heights, and its height less the oracle's
  at the oracle's interior samples;
- the natural G1 alternative over a chart that does not fold: the ruled
  chart of `D` (the walls joined at equal fractions of their straight
  curves, the cusp a collapsed side, the polyline `F_1 Q F_2` the far
  side), heights the walls' plus the end ball's bump carried along by
  cubic Hermite blending with the ball's own cross slope: the same
  measures.

A reference evaluation, independent of the oracle's surface and of
Arris; observations, not an implementation.
"""
import json
import math

import numpy as np

from oracle import occt_version, require_ocp

require_ocp()
from OCP.BRep import BRep_Tool  # noqa: E402
from OCP.BRepAdaptor import BRepAdaptor_Surface  # noqa: E402
from OCP.BRepClass import BRepClass_FaceClassifier  # noqa: E402
from OCP.BRepGProp import BRepGProp, BRepGProp_Domain, BRepGProp_Face, BRepGProp_Vinert  # noqa: E402
from OCP.GeomAbs import GeomAbs_BSplineSurface  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_IN  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402
from OCP.gp import gp_Pnt, gp_Pnt2d  # noqa: E402
from oracle.fixture import corpus_root, load_fixture  # noqa: E402
from oracle.recipe import build  # noqa: E402
from cusp_corner_law import along, foot, setting  # noqa: E402

SIZES = (0.02, 0.1, 0.3)
# Oracle interior samples: an N×N grid over the patch's (u, v)
# rectangle, kept where the face classifier puts them inside.
GRID = 24
# The graph's resolutions: uniform spans per direction, and halvings of
# the span graded into each of the three special points (two feet, cusp).
RESOLUTIONS = ((16, 3), (24, 4))
# Gauss–Legendre points per knot span and direction for the energy and
# the measures over D, kept where they fall inside it.
QUAD = 4
# Boundary samples per side for the penalties and the boundary check.
SIDE = 240
# Penalty weights, relative to the bending energy of a unit bump over D:
# heights in units of r², slopes dimensionless. Outside D the energy is
# weighted by EPS, which only fixes the coefficients D never reaches.
HEIGHT_WEIGHT, SLOPE_WEIGHT, EPS = 1e7, 1e5, 1e-4
# Gauss–Legendre points per direction for the ruled chart's measures.
GAUSS = 64


def explore(shape, kind, cast):
    e = TopExp_Explorer(shape, kind)
    while e.More():
        yield cast(e.Current())
        e.Next()


def point(p):
    return np.array([p.X(), p.Y(), p.Z()])


class Corner:
    """ADR-0084's boundary over the shared plane: wall `i`'s trace from its
    foot `F_i` (s = 0) to the cusp (s = 1) at height h − σr(1 − s), and the
    segment from `F_i` (s = 0) to `Q` (s = 1) at the end ball's height."""

    def __init__(self, name, r):
        self.walls, self.cusp, self.q, self.h, self.sigma = setting(name, r)
        self.r = r
        self.f = [foot(w, self.q) for w in self.walls]

    def wall(self, i, s):
        return along(self.walls[i], self.f[i], self.cusp, s), self.h - self.sigma * self.r * (1 - s)

    def segment(self, i, s):
        p = self.f[i] + (self.q - self.f[i]) * s
        rho = np.linalg.norm(p - self.q)
        return p, self.h - self.sigma * self.r + self.sigma * math.sqrt(max(self.r ** 2 - rho ** 2, 0.0))

    def polygon(self, m=600):
        s = np.linspace(0, 1, m)
        pts = [self.wall(0, 1 - x)[0] for x in s]
        pts += [self.segment(0, x)[0] for x in s[1:]]
        pts += [self.segment(1, 1 - x)[0] for x in s[1:]]
        pts += [self.wall(1, x)[0] for x in s[1:-1]]
        return np.array(pts)


def inside(poly, pts):
    x, y = pts[:, 0], pts[:, 1]
    res = np.zeros(len(pts), bool)
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        if a[1] == b[1]:
            continue
        cross = (a[1] > y) != (b[1] > y)
        res ^= cross & (x < a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]))
    return res


def basis(knots, p, t):
    """Every cubic basis function of the clamped `knots` at the points `t`,
    with its first two derivatives: arrays (len(t), n)."""
    t = np.clip(np.asarray(t, float), knots[0], knots[-1])
    m = len(knots) - 1
    last = np.searchsorted(knots, knots[-1]) - 1
    n0 = np.zeros((len(t), m))
    for i in range(m):
        if knots[i] < knots[i + 1]:
            n0[:, i] = (knots[i] <= t) & ((t < knots[i + 1]) | ((i == last) & (t == knots[-1])))
    levels = [n0]
    for d in range(1, p + 1):
        prev, cur = levels[-1], np.zeros((len(t), m - d))
        for i in range(m - d):
            a = knots[i + d] - knots[i]
            b = knots[i + d + 1] - knots[i + 1]
            if a > 0:
                cur[:, i] += (t - knots[i]) / a * prev[:, i]
            if b > 0:
                cur[:, i] += (knots[i + d + 1] - t) / b * prev[:, i + 1]
        levels.append(cur)

    def derivative(low, d):
        out = np.zeros((len(t), low.shape[1] - 1))
        for i in range(out.shape[1]):
            a = knots[i + d] - knots[i]
            b = knots[i + d + 1] - knots[i + 1]
            if a > 0:
                out[:, i] += d / a * low[:, i]
            if b > 0:
                out[:, i] -= d / b * low[:, i + 1]
        return out

    d1 = derivative(levels[p - 1], p)
    d2 = derivative(derivative(levels[p - 2], p - 1), p)
    return levels[p], d1, d2


def graded(lo, hi, spans, focus, halvings):
    br = set(np.linspace(lo, hi, spans + 1))
    step = (hi - lo) / spans
    for f in focus:
        for k in range(1, halvings + 1):
            for s in (f - step / 2 ** k, f + step / 2 ** k):
                if lo < s < hi:
                    br.add(s)
    return np.array(sorted(br))


class Graph:
    """ADR-0084 §5: the least-bending height graph over D, as a cubic tensor
    B-spline over a rectangle covering it."""

    def __init__(self, c, spans, halvings):
        poly = c.polygon()
        lo, hi = poly.min(0), poly.max(0)
        pad = 0.02 * (hi - lo)
        self.lo, self.hi = lo - pad, hi + pad
        p = 3
        self.k = []
        for d in range(2):
            br = graded(self.lo[d], self.hi[d], spans, [c.f[0][d], c.f[1][d], c.cusp[d]], halvings)
            self.k.append(np.concatenate([[br[0]] * p, br, [br[-1]] * p]))
        n = [len(k) - p - 1 for k in self.k]
        # Gauss–Legendre points in every knot span: the rule meets every
        # span however finely it is graded.
        gx_, gw_ = np.polynomial.legendre.leggauss(QUAD)
        axes, weights = [], []
        for d in range(2):
            br = np.unique(self.k[d])
            mid, half = (br[1:] + br[:-1]) / 2, (br[1:] - br[:-1]) / 2
            axes.append((mid[:, None] + half[:, None] * gx_).ravel())
            weights.append((half[:, None] * gw_).ravel())
        bx, by = basis(self.k[0], p, axes[0]), basis(self.k[1], p, axes[1])
        gx, gy = np.meshgrid(*axes, indexing="ij")
        mask = inside(poly, np.stack([gx.ravel(), gy.ravel()], 1)).reshape(gx.shape)
        cellw = np.outer(*weights)
        # The energy's Hessian, a 16 × 16 block per point: each point meets
        # 4 × 4 basis functions. Outside D the energy is weighted by EPS,
        # which only fixes the coefficients D never reaches, smoothly.
        ii, jj = np.nonzero(np.ones_like(mask))
        sx = np.array([np.searchsorted(self.k[0], x, side="right") - 4 for x in axes[0]])[ii]
        sy = np.array([np.searchsorted(self.k[1], y, side="right") - 4 for y in axes[1]])[jj]
        sx, sy = np.minimum(sx, n[0] - 4), np.minimum(sy, n[1] - 4)
        local = np.arange(4)
        idx = ((sx[:, None, None] + local[None, :, None]) * n[1]
               + (sy[:, None, None] + local[None, None, :])).reshape(len(ii), 16)
        cell = cellw[ii, jj] * np.where(mask[ii, jj], 1.0, EPS)
        energy = np.zeros((n[0] * n[1],) * 2)
        for a, b, m in ((2, 0, 1.0), (1, 1, 2.0), (0, 2, 1.0)):
            vx = np.take_along_axis(bx[a][ii], sx[:, None] + local, axis=1)
            vy = np.take_along_axis(by[b][jj], sy[:, None] + local, axis=1)
            v = np.einsum("ca,cb->cab", vx, vy).reshape(len(ii), 16)
            np.add.at(energy, (idx[:, :, None], idx[:, None, :]), m * cell[:, None, None] * v[:, :, None] * v[:, None, :])
        rows, rhs, weights = [], [], []
        s = np.linspace(0, 1, SIDE + 1)
        for i in range(2):
            for side in (c.wall, c.segment):
                pts = np.array([side(i, x)[0] for x in s])
                rows.append(self.rows(pts))
                rhs.append(np.array([side(i, x)[1] for x in s]))
                weights.append(np.full(len(s), HEIGHT_WEIGHT / c.r ** 2))
            inner = s[1:-1]
            pts = np.array([c.segment(i, x)[0] for x in inner])
            along_seg = (c.q - c.f[i]) / np.linalg.norm(c.q - c.f[i])
            across = np.array([-along_seg[1], along_seg[0]])
            rows.append(across[0] * self.rows(pts, 1, 0) + across[1] * self.rows(pts, 0, 1))
            rhs.append(np.zeros(len(inner)))
            weights.append(np.full(len(inner), SLOPE_WEIGHT))
        a, b, wt = np.vstack(rows), np.concatenate(rhs), np.concatenate(weights)
        self.coef = np.linalg.solve(energy + a.T @ (wt[:, None] * a), a.T @ (wt * b))
        self.c, self.mask, self.axes, self.cellw = c, mask, axes, cellw

    def rows(self, pts, dx=0, dy=0):
        ex = basis(self.k[0], 3, pts[:, 0])[dx]
        ey = basis(self.k[1], 3, pts[:, 1])[dy]
        return np.einsum("pi,pj->pij", ex, ey).reshape(len(pts), -1)

    def __call__(self, pts, dx=0, dy=0):
        return self.rows(pts, dx, dy) @ self.coef

    def measures(self):
        """Cone volume under the upward normal (the body's outward one at
        every oracle sample) and area, over the cells inside D."""
        gx, gy = np.meshgrid(*self.axes, indexing="ij")
        pts = np.stack([gx.ravel(), gy.ravel()], 1)[self.mask.ravel()]
        w = self.cellw.ravel()[self.mask.ravel()]
        z, fx, fy = self(pts), self(pts, 1, 0), self(pts, 0, 1)
        volume = np.sum(w * (-pts[:, 0] * fx - pts[:, 1] * fy + z)) / 3
        area = np.sum(w * np.sqrt(1 + fx * fx + fy * fy))
        return volume, area

    def boundary_miss(self):
        s = np.linspace(0, 1, 4 * SIDE + 1)
        miss = 0.0
        for i in range(2):
            for side in (self.c.wall, self.c.segment):
                pts = np.array([side(i, x)[0] for x in s])
                miss = max(miss, float(np.abs(self(pts) - np.array([side(i, x)[1] for x in s])).max()))
        return miss


def ruled_hermite(c):
    """The ruled chart's G1 Hermite fill: cone volume and area by Gauss."""
    def xy(s, t):
        far = c.segment(0, 2 * t)[0] if t <= 0.5 else c.segment(1, 2 * (1 - t))[0]
        chord = (1 - t) * c.f[0] + t * c.f[1]
        return (1 - t) * c.wall(0, s)[0] + t * c.wall(1, s)[0] + (1 - s) * (far - chord)

    def cross_slope(t):
        """The end ball's height's derivative along the chart's s at s = 0:
        G1 to the ball."""
        if t <= 0 or t >= 1:
            return 0.0
        e = 1e-7
        p = xy(0, t)
        d = p - c.q
        grad = -c.sigma * d / math.sqrt(max(c.r ** 2 - d @ d, 1e-300))
        return grad @ ((xy(e, t) - p) / e)

    rate = c.sigma * c.r

    def surface(s, t):
        far = (c.segment(0, 2 * t) if t <= 0.5 else c.segment(1, 2 * (1 - t)))[1]
        h0, h1 = 1 - 3 * s * s + 2 * s ** 3, s - 2 * s * s + s ** 3
        # The wall's own rate is taken back at the two feet, where G1 and
        # G0 on the wall cannot both hold: the bump vanishes on the walls.
        ends = (1 - t) ** 4 + t ** 4
        z = c.wall(0, s)[1] + (far - (c.h - c.sigma * c.r)) * h0 + (cross_slope(t) - rate * ends) * h1
        return np.array([*xy(s, t), z])

    x, w = np.polynomial.legendre.leggauss(GAUSS)
    x, w = (x + 1) / 2, w / 2
    e = 1e-6
    volume = area = 0.0
    for i in range(GAUSS):
        for j in range(GAUSS):
            s, t = x[i], x[j]
            p = surface(s, t)
            ds = (surface(s + e, t) - surface(s - e, t)) / (2 * e)
            dt = (surface(s, t + e) - surface(s, t - e)) / (2 * e)
            n = np.cross(ds, dt)
            n = n if n[2] > 0 else -n
            volume += w[i] * w[j] * (p @ n) / 3
            area += w[i] * w[j] * np.linalg.norm(n)
    return volume, area


def oracle_patch(shape):
    patch = next(f for f in explore(shape, TopAbs_FACE, TopoDS.Face)
                 if BRepAdaptor_Surface(f).GetType() == GeomAbs_BSplineSurface)
    tolerance, length = 0.0, 0.0
    for edge in explore(patch, TopAbs_EDGE, TopoDS.Edge):
        props = GProp_GProps()
        BRepGProp.LinearProperties_s(edge, props)
        length += props.Mass()
        tolerance = max(tolerance, BRep_Tool.Tolerance_s(edge))
    origin = gp_Pnt(0, 0, 0)
    # The cone volume of the face as oriented in the shell: what the patch
    # contributes to the body's volume.
    cone = BRepGProp_Vinert(BRepGProp_Face(patch), BRepGProp_Domain(patch), origin, origin, 1e-12)
    area = GProp_GProps()
    BRepGProp.SurfaceProperties_s(patch, area)
    bs = BRepAdaptor_Surface(patch)
    u0, u1, v0, v1 = bs.FirstUParameter(), bs.LastUParameter(), bs.FirstVParameter(), bs.LastVParameter()
    samples = []
    for i in range(1, GRID):
        for j in range(1, GRID):
            u, v = u0 + (u1 - u0) * i / GRID, v0 + (v1 - v0) * j / GRID
            if BRepClass_FaceClassifier(patch, gp_Pnt2d(u, v), 1e-9).State() == TopAbs_IN:
                samples.append(point(bs.Value(u, v)))
    return patch, area.Mass(), cone.Mass(), tolerance, length, np.array(samples)


def record(name, r, shape):
    _, area, cone, tolerance, length, samples = oracle_patch(shape)
    vol, body_area = GProp_GProps(), GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, vol)
    BRepGProp.SurfaceProperties_s(shape, body_area)
    bound_volume, bound_area = area * tolerance, length * tolerance
    c = Corner(name, r)

    def against(volume, law_area):
        return {"volume_over_bound": (volume - cone) / bound_volume,
                "area_over_bound": (law_area - area) / bound_area}

    graphs = []
    for spans, halvings in RESOLUTIONS:
        g = Graph(c, spans, halvings)
        dev = (g(samples[:, :2]) - samples[:, 2]) * c.sigma
        graphs.append({"spans": spans, "halvings": halvings, **against(*g.measures()),
                       "boundary_miss_over_r": g.boundary_miss() / r,
                       "below_oracle_over_r": {"mean": float(-dev.mean() / r), "max": float(-dev.min() / r)}})
    return {
        "oracle": {"patch_area": area, "patch_cone_volume": cone, "patch_tolerance": tolerance,
                   "boundary_length": length, "body_volume": vol.Mass(), "body_area": body_area.Mass(),
                   "samples": len(samples)},
        "least_bending_graph": graphs,
        "ruled_hermite": against(*ruled_hermite(c)),
    }


def records():
    out = []
    for directory in sorted((corpus_root() / "regression").glob("cusp-corner-*-fillet")):
        name = directory.name.removeprefix("cusp-corner-")
        for size in SIZES:
            fixture = load_fixture(directory)
            fixture["params"]["r"] = size
            shape, _ = build(fixture, "default", [])
            out.append({"fixture": directory.name, "size": size, **record(name, size, shape)})
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
