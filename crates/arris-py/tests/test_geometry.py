"""`Model.point`, `curve`, `surface` and `bounds`: each held to the
parameters the shape was built from, and every box to the positions
`tessellate` draws inside it."""

import math
import struct

import arris
import pytest

TAU = 2 * math.pi
# The profile plane of a revolve about z: (u, v) is (x, z).
XZ = dict(origin=(0, 0, 0), normal=(0, -1, 0), x_axis=(1, 0, 0))


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b, strict=True))


def positions(mesh):
    flat = struct.unpack(f"<{mesh.n_positions * 3}d", mesh.positions)
    return [flat[i : i + 3] for i in range(0, len(flat), 3)]


def inside(box, points):
    lo, hi = box
    return all(lo[k] <= p[k] <= hi[k] for p in points for k in range(3))


def surfaces(model, body):
    return {f: model.surface(f) for f in model.faces(body)}


def only(model, body, kind):
    [found] = [s for s in surfaces(model, body).values() if s.kind == kind]
    return found


def revolved(model, loop):
    body, _ = model.revolve(arris.Profile(loop, **XZ), (0, 0, 0), (0, 0, 1), TAU)
    return body


def test_a_box_is_its_corners_lines_and_outward_planes():
    model = arris.Model()
    body, _ = model.primitive_box((1, 2, 3), (4, 6, 8))
    corners = {(x, y, z) for x in (1, 4) for y in (2, 6) for z in (3, 8)}
    assert {model.point(v) for v in model.vertices(body)} == corners
    for edge in model.edges(body):
        line = model.curve(edge)
        assert line.kind == "line" and line.frame is None and line.radius is None
        assert sorted(abs(c) for c in line.axis) == [0.0, 0.0, 1.0]
    centre = (2.5, 4, 5.5)
    for face, plane in surfaces(model, body).items():
        assert plane.kind == "plane" and plane.axis is None and plane.radius is None
        # Outward: the normal points away from the box's centre, and the
        # origin lies on the face's own coordinate.
        [k] = [k for k in range(3) if plane.normal[k] != 0]
        assert abs(plane.normal[k]) == 1.0
        assert (plane.origin[k] - centre[k]) * plane.normal[k] > 0
        assert plane.origin[k] in ((1, 4), (2, 6), (3, 8))[k]
        assert plane.normal == model.face_frame(face).z


def test_an_edge_reached_reversed_starts_at_its_curves_end():
    model = arris.Model()
    body, _ = model.primitive_cylinder((0, 0, 0), (0, 0, 1), 2, 5)
    box, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
    for b in (body, box):
        for face in model.faces(b):
            for edge in model.edges_of(face):
                start, end = model.vertices_of(edge)
                curve = model.curve(edge)
                assert (curve.start, curve.end) == (model.point(start), model.point(end))


def test_a_cylinder_is_its_axis_radius_and_circles():
    model = arris.Model()
    body, _ = model.primitive_cylinder((1, 2, 3), (0, 0, 1), 2.5, 4)
    side = only(model, body, "cylinder")
    assert side.radius == 2.5 and side.normal is None
    assert side.axis == (0.0, 0.0, 1.0)
    assert side.origin[:2] == (1.0, 2.0)
    caps = sorted(s.normal for s in surfaces(model, body).values() if s.kind == "plane")
    assert caps == [(0.0, 0.0, -1.0), (0.0, 0.0, 1.0)]
    circles = [c for c in map(model.curve, model.edges(body)) if c.kind == "circle"]
    assert sorted(c.origin for c in circles) == [(1.0, 2.0, 3.0), (1.0, 2.0, 7.0)]
    for c in circles:
        assert c.radius == 2.5 and c.start == c.end and c.major_radius is None
        assert c.frame is not None and c.frame.z == c.axis
        assert sorted(abs(a) for a in c.axis) == [0.0, 0.0, 1.0]


def test_an_extruded_ellipse_is_an_elliptic_cylinder():
    model = arris.Model()
    profile = arris.Profile(arris.Loop.ellipse((0, 0), (3, 0), 1))
    body, _ = model.extrude(profile, (0, 0, 1), 2)
    side = only(model, body, "elliptic_cylinder")
    assert (side.major_radius, side.minor_radius) == (3.0, 1.0)
    assert side.radius is None and side.axis is not None
    ellipses = [c for c in map(model.curve, model.edges(body)) if c.kind == "ellipse"]
    assert len(ellipses) == 2
    for e in ellipses:
        assert (e.major_radius, e.minor_radius) == (3.0, 1.0) and e.radius is None
        assert close(e.frame.x, (1, 0, 0)) or close(e.frame.x, (-1, 0, 0))


def test_a_revolved_half_disc_is_a_sphere():
    model = arris.Model()
    loop = arris.Loop.path(
        (0, -2), [arris.Segment.arc(to=(0, 2), via=(2, 0)), arris.Segment.line((0, -2))]
    )
    sphere = only(model, revolved(model, loop), "sphere")
    assert sphere.radius == pytest.approx(2.0, abs=1e-12)
    assert close(sphere.origin, (0, 0, 0)) and close(sphere.axis, (0, 0, 1))
    # The seam is the profile's arc, pole to pole; each pole a degenerate
    # edge at its point.
    body = revolved(model, loop)
    curves = sorted((model.curve(e) for e in model.edges(body)), key=lambda c: c.kind)
    assert [c.kind for c in curves] == ["circle", "degenerate", "degenerate"]
    seam, *poles = curves
    assert seam.radius == pytest.approx(2.0, abs=1e-12) and close(seam.origin, (0, 0, 0))
    assert sorted((seam.start[2], seam.end[2])) == pytest.approx([-2.0, 2.0], abs=1e-12)
    assert sorted(p.start[2] for p in poles) == pytest.approx([-2.0, 2.0], abs=1e-12)
    assert all(p.start == p.end and p.axis is None for p in poles)


def test_a_revolved_circle_is_a_torus():
    model = arris.Model()
    torus = only(model, revolved(model, arris.Loop.circle((5, 0), 1.5)), "torus")
    assert torus.major_radius == pytest.approx(5.0, abs=1e-12)
    assert torus.minor_radius == pytest.approx(1.5, abs=1e-12)
    assert close(torus.origin, (0, 0, 0)) and close(torus.axis, (0, 0, 1))
    assert torus.radius is None and torus.half_angle is None


def test_a_revolved_triangle_is_a_cone():
    model = arris.Model()
    body = revolved(model, arris.Loop.polygon([(0, 0), (2, 0), (0, 3)]))
    cone = only(model, body, "cone")
    assert cone.half_angle == pytest.approx(math.atan2(2, 3), abs=1e-12)
    assert close(cone.axis, (0, 0, 1)) or close(cone.axis, (0, 0, -1))
    # The reference circle lies on the cone: its radius at its height.
    height = cone.origin[2]
    assert cone.radius == pytest.approx(2 * (3 - height) / 3, abs=1e-9)
    [base] = [c for c in map(model.curve, model.edges(body)) if c.kind == "circle"]
    assert base.radius == pytest.approx(2.0, abs=1e-12)


def nurbs_pipe(model):
    spine = arris.Path(
        (0, 0, 0),
        [arris.PathSegment.nurbs(2, [0, 0, 0, 1, 1, 1], [(0, 0, 0), (0, 0, 5), (3, 0, 8)])],
    )
    body, _ = model.sweep(arris.Profile(arris.Loop.circle((0, 0), 0.5)), spine)
    return body


def test_a_swept_side_is_a_nurbs_face_with_no_closed_form():
    model = arris.Model()
    body = nurbs_pipe(model)
    side = only(model, body, "nurbs")
    assert side.frame is None and side.origin is None and side.normal is None
    assert side.axis is None and side.radius is None
    # The start cap stays the profile's plane and circle.
    caps = [s for s in surfaces(model, body).values() if s.kind == "plane"]
    assert any(close(s.normal, (0, 0, -1)) and close(s.origin, (0, 0, 0)) for s in caps)
    # The edges are fitted too, the caps' closed ones 0.5 from the spine's
    # ends, the seam running between them.
    curves = [model.curve(e) for e in model.edges(body)]
    assert {c.kind for c in curves} == {"nurbs"}
    closed = [c for c in curves if c.start == c.end]
    assert math.dist(closed[0].start, (0, 0, 0)) == pytest.approx(0.5, abs=1e-9)
    assert math.dist(closed[1].start, (3, 0, 8)) == pytest.approx(0.5, abs=1e-9)
    for c in curves:
        assert c.origin is None and c.axis is None and c.frame is None


@pytest.mark.parametrize(
    "make",
    [
        lambda m: m.primitive_box((-1, 2, 0), (3, 5, 7))[0],
        lambda m: m.primitive_cylinder((1, 1, 1), (1, 2, 2), 1.5, 4)[0],
        lambda m: revolved(m, arris.Loop.circle((5, 0), 1.5)),
        lambda m: revolved(m, arris.Loop.polygon([(0, 0), (2, 0), (0, 3)])),
        nurbs_pipe,
    ],
    ids=["box", "tilted-cylinder", "torus", "cone", "nurbs-pipe"],
)
def test_every_box_contains_what_tessellate_draws(make):
    model = arris.Model()
    body = make(model)
    whole = model.bounds(body)
    assert whole is not None
    assert inside(whole, positions(model.tessellate(body, 0.01)))
    for face in model.faces(body):
        box = model.bounds(face)
        assert box is not None and inside(whole, box)
        assert inside(box, positions(model.tessellate_faces(body, [face], 0.01)))
    for edge in model.edges(body):
        assert inside(model.bounds(edge), (model.curve(edge).start, model.curve(edge).end))
    for vertex in model.vertices(body):
        assert inside(model.bounds(vertex), [model.point(vertex)])
    for shell in model.shells(body):
        assert model.bounds(shell) == whole


def test_an_analytic_box_is_tight():
    model = arris.Model()
    body, _ = model.primitive_cylinder((1, 1, 1), (0, 0, 1), 1.5, 4)
    lo, hi = model.bounds(body)
    tol = 1e-6
    assert close(lo, (-0.5, -0.5, 1), tol) and close(hi, (2.5, 2.5, 5), tol)
    box, _ = model.primitive_box((-1, 2, 0), (3, 5, 7))
    lo, hi = model.bounds(box)
    assert close(lo, (-1, 2, 0), tol) and close(hi, (3, 5, 7), tol)


def test_a_foreign_handle_is_refused():
    model, other = arris.Model(), arris.Model()
    body, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
    [face, *_] = model.faces(body)
    [edge, *_] = model.edges(body)
    [vertex, *_] = model.vertices(body)
    for call in (
        lambda: other.point(vertex),
        lambda: other.curve(edge),
        lambda: other.surface(face),
        lambda: other.bounds(body),
    ):
        with pytest.raises(arris.ForeignHandleError):
            call()
