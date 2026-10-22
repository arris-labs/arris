"""`arris.build.Workplane` and `Sketch`.

Every construction is held to the raw binding call on the same plane,
turned off the axes: the two bodies have equal mass properties. The path
builder's computed via-point is held to the circle it must lie on.
"""

import math

import arris
import pytest
from arris.build import Sketch, SketchError, Solid, Workplane

# Mass properties of two builds of one solid agree to rounding.
REL = 1e-9
ABS = 1e-9  # tol-literal: the same body built twice, only rounding apart


def tilted():
    """A plane off every axis, with its own origin."""
    return Workplane.of((1.0, -2.0, 3.0), (0.3, -0.5, 0.8), x_axis=(1.0, 0.2, 0.0))


def raw_profile(loop, plane, holes=()):
    return arris.Profile(
        loop, list(holes), origin=plane.origin, normal=plane.z, x_axis=plane.x
    )


def same_body(made, model_and_raw):
    model, raw = model_and_raw
    got = made.model.mass_properties(made.body)
    want = model.mass_properties(raw)
    assert got.volume == pytest.approx(want.volume, rel=REL)
    assert got.area == pytest.approx(want.area, rel=REL)
    assert got.centroid == pytest.approx(want.centroid, abs=ABS)
    for row_got, row_want in zip(got.inertia, want.inertia):
        assert row_got == pytest.approx(row_want, rel=REL, abs=ABS * want.volume)


def test_the_standard_planes_are_right_handed_frames():
    for plane in (Workplane.XY(), Workplane.YZ(), Workplane.XZ(), tilted()):
        x, y, z = plane.x, plane.y, plane.z
        cross = (
            x[1] * y[2] - x[2] * y[1],
            x[2] * y[0] - x[0] * y[2],
            x[0] * y[1] - x[1] * y[0],
        )
        assert cross == pytest.approx(z, abs=1e-12)
        assert math.dist(x, (0, 0, 0)) == pytest.approx(1.0, abs=1e-12)
    assert Workplane.XY(2).origin == (0.0, 0.0, 2.0)
    assert Workplane.YZ(2).origin == (2.0, 0.0, 0.0)
    assert Workplane.XZ(2).origin == (0.0, -2.0, 0.0)


def test_a_workplane_is_a_value():
    plane = tilted()
    assert plane.offset(1.5).offset(-1.5).origin == pytest.approx(plane.origin, abs=1e-12)
    assert plane.rotated(0.4).z == pytest.approx(plane.z, abs=1e-12)
    turned = plane.rotated(math.pi / 2)
    assert turned.x == pytest.approx(plane.y, abs=1e-12)
    assert plane.rotated(0.3, (1, 0, 0)).origin == plane.origin
    assert plane == tilted()


def test_a_bad_plane_is_a_sketch_error():
    with pytest.raises(SketchError):
        Workplane.of((0, 0, 0), (0, 0, 0))
    with pytest.raises(SketchError):
        Workplane.of((0, 0, 0), (0, 0, 1), x_axis=(0, 0, 2))
    with pytest.raises(SketchError):
        Workplane.XY().rotated(1.0, (0, 0, 0))


def test_a_planar_face_is_a_workplane_outward():
    block = Solid.box((0, 0, 0), (2, 3, 4))
    for direction in ((0, 0, 1), (0, 0, -1), (1, 0, 0), (0, -1, 0)):
        plane = block.faces().extreme(direction).workplane()
        assert plane.z == pytest.approx(direction, abs=1e-12)
        centre = plane.origin
        assert block.model.distance(centre, block.faces().extreme(direction).one()).distance < 1e-9
    with pytest.raises(arris.build.SelectionError):
        block.faces().workplane()
    pierced = block.cut(Solid.cylinder((1, 1.5, -1), (0, 0, 1), 0.5, 6))
    with pytest.raises(SketchError, match="cylinder"):
        pierced.faces().kind("cylinder").workplane()


def test_extrude_matches_the_raw_call_off_the_axes():
    plane = tilted()
    model = arris.Model()
    made = plane.polygon([(0, 0), (4, 0), (4, 2), (1, 3)]).extrude(2.5, model=model)
    raw = model.extrude(
        raw_profile(arris.Loop.polygon([(0, 0), (4, 0), (4, 2), (1, 3)]), plane), plane.z, 2.5
    )[0]
    same_body(made, (model, raw))

    # A hole, a negative distance and both sides.
    ring = plane.circle(3).cut(plane.circle(1, center=(0.5, 0)))
    loop = arris.Loop.circle((0, 0), 3)
    inner = arris.Loop.circle((0.5, 0), 1)
    down = ring.extrude(-2, model=model)
    raw = model.extrude(raw_profile(loop, plane, [inner]), tuple(-c for c in plane.z), 2)[0]
    same_body(down, (model, raw))

    both = ring.extrude(2, both=True, model=model)
    shifted = plane.offset(-2)
    raw = model.extrude(raw_profile(loop, shifted, [inner]), plane.z, 4)[0]
    same_body(both, (model, raw))


def test_slot_and_rect_have_their_closed_forms():
    plane = tilted()
    model = arris.Model()
    bar = plane.slot(9, 3, center=(1, 1), angle=0.7).extrude(2, model=model)
    area = (9 - 3) * 3 + math.pi * 1.5**2
    assert model.mass_properties(bar.body).volume == pytest.approx(2 * area, rel=REL)
    box = plane.rect(3, 5, center=(-1, 2)).extrude(1, model=model)
    assert model.mass_properties(box.body).volume == pytest.approx(15, rel=REL)
    with pytest.raises(SketchError):
        plane.slot(3, 3)
    with pytest.raises(SketchError):
        plane.slot(2, 3)


def test_arc_to_lies_on_the_circle_through_its_tangent():
    model = arris.Model()
    plane = tilted()
    # A quarter turn left from the origin heading +u, ending at (r, r): the
    # centre is (0, r), so with the two straight sides this is the quarter
    # disc of that centre.
    r = 3.0
    quarter = plane.start((0, 0)).arc_to((r, r), tangent=(1, 0)).line_to((0, r)).close()
    body = quarter.extrude(1, model=model)
    assert model.mass_properties(body.body).volume == pytest.approx(math.pi * r * r / 4, rel=REL)


def test_a_half_disc_by_a_tangent_arc_matches_the_loop_path():
    plane = tilted()
    model = arris.Model()
    r = 2.0
    built = (
        plane.start((-r, 0)).line_to((r, 0)).arc_to((-r, 0), tangent=(0, 1)).close()
    ).extrude(1, model=model)
    loop = arris.Loop.path(
        (-r, 0), [arris.Segment.line((r, 0)), arris.Segment.arc((-r, 0), (0, r))]
    )
    raw = model.extrude(raw_profile(loop, plane), plane.z, 1)[0]
    same_body(built, (model, raw))


def test_an_arc_follows_the_previous_segments_tangent():
    model = arris.Model()
    # A stadium by hand: line, tangent arc, line, tangent arc.
    path = (
        Workplane.XY()
        .start((0, 0))
        .line_to((4, 0))
        .arc_to((4, 2))
        .line_to((0, 2))
        .arc_to((0, 0))
        .close()
    )
    body = path.extrude(1, model=model)
    assert model.mass_properties(body.body).volume == pytest.approx(
        4 * 2 + math.pi * 1.0**2, rel=REL
    )


def test_an_arc_that_cannot_be_drawn_is_a_sketch_error():
    start = Workplane.XY().start((0, 0))
    with pytest.raises(SketchError, match="tangent"):
        start.arc_to((1, 1))
    with pytest.raises(SketchError, match="straight"):
        start.arc_to((2, 0), tangent=(1, 0))
    with pytest.raises(SketchError, match="length"):
        start.arc_to((2, 2), tangent=(0, 0))
    with pytest.raises(SketchError, match="no length"):
        start.line_to((0, 0))
    with pytest.raises(SketchError, match="no segment"):
        start.close()


def test_a_hole_from_another_plane_is_refused():
    with pytest.raises(SketchError, match="not on"):
        Workplane.XY().rect(4, 4).cut(Workplane.XY(1).circle(1))


def test_revolve_matches_the_raw_call_off_the_axes():
    plane = tilted()
    model = arris.Model()
    loop = arris.Loop.polygon([(2, -1), (4, -1), (4, 1), (2, 1)])
    sketch = plane.polygon([(2, -1), (4, -1), (4, 1), (2, 1)])
    for axis, through, onward in (
        ("y", (0, 0), (0, 1)),
        (((0, 0), (0, 1)), (0, 0), (0, 1)),
        (((1, 0), (1, 2)), (1, 0), (0, 1)),
    ):
        made = sketch.revolve(axis, math.pi * 1.5, model=model)
        raw = model.revolve(
            raw_profile(loop, plane),
            plane.point(*through),
            plane.direction(*onward),
            math.pi * 1.5,
        )[0]
        same_body(made, (model, raw))
    with pytest.raises(SketchError):
        sketch.revolve("z", 1.0)
    with pytest.raises(SketchError):
        sketch.revolve(((1, 1), (1, 1)), 1.0)


def test_sweep_matches_the_raw_call():
    model = arris.Model()
    r = 5.0
    c = math.sqrt(0.5)
    spine = arris.Path((r, 0, 0), [arris.PathSegment.arc(to=(0, 0, r), via=(r * c, 0, r * c))])
    law = arris.FrameLaw.rotation_minimizing()
    plane = Workplane.of((r, 0, 0), (0, 0, 1), x_axis=(1, 0, 0))
    made = plane.circle(1).sweep(spine, law, model=model)
    raw = model.sweep(
        raw_profile(arris.Loop.circle((0, 0), 1), plane), spine, law
    )[0]
    same_body(made, (model, raw))
    assert model.mass_properties(made.body).volume == pytest.approx(
        math.pi * (math.pi / 2 * r), rel=1e-6
    )


def test_loft_matches_the_raw_call():
    plane = tilted()
    model = arris.Model()
    lower = plane.rect(4, 4)
    upper = plane.offset(5).rotated(0.2).rect(2, 2)
    made = Sketch.loft([lower, upper], model=model)
    raw = model.loft([lower.profile(), upper.profile()], "ruled", False)[0]
    same_body(made, (model, raw))


def test_a_sketch_is_a_profile_only_when_asked():
    sketch = Workplane.XY(3).rect(2, 2)
    profile = sketch.profile()
    assert profile.outer == arris.Loop.polygon([(-1, -1), (1, -1), (1, 1), (-1, 1)])
    assert isinstance(profile, arris.Profile)


def test_a_workplane_sketch_uses_the_default_model_unless_given_one():
    plane = Workplane.XY()
    other = arris.Model()
    assert plane.rect(1, 1).extrude(1).model == arris.build.default_model()
    assert plane.rect(1, 1).extrude(1, model=other).model == other
