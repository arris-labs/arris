"""The training harness's tangency programs (tangency-faults, C12): the five
that build, each held to its closed form, its probes and the checker at
`full`, and the four refused by name. Each is its fixture's recipe
(`tests/fixtures/boolean/` and `blend/`, read for the programs and closed
forms in `fixture.json`) run on the raw binding. The two touching-lump
fuses use the build layer."""

import math

import arris
import pytest
from arris.build import Solid, Workplane

from conftest import AREA_REL, CENTROID_ABS, VOLUME_REL

PI = math.pi
TAU = 2 * PI

# `classify_points` codes (test_queries.py).
IN, OUT, ON = 1, 0, 2
CODE = {"in": IN, "out": OUT, "on": ON}

# The profile plane of a revolve about z: (u, v) is (x, z).
XZ = dict(origin=(0, 0, 0), normal=(0, -1, 0), x_axis=(1, 0, 0))


def hold(model, body, *, volume, area, genus, probes, centroid=None, counts=None):
    """The body's measures match the closed form, the checker is green at
    `full`, its genus and (where given) its Euler counts are the closed
    form's, and each probe classifies as its fixture says."""
    props = model.mass_properties(body)
    assert props.volume == pytest.approx(volume, rel=VOLUME_REL)
    assert props.area == pytest.approx(area, rel=AREA_REL)
    if centroid is not None:
        assert props.centroid == pytest.approx(tuple(centroid), abs=CENTROID_ABS, rel=0)
    report = model.check(body, "full")
    assert report.ok, report.violations
    assert not report.unchecked
    line = report.euler
    assert line is not None and line.genus == genus
    if counts is not None:
        assert (line.vertices, line.edges, line.faces, line.loops, line.shells) == counts
    codes = model.classify_points(body, [point for _, point, _ in probes])
    for (label, _, expect), got in zip(probes, codes):
        assert got == CODE[expect], label


def test_a_cylinder_tangent_inside_a_larger_one_fuses_to_its_closed_form():
    """The fixture `fuse-cylinder-tangent-inside`: a unit cylinder fused with
    a radius-2 cylinder centred at y = -1, touching its wall along x = 0,
    y = 1. V = 5 pi, A = 14 pi, centroid y = -4/5."""
    model = arris.Model()
    unit = model.primitive_cylinder((0, 0, -1), (0, 0, 1), 1, 2)[0]
    big = model.primitive_cylinder((0, -1, -0.5), (0, 0, 1), 2, 1)[0]
    result = model.fuse(unit, big)[0]
    hold(
        model,
        result,
        volume=5 * PI,
        area=14 * PI,
        centroid=(0, -0.8, 0),
        genus=0,
        counts=(8, 13, 7, 7, 1),
        probes=[
            ("unit cylinder's top", (0, 0, 0.9), "in"),
            ("big cylinder's far side", (0, -2, 0), "in"),
            ("big cylinder's far side, high", (0, -2, 0.4), "in"),
            ("above the big cylinder, beside the unit one", (1.5, 0, 0.8), "out"),
            ("past the tangent line", (0, 1.5, 0), "out"),
            ("over the unit cylinder", (0, 0, 1.2), "out"),
        ],
    )


def test_a_bar_with_a_boss_tangent_to_each_end_fuses_to_its_closed_form():
    """The fixture `fuse-bar-end-bosses-tangent`: a 30 x 5 x 1 bar with a
    radius-3 boss at each end, each rim tangent to the end face. With
    d = 2.5, a segment of the boss past the bar's side has area
    s = r^2 t - d sqrt(r^2 - d^2), t = acos(d / r), so V = 150 + 4 s."""
    model = arris.Model()
    r, d = 3.0, 2.5
    root = math.sqrt(r * r - d * d)
    t = math.atan(root / d)
    s = r * r * t - d * root
    bar = model.primitive_box((-15, -2.5, -0.5), (15, 2.5, 0.5))[0]
    boss_right = model.primitive_cylinder((12, 0, -0.5), (0, 0, 1), r, 1)[0]
    boss_left = model.primitive_cylinder((-12, 0, -0.5), (0, 0, 1), r, 1)[0]
    one = model.fuse(bar, boss_right)[0]
    result = model.fuse(one, boss_left)[0]
    hold(
        model,
        result,
        volume=150 + 4 * s,
        area=370 + 2 * (4 * s + 12 * t - 4 * root),
        centroid=(0, 0, 0),
        genus=0,
        counts=(30, 62, 34, 34, 1),
        probes=[
            ("first boss's centre", (12, 0, 0), "in"),
            ("first boss past the bar's side", (12, 2.9, 0), "in"),
            ("second boss past the bar's side", (-12, -2.9, 0.4), "in"),
            ("beside a boss, past its circle", (14.9, 2.9, 0), "out"),
            ("past the bar's side between the bosses", (0, 2.9, 0), "out"),
        ],
    )


def test_a_plate_whose_arc_is_tangent_inside_a_cylinder_fuses_to_its_closed_form():
    """The fixture `fuse-arc-plate-tangent-inside-cylinder`: a unit cylinder
    fused with a plate of a 1 x 1/2 rectangle and a half disc of radius 1/2
    on its top, whose arc's top touches the cylinder's wall along a line.
    V = 2 pi + (1/2 + pi/8) / 4, A = 6 pi + 1/2 + pi/8."""
    model = arris.Model()
    cylinder = model.primitive_cylinder((0, 0, -1), (0, 0, 1), 1, 2)[0]
    outline = arris.Loop.path(
        (0.5, 0),
        [
            arris.Segment.line((0.5, 0.5)),
            arris.Segment.arc((-0.5, 0.5), (0, 1)),
            arris.Segment.line((-0.5, 0)),
            arris.Segment.line((0.5, 0)),
        ],
    )
    profile = arris.Profile(
        outline, origin=(0, 0, -1.25), normal=(0, 0, 1), x_axis=(1, 0, 0)
    )
    plate = model.extrude(profile, (0, 0, 1), 1)[0]
    result = model.fuse(cylinder, plate)[0]
    hold(
        model,
        result,
        volume=2 * PI + (0.5 + PI / 8) / 4,
        area=6 * PI + 0.5 + PI / 8,
        genus=0,
        counts=(11, 17, 8, 8, 1),
        probes=[
            ("cylinder's middle", (0, 0, 0.5), "in"),
            ("plate, under the cylinder", (0.2, 0.2, -1.1), "in"),
            ("plate's half disc, under the cylinder", (0, 0.9, -1.1), "in"),
            ("beside the plate, under the cylinder", (0.9, 0.1, -1.1), "out"),
            ("under the plate", (0.2, 0.2, -1.4), "out"),
        ],
    )


def test_a_bore_cut_by_a_column_tangent_to_it_cuts_to_its_closed_form():
    """The fixture `cut-bore-tangent-column`: a 4 x 6 x 8 box with a radius-1/2
    bore down its middle, then a 1 x 1 x 20 column, tangent to the bore along
    four lines, cut through it. V = 4 * 6 * 8 - 8, A = 238, genus 1."""
    model = arris.Model()
    box = model.primitive_box((-2, -3, -4), (2, 3, 4))[0]
    bore = model.primitive_cylinder((0, 0, 4), (0, 0, -1), 0.5, 20)[0]
    bored = model.cut(box, bore)[0]
    column = model.primitive_box((-0.5, -0.5, -10), (0.5, 0.5, 10))[0]
    result = model.cut(bored, column)[0]
    hold(
        model,
        result,
        volume=4 * 6 * 8 - 8,
        area=238,
        centroid=(0, 0, 0),
        genus=1,
        counts=(24, 32, 10, 12, 1),
        probes=[
            ("in the box, past the hole", (1, 1, 0), "in"),
            ("just outside the hole's side", (0.55, 0, 0), "in"),
            ("in the hole's middle", (0, 0, 0), "out"),
            ("in the hole's corner", (0.45, 0.45, 0), "out"),
        ],
    )


def test_an_end_disc_tangent_to_a_wall_fuses_to_its_closed_form():
    """The fixture `fuse-end-disc-tangent-to-wall`: a vertical cylinder of
    radius 1 whose foot touches a horizontal cylinder's end disc along
    x = 6, y = 0, z in [2, 3], with the horizontal one 12 long.
    V = 15 pi - 8/3, A = 33 pi - 8."""
    model = arris.Model()
    vertical = model.primitive_cylinder((5, 0, 0), (0, 0, 1), 1, 3)[0]
    horizontal = model.primitive_cylinder((-6, 0, 3), (1, 0, 0), 1, 12)[0]
    result = model.fuse(vertical, horizontal)[0]
    hold(
        model,
        result,
        volume=15 * PI - 8 / 3,
        area=33 * PI - 8,
        genus=0,
        counts=(6, 9, 5, 5, 1),
        probes=[
            ("vertical cylinder's foot", (5, 0, 1), "in"),
            ("vertical cylinder, off the axis", (5.9, 0.2, 1), "in"),
            ("horizontal cylinder's far end", (-5, 0, 3), "in"),
            ("horizontal cylinder over the vertical one", (5, 0, 3.5), "in"),
            ("above the horizontal cylinder", (5, 0, 4.5), "out"),
            ("past the end disc", (6.5, 0, 3), "out"),
        ],
    )


def test_tangent_cylinders_with_flush_caps_fuse_to_two_lumps():
    """The cotter pin pose: two unit cylinders of height two whose flush
    rims meet at each end of the contact ruling."""
    model = arris.Model()
    right = Solid.cylinder((1, 0, -1), (0, 0, 1), 1, 2, model=model)
    left = Solid.cylinder((-1, 0, -1), (0, 0, 1), 1, 2, model=model)
    fused = right.fuse(left)
    hold(
        model,
        fused.body,
        volume=4 * PI,
        area=12 * PI,
        genus=0,
        counts=(4, 6, 6, 6, 2),
        probes=[
            ("contact ruling", (0, 0, 0), "on"),
            ("flush rims", (0, 0, 1), "on"),
        ],
    )


def test_a_cylinder_tangent_to_a_centred_box_side_fuses_to_two_lumps():
    """The hinge pose: the contact along the side ends on the two short
    edges of the centred plate."""
    model = arris.Model()
    plate = Solid.box((-2, -2, -0.5), (2, 2, 0.5), model=model)
    barrel = Solid.cylinder((3, -3, 0), (0, 1, 0), 1, 6, model=model)
    fused = plate.fuse(barrel)
    hold(
        model,
        fused.body,
        volume=16 + 6 * PI,
        area=48 + 14 * PI,
        genus=0,
        counts=(10, 15, 9, 9, 2),
        probes=[
            ("contact ruling", (2, 0, 0), "on"),
            ("contact ends on the box edge", (2, 2, 0), "on"),
        ],
    )


def test_a_torus_touching_a_box_at_its_outer_equator_fuses_to_two_lumps():
    """The torus link: a ring of radii 3 and 1 about y touches the centre
    of the box's near face at (4, 0, 0), at the torus's seam vertex."""
    model = arris.Model()
    torus = Workplane.XY().circle(1, center=(3, 0)).revolve("y", TAU, model=model)
    box = Solid.box((4, -1, -1), (6, 1, 1), model=model)
    fused = torus.fuse(box)
    volume = 6 * PI**2 + 8
    hold(
        model,
        fused.body,
        volume=volume,
        area=12 * PI**2 + 24,
        centroid=(40 / volume, 0, 0),
        genus=1,
        counts=(9, 14, 7, 7, 2),
        probes=[
            ("inside the ring", (3, 0, 0), "in"),
            ("inside the box", (5, 0, 0), "in"),
            ("contact point", (4, 0, 0), "on"),
            ("next to the contact", (3.99, 0, 0.5), "out"),
        ],
    )


def test_a_ball_touching_a_cylinder_at_its_pole_fuses_to_two_lumps():
    """The ball knob: a radius-two ball at the origin touches the top of
    its radius-one stem at (0, 0, -2)."""
    model = arris.Model()
    ball = (
        Workplane.XZ()
        .start((0, -2))
        .arc_to((0, 2), tangent=(1, 0))
        .close()
        .revolve("y", TAU, model=model)
    )
    stem = Solid.cylinder((0, 0, -5), (0, 0, 1), 1, 3, model=model)
    fused = stem.fuse(ball)
    hold(
        model,
        fused.body,
        volume=41 * PI / 3,
        area=24 * PI,
        centroid=(0, 0, -31.5 / 41),
        genus=0,
        counts=(4, 4, 4, 4, 2),
        probes=[
            ("inside the ball", (0, 0, 0), "in"),
            ("inside the stem", (0, 0, -3), "in"),
            ("south pole contact", (0, 0, -2), "on"),
            ("beside the disc", (1.5, 0, -2), "out"),
        ],
    )


def frustum():
    """The revolve of `fuse-frustum-box-rim-point`'s section: a frustum of
    radius 3 at its foot and 2 at its top, 4 high, on the axis z."""
    model = arris.Model()
    loop = arris.Loop.polygon([(0, 0), (3, 0), (2, 4), (0, 4)])
    body = model.revolve(arris.Profile(loop, **XZ), (0, 0, 0), (0, 0, 1), TAU)[0]
    return model, body


@pytest.mark.parametrize(
    "lo, hi",
    [((2, -1, 4), (4, 1, 6)), ((-4, -1, 4), (-2, 1, 6))],
    ids=["rim-point-at-the-seam", "rim-point-far-side"],
)
def test_a_frustum_and_a_box_touching_at_a_rim_point_give_two_lumps(lo, hi):
    """The box's bottom edge touches the frustum's top rim at the seam
    or opposite it. Each lump keeps its own copy of the contact vertex."""
    model, body = frustum()
    lug = model.primitive_box(lo, hi)[0]
    fused, _ = model.fuse(body, lug)
    report = model.check(fused, "full")
    assert report.ok and not report.unchecked
    assert len(model.shells(fused)) == 2
    assert model.mass_properties(fused).volume == pytest.approx(76 * math.pi / 3 + 8)
    touch = (2 if lo[0] > 0 else -2, 0, 4)
    vertices = [model.point(v) for v in model.vertices(fused)]
    assert sum(point == pytest.approx(touch) for point in vertices) == 2


def ring(model):
    """The revolve of `chamfer-two-rims-use-up-face`: a ring of outer radius
    10, bore 6 and height 8."""
    loop = arris.Loop.polygon([(6, 0), (10, 0), (10, 8), (6, 8)])
    return model.revolve(arris.Profile(loop, **XZ), (0, 0, 0), (0, 0, 1), TAU)[0]


def top_rims(model, body):
    """The ring's two circles on its top face, at radii 6 and 10."""
    rims = []
    for edge in model.edges(body):
        ends = [model.point(v) for v in model.vertices_of(edge)]
        if all(
            abs(p[2] - 8) < 1e-9 and round(math.hypot(p[0], p[1]), 6) in (6.0, 10.0)
            for p in ends
        ):
            rims.append(edge)
    assert len(rims) == 2
    return rims


def test_two_chamfers_that_use_up_the_face_between_them_are_too_large():
    """The fixture `chamfer-two-rims-use-up-face`: both rims of a 4-wide top
    face chamfered 2.5, their contacts crossing on the face. Refused
    `BlendTooLarge`, naming both edges and the face."""
    model = arris.Model()
    body = ring(model)
    rims = top_rims(model, body)
    with pytest.raises(arris.OpDegenerateError) as raised:
        model.chamfer(body, rims, 2.5)
    assert "does not fit its faces" in raised.value.reason
    entities = raised.value.entities
    assert len(entities) == 3
    assert all(rim in entities for rim in rims)


def test_the_chamfer_refuses_before_the_cut_that_follows_it():
    """The fixture `chamfer-two-rims-use-up-face-then-cut`: the same chamfer
    of the ring, then a cylinder cut across it, run as one program. The
    chamfer is refused `BlendTooLarge` (the too-large reason, not a cut's),
    so the cut is never reached."""
    model = arris.Model()
    body = ring(model)
    rims = top_rims(model, body)
    hole = model.primitive_cylinder((-12, 0, 2), (1, 0, 0), 1, 24)[0]
    reached_the_cut = False
    with pytest.raises(arris.OpDegenerateError) as raised:
        chamfered = model.chamfer(body, rims, 2.5)[0]
        reached_the_cut = True
        model.cut(chamfered, hole)
    assert not reached_the_cut
    assert "does not fit its faces" in raised.value.reason
