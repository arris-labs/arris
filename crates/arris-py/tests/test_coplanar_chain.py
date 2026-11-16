"""The four programs of the training harness that the flat-edge plan made
build (a flat edge read as a tangent contact, C12): the chain of three
cylinders on one plane, three slots through a cylinder, the interference of
a box against its own halves, and a bore cut with a counterbore as two
tools. Each is its fixture's recipe (`tests/fixtures/boolean/` and
`query/`) run on the raw binding, held to the fixture's closed form, its
probes and the checker at `full`."""

import math

import arris
import pytest

from conftest import AREA_REL, CENTROID_ABS, VOLUME_REL

PI = math.pi
SQRT3 = math.sqrt(3)

# `classify_points` codes (test_queries.py).
IN, OUT, ON = 1, 0, 2
CODE = {"in": IN, "out": OUT, "on": ON}


def hold(model, body, volume, area, centroid, genus, probes):
    """The body's measures match the closed form, the checker is green at
    `full`, its genus is the closed form's, and each probe classifies as
    its fixture says."""
    props = model.mass_properties(body)
    assert props.volume == pytest.approx(volume, rel=VOLUME_REL)
    assert props.area == pytest.approx(area, rel=AREA_REL)
    assert props.centroid == pytest.approx(tuple(centroid), abs=CENTROID_ABS, rel=0)
    report = model.check(body, "full")
    assert report.ok, report.violations
    assert report.euler is not None and report.euler.genus == genus
    codes = model.classify_points(body, [point for _, point, _ in probes])
    for (label, _, expect), got in zip(probes, codes):
        assert got == CODE[expect], label


def test_three_cylinders_on_one_plane_fuse_as_a_chain():
    """The fixture `chained-fuse-coplanar-cylinders`: (a + b) + c, three unit
    discs one unit apart at the corners of an equilateral triangle. Its
    volume is 3 pi / 2 + sqrt 3, its area 6 pi + 2 sqrt 3."""
    model = arris.Model()

    def disc(x, y):
        return model.primitive_cylinder((x, y, 0), (0, 0, 1), 1, 1)[0]

    a = disc(0, 0)
    b = disc(1, 0)
    c = disc(0.5, SQRT3 / 2)
    ab = model.fuse(a, b)[0]
    result = model.fuse(ab, c)[0]
    hold(
        model,
        result,
        volume=3 * PI / 2 + SQRT3,
        area=6 * PI + 2 * SQRT3,
        centroid=(0.5, SQRT3 / 6, 0.5),
        genus=0,
        probes=[
            ("inside_a", (-0.5, 0, 0.5), "in"),
            ("inside_triple", (0.5, 0.288675134594813, 0.5), "in"),
            ("outside_between", (-0.8, 0.9, 0.5), "out"),
            ("on_wall_a", (-1, 0, 0.5), "on"),
            ("on_end_plane", (0.5, 0.288675134594813, 0), "on"),
            ("above", (0.5, 0.288675134594813, 1.5), "out"),
        ],
    )


def test_three_slots_through_a_cylinder_cut_one_by_one():
    """The fixture `three-slots-through-cylinder`: a radius-4 cylinder cut by
    three 9 x 1 x 6 slots through its axis, 60 degrees apart, one cut each.
    Six windows and six columns make genus 5."""
    model = arris.Model()
    rho, a, height, t = 4.0, 0.5, 10.0, 6.0
    body = model.primitive_cylinder((0, 0, -height / 2), (0, 0, 1), rho, height)[0]
    slot0 = model.primitive_box((-9 / 2, -a, -t / 2), (9 / 2, a, t / 2))[0]
    slot1 = model.transform(slot0, axis=(0, 0, 1), angle=PI / 3)[0]
    slot2 = model.transform(slot0, axis=(0, 0, 1), angle=2 * PI / 3)[0]
    cut1 = model.cut(body, slot0)[0]
    cut2 = model.cut(cut1, slot1)[0]
    result = model.cut(cut2, slot2)[0]

    s = math.sqrt(rho**2 - a**2)
    th = math.atan(a / s)
    arc = 3 * (2 * a * s + 2 * rho**2 * th) - 3 * (8 * a**2 / SQRT3) + 2 * SQRT3 * a**2
    volume = PI * rho**2 * height - t * arc
    area = (
        2 * PI * rho**2
        + 2 * PI * rho * height
        - 6 * t * 2 * rho * th
        + 2 * arc
        + 12 * t * (s - SQRT3 * a)
    )
    hold(
        model,
        result,
        volume=volume,
        area=area,
        centroid=(0, 0, 0),
        genus=5,
        probes=[
            ("in_column", (0, 2, 0), "in"),
            ("in_slot_centre", (0, 0, 0), "out"),
            ("in_slot_arm", (3, 0, 0), "out"),
            ("in_slab", (3, 0, 4), "in"),
            ("on_slot_ceiling", (3, 0, 3), "on"),
            ("on_column_side", (3, 0.5, 0), "on"),
            ("outside", (5, 0, 0), "out"),
        ],
    )


def test_a_box_interferes_with_its_own_halves_at_its_whole_volume():
    """The query fixture `interference-box-against-its-halves`: a 2 x 2 x 1
    box against the same box fused from two halves across x = 1. The two
    solids are the same, so the overlap is the whole 4 and each lies in the
    other, in either order."""
    model = arris.Model()
    whole = model.primitive_box((0, 0, 0), (2, 2, 1))[0]
    left = model.primitive_box((0, 0, 0), (1, 2, 1))[0]
    right = model.primitive_box((1, 0, 0), (2, 2, 1))[0]
    halves = model.fuse(left, right)[0]
    for first, second in ((whole, halves), (halves, whole)):
        clash = model.interference(first, second)
        assert clash.kind == "overlapping"
        assert clash.volume == pytest.approx(4, abs=1e-9)
        assert clash.a_in_b is True and clash.b_in_a is True


def test_a_bore_and_its_counterbore_cut_as_two_tools_of_one_cut():
    """The fixture `cut-many-bore-over-bore-of-its-width`: a 10 x 8 x 4 box
    with a radius-1/2 through hole, then a bore of the hole's radius and a
    counterbore of radius 1, 1/2 deep, cut in one call. The tools' tops lie
    inside the part, so the chamber round the hole is removed, and the
    result is the box less the hole, less a ring of 3/8 pi."""
    model = arris.Model()
    r, big_r, d = 0.5, 1.0, 0.5
    box = model.primitive_box((-5, -4, -2), (5, 4, 2))[0]
    hole = model.primitive_cylinder((0, 0, -3), (0, 0, 1), r, 6)[0]
    drilled = model.cut(box, hole)[0]
    bore = model.primitive_cylinder((0, 0, -3), (0, 0, 1), r, 3)[0]
    counterbore = model.primitive_cylinder((0, 0, -d), (0, 0, 1), big_r, d)[0]
    result = model.cut_many(drilled, [bore, counterbore])[0]

    ring = PI * (big_r**2 - r**2) * d
    removed = PI * r**2 * 4 + ring
    hold(
        model,
        result,
        volume=10 * 8 * 4 - removed,
        area=(
            2 * (10 * 8 + 10 * 4 + 8 * 4)
            - 2 * PI * r**2
            + 2 * PI * r * 4
            - 2 * PI * r * d
            + 2 * PI * big_r * d
            + 2 * PI * (big_r**2 - r**2)
        ),
        centroid=(0, 0, ring * d / 2 / (320 - removed)),
        genus=1,
        probes=[
            ("in_chamber", (0.8, 0, -0.25), "out"),
            ("above_chamber", (0.8, 0, 0.25), "in"),
            ("below_chamber", (0.8, 0, -0.75), "in"),
            ("in_hole", (0, 0, 1), "out"),
            ("on_chamber_wall", (1, 0, -0.25), "on"),
            ("on_chamber_ceiling", (0.8, 0, 0), "on"),
            ("in_plate", (3, 2, 1), "in"),
        ],
    )
