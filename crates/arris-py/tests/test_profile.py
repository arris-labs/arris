import math

import arris
import pytest


def square(size=10.0):
    return arris.Loop.polygon([(0, 0), (size, 0), (size, size), (0, size)])


def role(part, *fields):
    return arris.Role("extrude", part, *fields)


def faces(shapes):
    return [s for s in shapes if isinstance(s, arris.Face)]


def test_area_and_centroid_match_the_closed_forms():
    model = arris.Model()
    disc = arris.Profile(arris.Loop.circle((3, 1), 2))
    area, (u, v) = disc.area_and_centroid(model)
    assert area == pytest.approx(4 * math.pi, abs=1e-12)
    assert (u, v) == pytest.approx((3, 1), abs=1e-12)

    ellipse = arris.Profile(arris.Loop.ellipse((0, 0), (3, 0), 2))
    assert ellipse.area_and_centroid(model)[0] == pytest.approx(6 * math.pi, abs=1e-9)

    # A half disc, drawn as a chord and an arc through the top.
    r = 2.0
    half = arris.Profile(
        arris.Loop.path(
            (-r, 0),
            [arris.Segment.line((r, 0)), arris.Segment.arc((-r, 0), (0, r))],
        )
    )
    area, (u, v) = half.area_and_centroid(model)
    assert area == pytest.approx(math.pi * r * r / 2, abs=1e-9)
    assert (u, v) == pytest.approx((0, 4 * r / (3 * math.pi)), abs=1e-9)

    plate = arris.Profile(square(10), holes=[arris.Loop.circle((5, 5), 1)])
    area, (u, v) = plate.area_and_centroid(model)
    assert area == pytest.approx(100 - math.pi, abs=1e-9)
    assert (u, v) == pytest.approx((5, 5), abs=1e-9)


def test_an_elliptic_arc_segment_is_part_of_a_path():
    model = arris.Model()
    a, b = 3.0, 2.0
    quarter = arris.Profile(
        arris.Loop.path(
            (0, 0),
            [
                arris.Segment.line((a, 0)),
                arris.Segment.ellipse_arc((0, b), (0, 0), (a, 0), b, True),
                arris.Segment.line((0, 0)),
            ],
        )
    )
    area, _ = quarter.area_and_centroid(model)
    assert area == pytest.approx(math.pi * a * b / 4, abs=1e-9)


def test_loops_and_profiles_are_values():
    assert arris.Loop.circle((0, 0), 1) == arris.Loop.circle((0, 0), 1)
    assert arris.Loop.circle((0, 0), 1) != arris.Loop.circle((0, 0), 2)
    one = arris.Profile(square())
    assert one == arris.Profile(square())
    assert one.outer == square() and one.holes == []


def test_extrude_names_every_entity_by_the_part_of_the_sketch():
    model = arris.Model()
    profile = arris.Profile(square(10), holes=[arris.Loop.circle((5, 5), 1)])
    body, record = model.extrude(profile, (0, 0, 1), 4)
    assert model.contains(body)
    for part in ("StartCap", "EndCap"):
        assert len(faces(record.generated_from(role(part)))) == 1
    for segment in range(4):
        assert len(faces(record.generated_from(role("Side", 0, segment)))) == 1
    assert len(faces(record.generated_from(role("Side", 1, 0)))) == 1
    sides = {
        f
        for segment in range(4)
        for f in faces(record.generated_from(role("Side", 0, segment)))
    }
    assert len(sides) == 4


def test_revolve_names_its_sides_too():
    model = arris.Model()
    # A rectangle off the axis, swept a full turn about z: a tube.
    rect = arris.Profile(
        arris.Loop.polygon([(2, 0), (3, 0), (3, 5), (2, 5)]),
        normal=(0, 1, 0),
        x_axis=(1, 0, 0),
    )
    body, record = model.revolve(rect, (0, 0, 0), (0, 0, 1), 2 * math.pi)
    assert model.contains(body)
    for segment in range(4):
        assert len(faces(record.generated_from(arris.Role("revolve", "Side", 0, segment)))) == 1


def test_an_invalid_sketch_is_a_profile_error_when_it_is_swept():
    model = arris.Model()
    open_path = arris.Profile(arris.Loop.path((0, 0), [arris.Segment.line((1, 0))]))
    with pytest.raises(arris.OpProfileError) as raised:
        model.extrude(open_path, (0, 0, 1), 1)
    assert raised.value.detail
    with pytest.raises(arris.OpProfileError):
        open_path.area_and_centroid(model)


def test_arguments_the_kernel_refuses_are_degenerate_errors():
    model = arris.Model()
    profile = arris.Profile(square())
    for call in (
        lambda: arris.Profile(square(), normal=(0, 0, 0)),
        lambda: arris.Profile(square(), x_axis=(0, 0, 1)),
        lambda: arris.Profile(square(), origin=(math.nan, 0, 0)),
        lambda: model.extrude(profile, (0, 0, 0), 1),
        lambda: model.extrude(profile, (0, 0, 1), -1),
        lambda: model.extrude(profile, (1, 0, 0), 1),
        lambda: model.revolve(profile, (0, 0, 0), (0, 0, 0), 1.0),
        lambda: model.revolve(profile, (0, 0, 0), (0, 1, 0), 0.0),
        # The profile spans the axis.
        lambda: model.revolve(profile, (5, 0, 0), (0, 1, 0), math.pi),
    ):
        with pytest.raises(arris.OpDegenerateError):
            call()
