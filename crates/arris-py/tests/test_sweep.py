import math

import arris
import pytest


def disc(radius=1.0):
    return arris.Profile(arris.Loop.circle((0, 0), radius))


def test_a_straight_pipe_is_a_cylinder():
    model = arris.Model()
    spine = arris.Path((0, 0, 0), [arris.PathSegment.line((0, 0, 10))])
    body, record = model.sweep(disc(), spine)
    assert model.mass_properties(body).volume == pytest.approx(10 * math.pi, abs=1e-9)
    assert record.generated_from(arris.Role("sweep", "Side", 0, 0, 0))
    assert record.generated_from(arris.Role("sweep", "StartCap", 0))
    assert record.generated_from(arris.Role("sweep", "EndCap", 0))


def test_a_bent_pipe_is_pappus_volume():
    # A quarter turn of radius 5 in the xz plane, the circle in the xy plane
    # at the start: the pipe of Pappus, area * path length of the centroid.
    r = 5.0
    c = math.sqrt(0.5)
    spine = arris.Path(
        (r, 0, 0),
        [arris.PathSegment.arc(to=(0, 0, r), via=(r * c, 0, r * c))],
    )
    profile = arris.Profile(
        arris.Loop.circle((0, 0), 1), origin=(r, 0, 0), normal=(0, 0, 1), x_axis=(1, 0, 0)
    )
    model = arris.Model()
    body, record = model.sweep(profile, spine, arris.FrameLaw.rotation_minimizing())
    assert model.mass_properties(body).volume == pytest.approx(
        math.pi * (math.pi / 2 * r), rel=1e-6
    )
    assert record.generated_from(arris.Role("sweep", "Side", 0, 0, 0))


def test_a_spring_under_a_fixed_binormal():
    # A wire of radius 0.2 wound twice about z at radius 2, pitch 3.
    profile = arris.Profile(
        arris.Loop.circle((0, 0), 0.2),
        origin=(2, 0, 0),
        normal=(0, 1, 0),
        x_axis=(1, 0, 0),
    )
    coil = arris.Path(
        (2, 0, 0),
        [arris.PathSegment.helix((0, 0, 0), (0, 0, 1), pitch=3.0, angle=4 * math.pi)],
    )
    model = arris.Model()
    body, _ = model.sweep(profile, coil, arris.FrameLaw.fixed_binormal((0, 0, 1)))
    # The profile lies in the axial plane, so its centroid's travel across
    # that plane is the turns' arc at radius 2 (Pappus), not the helix.
    assert model.mass_properties(body).volume == pytest.approx(
        math.pi * 0.2**2 * (2 * 2 * math.pi * 2), rel=1e-6
    )


def test_a_gap_is_a_path_error_naming_the_segment():
    spine = arris.Path(
        (0, 0, 0),
        [
            arris.PathSegment.line((0, 0, 5)),
            arris.PathSegment.nurbs(1, [0, 0, 1, 1], [(1, 0, 5), (1, 0, 9)]),
        ],
    )
    with pytest.raises(arris.OpPathError) as raised:
        arris.Model().sweep(disc(), spine)
    assert "segment 1" in raised.value.detail


def test_a_corner_is_refused():
    spine = arris.Path(
        (0, 0, 0),
        [arris.PathSegment.line((0, 0, 5)), arris.PathSegment.line((5, 0, 5))],
    )
    with pytest.raises(arris.OpPathError):
        arris.Model().sweep(disc(), spine)


def test_a_nurbs_span_is_a_fitted_pipe():
    spine = arris.Path(
        (0, 0, 0),
        [
            arris.PathSegment.nurbs(
                2, [0, 0, 0, 1, 1, 1], [(0, 0, 0), (0, 0, 5), (3, 0, 8)]
            )
        ],
    )
    model = arris.Model()
    body, _ = model.sweep(disc(0.5), spine)
    assert model.body_kind(body) == "solid"
    assert model.mass_properties(body).volume > 0


def test_a_binormal_along_the_tangent_is_refused():
    spine = arris.Path((0, 0, 0), [arris.PathSegment.line((0, 0, 10))])
    with pytest.raises(arris.OpPathError):
        arris.Model().sweep(disc(), spine, arris.FrameLaw.fixed_binormal((0, 0, 1)))


def test_a_malformed_curve_or_direction_is_refused_at_construction():
    with pytest.raises(ValueError):
        arris.PathSegment.nurbs(3, [0, 1], [(0, 0, 0)])
    with pytest.raises(arris.OpDegenerateError):
        arris.FrameLaw.fixed_binormal((0, 0, 0))
    with pytest.raises(arris.OpDegenerateError):
        arris.PathSegment.helix((0, 0, 0), (0, 0, 0), 1.0, 1.0)


def test_values_compare_and_read_back():
    a = arris.PathSegment.line((1, 2, 3))
    assert a == arris.PathSegment.line((1, 2, 3))
    assert a != arris.PathSegment.line((1, 2, 4))
    spine = arris.Path((0, 0, 0), [a])
    assert spine.start == (0.0, 0.0, 0.0)
    assert spine.segments == [a]
    assert arris.FrameLaw.rotation_minimizing() == arris.FrameLaw.rotation_minimizing()


def test_a_budget_stops_the_sweep():
    spine = arris.Path((0, 0, 0), [arris.PathSegment.line((0, 0, 10))])
    model = arris.Model()
    with pytest.raises(arris.Interrupted):
        model.sweep(disc(), spine, budget=0)
