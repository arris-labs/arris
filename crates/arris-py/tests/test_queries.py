import math

import arris
import pytest
from conftest import matches_oracle


def plate(model, hi=(40, 30, 10)):
    return model.primitive_box((0, 0, 0), hi)[0]


def test_a_through_hole_is_the_oracles():
    model = arris.Model()
    tool, _ = model.primitive_cylinder((20, 15, -1), (0, 0, 1), 4, 12)
    body, _ = model.cut(plate(model), tool)
    matches_oracle(model, body, "boolean/through-hole")


def test_a_blind_hole_is_the_oracles():
    model = arris.Model()
    tool, _ = model.primitive_cylinder((20, 15, 4), (0, 0, 1), 4, 12)
    body, _ = model.cut(plate(model), tool)
    matches_oracle(model, body, "boolean/blind-hole")


def test_the_bolt_pattern_is_the_oracles():
    model = arris.Model()
    body = plate(model, (100, 100, 10))
    for i in range(8):
        angle = math.radians(45 * i)
        centre = (50 + 35 * math.cos(angle), 50 + 35 * math.sin(angle), -1)
        tool, _ = model.primitive_cylinder(centre, (0, 0, 1), 3, 12)
        body, _ = model.cut(body, tool)
    matches_oracle(model, body, "boolean/bolt-pattern-8")


def test_a_filleted_box_is_the_oracles():
    model = arris.Model()
    cube, record = model.primitive_box((0, 0, 0), (2, 2, 2))
    # The vertical edge at x = 2, y = 2 is the box's own `Edge(Z, Max, Max)`.
    edge = record.generated_from(arris.Role("box", "Edge", "Z", "Max", "Max"))
    edges = [e for e in edge if isinstance(e, arris.Edge)]
    body, _ = model.fillet(cube, edges, 0.2)
    matches_oracle(model, body, "blend/box-edge-fillet")


def test_an_extruded_profile_is_the_oracles():
    model = arris.Model()
    sketch = arris.Profile(
        arris.Loop.polygon([(0, 0), (40, 0), (40, 30), (0, 30)]),
        holes=[arris.Loop.circle((20, 15), 4)],
    )
    body, _ = model.extrude(sketch, (0, 0, 1), 10)
    matches_oracle(model, body, "sweep/extrude-plate-with-hole")


def frustum_profile():
    # The fixture's plane: x along x, y along z, so the normal is -y.
    return arris.Profile(
        arris.Loop.polygon([(1, -1), (4, -1), (2, 1), (1, 1)]),
        normal=(0, -1, 0),
        x_axis=(1, 0, 0),
    )


def test_a_revolved_profile_is_the_oracles():
    model = arris.Model()
    body, _ = model.revolve(frustum_profile(), (0, 0, 0), (0, 0, 1), 2 * math.pi)
    matches_oracle(model, body, "sweep/revolve-frustum")


def test_pappus_for_an_extrusion_and_a_revolution():
    model = arris.Model()
    sketch = arris.Profile(
        arris.Loop.polygon([(0, 0), (6, 0), (6, 4), (0, 4)]),
        holes=[arris.Loop.circle((3, 2), 1)],
    )
    area, _ = sketch.area_and_centroid(model)
    body, _ = model.extrude(sketch, (0, 0, 1), 5)
    assert model.mass_properties(body).volume == pytest.approx(area * 5, rel=1e-9)

    # A ring section 1 wide and 5 tall, its centroid 2.5 from the axis.
    ring = arris.Profile(
        arris.Loop.polygon([(2, 0), (3, 0), (3, 5), (2, 5)]),
        normal=(0, -1, 0),
        x_axis=(1, 0, 0),
    )
    area, (u, _) = ring.area_and_centroid(model)
    body, _ = model.revolve(ring, (0, 0, 0), (0, 0, 1), 2 * math.pi)
    assert model.mass_properties(body).volume == pytest.approx(2 * math.pi * u * area, rel=1e-9)
    # A half turn sweeps half the volume.
    half, _ = model.revolve(ring, (0, 0, 0), (0, 0, 1), math.pi)
    assert model.mass_properties(half).volume == pytest.approx(math.pi * u * area, rel=1e-9)


def test_the_inertia_is_about_the_centroid_and_moves_by_parallel_axes():
    model = arris.Model()
    body, _ = model.primitive_box((-1, -1, -1), (1, 1, 1))
    props = model.mass_properties(body)
    assert props.inertia[0][0] == pytest.approx(8 * 4 / 6, rel=1e-12)
    corner = props.inertia_about((1, 1, 1))
    assert corner[0][0] == pytest.approx(8 * 4 / 6 + 8 * 2, rel=1e-12)


def test_the_measures_stop_for_a_budget_and_refuse_a_foreign_body():
    model = arris.Model()
    body, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
    with pytest.raises(arris.Interrupted):
        model.mass_properties(body, budget=0)
    with pytest.raises(arris.ForeignHandleError):
        arris.Model().mass_properties(body)


def test_a_box_walks_as_six_faces_twelve_edges_eight_vertices():
    model = arris.Model()
    body, _ = model.primitive_box((0, 0, 0), (1, 2, 3))
    assert len(model.shells(body)) == 1
    faces, edges, vertices = model.faces(body), model.edges(body), model.vertices(body)
    assert (len(faces), len(edges), len(vertices)) == (6, 12, 8)
    for face in faces:
        assert len(model.edges_of(face)) == 4
    for edge in edges:
        start, end = model.vertices_of(edge)
        assert start != end
        assert len(model.faces_of(body, edge)) == 2
        assert {start, end} <= set(vertices)
    for vertex in vertices:
        assert len(model.edges_at(body, vertex)) == 3


def test_a_reversed_edge_swaps_its_ends():
    model = arris.Model()
    body, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
    for edge in model.edges(body):
        start, end = model.vertices_of(edge)
        assert model.vertices_of(edge) == (start, end)
    # An edge reached through a face use may be reversed; its ends follow.
    seen = {
        (e.index, e.reversed): model.vertices_of(e)
        for face in model.faces(body)
        for e in model.edges_of(face)
    }
    for (index, reversed_), (start, end) in seen.items():
        other = seen.get((index, not reversed_))
        if other is not None:
            assert other == (end, start)


def test_a_cylinders_seam_is_one_edge_of_its_wall():
    model = arris.Model()
    body, record = model.primitive_cylinder((0, 0, 0), (0, 0, 1), 2, 5)
    wall = record.generated_from(arris.Role("cylinder", "Wall"))[0]
    seam = record.generated_from(arris.Role("cylinder", "Seam"))[0]
    edges_of_wall = model.edges_of(wall)
    assert edges_of_wall.count(seam) == 1
    assert len(edges_of_wall) == 3
    assert model.faces_of(body, seam) == [wall]


def test_face_frames_point_outward():
    model = arris.Model()
    body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    for coord, axis in zip("XYZ", ((1, 0, 0), (0, 1, 0), (0, 0, 1))):
        for side, sign in (("Max", 1), ("Min", -1)):
            face = record.generated_from(arris.Role("box", "Face", coord, side))[0]
            frame = model.face_frame(face)
            assert frame.z == pytest.approx(tuple(sign * a for a in axis), abs=1e-12)
    cylinder, made = model.primitive_cylinder((0, 0, 0), (0, 0, 1), 1, 2)
    wall = made.generated_from(arris.Role("cylinder", "Wall"))[0]
    with pytest.raises(arris.OpDegenerateError):
        model.face_frame(wall)
    frame = model.frame_at(wall, math.pi / 2, 1.0)
    assert frame.z == pytest.approx((0, 1, 0), abs=1e-12)
    assert frame.origin == pytest.approx((0, 1, 1), abs=1e-12)
    with pytest.raises(arris.OpDegenerateError):
        model.frame_at(wall, math.pi / 2, 50.0)


def test_the_checker_reports_at_both_levels_and_refuses_a_bad_level():
    model = arris.Model()
    body, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
    fast, full = model.check(body), model.check(body, "full")
    assert fast.ok and full.ok and bool(fast) and len(full) == 0
    assert fast.violations == [] and str(full) == ""
    assert full.euler.faces == 6 and full.euler.genus == 0
    with pytest.raises(ValueError):
        model.check(body, "paranoid")
    with pytest.raises(arris.ForeignHandleError):
        arris.Model().check(body)
