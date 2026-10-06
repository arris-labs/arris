import math
import struct
import sys

import arris
import pytest
from conftest import oracle, tolerances

FIXTURE = "primitive/cylinder"


def cylinder(model):
    return model.primitive_cylinder((0, 0, 0), (0, 0, 1), 4, 12)[0]


def unpack(mesh):
    positions = struct.unpack(f"<{3 * mesh.n_positions}d", mesh.positions)
    triangles = struct.unpack(f"<{3 * mesh.n_triangles}I", mesh.triangles)
    return positions, triangles


def test_a_cylinders_mesh_is_closed_and_has_the_oracles_volume():
    model = arris.Model()
    body = cylinder(model)
    held = tolerances(FIXTURE)
    mesh = model.tessellate(body, held["mesh_chord"])
    assert mesh.is_closed()
    expected = oracle(FIXTURE)["volume"]
    volume = mesh.signed_volume()
    assert volume > 0
    assert abs(volume - expected) <= held["mesh_volume_rel"] * expected
    # The mesh agrees with its own bytes: recompute the volume from them.
    positions, triangles = unpack(mesh)
    total = 0.0
    for i in range(0, len(triangles), 3):
        a, b, c = (positions[3 * k : 3 * k + 3] for k in triangles[i : i + 3])
        total += (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        ) / 6
    assert total == pytest.approx(volume, rel=1e-9)


def test_the_buffers_have_the_documented_sizes_and_indices_are_in_range():
    model = arris.Model()
    mesh = model.tessellate(cylinder(model), 1e-2)
    assert len(mesh.positions) == 24 * mesh.n_positions
    assert len(mesh.triangles) == 12 * mesh.n_triangles
    _, triangles = unpack(mesh)
    assert max(triangles) < mesh.n_positions
    low, high = mesh.bounds()
    # An inscribed polygon: within the chord of the true radius, and the
    # caps are flat at exactly 0 and 12.
    assert low[:2] == pytest.approx((-4, -4), abs=1e-2) and low[2] == 0.0
    assert high[:2] == pytest.approx((4, 4), abs=1e-2) and high[2] == 12.0
    assert all(abs(c) <= 4 for c in low[:2] + high[:2])
    assert mesh.area() > 0


def test_every_face_and_edge_has_its_run():
    model = arris.Model()
    body = cylinder(model)
    mesh = model.tessellate(body, 1e-2)
    assert [face for face, _, _ in mesh.faces] == model.faces(body)
    assert [edge for edge, _, _ in mesh.edges] == model.edges(body)
    runs = [(start, stop) for _, start, stop in mesh.faces]
    assert runs[0][0] == 0 and runs[-1][1] == mesh.n_triangles
    assert all(a[1] == b[0] for a, b in zip(runs, runs[1:]))
    assert len(mesh.edge_indices) % 4 == 0


def test_a_finer_chord_makes_more_triangles():
    model = arris.Model()
    body = cylinder(model)
    coarse, fine = model.tessellate(body, 1e-1), model.tessellate(body, 1e-3)
    assert fine.n_triangles > coarse.n_triangles
    assert model.tessellate(body, 1e-3).positions == fine.positions


def test_a_bad_chord_is_a_mesh_error():
    model = arris.Model()
    body = cylinder(model)
    for chord in (0.0, -1.0, math.nan, math.inf):
        with pytest.raises(arris.MeshChordError) as raised:
            model.tessellate(body, chord)
        assert isinstance(raised.value, arris.MeshError)


def test_tessellation_stops_for_a_budget_or_a_token_and_refuses_a_foreign_body():
    model = arris.Model()
    body = cylinder(model)
    with pytest.raises(arris.Interrupted):
        model.tessellate(body, 1e-3, budget=1)
    token = arris.Cancel()
    token.set()
    with pytest.raises(InterruptedError):
        model.tessellate(body, 1e-3, cancel=token)
    with pytest.raises(arris.ForeignHandleError):
        arris.Model().tessellate(body, 1e-3)


def test_to_numpy_agrees_with_the_bytes():
    numpy = pytest.importorskip("numpy")
    model = arris.Model()
    mesh = model.tessellate(cylinder(model), 1e-2)
    positions, triangles = mesh.to_numpy()
    flat_p, flat_t = unpack(mesh)
    assert positions.dtype == numpy.float64 and positions.shape == (mesh.n_positions, 3)
    assert triangles.dtype == numpy.uint32 and triangles.shape == (mesh.n_triangles, 3)
    assert positions.ravel().tolist() == list(flat_p)
    assert triangles.ravel().tolist() == list(flat_t)
    positions[0, 0] = 99.0  # a copy: the mesh is not touched
    assert unpack(mesh)[0][0] != 99.0


def test_to_numpy_without_numpy_is_a_clear_import_error(monkeypatch):
    model = arris.Model()
    mesh = model.tessellate(cylinder(model), 1e-2)
    monkeypatch.setitem(sys.modules, "numpy", None)
    with pytest.raises(ImportError, match="pip install numpy"):
        mesh.to_numpy()
    # The bytes need nothing.
    assert unpack(mesh)[1]


def test_a_boxs_faces_meshed_one_by_one_weld_to_its_mesh():
    model = arris.Model()
    body, _ = model.primitive_box((0, 0, 0), (1, 2, 3))
    whole = model.tessellate(body, 1e-3)
    faces = model.faces(body)
    parts = [model.tessellate_faces(body, [f], 1e-3) for f in faces]
    assert all(not p.is_closed() for p in parts)
    welded = arris.weld(parts[::-1])
    assert welded.is_closed()
    assert welded.n_triangles == whole.n_triangles
    assert abs(welded.signed_volume() - whole.signed_volume()) < 1e-12
    # A subset, in any order, has the faces asked for once, in the body's order.
    subset = model.tessellate_faces(body, [faces[3], faces[1], faces[3]], 1e-3)
    assert [f for f, _, _ in subset.faces] == [faces[1], faces[3]]


def test_a_face_of_another_body_is_named_and_an_edge_at_two_chords_is_refused():
    model = arris.Model()
    box, _ = model.primitive_box((0, 0, 0), (1, 2, 3))
    other, _ = model.primitive_box((5, 5, 5), (6, 6, 6))
    with pytest.raises(arris.MeshNotInBodyError):
        model.tessellate_faces(box, [model.faces(other)[0]], 1e-3)
    cyl = cylinder(model)
    a, b = model.faces(cyl)[:2]
    fine = model.tessellate_faces(cyl, [a], 1e-3)
    coarse = model.tessellate_faces(cyl, [b], 1e-1)
    with pytest.raises(arris.MeshWeldMismatchError) as caught:
        arris.weld([fine, coarse])
    assert caught.value.edge is not None


def test_weld_needs_meshes_of_one_model():
    with pytest.raises(ValueError):
        arris.weld([])
    one, two = arris.Model(), arris.Model()
    a, _ = one.primitive_box((0, 0, 0), (1, 1, 1))
    b, _ = two.primitive_box((0, 0, 0), (1, 1, 1))
    with pytest.raises(ValueError):
        arris.weld([one.tessellate(a, 1e-2), two.tessellate(b, 1e-2)])
