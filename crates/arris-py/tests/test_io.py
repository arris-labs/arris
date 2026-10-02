import json
import struct

import arris
import pytest
from conftest import oracle, tolerances

FIXTURE = "blend/box-edge-fillet"


def filleted_box(model):
    cube, record = model.primitive_box((0, 0, 0), (2, 2, 2))
    edges = record.generated_from(arris.Role("box", "Edge", "Z", "Max", "Max"))
    edges = [e for e in edges if isinstance(e, arris.Edge)]
    return model.fillet(cube, edges, 0.2)


def test_a_step_round_trip_of_a_filleted_body_keeps_its_volume():
    model = arris.Model()
    body, _ = filleted_box(model)
    text = model.write_step([body])
    assert text.startswith("ISO-10303-21;")
    assert model.write_step([body]) == text, "deterministic"

    back = arris.Model()
    read = back.read_step(text)
    [solid] = read.solids
    assert solid.ok and solid.refusal is None
    assert solid.file_id > 0 and solid.instance == 0
    assert back.check(solid.body, "full")
    held = tolerances(FIXTURE)
    expected = oracle(FIXTURE)["volume"]
    volume = back.mass_properties(solid.body).volume
    assert volume == pytest.approx(expected, rel=held["volume_rel"])
    assert volume == pytest.approx(model.mass_properties(body).volume, rel=1e-9)
    # Every entity of the body was read from a file entity.
    assert solid.provenance.generated


def test_the_product_tree_comes_with_the_solids():
    model = arris.Model()
    a, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
    b, _ = model.primitive_box((5, 0, 0), (6, 1, 1))
    read = arris.Model().read_step(model.write_step([a, b]))
    assert len(read.solids) == 2
    [root] = read.products
    assert root.name == "arris" and root.solids == [0, 1] and root.children == []
    identity = [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]
    assert root.placement == identity
    assert root.placement_refusal is None and read.face_colours == []


def test_a_step_read_converts_to_the_unit_asked():
    model = arris.Model()
    body, _ = model.primitive_box((0, 0, 0), (25.4, 25.4, 25.4))
    text = model.write_step([body])
    inches = arris.Model()
    [solid] = inches.read_step(text, length_unit="in").solids
    assert inches.mass_properties(solid.body).volume == pytest.approx(1.0, rel=1e-12)
    with pytest.raises(ValueError, match="unknown length unit"):
        inches.read_step(text, length_unit="furlong")


def test_the_step_errors_are_typed():
    model = arris.Model()
    with pytest.raises(arris.StepNoBodiesError):
        model.write_step([])
    with pytest.raises(arris.StepParseError) as caught:
        model.read_step("this is not a STEP file")
    assert isinstance(caught.value, arris.StepError)
    assert caught.value.line == 1 and caught.value.column >= 1
    assert caught.value.instance is None and caught.value.detail
    other, _ = arris.Model().primitive_box((0, 0, 0), (1, 1, 1))
    with pytest.raises(arris.ForeignHandleError):
        model.write_step([other])


def test_a_step_read_stopped_by_a_budget_leaves_the_model_as_it_was():
    model = arris.Model()
    body, _ = filleted_box(model)
    text = model.write_step([body])
    target = arris.Model()
    before = target.to_native_json()
    with pytest.raises(arris.Interrupted) as stopped:
        target.read_step(text, budget=1)
    assert stopped.value.by == "budget"
    assert target.to_native_json() == before


def test_body_bytes_cross_to_a_second_model_with_their_record():
    a = arris.Model()
    plate, plate_record = a.primitive_box((0, 0, 0), (100, 100, 10))
    data = a.write_body(plate, plate_record)
    assert data[:8] == b"ARRISBDY"
    assert a.write_body(plate, plate_record) == data

    b = arris.Model()
    imported = b.read_body(data)
    assert imported.version >= 1
    assert imported.foreign() == []
    assert b.mass_properties(imported.body).volume == pytest.approx(
        a.mass_properties(plate).volume, rel=1e-12
    )
    assert b.check(imported.body, "full")

    record = imported.translated()
    assert len(record.generated) == len(plate_record.generated)
    top = record.generated_from(arris.Role("box", "Face", "Z", "Max"))
    [face] = [f for f in top if isinstance(f, arris.Face)]
    assert b.contains(face) and face in b.faces(imported.body)
    # Its handles are the second model's: the first refuses them.
    with pytest.raises(arris.ForeignHandleError):
        a.contains(face)
    with pytest.raises(arris.ForeignHandleError):
        a.write_body(imported.body)
    with pytest.raises(arris.ForeignHandleError):
        b.write_body(imported.body, plate_record)


def test_a_record_naming_inputs_has_no_handles_for_them():
    a = arris.Model()
    plate, _ = a.primitive_box((0, 0, 0), (10, 10, 10))
    tool, _ = a.primitive_cylinder((5, 5, -1), (0, 0, 1), 2, 12)
    holed, record = a.cut(plate, tool)
    b = arris.Model()
    imported = b.read_body(a.write_body(holed, record))
    assert imported.foreign(), "the cut's inputs are outside the body"
    with pytest.raises(ValueError, match="no handle in this model"):
        imported.translated()
    assert b.mass_properties(imported.body).volume == pytest.approx(
        a.mass_properties(holed).volume, rel=1e-12
    )


def test_body_json_is_the_same_body_as_text():
    a = arris.Model()
    body, record = a.primitive_box((0, 0, 0), (1, 2, 3))
    text = a.write_body_json(body, record)
    assert json.loads(text)["magic"] == "ARRISBDY"
    b = arris.Model()
    imported = b.read_body_json(text)
    assert b.mass_properties(imported.body).volume == pytest.approx(6.0, rel=1e-12)
    assert b.write_body(imported.body, imported.translated()) == a.write_body(body, record)


def test_body_reading_errors_are_typed_and_leave_the_model_as_it_was():
    a = arris.Model()
    body, record = a.primitive_box((0, 0, 0), (1, 1, 1))
    data = a.write_body(body, record)
    b = arris.Model()
    before = b.to_native_json()
    with pytest.raises(arris.BodyMagicError):
        b.read_body(b"not a body at all")
    with pytest.raises(arris.BodyDecodeError):
        b.read_body(data[: len(data) // 2])
    with pytest.raises(arris.BodyMagicError):
        b.read_body_json('{"magic": "nope"}')
    with pytest.raises(arris.Interrupted):
        b.read_body(data, budget=0)
    assert b.to_native_json() == before


def test_the_native_format_round_trips_a_model_to_the_same_dump():
    model = arris.Model()
    filleted_box(model)
    data = model.to_native()
    text = model.to_native_json()
    assert model.to_native() == data
    assert json.loads(text)["version"] >= 1

    again = arris.Model.from_native(data)
    assert again != model, "a model of its own"
    assert again.to_native() == data
    assert arris.Model.from_native_json(text).to_native_json() == text

    with pytest.raises(arris.NativeDecodeError):
        arris.Model.from_native(b"")
    with pytest.raises(arris.NativeVersionError):
        arris.Model.from_native(b"\x00\x01garbage")
    with pytest.raises(arris.NativeDecodeError):
        arris.Model.from_native_json("{")
    version = json.dumps({"version": 999, "model": {}})
    with pytest.raises(arris.NativeVersionError) as caught:
        arris.Model.from_native_json(version)
    assert caught.value.found == 999 and caught.value.supported >= 1


def test_the_mesh_writers_write_the_meshes_triangles():
    model = arris.Model()
    cube, _ = model.primitive_box((0, 0, 0), (1, 2, 3))
    cylinder, _ = model.primitive_cylinder((5, 0, 0), (0, 0, 1), 1, 2)
    meshes = [model.tessellate(cube, 1e-3), model.tessellate(cylinder, 1e-2)]
    triangles = sum(m.n_triangles for m in meshes)

    binary = arris.stl_binary(meshes, "two")
    assert binary[:3] == b"two"
    assert struct.unpack("<I", binary[80:84])[0] == triangles
    assert len(binary) == 84 + 50 * triangles

    ascii_ = arris.stl_ascii(meshes, "two")
    assert ascii_.startswith("solid two") and ascii_.count("facet normal") == triangles
    assert arris.stl_ascii(meshes, "two") == ascii_

    text = arris.obj(meshes)
    assert sum(1 for line in text.splitlines() if line.startswith("v ")) == sum(
        m.n_positions for m in meshes
    )
    assert sum(1 for line in text.splitlines() if line.startswith("f ")) == triangles
