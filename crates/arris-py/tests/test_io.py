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


def test_a_step_write_declares_the_unit_and_reads_back_in_it():
    model = arris.Model()
    body, _ = model.primitive_box((0, 0, 0), (0.1, 0.2, 0.3))
    text = model.write_step([body], length_unit="m")
    assert "SI_UNIT($,.METRE.)" in text
    assert model.write_step([body]) == model.write_step([body], length_unit="mm")
    metres = arris.Model()
    [solid] = metres.read_step(text, length_unit="m").solids
    assert metres.mass_properties(solid.body).volume == pytest.approx(0.006, rel=1e-12)
    millimetres = arris.Model()
    [solid] = millimetres.read_step(text).solids
    assert millimetres.mass_properties(solid.body).volume == pytest.approx(6e6, rel=1e-12)
    with pytest.raises(ValueError, match="unknown length unit"):
        model.write_step([body], length_unit="furlong")


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


FACE_BASED = """ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('','',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#2),#3);
#2=FACE_BASED_SURFACE_MODEL('',(#4));
#3=( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#30)) GLOBAL_UNIT_ASSIGNED_CONTEXT((#31,#32)) REPRESENTATION_CONTEXT('','') );
#4=CONNECTED_FACE_SET('',(#5));
#5=ADVANCED_FACE('',(#6),#20,.T.);
#6=FACE_OUTER_BOUND('',#7,.T.);
#7=EDGE_LOOP('',(#8,#9,#10,#11));
#8=ORIENTED_EDGE('',*,*,#12,.T.);
#9=ORIENTED_EDGE('',*,*,#13,.T.);
#10=ORIENTED_EDGE('',*,*,#14,.T.);
#11=ORIENTED_EDGE('',*,*,#15,.T.);
#12=EDGE_CURVE('',#40,#41,#50,.T.);
#13=EDGE_CURVE('',#41,#42,#51,.T.);
#14=EDGE_CURVE('',#42,#43,#52,.T.);
#15=EDGE_CURVE('',#43,#40,#53,.T.);
#20=PLANE('',#21);
#21=AXIS2_PLACEMENT_3D('',#22,#23,#24);
#22=CARTESIAN_POINT('',(0.,0.,0.));
#23=DIRECTION('',(0.,0.,1.));
#24=DIRECTION('',(1.,0.,0.));
#30=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-05),#31,'distance_accuracy_value','');
#31=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );
#32=( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );
#40=VERTEX_POINT('',#44);
#41=VERTEX_POINT('',#45);
#42=VERTEX_POINT('',#46);
#43=VERTEX_POINT('',#47);
#44=CARTESIAN_POINT('',(0.,0.,0.));
#45=CARTESIAN_POINT('',(4.,0.,0.));
#46=CARTESIAN_POINT('',(4.,3.,0.));
#47=CARTESIAN_POINT('',(0.,3.,0.));
#50=LINE('',#44,#60);
#51=LINE('',#45,#61);
#52=LINE('',#46,#62);
#53=LINE('',#47,#63);
#60=VECTOR('',#64,4.);
#61=VECTOR('',#65,3.);
#62=VECTOR('',#66,4.);
#63=VECTOR('',#67,3.);
#64=DIRECTION('',(1.,0.,0.));
#65=DIRECTION('',(0.,1.,0.));
#66=DIRECTION('',(-1.,0.,0.));
#67=DIRECTION('',(0.,-1.,0.));
ENDSEC;
END-ISO-10303-21;
"""


def test_a_surface_model_reads_as_a_sheet_with_its_surface_properties():
    model = arris.Model()
    [sheet] = model.read_step(FACE_BASED).solids
    assert sheet.ok and sheet.refusal is None
    assert model.body_kind(sheet.body) == "sheet"
    props = model.surface_properties(sheet.body)
    assert props.area == pytest.approx(12, rel=1e-12)
    assert props.centroid == pytest.approx((2, 1.5, 0), abs=1e-12)
    with pytest.raises(arris.OpDegenerateError):
        model.mass_properties(sheet.body)
    solid, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
    assert model.body_kind(solid) == "solid"


def test_a_sheet_is_written_as_a_surface_model_and_read_back_as_a_sheet():
    model = arris.Model()
    sketch = arris.Profile(arris.Loop.polygon([(0, 0), (4, 0), (4, 3), (0, 3)]))
    sheet, _ = model.planar_face(sketch)
    text = model.write_step([sheet])
    assert "SHELL_BASED_SURFACE_MODEL" in text

    back = arris.Model()
    [solid] = back.read_step(text).solids
    assert solid.ok and back.body_kind(solid.body) == "sheet"
    assert back.surface_properties(solid.body).area == pytest.approx(12, abs=1e-9)
