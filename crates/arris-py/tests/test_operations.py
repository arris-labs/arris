import math
import os
import signal
import threading
import time

import arris
import pytest


def box(model, lo=(0, 0, 0), hi=(10, 10, 10), **kw):
    return model.primitive_box(lo, hi, **kw)


def hole(model, x, y, **kw):
    return model.primitive_cylinder((x, y, -1), (0, 0, 1), 3, 12, **kw)


def faces(shapes):
    return [s for s in shapes if isinstance(s, arris.Face)]


def test_a_box_names_every_entity_by_its_role():
    model = arris.Model()
    body, record = box(model)
    assert isinstance(body, arris.Body) and isinstance(record, arris.Provenance)
    for coord in "XYZ":
        for side in ("Min", "Max"):
            made = record.generated_from(arris.Role("box", "Face", coord, side))
            assert len(faces(made)) == 1
    assert len(record.generated_from(arris.Role("box", "Body"))) == 1
    assert not record.deleted and not record.modified


def test_a_cylinder_has_a_wall_and_two_caps():
    model = arris.Model()
    _, record = model.primitive_cylinder((0, 0, 0), (0, 0, 1), 2, 5)
    for part in ("Wall", "BottomCap", "TopCap"):
        assert len(faces(record.generated_from(arris.Role("cylinder", part)))) == 1


def test_a_bolt_pattern_has_eight_walls_under_one_origin():
    model = arris.Model()
    body, whole = box(model, (0, 0, 0), (100, 100, 10))
    for i in range(8):
        a = i * math.pi / 4
        tool, made = hole(model, 50 + 35 * math.cos(a), 50 + 35 * math.sin(a))
        body, cut = model.cut(body, tool)
        whole = whole.then(made).then(cut)
    walls = faces(whole.generated_from(arris.Role("cylinder", "Wall")))
    assert len(set(walls)) == 8
    assert all(model.contains(w) for w in walls)
    for wall in walls:
        assert [origin for _, origin in whole.origins(wall)] == [
            arris.Role("cylinder", "Wall")
        ]


def test_the_booleans_return_new_bodies_and_leave_their_operands():
    model = arris.Model()
    a, _ = box(model)
    b, _ = box(model, (5, 5, 5), (15, 15, 15))
    results = [op(a, b) for op in (model.fuse, model.cut, model.common)]
    bodies = {a, b} | {body for body, _ in results}
    assert len(bodies) == 5
    assert all(model.contains(body) for body in bodies)


def test_transform_and_mirror_return_a_body_and_its_record():
    model = arris.Model()
    a, _ = box(model)
    moved, record = model.transform(a, (1, 2, 3), axis=(0, 0, 1), angle=math.pi / 2)
    mirrored, mirror_record = model.mirror(a, (0, 0, 0), (1, 0, 0))
    assert len({a, moved, mirrored}) == 3
    assert record and mirror_record


def test_fillet_and_chamfer_take_edges_found_through_provenance():
    model = arris.Model()
    a, record = box(model)
    edge = record.generated_from(arris.Role("box", "Edge", "Z", "Min", "Min"))
    edges = [e for e in edge if isinstance(e, arris.Edge)]
    assert len(edges) == 1
    for op, size in ((model.fillet, 1.0), (model.chamfer, 1.0)):
        body, made = op(a, edges, size)
        assert body != a and made


def test_a_foreign_handle_is_refused_before_the_kernel():
    one, other = arris.Model(), arris.Model()
    a, _ = box(one)
    b, _ = box(other)
    for call in (
        lambda: other.cut(a, b),
        lambda: other.fuse(b, a),
        lambda: other.transform(a),
        lambda: other.mirror(a, (0, 0, 0), (1, 0, 0)),
    ):
        with pytest.raises(arris.ForeignHandleError):
            call()
    # The same slot in each model: the kernel alone could not tell them apart.
    assert (a.index, a.generation) == (b.index, b.generation)


def test_a_freed_body_is_stale():
    model = arris.Model()
    kept, _ = box(model)
    freed, _ = box(model, (20, 20, 20), (30, 30, 30))
    assert model.retain([kept]) > 0
    assert model.contains(kept) and not model.contains(freed)
    with pytest.raises(arris.StaleHandleError):
        model.cut(kept, freed)


def test_a_boolean_that_selects_nothing_names_why():
    model = arris.Model()
    a, _ = box(model)
    far, _ = box(model, (50, 50, 50), (60, 60, 60))
    with pytest.raises(arris.OpDegenerateError) as raised:
        model.common(a, far)
    assert isinstance(raised.value, arris.OpError)
    assert "no material" in raised.value.reason
    assert set(raised.value.entities) == {a, far}


@pytest.mark.parametrize(
    "call",
    [
        lambda m: m.primitive_box((0, 0, 0), (0, 1, 1)),
        lambda m: m.primitive_box((0, 0, 0), (math.nan, 1, 1)),
        lambda m: m.primitive_cylinder((0, 0, 0), (0, 0, 0), 1, 1),
        lambda m: m.primitive_cylinder((0, 0, 0), (0, 0, 1), -1, 1),
        lambda m: m.primitive_cylinder((math.inf, 0, 0), (0, 0, 1), 1, 1),
    ],
)
def test_a_bad_argument_is_a_typed_error(call):
    with pytest.raises(arris.OpDegenerateError):
        call(arris.Model())


def test_a_set_token_stops_the_call_and_leaves_the_model_as_it_was():
    token = arris.Cancel()
    token.set()
    stopped, plain = arris.Model(), arris.Model()
    for model in (stopped, plain):
        a, _ = box(model)
        tool, _ = hole(model, 5, 5)
        if model is stopped:
            with pytest.raises(InterruptedError) as raised:
                model.cut(a, tool, cancel=token)
            assert isinstance(raised.value, arris.Interrupted)
            assert isinstance(raised.value, arris.ArrisError)
            assert raised.value.by == "poll"
    # Ids are not consumed by the stopped call.
    after_stopped, _ = box(stopped, (40, 40, 40), (50, 50, 50))
    after_plain, _ = box(plain, (40, 40, 40), (50, 50, 50))
    assert after_stopped.index == after_plain.index
    token.reset()
    assert not token.is_set()
    a, _ = box(stopped, (60, 60, 60), (70, 70, 70))
    tool, _ = hole(stopped, 65, 65)
    stopped.cut(a, tool, cancel=token)


def test_a_token_set_from_another_thread_stops_a_running_loop():
    model = arris.Model()
    a, _ = box(model, (0, 0, 0), (100, 100, 10))
    tool, _ = hole(model, 50, 50)
    token = arris.Cancel()
    threading.Timer(0.05, token.set).start()
    before = model.retain([a, tool])
    deadline = time.monotonic() + 30
    with pytest.raises(arris.Interrupted):
        while time.monotonic() < deadline:
            model.cut(a, tool, cancel=token)
    assert before == 0 and model.contains(a) and model.contains(tool)
    token.reset()
    model.cut(a, tool, cancel=token)


def test_a_budget_stops_at_the_same_step_twice():
    def stop_at(budget):
        model = arris.Model()
        a, _ = box(model)
        tool, _ = hole(model, 5, 5)
        with pytest.raises(arris.Interrupted) as raised:
            model.cut(a, tool, budget=budget)
        return raised.value.by, raised.value.steps, (a.index, tool.index)

    assert stop_at(3) == stop_at(3) == ("budget", 3, (0, 1))
    assert stop_at(0)[1] == 0
    model = arris.Model()
    a, _ = box(model)
    tool, _ = hole(model, 5, 5)
    model.cut(a, tool, budget=10_000)


@pytest.mark.skipif(os.name != "posix", reason="needs SIGINT")
def test_ctrl_c_is_a_keyboard_interrupt_and_the_model_stays_usable():
    model = arris.Model()
    a, _ = box(model, (0, 0, 0), (100, 100, 10))
    tool, _ = hole(model, 50, 50)
    threading.Timer(0.05, lambda: os.kill(os.getpid(), signal.SIGINT)).start()
    deadline = time.monotonic() + 30
    with pytest.raises(KeyboardInterrupt):
        while time.monotonic() < deadline:
            model.cut(a, tool)
    assert model.contains(a) and model.contains(tool)
    model.cut(a, tool)


def top_of(record):
    return faces(record.generated_from(arris.Role("box", "Face", "Z", "Max")))


def test_offset_faces_moves_a_face_and_records_it():
    model = arris.Model()
    a, record = box(model)
    taller, made = model.offset_faces(a, top_of(record), 2.0)
    assert taller != a and made
    assert abs(model.mass_properties(taller).volume - 1200) < 1e-6
    shorter, _ = model.offset_faces(a, top_of(record), -2.0)
    assert abs(model.mass_properties(shorter).volume - 800) < 1e-6
    assert model.check(taller, "full").ok


def refused(model, *args):
    with pytest.raises(arris.OpDegenerateError) as raised:
        model.offset_faces(*args)
    return raised.value


def test_offset_faces_refuses_its_inputs_by_name():
    model = arris.Model()
    a, record = box(model)
    other, other_record = box(model, (20, 0, 0), (30, 10, 10))
    top = top_of(record)
    assert "no faces" in refused(model, a, [], 1.0).reason
    assert "twice" in refused(model, a, top + top, 1.0).reason
    assert "not a face of the body" in refused(model, a, top_of(other_record), 1.0).reason
    for bad in (0.0, math.nan, math.inf):
        refused(model, a, top, bad)


def test_offset_faces_refuses_a_move_that_changes_topology():
    model = arris.Model()
    a, record = box(model)
    # Pulled through the opposite face: the walls would turn inside out.
    error = refused(model, a, top_of(record), -12.0)
    assert "vanish" in error.reason
    assert all(isinstance(e, (arris.Face, arris.Edge)) for e in error.entities)


def test_offset_faces_refuses_a_result_that_runs_into_itself():
    model = arris.Model()
    block, _ = box(model, (0, 0, 0), (40, 30, 10))
    pocket, _ = box(model, (10, 10, 5), (30, 20, 11))
    cut, _ = model.cut(block, pocket)
    floor = [
        f
        for f in model.faces(cut)
        if abs(model.face_frame(f).origin[2] - 5) < 1e-9
        and abs(model.face_frame(f).z[2] - 1) < 1e-9
    ]
    assert len(floor) == 1
    # Shallower is fine; pulled below the block's bottom the walls cross it.
    model.offset_faces(cut, floor, 2.0)
    error = refused(model, cut, floor, -6.0)
    assert "run into each other" in error.reason
    assert all(isinstance(e, arris.Face) for e in error.entities) and error.entities
    assert model.contains(cut)
    assert model.check(cut, "full").ok


def test_offset_faces_takes_handles_of_its_own_model_only():
    one, other = arris.Model(), arris.Model()
    a, record = box(one)
    b, _ = box(other)
    with pytest.raises(arris.ForeignHandleError):
        other.offset_faces(b, top_of(record), 1.0)


def test_offset_faces_honours_cancel_and_budget():
    model = arris.Model()
    a, record = box(model)
    token = arris.Cancel()
    token.set()
    with pytest.raises(arris.Interrupted):
        model.offset_faces(a, top_of(record), 1.0, cancel=token)
    with pytest.raises(arris.Interrupted):
        model.offset_faces(a, top_of(record), 1.0, budget=0)
    assert model.contains(a)
    token.reset()
    model.offset_faces(a, top_of(record), 1.0, cancel=token, budget=10_000)
