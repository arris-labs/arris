import arris
import pytest


def test_a_role_reads_as_kind_part_and_fields():
    top = arris.Role("box", "Face", "Z", "Max")
    assert (top.kind, top.part, top.fields) == ("box", "Face", ("Z", "Max"))
    assert str(top) == "box:Face(Z, Max)"
    side = arris.Role("extrude", "Side", 0, 3)
    assert side.fields == (0, 3)
    assert arris.Role("cylinder", "Wall").fields == ()


def test_roles_are_values():
    a = arris.Role("box", "Face", "Z", "Max")
    assert a == arris.Role("box", "Face", "Z", "Max")
    assert a != arris.Role("box", "Face", "Z", "Min")
    assert len({a, arris.Role("box", "Face", "Z", "Max")}) == 1


def test_a_consumer_role_carries_its_namespace_and_key():
    role = arris.Role.consumer(7, 42)
    assert (role.namespace, role.key) == (7, 42)
    assert (role.kind, role.part, role.fields) == ("consumer", "Key", (7, 42))
    assert role == arris.Role("consumer", "Key", 7, 42)
    assert str(role) == "consumer:7/42"
    assert arris.Role("box", "Body").namespace is None


@pytest.mark.parametrize(
    "request_",
    [
        ("sphere", "Wall"),
        ("box", "Wall"),
        ("box", "Face", "Z"),
        ("box", "Face", "W", "Max"),
        ("box", "Face", "Z", "Up"),
        ("box", "Body", "Z"),
        ("extrude", "Side", 0),
        ("consumer", "Key", 2**40, 1),
    ],
)
def test_a_request_that_is_no_role_is_a_value_error(request_):
    with pytest.raises(ValueError):
        arris.Role(*request_)
