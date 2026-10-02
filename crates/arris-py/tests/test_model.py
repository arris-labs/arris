import arris
import pytest


def test_models_are_equal_only_to_themselves():
    a, b = arris.Model(), arris.Model()
    assert a == a and a != b
    assert len({a, b, a}) == 2


def test_the_handle_errors_are_arris_errors():
    for name in ("ForeignHandleError", "StaleHandleError", "ModelPoisonedError"):
        assert issubclass(getattr(arris, name), arris.ArrisError)


def test_retain_of_nothing_frees_nothing_in_an_empty_model():
    assert arris.Model().retain([]) == 0


def test_contains_takes_a_handle_and_nothing_else():
    with pytest.raises(TypeError):
        arris.Model().contains(3)
