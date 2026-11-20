import math

import arris
import pytest

# The families: one per kernel error enum, each under ArrisError. A variant
# class is named for its family's prefix and derives from the family.
FAMILIES = {
    "Op": "OpError",
    "Geom": "GeomError",
    "Fit": "FitError",
    "Topo": "TopoError",
    "Step": "StepError",
    "Body": "BodyError",
    "Mesh": "MeshError",
}


def exception_classes():
    for name in arris.__all__:
        obj = getattr(arris, name)
        if isinstance(obj, type) and issubclass(obj, BaseException):
            yield name, obj


def test_every_exception_is_an_arris_error():
    classes = dict(exception_classes())
    assert len(classes) > 40
    for name, cls in classes.items():
        assert issubclass(cls, arris.ArrisError), name


def test_every_variant_derives_from_its_family():
    classes = dict(exception_classes())
    for family, base in FAMILIES.items():
        assert classes[base].__bases__ == (arris.ArrisError,)
        variants = [
            cls
            for name, cls in classes.items()
            if name.startswith(family) and name != base and name.endswith("Error")
        ]
        assert variants, family
        for cls in variants:
            assert cls.__bases__ == (classes[base],), cls.__name__


def test_the_shared_conditions_are_one_class_each():
    # A NotFound is stale whichever operation raised it; a stop is
    # Interrupted whichever operation was stopped.
    assert arris.StaleHandleError.__bases__ == (arris.ArrisError,)
    # Also the builtin, so `except InterruptedError` catches a stop.
    assert issubclass(arris.Interrupted, arris.ArrisError)
    assert issubclass(arris.Interrupted, InterruptedError)


def test_the_exceptions_say_where_they_live():
    assert arris.OpUnsupportedError.__module__ == "arris"


def test_revolve_names_an_arc_that_touches_the_axis_inside_its_range():
    profile = arris.Profile(
        arris.Loop.path(
            (1, -1),
            [arris.Segment.line((1, 1)), arris.Segment.arc(to=(1, -1), via=(0, 0))],
        )
    )
    with pytest.raises(arris.ArcTouchesAxis) as error:
        arris.Model().revolve(profile, (0, 0, 0), (0, 1, 0), math.pi)
    assert isinstance(error.value, arris.OpDegenerateError)
    assert (error.value.loop_index, error.value.segment) == (0, 1)
    assert error.value.entities == []
    assert "touches the revolve axis inside its range" in error.value.reason
