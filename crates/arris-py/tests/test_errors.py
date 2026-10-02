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
    assert arris.Interrupted.__bases__ == (arris.ArrisError,)


def test_the_exceptions_say_where_they_live():
    assert arris.OpUnsupportedError.__module__ == "arris"
