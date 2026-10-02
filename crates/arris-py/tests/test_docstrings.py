"""Every public item has a docstring, and every example in one runs.

The examples are the fenced ```python blocks the Rust doc comments carry
(`pyo3` makes them the objects' `__doc__`). A class, a module function and
every method of `Model` must hold one: an operation without an example is
unfinished (`.agents/rules/kernel.md` §API).
"""

import inspect
import re
import textwrap

import arris
import pytest

FENCE = re.compile(r"```python\n(.*?)```", re.DOTALL)

# Items whose example lives on the class that holds them: the handle and
# value types' members are read in the class example, and an exception's
# attributes are in its message.
PLAIN_VALUES = {"Body", "Shell", "Face", "Edge", "Vertex"}


def is_exception(obj):
    return isinstance(obj, type) and issubclass(obj, BaseException)


def public_members(cls):
    for name, member in vars(cls).items():
        if name.startswith("_"):
            continue
        if isinstance(member, (staticmethod, classmethod)):
            member = member.__func__
        yield name, member


def items():
    """`(qualified name, object, needs_example)` for every public item."""
    for name in arris.__all__:
        obj = getattr(arris, name)
        if name.startswith("__") or isinstance(obj, str):
            continue
        if is_exception(obj):
            yield name, obj, False
        elif inspect.isclass(obj):
            yield name, obj, True
            for member_name, member in public_members(obj):
                # Methods of `Model` need their own example; a value
                # class's members are covered by its class example.
                needs = name == "Model" and not isinstance(member, property)
                if inspect.isroutine(member) or isinstance(member, property) or hasattr(
                    member, "__get__"
                ):
                    yield f"{name}.{member_name}", member, needs
        else:
            yield name, obj, True


ITEMS = list(items())


@pytest.mark.parametrize("name,obj,needs", ITEMS, ids=[i[0] for i in ITEMS])
def test_every_public_item_is_documented(name, obj, needs):
    doc = inspect.getdoc(obj)
    assert doc and doc.strip(), f"{name} has no docstring"
    if needs:
        assert FENCE.search(doc), f"{name} has no ```python example"


@pytest.mark.parametrize(
    "name,obj", [(n, o) for n, o, _ in ITEMS if FENCE.search(inspect.getdoc(o) or "")]
)
def test_every_example_runs(name, obj):
    for block in FENCE.findall(inspect.getdoc(obj)):
        exec(compile(textwrap.dedent(block), f"<{name}>", "exec"), {})
