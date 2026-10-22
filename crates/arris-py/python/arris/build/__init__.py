"""The modelling layer: immutable solids with chained operations over the
binding `arris._arris` (ADR-0067).

A `Solid` is a value: each operation returns a new one, carrying the
provenance of every operation that made it and the tags its faces carry.
With no `model=` a constructor uses the module's default model,
`default_model()`.

```python
from arris.build import Solid

plate = Solid.box((0, 0, 0), (40, 30, 5))
hole = Solid.cylinder((20, 15, -1), (0, 0, 1), 4, 7)
part = plate.tag("plate", plate.faces()).cut(hole)
assert len(part.tagged("plate")) == 6
assert part.model.surface(part.faces().created_by(hole).one()).kind == "cylinder"
```
"""

from ._selection import Selection, SelectionError
from ._solid import Solid, default_model

__all__ = ["Selection", "SelectionError", "Solid", "default_model"]
