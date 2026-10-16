"""Profiles lofted through placed sections: `BRepOffsetAPI_ThruSections`.

The `loft` op of the recipe grammar (`recipe.py` documents it) reaches
here with its sections already built as faces, each a profile where it
stands. Loop `k` of every section is lofted on its own (the outer wire,
then each hole's, in the order the face lists them), so the sections must
agree on their loop count; an open loft is then sewn with the first and
the last section's faces as its caps, a closed one repeats the first
section and has none.

`ruled` builds each span between consecutive sections straight; smooth
builds one B-spline surface through all of them. `CheckCompatibility` is
on: with it off `ThruSections` does not split a wire with fewer edges and
lofts the first edge of each (one face, no solid). The compatible-wires
pass it then runs re-starts a wire at the vertex nearest the previous
section's start and cuts the wire with fewer edges where the other's
vertices project onto it (plans/loft step 1, ADR-0064), so fixtures place
their start points where that pass keeps them.
"""

from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCP.ShapeFix import ShapeFix_Solid
from OCP.TopAbs import TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape

from . import OracleError
from .sweep import _wires


def _shell_of_loop(wires: list, ruled: bool) -> TopoDS_Shape:
    """One loop's lofted shell: the wires added in order, built."""
    mk = BRepOffsetAPI_ThruSections(False, ruled, 1e-6)
    mk.CheckCompatibility(True)
    for wire in wires:
        mk.AddWire(TopoDS.Wire(wire))
    mk.Build()
    if not mk.IsDone():
        raise OracleError("loft: Open CASCADE reports not done")
    return mk.Shape()


def through(sections: list[TopoDS_Shape], kind: str, closed: bool) -> TopoDS_Shape:
    """`sections`, faces, lofted to a solid. `kind` is `"ruled"` or
    `"smooth"`; `closed` repeats the first section and caps nothing."""
    if kind not in ("ruled", "smooth"):
        raise OracleError(f"loft kind must be \"ruled\" or \"smooth\", not {kind!r}")
    if len(sections) < (3 if closed else 2):
        raise OracleError("loft needs two sections, a closed one three")
    loops = []
    for face in sections:
        outer, holes = _wires(face)
        loops.append([outer] + holes)
    if any(len(l) != len(loops[0]) for l in loops):
        raise OracleError("loft: the sections do not have the same number of loops")
    ruled = kind == "ruled"
    count = len(loops[0])
    shells = []
    for k in range(count):
        wires = [l[k] for l in loops]
        if closed:
            wires.append(wires[0])
        shells.append(_shell_of_loop(wires, ruled))
    if closed:
        if count != 1:
            raise OracleError("loft: a closed loft with a holed section is not a fixture the oracle builds")
        solid = BRepBuilderAPI_MakeSolid(TopoDS.Shell(shells[0])).Solid()
    else:
        sewing = BRepBuilderAPI_Sewing(1e-6)
        for shell in shells:
            sewing.Add(shell)
        sewing.Add(sections[0])
        sewing.Add(sections[-1])
        sewing.Perform()
        found = []
        explorer = TopExp_Explorer(sewing.SewedShape(), TopAbs_SHELL)
        while explorer.More():
            found.append(TopoDS.Shell(explorer.Current()))
            explorer.Next()
        if len(found) != 1:
            raise OracleError(f"loft: the sewn pieces make {len(found)} shells, not one")
        solid = BRepBuilderAPI_MakeSolid(found[0]).Solid()
    fix = ShapeFix_Solid(solid)
    fix.Perform()
    return fix.Solid()
