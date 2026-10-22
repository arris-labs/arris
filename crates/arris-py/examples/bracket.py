"""A motor-mount bracket: an L plate, a boss, a bolt pattern, a bore, fillets
and a chamfer; every final face labelled by the operation that made it.

    python bracket.py OUT_DIR

writes `bracket.step`, `bracket.mesh` (two little-endian u32 counts, then the
mesh's f64 positions and u32 triangles) and `bracket.labels.json` (the label
names and each triangle's label) into OUT_DIR. `tools/showcase.sh` renders them.
"""

import json
import struct
import sys
from pathlib import Path

import arris
from arris.build import Solid, Workplane

LABELS = ["plate", "boss", "bore", "bolt holes", "counterbores", "bend fillet", "rim fillets", "chamfer"]


def labelled(solid, name):
    """`solid` with every face no tag names yet tagged `name`: the faces the
    last operation made, since tags follow the rest through provenance."""
    seen = set().union(*solid.tags.values()) if solid.tags else set()
    return solid.tag(name, [face for face in solid.faces() if face not in seen])


def build():
    """The bracket as a `Solid`: its faces tagged by the operation that made
    them, each tag one of `LABELS`."""
    # The plate: an L profile in the xz plane, extruded along y.
    plate = Workplane.XZ().polygon([(0, 0), (60, 0), (60, 8), (8, 8), (8, 50), (0, 50)]).extrude(-40)
    part = labelled(plate, "plate")

    # A boss on the upright leg, and the pattern cut in one call.
    boss = labelled(Solid.cylinder((8, 20, 32), (1, 0, 0), 14, 12), "boss")
    part = labelled(part.fuse(boss), "boss")

    bore = labelled(Solid.cylinder((-1, 20, 32), (1, 0, 0), 8, 23), "bore")
    holes = [labelled(Solid.cylinder((40, y, -1), (0, 0, 1), 3.5, 10), "bolt holes") for y in (8, 32)]
    wells = [labelled(Solid.cylinder((40, y, 5), (0, 0, 1), 6, 4), "counterbores") for y in (8, 32)]
    part = labelled(part.cut(bore, holes[0], wells[0], holes[1], wells[1]), "bolt holes")

    # Fillets and the chamfer, each edge chosen by what made its faces.
    part.check("full")
    bend = part.edges().from_role(arris.Role("extrude", "Rise", 0, 3)).one()
    part = labelled(part.fillet([bend], 4), "bend fillet")

    faces = part.faces()
    rims = faces.tagged("plate").edges() & faces.tagged("counterbores").edges()
    part = labelled(part.fillet(rims, 0.8), "rim fillets")

    faces = part.faces().tagged("boss")
    lip = faces.kind("plane").edges() & faces.kind("cylinder").edges()
    return labelled(part.chamfer([lip.one()], 1.5), "chamfer")


def write(part, out_dir, chord=0.02):
    """Writes the STEP, the mesh and the labels of `part` into `out_dir`."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    part.export_step(out / "bracket.step")
    mesh = part.mesh(chord)
    header = struct.pack("<II", mesh.n_positions, mesh.n_triangles)
    (out / "bracket.mesh").write_bytes(header + mesh.positions + mesh.triangles)
    label_of = {face: LABELS.index(name) for name, faces in part.tags.items() for face in faces}
    triangle_label = [0] * mesh.n_triangles
    for face, start, stop in mesh.faces:
        triangle_label[start:stop] = [label_of[face]] * (stop - start)
    document = {"names": LABELS, "triangles": triangle_label}
    (out / "bracket.labels.json").write_text(json.dumps(document))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    write(build(), sys.argv[1])
