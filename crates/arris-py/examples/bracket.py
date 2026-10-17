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

LABELS = ["plate", "boss", "bore", "bolt holes", "counterbores", "bend fillet", "rim fillets", "chamfer"]


class Labelled:
    """A body and the operation that made each of its faces, followed through
    every operation's provenance."""

    def __init__(self, model, body, record, label):
        self.model, self.body = model, body
        self.labels = {f: label for f in model.faces(body)}
        self.record = record

    def step(self, body, record, label, tools=()):
        """The result of one operation: a face is labelled from the face it
        was modified from, else from the tool face it was generated from,
        else `label`; a face the record leaves alone keeps its own."""
        known = dict(self.labels)
        for tool in tools:
            known.update(tool.labels)
        inherited = {}
        for relation in (record.modified, record.generated):
            for origin, outputs in relation:
                if origin in known:
                    for output in outputs:
                        inherited.setdefault(output, known[origin])
        self.labels = {
            face: inherited.get(face, self.labels.get(face, label))
            for face in self.model.faces(body)
        }
        self.body, self.record = body, self.record.then(record)
        return self


def build(model):
    """The bracket as a `Labelled`: its body, its labels and its provenance."""
    # The plate: an L profile in the xz plane, extruded along y.
    profile = arris.Profile(
        arris.Loop.polygon([(0, 0), (60, 0), (60, 8), (8, 8), (8, 50), (0, 50)]),
        origin=(0, 0, 0),
        normal=(0, -1, 0),
        x_axis=(1, 0, 0),
    )
    plate, record = model.extrude(profile, (0, 1, 0), 40)
    part = Labelled(model, plate, record, "plate")

    # A boss on the upright leg, and the pattern cut in one call.
    boss, boss_record = model.primitive_cylinder((8, 20, 32), (1, 0, 0), 14, 12)
    body, record = model.fuse(part.body, boss)
    part.step(body, record, "boss", [Labelled(model, boss, boss_record, "boss")])

    tools = []
    bore, bore_record = model.primitive_cylinder((-1, 20, 32), (1, 0, 0), 8, 23)
    tools.append(Labelled(model, bore, bore_record, "bore"))
    for y in (8, 32):
        hole, hole_record = model.primitive_cylinder((40, y, -1), (0, 0, 1), 3.5, 10)
        tools.append(Labelled(model, hole, hole_record, "bolt holes"))
        well, well_record = model.primitive_cylinder((40, y, 5), (0, 0, 1), 6, 4)
        tools.append(Labelled(model, well, well_record, "counterbores"))
    body, record = model.cut_many(part.body, [t.body for t in tools])
    part.step(body, record, "bolt holes", tools)

    # Fillets and the chamfer, each edge chosen by what made its faces.
    model.check(part.body, "full")
    [bend] = [
        e
        for e in part.record.generated_from(arris.Role("extrude", "Rise", 0, 3))
        if isinstance(e, arris.Edge)
    ]
    body, record = model.fillet(part.body, [bend], 4)
    part.step(body, record, "bend fillet")

    rims = edges_between(part, "plate", "counterbores")
    body, record = model.fillet(part.body, rims, 0.8)
    part.step(body, record, "rim fillets")

    [lip] = edges_between(part, "boss", "boss")
    body, record = model.chamfer(part.body, [lip], 1.5)
    part.step(body, record, "chamfer")
    return part


def edges_between(part, a, b):
    """The edges of `part` whose two faces are labelled `a` and `b`."""
    found = []
    for edge in part.model.edges(part.body):
        faces = part.model.faces_of(part.body, edge)
        if len(faces) == 2 and sorted(part.labels[f] for f in faces) == sorted([a, b]):
            if faces[0] != faces[1]:
                found.append(edge)
    return found


def write(part, out_dir, chord=0.02):
    """Writes the STEP, the mesh and the labels of `part` into `out_dir`."""
    model, out = part.model, Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "bracket.step").write_text(model.write_step([part.body]))
    mesh = model.tessellate(part.body, chord)
    header = struct.pack("<II", mesh.n_positions, mesh.n_triangles)
    (out / "bracket.mesh").write_bytes(header + mesh.positions + mesh.triangles)
    triangle_label = [0] * mesh.n_triangles
    for face, start, stop in mesh.faces:
        triangle_label[start:stop] = [LABELS.index(part.labels[face])] * (stop - start)
    document = {"names": LABELS, "triangles": triangle_label}
    (out / "bracket.labels.json").write_text(json.dumps(document))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    write(build(arris.Model()), sys.argv[1])
