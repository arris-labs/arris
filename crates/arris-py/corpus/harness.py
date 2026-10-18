"""Runs a script-corpus part: `part.py` defines `build(run)` and calls each
operation as `run.stage("<name>", lambda: run.model.<op>(...))`. The harness
holds every stage to the checker at `full`, records what it did (`ok`, or
the typed refusal as the binding raised it) and keeps the body of each
stage that was built, for the test to hold to the oracle's checkpoints.

Not part of the wheel: the corpus is outside `python/`.
"""

import importlib.util
import json
import re
import struct
from pathlib import Path

import arris

CORPUS = Path(__file__).resolve().parent

# The attributes a refusal is recorded by, where the exception class has
# them: `OpDegenerateError.reason`, the two kinds of an `*UnsupportedError`,
# the `detail` of the rest. An entity handle is never recorded.
ATTRIBUTES = ("reason", "a_kind", "b_kind", "detail")


# The one undecided row a built body may carry: a face pair of two NURBS
# faces, which the surface intersector has no form for yet (the corpus
# runner leaves the same ones, `arris_debug::corpus::check_leaving_nurbs_pairs`).
NURBS_PAIR = re.compile(r"^(S5|B1)\? .*\(NURBS\) against .*\(NURBS\) is not decided$")


class Fault(Exception):
    """What a part met that is a kernel fault and never an outcome: an
    `OpInternalError`, a poisoned model, a stage the checker finds red or
    cannot decide. The test fails until the fault is a `regression/`
    fixture and the part is listed in `waits`."""


class Ended(Exception):
    """A refused stage that was not a `finish` one: the part stops here."""


def refusal(error):
    """The outcome a refusal is recorded as: its class and its attributes."""
    out = {"refused": type(error).__name__}
    for name in ATTRIBUTES:
        if hasattr(error, name):
            out[name] = getattr(error, name)
    return out


class Run:
    """One run of a part: its model, its stages and their outcomes."""

    def __init__(self, stages):
        self.model = arris.Model()
        self.outcome = {name: "not reached" for name in stages}
        self.bodies = {}
        # False once a `finish` stage was refused: a later stage is built on
        # the body before it, so the oracle's checkpoint there (which has
        # the finish) is not the one to compare it with.
        self.comparable = {}
        self._finished_short = False
        self.last = None
        self.labelled = None

    def stage(self, name, op, finish=False):
        """Runs `op()`, which returns a `(body, provenance)` pair, as stage
        `name`. Returns that pair, or `None` for a refused `finish` stage;
        any other refusal ends the part."""
        if name not in self.outcome:
            raise KeyError(f"{name!r} is not a stage of the part's oracle.json checkpoints")
        try:
            made = op()
        except (arris.OpInternalError, arris.ModelPoisonedError) as error:
            raise Fault(f"{name}: {type(error).__name__}: {error}") from error
        except arris.ArrisError as error:
            self.outcome[name] = refusal(error)
            if finish:
                self._finished_short = True
                return None
            raise Ended() from error
        body = made[0]
        report = self.model.check(body, "full")
        undecided = [row for row in report.unchecked if not NURBS_PAIR.match(str(row))]
        if not report or undecided:
            raise Fault(f"{name}: the checker at full: {report}, unchecked {undecided}")
        self.outcome[name] = "ok"
        self.bodies[name] = body
        self.comparable[name] = not self._finished_short
        self.last = body
        return made

    def edge_points(self, body, chord=0.05):
        """Every edge of `body` with the polyline of points it is drawn as
        (`tessellate`'s edge ranges): what a script asks of an edge's
        place, since the binding gives a handle and no coordinates."""
        mesh = self.model.tessellate(body, chord)
        positions = struct.unpack(f"<{mesh.n_positions * 3}d", mesh.positions)
        indices = struct.unpack(f"<{len(mesh.edge_indices) // 4}I", mesh.edge_indices)
        return {
            edge: [tuple(positions[3 * i : 3 * i + 3]) for i in indices[start:stop]]
            for edge, start, stop in mesh.edges
        }

    def plane(self, face):
        """`(origin, normal)` of a planar face, `None` for any other."""
        try:
            frame = self.model.face_frame(face)
        except arris.OpDegenerateError:
            return None
        return frame.origin, frame.z


def load(slug):
    """The module of `corpus/<slug>/part.py`, and its checkpoint names."""
    directory = CORPUS / slug
    spec = importlib.util.spec_from_file_location(f"corpus_{slug.replace('-', '_')}", directory / "part.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    stages = json.loads((directory / "oracle.json").read_text())["checkpoints"]
    return module, stages


def run_part(slug):
    """Runs the part and returns its `Run`."""
    module, stages = load(slug)
    run = Run(stages)
    try:
        module.build(run)
    except Ended:
        pass
    return run


def slugs():
    """Every part, in name order."""
    return sorted(p.parent.name for p in CORPUS.glob("*/part.py"))


class Labelled:
    """A body and the tag/label on each of its faces, tracked through
    operations by provenance."""

    def __init__(self, model, body, record=None, label=None):
        self.model, self.body = model, body
        if isinstance(label, str):
            self.labels = {f: label for f in model.faces(body)}
        elif isinstance(label, dict):
            self.labels = dict(label)
        elif callable(label):
            self.labels = {f: label(f) for f in model.faces(body)}
        else:
            self.labels = {}
        self.record = record

    def tag(self, face, label):
        """Tags a face."""
        self.labels[face] = label
        return self

    def tag_many(self, faces, label):
        """Tags multiple faces."""
        for f in faces:
            self.labels[f] = label
        return self

    def step(self, body, record, label=None, tools=()):
        """Updates the body and face labels after an operation. A face inherits
        its label from the face it was modified from, else from the tool face
        it was generated from, else keeps its existing label, else `label`."""
        known = dict(self.labels)
        for tool in tools:
            if isinstance(tool, Labelled):
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
        self.body = body
        self.record = self.record.then(record) if self.record is not None else record
        return self

    def assert_tagged(self, tags=None, chord=0.05):
        """Asserts that every face is tagged, that the tag set matches `tags`
        (if provided), that every face has a non-empty triangle range in
        tessellation, and that all tags reach the mesh triangles."""
        faces = set(self.model.faces(self.body))
        assert set(self.labels) == faces, f"labels mismatch: {set(self.labels)} vs {faces}"
        if tags is not None:
            assert set(self.labels.values()) == set(tags), (
                f"tag set mismatch: {set(self.labels.values())} vs {set(tags)}"
            )
        mesh = self.model.tessellate(self.body, chord)
        mesh_faces = {face for face, start, stop in mesh.faces if stop > start}
        assert mesh_faces == faces, f"mesh faces mismatch: {mesh_faces} vs {faces}"
        mesh_tags = {self.labels[face] for face, start, stop in mesh.faces if stop > start}
        if tags is not None:
            assert mesh_tags == set(tags), f"mesh tags mismatch: {mesh_tags} vs {set(tags)}"
        return mesh

