"""Type stubs for the extension module `arris._arris`.

Hand-written; `mypy.stubtest` (CI's `python` job) holds them to the built
module, and the docstrings live in the module, not here.
"""

from collections.abc import Sequence
from typing import Any, ClassVar, TypeAlias, final

__version__: str

_Handle: TypeAlias = Body | Shell | Face | Edge | Vertex
# What an exception names an entity as: a handle of the model the call was
# made on, or the kernel's text (`f3`) when the ids are not that model's.
_Entity: TypeAlias = _Handle | str
_Point3: TypeAlias = Sequence[float]
_Point2: TypeAlias = Sequence[float]
_Rows: TypeAlias = tuple[
    tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]
]

class ArrisError(Exception): ...


@final
class Cancel:
    def __init__(self) -> None: ...
    def set(self) -> None: ...
    def reset(self) -> None: ...
    def is_set(self) -> bool: ...

@final
class Body:
    @property
    def index(self) -> int: ...
    @property
    def generation(self) -> int: ...
    @property
    def reversed(self) -> bool: ...
    @property
    def model(self) -> Model: ...
    def __eq__(self, value: object, /) -> bool: ...
    def __hash__(self) -> int: ...

@final
class Shell:
    @property
    def index(self) -> int: ...
    @property
    def generation(self) -> int: ...
    @property
    def reversed(self) -> bool: ...
    @property
    def model(self) -> Model: ...
    def __eq__(self, value: object, /) -> bool: ...
    def __hash__(self) -> int: ...

@final
class Face:
    @property
    def index(self) -> int: ...
    @property
    def generation(self) -> int: ...
    @property
    def reversed(self) -> bool: ...
    @property
    def model(self) -> Model: ...
    def __eq__(self, value: object, /) -> bool: ...
    def __hash__(self) -> int: ...

@final
class Edge:
    @property
    def index(self) -> int: ...
    @property
    def generation(self) -> int: ...
    @property
    def reversed(self) -> bool: ...
    @property
    def model(self) -> Model: ...
    def __eq__(self, value: object, /) -> bool: ...
    def __hash__(self) -> int: ...

@final
class Vertex:
    @property
    def index(self) -> int: ...
    @property
    def generation(self) -> int: ...
    @property
    def reversed(self) -> bool: ...
    @property
    def model(self) -> Model: ...
    def __eq__(self, value: object, /) -> bool: ...
    def __hash__(self) -> int: ...

@final
class Role:
    def __new__(cls, kind: str, part: str, *fields: str | int) -> Role: ...
    @staticmethod
    def consumer(namespace: int, key: int) -> Role: ...
    @property
    def kind(self) -> str: ...
    @property
    def part(self) -> str: ...
    @property
    def fields(self) -> tuple[str | int, ...]: ...
    @property
    def namespace(self) -> int | None: ...
    @property
    def key(self) -> int | None: ...
    def __eq__(self, value: object, /) -> bool: ...
    def __hash__(self) -> int: ...

@final
class Segment:
    @staticmethod
    def line(to: _Point2) -> Segment: ...
    @staticmethod
    def arc(to: _Point2, via: _Point2) -> Segment: ...
    @staticmethod
    def ellipse_arc(
        to: _Point2, center: _Point2, major: _Point2, minor_radius: float, ccw: bool
    ) -> Segment: ...
    @property
    def end(self) -> tuple[float, float]: ...
    def __eq__(self, value: object, /) -> bool: ...
    __hash__: ClassVar[None]  # type: ignore[assignment]

@final
class Loop:
    @staticmethod
    def circle(center: _Point2, radius: float) -> Loop: ...
    @staticmethod
    def ellipse(center: _Point2, major: _Point2, minor_radius: float) -> Loop: ...
    @staticmethod
    def path(start: _Point2, segments: Sequence[Segment]) -> Loop: ...
    @staticmethod
    def polygon(points: Sequence[_Point2]) -> Loop: ...
    def __eq__(self, value: object, /) -> bool: ...
    __hash__: ClassVar[None]  # type: ignore[assignment]

@final
class Profile:
    def __new__(
        cls,
        outer: Loop,
        holes: Sequence[Loop] = ...,
        *,
        origin: _Point3 = ...,
        normal: _Point3 = ...,
        x_axis: _Point3 = ...,
    ) -> Profile: ...
    @property
    def outer(self) -> Loop: ...
    @property
    def holes(self) -> list[Loop]: ...
    def area_and_centroid(self, model: Model) -> tuple[float, tuple[float, float]]: ...
    def __eq__(self, value: object, /) -> bool: ...
    __hash__: ClassVar[None]  # type: ignore[assignment]

@final
class Provenance:
    @property
    def generated(self) -> list[tuple[_Handle | Role, list[_Handle]]]: ...
    @property
    def modified(self) -> list[tuple[_Handle | Role, list[_Handle]]]: ...
    @property
    def deleted(self) -> list[_Handle]: ...
    def generated_from(self, origin: _Handle | Role) -> list[_Handle]: ...
    def modified_from(self, origin: _Handle | Role) -> list[_Handle]: ...
    def is_deleted(self, entity: _Handle) -> bool: ...
    def origins(self, output: _Handle) -> list[tuple[str, _Handle | Role]]: ...
    def then(self, next: Provenance) -> Provenance: ...
    def __bool__(self) -> bool: ...
    def __eq__(self, value: object, /) -> bool: ...
    __hash__: ClassVar[None]  # type: ignore[assignment]

@final
class Frame:
    @property
    def origin(self) -> tuple[float, float, float]: ...
    @property
    def x(self) -> tuple[float, float, float]: ...
    @property
    def y(self) -> tuple[float, float, float]: ...
    @property
    def z(self) -> tuple[float, float, float]: ...
    def __eq__(self, value: object, /) -> bool: ...
    __hash__: ClassVar[None]  # type: ignore[assignment]

@final
class MassProperties:
    @property
    def volume(self) -> float: ...
    @property
    def area(self) -> float: ...
    @property
    def centroid(self) -> tuple[float, float, float]: ...
    @property
    def inertia(self) -> _Rows: ...
    def inertia_about(self, point: _Point3) -> _Rows: ...
    def __eq__(self, value: object, /) -> bool: ...
    __hash__: ClassVar[None]  # type: ignore[assignment]

@final
class EulerLine:
    @property
    def vertices(self) -> int: ...
    @property
    def edges(self) -> int: ...
    @property
    def faces(self) -> int: ...
    @property
    def loops(self) -> int: ...
    @property
    def shells(self) -> int: ...
    @property
    def genus(self) -> int: ...
    def __eq__(self, value: object, /) -> bool: ...
    def __hash__(self) -> int: ...

@final
class Violation:
    @property
    def code(self) -> str: ...
    @property
    def entity(self) -> _Entity: ...
    @property
    def level(self) -> str: ...
    def __eq__(self, value: object, /) -> bool: ...
    def __hash__(self) -> int: ...

@final
class UncheckedRow:
    @property
    def code(self) -> str: ...
    @property
    def entity(self) -> _Entity: ...
    def __eq__(self, value: object, /) -> bool: ...
    def __hash__(self) -> int: ...

@final
class Report:
    @property
    def ok(self) -> bool: ...
    @property
    def violations(self) -> list[Violation]: ...
    @property
    def unchecked(self) -> list[UncheckedRow]: ...
    @property
    def euler(self) -> EulerLine | None: ...
    def __bool__(self) -> bool: ...
    def __len__(self) -> int: ...
    def __eq__(self, value: object, /) -> bool: ...
    def __hash__(self) -> int: ...

@final
class Mesh:
    @property
    def positions(self) -> bytes: ...
    @property
    def triangles(self) -> bytes: ...
    @property
    def edge_indices(self) -> bytes: ...
    @property
    def n_positions(self) -> int: ...
    @property
    def n_triangles(self) -> int: ...
    @property
    def faces(self) -> list[tuple[Face, int, int]]: ...
    @property
    def edges(self) -> list[tuple[Edge, int, int]]: ...
    def is_closed(self) -> bool: ...
    def signed_volume(self) -> float | None: ...
    def area(self) -> float: ...
    def bounds(self) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None: ...
    def to_numpy(self) -> tuple[Any, Any]: ...
    def __eq__(self, value: object, /) -> bool: ...
    def __hash__(self) -> int: ...

@final
class Refusal:
    @property
    def kind(self) -> str: ...
    @property
    def entity(self) -> int: ...
    @property
    def message(self) -> str: ...
    def __eq__(self, value: object, /) -> bool: ...
    __hash__: ClassVar[None]  # type: ignore[assignment]

@final
class StepSolid:
    @property
    def file_id(self) -> int: ...
    @property
    def instance(self) -> int: ...
    @property
    def uncertainty(self) -> float | None: ...
    @property
    def body(self) -> Body | None: ...
    @property
    def provenance(self) -> Provenance | None: ...
    @property
    def refusal(self) -> Refusal | None: ...
    @property
    def ok(self) -> bool: ...

@final
class Occurrence:
    @property
    def product(self) -> int | None: ...
    @property
    def name(self) -> str: ...
    @property
    def placement(self) -> list[list[float]] | None: ...
    @property
    def placement_refusal(self) -> Refusal | None: ...
    @property
    def colour(self) -> tuple[float, float, float] | None: ...
    @property
    def solids(self) -> list[int]: ...
    @property
    def children(self) -> list[Occurrence]: ...

@final
class StepRead:
    @property
    def solids(self) -> list[StepSolid]: ...
    @property
    def products(self) -> list[Occurrence]: ...
    @property
    def face_colours(self) -> list[tuple[int, Face, tuple[float, float, float]]]: ...

@final
class Imported:
    @property
    def body(self) -> Body: ...
    @property
    def version(self) -> int: ...
    def foreign(self) -> list[str]: ...
    def translated(self) -> Provenance: ...

_Made: TypeAlias = tuple[Body, Provenance]

@final
class Model:
    def __init__(self) -> None: ...
    def contains(self, handle: _Handle) -> bool: ...
    def retain(self, keep: Sequence[Body]) -> int: ...
    def primitive_box(
        self, min: _Point3, max: _Point3, *, cancel: Cancel | None = None, budget: int | None = None
    ) -> _Made: ...
    def primitive_cylinder(
        self,
        origin: _Point3,
        axis: _Point3,
        radius: float,
        height: float,
        *,
        cancel: Cancel | None = None,
        budget: int | None = None,
    ) -> _Made: ...
    def transform(
        self,
        body: Body,
        translation: _Point3 = ...,
        *,
        axis: _Point3 = ...,
        angle: float = 0.0,
        cancel: Cancel | None = None,
        budget: int | None = None,
    ) -> _Made: ...
    def mirror(
        self,
        body: Body,
        origin: _Point3,
        normal: _Point3,
        *,
        cancel: Cancel | None = None,
        budget: int | None = None,
    ) -> _Made: ...
    def cut(
        self, target: Body, tool: Body, *, cancel: Cancel | None = None, budget: int | None = None
    ) -> _Made: ...
    def fuse(
        self, a: Body, b: Body, *, cancel: Cancel | None = None, budget: int | None = None
    ) -> _Made: ...
    def common(
        self, a: Body, b: Body, *, cancel: Cancel | None = None, budget: int | None = None
    ) -> _Made: ...
    def extrude(
        self,
        profile: Profile,
        direction: _Point3,
        length: float,
        *,
        cancel: Cancel | None = None,
        budget: int | None = None,
    ) -> _Made: ...
    def revolve(
        self,
        profile: Profile,
        origin: _Point3,
        axis: _Point3,
        angle: float,
        *,
        cancel: Cancel | None = None,
        budget: int | None = None,
    ) -> _Made: ...
    def fillet(
        self,
        body: Body,
        edges: Sequence[Edge],
        radius: float,
        *,
        cancel: Cancel | None = None,
        budget: int | None = None,
    ) -> _Made: ...
    def chamfer(
        self,
        body: Body,
        edges: Sequence[Edge],
        distance: float,
        *,
        cancel: Cancel | None = None,
        budget: int | None = None,
    ) -> _Made: ...
    def mass_properties(
        self, body: Body, *, cancel: Cancel | None = None, budget: int | None = None
    ) -> MassProperties: ...
    def face_frame(self, face: Face) -> Frame: ...
    def frame_at(self, face: Face, u: float, v: float) -> Frame: ...
    def check(self, body: Body, level: str = "fast") -> Report: ...
    def tessellate(
        self, body: Body, chord: float, *, cancel: Cancel | None = None, budget: int | None = None
    ) -> Mesh: ...
    def shells(self, body: Body) -> list[Shell]: ...
    def faces(self, body: Body) -> list[Face]: ...
    def edges(self, body: Body) -> list[Edge]: ...
    def vertices(self, body: Body) -> list[Vertex]: ...
    def edges_of(self, face: Face) -> list[Edge]: ...
    def vertices_of(self, edge: Edge) -> tuple[Vertex, Vertex]: ...
    def faces_of(self, body: Body, edge: Edge) -> list[Face]: ...
    def edges_at(self, body: Body, vertex: Vertex) -> list[Edge]: ...
    def write_step(self, bodies: Sequence[Body]) -> str: ...
    def read_step(
        self,
        text: str,
        *,
        length_unit: str = "mm",
        cancel: Cancel | None = None,
        budget: int | None = None,
    ) -> StepRead: ...
    def write_body(self, body: Body, provenance: Provenance | None = None) -> bytes: ...
    def write_body_json(self, body: Body, provenance: Provenance | None = None) -> str: ...
    def read_body(
        self, data: bytes, *, cancel: Cancel | None = None, budget: int | None = None
    ) -> Imported: ...
    def read_body_json(
        self, text: str, *, cancel: Cancel | None = None, budget: int | None = None
    ) -> Imported: ...
    def to_native(self) -> bytes: ...
    def to_native_json(self) -> str: ...
    @staticmethod
    def from_native(data: bytes) -> Model: ...
    @staticmethod
    def from_native_json(text: str) -> Model: ...
    def __eq__(self, value: object, /) -> bool: ...
    def __hash__(self) -> int: ...

def stl_binary(meshes: Sequence[Mesh], name: str = "arris") -> bytes: ...
def stl_ascii(meshes: Sequence[Mesh], name: str = "arris") -> str: ...
def obj(meshes: Sequence[Mesh]) -> str: ...

class ForeignHandleError(ArrisError):
    ...

class StaleHandleError(ArrisError):
    entity: _Entity

class ModelPoisonedError(ArrisError):
    ...

class Interrupted(ArrisError, InterruptedError):
    by: str
    steps: int

class OpError(ArrisError):
    ...

class OpInvalidInputError(OpError):
    body: _Entity
    report: Report

class OpUnsupportedError(OpError):
    a_kind: str
    a: _Entity
    b_kind: str
    b: _Entity

class OpDegenerateError(OpError):
    entities: list[_Entity]
    reason: str

class OpProfileError(OpError):
    detail: str

class OpToleranceError(OpError):
    entity: _Entity
    wanted: float

class OpInternalError(OpError):
    detail: str

class OpUnkeyedError(OpError):
    slot: str

class OpRejectedError(OpError):
    detail: str

class GeomError(ArrisError):
    ...

class GeomUnsupportedError(GeomError):
    a_kind: str
    b_kind: str

class GeomDegenerateError(GeomError):
    kind: str
    reason: str

class GeomInvalidToleranceError(GeomError):
    linear: float
    angular: float

class GeomAmbiguousError(GeomError):
    kind: str
    locus: str
    point: tuple[float, ...]

class GeomAmbiguousUvError(GeomError):
    kind: str
    locus: str
    point: tuple[float, ...]

class GeomNotOnSurfaceError(GeomError):
    curve: str
    surface: str
    t: float
    distance: float

class GeomThroughSingularityError(GeomError):
    curve: str
    surface: str
    t: float

class GeomDegenerateSectionError(GeomError):
    a_kind: str
    b_kind: str
    fault: str

class FitError(ArrisError):
    ...

class FitDegenerateError(FitError):
    reason: str

class FitInvalidToleranceError(FitError):
    tolerance: float

class FitNonFiniteError(FitError):
    t: float

class FitDivergedError(FitError):
    spans: int
    deviation: float

class TopoError(ArrisError):
    ...

class TopoPrecisionError(TopoError):
    precision: str

class StepError(ArrisError):
    ...

class StepParseError(StepError):
    line: int
    column: int
    instance: int | None
    detail: str

class StepUnsupportedError(StepError):
    body: _Entity
    what: str

class StepLumpsError(StepError):
    body: _Entity
    detail: str

class StepNonFiniteError(StepError):
    id: _Entity

class StepNoBodiesError(StepError):
    ...

class StepTreeError(StepError):
    detail: str

class BodyError(ArrisError):
    ...

class BodyMagicError(BodyError):
    ...

class BodyVersionError(BodyError):
    found: int
    newest: int

class BodyEncodeError(BodyError):
    detail: str

class BodyDecodeError(BodyError):
    detail: str

class BodyPrecisionError(BodyError):
    entity: str
    tolerance: float
    min: float
    max: float

class BodyRejectedError(BodyError):
    report: Report

class NativeError(ArrisError):
    ...

class NativeVersionError(NativeError):
    found: int
    supported: int

class NativeEncodeError(NativeError):
    detail: str

class NativeDecodeError(NativeError):
    detail: str

class MeshError(ArrisError):
    ...

class MeshTooManyTrianglesError(MeshError):
    triangles: int

class MeshIndexOutOfRangeError(MeshError):
    index: int
    positions: int

class MeshRangeOutOfBoundsError(MeshError):
    start: int
    end: int
    len: int

class MeshNonFinitePositionError(MeshError):
    index: int

class MeshInvalidInputError(MeshError):
    body: _Entity
    report: Report

class MeshChordError(MeshError):
    chord: float

class MeshFaceError(MeshError):
    face: _Entity
    detail: str

class MeshGridTooLargeError(MeshError):
    face: _Entity
    points: int

class MeshCornersError(MeshError):
    detail: str

class MeshInternalError(MeshError):
    detail: str
