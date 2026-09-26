//! The fixture corpus: `tests/fixtures/<area>/<slug>/` with a recipe
//! (`fixture.json`) both sides evaluate and the oracle's answer
//! (`expected.json`). The format is `tests/fixtures/README.md`; this module
//! is its Rust reading, and [`lint`] is the corpus lint the facade's tests
//! run over every directory.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use arris_math::Precision;
use arris_topo::euler::EulerLine;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub mod expr;
pub mod geom;

pub use expr::{ExprError, eval};

/// The three fixture kinds: a solid built by a recipe and measured,
/// analytic geometry evaluated, projected onto and intersected, or a part
/// read from a STEP file Arris did not write (`tests/fixtures/README.md`).
/// `fixture.json` names it in `"kind"`; absent means solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Kind {
    /// A recipe of steps ending in a solid, with volume, area, counts and
    /// probes to compare.
    Solid,
    /// Named surfaces and curves with samples and pairs; [`geom`].
    Geometry,
    /// A STEP file beside the fixture, every solid of it read and held to
    /// the oracle's reading or to its recorded refusal
    /// (`crate::part`, ADR-0026).
    Part,
}

/// The kind of the fixture in `dir`, from its `fixture.json`.
pub fn kind_of(dir: &Path) -> Result<Kind, FixtureError> {
    let raw: serde_json::Value = read_json(&dir.join("fixture.json"))?;
    kind_of_raw(&raw).map_err(|kind| FixtureError::UnknownKind {
        path: dir.join("fixture.json"),
        kind,
    })
}

/// `"kind"` absent or `"solid"` is [`Kind::Solid`], `"geometry"` is
/// [`Kind::Geometry`], `"part"` is [`Kind::Part`]; anything else is `Err`
/// with the string found, as the oracle's `fixture_kind` also refuses it.
fn kind_of_raw(raw: &serde_json::Value) -> Result<Kind, String> {
    match raw.get("kind").and_then(|k| k.as_str()) {
        None | Some("solid") => Ok(Kind::Solid),
        Some("geometry") => Ok(Kind::Geometry),
        Some("part") => Ok(Kind::Part),
        Some(other) => Err(other.to_string()),
    }
}

/// `<area>/<slug>`: the last two components of a fixture directory.
pub fn name_of(dir: &Path) -> String {
    dir.components()
        .rev()
        .take(2)
        .map(|c| c.as_os_str().to_string_lossy().into_owned())
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .collect::<Vec<_>>()
        .join("/")
}

/// A number in a recipe: a literal, or an expression over the recipe's
/// params (`"50 + R * cos(radians(45))"`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Num {
    /// A JSON number.
    Literal(f64),
    /// An expression; see [`expr::eval`].
    Expr(String),
}

impl Num {
    /// The value under `params`.
    pub fn eval(&self, params: &BTreeMap<String, f64>) -> Result<f64, ExprError> {
        match self {
            Num::Literal(v) => Ok(*v),
            Num::Expr(text) => expr::eval(text, params),
        }
    }
}

impl From<f64> for Num {
    fn from(v: f64) -> Self {
        Num::Literal(v)
    }
}

/// Where a point lies relative to a solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Class {
    /// Strictly inside the material.
    In,
    /// Strictly outside.
    Out,
    /// On the boundary, within tolerance.
    On,
}

/// A point both sides classify.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Probe {
    /// A name for the report; unique within the fixture.
    pub label: String,
    /// The point.
    pub point: [Num; 3],
    /// The classification the fixture's author expects, if the answer is
    /// the same in every variant; `None` leaves it to the oracle.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub expect: Option<Class>,
}

/// The plane of a profile, by origin and two orthogonal in-plane axes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Plane {
    /// Origin.
    pub origin: [Num; 3],
    /// The u axis (normalised by the interpreter).
    pub x: [Num; 3],
    /// The v axis.
    pub y: [Num; 3],
}

/// One segment of a profile loop, from the previous point.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Segment {
    /// A straight segment to a point.
    Line {
        /// The end point in (u, v).
        line_to: [Num; 2],
    },
    /// A circular arc through `via` to a point.
    Arc {
        /// The end point in (u, v).
        arc_to: [Num; 2],
        /// A point on the arc between the ends.
        via: [Num; 2],
    },
    /// An arc of the ellipse of `center`, `major` and `minor_radius` to a
    /// point, counter-clockwise in (u, v) when `ccw` (ADR-0014).
    Ellipse {
        /// The end point in (u, v).
        ellipse_to: [Num; 2],
        /// The ellipse's centre in (u, v).
        center: [Num; 2],
        /// From the centre to one end of the major axis.
        major: [Num; 2],
        /// The minor radius.
        minor_radius: Num,
        /// Counter-clockwise from the previous point to `ellipse_to`.
        ccw: bool,
    },
}

/// A closed loop of a profile.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Loop {
    /// A full circle.
    Circle {
        /// The circle.
        circle: Circle,
    },
    /// A full ellipse.
    Ellipse {
        /// The ellipse.
        ellipse: Ellipse,
    },
    /// A chain of segments starting at `start` and returning to it.
    Path {
        /// The first point in (u, v).
        start: [Num; 2],
        /// The segments; the last ends at `start`.
        segments: Vec<Segment>,
    },
}

/// A circle in a profile plane.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Circle {
    /// Centre in (u, v).
    pub center: [Num; 2],
    /// Radius.
    pub radius: Num,
}

/// An ellipse in a profile plane (ADR-0014).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ellipse {
    /// Centre in (u, v).
    pub center: [Num; 2],
    /// From the centre to one end of the major axis.
    pub major: [Num; 2],
    /// The minor radius.
    pub minor_radius: Num,
}

/// A revolve axis.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Axis {
    /// A point on the axis.
    pub origin: [Num; 3],
    /// Its direction.
    pub direction: [Num; 3],
}

/// The rotation part of a transform.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Rotate {
    /// The axis direction.
    pub axis: [Num; 3],
    /// A point on the axis; the origin if absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin: Option<[Num; 3]>,
    /// The angle in degrees, right-handed about `axis`.
    pub angle_deg: Num,
}

/// One step of a recipe. The `name` is what later steps refer to.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "lowercase")]
pub enum Step {
    /// An axis-aligned box.
    Box {
        /// Step name.
        name: String,
        /// Minimum corner.
        min: [Num; 3],
        /// Maximum corner.
        max: [Num; 3],
    },
    /// A cylinder from `base` along `axis`.
    Cylinder {
        /// Step name.
        name: String,
        /// Centre of the base cap.
        base: [Num; 3],
        /// Axis direction.
        axis: [Num; 3],
        /// Radius.
        radius: Num,
        /// Height along the axis.
        height: Num,
    },
    /// A planar face from an outer loop and holes.
    Profile {
        /// Step name.
        name: String,
        /// The plane.
        plane: Plane,
        /// The outer loop.
        outer: Loop,
        /// Holes.
        #[serde(default)]
        holes: Vec<Loop>,
    },
    /// A profile swept along a direction.
    Extrude {
        /// Step name.
        name: String,
        /// The profile step.
        profile: String,
        /// Direction (normalised by the interpreter).
        direction: [Num; 3],
        /// Distance.
        length: Num,
    },
    /// A profile swept about an axis.
    Revolve {
        /// Step name.
        name: String,
        /// The profile step.
        profile: String,
        /// The axis.
        axis: Axis,
        /// Angle in degrees, 360 for a full turn.
        angle_deg: Num,
    },
    /// A rigid transform of a step's shape: rotate, then translate.
    Transform {
        /// Step name.
        name: String,
        /// The step transformed.
        of: String,
        /// Translation.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        translate: Option<[Num; 3]>,
        /// Rotation.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        rotate: Option<Rotate>,
    },
    /// Boolean union.
    Fuse {
        /// Step name.
        name: String,
        /// First operand.
        a: String,
        /// Second operand.
        b: String,
    },
    /// Boolean intersection.
    Common {
        /// Step name.
        name: String,
        /// First operand.
        a: String,
        /// Second operand.
        b: String,
    },
    /// Boolean difference.
    Cut {
        /// Step name.
        name: String,
        /// The body cut from.
        target: String,
        /// The body cut away.
        tool: String,
    },
    /// A constant-radius fillet of edges of a step's body, named by a
    /// point on each: Arris takes the edge `classify_point` answers
    /// `On(Edge)` for, the oracle the nearest edge by `BRepExtrema`, and
    /// both refuse a point within `probe` of two edges or on none
    /// (`tests/fixtures/README.md`).
    Fillet {
        /// Step name.
        name: String,
        /// The body blended.
        of: String,
        /// One point on each edge to blend.
        edges: Vec<[Num; 3]>,
        /// The ball's radius.
        radius: Num,
    },
    /// An equal-distance chamfer of edges of a step's body, each named by
    /// a point on it as a `Fillet`'s is.
    Chamfer {
        /// Step name.
        name: String,
        /// The body chamfered.
        of: String,
        /// One point on each edge to chamfer.
        edges: Vec<[Num; 3]>,
        /// The distance from the edge, measured on each of its faces.
        distance: Num,
    },
    /// A solid bounded by planar faces over `points`, each face a list of
    /// loops of point indices — the outer loop counter-clockwise seen
    /// from outside, a hole's clockwise: a consumer's own topology. Arris
    /// builds it through `Builder::assemble` and `ops::build`, keyed by
    /// [`crate::polyhedron`]'s convention in `namespace`; the oracle sews
    /// polygon faces into a solid.
    Polyhedron {
        /// Step name.
        name: String,
        /// The points.
        points: Vec<[Num; 3]>,
        /// The faces, each its loops of indices into `points`.
        faces: Vec<Vec<Vec<usize>>>,
        /// The namespace the keys are in.
        namespace: u32,
    },
    /// A solid read from a STEP file beside the fixture (`"op": "step"`):
    /// Arris through `arris_io::step::read`, the oracle through Open
    /// CASCADE's reader, healed (ADR-0026 §3). The solid is named by its
    /// file entity and, where an assembly places it more than once, by a
    /// point its placement's centroid is nearest, so that neither side
    /// depends on the other's order of instances.
    #[serde(rename = "step")]
    Read {
        /// Step name.
        name: String,
        /// The file, relative to the fixture's directory.
        file: String,
        /// The file's SHA-256, lower-case hex: both sides refuse a file
        /// that does not match, and the recipe hash moves with it.
        sha256: String,
        /// The `#id` of the `MANIFOLD_SOLID_BREP` or `BREP_WITH_VOIDS`.
        id: u64,
        /// Which placement of it, where the file has several: the one
        /// whose centroid is nearest this point. Required then; a tie
        /// within `probe` is refused.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        near: Option<[Num; 3]>,
    },
}

impl Step {
    /// The step's name.
    pub fn name(&self) -> &str {
        match self {
            Step::Box { name, .. }
            | Step::Cylinder { name, .. }
            | Step::Profile { name, .. }
            | Step::Extrude { name, .. }
            | Step::Revolve { name, .. }
            | Step::Transform { name, .. }
            | Step::Fuse { name, .. }
            | Step::Common { name, .. }
            | Step::Cut { name, .. }
            | Step::Fillet { name, .. }
            | Step::Chamfer { name, .. }
            | Step::Polyhedron { name, .. }
            | Step::Read { name, .. } => name,
        }
    }
}

/// The model's tolerance configuration a fixture builds under: every
/// field of [`Precision`] the recipe names, the rest
/// `Precision::DEFAULT`. Arris carries no unit, so this is what makes a
/// fixture's numbers mean metres rather than millimetres: a consumer in
/// metres sets `default_tolerance` at the micrometre scale
/// (`docs/ARCHITECTURE.md` §Units), and the corpus's `*-m` fixtures are
/// the proof that the operations hold there.
///
/// It is Arris's alone and never reaches the oracle — Open CASCADE's
/// `Precision::Confusion` is a constant of its build — so it is outside
/// the recipe hash ([`SOLID_KEYS`]), like `tolerances`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PrecisionSpec {
    /// The tolerance a primitive's entities are created with.
    pub default_tolerance: f64,
    /// The floor no entity's tolerance goes below.
    pub min_tolerance: f64,
    /// The ceiling above which an operation errors instead of widening.
    pub max_tolerance: f64,
    /// Angle in radians below which two directions are parallel.
    pub angular_tolerance: f64,
    /// A pcurve's allowed deviation in (u, v) at unit parametric speed.
    pub parametric_tolerance: f64,
    /// How many parameters the checker samples along an edge.
    pub check_samples: usize,
}

impl Default for PrecisionSpec {
    /// [`Precision::DEFAULT`], field for field.
    fn default() -> Self {
        let p = Precision::DEFAULT;
        PrecisionSpec {
            default_tolerance: p.default_tolerance,
            min_tolerance: p.min_tolerance,
            max_tolerance: p.max_tolerance,
            angular_tolerance: p.angular_tolerance,
            parametric_tolerance: p.parametric_tolerance,
            check_samples: p.check_samples,
        }
    }
}

impl PrecisionSpec {
    /// The [`Precision`] a model is created with. It is not checked here:
    /// `Model::new` refuses an inconsistent one, and the corpus runner
    /// reports that refusal as the fixture's own error.
    pub fn precision(&self) -> Precision {
        Precision {
            default_tolerance: self.default_tolerance,
            min_tolerance: self.min_tolerance,
            max_tolerance: self.max_tolerance,
            angular_tolerance: self.angular_tolerance,
            parametric_tolerance: self.parametric_tolerance,
            check_samples: self.check_samples,
        }
    }
}

/// Comparison tolerances of a fixture. Counts and classifications are
/// always exact.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Tolerances {
    /// Relative, on volume.
    pub volume_rel: f64,
    /// Relative, on area.
    pub area_rel: f64,
    /// Absolute, on the centroid's distance.
    pub centroid_abs: f64,
    /// The distance within which a probe is "on" the boundary.
    pub probe: f64,
    /// The chord tolerance the corpus runner tessellates the result at
    /// (Arris only; the oracle does not mesh).
    pub mesh_chord: f64,
    /// Relative, on the mesh's signed volume against the oracle's volume
    /// (Arris only). The default is sized by the closed form of an
    /// inscribed prism, `4δ / (3r)` at `mesh_chord` on the corpus's
    /// smallest radius (ADR-0003).
    pub mesh_volume_rel: f64,
    /// Relative, on each component of the inertia tensor, once `measure`
    /// records one.
    pub inertia_rel: f64,
}

impl Default for Tolerances {
    /// The oracle's `DEFAULT_TOLERANCES`, and the runner's own for the
    /// mesh and `measure` stages.
    fn default() -> Self {
        Tolerances {
            volume_rel: 1e-9,
            area_rel: 1e-9,
            centroid_abs: 1e-7,
            probe: 1e-7,
            mesh_chord: 1e-3,
            mesh_volume_rel: 2e-3,
            inertia_rel: 1e-9,
        }
    }
}

impl Tolerances {
    /// These tolerances widened to what a boundary known only to within
    /// `t` supports — the first-order bound of the oracle's
    /// `within_own_tolerance`, over the sizes `measured.own` records: `A·t`
    /// of volume, `L·t` of area, `A·t·R / V` of centroid and `A·t·R²` of
    /// each inertia component, for area `A`, volume `V`, edge length `L`
    /// and reach `R`; and `A·(δ + t)` of the mesh's volume, its
    /// tessellation lying within `mesh_chord` `δ` of a boundary itself
    /// known to `t`. Never narrower than `self`; `self` unchanged when
    /// `measured` records no `own` or no volume and area. The differential
    /// passes the sum of both shapes' tolerances: each kernel vouches for
    /// its boundary to within its own, so no comparison of the two is
    /// finer than that.
    ///
    /// ```
    /// use arris_debug::fixtures::{Counts, Measured, Own, Tolerances};
    ///
    /// let measured = Measured {
    ///     degenerate: false,
    ///     counts: Counts { vertices: 8, edges: 12, faces: 6, loops: 6, shells: 1, solids: 1 },
    ///     volume: Some(1.0),
    ///     area: Some(6.0),
    ///     centroid: Some([0.5; 3]),
    ///     inertia: None,
    ///     euler_characteristic: Some(2),
    ///     genus: Some(0),
    ///     probes: Vec::new(),
    ///     nurbs_counts: None,
    ///     nurbs_fails: None,
    ///     own: Some(Own { tolerance: 1e-7, edge_length: 12.0, reach: 0.87, removable_vertices: 0 }),
    /// };
    /// let held = Tolerances::default().within(&measured, 1e-7);
    /// assert!((held.volume_rel - 6e-7).abs() < 1e-20);
    /// assert_eq!(held.probe, Tolerances::default().probe);
    /// ```
    pub fn within(&self, measured: &Measured, t: f64) -> Tolerances {
        let (Some(own), Some(volume), Some(area)) = (measured.own, measured.volume, measured.area)
        else {
            return *self;
        };
        let volume = volume.abs();
        if !(volume > 0.0 && area > 0.0 && t.is_finite()) {
            return *self;
        }
        let scale = measured
            .inertia
            .iter()
            .flatten()
            .flatten()
            .fold(0.0f64, |m, x| m.max(x.abs()));
        let scale = if scale > 0.0 { scale } else { 1.0 };
        Tolerances {
            volume_rel: self.volume_rel.max(area * t / volume),
            area_rel: self.area_rel.max(own.edge_length * t / area),
            centroid_abs: self.centroid_abs.max(area * t * own.reach / volume),
            inertia_rel: self
                .inertia_rel
                .max(area * t * own.reach * own.reach / scale),
            mesh_volume_rel: self
                .mesh_volume_rel
                .max(area * (self.mesh_chord + t) / volume),
            ..*self
        }
    }
}

/// Entity counts of a result.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Counts {
    /// Unique vertices.
    pub vertices: usize,
    /// Unique edges; a seam counts once.
    pub edges: usize,
    /// Faces.
    pub faces: usize,
    /// Loops (wires).
    pub loops: usize,
    /// Shells.
    #[serde(default = "one")]
    pub shells: usize,
    /// Solids.
    #[serde(default = "one")]
    pub solids: usize,
}

fn one() -> usize {
    1
}

/// A result the oracle builds and Arris refuses by design: the typed
/// error the corpus runner asserts instead of comparing the result
/// (ADR-0004). The oracle's numbers are recorded in `expected.json` as the
/// record of what Open CASCADE makes, and not compared.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ExpectError {
    /// `OpError::Degenerate` with `Reason::TangentContact`: two faces
    /// touch along a curve interior to both. The oracle's result carries
    /// the contact as an edge of four faces; where that makes its Euler
    /// characteristic odd it records no genus, and the recipe states none.
    TangentContact,
    /// `OpError::Degenerate` with `Reason::NonManifold`: two shells of the
    /// result would share an edge or a vertex (ADR-0006). The oracle's
    /// compound of solids sharing it has an odd Euler characteristic, so
    /// it records no genus and the recipe states none.
    NonManifold,
    /// `OpError::Degenerate` with `Reason::BlendTooLarge`: a blend's
    /// contact or end arc leaves its face (ADR-0007).
    BlendTooLarge,
    /// `OpError::Degenerate` with `Reason::TangentChain`: a blended edge's
    /// faces meet at a tangent dihedral, or the edge runs into a blend
    /// face (ADR-0007).
    TangentChain,
    /// `OpError::Degenerate` with `Reason::VertexBlend`: a corner the
    /// blend's closed forms do not cover (ADR-0007).
    VertexBlend,
    /// `OpError::Degenerate` with `Reason::EllipticRevolve`: a revolve of
    /// a profile with an elliptic segment, whose swept surface has no
    /// variant (ADR-0014).
    EllipticRevolve,
    /// `OpError::Unsupported` with a NURBS surface or curve in the pair:
    /// an operation on a free-form face the kernel has no closed form
    /// for, the NURBS cycle's (ADR-0026 §5).
    Nurbs,
}

/// A refusal the STEP reader is expected to return for Open CASCADE's own
/// file of a result (`analytic.occt_step_refused`).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReadRefused {
    /// The refusal's kind, as `arris_io::step::RefusalKind` prints it
    /// (`"not a closed shell"`, `"open loop"`).
    pub kind: String,
    /// Why the file is refused: what in it describes no solid.
    pub why: String,
}

/// The closed forms a fixture's author states, cross-checking the oracle.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct Analytic {
    /// The result has no volume (Arris: `OpError::Degenerate`).
    pub degenerate: bool,
    /// Arris refuses the result the oracle builds, with this error.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expect_error: Option<ExpectError>,
    /// Volume.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub volume: Option<Num>,
    /// Surface area.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub area: Option<Num>,
    /// Centroid.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub centroid: Option<[Num; 3]>,
    /// Entity counts: the oracle's, cross-checked by the lint — or, with
    /// `counts_differ`, Arris's.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub counts: Option<Counts>,
    /// Why Arris's counts differ from the oracle's by a stated convention
    /// (`boolean/tangent-outside-cut`: Open CASCADE imprints the tangent
    /// ruling on the touched face, Arris keeps the face whole). Then
    /// `counts` is required, is Arris's, and is what the runner and the
    /// oracle's `compare.py` hold the result to; the oracle's counts stay
    /// in `expected.json` as the record, and the lint holds both to the
    /// Euler line with `genus`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub counts_differ: Option<String>,
    /// The inertia tensor about the centroid at unit density, as rows, in
    /// `expected.json`'s convention (the products of inertia negated).
    /// Cross-checked against the oracle's like the other closed forms;
    /// required under `measure_differs`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inertia: Option<[[Num; 3]; 3]>,
    /// Why the oracle's measurements of this result are wrong, and the
    /// closed forms right (ADR-0015: a motion that is the identity on the
    /// solid moves Open CASCADE's volume). Then `volume`, `area`,
    /// `centroid` and `inertia` are required and are what the runner and
    /// the oracle's `compare.py` hold the result to; the oracle's values
    /// stay in `expected.json` as the record, and the lint requires at
    /// least one of them to differ from its closed form in every variant.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub measure_differs: Option<String>,
    /// Why Open CASCADE's own STEP of this result does not read back as
    /// the result (ADR-0023: its reader hands back other counts or
    /// another solid for its own file). Only the oracle's self-test reads
    /// it, and skips that round trip; the runner and `compare.py` still
    /// read Arris's STEP of the result back through Open CASCADE and hold
    /// it to `expected.json`, so nothing Arris is held to changes.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub step_differs: Option<String>,
    /// Why Arris's STEP reader refuses Open CASCADE's own STEP of this
    /// result, and with which refusal (ADR-0025): a file that describes no
    /// solid by the standard — Open CASCADE's writer turning a face's
    /// bounds, or its own boolean leaving a face that does not close. The
    /// runner's read-back stage then asserts that refusal instead of
    /// comparing, and fails once the file reads, so the entry is lifted
    /// with the change that reads it.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub occt_step_refused: Option<ReadRefused>,
    /// Genus.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub genus: Option<i64>,
}

/// A `fixture.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Recipe {
    /// What the fixture is for.
    #[serde(default)]
    pub description: String,
    /// Named numbers the steps may use in expressions.
    #[serde(default)]
    pub params: BTreeMap<String, f64>,
    /// Param overrides, each a variant with its own `expected` result.
    #[serde(default)]
    pub variants: BTreeMap<String, BTreeMap<String, f64>>,
    /// The steps, in order.
    pub steps: Vec<Step>,
    /// The step whose shape is the fixture's result.
    pub result: String,
    /// Probe points.
    #[serde(default)]
    pub probes: Vec<Probe>,
    /// The precision the model is created with; `Precision::DEFAULT`
    /// where the recipe names none.
    #[serde(default)]
    pub precision: PrecisionSpec,
    /// Comparison tolerances.
    #[serde(default)]
    pub tolerances: Tolerances,
    /// Closed forms.
    #[serde(default)]
    pub analytic: Analytic,
}

impl Recipe {
    /// The variant names, `default` first.
    pub fn variant_names(&self) -> Vec<String> {
        let mut v = vec![String::from("default")];
        v.extend(self.variants.keys().cloned());
        v
    }

    /// The params of a variant, or `None` for an unknown one.
    pub fn params_of(&self, variant: &str) -> Option<BTreeMap<String, f64>> {
        let mut p = self.params.clone();
        if variant != "default" {
            p.extend(
                self.variants
                    .get(variant)?
                    .iter()
                    .map(|(k, v)| (k.clone(), *v)),
            );
        }
        Some(p)
    }
}

/// A probe's classification by the oracle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProbeResult {
    /// The probe's label.
    pub label: String,
    /// The point.
    pub point: [f64; 3],
    /// The oracle's answer.
    pub class: Class,
}

/// The oracle's measurements of one variant's result.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Measured {
    /// No solid in the result.
    pub degenerate: bool,
    /// Entity counts.
    pub counts: Counts,
    /// Volume; absent when degenerate.
    #[serde(default)]
    pub volume: Option<f64>,
    /// Area.
    #[serde(default)]
    pub area: Option<f64>,
    /// Centroid.
    #[serde(default)]
    pub centroid: Option<[f64; 3]>,
    /// The inertia tensor about the centroid at unit density, as rows,
    /// in the physical convention (the products of inertia negated):
    /// OCCT's `MatrixOfInertia`, and `arris_ops::measure`'s.
    #[serde(default)]
    pub inertia: Option<[[f64; 3]; 3]>,
    /// `V − E + 2F − L`.
    #[serde(default)]
    pub euler_characteristic: Option<i64>,
    /// `S − χ / 2`.
    #[serde(default)]
    pub genus: Option<i64>,
    /// Probe classifications.
    #[serde(default)]
    pub probes: Vec<ProbeResult>,
    /// The counts of the result converted to B-splines by
    /// `BRepBuilderAPI_NurbsConvert`, which may gain seams: what Arris's
    /// reader of Open CASCADE's STEP of the converted result is held to
    /// (ADR-0025). Absent for a degenerate
    /// result and one the oracle builds as a non-manifold compound.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nurbs_counts: Option<Counts>,
    /// Why Open CASCADE could not convert the result to B-splines, where
    /// it could not: then there are no `nurbs_counts`, and the runner
    /// reads no converted file back.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nurbs_fails: Option<String>,
    /// The oracle shape's own tolerance and the sizes a boundary moved
    /// within it is measured over: written by `expected.py --own`, which
    /// only the differential runs, and for a part's battery, which is held
    /// as the differential holds a draw (`crate::battery`); absent from
    /// every solid fixture of the corpus.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub own: Option<Own>,
}

/// What the oracle's shape declares of itself (`oracle.measure.own_measures`):
/// the first-order bound a boundary moved within `tolerance` puts on each
/// mass property is taken over these ([`Tolerances::within`]).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Own {
    /// The shape's largest vertex tolerance.
    pub tolerance: f64,
    /// The total length of its edges.
    pub edge_length: f64,
    /// The farthest corner of its bounding box from its centroid.
    pub reach: f64,
    /// Its vertices that only split an edge between two faces in two
    /// (`oracle.measure.removable_vertices`): the differential compares
    /// counts net of them.
    #[serde(default)]
    pub removable_vertices: usize,
}

/// An `expected.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Expected {
    /// The `cadquery-ocp` version that wrote it.
    pub occt: String,
    /// The recipe hash it was computed from; see [`recipe_hash`].
    pub recipe_sha256: String,
    /// One result per variant, `default` always present.
    pub results: BTreeMap<String, Measured>,
}

/// A loaded fixture directory.
#[derive(Debug, Clone, PartialEq)]
pub struct Fixture {
    /// The directory.
    pub dir: PathBuf,
    /// `<area>/<slug>`.
    pub name: String,
    /// The recipe.
    pub recipe: Recipe,
    /// The hash of the recipe as loaded, to check against `expected`.
    pub recipe_sha256: String,
    /// The oracle's answer.
    pub expected: Expected,
}

/// Why a fixture could not be loaded.
#[derive(Debug, thiserror::Error)]
pub enum FixtureError {
    /// A file is missing or unreadable.
    #[error("{path}: {source}")]
    Io {
        /// The file.
        path: PathBuf,
        /// The cause.
        source: std::io::Error,
    },
    /// A file is not valid for its schema.
    #[error("{path}: {source}")]
    Json {
        /// The file.
        path: PathBuf,
        /// The cause.
        source: serde_json::Error,
    },
    /// `"kind"` is neither absent, `"solid"` nor `"geometry"`.
    #[error("{path}: unknown fixture kind {kind:?}")]
    UnknownKind {
        /// The file.
        path: PathBuf,
        /// The value found.
        kind: String,
    },
}

/// The corpus root: `tests/fixtures/` at the workspace root.
pub fn corpus_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/fixtures")
}

/// Every fixture directory under [`corpus_root`] (a directory holding a
/// `fixture.json`), sorted by path.
pub fn corpus() -> Vec<PathBuf> {
    fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
        let Ok(entries) = std::fs::read_dir(dir) else {
            return;
        };
        let mut paths: Vec<PathBuf> = entries.flatten().map(|e| e.path()).collect();
        paths.sort();
        for p in paths {
            if p.is_dir() {
                if p.join("fixture.json").is_file() {
                    out.push(p);
                } else {
                    walk(&p, out);
                }
            }
        }
    }
    let mut out = Vec::new();
    walk(&corpus_root(), &mut out);
    out
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T, FixtureError> {
    let text = std::fs::read_to_string(path).map_err(|source| FixtureError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    serde_json::from_str(&text).map_err(|source| FixtureError::Json {
        path: path.to_path_buf(),
        source,
    })
}

/// The keys of a solid recipe the oracle evaluates and hashes.
pub const SOLID_KEYS: [&str; 5] = ["params", "variants", "steps", "result", "probes"];
/// The keys of a geometry recipe the oracle evaluates and hashes.
pub const GEOMETRY_KEYS: [&str; 6] = ["kind", "params", "surfaces", "curves", "samples", "pairs"];
/// The keys of a part fixture the oracle reads and hashes: the file, by
/// name and by content, and the battery's operands it builds. The
/// solids' outcomes and the battery's recorded classes are Arris's, not
/// the oracle's.
pub const PART_KEYS: [&str; 4] = ["kind", "file", "sha256", "battery"];

/// The SHA-256 the oracle records: over the recipe's evaluated keys as
/// parsed — [`SOLID_KEYS`], [`GEOMETRY_KEYS`] or [`PART_KEYS`] by
/// [`Kind`] — encoded
/// with sorted keys and no whitespace (serde_json's float formatting; the
/// oracle matches it). Editing `analytic` or `description` does not
/// change it. `Err` is the unknown `"kind"` string, when there is one.
pub fn recipe_hash(raw: &serde_json::Value) -> Result<String, String> {
    let keys: &[&str] = match kind_of_raw(raw)? {
        Kind::Solid => &SOLID_KEYS,
        Kind::Geometry => &GEOMETRY_KEYS,
        Kind::Part => &PART_KEYS,
    };
    let mut evaluated = serde_json::Map::new();
    for key in keys {
        evaluated.insert(
            key.to_string(),
            raw.get(key).cloned().unwrap_or(serde_json::Value::Null),
        );
    }
    let text = serde_json::Value::Object(evaluated).to_string();
    let digest = Sha256::digest(text.as_bytes());
    Ok(digest.iter().map(|b| format!("{b:02x}")).collect())
}

/// Loads a fixture directory: both files, and the recipe's hash.
pub fn load(dir: &Path) -> Result<Fixture, FixtureError> {
    let raw: serde_json::Value = read_json(&dir.join("fixture.json"))?;
    let recipe: Recipe =
        serde_json::from_value(raw.clone()).map_err(|source| FixtureError::Json {
            path: dir.join("fixture.json"),
            source,
        })?;
    let expected: Expected = read_json(&dir.join("expected.json"))?;
    let recipe_sha256 = recipe_hash(&raw).map_err(|kind| FixtureError::UnknownKind {
        path: dir.join("fixture.json"),
        kind,
    })?;
    Ok(Fixture {
        dir: dir.to_path_buf(),
        name: name_of(dir),
        recipe_sha256,
        recipe,
        expected,
    })
}

/// The areas of the corpus whose comparable solid fixtures must carry a
/// committed dump per variant: every area a corpus test runs, so a
/// fixture there has passed and been blessed, and none is `#[ignore]`d.
/// `real/` holds parts, whose lint holds their dumps the same way
/// (`crate::part::lint`).
pub const DUMPED_AREAS: [&str; 7] = [
    "primitive",
    "build",
    "transform",
    "boolean",
    "sweep",
    "provenance",
    "blend",
];

/// The area a failure shrunk to a fixture waits in until it passes
/// (`.agents/rules/kernel.md` §Testing): outside [`DUMPED_AREAS`], so it
/// needs no dump, and holding none, since a fixture with a blessed dump
/// passes and belongs in its own area.
pub const REGRESSION_AREA: &str = "regression";

/// Relative tolerance the corpus lint holds `analytic` to the oracle at.
/// Looser than the fixture's comparison tolerance on purpose: a closed form
/// typed by hand is a cross-check of conventions, not a second oracle.
pub const ANALYTIC_REL: f64 = 1e-6;

/// The corpus lint for one directory, by its [`Kind`]. A solid: both
/// files present and parseable, the recipe hash matches `expected.json`,
/// every variant has a result, the Euler line is zero (`χ = 2(S − G)`
/// with the oracle's counts and the fixture's `analytic.genus`, or under
/// `counts_differ` the oracle's own genus, `analytic.genus` then closing
/// Arris's counts), and
/// every `analytic` value matches the oracle within [`ANALYTIC_REL`]
/// (counts, degeneracy and probe expectations exactly) — except the
/// counts under `counts_differ`, which must differ, and the volume, area,
/// centroid and inertia under `measure_differs`, which must all be stated
/// and of which at least one must differ (ADR-0015); and a solid in
/// one of [`DUMPED_AREAS`] that the runner compares — the oracle built a
/// solid and the recipe expects no refusal — has its dump committed for
/// every variant (`corpus::dump_path`), which a fixture only has once it
/// passed and was blessed; while a fixture in [`REGRESSION_AREA`] has
/// no committed dump for any variant. A geometry
/// fixture: [`geom::lint`] — presence, hash and shape, the values being
/// the geometry oracle test's to compare. Returns every problem found,
/// empty when clean.
pub fn lint(dir: &Path) -> Vec<String> {
    match kind_of(dir) {
        Ok(Kind::Geometry) => return geom::lint(dir),
        Ok(Kind::Part) => return crate::part::lint(dir),
        Ok(Kind::Solid) => {}
        Err(e) => return vec![format!("{}: {e}", dir.display())],
    }
    let mut problems = Vec::new();
    let fixture = match load(dir) {
        Ok(f) => f,
        Err(e) => return vec![format!("{}: {e}", dir.display())],
    };
    let name = &fixture.name;
    let mut problem = |text: String| problems.push(format!("{name}: {text}"));
    let r = &fixture.recipe;
    let x = &fixture.expected;
    if x.recipe_sha256 != fixture.recipe_sha256 {
        problem(format!(
            "expected.json is stale: recipe hash {} but the recipe hashes to {} — rerun tools/oracle/expected.py",
            x.recipe_sha256, fixture.recipe_sha256
        ));
    }
    let names: std::collections::BTreeSet<&str> = r.steps.iter().map(Step::name).collect();
    if names.len() != r.steps.len() {
        problem("duplicate step names".into());
    }
    if !names.contains(r.result.as_str()) {
        problem(format!("result {:?} is not a step", r.result));
    }
    let rel = |a: f64, b: f64| (a - b).abs() <= ANALYTIC_REL * a.abs().max(b.abs()).max(1e-300);
    let area = name.split('/').next().unwrap_or_default();
    let dumped = DUMPED_AREAS.contains(&area);
    for variant in r.variant_names() {
        let Some(m) = x.results.get(&variant) else {
            problem(format!(
                "variant {variant:?} has no result in expected.json"
            ));
            continue;
        };
        let Some(params) = r.params_of(&variant) else {
            continue;
        };
        let a = &r.analytic;
        let dump = crate::corpus::dump_path(dir, &variant);
        if area == REGRESSION_AREA && dump.is_file() {
            problem(format!(
                "[{variant}] {} is committed under {REGRESSION_AREA}/: the fixture passes, so it moves into its area with its test renamed",
                dump.file_name().unwrap_or_default().to_string_lossy()
            ));
        }
        if a.degenerate && a.expect_error.is_some() {
            problem("analytic.degenerate and analytic.expect_error are both set".into());
        }
        if a.counts_differ.is_some() {
            if a.counts.is_none() {
                problem("analytic.counts_differ needs analytic.counts, Arris's".into());
            }
            if a.degenerate || a.expect_error.is_some() {
                problem("analytic.counts_differ needs a result Arris builds".into());
            }
        }
        if a.measure_differs.is_some() {
            let missing: Vec<&str> = [
                ("volume", a.volume.is_none()),
                ("area", a.area.is_none()),
                ("centroid", a.centroid.is_none()),
                ("inertia", a.inertia.is_none()),
            ]
            .into_iter()
            .filter_map(|(field, absent)| absent.then_some(field))
            .collect();
            if !missing.is_empty() {
                problem(format!(
                    "analytic.measure_differs needs the closed forms it is held to: analytic.{} missing",
                    missing.join(", analytic.")
                ));
            }
            if a.degenerate || a.expect_error.is_some() {
                problem("analytic.measure_differs needs a result Arris builds".into());
            }
        }
        if a.step_differs.is_some() && (a.degenerate || a.expect_error.is_some()) {
            problem("analytic.step_differs needs a result Arris builds".into());
        }
        if a.occt_step_refused.is_some()
            && (a.degenerate || a.expect_error.is_some() || a.step_differs.is_some())
        {
            problem(
                "analytic.occt_step_refused needs a result Arris builds whose Open CASCADE STEP is read back"
                    .into(),
            );
        }
        if a.degenerate != m.degenerate {
            problem(format!(
                "[{variant}] analytic.degenerate {} but the oracle says {}",
                a.degenerate, m.degenerate
            ));
            continue;
        }
        if m.degenerate {
            continue;
        }
        if dumped && a.expect_error.is_none() && !dump.is_file() {
            problem(format!(
                "[{variant}] {} is not committed: a fixture the runner compares carries its blessed dump (ARRIS_BLESS=1), so an #[ignore]d one fails here",
                dump.file_name().unwrap_or_default().to_string_lossy()
            ));
        }
        let Some(chi) = m.euler_characteristic else {
            problem(format!(
                "[{variant}] expected.json lacks euler_characteristic"
            ));
            continue;
        };
        let c = m.counts;
        let counted = EulerLine::new(c.vertices, c.edges, c.faces, c.loops, c.shells);
        let chi_from_counts = counted.characteristic();
        if chi != chi_from_counts {
            problem(format!(
                "[{variant}] euler_characteristic {chi} does not match the counts ({chi_from_counts})"
            ));
        }
        let non_manifold = matches!(
            a.expect_error,
            Some(ExpectError::NonManifold | ExpectError::TangentContact)
        );
        let genus = match (m.genus, non_manifold) {
            (Some(genus), _) => Some(genus),
            // Solids sharing an edge or a vertex, or faces sharing a
            // tangent contact as an edge of four, close no Euler line.
            (None, true) => {
                if a.genus.is_some() {
                    problem(format!(
                        "[{variant}] analytic.genus is set for a non-manifold result, which has none"
                    ));
                }
                None
            }
            (None, false) => {
                problem(format!("[{variant}] expected.json lacks genus"));
                continue;
            }
        };
        if let (Some(g), Some(genus)) = (a.genus, genus) {
            // The Euler line: V − E + F − (L − F) − 2(S − G) = 0. Under
            // `counts_differ` the analytic genus is Arris's, and the
            // oracle's counts close at the oracle's own.
            let at = if a.counts_differ.is_some() { genus } else { g };
            let line = counted.at_genus(at);
            if line != 0 {
                problem(format!(
                    "[{variant}] Euler line is {line}, not 0: counts {c:?} with analytic genus {g} (oracle genus {genus})"
                ));
            }
            if let (Some(_), Some(ac)) = (&a.counts_differ, a.counts) {
                let line = EulerLine::new(ac.vertices, ac.edges, ac.faces, ac.loops, ac.shells)
                    .at_genus(g);
                if line != 0 {
                    problem(format!(
                        "[{variant}] Euler line of Arris's counts is {line}, not 0: {ac:?} with analytic genus {g}"
                    ));
                }
            }
        }
        // The measurements that differ from their closed forms: a problem
        // each, or under `measure_differs` the claim that at least one
        // does (ADR-0015).
        let mut differs = Vec::new();
        if let Some(v) = &a.volume {
            match (v.eval(&params), m.volume) {
                (Ok(av), Some(ov)) if !rel(av, ov) => {
                    differs.push(format!("analytic volume {av} vs oracle {ov}"))
                }
                (Err(e), _) => problem(format!("[{variant}] analytic volume: {e}")),
                _ => {}
            }
        }
        if let Some(v) = &a.area {
            match (v.eval(&params), m.area) {
                (Ok(av), Some(ov)) if !rel(av, ov) => {
                    differs.push(format!("analytic area {av} vs oracle {ov}"))
                }
                (Err(e), _) => problem(format!("[{variant}] analytic area: {e}")),
                _ => {}
            }
        }
        if let (Some(cent), Some(oc)) = (&a.centroid, m.centroid) {
            for (i, n) in cent.iter().enumerate() {
                match n.eval(&params) {
                    Ok(av) if (av - oc[i]).abs() > r.tolerances.centroid_abs.max(ANALYTIC_REL) => {
                        differs.push(format!("analytic centroid[{i}] {av} vs oracle {}", oc[i]));
                    }
                    Err(e) => problem(format!("[{variant}] analytic centroid: {e}")),
                    _ => {}
                }
            }
        }
        if let (Some(tensor), Some(oi)) = (&a.inertia, m.inertia) {
            // Relative to the tensor's largest component, as the runner
            // compares it: a product of inertia that cancels to zero is
            // not compared against itself.
            let scale = oi
                .iter()
                .flatten()
                .fold(0.0f64, |s, x| s.max(x.abs()))
                .max(f64::MIN_POSITIVE);
            for (i, row) in tensor.iter().enumerate() {
                for (j, n) in row.iter().enumerate() {
                    match n.eval(&params) {
                        Ok(av) if (av - oi[i][j]).abs() > ANALYTIC_REL * scale => {
                            differs.push(format!(
                                "analytic inertia[{i}][{j}] {av} vs oracle {}",
                                oi[i][j]
                            ));
                        }
                        Err(e) => problem(format!("[{variant}] analytic inertia: {e}")),
                        _ => {}
                    }
                }
            }
        }
        if a.measure_differs.is_none() {
            for d in differs {
                problem(format!("[{variant}] {d}"));
            }
        } else if differs.is_empty() {
            problem(format!(
                "[{variant}] analytic.measure_differs is set but every closed form is the oracle's measurement"
            ));
        }
        if let Some(ac) = a.counts {
            if a.counts_differ.is_none() && ac != c {
                problem(format!(
                    "[{variant}] analytic counts {ac:?} vs oracle {c:?}"
                ));
            }
            if a.counts_differ.is_some() && ac == c {
                problem(format!(
                    "[{variant}] analytic.counts_differ is set but the counts {ac:?} are the oracle's"
                ));
            }
        }
        let by_label: BTreeMap<&str, Class> = m
            .probes
            .iter()
            .map(|p| (p.label.as_str(), p.class))
            .collect();
        for p in &r.probes {
            match (p.expect, by_label.get(p.label.as_str())) {
                (_, None) => problem(format!(
                    "[{variant}] probe {:?} has no oracle result",
                    p.label
                )),
                (Some(e), Some(o)) if e != *o => {
                    problem(format!(
                        "[{variant}] probe {:?} expected {e:?}, oracle says {o:?}",
                        p.label
                    ));
                }
                _ => {}
            }
        }
    }
    problems
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recipe_types_round_trip_the_grammar() {
        let text = r#"{
            "params": {"r": 4},
            "steps": [
                {"name": "sk", "op": "profile", "plane": {"origin": [0,0,0], "x": [1,0,0], "y": [0,1,0]},
                 "outer": {"start": [0,0], "segments": [{"line_to": [4,0]}, {"arc_to": [0,0], "via": [2,2]}]},
                 "holes": [{"circle": {"center": [2,1], "radius": "r / 8"}}]},
                {"name": "body", "op": "extrude", "profile": "sk", "direction": [0,0,1], "length": 3},
                {"name": "moved", "op": "transform", "of": "body", "translate": [1,0,0]}
            ],
            "result": "moved",
            "probes": [{"label": "p", "point": [1, 1, 1]}]
        }"#;
        let r: Recipe = serde_json::from_str(text).unwrap();
        assert_eq!(r.steps.len(), 3);
        assert_eq!(r.steps[0].name(), "sk");
        assert!(matches!(&r.steps[0], Step::Profile { holes, .. } if holes.len() == 1));
        assert!(matches!(&r.steps[2], Step::Transform { rotate: None, .. }));
        assert_eq!(r.probes[0].expect, None);
        assert_eq!(r.tolerances, Tolerances::default());
        assert_eq!(r.precision, PrecisionSpec::default());
        assert_eq!(r.variant_names(), ["default"]);
        let params = r.params_of("default").unwrap();
        if let Step::Profile { holes, .. } = &r.steps[0] {
            if let Loop::Circle { circle } = &holes[0] {
                assert_eq!(circle.radius.eval(&params), Ok(0.5));
            }
        }
        assert!(r.params_of("nope").is_none());
    }

    #[test]
    fn a_recipe_names_only_the_precision_fields_it_changes() {
        let text = r#"{
            "steps": [], "result": "x",
            "precision": {"default_tolerance": 1e-6}
        }"#;
        let r: Recipe = serde_json::from_str(text).unwrap();
        let p = r.precision.precision();
        assert_eq!(p.default_tolerance, 1e-6);
        assert_eq!(p.min_tolerance, Precision::DEFAULT.min_tolerance);
        assert_eq!(p.check_samples, Precision::DEFAULT.check_samples);
        assert!(p.is_consistent());
        // And it is outside the hash: the oracle never reads it.
        let with: serde_json::Value = serde_json::from_str(text).unwrap();
        let without: serde_json::Value =
            serde_json::from_str(r#"{"steps": [], "result": "x"}"#).unwrap();
        assert_eq!(recipe_hash(&with).unwrap(), recipe_hash(&without).unwrap());
    }

    #[test]
    fn hash_ignores_analytic_and_description() {
        let a: serde_json::Value = serde_json::from_str(
            r#"{"steps": [], "result": "x", "analytic": {"volume": 1}, "description": "a"}"#,
        )
        .unwrap();
        let b: serde_json::Value = serde_json::from_str(
            r#"{"description": "b", "result": "x", "steps": [], "analytic": {"volume": 2}}"#,
        )
        .unwrap();
        let c: serde_json::Value =
            serde_json::from_str(r#"{"steps": [1], "result": "x"}"#).unwrap();
        assert_eq!(recipe_hash(&a).unwrap(), recipe_hash(&b).unwrap());
        assert_ne!(recipe_hash(&a).unwrap(), recipe_hash(&c).unwrap());
        assert_eq!(recipe_hash(&a).unwrap().len(), 64);
    }

    #[test]
    fn hash_matches_the_oracle_on_a_known_input() {
        // sha256 of {"params":null,"probes":null,"result":"x","steps":[],"variants":null},
        // the canonical text the oracle's canonical_json produces.
        let v: serde_json::Value = serde_json::from_str(r#"{"steps": [], "result": "x"}"#).unwrap();
        let text = r#"{"params":null,"probes":null,"result":"x","steps":[],"variants":null}"#;
        let expected = Sha256::digest(text.as_bytes())
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect::<String>();
        assert_eq!(recipe_hash(&v).unwrap(), expected);
    }

    /// serde_json writes a float down to 1e-5 with a decimal point and
    /// below it with an exponent, and a positive exponent with its sign;
    /// the oracle's `canonical_json` follows those rules, where Python's
    /// `repr` switches below 1e-4.
    #[test]
    fn floats_are_hashed_as_serde_json_writes_them() {
        let near = |x: f64| serde_json::Value::from(x).to_string();
        assert_eq!(near(5.166967270501008e-5), "0.00005166967270501008");
        assert_eq!(near(-1.5e-5), "-0.000015");
        assert_eq!(near(1e-6), "1e-6");
        assert_eq!(near(1e16), "1e+16");
        assert_eq!(near(1e-4), "0.0001");
    }

    #[test]
    fn an_unknown_kind_is_an_error_as_it_is_in_python() {
        let v: serde_json::Value =
            serde_json::from_str(r#"{"kind": "wire", "steps": [], "result": "x"}"#).unwrap();
        assert_eq!(kind_of_raw(&v), Err("wire".to_string()));
        assert_eq!(recipe_hash(&v), Err("wire".to_string()));
    }
}
