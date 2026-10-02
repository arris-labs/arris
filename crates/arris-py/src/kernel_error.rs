//! The kernel's errors as Python exceptions.
//!
//! Each kernel error enum has one function here, an exhaustive `match` with
//! no wildcard arm: a variant added in the kernel stops this crate compiling
//! until it has a class and its attributes (`.agents/rules/kernel.md` §API).
//! The functions are pure — they build a [`Mapped`], the class, the message
//! and the attributes — so a test can check the mapping without an
//! interpreter; [`Mapped::raise`] makes the exception.
//!
//! Variants that name the same condition share one class: every `NotFound`
//! is `StaleHandleError` and every `Interrupted` is `Interrupted`, so a
//! caller catches each once, whichever operation raised it. A nested kernel
//! error that has no structure a caller would branch on (a checker report, a
//! builder refusal, a profile loop's index) is the exception's `detail` text.

use arris::geom::{FitError, GeomError};
use arris::io::body::BodyError;
use arris::io::step::StepError;
use arris::mesh::MeshError;
use arris::ops::OpError;
use arris::topo::{AnyId, EntityId, Shape, TopoError};
use arris::{Interrupted, Stop};
use pyo3::IntoPyObjectExt;
use pyo3::prelude::*;
use pyo3::types::PyTuple;

use crate::error::Class;
use crate::handle::{Body, Edge, Face, Shell, Vertex};
use crate::model::Model;

/// A value an exception carries as an attribute.
#[derive(Debug, Clone, PartialEq)]
pub enum Attr {
    /// Text.
    Str(String),
    /// A count or an index.
    Uint(u64),
    /// A real number.
    Float(f64),
    /// An entity, as a handle of the model the call was made on.
    Shape(Shape),
    /// An id of an entity or a geometry value: a handle for an entity, its
    /// text form (`S3`) for a geometry value.
    Any(AnyId),
    /// Several entities, as a list of handles.
    Shapes(Vec<Shape>),
    /// Several real numbers, as a tuple.
    Floats(Vec<f64>),
}

/// A kernel error as the exception that will be raised for it.
#[derive(Debug, Clone, PartialEq)]
pub struct Mapped {
    /// The exception class.
    pub class: Class,
    /// The exception's message: the kernel error's own text.
    pub message: String,
    /// The attributes set on the exception, in order.
    pub attrs: Vec<(&'static str, Attr)>,
}

impl Mapped {
    fn new(class: Class, message: impl ToString) -> Self {
        Mapped {
            class,
            message: message.to_string(),
            attrs: Vec::new(),
        }
    }

    fn with(mut self, name: &'static str, attr: Attr) -> Self {
        self.attrs.push((name, attr));
        self
    }

    fn text(self, name: &'static str, text: impl ToString) -> Self {
        self.with(name, Attr::Str(text.to_string()))
    }

    /// The exception, with its attributes set. Entities become handles of
    /// `model`; with `None` (ids that belong to some other model, as a body
    /// read from bytes names) they stay as their text form.
    pub fn raise(self, py: Python<'_>, model: Option<&Model>) -> PyErr {
        let error = self.class.new_err(self.message);
        let value = error.value(py);
        for (name, attr) in self.attrs {
            let set = attr
                .into_py(py, model)
                .and_then(|object| value.setattr(name, object));
            if let Err(failure) = set {
                return failure;
            }
        }
        error
    }
}

impl Attr {
    fn into_py<'py>(self, py: Python<'py>, model: Option<&Model>) -> PyResult<Bound<'py, PyAny>> {
        match self {
            Attr::Str(text) => text.into_bound_py_any(py),
            Attr::Uint(n) => n.into_bound_py_any(py),
            Attr::Float(x) => x.into_bound_py_any(py),
            Attr::Shape(shape) => entity(py, model, shape),
            Attr::Any(AnyId::Entity(id)) => entity(py, model, Shape::new(id, Default::default())),
            Attr::Any(AnyId::Geometry(id)) => id.to_string().into_bound_py_any(py),
            Attr::Shapes(shapes) => shapes
                .into_iter()
                .map(|shape| entity(py, model, shape))
                .collect::<PyResult<Vec<_>>>()?
                .into_bound_py_any(py),
            Attr::Floats(xs) => PyTuple::new(py, xs)?.into_bound_py_any(py),
        }
    }
}

pub(crate) fn entity<'py>(
    py: Python<'py>,
    model: Option<&Model>,
    shape: Shape,
) -> PyResult<Bound<'py, PyAny>> {
    let Some(model) = model else {
        return shape.to_string().into_bound_py_any(py);
    };
    use arris::topo::{Body as K, Edge as E, Face as F, Shell as S, Vertex as V};
    let o = shape.orientation;
    match shape.id {
        EntityId::Body(id) => Body::minted_by(model, K::new(id, o)).into_bound_py_any(py),
        EntityId::Shell(id) => Shell::minted_by(model, S::new(id, o)).into_bound_py_any(py),
        EntityId::Face(id) => Face::minted_by(model, F::new(id, o)).into_bound_py_any(py),
        EntityId::Edge(id) => Edge::minted_by(model, E::new(id, o)).into_bound_py_any(py),
        EntityId::Vertex(id) => Vertex::minted_by(model, V::new(id, o)).into_bound_py_any(py),
    }
}

fn stale(id: AnyId, message: impl ToString) -> Mapped {
    Mapped::new(Class::StaleHandleError, message).with("entity", Attr::Any(id))
}

fn interrupted(stop: &Interrupted) -> Mapped {
    let by = match stop.by {
        Stop::Poll => "poll",
        Stop::Budget => "budget",
    };
    Mapped::new(Class::Interrupted, stop)
        .text("by", by)
        .with("steps", Attr::Uint(stop.steps))
}

/// An operation's error.
pub fn op_error(error: &OpError) -> Mapped {
    let message = error.to_string();
    match error {
        OpError::InvalidInput { body, report } => Mapped::new(Class::OpInvalidInputError, message)
            .with("body", Attr::Shape(body.shape()))
            .text("report", report),
        OpError::Unsupported { a, b } => Mapped::new(Class::OpUnsupportedError, message)
            .text("a_kind", a.0)
            .with("a", Attr::Shape(a.1))
            .text("b_kind", b.0)
            .with("b", Attr::Shape(b.1)),
        OpError::Degenerate { entities, reason } => Mapped::new(Class::OpDegenerateError, message)
            .with("entities", Attr::Shapes(entities.clone()))
            .text("reason", reason),
        OpError::Profile(detail) => {
            Mapped::new(Class::OpProfileError, message).text("detail", detail)
        }
        OpError::Tolerance { entity, wanted } => Mapped::new(Class::OpToleranceError, message)
            .with("entity", Attr::Shape(*entity))
            .with("wanted", Attr::Float(*wanted)),
        OpError::NotFound(id) => stale(*id, message),
        OpError::Internal(fault) => {
            Mapped::new(Class::OpInternalError, message).text("detail", fault)
        }
        OpError::Unkeyed { slot } => Mapped::new(Class::OpUnkeyedError, message).text("slot", slot),
        OpError::Rejected(rejection) => {
            Mapped::new(Class::OpRejectedError, message).text("detail", rejection)
        }
        OpError::Interrupted(stop) => interrupted(stop),
    }
}

/// A geometric query's error.
pub fn geom_error(error: &GeomError) -> Mapped {
    let message = error.to_string();
    match error {
        GeomError::Unsupported { a, b } => Mapped::new(Class::GeomUnsupportedError, message)
            .text("a_kind", a)
            .text("b_kind", b),
        GeomError::Degenerate { kind, reason } => Mapped::new(Class::GeomDegenerateError, message)
            .text("kind", kind)
            .text("reason", reason),
        GeomError::InvalidTolerance(tolerance) => {
            Mapped::new(Class::GeomInvalidToleranceError, message)
                .with("linear", Attr::Float(tolerance.linear))
                .with("angular", Attr::Float(tolerance.angular))
        }
        GeomError::Ambiguous { kind, locus, point } => {
            Mapped::new(Class::GeomAmbiguousError, message)
                .text("kind", kind)
                .text("locus", locus)
                .with("point", Attr::Floats(vec![point.x, point.y, point.z]))
        }
        GeomError::AmbiguousUv { kind, locus, point } => {
            Mapped::new(Class::GeomAmbiguousUvError, message)
                .text("kind", kind)
                .text("locus", locus)
                .with("point", Attr::Floats(vec![point.x, point.y]))
        }
        GeomError::NotOnSurface {
            curve,
            surface,
            t,
            distance,
        } => Mapped::new(Class::GeomNotOnSurfaceError, message)
            .text("curve", curve)
            .text("surface", surface)
            .with("t", Attr::Float(*t))
            .with("distance", Attr::Float(*distance)),
        GeomError::ThroughSingularity { curve, surface, t } => {
            Mapped::new(Class::GeomThroughSingularityError, message)
                .text("curve", curve)
                .text("surface", surface)
                .with("t", Attr::Float(*t))
        }
        GeomError::DegenerateSection { a, b, fault } => {
            Mapped::new(Class::GeomDegenerateSectionError, message)
                .text("a_kind", a)
                .text("b_kind", b)
                .text("fault", fault)
        }
        GeomError::Fit(fit) => fit_error(fit),
        GeomError::Interrupted(stop) => interrupted(stop),
    }
}

/// A NURBS fit's error.
pub fn fit_error(error: &FitError) -> Mapped {
    let message = error.to_string();
    match error {
        FitError::Degenerate(reason) => {
            Mapped::new(Class::FitDegenerateError, message).text("reason", reason)
        }
        FitError::InvalidTolerance(tolerance) => {
            Mapped::new(Class::FitInvalidToleranceError, message)
                .with("tolerance", Attr::Float(*tolerance))
        }
        FitError::NonFinite { t } => {
            Mapped::new(Class::FitNonFiniteError, message).with("t", Attr::Float(*t))
        }
        FitError::Diverged { spans, deviation } => Mapped::new(Class::FitDivergedError, message)
            .with("spans", Attr::Uint(*spans as u64))
            .with("deviation", Attr::Float(*deviation)),
        FitError::Interrupted(stop) => interrupted(stop),
    }
}

/// A topology error.
pub fn topo_error(error: &TopoError) -> Mapped {
    let message = error.to_string();
    match error {
        TopoError::NotFound(not_found) => stale(not_found.id, message),
        TopoError::Precision(precision) => Mapped::new(Class::TopoPrecisionError, message)
            .text("precision", format!("{precision:?}")),
    }
}

/// A STEP writer's error.
pub fn step_error(error: &StepError) -> Mapped {
    let message = error.to_string();
    match error {
        StepError::NotFound(not_found) => stale(not_found.id, message),
        StepError::Unsupported { body, what } => Mapped::new(Class::StepUnsupportedError, message)
            .with("body", Attr::Shape(body.shape()))
            .text("what", what),
        StepError::Lumps { body, source } => Mapped::new(Class::StepLumpsError, message)
            .with("body", Attr::Shape(body.shape()))
            .text("detail", source),
        StepError::NonFinite { id } => {
            Mapped::new(Class::StepNonFiniteError, message).with("id", Attr::Any(*id))
        }
        StepError::NoBodies => Mapped::new(Class::StepNoBodiesError, message),
        StepError::Tree(tree) => Mapped::new(Class::StepTreeError, message).text("detail", tree),
        StepError::Interrupted(stop) => interrupted(stop),
    }
}

/// A body-bytes error. The ids it names are the writer's or the decoded
/// model's, never the caller's, so a caller raises it with no model.
pub fn body_error(error: &BodyError) -> Mapped {
    let message = error.to_string();
    match error {
        BodyError::Magic => Mapped::new(Class::BodyMagicError, message),
        BodyError::Version { found, newest } => Mapped::new(Class::BodyVersionError, message)
            .with("found", Attr::Uint(u64::from(*found)))
            .with("newest", Attr::Uint(u64::from(*newest))),
        BodyError::Encode(detail) => {
            Mapped::new(Class::BodyEncodeError, message).text("detail", detail)
        }
        BodyError::Decode(detail) => {
            Mapped::new(Class::BodyDecodeError, message).text("detail", detail)
        }
        BodyError::Topo(topo) => topo_error(topo),
        BodyError::Precision {
            entity,
            tolerance,
            min,
            max,
        } => Mapped::new(Class::BodyPrecisionError, message)
            .text("entity", entity)
            .with("tolerance", Attr::Float(*tolerance))
            .with("min", Attr::Float(*min))
            .with("max", Attr::Float(*max)),
        BodyError::Rejected(report) => {
            Mapped::new(Class::BodyRejectedError, message).text("report", report)
        }
        BodyError::Interrupted(stop) => interrupted(stop),
    }
}

/// A tessellation or mesh error.
pub fn mesh_error(error: &MeshError) -> Mapped {
    let message = error.to_string();
    match error {
        MeshError::IndexOutOfRange { index, positions } => {
            Mapped::new(Class::MeshIndexOutOfRangeError, message)
                .with("index", Attr::Uint(u64::from(*index)))
                .with("positions", Attr::Uint(*positions as u64))
        }
        MeshError::RangeOutOfBounds { start, end, len } => {
            Mapped::new(Class::MeshRangeOutOfBoundsError, message)
                .with("start", Attr::Uint(*start as u64))
                .with("end", Attr::Uint(*end as u64))
                .with("len", Attr::Uint(*len as u64))
        }
        MeshError::NonFinitePosition { index } => {
            Mapped::new(Class::MeshNonFinitePositionError, message)
                .with("index", Attr::Uint(*index as u64))
        }
        MeshError::NotFound(not_found) => stale(not_found.id, message),
        MeshError::InvalidInput { body, report } => {
            Mapped::new(Class::MeshInvalidInputError, message)
                .with("body", Attr::Shape(body.shape()))
                .text("report", report)
        }
        MeshError::Chord(chord) => {
            Mapped::new(Class::MeshChordError, message).with("chord", Attr::Float(*chord))
        }
        MeshError::Face { face, source } => Mapped::new(Class::MeshFaceError, message)
            .with(
                "face",
                Attr::Shape(arris::topo::Face::forward(*face).shape()),
            )
            .text("detail", source),
        MeshError::GridTooLarge { face, points } => {
            Mapped::new(Class::MeshGridTooLargeError, message)
                .with(
                    "face",
                    Attr::Shape(arris::topo::Face::forward(*face).shape()),
                )
                .with("points", Attr::Uint(*points as u64))
        }
        MeshError::Corners(detail) => {
            Mapped::new(Class::MeshCornersError, message).text("detail", detail)
        }
        MeshError::Internal(detail) => {
            Mapped::new(Class::MeshInternalError, message).text("detail", detail)
        }
        MeshError::Interrupted(stop) => interrupted(stop),
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use arris::check::Report;
    use arris::geom::{AmbiguousLocus, GeomKind, ProfileError, SectionFault, SurfaceKind};
    use arris::io::step::{TreeError, Unsupported};
    use arris::math::{Point2, Point3, Precision, Tolerance};
    use arris::mesh::cdt::CdtError;
    use arris::ops::{BuildSlot, Fault, Reason, Rejection};
    use arris::topo::entity::BodyKind;
    use arris::topo::{BodyId, EntityId, FaceId, NotFound};

    use super::*;
    use crate::error::BindError;

    fn body() -> arris::topo::Body {
        arris::topo::Body::forward(BodyId::new(0, 0))
    }
    fn face() -> FaceId {
        FaceId::new(1, 0)
    }
    fn shape() -> Shape {
        arris::topo::Face::forward(face()).shape()
    }
    fn any() -> AnyId {
        EntityId::Face(face()).into()
    }
    fn stop() -> Interrupted {
        Interrupted {
            by: Stop::Budget,
            steps: 7,
        }
    }
    fn report() -> Box<Report> {
        Box::new(Report::new(Vec::new()))
    }

    /// One value of every variant of every error enum the binding maps.
    fn every_variant() -> Vec<Mapped> {
        let kind = GeomKind::Surface(SurfaceKind::Plane);
        let tolerance = Tolerance {
            linear: -1.0,
            angular: 0.0,
        };
        let all = vec![
            op_error(&OpError::InvalidInput {
                body: body(),
                report: report(),
            }),
            op_error(&OpError::Unsupported {
                a: (kind, shape()),
                b: (kind, shape()),
            }),
            op_error(&OpError::Degenerate {
                entities: vec![shape()],
                reason: Reason::ZeroThickness,
            }),
            op_error(&OpError::Profile(ProfileError::TooFewSegments {
                loop_index: 0,
            })),
            op_error(&OpError::Tolerance {
                entity: shape(),
                wanted: 1.0,
            }),
            op_error(&OpError::NotFound(any())),
            op_error(&OpError::Internal(Fault::NoNormal { face: face() })),
            op_error(&OpError::Unkeyed {
                slot: BuildSlot::Shell(0),
            }),
            op_error(&OpError::Rejected(Rejection::Kept(BuildSlot::Shell(0)))),
            op_error(&OpError::Interrupted(stop())),
            geom_error(&GeomError::Unsupported { a: kind, b: kind }),
            geom_error(&GeomError::Degenerate {
                kind,
                reason: String::new(),
            }),
            geom_error(&GeomError::InvalidTolerance(tolerance)),
            geom_error(&GeomError::Ambiguous {
                kind,
                locus: AmbiguousLocus::Axis,
                point: Point3::origin(),
            }),
            geom_error(&GeomError::AmbiguousUv {
                kind,
                locus: AmbiguousLocus::Axis,
                point: Point2::origin(),
            }),
            geom_error(&GeomError::NotOnSurface {
                curve: kind,
                surface: kind,
                t: 0.0,
                distance: 1.0,
            }),
            geom_error(&GeomError::ThroughSingularity {
                curve: kind,
                surface: kind,
                t: 0.0,
            }),
            geom_error(&GeomError::DegenerateSection {
                a: kind,
                b: kind,
                fault: SectionFault::TangentAlongCurve,
            }),
            geom_error(&GeomError::Fit(FitError::Degenerate(String::new()))),
            geom_error(&GeomError::Interrupted(stop())),
            fit_error(&FitError::Degenerate(String::new())),
            fit_error(&FitError::InvalidTolerance(0.0)),
            fit_error(&FitError::NonFinite { t: 0.0 }),
            fit_error(&FitError::Diverged {
                spans: 4096,
                deviation: 1.0,
            }),
            fit_error(&FitError::Interrupted(stop())),
            topo_error(&TopoError::NotFound(NotFound::new(body().id))),
            topo_error(&TopoError::Precision(Precision::default())),
            step_error(&StepError::NotFound(NotFound::new(body().id))),
            step_error(&StepError::Unsupported {
                body: body(),
                what: Unsupported::Kind(BodyKind::Solid),
            }),
            step_error(&StepError::Lumps {
                body: body(),
                source: arris::check::LumpError::NotFound(NotFound::new(body().id)),
            }),
            step_error(&StepError::NonFinite { id: any() }),
            step_error(&StepError::NoBodies),
            step_error(&StepError::Tree(TreeError::SolidShared { index: 0 })),
            step_error(&StepError::Interrupted(stop())),
            body_error(&BodyError::Magic),
            body_error(&BodyError::Version {
                found: 2,
                newest: 1,
            }),
            body_error(&BodyError::Encode(String::new())),
            body_error(&BodyError::Decode(String::new())),
            body_error(&BodyError::Topo(TopoError::NotFound(NotFound::new(
                body().id,
            )))),
            body_error(&BodyError::Precision {
                entity: EntityId::Face(face()),
                tolerance: 1.0,
                min: 0.0,
                max: 0.5,
            }),
            body_error(&BodyError::Rejected(report())),
            body_error(&BodyError::Interrupted(stop())),
            mesh_error(&MeshError::IndexOutOfRange {
                index: 3,
                positions: 2,
            }),
            mesh_error(&MeshError::RangeOutOfBounds {
                start: 0,
                end: 3,
                len: 2,
            }),
            mesh_error(&MeshError::NonFinitePosition { index: 0 }),
            mesh_error(&MeshError::NotFound(NotFound::new(body().id))),
            mesh_error(&MeshError::InvalidInput {
                body: body(),
                report: report(),
            }),
            mesh_error(&MeshError::Chord(0.0)),
            mesh_error(&MeshError::Face {
                face: face(),
                source: CdtError::Polygon {
                    polygon: 0,
                    points: 2,
                },
            }),
            mesh_error(&MeshError::GridTooLarge {
                face: face(),
                points: 1 << 30,
            }),
            mesh_error(&MeshError::Corners(String::new())),
            mesh_error(&MeshError::Internal("bookkeeping")),
            mesh_error(&MeshError::Interrupted(stop())),
        ];
        all
    }

    #[test]
    fn every_variant_maps_to_a_class_of_its_own_and_every_class_is_reached() {
        let mut produced: BTreeSet<Class> = every_variant().iter().map(|m| m.class).collect();
        // The binding's own refusals, from `BindError`.
        let (owner, this) = (1, 2);
        produced.insert(
            BindError::Foreign {
                entity: EntityId::Face(face()),
                owner,
                this,
            }
            .class(),
        );
        produced.insert(BindError::Stale(NotFound::new(face())).class());
        produced.insert(BindError::Poisoned { model: this }.class());
        let bases: BTreeSet<&str> = Class::ALL.iter().map(|c| c.base()).collect();
        // The families (`OpError`, `GeomError`, …) are bases, never raised bare.
        for class in Class::ALL {
            if bases.contains(class.name()) {
                assert!(
                    !produced.contains(class),
                    "{} is a family, not a variant",
                    class.name()
                );
            } else {
                assert!(
                    produced.contains(class),
                    "no kernel variant maps to {}",
                    class.name()
                );
            }
        }
    }

    #[test]
    fn the_shared_conditions_have_one_class_each() {
        let mapped = every_variant();
        let of = |c: Class| mapped.iter().filter(|m| m.class == c).count();
        assert_eq!(
            of(Class::StaleHandleError),
            5,
            "every NotFound, the one inside BodyError included"
        );
        assert_eq!(
            of(Class::Interrupted),
            6,
            "the Interrupted of every one of the six enums"
        );
    }

    #[test]
    fn a_boolean_s_unsupported_carries_both_operands_and_their_kinds() {
        let mapped = op_error(&OpError::Unsupported {
            a: (GeomKind::Surface(SurfaceKind::Plane), shape()),
            b: (
                GeomKind::Surface(SurfaceKind::Plane),
                arris::topo::Face::forward(FaceId::new(2, 0)).shape(),
            ),
        });
        assert_eq!(mapped.class, Class::OpUnsupportedError);
        assert_eq!(
            mapped.attrs[0],
            ("a_kind", Attr::Str("plane surface".into()))
        );
        assert_eq!(mapped.attrs[1].1, Attr::Shape(shape()));
        assert_eq!(mapped.attrs[3].0, "b");
    }

    #[test]
    fn an_interruption_says_what_stopped_it_and_after_how_many_steps() {
        let mapped = interrupted(&stop());
        assert_eq!(
            mapped.attrs,
            vec![("by", Attr::Str("budget".into())), ("steps", Attr::Uint(7))]
        );
    }

    #[test]
    fn class_names_are_unique_and_every_base_is_declared() {
        let names: BTreeSet<&str> = Class::ALL.iter().map(|c| c.name()).collect();
        assert_eq!(names.len(), Class::ALL.len());
        for class in Class::ALL {
            let base = class.base();
            assert!(
                base == "Exception" || names.contains(base),
                "{} derives from {base}",
                class.name()
            );
        }
    }
}
