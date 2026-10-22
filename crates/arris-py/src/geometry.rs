//! What an entity's geometry is, as Python values: an edge's curve, a
//! face's surface and the box an entity lies in.

use arris::check::domain::FaceDomain;
use arris::geom::{Curve, Surface};
use arris::math::{Aabb, Frame as KernelFrame, Point3, UnitVec3};
use arris::topo::{self, EntityId, NotFound, Orientation};
use pyo3::prelude::*;

use crate::query::Frame;

type Triple = (f64, f64, f64);

/// A box as Python sees it: `(min, max)`.
pub(crate) type Corners = (Triple, Triple);

fn point(p: Point3) -> Triple {
    (p.x, p.y, p.z)
}

fn vector(v: UnitVec3) -> Triple {
    (v.x, v.y, v.z)
}

/// The curve under an edge, from `Model.curve`: its `kind`, its ends in the
/// direction the edge was reached, and the closed form's parameters where
/// one exists.
///
/// `kind` is `"line"`, `"circle"`, `"ellipse"`, `"nurbs"`, or
/// `"degenerate"` for an edge with no 3D curve (at a pole or an apex).
/// `origin` is a line's point at `t = 0` or a conic's centre; `axis` a
/// line's direction or a conic's normal; `frame` a conic's frame, with `x`
/// at its `t = 0` (an ellipse's major axis). These are the curve's own,
/// unchanged by the edge's orientation; a parameter the kind has not is
/// `None`.
///
/// ```python
/// import arris
///
/// model = arris.Model()
/// body, _ = model.primitive_cylinder((0, 0, 0), (0, 0, 1), 2, 5)
/// circles = [model.curve(e) for e in model.edges(body)]
/// circles = [c for c in circles if c.kind == "circle"]
/// assert len(circles) == 2 and all(c.radius == 2.0 for c in circles)
/// assert sorted(c.origin[2] for c in circles) == [0.0, 5.0]
/// assert all(c.start == c.end for c in circles)
/// ```
#[pyclass(frozen, module = "arris")]
#[derive(Clone, Debug)]
pub struct CurveInfo {
    kind: &'static str,
    start: Point3,
    end: Point3,
    origin: Option<Point3>,
    axis: Option<UnitVec3>,
    frame: Option<KernelFrame>,
    radius: Option<f64>,
    radii: Option<(f64, f64)>,
}

impl CurveInfo {
    /// The value for `edge` of `model`, its ends swapped when reached
    /// reversed.
    pub(crate) fn of(model: &topo::Model, edge: topo::Edge) -> Result<CurveInfo, NotFound> {
        let entity = model.edge(edge.id)?;
        let (start, end) = (
            model.vertex(entity.start())?.point(),
            model.vertex(entity.end())?.point(),
        );
        let (start, end) = match edge.orientation {
            Orientation::Forward => (start, end),
            Orientation::Reversed => (end, start),
        };
        let mut info = CurveInfo {
            kind: "degenerate",
            start,
            end,
            origin: None,
            axis: None,
            frame: None,
            radius: None,
            radii: None,
        };
        let Some((curve, _)) = entity.curve() else {
            return Ok(info);
        };
        match model.curve(curve)? {
            &Curve::Line { origin, direction } => {
                info.kind = "line";
                info.origin = Some(origin);
                info.axis = Some(direction);
            }
            &Curve::Circle { frame, radius } => {
                info.kind = "circle";
                info.radius = Some(radius);
                info.conic(frame);
            }
            &Curve::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => {
                info.kind = "ellipse";
                info.radii = Some((major_radius, minor_radius));
                info.conic(frame);
            }
            Curve::Nurbs(_) => info.kind = "nurbs",
        }
        Ok(info)
    }

    fn conic(&mut self, frame: KernelFrame) {
        self.origin = Some(frame.origin());
        self.axis = Some(frame.z());
        self.frame = Some(frame);
    }
}

#[pymethods]
impl CurveInfo {
    /// `"line"`, `"circle"`, `"ellipse"`, `"nurbs"` or `"degenerate"`.
    #[getter]
    fn kind(&self) -> &'static str {
        self.kind
    }

    /// The point the edge starts at, in the direction it was reached.
    #[getter]
    fn start(&self) -> Triple {
        point(self.start)
    }

    /// The point the edge ends at: `start` again for a closed edge.
    #[getter]
    fn end(&self) -> Triple {
        point(self.end)
    }

    /// A line's point at `t = 0`, a conic's centre, else `None`.
    #[getter]
    fn origin(&self) -> Option<Triple> {
        self.origin.map(point)
    }

    /// A line's unit direction, a conic's unit normal, else `None`.
    #[getter]
    fn axis(&self) -> Option<Triple> {
        self.axis.map(vector)
    }

    /// A conic's frame (`z` its normal, `x` towards `t = 0`), else `None`.
    #[getter]
    fn frame(&self) -> Option<Frame> {
        self.frame.map(Frame::new)
    }

    /// A circle's radius, else `None`.
    #[getter]
    fn radius(&self) -> Option<f64> {
        self.radius
    }

    /// An ellipse's semi-axis along `frame.x`, else `None`.
    #[getter]
    fn major_radius(&self) -> Option<f64> {
        self.radii.map(|r| r.0)
    }

    /// An ellipse's semi-axis along `frame.y`, else `None`.
    #[getter]
    fn minor_radius(&self) -> Option<f64> {
        self.radii.map(|r| r.1)
    }

    fn __repr__(&self) -> String {
        format!(
            "CurveInfo(kind={:?}, start={:?}, end={:?})",
            self.kind,
            self.start(),
            self.end()
        )
    }
}

/// The surface under a face, from `Model.surface`: its `kind` and the
/// closed form's parameters where one exists.
///
/// `kind` is `"plane"`, `"cylinder"`, `"elliptic_cylinder"`, `"cone"`,
/// `"sphere"`, `"torus"` or `"nurbs"`. `frame` is the surface's own frame
/// (`None` for a NURBS) and `origin` its point: a plane's origin, the
/// centre of a sphere or a torus, the point on the axis of a cylinder or of
/// a cone's reference circle. `normal` is a plane's normal pointing out of
/// the face as it was reached (flipped for a reversed face), `axis` the
/// axis of every other analytic kind, as the frame's `z`.
///
/// ```python
/// import arris
///
/// model = arris.Model()
/// body, _ = model.primitive_cylinder((1, 2, 0), (0, 0, 1), 3, 4)
/// kinds = sorted(model.surface(f).kind for f in model.faces(body))
/// assert kinds == ["cylinder", "plane", "plane"]
/// [side] = [model.surface(f) for f in model.faces(body)
///           if model.surface(f).kind == "cylinder"]
/// assert side.radius == 3.0 and side.axis == (0.0, 0.0, 1.0)
/// normals = sorted(model.surface(f).normal for f in model.faces(body)
///                  if model.surface(f).kind == "plane")
/// assert normals == [(0.0, 0.0, -1.0), (0.0, 0.0, 1.0)]
/// ```
#[pyclass(frozen, module = "arris")]
#[derive(Clone, Debug)]
pub struct SurfaceInfo {
    kind: &'static str,
    frame: Option<KernelFrame>,
    normal: Option<UnitVec3>,
    radius: Option<f64>,
    radii: Option<(f64, f64)>,
    half_angle: Option<f64>,
}

impl SurfaceInfo {
    /// The value for `face` of `model`.
    pub(crate) fn of(model: &topo::Model, face: topo::Face) -> Result<SurfaceInfo, NotFound> {
        let entity = model.face(face.id)?;
        let mut info = SurfaceInfo {
            kind: "nurbs",
            frame: model.surface(entity.surface())?.frame().copied(),
            normal: None,
            radius: None,
            radii: None,
            half_angle: None,
        };
        match model.surface(entity.surface())? {
            &Surface::Plane { frame } => {
                info.kind = "plane";
                info.normal = Some(match face.orientation {
                    Orientation::Forward => frame.z(),
                    Orientation::Reversed => -frame.z(),
                });
            }
            &Surface::Cylinder { radius, .. } => {
                info.kind = "cylinder";
                info.radius = Some(radius);
            }
            &Surface::EllipticCylinder {
                major_radius,
                minor_radius,
                ..
            } => {
                info.kind = "elliptic_cylinder";
                info.radii = Some((major_radius, minor_radius));
            }
            &Surface::Cone {
                radius, half_angle, ..
            } => {
                info.kind = "cone";
                info.radius = Some(radius);
                info.half_angle = Some(half_angle);
            }
            &Surface::Sphere { radius, .. } => {
                info.kind = "sphere";
                info.radius = Some(radius);
            }
            &Surface::Torus {
                major_radius,
                minor_radius,
                ..
            } => {
                info.kind = "torus";
                info.radii = Some((major_radius, minor_radius));
            }
            Surface::Nurbs(_) => {}
        }
        Ok(info)
    }
}

#[pymethods]
impl SurfaceInfo {
    /// `"plane"`, `"cylinder"`, `"elliptic_cylinder"`, `"cone"`,
    /// `"sphere"`, `"torus"` or `"nurbs"`.
    #[getter]
    fn kind(&self) -> &'static str {
        self.kind
    }

    /// The surface's own frame, `None` for a NURBS.
    #[getter]
    fn frame(&self) -> Option<Frame> {
        self.frame.map(Frame::new)
    }

    /// The frame's origin, `None` for a NURBS.
    #[getter]
    fn origin(&self) -> Option<Triple> {
        self.frame.map(|f| point(f.origin()))
    }

    /// A plane's unit normal out of the face as reached, else `None`.
    #[getter]
    fn normal(&self) -> Option<Triple> {
        self.normal.map(vector)
    }

    /// The unit axis of a cylinder, a cone, a sphere (pole to pole) or a
    /// torus: the frame's `z`. `None` for a plane or a NURBS.
    #[getter]
    fn axis(&self) -> Option<Triple> {
        match self.kind {
            "plane" | "nurbs" => None,
            _ => self.frame.map(|f| vector(f.z())),
        }
    }

    /// The radius of a cylinder, a sphere, or a cone at its reference
    /// circle; else `None`.
    #[getter]
    fn radius(&self) -> Option<f64> {
        self.radius
    }

    /// A torus's radius to its tube's centre circle, an elliptic
    /// cylinder's semi-axis along `frame.x`; else `None`.
    #[getter]
    fn major_radius(&self) -> Option<f64> {
        self.radii.map(|r| r.0)
    }

    /// A torus's tube radius, an elliptic cylinder's semi-axis along
    /// `frame.y`; else `None`.
    #[getter]
    fn minor_radius(&self) -> Option<f64> {
        self.radii.map(|r| r.1)
    }

    /// A cone's angle between a ruling and its axis, in radians; else
    /// `None`.
    #[getter]
    fn half_angle(&self) -> Option<f64> {
        self.half_angle
    }

    fn __repr__(&self) -> String {
        match self.origin() {
            Some(o) => format!("SurfaceInfo(kind={:?}, origin={o:?})", self.kind),
            None => format!("SurfaceInfo(kind={:?})", self.kind),
        }
    }
}

/// The box `entity` lies in, grown by every tolerance it carries: exact
/// for a vertex, a line or a conic, the trimmed surface's box for an
/// analytic face, a control hull's for a NURBS one; a shell or a body is
/// the union of its entities'. `None` when one of its faces has no
/// finite box.
pub(crate) fn bounds(model: &topo::Model, entity: EntityId) -> Result<Option<Aabb>, NotFound> {
    let faces = match entity {
        EntityId::Vertex(v) => return vertex_box(model, v).map(Some),
        EntityId::Edge(e) => return edge_box(model, e).map(Some),
        EntityId::Face(f) => return face_box(model, f),
        EntityId::Shell(s) => model
            .shell(s)?
            .faces()
            .iter()
            .map(|f| f.id)
            .collect::<Vec<_>>(),
        EntityId::Body(b) => {
            // A wire or an acorn body has edges or vertices outside any face.
            let closure = model.closure(topo::Body::forward(b))?;
            let mut all: Option<Aabb> = None;
            for e in closure.edges {
                all = Some(union(all, edge_box(model, e)?));
            }
            for v in closure.vertices {
                all = Some(union(all, vertex_box(model, v)?));
            }
            return faces_box(model, &closure.faces, all);
        }
    };
    faces_box(model, &faces, None)
}

fn union(acc: Option<Aabb>, b: Aabb) -> Aabb {
    acc.map_or(b, |a| a.union(b))
}

fn faces_box(
    model: &topo::Model,
    faces: &[topo::FaceId],
    mut all: Option<Aabb>,
) -> Result<Option<Aabb>, NotFound> {
    for &face in faces {
        let Some(b) = face_box(model, face)? else {
            return Ok(None);
        };
        all = Some(union(all, b));
    }
    Ok(all)
}

fn vertex_box(model: &topo::Model, vertex: topo::VertexId) -> Result<Aabb, NotFound> {
    let v = model.vertex(vertex)?;
    Ok(Aabb::of_point(v.point()).inflated(v.tolerance()))
}

/// The curve's box over the edge's range and its two vertices' balls.
fn edge_box(model: &topo::Model, edge: topo::EdgeId) -> Result<Aabb, NotFound> {
    let entity = model.edge(edge)?;
    let mut b = vertex_box(model, entity.start())?.union(vertex_box(model, entity.end())?);
    if let Some((curve, range)) = entity.curve()
        && let Some(c) = model.curve(curve)?.bounds(range)
    {
        b = b.union(c.inflated(entity.tolerance()));
    }
    Ok(b)
}

/// The trimmed face's box, or for a face with no loop (a whole sphere or
/// torus) its surface's over the whole domain; with its edges' boxes.
fn face_box(model: &topo::Model, face: topo::FaceId) -> Result<Option<Aabb>, NotFound> {
    let entity = model.face(face)?;
    let tolerance = model.precision().parametric_tolerance;
    let surface = model.surface(entity.surface())?;
    let trimmed = FaceDomain::of(model, face, tolerance)?.bounds();
    let Some(mut b) = trimmed.or_else(|| {
        surface
            .bounds(surface.domain())
            .map(|b| b.inflated(entity.tolerance()))
    }) else {
        return Ok(None);
    };
    for coedge in entity.loops().iter().flat_map(|l| l.coedges()) {
        b = b.union(edge_box(model, coedge.edge())?);
    }
    Ok(Some(b))
}
