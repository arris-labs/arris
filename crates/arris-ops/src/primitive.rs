//! The primitives: a box and a cylinder, built from numbers through the
//! Euler operators (`docs/DATA-MODEL.md` §Euler operators) with every
//! entity recorded under its [`Role`].

use core::f64::consts::TAU;

use arris_geom::{Curve, Curve2, Surface};
use arris_math::{
    Axis, Control, Frame, Frame2, Interval, Meter, Point2, Point3, UnitVec2, UnitVec3, Vec2, Vec3,
};
use arris_topo::builder::{
    BuildError, Builder, EdgeRef, FaceRef, Position, Seed, Split, Strut, VertexRef,
};
use arris_topo::entity::{BodyKind, EdgeGeometry};
use arris_topo::provenance::{BoxPart, Coord, CylinderPart, Side};
use arris_topo::{Body, Model, Orientation, Provenance, Role};

use crate::build::roles;
use crate::error::{Fault, OpError, Reason};
use crate::verify;

fn finite(what: &'static str, p: Point3) -> Result<(), OpError> {
    if p.coords.iter().all(|c| c.is_finite()) {
        Ok(())
    } else {
        Err(OpError::Degenerate {
            entities: Vec::new(),
            reason: Reason::NonFinite { what },
        })
    }
}

fn positive(what: &'static str, value: f64) -> Result<f64, OpError> {
    if !value.is_finite() {
        Err(OpError::Degenerate {
            entities: Vec::new(),
            reason: Reason::NonFinite { what },
        })
    } else if value <= 0.0 {
        Err(OpError::Degenerate {
            entities: Vec::new(),
            reason: Reason::NotPositive { what, value },
        })
    } else {
        Ok(value)
    }
}

/// A line edge's geometry from `p` to `q`, its range the length.
fn line(m: &mut Model, p: Point3, q: Point3) -> Result<EdgeGeometry, BuildError> {
    let d = q - p;
    let length = d.norm();
    let curve = m.add_curve(Curve::Line {
        origin: p,
        direction: UnitVec3::new_normalize(d),
    });
    // The endpoints were validated apart, so the length is finite and
    // positive; a failure here is the builder's `NotFound` class of bug.
    let range = Interval::new(0.0, length).map_err(|_| BuildError::Empty)?;
    Ok(EdgeGeometry::Curve { curve, range })
}

/// The pcurve of the line from `p` to `q` in `plane`, at the edge's own
/// parameter.
fn line_in_plane(m: &mut Model, plane: &Frame, p: Point3, q: Point3) -> arris_topo::Curve2Id {
    let origin = plane.to_local(p);
    let direction = plane.vec_to_local(q - p);
    m.add_curve2(Curve2::Line {
        origin: Point2::new(origin.x, origin.y),
        direction: UnitVec2::new_normalize(Vec2::new(direction.x, direction.y)),
    })
}

/// Gives every use of every planar face its pcurve: the edge's line in
/// the face's plane. `plane_of` maps a face to its frame.
fn plane_pcurves(
    m: &mut Model,
    b: &mut Builder,
    plane_of: impl Fn(FaceRef) -> Result<Frame, OpError>,
) -> Result<(), OpError> {
    let mut wanted = Vec::new();
    for (f, face) in b.faces() {
        let frame = plane_of(f)?;
        for (li, lp) in face.loops().iter().enumerate() {
            for (ci, u) in lp.uses().iter().enumerate() {
                let e = b.edge(u.edge)?;
                let p = b.vertex(e.start())?.point();
                let q = b.vertex(e.end())?.point();
                wanted.push((Position::new(f, li, ci), p, q, frame));
            }
        }
    }
    for (at, p, q, frame) in wanted {
        let pcurve = line_in_plane(m, &frame, p, q);
        b.set_pcurve(at, pcurve)?;
    }
    Ok(())
}

/// The axis-aligned box from `min` to `max` as a solid: eight vertices,
/// twelve line edges, six planar faces of one loop each, every plane's
/// `Z` its face's outward normal and every face used `Forward`, every
/// tolerance the model's `default_tolerance`. Built through the Euler
/// operators — a rectangle of struts closed by `mef`, four struts up,
/// four `mef`s for the sides — and recorded as `Generated` from
/// [`BoxPart`] roles: a face by its outward normal's coordinate and side,
/// an edge by the coordinate it runs along and its sides, a vertex by its
/// three sides.
///
/// Errors: [`OpError::Degenerate`] with [`Reason::NonFinite`] for a
/// coordinate that is not finite, [`Reason::NotPositive`] naming the
/// extent when `min` is not strictly below `max` along every coordinate.
/// The model is untouched on error.
///
/// ```
/// use arris_ops::primitive_box;
/// use arris_topo::Model;
/// use arris_math::Point3;
/// use arris_topo::provenance::{BoxPart, Coord, Role, Side};
///
/// let mut m = Model::default();
/// let (body, provenance) = primitive_box(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0), &arris_ops::Control::NONE).unwrap();
/// assert_eq!(m.faces(body).unwrap().len(), 6);
/// let top = Role::Box(BoxPart::Face(Coord::Z, Side::Max));
/// assert_eq!(provenance.generated_from(top).len(), 1);
/// ```
pub fn primitive_box(
    m: &mut Model,
    min: impl Into<Point3>,
    max: impl Into<Point3>,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    let (min, max) = (min.into(), max.into());
    finite("min", min)?;
    finite("max", max)?;
    positive("x extent", max.x - min.x)?;
    positive("y extent", max.y - min.y)?;
    positive("z extent", max.z - min.z)?;
    Meter::new(control).tick()?;
    let side = |value: f64, lo: f64| if value == lo { Side::Min } else { Side::Max };
    let vertex_role = |p: Point3| {
        Role::Box(BoxPart::Vertex([
            side(p.x, min.x),
            side(p.y, min.y),
            side(p.z, min.z),
        ]))
    };
    let edge_role = |p: Point3, q: Point3| {
        let (along, sides) = if p.x != q.x {
            (Coord::X, [side(p.y, min.y), side(p.z, min.z)])
        } else if p.y != q.y {
            (Coord::Y, [side(p.x, min.x), side(p.z, min.z)])
        } else {
            (Coord::Z, [side(p.x, min.x), side(p.y, min.y)])
        };
        Role::Box(BoxPart::Edge { along, sides })
    };
    let tol = m.precision().default_tolerance;
    // Bottom corners counter-clockwise from above, and the top ones over
    // them.
    let b = [
        Point3::new(min.x, min.y, min.z),
        Point3::new(max.x, min.y, min.z),
        Point3::new(max.x, max.y, min.z),
        Point3::new(min.x, max.y, min.z),
    ];
    let t: [Point3; 4] = core::array::from_fn(|i| Point3::new(b[i].x, b[i].y, max.z));
    let face_roles = [
        Role::Box(BoxPart::Face(Coord::Z, Side::Min)),
        Role::Box(BoxPart::Face(Coord::Z, Side::Max)),
        Role::Box(BoxPart::Face(Coord::Y, Side::Min)),
        Role::Box(BoxPart::Face(Coord::X, Side::Max)),
        Role::Box(BoxPart::Face(Coord::Y, Side::Max)),
        Role::Box(BoxPart::Face(Coord::X, Side::Min)),
    ];
    let side_normals = [-Vec3::y(), Vec3::x(), Vec3::y(), -Vec3::x()];
    let mut frames = Vec::with_capacity(6);
    frames.push(Frame::new(b[0], -Vec3::z(), Vec3::x())?);
    frames.push(Frame::new(t[0], Vec3::z(), Vec3::x())?);
    for i in 0..4 {
        frames.push(Frame::new(b[i], side_normals[i], b[(i + 1) % 4] - b[i])?);
    }
    m.transaction(|m| {
        let surfaces: Vec<_> = frames
            .iter()
            .map(|&frame| m.add_surface(Surface::Plane { frame }))
            .collect();
        let mut bd = Builder::new(tol);
        let mut face_of = Vec::new();
        let (v0, f_bottom) = bd.mvfs(Seed {
            point: b[0],
            surface: surfaces[0],
            orientation: Orientation::Forward,
        })?;
        face_of.push((f_bottom, 0));
        let mut corners = vec![v0];
        for i in 1..4 {
            let at = bd.find_position(f_bottom, 0, corners[i - 1])?;
            let strut = Strut {
                point: b[i],
                geometry: line(m, b[i - 1], b[i])?,
                pcurves: [None, None],
            };
            corners.push(bd.mev(at, strut)?.0);
        }
        let from = bd.find_position(f_bottom, 0, corners[3])?;
        let to = bd.find_position(f_bottom, 0, corners[0])?;
        let split = Split {
            geometry: line(m, b[3], b[0])?,
            surface: surfaces[1],
            orientation: Orientation::Forward,
            pcurves: [None, None],
        };
        let (_, f_top) = bd.mef(from, to, split)?;
        face_of.push((f_top, 1));
        let mut tops = Vec::with_capacity(4);
        for i in 0..4 {
            let at = bd.find_position(f_top, 0, corners[i])?;
            let strut = Strut {
                point: t[i],
                geometry: line(m, b[i], t[i])?,
                pcurves: [None, None],
            };
            tops.push(bd.mev(at, strut)?.0);
        }
        for i in 0..4 {
            let from = bd.find_position(f_top, 0, tops[(i + 1) % 4])?;
            let to = bd.find_position(f_top, 0, tops[i])?;
            let split = Split {
                geometry: line(m, t[(i + 1) % 4], t[i])?,
                surface: surfaces[2 + i],
                orientation: Orientation::Forward,
                pcurves: [None, None],
            };
            let (_, f) = bd.mef(from, to, split)?;
            face_of.push((f, 2 + i));
        }
        let invariant = |what: &'static str| OpError::Internal(Fault::Invariant { what });
        let which = |f: FaceRef| -> Result<usize, OpError> {
            face_of
                .iter()
                .find(|(r, _)| *r == f)
                .map(|(_, i)| *i)
                .ok_or_else(|| invariant("a box face's role index"))
        };
        plane_pcurves(m, &mut bd, |f| Ok(frames[which(f)?]))?;
        let points: Vec<(VertexRef, Point3)> = bd.vertices().map(|(v, s)| (v, s.point())).collect();
        let point_of = |v: VertexRef| -> Result<Point3, OpError> {
            points
                .iter()
                .find(|(r, _)| *r == v)
                .map(|(_, p)| *p)
                .ok_or_else(|| invariant("a box vertex's point"))
        };
        let ends: Vec<(EdgeRef, Point3, Point3)> = bd
            .edges()
            .map(|(e, s)| Ok((e, point_of(s.start())?, point_of(s.end())?)))
            .collect::<Result<_, OpError>>()?;
        let built = bd.finish(m, BodyKind::Solid)?;
        verify(m, built.body)?;
        let provenance = roles(
            &built,
            |v| Ok(vertex_role(point_of(v)?)),
            |e| {
                ends.iter()
                    .find(|(r, _, _)| *r == e)
                    .map(|(_, p, q)| edge_role(*p, *q))
                    .ok_or_else(|| invariant("a box edge's endpoints"))
            },
            |f| Ok(face_roles[which(f)?]),
            |_| Ok(Role::Box(BoxPart::Shell)),
            Role::Box(BoxPart::Body),
        )?;
        Ok((built.body, provenance))
    })
}

/// A cylinder of `radius` and `height` along `axis` with its bottom cap
/// centred at the axis origin, as a solid: two vertices on the seam, a
/// bottom circle, the seam line, a top circle; one wall face on the
/// cylinder whose one loop is bottom circle, seam up, top circle, seam
/// down (the seam pcurves at `u = 2π` and `u = 0`); two cap faces on
/// planes whose `Z` is the axis, the bottom cap used `Reversed`, as the
/// reference tree's one-axis primitive places them. The cylinder's frame
/// is `Frame::from_z` of the axis, so it seams where Open CASCADE's
/// does. Built as `mvfs → mef → mev → mef` through the Euler operators
/// and recorded as `Generated` from [`CylinderPart`] roles. Every
/// tolerance is the model's `default_tolerance`.
///
/// Errors: [`OpError::Degenerate`] with [`Reason::NotPositive`] or
/// [`Reason::NonFinite`] naming the radius, the height or the axis
/// origin. The model is untouched on error.
///
/// ```
/// use arris_ops::primitive_cylinder;
/// use arris_topo::Model;
/// use arris_math::{Axis, Point3};
/// use arris_topo::provenance::{CylinderPart, Role};
///
/// let mut m = Model::default();
/// let (body, provenance) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 4.0, 12.0, &arris_ops::Control::NONE).unwrap();
/// assert_eq!(m.edges(body).unwrap().len(), 3, "the seam once");
/// assert_eq!(provenance.generated_from(Role::Cylinder(CylinderPart::Seam)).len(), 1);
/// ```
pub fn primitive_cylinder(
    m: &mut Model,
    axis: Axis,
    radius: f64,
    height: f64,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    finite("axis origin", axis.origin)?;
    let radius = positive("radius", radius)?;
    let height = positive("height", height)?;
    Meter::new(control).tick()?;
    let tol = m.precision().default_tolerance;
    let base = Frame::from_z(axis.origin, axis.direction.into_inner())?;
    let top = base.with_origin(axis.at(height));
    let bottom_point = base.origin() + radius * base.x().into_inner();
    let top_point = bottom_point + height * base.z().into_inner();
    m.transaction(|m| {
        let wall = m.add_surface(Surface::Cylinder {
            frame: base,
            radius,
        });
        let bottom_plane = m.add_surface(Surface::Plane { frame: base });
        let top_plane = m.add_surface(Surface::Plane { frame: top });
        let uv_line = |m: &mut Model, u: f64, v: f64, along_u: bool| {
            m.add_curve2(Curve2::Line {
                origin: Point2::new(u, v),
                direction: if along_u {
                    Vec2::x_axis()
                } else {
                    Vec2::y_axis()
                },
            })
        };
        // A cap's circle in the cap plane's own (u, v): the plane shares
        // the cylinder's frame, so it is the identity circle.
        let cap_circle = |m: &mut Model| {
            m.add_curve2(Curve2::Circle {
                frame: Frame2::identity(),
                radius,
            })
        };
        let mut bd = Builder::new(tol);
        let (v0, f_bottom) = bd.mvfs(Seed {
            point: bottom_point,
            surface: bottom_plane,
            orientation: Orientation::Reversed,
        })?;
        let at = Position::new(f_bottom, 0, 0);
        let bottom_circle = m.add_curve(Curve::Circle {
            frame: base,
            radius,
        });
        let (p_wall_bottom, p_cap_bottom) = (uv_line(m, 0.0, 0.0, true), cap_circle(m));
        let (e_bottom, f_wall) = bd.mef(
            at,
            at,
            Split {
                geometry: EdgeGeometry::Curve {
                    curve: bottom_circle,
                    range: Interval::TURN,
                },
                surface: wall,
                orientation: Orientation::Forward,
                pcurves: [Some(p_wall_bottom), Some(p_cap_bottom)],
            },
        )?;
        let seam = m.add_curve(Curve::Line {
            origin: bottom_point,
            direction: base.z(),
        });
        let (p_up, p_down) = (uv_line(m, TAU, 0.0, false), uv_line(m, 0.0, 0.0, false));
        let rise = Interval::new(0.0, height).map_err(|_| BuildError::Empty)?;
        let at = bd.find_position(f_wall, 0, v0)?;
        let (v1, e_seam) = bd.mev(
            at,
            Strut {
                point: top_point,
                geometry: EdgeGeometry::Curve {
                    curve: seam,
                    range: rise,
                },
                pcurves: [Some(p_up), Some(p_down)],
            },
        )?;
        let top_circle = m.add_curve(Curve::Circle { frame: top, radius });
        let (p_wall_top, p_cap_top) = (uv_line(m, 0.0, height, true), cap_circle(m));
        let at = bd.find_position(f_wall, 0, v1)?;
        let (e_top, f_top) = bd.mef(
            at,
            at,
            Split {
                geometry: EdgeGeometry::Curve {
                    curve: top_circle,
                    range: Interval::TURN,
                },
                surface: top_plane,
                orientation: Orientation::Forward,
                pcurves: [Some(p_cap_top), Some(p_wall_top)],
            },
        )?;
        let built = bd.finish(m, BodyKind::Solid)?;
        verify(m, built.body)?;
        let part = |p: CylinderPart| Role::Cylinder(p);
        let provenance = roles(
            &built,
            |v| {
                Ok(part(if v == v0 {
                    CylinderPart::BottomVertex
                } else {
                    CylinderPart::TopVertex
                }))
            },
            |e| {
                Ok(part(if e == e_bottom {
                    CylinderPart::BottomRim
                } else if e == e_seam {
                    CylinderPart::Seam
                } else if e == e_top {
                    CylinderPart::TopRim
                } else {
                    CylinderPart::Body
                }))
            },
            |f| {
                Ok(part(if f == f_bottom {
                    CylinderPart::BottomCap
                } else if f == f_wall {
                    CylinderPart::Wall
                } else if f == f_top {
                    CylinderPart::TopCap
                } else {
                    CylinderPart::Body
                }))
            },
            |_| Ok(part(CylinderPart::Shell)),
            part(CylinderPart::Body),
        )?;
        Ok((built.body, provenance))
    })
}
