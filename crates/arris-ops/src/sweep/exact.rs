//! The exact spans of a sweep (ADR-0063 §7): the two cases where a span's
//! faces are known outright and need no fit.
//!
//! A **line** span carries the profile by a translation, so it sweeps what
//! `extrude` does: a plane for a line, a cylinder for a circle normal to
//! the path, an elliptic cylinder for an ellipse normal to it and for a
//! circle or an ellipse oblique to it, and a vertex runs a straight rise.
//! An **arc** span carries the profile by a rotation about the arc's
//! axis; when the carried profile's plane holds that axis and no point of
//! it is on the axis, it sweeps what `revolve` does, and a vertex runs a
//! circle about the axis. Any other span — a helix, a NURBS segment, an
//! arc under a binormal that is not its axis, a tilted profile, a profile
//! on the axis, an elliptic segment — is fitted, and the caller says so
//! by [`exact_span`] returning `None`.

use arris_geom::{Curve, FrameField, PathSegment, ProfileEdge, Surface};
use arris_math::nalgebra::UnitQuaternion;
use arris_math::{Axis, Frame, Interval, Isometry, Point3, Tolerance, UnitVec3, Vec3};

use crate::error::OpError;

use super::revolve::{axis_in_plane, orient, rho_range, swept_surface};

/// The faces and rails of one exact span: for each loop and each walk
/// index `j`, the surface edge `j` sweeps and the curve the vertex edge
/// `j` starts at runs along, over `rail_range`.
pub(super) struct ExactSpan {
    pub surfaces: Vec<Vec<Surface>>,
    pub rails: Vec<Vec<Curve>>,
    pub rail_range: Interval,
}

/// `span` of `segment` as exact faces, or `None` where it is fitted.
/// `motion` carries the profile `plane` to the span's start and `end` to
/// its end; the faces are those of the carried profile `loops`.
pub(super) fn exact_span(
    field: &FrameField,
    span: usize,
    segment: &PathSegment,
    loops: &[Vec<ProfileEdge>],
    plane: &Frame,
    (motion, end): (&Isometry, &Isometry),
    tol: Tolerance,
) -> Result<Option<ExactSpan>, OpError> {
    let (lo, hi) = field.span_range(span);
    let Ok(range) = Interval::new(0.0, hi - lo) else {
        return Ok(None);
    };
    // What the span's motion has to be, from the carried start to the
    // carried end; the field's own motions must agree with it.
    let relative = motion.inverse().then(end);
    match segment {
        PathSegment::LineTo(_) => {
            let t = field.at(lo).tangent;
            let shift = Isometry::from_translation(t.into_inner() * (hi - lo));
            if !agrees(&relative, &shift, &plane.origin(), tol) {
                return Ok(None);
            }
            line_span(loops, plane, motion, t, range, tol)
        }
        PathSegment::ArcTo { .. } => {
            let [p0, pm, p1] = [lo, 0.5 * (lo + hi), hi].map(|s| field.at(s).origin);
            let Some((centre, radius)) = circumcircle(&p0, &pm, &p1) else {
                return Ok(None);
            };
            let t0 = field.at(lo).tangent.into_inner();
            let Some(direction) = UnitVec3::try_new((p0 - centre).cross(&t0), 0.0) else {
                return Ok(None);
            };
            let angle = (hi - lo) / radius;
            let rotation = UnitQuaternion::from_axis_angle(&direction, angle);
            let about = Isometry::new(
                rotation,
                centre.coords - Isometry::from_rotation(rotation).apply_vec(centre.coords),
            );
            if !agrees(&relative, &about, &plane.origin(), tol) {
                return Ok(None);
            }
            let Ok(sweep) = Interval::new(0.0, angle) else {
                return Ok(None);
            };
            arc_span(
                loops,
                plane,
                motion,
                Axis {
                    origin: centre,
                    direction,
                },
                sweep,
                tol,
            )
        }
        PathSegment::Helix { .. } | PathSegment::Nurbs(_) => Ok(None),
    }
}

/// Whether the motion `got` is the motion `want` within the tolerances,
/// read as the turn between them and the miss of the point `probe`
/// (which is on the carried profile's plane at the span's start).
fn agrees(got: &Isometry, want: &Isometry, probe: &Point3, tol: Tolerance) -> bool {
    let turn = (got.rotation() * want.rotation().inverse()).angle();
    turn <= tol.angular && (got.apply(*probe) - want.apply(*probe)).norm() <= tol.linear
}

/// The centre and radius of the circle through three points, `None` when
/// they are collinear within rounding.
fn circumcircle(a: &Point3, b: &Point3, c: &Point3) -> Option<(Point3, f64)> {
    let (u, v) = (b - a, c - a);
    let w = u.cross(&v);
    let area2 = w.norm_squared();
    if area2 <= f64::EPSILON * u.norm_squared() * v.norm_squared() {
        return None;
    }
    let centre = a + (v * u.norm_squared() - u * v.norm_squared()).cross(&w) / (2.0 * area2);
    Some((centre, (a - centre).norm()))
}

/// A line span of tangent `t`: extrude's faces on the profile carried by
/// `motion`.
fn line_span(
    loops: &[Vec<ProfileEdge>],
    plane: &Frame,
    motion: &Isometry,
    t: UnitVec3,
    range: Interval,
    tol: Tolerance,
) -> Result<Option<ExactSpan>, OpError> {
    let carried_plane = plane.transformed(motion);
    let normal = carried_plane.z().into_inner();
    let oblique = t.cross(&normal).norm().atan2(t.dot(&normal).abs()) > tol.angular;
    let mut surfaces = Vec::with_capacity(loops.len());
    let mut rails = Vec::with_capacity(loops.len());
    for edges in loops {
        let mut row = Vec::with_capacity(edges.len());
        let mut rise = Vec::with_capacity(edges.len());
        for edge in edges {
            let curve = edge.curve.transformed(motion);
            let surface = match &curve {
                Curve::Line { direction, .. } => {
                    let Ok(frame) = Frame::new(
                        curve.point(edge.range.lo()),
                        direction.cross(&t),
                        direction.into_inner(),
                    ) else {
                        return Ok(None);
                    };
                    Surface::Plane { frame }
                }
                &Curve::Circle { ref frame, radius } if !oblique => {
                    let Ok(frame) = Frame::new(
                        frame.origin(),
                        t.into_inner(),
                        carried_plane.x().into_inner(),
                    ) else {
                        return Ok(None);
                    };
                    Surface::Cylinder { frame, radius }
                }
                &Curve::Ellipse {
                    ref frame,
                    major_radius,
                    minor_radius,
                } if !oblique => {
                    let Ok(frame) =
                        Frame::new(frame.origin(), t.into_inner(), frame.x().into_inner())
                    else {
                        return Ok(None);
                    };
                    Surface::EllipticCylinder {
                        frame,
                        major_radius,
                        minor_radius,
                    }
                }
                &Curve::Circle { ref frame, radius } => {
                    match projected(frame, radius, radius, &t) {
                        Some(surface) => surface,
                        None => return Ok(None),
                    }
                }
                &Curve::Ellipse {
                    ref frame,
                    major_radius,
                    minor_radius,
                } => match projected(frame, major_radius, minor_radius, &t) {
                    Some(surface) => surface,
                    None => return Ok(None),
                },
                // `Profile::edges` makes lines, circles and ellipses only.
                Curve::Nurbs(_) => return Ok(None),
            };
            row.push(surface);
            rise.push(Curve::Line {
                origin: curve.point(edge.range.lo()),
                direction: t,
            });
        }
        surfaces.push(row);
        rails.push(rise);
    }
    Ok(Some(ExactSpan {
        surfaces,
        rails,
        rail_range: range,
    }))
}

/// The elliptic cylinder along `t` that holds the conic with semi-axes
/// `a` along `frame`'s `X` and `b` along its `Y`, centred at the frame's
/// origin: its section perpendicular to `t` is the conic projected along
/// `t`, an ellipse whose axes are the singular directions of the
/// projection. `None` for a section that is not an ellipse (the conic is
/// seen edge on).
fn projected(frame: &Frame, a: f64, b: f64, t: &UnitVec3) -> Option<Surface> {
    let seat = Frame::from_z(frame.origin(), t.into_inner()).ok()?;
    let [e1, e2] = [seat.x().into_inner(), seat.y().into_inner()];
    let (x, y) = (frame.x().into_inner(), frame.y().into_inner());
    // The conic's image in the section's plane is `M (cos, sin)` with the
    // columns `a·X` and `b·Y` seen in `(e1, e2)`; its axes are the
    // eigenvectors of `M Mᵀ`.
    let (m11, m21) = (a * x.dot(&e1), a * x.dot(&e2));
    let (m12, m22) = (b * y.dot(&e1), b * y.dot(&e2));
    let s11 = m11 * m11 + m12 * m12;
    let s22 = m21 * m21 + m22 * m22;
    let s12 = m11 * m21 + m12 * m22;
    let mean = 0.5 * (s11 + s22);
    let spread = (0.25 * (s11 - s22) * (s11 - s22) + s12 * s12).sqrt();
    let (big, small) = (mean + spread, mean - spread);
    if small.is_nan() || small <= 0.0 {
        return None;
    }
    let angle = 0.5 * (2.0 * s12).atan2(s11 - s22);
    let (s, c) = angle.sin_cos();
    let major: Vec3 = e1 * c + e2 * s;
    let frame = Frame::new(frame.origin(), t.into_inner(), major).ok()?;
    Some(Surface::EllipticCylinder {
        frame,
        major_radius: big.sqrt(),
        minor_radius: small.sqrt(),
    })
}

/// An arc span about `axis` through `sweep`: revolve's faces on the
/// profile carried by `motion`, when the carried plane holds the axis and
/// every point of every loop is a line or a circle off it.
fn arc_span(
    loops: &[Vec<ProfileEdge>],
    plane: &Frame,
    motion: &Isometry,
    axis: Axis,
    sweep: Interval,
    tol: Tolerance,
) -> Result<Option<ExactSpan>, OpError> {
    if loops
        .iter()
        .flatten()
        .any(|e| matches!(e.curve, Curve::Ellipse { .. } | Curve::Nurbs(_)))
    {
        return Ok(None);
    }
    let carried_plane = plane.transformed(motion);
    let Ok((axis, in_plane)) = axis_in_plane(&carried_plane, axis, tol) else {
        return Ok(None);
    };
    let Ok(in_plane) = orient(loops, in_plane, tol) else {
        return Ok(None);
    };
    // A point on the axis sweeps no rise: those spans are fitted.
    let nearest = loops
        .iter()
        .flatten()
        .map(|e| rho_range(e, &in_plane).0)
        .fold(f64::INFINITY, f64::min);
    if nearest <= tol.linear {
        return Ok(None);
    }
    let radial = carried_plane.vec_to_world(Vec3::new(in_plane.radial.x, in_plane.radial.y, 0.0));
    let Ok(base) = Frame::new(axis.origin, axis.direction.into_inner(), radial) else {
        return Ok(None);
    };
    let mut surfaces = Vec::with_capacity(loops.len());
    let mut rails = Vec::with_capacity(loops.len());
    for edges in loops {
        let mut row = Vec::with_capacity(edges.len());
        let mut circles = Vec::with_capacity(edges.len());
        for edge in edges {
            let Ok((surface, _)) = swept_surface(edge, &in_plane, &base, |t| axis.at(t), tol)
            else {
                return Ok(None);
            };
            row.push(surface);
            circles.push(Curve::Circle {
                frame: base.with_origin(axis.at(in_plane.t(edge.start))),
                radius: in_plane.rho(edge.start),
            });
        }
        surfaces.push(row);
        rails.push(circles);
    }
    Ok(Some(ExactSpan {
        surfaces,
        rails,
        rail_range: sweep,
    }))
}
