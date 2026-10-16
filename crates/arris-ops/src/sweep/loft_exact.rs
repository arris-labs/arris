//! The exact spans of a ruled loft (ADR-0065 §6): a face between two
//! matched segments is known outright where the pair is a kind a surface
//! holds, and is classified by *check* on the segments' geometry, not by the
//! consumer's segment kind — a split arc and a line raised to a rational
//! quadratic are what they are.
//!
//! Two straight segments whose four ends are coplanar give a `Plane`; two
//! circular arcs about one axis, in planes across it, at the same angles
//! along their parameter, give a `Cylinder` (equal radii) or a `Cone`.
//! Anything else stays a degree-one NURBS.

use arris_geom::{Curve, NurbsCurve, Surface};
use arris_math::real;
use arris_math::{Frame, Interval, Point3, Tolerance, UnitVec3};

/// Samples of a segment over its parameter range: enough to tell a circle
/// from any other rational quadratic, which is fixed by three points and
/// would have to meet it at thirteen more.
const SAMPLES: usize = 16;

/// What a matched segment is as a curve.
#[derive(Debug, Clone, Copy)]
pub(super) enum Kind {
    /// A straight segment from `from` to `to`.
    Line { from: Point3, to: Point3 },
    /// A circular arc: counter-clockwise about `normal` from the radial
    /// direction `x` at its start through `sweep` radians.
    Arc {
        centre: Point3,
        normal: UnitVec3,
        x: UnitVec3,
        radius: f64,
        sweep: f64,
    },
    /// Neither.
    Other,
}

fn samples(c: &NurbsCurve) -> Vec<Point3> {
    let d = c.domain();
    (0..=SAMPLES)
        .map(|i| c.eval(d.lerp(i as f64 / SAMPLES as f64)).point)
        .collect()
}

/// The distance of `p` from the line through `a` and `b`.
fn off_line(p: &Point3, a: &Point3, b: &Point3) -> f64 {
    (p - a).cross(&(b - a)).norm() / (b - a).norm()
}

/// What the segment `c` is, within the tolerance.
pub(super) fn kind_of(c: &NurbsCurve, tol: Tolerance) -> Kind {
    let points = samples(c);
    let (from, to) = (points[0], points[SAMPLES]);
    if (to - from).norm() > tol.linear
        && points.iter().all(|p| off_line(p, &from, &to) <= tol.linear)
    {
        return Kind::Line { from, to };
    }
    let (a, b, d) = (points[0], points[SAMPLES / 3], points[2 * SAMPLES / 3]);
    let (u, v) = (b - a, d - a);
    let w = u.cross(&v);
    let area2 = w.norm_squared();
    if area2 <= f64::EPSILON * u.norm_squared() * v.norm_squared() {
        return Kind::Other;
    }
    let centre = a + (v * u.norm_squared() - u * v.norm_squared()).cross(&w) / (2.0 * area2);
    let radius = (a - centre).norm();
    let (Some(normal), Some(x)) = (
        UnitVec3::try_new(w, 0.0),
        UnitVec3::try_new(a - centre, 0.0),
    ) else {
        return Kind::Other;
    };
    let y = normal.cross(&x);
    let on_circle = points.iter().all(|p| {
        ((p - centre).norm() - radius).abs() <= tol.linear
            && (p - centre).dot(&normal).abs() <= tol.linear
    });
    if !on_circle || radius <= tol.linear {
        return Kind::Other;
    }
    // The angle, unwrapped step by step: it must only grow.
    let mut sweep = 0.0;
    let mut last = 0.0;
    for p in &points[1..] {
        let q = p - centre;
        let angle = real::atan2(q.dot(&y), q.dot(&x));
        let mut step = angle - last;
        if step < -core::f64::consts::PI {
            step += core::f64::consts::TAU;
        }
        if step <= 0.0 {
            return Kind::Other;
        }
        sweep += step;
        last = angle;
    }
    // A closed segment is a whole turn to the bit, not to the rounding of
    // its sampled angles.
    let sweep = if (sweep - core::f64::consts::TAU).abs() * radius <= tol.linear {
        core::f64::consts::TAU
    } else {
        sweep
    };
    Kind::Arc {
        centre,
        normal,
        x,
        radius,
        sweep,
    }
}

/// The curve of a ring edge that is a line or an arc, with its range:
/// the line by arc length from its start, the circle by angle from `x`.
pub(super) fn analytic_curve(kind: &Kind) -> Option<(Curve, Interval)> {
    match *kind {
        Kind::Line { from, to } => {
            let direction = UnitVec3::try_new(to - from, 0.0)?;
            let range = Interval::new(0.0, (to - from).norm()).ok()?;
            Some((
                Curve::Line {
                    origin: from,
                    direction,
                },
                range,
            ))
        }
        Kind::Arc {
            centre,
            normal,
            x,
            radius,
            sweep,
        } => {
            let frame = Frame::new(centre, normal.into_inner(), x.into_inner()).ok()?;
            let range = Interval::new(0.0, sweep).ok()?;
            Some((Curve::Circle { frame, radius }, range))
        }
        Kind::Other => None,
    }
}

/// The surface of the face between segments `a` and `b` of kinds `ka` and
/// `kb`, or `None` where it is a NURBS.
pub(super) fn exact_face(
    (a, ka): (&NurbsCurve, &Kind),
    (b, kb): (&NurbsCurve, &Kind),
    tol: Tolerance,
) -> Option<Surface> {
    match (ka, kb) {
        (Kind::Line { from: p0, to: p1 }, Kind::Line { from: q0, to: q1 }) => {
            let candidates = [
                (p1 - p0).cross(&(q0 - p0)),
                (p1 - p0).cross(&(q1 - p0)),
                (q1 - q0).cross(&(p0 - q0)),
                (q1 - q0).cross(&(p1 - q0)),
            ];
            let normal = candidates
                .into_iter()
                .max_by(|x, y| x.norm_squared().total_cmp(&y.norm_squared()))?;
            let normal = UnitVec3::try_new(normal, 0.0)?;
            let flat = [q0, q1, p1]
                .iter()
                .all(|p| (*p - p0).dot(&normal).abs() <= tol.linear);
            if !flat {
                return None;
            }
            let frame = Frame::new(*p0, normal.into_inner(), p1 - p0).ok()?;
            Some(Surface::Plane { frame })
        }
        (
            Kind::Arc {
                centre: ca,
                normal: na,
                radius: ra,
                ..
            },
            Kind::Arc {
                centre: cb,
                normal: nb,
                radius: rb,
                ..
            },
        ) => {
            let between = cb - ca;
            let h = between.norm();
            if h <= tol.linear {
                return None;
            }
            let axis = UnitVec3::try_new(between, 0.0)?;
            if na.cross(&axis).norm() > tol.angular || nb.cross(&axis).norm() > tol.angular {
                return None;
            }
            // The two arcs must meet at the same angles about the axis at
            // every sample of their common parameter, or a ruling is not
            // a generator.
            let (pa, pb) = (samples(a), samples(b));
            let x0 = Frame::new(*ca, axis.into_inner(), pa[0] - ca).ok()?;
            let angle_of = |p: &Point3, c: &Point3| {
                let q = p - c;
                real::atan2(q.dot(&x0.y()), q.dot(&x0.x()))
            };
            for (p, q) in pa.iter().zip(&pb) {
                let mut d = angle_of(q, cb) - angle_of(p, ca);
                d -= core::f64::consts::TAU * (d / core::f64::consts::TAU).round();
                if d.abs() > tol.angular {
                    return None;
                }
            }
            if (ra - rb).abs() <= tol.linear {
                return Some(Surface::Cylinder {
                    frame: x0,
                    radius: *ra,
                });
            }
            // A cone grows along its axis: the frame stands on the
            // smaller circle and points at the larger.
            let (origin, z, radius, other, from) = if rb > ra {
                (*ca, axis.into_inner(), *ra, *rb, &pa[0])
            } else {
                (*cb, -axis.into_inner(), *rb, *ra, &pb[0])
            };
            let frame = Frame::new(origin, z, from - origin).ok()?;
            let half_angle = real::atan2(other - radius, h);
            Some(Surface::Cone {
                frame,
                radius,
                half_angle,
            })
        }
        _ => None,
    }
}
