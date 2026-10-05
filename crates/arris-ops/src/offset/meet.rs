//! Where the offset's surfaces meet: a vertex as the point nearest its old
//! one on every surface around it, found by Gauss–Newton on signed
//! distances, and the plane that stands for a seam, so a seam edge is
//! found like any other, as two surfaces' section.

use arris_geom::{Curve, GeomKind, Surface};
use arris_math::{Frame, Point3, Tolerance, Vec3};
use arris_topo::FaceId;

use crate::error::OpError;
use crate::rebuild::forward;

/// The most Gauss–Newton steps a vertex takes: its surfaces start within
/// the offset distance of their meeting, so a handful converge, and a
/// vertex that has not by then is one the offset splits.
const MAX_STEPS: usize = 40;

/// A step shorter than this fraction of the linear tolerance has
/// converged: the next would move the point by less than the model can
/// tell apart.
const STEP_FRACTION: f64 = 1e-3;

/// A surface a vertex lies on once the offset is made, and the face it
/// belongs to, for the refusal that names it.
#[derive(Debug, Clone)]
pub(super) struct Constraint {
    pub(super) surface: Surface,
    pub(super) face: FaceId,
}

/// The signed distance of `p` from `surface` and its gradient there (the
/// unit normal at the nearest point), or `None` where the surface has no
/// nearest point or no normal at it — a cone's apex, a pole.
fn distance_at(surface: &Surface, p: Point3) -> Option<(f64, Vec3)> {
    // A sphere has a distance and a normal everywhere but its centre,
    // poles included, where its (u, v) normal is undefined.
    if let Surface::Sphere { frame, radius } = surface {
        let from = p - frame.origin();
        let r = from.norm();
        return (r > 0.0).then(|| (r - radius, from / r));
    }
    let nearest = surface.project(p).ok()?;
    let n = surface.normal(nearest.uv.x, nearest.uv.y)?.into_inner();
    Some(((p - nearest.point).dot(&n), n))
}

/// The point nearest `old` on every constraint, within `tolerance` of
/// each: Gauss–Newton on the signed distances, each step the shortest
/// that clears the first three independent ones (their normals more than
/// `tol.angular` from the span of those before), so a vertex on two
/// surfaces slides square to their section's tangent, as it does along
/// a seam. `Ok(None)` where the surfaces do not meet there within
/// `tolerance` of every one; [`OpError::Unsupported`] naming the face
/// where a surface has no distance at the iterate.
pub(super) fn meet_near(
    old: Point3,
    constraints: &[Constraint],
    tol: Tolerance,
    tolerance: f64,
) -> Result<Option<Point3>, OpError> {
    let mut p = old;
    let rows = |p: Point3| -> Result<Vec<(f64, Vec3)>, OpError> {
        constraints
            .iter()
            .map(|c| {
                distance_at(&c.surface, p).ok_or_else(|| {
                    let kind = GeomKind::Surface(c.surface.kind());
                    OpError::Unsupported {
                        a: (kind, forward(c.face)),
                        b: (kind, forward(c.face)),
                    }
                })
            })
            .collect()
    };
    for _ in 0..MAX_STEPS {
        let step = step_of(&rows(p)?, tol);
        p += step;
        if step.norm() <= tol.linear * STEP_FRACTION {
            break;
        }
    }
    let within = rows(p)?.iter().all(|(f, _)| f.abs() <= tolerance);
    Ok(within.then_some(p))
}

/// The shortest move that zeroes the first three independent of `rows`
/// (distance, unit normal), by the dual basis of their normals.
fn step_of(rows: &[(f64, Vec3)], tol: Tolerance) -> Vec3 {
    let mut basis: Vec<Vec3> = Vec::new();
    let mut kept: Vec<(f64, Vec3)> = Vec::new();
    for &(f, n) in rows {
        let mut r = n;
        for b in &basis {
            r -= b * b.dot(&r);
        }
        let sine = r.norm();
        if sine > tol.angular {
            basis.push(r / sine);
            kept.push((-f, n));
        }
        if kept.len() == 3 {
            break;
        }
    }
    match kept.as_slice() {
        [(ga, a)] => a * *ga,
        [(ga, a), (gb, b)] => {
            let k = a.dot(b);
            let det = 1.0 - k * k;
            a * ((ga - k * gb) / det) + b * ((gb - k * ga) / det)
        }
        [(ga, a), (gb, b), (gc, c)] => {
            let det = a.dot(&b.cross(c));
            (b.cross(c) * *ga + c.cross(a) * *gb + a.cross(b) * *gc) / det
        }
        _ => Vec3::zeros(),
    }
}

/// The plane a seam edge of `surface` lies in, so that the edge is the
/// section of the surface and that plane: through the surface's axis for
/// a line or a meridian circle, square to it for a circle about it (a
/// torus's equator). `at` is a point of the edge; `None` for a curve
/// that is neither, a surface with no axis, or an edge on the axis.
pub(super) fn seam_plane(
    surface: &Surface,
    curve: &Curve,
    at: Point3,
    tol: Tolerance,
) -> Option<Surface> {
    let axis = surface.frame()?;
    let (z, o) = (axis.z().into_inner(), axis.origin());
    let plane = |origin: Point3, normal: Vec3| -> Option<Surface> {
        Some(Surface::Plane {
            frame: Frame::from_z(origin, normal).ok()?,
        })
    };
    match curve {
        Curve::Line { .. } => {
            let across = z.cross(&(at - o));
            (across.norm() > tol.linear).then(|| plane(o, across))?
        }
        Curve::Circle { frame, .. } => {
            let cz = frame.z().into_inner();
            if cz.cross(&z).norm() <= tol.angular {
                plane(frame.origin(), z)
            } else {
                plane(o, cz)
            }
        }
        Curve::Ellipse { .. } | Curve::Nurbs(_) => None,
    }
}
