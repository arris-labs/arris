//! Point projection: the nearest point of a surface or a curve, with its
//! parameters.

use arris_math::real;
use core::f64::consts::FRAC_PI_2;

use arris_math::{Frame, Point2, Point3, is_negligible, wrap_angle as wrap_turn};

use crate::{AmbiguousLocus, Curve, CurveKind, GeomError, GeomKind, Surface};

/// The nearest point of a surface to a query point, with its parameters.
///
/// `point == surface.point(uv.x, uv.y)` to rounding, and `distance` is the
/// Euclidean distance from the query to it, computed from the surface's
/// closed form rather than by subtraction so it is exact for a point on
/// the surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceProjection {
    /// `(u, v)` of the nearest point: a periodic `u` in `[0, 2π)`, the
    /// sphere's `v` in `[−π/2, π/2]`, the torus's `v` in `[0, 2π)`.
    pub uv: Point2,
    /// The nearest point.
    pub point: Point3,
    /// How far the query is from it, `≥ 0`.
    pub distance: f64,
}

/// The nearest point of a curve to a query point, with its parameter.
///
/// `point == curve.point(t)` to rounding and `distance` is the Euclidean
/// distance from the query to it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveProjection {
    /// The parameter of the nearest point: a periodic `t` in `[0, 2π)`.
    pub t: f64,
    /// The nearest point.
    pub point: Point3,
    /// How far the query is from it, `≥ 0`.
    pub distance: f64,
}

/// The magnitude at which a local coordinate of `p` in `frame` is rounding
/// noise: `to_local` subtracts the origin and projects, so its error is a
/// few ulps of the larger of the two magnitudes.
fn local_noise_scale(frame: &Frame, p: Point3) -> f64 {
    p.coords.norm() + frame.origin().coords.norm()
}

impl Surface {
    /// The nearest point of the surface to `p` with its `(u, v)`, by the
    /// closed form of each variant: the point is exact to rounding, never
    /// the result of an iteration, and the parameters are the
    /// parametrisation's own (`docs/DATA-MODEL.md` §Surfaces).
    ///
    /// The whole parametric surface is the target — both nappes of a
    /// cone, the full plane — not a face's trimmed part of it.
    ///
    /// Errors: [`GeomError::Ambiguous`] where the nearest point or its
    /// parameter is not unique, decided to rounding and never by a silent
    /// choice — a point on the axis of a cylinder, a cone or a torus, in
    /// the plane through a cone's apex, at a sphere's centre, on a
    /// torus's centre circle, or on an elliptic cylinder's axis or the
    /// strip over the segment of its section's major axis inside the
    /// evolute (the section's own ambiguities, swept along the axis; the
    /// section projects through the quartic of [`Curve::project`]). A
    /// point on a sphere's axis off its centre
    /// projects to the pole, whose `u` is `0` by convention: the point is
    /// unique, only the degenerate parameter is not.
    /// A NURBS surface has no closed form: it is projected by
    /// [`crate::NurbsSurface::project`], the global nearest over its
    /// Bézier patches, ambiguous with [`AmbiguousLocus::MedialAxis`] where
    /// two distinct points of it are as near as each other.
    ///
    /// ```
    /// use arris_geom::Surface;
    /// use arris_math::{Frame, Point3};
    ///
    /// let cyl = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
    /// let proj = cyl.project(Point3::new(0.0, -5.0, 3.0)).unwrap();
    /// assert!((proj.uv.x - 3.0 * core::f64::consts::FRAC_PI_2).abs() < 1e-15);
    /// assert_eq!(proj.uv.y, 3.0);
    /// assert!((proj.distance - 3.0).abs() < 1e-15);
    /// assert!((proj.point - Point3::new(0.0, -2.0, 3.0)).norm() < 1e-15);
    /// assert!(cyl.project(Point3::new(0.0, 0.0, 7.0)).is_err());
    /// ```
    pub fn project(&self, p: Point3) -> Result<SurfaceProjection, GeomError> {
        let Some(frame) = self.frame() else {
            return match self {
                Surface::Nurbs(s) => s.project(p),
                // An analytic surface has a frame and did not get here;
                // the arm keeps the match exhaustive without a wildcard.
                Surface::Plane { .. }
                | Surface::Cylinder { .. }
                | Surface::EllipticCylinder { .. }
                | Surface::Cone { .. }
                | Surface::Sphere { .. }
                | Surface::Torus { .. } => Err(GeomError::Unsupported {
                    a: GeomKind::Point,
                    b: GeomKind::Surface(self.kind()),
                }),
            };
        };
        let q = frame.to_local(p);
        let noise = local_noise_scale(frame, p);
        let rho = real::hypot(q.x, q.y);
        let ambiguous = |locus| GeomError::Ambiguous {
            kind: GeomKind::Surface(self.kind()),
            locus,
            point: p,
        };
        let (u, v, distance) = match *self {
            // A NURBS has no frame and returned above; the arm is here so
            // the match stays exhaustive without a wildcard.
            Surface::Plane { .. } | Surface::Nurbs(_) => (q.x, q.y, q.z.abs()),
            Surface::Cylinder { radius, .. } => {
                if is_negligible(rho, noise) {
                    return Err(ambiguous(AmbiguousLocus::Axis));
                }
                (wrap_turn(real::atan2(q.y, q.x)), q.z, (rho - radius).abs())
            }
            Surface::EllipticCylinder {
                major_radius,
                minor_radius,
                ..
            } => {
                let (u, distance) = ellipse_nearest(major_radius, minor_radius, q.x, q.y, noise)
                    .map_err(|locus| {
                        ambiguous(match locus {
                            AmbiguousLocus::Centre => AmbiguousLocus::Axis,
                            other => other,
                        })
                    })?;
                (u, q.z, distance)
            }
            Surface::Cone {
                radius, half_angle, ..
            } => {
                if is_negligible(rho, noise) {
                    return Err(ambiguous(AmbiguousLocus::Axis));
                }
                let (sa, ca) = real::sin_cos(half_angle);
                // In the axial half-plane through `p` the surface is two
                // rulings crossing at the apex, one per nappe; `side` is
                // the point's height above the plane through the apex,
                // scaled by sin α, and its sign says which ruling is
                // nearer (they are equally near on that plane).
                let side = radius * ca + q.z * sa;
                if is_negligible(side, radius.abs() + noise) {
                    return Err(ambiguous(AmbiguousLocus::ApexPlane));
                }
                let (u, signed_rho) = if side < 0.0 {
                    (wrap_turn(real::atan2(-q.y, -q.x)), -rho)
                } else {
                    (wrap_turn(real::atan2(q.y, q.x)), rho)
                };
                let along = signed_rho - radius;
                (u, sa * along + ca * q.z, (ca * along - sa * q.z).abs())
            }
            Surface::Sphere { radius, .. } => {
                let r = q.coords.norm();
                if is_negligible(r, noise) {
                    return Err(ambiguous(AmbiguousLocus::Centre));
                }
                if is_negligible(rho, noise) {
                    let v = if q.z > 0.0 { FRAC_PI_2 } else { -FRAC_PI_2 };
                    (0.0, v, (r - radius).abs())
                } else {
                    (
                        wrap_turn(real::atan2(q.y, q.x)),
                        real::atan2(q.z, rho),
                        (r - radius).abs(),
                    )
                }
            }
            Surface::Torus {
                major_radius,
                minor_radius,
                ..
            } => {
                if is_negligible(rho, noise) {
                    return Err(ambiguous(AmbiguousLocus::Axis));
                }
                let across = rho - major_radius;
                let tube = real::hypot(across, q.z);
                if is_negligible(tube, noise + major_radius.abs()) {
                    return Err(ambiguous(AmbiguousLocus::CentreCircle));
                }
                (
                    wrap_turn(real::atan2(q.y, q.x)),
                    wrap_turn(real::atan2(q.z, across)),
                    (tube - minor_radius).abs(),
                )
            }
        };
        Ok(SurfaceProjection {
            uv: Point2::new(u, v),
            point: self.point(u, v),
            distance,
        })
    }
}

impl Curve {
    /// The nearest point of the curve to `p` with its parameter, by the
    /// closed form of each variant: the point is exact to rounding and the
    /// parameter is the parametrisation's own (`docs/DATA-MODEL.md`
    /// §Curves).
    ///
    /// Errors: [`GeomError::Ambiguous`] where the nearest point is not
    /// unique, decided to rounding — a point on a circle's axis, an
    /// ellipse's centre, or the open segment of an ellipse's major axis
    /// inside its evolute, where two mirror-image points are equally
    /// near. An ellipse projects through the quartic of
    /// [`arris_math::roots`]; the residual `(p − C(t)) · C′(t)` is zero
    /// to rounding. A NURBS curve projects by sampling and bracketed
    /// Newton ([`crate::NurbsCurve::project_parameter`]): the nearest local
    /// minimum from the best sample, never `Ambiguous`.
    ///
    /// ```
    /// use arris_geom::Curve;
    /// use arris_math::{Frame, Point3};
    /// use core::f64::consts::PI;
    ///
    /// let c = Curve::Circle { frame: Frame::world(), radius: 2.0 };
    /// let proj = c.project(Point3::new(-4.0, 0.0, 3.0)).unwrap();
    /// assert!((proj.t - PI).abs() < 1e-15);
    /// assert!((proj.distance - 13f64.sqrt()).abs() < 1e-15);
    /// assert!(c.project(Point3::new(0.0, 0.0, 3.0)).is_err());
    /// ```
    pub fn project(&self, p: Point3) -> Result<CurveProjection, GeomError> {
        match self {
            &Curve::Line { origin, direction } => {
                let t = (p - origin).dot(&direction);
                let point = origin + t * direction.into_inner();
                Ok(CurveProjection {
                    t,
                    point,
                    distance: (p - point).norm(),
                })
            }
            &Curve::Circle { frame, radius } => {
                let q = frame.to_local(p);
                let rho = real::hypot(q.x, q.y);
                if is_negligible(rho, local_noise_scale(&frame, p)) {
                    return Err(GeomError::Ambiguous {
                        kind: GeomKind::Curve(CurveKind::Circle),
                        locus: AmbiguousLocus::Axis,
                        point: p,
                    });
                }
                let t = wrap_turn(real::atan2(q.y, q.x));
                Ok(CurveProjection {
                    t,
                    point: self.point(t),
                    distance: real::hypot(rho - radius, q.z),
                })
            }
            &Curve::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => {
                let q = frame.to_local(p);
                let noise = local_noise_scale(&frame, p);
                let (t, in_plane) = ellipse_nearest(major_radius, minor_radius, q.x, q.y, noise)
                    .map_err(|locus| GeomError::Ambiguous {
                        kind: GeomKind::Curve(CurveKind::Ellipse),
                        locus,
                        point: p,
                    })?;
                Ok(CurveProjection {
                    t,
                    point: self.point(t),
                    distance: real::hypot(in_plane, q.z),
                })
            }
            Curve::Nurbs(c) => {
                let t = c.project_parameter(p);
                let point = c.eval(t).point;
                Ok(CurveProjection {
                    t,
                    point,
                    distance: (p - point).norm(),
                })
            }
        }
    }
}

/// The parameter of the point of the ellipse `(a cos t, b sin t)` nearest
/// to `(px, py)` in its plane, and the in-plane distance, or the locus
/// that makes the answer ambiguous.
///
/// The squared distance is stationary where
/// `g(t) = e sin t cos t + a·px sin t − b·py cos t` vanishes, with
/// `e = b² − a²`. With `s = tan(t/2)` that is the quartic
/// `b·py s⁴ + 2(a·px − e) s³ + 2(e + a·px) s − b·py = 0`, whose real
/// roots are the candidates, plus `t = π` (`s = ∞`) when the point is on
/// the major axis to rounding and the leading coefficient vanishes. Each
/// candidate is polished by Newton on `g`, the nearest wins, and two
/// nearest at the same distance to rounding (mirror images across the
/// major axis) are the ambiguity.
pub(crate) fn ellipse_nearest(
    a: f64,
    b: f64,
    px: f64,
    py: f64,
    noise: f64,
) -> Result<(f64, f64), AmbiguousLocus> {
    let e = b * b - a * a;
    let lead = b * py;
    let c3 = 2.0 * (a * px - e);
    let c1 = 2.0 * (e + a * px);
    let on_major_axis = is_negligible(lead, c3.abs().max(c1.abs()));
    let roots = if on_major_axis {
        arris_math::roots::cubic(c3, 0.0, c1, -lead)
    } else {
        arris_math::roots::quartic(lead, c3, 0.0, c1, -lead)
    };
    let roots = match roots {
        Ok(r) => r,
        // Every coefficient zero: `a == b` and the point is the centre.
        Err(arris_math::roots::RootError::Zero) => return Err(AmbiguousLocus::Centre),
        // Non-finite input reaches here only from a non-finite ellipse.
        Err(_) => return Err(AmbiguousLocus::Centre),
    };
    let g = |t: f64| {
        let (st, ct) = real::sin_cos(t);
        e * st * ct + a * px * st - b * py * ct
    };
    let dg = |t: f64| {
        let (st, ct) = real::sin_cos(t);
        e * real::cos(2.0 * t) + a * px * ct + b * py * st
    };
    let distance = |t: f64| {
        let (st, ct) = real::sin_cos(t);
        real::hypot(a * ct - px, b * st - py)
    };
    // Up to four roots and the seam candidate.
    let mut best: Option<(f64, f64)> = None;
    let mut runner_up: Option<(f64, f64)> = None;
    let candidates = roots
        .iter()
        .map(|r| 2.0 * real::atan(r.value))
        .chain(on_major_axis.then_some(core::f64::consts::PI));
    for t0 in candidates {
        let mut t = t0;
        for _ in 0..NEWTON_POLISH_STEPS {
            let (gt, dgt) = (g(t), dg(t));
            if gt == 0.0 || dgt == 0.0 {
                break;
            }
            let next = t - gt / dgt;
            if g(next).abs() < gt.abs() {
                t = next;
            } else {
                break;
            }
        }
        let d = distance(t);
        match best {
            Some((_, bd)) if d >= bd => {
                if runner_up.is_none_or(|(_, rd)| d < rd) {
                    runner_up = Some((t, d));
                }
            }
            _ => {
                runner_up = best;
                best = Some((t, d));
            }
        }
    }
    let Some((t, d)) = best else {
        // Unreachable: the quartic has at least two real roots because
        // the distance has a minimum and a maximum, and the cubic form
        // carries `t = π`.
        return Err(AmbiguousLocus::Centre);
    };
    if let Some((t2, d2)) = runner_up {
        let (s1, c1) = real::sin_cos(t);
        let (s2, c2) = real::sin_cos(t2);
        let apart = real::hypot(a * (c1 - c2), b * (s1 - s2));
        if is_negligible(d2 - d, noise + a) && !is_negligible(apart, a) {
            return Err(if is_negligible(px, noise) && is_negligible(py, noise) {
                AmbiguousLocus::Centre
            } else {
                AmbiguousLocus::MajorAxis
            });
        }
    }
    Ok((wrap_turn(t), d))
}

/// The distance from `(px, py)` to the ellipse `(a cos t, b sin t)` in
/// its plane: [`ellipse_nearest`]'s, exact — or, for a point whose
/// nearest point is not unique (the centre, the major axis inside the
/// evolute), the distance to the ellipse's point at the query's own
/// eccentric anomaly `atan2(py / b, px / a)`. That is an upper bound on
/// the true distance, so a point clear of the ellipse is never taken for
/// one on it; the bound overstates only inside the evolute, where the
/// true distance is at least `b² / a`.
pub(crate) fn ellipse_distance(a: f64, b: f64, px: f64, py: f64, noise: f64) -> f64 {
    match ellipse_nearest(a, b, px, py, noise) {
        Ok((_, d)) => d,
        Err(_) => {
            let t = real::atan2(py / b, px / a);
            let (st, ct) = real::sin_cos(t);
            real::hypot(a * ct - px, b * st - py)
        }
    }
}

/// Newton steps that polish a candidate parameter from the half-angle
/// quartic: two suffice from a root accurate to rounding, and each is
/// taken only while it reduces `|g|`.
const NEWTON_POLISH_STEPS: usize = 3;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_elliptic_cylinder_projects_through_its_section() {
        let s = Surface::EllipticCylinder {
            frame: Frame::world(),
            major_radius: 3.0,
            minor_radius: 2.0,
        };
        // Straight out along the minor axis: the nearest point is the
        // section's minor vertex at this height.
        let proj = s.project(Point3::new(0.0, 5.0, 4.0)).unwrap();
        assert!((proj.uv.x - FRAC_PI_2).abs() < 1e-12, "{proj:?}");
        assert_eq!(proj.uv.y, 4.0);
        assert!((proj.distance - 3.0).abs() < 1e-12);
        assert!((proj.point - Point3::new(0.0, 2.0, 4.0)).norm() < 1e-12);
        // The axis, and the strip over the major axis inside the evolute
        // (its cusps are at ±(a² − b²)/a = ±5/3), are ambiguous.
        assert!(matches!(
            s.project(Point3::new(0.0, 0.0, 7.0)),
            Err(GeomError::Ambiguous {
                locus: AmbiguousLocus::Axis,
                ..
            })
        ));
        assert!(matches!(
            s.project(Point3::new(1.0, 0.0, 7.0)),
            Err(GeomError::Ambiguous {
                locus: AmbiguousLocus::MajorAxis,
                ..
            })
        ));
        // Past the cusp the major vertex is the unique nearest point.
        let proj = s.project(Point3::new(2.0, 0.0, 1.0)).unwrap();
        assert!(proj.uv.x.abs() < 1e-12 && (proj.distance - 1.0).abs() < 1e-12);
    }

    #[test]
    fn a_plane_projects_by_local_coordinates() {
        let plane = Surface::Plane {
            frame: Frame::world(),
        };
        let proj = plane.project(Point3::new(3.0, -4.0, 5.0)).unwrap();
        assert_eq!(proj.uv, Point2::new(3.0, -4.0));
        assert_eq!(proj.distance, 5.0);
        assert_eq!(proj.point, Point3::new(3.0, -4.0, 0.0));
    }
}
