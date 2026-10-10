//! Extrema against a surface: the feet of a point on a surface, and the
//! stationary pairs of the distance between a curve and a surface, both on
//! the *untrimmed* carriers (`docs/DATA-MODEL.md` §Surfaces, §Extrema).
//!
//! **The reduction.** The distance between `C(t)` and `S(u, v)` is
//! stationary where `C − S` is along the surface's normal at `S` and
//! across the curve's tangent at `C`. Each analytic surface is brought down
//! to a curve or a point so that the unknowns left are the curve's alone,
//! which the certified solver of the parent module takes:
//!
//! - **Offsets.** A sphere is the points at `R` from its centre, a
//!   cylinder from its axis, a torus from its centre circle. `C − S` along
//!   the normal puts `S` on the line from `C` to its foot `B` on the base,
//!   at `R` either side of `B`, and the distance is then stationary in `t`
//!   exactly where the distance from `C` to the base is. So the pairs are
//!   those of `C` against the base ([`Curve::feet`] of the centre,
//!   [`extrema_curve_curve`] against the axis or the circle), each lifted to
//!   the two surface points `B ± R·n̂`, at the distances `|d − R|` and
//!   `d + R`.
//! - **A plane** is the height above it. The pairs are the critical points
//!   of the height along the curve (roots of `N · n`, [`Piece`]'s
//!   `against_height`), each paired with its projection. A line is
//!   parallel to the plane, a continuum, or has no critical point.
//! - **A cone** is two lines in each meridian plane through its axis. In
//!   the plane of `C(t)` the foot is the projection onto either line, and
//!   the distance to line `σ` (±1) is `s = σρ cos α − (z − z_apex) sin α`,
//!   with `ρ` the distance from the axis. It is stationary where
//!   `σρ′ cos α = z′ sin α`; the square `(ρρ′)² cos² α = ρ² z′² sin² α`,
//!   which is `(x x′ + y y′)² cos² α = (x² + y²) z′² sin² α`, has no square
//!   root, and is a polynomial in the curve's parameter: degree `6n − 2` on a
//!   Bézier piece of degree `n` (rational: the weights cancel), at most
//!   quadratic on a line. Its sign changes and its derivative's — a pair
//!   where both lines are stationary at once is a double zero — are the
//!   candidates, each polished on the unsquared equation of whichever line
//!   it satisfies.
//! - **An elliptic cylinder** is its section ellipse swept along the axis:
//!   the foot has the height of the curve's point, so the pairs are those of
//!   the curve flattened onto the section's plane against the ellipse.
//! - **Crossings.** Where the curve meets the surface the distance is zero
//!   and stationary: [`intersect_curve_surface`] gives them, and a curve on
//!   the surface is a continuum at distance 0.
//!
//! A NURBS surface has no reduction here: a curve against one is
//! [`GeomError::Unsupported`], and a point's feet on one are its projection
//! (the nearest point, not every critical point).
//!
//! **What is not reported.** A curve through the axis of a surface of
//! revolution, or through a sphere's centre or a torus's centre circle,
//! has stationary pairs on the whole circle of feet that are saddles or
//! maxima of the distance (the distance to the surface has a kink there,
//! concave), never a minimum; they are left out unless the curve runs along
//! the axis or the circle within the tolerance, which makes the whole family
//! a continuum. Every local minimum, and every other stationary pair, is
//! reported.

use core::f64::consts::PI;

use arris_math::{Frame, Meter, Point2, Point3, Tolerance, Vec3, wrap_angle};

use super::{
    Binomials, Found, Piece, T1, Target, continuum_within, dedup, extrema_curve_curve, newton1,
    pieces, sampled_roots, sign_change_candidates, with_ends, wrap,
};
use super::{Extrema, Feet};
use crate::{
    Curve, CurveSurfaceIntersection, GeomError, GeomKind, NurbsCurve, Surface, SurfaceKind,
    SurfaceProjection, intersect_curve_surface,
};

/// How near the axis of a surface of revolution a point of a curve is
/// taken to be on it, as a multiple of the largest coordinate involved:
/// the angle about the axis is rounding there. A ratio, not a tolerance.
const AXIS_NOISE: f64 = 64.0 * f64::EPSILON;

/// One stationary pair of the distance between a curve and a surface.
///
/// `curve_point == curve.point(t)` and `surface_point == surface.point(u,
/// v)` to rounding, `distance == |curve_point − surface_point|`, and the
/// offset between them is across the curve's tangent and along the
/// surface's normal within the tolerance. A periodic parameter is in
/// `[0, 2π)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveSurfacePair {
    /// The curve's parameter.
    pub t: f64,
    /// The surface's `(u, v)`.
    pub uv: Point2,
    /// `curve.point(t)`.
    pub curve_point: Point3,
    /// `surface.point(u, v)`.
    pub surface_point: Point3,
    /// `|curve_point − surface_point|`.
    pub distance: f64,
}

/// Every stationary pair of the distance between a curve and a surface.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CurveSurfaceExtrema {
    /// The isolated stationary pairs — minima, maxima and saddles alike,
    /// and the points where the curve meets the surface — ascending by `t`,
    /// then `u`, then `v`.
    pub isolated: Vec<CurveSurfacePair>,
    /// One pair on each family of stationary pairs along which the distance
    /// is constant within `tol.linear`, ascending by distance. Two families
    /// at one distance are one entry.
    pub continua: Vec<CurveSurfacePair>,
}

impl CurveSurfaceExtrema {
    /// The pair of least distance, isolated or on a continuum; the lower
    /// `t`, then `u`, then `v`, of two at one distance. `None` when there
    /// is no stationary pair at all.
    pub fn nearest(&self) -> Option<CurveSurfacePair> {
        self.isolated
            .iter()
            .chain(&self.continua)
            .copied()
            .min_by(|x, y| x.distance.total_cmp(&y.distance).then(order(x, y)))
    }

    fn finish(mut self, tol: Tolerance) -> Self {
        self.isolated.sort_by(order);
        self.isolated = dedup(self.isolated, |x, y| {
            (x.curve_point - y.curve_point).norm() <= tol.linear
                && (x.surface_point - y.surface_point).norm() <= tol.linear
        });
        self.continua
            .sort_by(|x, y| x.distance.total_cmp(&y.distance).then(order(x, y)));
        self.continua
            .dedup_by(|x, y| (x.distance - y.distance).abs() <= tol.linear);
        self
    }
}

fn order(x: &CurveSurfacePair, y: &CurveSurfacePair) -> core::cmp::Ordering {
    x.t.total_cmp(&y.t)
        .then(x.uv.x.total_cmp(&y.uv.x))
        .then(x.uv.y.total_cmp(&y.uv.y))
}

// --- the feet of a point ------------------------------------------------------

impl Surface {
    /// Every foot of `p` on the surface: each `(u, v)` at which the
    /// distance from `p` is stationary, nearest and farthest alike, on the
    /// whole parametric surface (both nappes of a cone, the full plane).
    /// Never `Ambiguous`: a point on the axis of a cylinder, a cone or a
    /// torus, at a sphere's centre, or on a torus's centre circle has a
    /// whole circle of feet, which is a continuum with one foot on it.
    ///
    /// A plane has one foot, a sphere two (the near and the far, on the
    /// line from the centre), a cylinder two (the two rulings in the
    /// meridian plane of `p`), an elliptic cylinder those of its section
    /// ellipse (two to four rulings), a cone two (the feet on its two lines
    /// in the meridian plane) and a torus four (the near and the far foot on
    /// the centre circle, each lifted to the tube's two sides). A NURBS
    /// surface has the one foot [`Surface::project`] gives: its nearest
    /// point, never a guarantee against another local minimum, and
    /// `Ambiguous` where two points are as near as each other.
    ///
    /// Errors: [`GeomError::InvalidTolerance`] and, for a NURBS surface,
    /// those of [`Surface::project`]. The meter is ticked once per foot
    /// search through an ellipse.
    ///
    /// ```
    /// use arris_geom::Surface;
    /// use arris_math::{Frame, Meter, Point3, Precision};
    ///
    /// let cyl = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
    /// let tol = Precision::DEFAULT.tolerance();
    /// let feet = cyl.feet(Point3::new(5.0, 0.0, 1.0), tol, &mut Meter::default()).unwrap();
    /// assert_eq!(feet.isolated.len(), 2);
    /// assert!((feet.isolated[0].distance - 3.0).abs() < 1e-15);
    /// assert!((feet.isolated[1].distance - 7.0).abs() < 1e-15);
    ///
    /// // On the axis every ruling is as far.
    /// let axis = cyl.feet(Point3::new(0.0, 0.0, 1.0), tol, &mut Meter::default()).unwrap();
    /// assert!(axis.isolated.is_empty());
    /// assert!((axis.continua[0].distance - 2.0).abs() < 1e-15);
    /// ```
    pub fn feet(
        &self,
        p: Point3,
        tol: Tolerance,
        meter: &mut Meter,
    ) -> Result<Feet<SurfaceProjection>, GeomError> {
        if !tol.is_consistent() {
            return Err(GeomError::InvalidTolerance(tol));
        }
        let foot = |u: f64, v: f64| {
            let point = self.point(u, v);
            SurfaceProjection {
                uv: Point2::new(u, v),
                point,
                distance: (p - point).norm(),
            }
        };
        let sorted = |mut isolated: Vec<SurfaceProjection>,
                      mut continua: Vec<SurfaceProjection>| {
            let by_uv = |x: &SurfaceProjection, y: &SurfaceProjection| {
                x.uv.x.total_cmp(&y.uv.x).then(x.uv.y.total_cmp(&y.uv.y))
            };
            isolated.sort_by(by_uv);
            continua.sort_by(|x, y| x.distance.total_cmp(&y.distance).then(by_uv(x, y)));
            continua.dedup_by(|x, y| (x.distance - y.distance).abs() <= tol.linear);
            Feet { isolated, continua }
        };
        match self {
            Surface::Plane { frame } => {
                let q = frame.to_local(p);
                Ok(Feet {
                    isolated: vec![SurfaceProjection {
                        uv: Point2::new(q.x, q.y),
                        point: self.point(q.x, q.y),
                        distance: q.z.abs(),
                    }],
                    continua: vec![],
                })
            }
            Surface::Sphere { frame, .. } => {
                let q = frame.to_local(p);
                let r = q.coords.norm();
                // The distance from `p` to the sphere's points varies by
                // `2r`; within the tolerance of each other they are one
                // foot.
                if 2.0 * r <= tol.linear {
                    return Ok(sorted(vec![], vec![foot(0.0, 0.0)]));
                }
                let (u, v) = sphere_uv(q.coords / r);
                let (u2, v2) = sphere_uv(-q.coords / r);
                Ok(sorted(vec![foot(u, v), foot(u2, v2)], vec![]))
            }
            Surface::Cylinder { frame, .. } => {
                let q = frame.to_local(p);
                let rho = q.x.hypot(q.y);
                if 2.0 * rho <= tol.linear {
                    return Ok(sorted(vec![], vec![foot(0.0, q.z)]));
                }
                let u = wrap_angle(q.y.atan2(q.x));
                Ok(sorted(
                    vec![foot(u, q.z), foot(wrap_angle(u + PI), q.z)],
                    vec![],
                ))
            }
            &Surface::EllipticCylinder {
                frame,
                major_radius,
                minor_radius,
            } => {
                let q = frame.to_local(p);
                let section = Curve::Ellipse {
                    frame,
                    major_radius,
                    minor_radius,
                };
                let in_plane = frame.to_world(Point3::new(q.x, q.y, 0.0));
                let feet = section.feet(in_plane, tol, meter)?;
                Ok(sorted(
                    feet.isolated.iter().map(|f| foot(f.t, q.z)).collect(),
                    feet.continua.iter().map(|f| foot(f.t, q.z)).collect(),
                ))
            }
            &Surface::Cone {
                frame,
                radius,
                half_angle,
            } => {
                let q = frame.to_local(p);
                let rho = q.x.hypot(q.y);
                if 2.0 * rho <= tol.linear {
                    let continua = [1.0, -1.0]
                        .map(|side| foot(0.0, cone_foot_v(radius, half_angle, rho, q.z, side)))
                        .to_vec();
                    return Ok(sorted(vec![], continua));
                }
                let phi = q.y.atan2(q.x);
                let isolated = [1.0, -1.0]
                    .map(|side| {
                        let u = wrap_angle(if side > 0.0 { phi } else { phi + PI });
                        foot(u, cone_foot_v(radius, half_angle, rho, q.z, side))
                    })
                    .to_vec();
                Ok(sorted(isolated, vec![]))
            }
            &Surface::Torus {
                frame,
                major_radius,
                ..
            } => {
                let q = frame.to_local(p);
                let rho = q.x.hypot(q.y);
                let phi = q.y.atan2(q.x);
                let on_axis = 2.0 * rho <= tol.linear;
                let angles: Vec<f64> = if on_axis {
                    vec![0.0]
                } else {
                    vec![phi, phi + PI]
                };
                let mut isolated = Vec::new();
                let mut continua = Vec::new();
                for psi in angles {
                    // `p` and the circle's point `B` at angle `psi`, in the
                    // meridian plane `(e_σ, Z)` of `B`.
                    let (sp, cp) = psi.sin_cos();
                    let across = rho_along(q, cp, sp) - major_radius;
                    let d = across.hypot(q.z);
                    // On the centre circle every point of the tube's
                    // meridian circle is as far, `r`: one continuum.
                    let (cs, sn) = if 2.0 * d <= tol.linear {
                        (1.0, 0.0)
                    } else {
                        (across / d, q.z / d)
                    };
                    for sign in [1.0, -1.0] {
                        // `S = B + sign·r·n̂`, in the tube's angle.
                        let v = wrap_angle((sign * sn).atan2(sign * cs));
                        let f = foot(wrap_angle(psi), v);
                        if on_axis || 2.0 * d <= tol.linear {
                            continua.push(f);
                        } else {
                            isolated.push(f);
                        }
                    }
                }
                Ok(sorted(isolated, continua))
            }
            Surface::Nurbs(_) => Ok(Feet {
                isolated: vec![self.project(p)?],
                continua: vec![],
            }),
        }
    }
}

/// The component of the local point `q` along the horizontal direction at
/// angle `psi` (`cos`, `sin` given).
fn rho_along(q: Point3, cos: f64, sin: f64) -> f64 {
    q.x * cos + q.y * sin
}

/// `(u, v)` of a sphere's point in the direction `d`, a unit vector in the
/// sphere's frame: a pole takes `u = 0`.
fn sphere_uv(d: Vec3) -> (f64, f64) {
    let rho = d.x.hypot(d.y);
    let u = if rho <= AXIS_NOISE {
        0.0
    } else {
        wrap_angle(d.y.atan2(d.x))
    };
    (u, d.z.atan2(rho))
}

/// The `v` of the foot on the line `side` (±1) of a cone in the meridian
/// plane of a point at distance `rho` from the axis and height `z`.
fn cone_foot_v(radius: f64, half_angle: f64, rho: f64, z: f64, side: f64) -> f64 {
    let (sa, ca) = half_angle.sin_cos();
    let apex = -radius * ca / sa;
    let lambda = side * rho * sa + (z - apex) * ca;
    (apex + lambda * ca) / ca
}

// --- the extrema of a curve against a surface ---------------------------------

/// Every stationary pair of the distance between `curve` and `surface` on
/// the whole of each (module docs): the curve's parameter is `t`.
///
/// Guarantees: every local minimum of the squared distance, isolated or on
/// a continuum, is reported, and every other stationary pair except those
/// the module docs list; each pair is polished to rounding and no two lie
/// within `tol.linear` of each other on both operands; a curve lying on the
/// surface, or running along a family of stationary pairs, is one
/// [`CurveSurfaceExtrema::continua`] entry; the answer is deterministic.
/// A curve meeting the surface has the meeting for an isolated pair at
/// distance 0.
///
/// Errors: [`GeomError::InvalidTolerance`], [`GeomError::Degenerate`] for
/// a conic radius that is not finite and positive,
/// [`GeomError::Unsupported`] for a NURBS surface, and
/// [`GeomError::Interrupted`] by the meter.
///
/// ```
/// use arris_geom::{Curve, Surface, extrema_curve_surface};
/// use arris_math::{Frame, Meter, Point3, Precision, UnitVec3, Vec3};
///
/// let wall = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
/// let rod = Curve::Line {
///     origin: Point3::new(5.0, 0.0, 0.0),
///     direction: UnitVec3::new_normalize(Vec3::new(0.0, 0.0, 1.0)),
/// };
/// let tol = Precision::DEFAULT.tolerance();
/// let e = extrema_curve_surface(&rod, &wall, tol, &mut Meter::default()).unwrap();
/// // The rod runs along the wall: every point is 3 from it.
/// assert!((e.nearest().unwrap().distance - 3.0).abs() < 1e-12);
/// assert_eq!(e.continua.len(), 2);
/// ```
pub fn extrema_curve_surface(
    curve: &Curve,
    surface: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<CurveSurfaceExtrema, GeomError> {
    if !tol.is_consistent() {
        return Err(GeomError::InvalidTolerance(tol));
    }
    let mut out = CurveSurfaceExtrema::default();
    match surface {
        Surface::Plane { frame } => plane(curve, surface, frame, tol, meter, &mut out)?,
        &Surface::Sphere { frame, .. } => {
            sphere(curve, surface, &frame, tol, meter, &mut out)?;
        }
        &Surface::Cylinder { frame, radius } => {
            cylinder(curve, surface, &frame, radius, tol, meter, &mut out)?;
        }
        &Surface::Torus {
            frame,
            major_radius,
            minor_radius,
        } => torus(
            curve,
            surface,
            &frame,
            major_radius,
            minor_radius,
            tol,
            meter,
            &mut out,
        )?,
        &Surface::Cone {
            frame,
            radius,
            half_angle,
        } => cone(
            curve, surface, &frame, radius, half_angle, tol, meter, &mut out,
        )?,
        &Surface::EllipticCylinder {
            frame,
            major_radius,
            minor_radius,
        } => elliptic(
            curve,
            surface,
            &frame,
            major_radius,
            minor_radius,
            tol,
            meter,
            &mut out,
        )?,
        Surface::Nurbs(_) => {
            return Err(GeomError::Unsupported {
                a: GeomKind::Curve(curve.kind()),
                b: GeomKind::Surface(SurfaceKind::Nurbs),
            });
        }
    }
    meetings(curve, surface, tol, meter, &mut out)?;
    Ok(out.finish(tol))
}

fn pair(curve: &Curve, surface: &Surface, t: f64, u: f64, v: f64) -> CurveSurfacePair {
    let (curve_point, surface_point) = (curve.point(t), surface.point(u, v));
    CurveSurfacePair {
        t,
        uv: Point2::new(u, v),
        curve_point,
        surface_point,
        distance: (curve_point - surface_point).norm(),
    }
}

/// The curve's first parameter that is a number: a line's `0`, a conic's
/// `0`, a NURBS curve's start.
fn start(curve: &Curve) -> f64 {
    let lo = curve.domain().lo();
    if lo.is_finite() { lo } else { 0.0 }
}

/// The nearest foot of `p` on `surface`, for the `(u, v)` of a point known
/// to lie on it.
fn nearest_uv(
    surface: &Surface,
    p: Point3,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Point2, GeomError> {
    let feet = surface.feet(p, tol, meter)?;
    Ok(feet
        .isolated
        .iter()
        .chain(&feet.continua)
        .min_by(|x, y| x.distance.total_cmp(&y.distance))
        .map_or(Point2::new(0.0, 0.0), |f| f.uv))
}

/// The points where the curve meets the surface, at distance 0, and a
/// curve on it as a continuum.
fn meetings(
    curve: &Curve,
    surface: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
    out: &mut CurveSurfaceExtrema,
) -> Result<(), GeomError> {
    match intersect_curve_surface(curve, surface, tol, meter)? {
        CurveSurfaceIntersection::Points(hits) => {
            for h in hits {
                out.isolated.push(pair(curve, surface, h.t, h.uv.x, h.uv.y));
            }
        }
        CurveSurfaceIntersection::Coincident => {
            let t = start(curve);
            let uv = nearest_uv(surface, curve.point(t), tol, meter)?;
            out.continua.push(pair(curve, surface, t, uv.x, uv.y));
        }
    }
    Ok(())
}

// --- the plane ----------------------------------------------------------------

fn plane(
    curve: &Curve,
    surface: &Surface,
    frame: &Frame,
    tol: Tolerance,
    meter: &mut Meter,
    out: &mut CurveSurfaceExtrema,
) -> Result<(), GeomError> {
    let n = frame.z().into_inner();
    let uv_of = |t: f64| {
        let q = frame.to_local(curve.point(t));
        (q.x, q.y)
    };
    if let &Curve::Line { direction, .. } = curve {
        // Parallel within the angle: every point of the line is as far.
        if direction.dot(&n).abs().asin() <= tol.angular {
            let (u, v) = uv_of(0.0);
            out.continua.push(pair(curve, surface, 0.0, u, v));
        }
        return Ok(());
    }
    let target = Target::Plane {
        origin: frame.origin(),
        normal: n,
    };
    let mut found = Found::default();
    for piece in pieces(curve, None)? {
        meter.tick().map_err(GeomError::Interrupted)?;
        piece.against(&target, tol, &mut found);
    }
    for (t, _) in found.isolated {
        let (u, v) = uv_of(t);
        out.isolated.push(pair(curve, surface, t, u, v));
    }
    for (t, _) in found.continua {
        let (u, v) = uv_of(t);
        out.continua.push(pair(curve, surface, t, u, v));
    }
    Ok(())
}

// --- offsets: sphere, cylinder, torus -----------------------------------------

fn sphere(
    curve: &Curve,
    surface: &Surface,
    frame: &Frame,
    tol: Tolerance,
    meter: &mut Meter,
    out: &mut CurveSurfaceExtrema,
) -> Result<(), GeomError> {
    let centre = frame.origin();
    let feet = curve.feet(centre, tol, meter)?;
    for (list, continuum) in [(&feet.isolated, false), (&feet.continua, true)] {
        for f in list {
            // A curve through the centre: see the module docs.
            if 2.0 * f.distance <= tol.linear {
                continue;
            }
            let n = (f.point - centre) / f.distance;
            for sign in [1.0, -1.0] {
                let local = frame.vec_to_local(sign * n);
                let (u, v) = sphere_uv(local);
                let p = pair(curve, surface, f.t, u, v);
                if continuum {
                    out.continua.push(p);
                } else {
                    out.isolated.push(p);
                }
            }
        }
    }
    Ok(())
}

fn cylinder(
    curve: &Curve,
    surface: &Surface,
    frame: &Frame,
    radius: f64,
    tol: Tolerance,
    meter: &mut Meter,
    out: &mut CurveSurfaceExtrema,
) -> Result<(), GeomError> {
    let z = frame.z().into_inner();
    let axis = Curve::Line {
        origin: frame.origin(),
        direction: frame.z(),
    };
    let e = extrema_curve_curve(curve, &axis, tol, meter)?;
    for (list, continuum) in [(&e.isolated, false), (&e.continua, true)] {
        for x in list {
            let rho = x.distance;
            if 2.0 * rho <= tol.linear {
                // On the axis the whole circle of feet is stationary only
                // where the curve runs along it (module docs).
                let tangent = curve.eval(x.ta).d1;
                let across = tangent - tangent.dot(&z) * z;
                let along = tangent.norm();
                if continuum || along == 0.0 || radius * across.norm() / along <= tol.linear {
                    out.continua.push(pair(curve, surface, x.ta, 0.0, x.tb));
                }
                continue;
            }
            let n = (x.a - x.b) / rho;
            for sign in [1.0, -1.0] {
                let local = frame.vec_to_local(sign * n);
                let u = wrap_angle(local.y.atan2(local.x));
                let p = pair(curve, surface, x.ta, u, x.tb);
                if continuum {
                    out.continua.push(p);
                } else {
                    out.isolated.push(p);
                }
            }
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn torus(
    curve: &Curve,
    surface: &Surface,
    frame: &Frame,
    major: f64,
    minor: f64,
    tol: Tolerance,
    meter: &mut Meter,
    out: &mut CurveSurfaceExtrema,
) -> Result<(), GeomError> {
    let centre_circle = Curve::Circle {
        frame: *frame,
        radius: major,
    };
    let e = extrema_curve_curve(curve, &centre_circle, tol, meter)?;
    for (list, continuum) in [(&e.isolated, false), (&e.continua, true)] {
        for x in list {
            let d = x.distance;
            let psi = x.tb;
            let (sp, cp) = psi.sin_cos();
            let e_sigma = frame.vec_to_world(Vec3::new(cp, sp, 0.0));
            let z = frame.z().into_inner();
            if 2.0 * d <= tol.linear {
                // On the centre circle every point of the tube's meridian
                // circle is as far; stationary in `t` for all of them only
                // where the curve runs along the circle (module docs).
                let tangent = curve.eval(x.ta).d1;
                let along_circle = frame.vec_to_world(Vec3::new(-sp, cp, 0.0));
                let meridian = tangent - tangent.dot(&along_circle) * along_circle;
                let speed = tangent.norm();
                if continuum || speed == 0.0 || minor * meridian.norm() / speed <= tol.linear {
                    out.continua.push(pair(curve, surface, x.ta, psi, 0.0));
                }
                continue;
            }
            let n = (x.a - x.b) / d;
            for sign in [1.0, -1.0] {
                let m = sign * n;
                let v = wrap_angle(m.dot(&z).atan2(m.dot(&e_sigma)));
                let p = pair(curve, surface, x.ta, psi, v);
                if continuum {
                    out.continua.push(p);
                } else {
                    out.isolated.push(p);
                }
            }
        }
    }
    Ok(())
}

// --- the cone -----------------------------------------------------------------

/// The kinematics of a curve point about a cone's axis, in the cone's frame.
struct Radial {
    /// `ρ`, `ρ′`, `ρ″`.
    rho: [f64; 3],
    /// `z − z_apex`, `z′`, `z″`.
    z: [f64; 3],
    /// `|C′|`.
    speed: f64,
}

impl Radial {
    /// The curve at `t`, `None` on the axis, where `ρ` has no derivative.
    fn at(curve: &Curve, frame: &Frame, apex: f64, t: f64) -> Option<Radial> {
        let e = curve.eval(t);
        let p = frame.to_local(e.point);
        let (d1, d2) = (frame.vec_to_local(e.d1), frame.vec_to_local(e.d2));
        let rho = p.x.hypot(p.y);
        if rho <= AXIS_NOISE * (1.0 + p.coords.norm()) {
            return None;
        }
        let m = p.x * d1.x + p.y * d1.y;
        let drho = m / rho;
        let ddrho =
            (d1.x * d1.x + d1.y * d1.y + p.x * d2.x + p.y * d2.y) / rho - m * m / rho.powi(3);
        Some(Radial {
            rho: [rho, drho, ddrho],
            z: [p.z - apex, d1.z, d2.z],
            speed: e.d1.norm(),
        })
    }

    /// `(s, s′, s″)` of line `side` (±1): the signed distance to it in the
    /// meridian plane, and its derivatives in `t`.
    fn side(&self, side: f64, half_angle: f64) -> [f64; 3] {
        let (sa, ca) = half_angle.sin_cos();
        [0, 1, 2].map(|k| side * self.rho[k] * ca - self.z[k] * sa)
    }
}

#[allow(clippy::too_many_arguments)]
fn cone(
    curve: &Curve,
    surface: &Surface,
    frame: &Frame,
    radius: f64,
    half_angle: f64,
    tol: Tolerance,
    meter: &mut Meter,
    out: &mut CurveSurfaceExtrema,
) -> Result<(), GeomError> {
    let (sa, ca) = half_angle.sin_cos();
    let apex = -radius * ca / sa;
    let state = |side: f64, t: f64| {
        Radial::at(curve, frame, apex, t).map(|r| {
            let s = r.side(side, half_angle);
            (s, r.speed)
        })
    };
    // The stationary condition of line `side` and its derivative.
    let residual =
        |side: f64| move |t: f64| state(side, t).map_or((0.0, 0.0), |(s, _)| (s[1], s[2]));
    // `(C − S)·T̂` of a pair on line `side`.
    let slope = |side: f64, t: f64| {
        state(side, t).map_or(0.0, |(s, speed)| {
            if speed == 0.0 {
                0.0
            } else {
                s[0] * s[1] / speed
            }
        })
    };
    let push = |side: f64, t: f64, continuum: bool, out: &mut CurveSurfaceExtrema| {
        let q = frame.to_local(curve.point(t));
        let rho = q.x.hypot(q.y);
        let v = cone_foot_v(radius, half_angle, rho, q.z, side);
        let u = if rho <= AXIS_NOISE * (1.0 + q.coords.norm()) {
            0.0
        } else {
            wrap_angle(q.y.atan2(q.x) + if side > 0.0 { 0.0 } else { PI })
        };
        let p = pair(curve, surface, t, u, v);
        if continuum {
            out.continua.push(p);
        } else {
            out.isolated.push(p);
        }
    };
    let range = [curve.domain().lo(), curve.domain().hi()];

    if let &Curve::Line { origin, direction } = curve {
        // The squared condition is a quadratic in the line's parameter.
        let o = frame.to_local(origin);
        let d = frame.vec_to_local(direction.into_inner());
        let a1 = d.x * d.x + d.y * d.y;
        let a0 = o.x * d.x + o.y * d.y;
        let (c2, s2) = (ca * ca, sa * sa * d.z * d.z);
        let coef = [
            c2 * a0 * a0 - s2 * (o.x * o.x + o.y * o.y),
            2.0 * c2 * a0 * a1 - 2.0 * s2 * a0,
            c2 * a1 * a1 - s2 * a1,
        ];
        let scale = [
            c2 * a0 * a0 + s2 * (o.x * o.x + o.y * o.y),
            2.0 * (c2 * (a0 * a1).abs() + s2 * a0.abs()),
            c2 * a1 * a1 + s2 * a1,
        ];
        let floor = |k: usize| crate::implicit::BERNSTEIN_ROUNDING * scale[k];
        if (0..3).all(|k| coef[k].abs() <= floor(k)) {
            // Below rounding: a ruling, or a line the squared condition
            // cannot tell apart from one. A side along which the distance
            // is constant and stationary at every sampled point is a
            // continuum; the other side has no stationary point (its
            // distance is linear along the line, but for the apex).
            let reach = 1.0 + o.coords.norm();
            for side in [1.0, -1.0] {
                let ok = [-reach, -1.0, 0.0, 1.0, reach].iter().all(|&t| {
                    state(side, t).is_some_and(|(s, speed)| {
                        speed == 0.0 || (s[0] * s[1] / speed).abs() <= tol.linear
                    })
                });
                let values: Vec<f64> = [-reach, 0.0, reach]
                    .iter()
                    .filter_map(|&t| state(side, t).map(|(s, _)| s[0].abs()))
                    .collect();
                if ok
                    && values.len() == 3
                    && values.iter().cloned().fold(0.0f64, f64::max)
                        - values.iter().cloned().fold(f64::INFINITY, f64::min)
                        <= tol.linear
                {
                    push(side, 0.0, true, out);
                }
            }
            return Ok(());
        }
        let Ok(roots) = arris_math::roots::quadratic(coef[2], coef[1], coef[0]) else {
            return Ok(());
        };
        for root in roots.iter() {
            for side in [1.0, -1.0] {
                let t = newton1(curve, root.value, range, residual(side));
                if slope(side, t).abs() <= tol.linear && state(side, t).is_some() {
                    push(side, t, false, out);
                }
            }
        }
        return Ok(());
    }

    for piece in pieces(curve, None)? {
        meter.tick().map_err(GeomError::Interrupted)?;
        let b = Binomials::new(6 * piece.degree + 2);
        let p = cone_polynomial(&piece, frame, half_angle, &b);
        let floor = p.floor();
        let starts = if p.v.iter().all(|c| c.abs() <= floor) {
            // Below rounding: a family of stationary points on one line,
            // as the tolerance reads it, or the exact residual's roots.
            let mut starts = Vec::new();
            for side in [1.0, -1.0] {
                let measure = |s: f64| {
                    let t = piece.param(s);
                    state(side, t).map_or((f64::INFINITY, f64::INFINITY), |(sv, _)| {
                        (sv[0].abs(), slope(side, t))
                    })
                };
                if continuum_within(measure, tol) {
                    push(side, piece.param(0.0), true, out);
                    continue;
                }
                starts.extend(sampled_roots(|s| {
                    state(side, piece.param(s)).map_or(0.0, |(sv, _)| sv[1])
                }));
            }
            starts
        } else {
            let mut starts = sign_change_candidates(&p.v, floor);
            let dp = p.derivative();
            starts.extend(sign_change_candidates(&dp.v, dp.floor()));
            with_ends(starts)
        };
        for s in starts {
            let t0 = piece.param(s);
            for side in [1.0, -1.0] {
                let t = wrap(curve, newton1(curve, t0, piece.range, residual(side)));
                if state(side, t).is_some() && slope(side, t).abs() <= tol.linear {
                    push(side, t, false, out);
                }
            }
        }
    }
    Ok(())
}

/// `cos² α·(X N_x + Y N_y)² − sin² α·(X² + Y²)·N_z²` on a piece, in the
/// cone's frame: `(ρρ′)² cos² α − ρ² z′² sin² α` times `W⁶`.
fn cone_polynomial(piece: &Piece<'_>, frame: &Frame, half_angle: f64, b: &Binomials) -> T1 {
    let (sa, ca) = half_angle.sin_cos();
    let n = piece.tangent(b);
    let c0 = piece.centre - frame.origin();
    let axes = [
        frame.x().into_inner(),
        frame.y().into_inner(),
        frame.z().into_inner(),
    ];
    let coords: Vec<T1> = axes
        .iter()
        .map(|e| {
            T1::sum(&[
                (e[0], &piece.x[0]),
                (e[1], &piece.x[1]),
                (e[2], &piece.x[2]),
                (e.dot(&c0), &piece.w),
            ])
        })
        .collect();
    let tangents: Vec<T1> = axes
        .iter()
        .map(|e| T1::sum(&[(e[0], &n[0]), (e[1], &n[1]), (e[2], &n[2])]))
        .collect();
    let a = T1::sum(&[
        (1.0, &coords[0].mul(&tangents[0], b)),
        (1.0, &coords[1].mul(&tangents[1], b)),
    ]);
    let rho2 = T1::sum(&[
        (1.0, &coords[0].mul(&coords[0], b)),
        (1.0, &coords[1].mul(&coords[1], b)),
    ]);
    let lhs = a.mul(&a, b);
    let rhs = rho2.mul(&tangents[2].mul(&tangents[2], b), b);
    T1::sum(&[(ca * ca, &lhs), (-sa * sa, &rhs)])
}

// --- the elliptic cylinder ----------------------------------------------------

/// A curve flattened onto an elliptic cylinder's section plane.
enum Flat {
    /// A line along the axis: a point of the plane.
    Point(Point3),
    /// A curve in the plane, with the original it came from.
    Curve {
        flat: Curve,
        /// The original curve's parameter at the flat curve's `s`.
        back: Box<dyn Fn(f64) -> f64>,
    },
}

#[allow(clippy::too_many_arguments)]
fn elliptic(
    curve: &Curve,
    surface: &Surface,
    frame: &Frame,
    major: f64,
    minor: f64,
    tol: Tolerance,
    meter: &mut Meter,
    out: &mut CurveSurfaceExtrema,
) -> Result<(), GeomError> {
    let z = frame.z().into_inner();
    let section = Curve::Ellipse {
        frame: *frame,
        major_radius: major,
        minor_radius: minor,
    };
    // The planar image of a point.
    let flatten = |p: Point3| {
        let q = frame.to_local(p);
        frame.to_world(Point3::new(q.x, q.y, 0.0))
    };
    let height = |p: Point3| (p - frame.origin()).dot(&z);
    let flat = match curve {
        &Curve::Line { origin, direction } => {
            let d = direction.into_inner();
            let across = d - d.dot(&z) * z;
            if across.norm() <= tol.angular {
                Flat::Point(flatten(origin))
            } else {
                let scale = across.norm();
                Flat::Curve {
                    flat: Curve::Line {
                        origin: flatten(origin),
                        direction: arris_math::UnitVec3::new_normalize(across),
                    },
                    back: Box::new(move |s| s / scale),
                }
            }
        }
        Curve::Circle { .. } | Curve::Ellipse { .. } => {
            let whole = match curve {
                &Curve::Circle { frame: f, radius } => {
                    NurbsCurve::circle(&f, radius, arris_math::Interval::TURN)?
                }
                &Curve::Ellipse {
                    frame: f,
                    major_radius,
                    minor_radius,
                } => {
                    NurbsCurve::ellipse(&f, major_radius, minor_radius, arris_math::Interval::TURN)?
                }
                Curve::Line { .. } | Curve::Nurbs(_) => unreachable!("matched above"),
            };
            let image = NurbsCurve::new(
                whole.degree(),
                whole.knots().to_vec(),
                whole.control_points().iter().map(|&p| flatten(p)).collect(),
                whole.weights().to_vec(),
            )?;
            let original = curve.clone();
            let lifted = Curve::Nurbs(whole);
            Flat::Curve {
                flat: Curve::Nurbs(image),
                back: Box::new(move |s| {
                    // The conic's parameter is the angle of its point.
                    let p = lifted.point(s);
                    match &original {
                        &Curve::Circle { frame: f, .. } => {
                            let q = f.to_local(p);
                            wrap_angle(q.y.atan2(q.x))
                        }
                        &Curve::Ellipse {
                            frame: f,
                            major_radius,
                            minor_radius,
                        } => {
                            let q = f.to_local(p);
                            wrap_angle((q.y / minor_radius).atan2(q.x / major_radius))
                        }
                        Curve::Line { .. } | Curve::Nurbs(_) => s,
                    }
                }),
            }
        }
        Curve::Nurbs(c) => {
            let image = NurbsCurve::new(
                c.degree(),
                c.knots().to_vec(),
                c.control_points().iter().map(|&p| flatten(p)).collect(),
                c.weights().to_vec(),
            )?;
            Flat::Curve {
                flat: Curve::Nurbs(image),
                back: Box::new(|s| s),
            }
        }
    };
    let lift = |t: f64, u: f64| {
        let v = height(curve.point(t));
        pair(curve, surface, t, u, v)
    };
    match flat {
        Flat::Point(p) => {
            // Every point of the line is as far from the section: each
            // foot of the point on the ellipse is a family.
            let feet = section.feet(p, tol, meter)?;
            for f in feet.isolated.iter().chain(&feet.continua) {
                out.continua.push(lift(0.0, f.t));
            }
        }
        Flat::Curve { flat, back } => {
            let e: Extrema = extrema_curve_curve(&flat, &section, tol, meter)?;
            for x in &e.isolated {
                out.isolated.push(lift(back(x.ta), x.tb));
            }
            for x in &e.continua {
                out.continua.push(lift(back(x.ta), x.tb));
            }
        }
    }
    Ok(())
}
