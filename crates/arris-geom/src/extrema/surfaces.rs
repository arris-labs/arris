//! Extrema between two surfaces: every stationary pair of the distance
//! between two whole surfaces, away from where they meet
//! (`docs/DATA-MODEL.md` §Surfaces, §Extrema).
//!
//! **The reduction.** The distance between `S ∈ A` and `Q ∈ B` is
//! stationary where `Q − S` is along both surfaces' normals. A sphere, a
//! cylinder and a torus are *offsets* of a base — a point, a line, a
//! circle — at `r`: `S = B + r·m` with `m ⟂` the base and `Q − B` along
//! `m`. So the pairs are those of the **base** against the other surface
//! ([`Surface::feet`] of the centre, [`extrema_curve_surface`] of the axis
//! or of the centre circle), each lifted to the two points `B ± r·n̂` of the
//! offset, at the distances `|d − r|` and `d + r`, where `d` is the base
//! pair's distance. A sphere is reduced before a cylinder and a cylinder
//! before a torus, so a pair is reduced the way that needs the most
//! special cases the fewest: a ball on a pipe's axis is a point against the
//! pipe, a continuum of feet, not a line through a sphere's centre.
//!
//! Where the base meets the other surface (`d = 0`) the lift has no
//! direction `n̂`: the offset surface and the other are *tangent* there if
//! the base is along the surface — a line in a plane, a ruling — and the
//! pairs are `Q ± r·n_B(Q)` at distance `r`; a base crossing the surface
//! gives none, the offset meets it.
//!
//! Without an offset, three pairs have a closed form: two planes, a plane
//! against an elliptic cylinder or a cone (the surface's rulings where its
//! normal is the plane's), two coaxial cones, two elliptic cylinders with
//! parallel axes (the section ellipses' pairs, each a family along the
//! axis). A NURBS surface against a plane is the critical points of the
//! height (`docs/DATA-MODEL.md` §Surfaces); against a sphere, the
//! critical feet of its centre. Every other pair — a cone against a cone
//! or an elliptic cylinder off a common axis, an elliptic cylinder against
//! one not parallel to it, a NURBS surface against anything else — has no
//! reduction below three unknowns and is [`GeomError::Unsupported`].
//!
//! **What is not reported.** The surfaces' meeting, along which the
//! distance is zero: [`crate::intersect_surfaces`]'s. A base that crosses
//! another surface's axis, or a sphere's centre, has stationary pairs on
//! the circle of feet that are saddles or maxima of the distance (module
//! `surface`); they are left out as there. Every other local minimum is
//! reported.

use core::f64::consts::PI;

use arris_math::{Frame, Meter, Point2, Point3, Tolerance, Vec3, wrap_angle};

use super::surface::sphere_uv;
use super::{Extrema, extrema_curve_curve, extrema_curve_surface};
use crate::{Curve, GeomError, GeomKind, Surface};

/// One stationary pair of the distance between two surfaces.
///
/// `point_a == a.point(uv_a)` and `point_b == b.point(uv_b)` to rounding,
/// `distance == |point_a − point_b|`, and the offset between them is along
/// both surfaces' normals within the tolerance. A periodic parameter is in
/// `[0, 2π)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfacePair {
    /// The first surface's `(u, v)`.
    pub uv_a: Point2,
    /// The second surface's `(u, v)`.
    pub uv_b: Point2,
    /// `a.point(uv_a)`.
    pub point_a: Point3,
    /// `b.point(uv_b)`.
    pub point_b: Point3,
    /// `|point_a − point_b|`.
    pub distance: f64,
}

/// Every stationary pair of the distance between two surfaces, away from
/// where they meet.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceExtrema {
    /// The isolated stationary pairs — minima, maxima and saddles alike —
    /// ascending by the first surface's `(u, v)`, then the second's.
    pub isolated: Vec<SurfacePair>,
    /// One pair on each family of stationary pairs along which the
    /// distance is constant within `tol.linear`, ascending by distance.
    /// Two families at one distance are one entry.
    pub continua: Vec<SurfacePair>,
}

impl SurfaceExtrema {
    /// The pair of least distance, isolated or on a continuum; the lower
    /// `uv_a`, then `uv_b`, of two at one distance. `None` when there is no
    /// stationary pair at all — as when the surfaces meet and nothing else
    /// is stationary.
    pub fn nearest(&self) -> Option<SurfacePair> {
        self.isolated
            .iter()
            .chain(&self.continua)
            .copied()
            .min_by(|x, y| x.distance.total_cmp(&y.distance).then(order(x, y)))
    }

    fn swapped(self) -> SurfaceExtrema {
        let swap = |p: SurfacePair| SurfacePair {
            uv_a: p.uv_b,
            uv_b: p.uv_a,
            point_a: p.point_b,
            point_b: p.point_a,
            distance: p.distance,
        };
        SurfaceExtrema {
            isolated: self.isolated.into_iter().map(swap).collect(),
            continua: self.continua.into_iter().map(swap).collect(),
        }
    }

    fn finish(mut self, tol: Tolerance) -> Self {
        self.isolated.sort_by(order);
        self.isolated = super::dedup(self.isolated, |x, y| {
            (x.point_a - y.point_a).norm() <= tol.linear
                && (x.point_b - y.point_b).norm() <= tol.linear
        });
        self.continua
            .sort_by(|x, y| x.distance.total_cmp(&y.distance).then(order(x, y)));
        self.continua
            .dedup_by(|x, y| (x.distance - y.distance).abs() <= tol.linear);
        self
    }
}

fn order(x: &SurfacePair, y: &SurfacePair) -> core::cmp::Ordering {
    x.uv_a
        .x
        .total_cmp(&y.uv_a.x)
        .then(x.uv_a.y.total_cmp(&y.uv_a.y))
        .then(x.uv_b.x.total_cmp(&y.uv_b.x))
        .then(x.uv_b.y.total_cmp(&y.uv_b.y))
}

/// The order a pair is solved in, whichever way it is asked: a sphere is
/// reduced before a cylinder, a cylinder before a torus, those before the
/// rest (module docs).
fn rank(s: &Surface) -> u8 {
    match s {
        Surface::Sphere { .. } => 0,
        Surface::Cylinder { .. } => 1,
        Surface::Torus { .. } => 2,
        Surface::Plane { .. } => 3,
        Surface::Cone { .. } => 4,
        Surface::EllipticCylinder { .. } => 5,
        Surface::Nurbs(_) => 6,
    }
}

fn unsupported(a: &Surface, b: &Surface) -> GeomError {
    GeomError::Unsupported {
        a: GeomKind::Surface(a.kind()),
        b: GeomKind::Surface(b.kind()),
    }
}

/// Every stationary pair of the distance between the surfaces `a` and `b`
/// on the whole of each (module docs): `a`'s parameters are `uv_a`.
///
/// Guarantees: every local minimum of the squared distance away from where
/// the surfaces meet, isolated or on a continuum, is reported, and every
/// other stationary pair except those the module docs list; each pair is
/// polished to rounding and no two lie within `tol.linear` of each other on
/// both surfaces; a family of pairs at one distance (parallel planes,
/// coaxial pipes, a ball on a pipe's axis, a cone along a parallel cone) is
/// one [`SurfaceExtrema::continua`] entry; the answer is deterministic and
/// swapping the operands swaps it. The surfaces' meeting is not reported:
/// two surfaces that meet have no pair at distance zero, and what is left
/// is stationary away from it.
///
/// Errors: [`GeomError::InvalidTolerance`], [`GeomError::Unsupported`] for
/// the pairs without a reduction (module docs), [`GeomError::Degenerate`]
/// for a conic radius that is not finite and positive, and
/// [`GeomError::Interrupted`] by the meter.
///
/// ```
/// use arris_geom::{Surface, extrema_surface_surface};
/// use arris_math::{Frame, Meter, Point3, Precision, Vec3};
///
/// // A ball beside a pipe: the pipe is 3 from the ball's centre, so the
/// // surfaces are 3 − 1 − 0.5 apart at the nearest.
/// let pipe = Surface::Cylinder { frame: Frame::world(), radius: 1.0 };
/// let ball = Surface::Sphere {
///     frame: Frame::from_z(Point3::new(3.0, 0.0, 2.0), Vec3::z()).unwrap(),
///     radius: 0.5,
/// };
/// let tol = Precision::DEFAULT.tolerance();
/// let e = extrema_surface_surface(&ball, &pipe, tol, &mut Meter::default()).unwrap();
/// assert!((e.nearest().unwrap().distance - 1.5).abs() < 1e-12);
/// ```
pub fn extrema_surface_surface(
    a: &Surface,
    b: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<SurfaceExtrema, GeomError> {
    if !tol.is_consistent() {
        return Err(GeomError::InvalidTolerance(tol));
    }
    if rank(b) < rank(a) {
        return Ok(extrema_surface_surface(b, a, tol, meter)?.swapped());
    }
    let mut out = SurfaceExtrema::default();
    match (a, b) {
        (&Surface::Sphere { frame, .. }, _) => sphere(a, b, &frame, tol, meter, &mut out)?,
        (&Surface::Cylinder { frame, .. }, _) => cylinder(a, b, &frame, tol, meter, &mut out)?,
        (
            &Surface::Torus {
                frame,
                major_radius,
                ..
            },
            _,
        ) => torus(a, b, &frame, major_radius, tol, meter, &mut out)?,
        (Surface::Plane { frame: fa }, Surface::Plane { frame: fb }) => {
            plane_plane(a, b, fa, fb, tol, &mut out);
        }
        (&Surface::Plane { frame }, Surface::EllipticCylinder { .. })
        | (&Surface::Plane { frame }, Surface::Cone { .. }) => {
            plane_quadric(a, b, &frame, tol, &mut out);
        }
        (&Surface::Plane { frame }, Surface::Nurbs(n)) => {
            plane_nurbs(a, b, &frame, n, tol, meter, &mut out)?;
        }
        (Surface::Cone { .. }, Surface::Cone { .. }) => cone_cone(a, b, tol, meter, &mut out)?,
        (Surface::EllipticCylinder { .. }, Surface::EllipticCylinder { .. }) => {
            elliptic_elliptic(a, b, tol, meter, &mut out)?;
        }
        (Surface::Cone { .. }, Surface::EllipticCylinder { .. } | Surface::Nurbs(_))
        | (Surface::EllipticCylinder { .. }, Surface::Nurbs(_))
        | (Surface::Nurbs(_), _) => return Err(unsupported(a, b)),
        // Ranked after their partner: the swap above sent them the other
        // way round, so these are never asked.
        (
            Surface::Plane { .. },
            Surface::Sphere { .. } | Surface::Cylinder { .. } | Surface::Torus { .. },
        )
        | (
            Surface::Cone { .. },
            Surface::Plane { .. }
            | Surface::Sphere { .. }
            | Surface::Cylinder { .. }
            | Surface::Torus { .. },
        )
        | (
            Surface::EllipticCylinder { .. },
            Surface::Plane { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Cylinder { .. }
            | Surface::Torus { .. },
        ) => return Err(unsupported(a, b)),
    }
    Ok(out.finish(tol))
}

fn push(out: &mut SurfaceExtrema, continuum: bool, pair: SurfacePair) {
    if continuum {
        out.continua.push(pair);
    } else {
        out.isolated.push(pair);
    }
}

fn pair_of(a: &Surface, b: &Surface, uv_a: Point2, uv_b: Point2) -> SurfacePair {
    let (point_a, point_b) = (a.point(uv_a.x, uv_a.y), b.point(uv_b.x, uv_b.y));
    SurfacePair {
        uv_a,
        uv_b,
        point_a,
        point_b,
        distance: (point_a - point_b).norm(),
    }
}

// --- offsets ------------------------------------------------------------------

fn sphere(
    a: &Surface,
    b: &Surface,
    frame: &Frame,
    tol: Tolerance,
    meter: &mut Meter,
    out: &mut SurfaceExtrema,
) -> Result<(), GeomError> {
    let centre = frame.origin();
    let feet = b.feet(centre, tol, meter)?;
    for (list, continuum) in [(&feet.isolated, false), (&feet.continua, true)] {
        for f in list {
            let q = f.point;
            let d = f.distance;
            // Inside the tolerance of the surface the ball is tangent
            // where it has a normal: `S = Q ± R·n`.
            let normal = if 2.0 * d > tol.linear {
                Some((q - centre) / d)
            } else {
                b.normal(f.uv.x, f.uv.y).map(|n| n.into_inner())
            };
            let Some(n) = normal else { continue };
            for sign in [1.0, -1.0] {
                let local = frame.vec_to_local(sign * n);
                let (u, v) = sphere_uv(local);
                let uv_a = Point2::new(u, v);
                push(out, continuum, pair_of(a, b, uv_a, f.uv));
            }
        }
    }
    Ok(())
}

fn cylinder(
    a: &Surface,
    b: &Surface,
    frame: &Frame,
    tol: Tolerance,
    meter: &mut Meter,
    out: &mut SurfaceExtrema,
) -> Result<(), GeomError> {
    let z = frame.z().into_inner();
    let axis = Curve::Line {
        origin: frame.origin(),
        direction: frame.z(),
    };
    let e = extrema_curve_surface(&axis, b, tol, meter).map_err(|e| match e {
        GeomError::Unsupported { .. } => unsupported(a, b),
        other => other,
    })?;
    for (list, continuum) in [(&e.isolated, false), (&e.continua, true)] {
        for p in list {
            let d = p.distance;
            let normal = if 2.0 * d > tol.linear {
                Some((p.surface_point - p.curve_point) / d)
            } else {
                // Tangent where the surface's normal is across the axis.
                b.normal(p.uv.x, p.uv.y)
                    .map(|n| n.into_inner())
                    .filter(|n| n.dot(&z).abs() <= tol.angular.sin())
            };
            let Some(n) = normal else { continue };
            for sign in [1.0, -1.0] {
                let local = frame.vec_to_local(sign * n);
                let u = wrap_angle(local.y.atan2(local.x));
                push(out, continuum, pair_of(a, b, Point2::new(u, p.t), p.uv));
            }
        }
    }
    Ok(())
}

fn torus(
    a: &Surface,
    b: &Surface,
    frame: &Frame,
    major: f64,
    tol: Tolerance,
    meter: &mut Meter,
    out: &mut SurfaceExtrema,
) -> Result<(), GeomError> {
    let z = frame.z().into_inner();
    let centre_circle = Curve::Circle {
        frame: *frame,
        radius: major,
    };
    let e = extrema_curve_surface(&centre_circle, b, tol, meter).map_err(|e| match e {
        GeomError::Unsupported { .. } => unsupported(a, b),
        other => other,
    })?;
    for (list, continuum) in [(&e.isolated, false), (&e.continua, true)] {
        for p in list {
            let d = p.distance;
            let psi = p.t;
            let (sp, cp) = psi.sin_cos();
            let e_sigma = frame.vec_to_world(Vec3::new(cp, sp, 0.0));
            let tangent = frame.vec_to_world(Vec3::new(-sp, cp, 0.0));
            let normal = if 2.0 * d > tol.linear {
                Some((p.surface_point - p.curve_point) / d)
            } else {
                // Tangent where the surface's normal is in the circle's
                // meridian plane.
                b.normal(p.uv.x, p.uv.y)
                    .map(|n| n.into_inner())
                    .filter(|n| n.dot(&tangent).abs() <= tol.angular.sin())
            };
            let Some(n) = normal else { continue };
            for sign in [1.0, -1.0] {
                let m = sign * n;
                let v = wrap_angle(m.dot(&z).atan2(m.dot(&e_sigma)));
                push(
                    out,
                    continuum,
                    pair_of(a, b, Point2::new(wrap_angle(psi), v), p.uv),
                );
            }
        }
    }
    Ok(())
}

// --- the closed forms ---------------------------------------------------------

fn plane_plane(
    a: &Surface,
    b: &Surface,
    fa: &Frame,
    fb: &Frame,
    tol: Tolerance,
    out: &mut SurfaceExtrema,
) {
    let (na, nb) = (fa.z().into_inner(), fb.z().into_inner());
    // Parallel within the angle: every point of one is as far from the
    // other. Otherwise they meet.
    if na.cross(&nb).norm().atan2(na.dot(&nb).abs()) > tol.angular {
        return;
    }
    let gap = (fa.origin() - fb.origin()).dot(&nb);
    let foot = fa.origin() - gap * nb;
    let q = fb.to_local(foot);
    out.continua
        .push(pair_of(a, b, Point2::new(0.0, 0.0), Point2::new(q.x, q.y)));
}

/// A plane against an elliptic cylinder or a cone: stationary where the
/// quadric's normal is the plane's, which is along a ruling.
fn plane_quadric(
    a: &Surface,
    b: &Surface,
    plane: &Frame,
    tol: Tolerance,
    out: &mut SurfaceExtrema,
) {
    let n = plane.z().into_inner();
    let rulings: Vec<Point2> = match *b {
        Surface::EllipticCylinder {
            frame,
            major_radius,
            minor_radius,
        } => {
            let local = frame.vec_to_local(n);
            // The plane's normal across the axis: the axis is along the
            // plane. The section's normal, `(cos u / a, sin u / b)`, is the
            // plane's horizontal part at the two points `u` and `u + π`.
            if local.z.abs() > tol.angular.sin() {
                return;
            }
            let u = (minor_radius * local.y).atan2(major_radius * local.x);
            vec![
                Point2::new(wrap_angle(u), 0.0),
                Point2::new(wrap_angle(u + PI), 0.0),
            ]
        }
        Surface::Cone {
            frame, half_angle, ..
        } => {
            let local = frame.vec_to_local(n);
            // The cone's normal along a ruling is
            // `(cos α cos u, cos α sin u, −sin α)`: the plane's normal is
            // its negative or itself, which is in the angle of the plane to
            // the axis.
            if (local.z.abs().asin() - half_angle).abs() > tol.angular {
                return;
            }
            let u = if local.z < 0.0 {
                local.y.atan2(local.x)
            } else {
                (-local.y).atan2(-local.x)
            };
            vec![Point2::new(wrap_angle(u), 0.0)]
        }
        Surface::Plane { .. }
        | Surface::Cylinder { .. }
        | Surface::Sphere { .. }
        | Surface::Torus { .. }
        | Surface::Nurbs(_) => return,
    };
    for uv_b in rulings {
        let s = b.point(uv_b.x, uv_b.y);
        let h = (s - plane.origin()).dot(&n);
        let foot = plane.to_local(s - h * n);
        out.continua
            .push(pair_of(a, b, Point2::new(foot.x, foot.y), uv_b));
    }
}

fn plane_nurbs(
    a: &Surface,
    b: &Surface,
    plane: &Frame,
    nurbs: &crate::NurbsSurface,
    tol: Tolerance,
    meter: &mut Meter,
    out: &mut SurfaceExtrema,
) -> Result<(), GeomError> {
    let n = plane.z().into_inner();
    let field = crate::nurbs::Field::Height {
        origin: plane.origin(),
        normal: n,
    };
    let critical = nurbs.critical_points(&field, tol, meter)?;
    for (list, continuum) in [(&critical.isolated, false), (&critical.continua, true)] {
        for &uv_b in list {
            let s = b.point(uv_b.x, uv_b.y);
            let h = (s - plane.origin()).dot(&n);
            let foot = plane.to_local(s - h * n);
            push(
                out,
                continuum,
                pair_of(a, b, Point2::new(foot.x, foot.y), uv_b),
            );
        }
    }
    Ok(())
}

/// Two cones about one axis: in each meridian plane their rulings are
/// lines, and two lines are at a distance only if they are parallel — the
/// cones' angles are equal — when every pair of rulings is as far.
fn cone_cone(
    a: &Surface,
    b: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
    out: &mut SurfaceExtrema,
) -> Result<(), GeomError> {
    let (
        &Surface::Cone {
            frame: fa,
            radius: ra,
            half_angle: aa,
        },
        &Surface::Cone {
            frame: fb,
            radius: rb,
            half_angle: ab,
        },
    ) = (a, b)
    else {
        return Err(unsupported(a, b));
    };
    let apex = |f: &Frame, r: f64, alpha: f64| {
        let (sa, ca) = alpha.sin_cos();
        f.origin() - f.z().into_inner() * (r * ca / sa)
    };
    let (za, zb) = (fa.z().into_inner(), fb.z().into_inner());
    let apex_b = apex(&fb, rb, ab);
    let off = apex_b - apex(&fa, ra, aa);
    let lateral = (off - off.dot(&za) * za).norm();
    let coaxial =
        za.cross(&zb).norm().atan2(za.dot(&zb).abs()) <= tol.angular && lateral <= tol.linear;
    if !coaxial {
        return Err(unsupported(a, b));
    }
    if (aa - ab).abs() > tol.angular {
        return Ok(());
    }
    // A point of `a`, and its nearer foot on `b`: the rulings are parallel,
    // so the distance is the same along both.
    let uv_a = Point2::new(0.0, 0.0);
    let s = a.point(uv_a.x, uv_a.y);
    let feet = b.feet(s, tol, meter)?;
    // The foot on the ruling parallel to `a`'s through `s`: the offset to
    // it is across that ruling, which the other foot's, on the mirror
    // ruling that crosses it, is not.
    let ruling = a.eval(uv_a.x, uv_a.y).dv.normalize();
    let Some(f) = feet.isolated.iter().chain(&feet.continua).min_by(|x, y| {
        let across = |f: &crate::SurfaceProjection| (s - f.point).dot(&ruling).abs();
        across(x)
            .total_cmp(&across(y))
            .then(x.uv.x.total_cmp(&y.uv.x))
    }) else {
        return Ok(());
    };
    out.continua.push(pair_of(a, b, uv_a, f.uv));
    Ok(())
}

/// Two elliptic cylinders with parallel axes: stationary where the
/// section ellipses are, each pair a family along the axis.
fn elliptic_elliptic(
    a: &Surface,
    b: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
    out: &mut SurfaceExtrema,
) -> Result<(), GeomError> {
    let (
        &Surface::EllipticCylinder {
            frame: fa,
            major_radius: aa,
            minor_radius: ab,
        },
        &Surface::EllipticCylinder {
            frame: fb,
            major_radius: ba,
            minor_radius: bb,
        },
    ) = (a, b)
    else {
        return Err(unsupported(a, b));
    };
    let (za, zb) = (fa.z().into_inner(), fb.z().into_inner());
    if za.cross(&zb).norm().atan2(za.dot(&zb).abs()) > tol.angular {
        return Err(unsupported(a, b));
    }
    // The second section in the first's plane: its centre moved along the
    // axis onto it.
    let off = fb.origin() - fa.origin();
    let shifted = fb.with_origin(fb.origin() - off.dot(&za) * za);
    let section_a = Curve::Ellipse {
        frame: fa,
        major_radius: aa,
        minor_radius: ab,
    };
    let section_b = Curve::Ellipse {
        frame: shifted,
        major_radius: ba,
        minor_radius: bb,
    };
    let e: Extrema = extrema_curve_curve(&section_a, &section_b, tol, meter)?;
    let height = |f: &Frame, p: Point3| (p - f.origin()).dot(&f.z().into_inner());
    for x in e.isolated.iter().chain(&e.continua) {
        {
            // Every pair is a family along the axis: the representative at
            // the first section's plane, the other's point at the same
            // height.
            let uv_a = Point2::new(x.ta, 0.0);
            let uv_b = Point2::new(x.tb, height(&fb, a.point(uv_a.x, uv_a.y)));
            out.continua.push(pair_of(a, b, uv_a, uv_b));
        }
    }
    Ok(())
}
