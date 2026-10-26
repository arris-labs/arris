//! Pcurves on the surfaces of revolution: cylinder, elliptic cylinder, cone, sphere and torus.

use arris_math::real;
use core::f64::consts::{FRAC_PI_2, TAU};

use arris_math::{
    Frame, Interval, Meter, Point2, Tolerance, UnitVec2, Vec2, Vec3, is_negligible,
    wrap_angle as wrap_turn,
};

use crate::project::ellipse_nearest;
use crate::{Curve, Curve2, GeomError, GeomKind, PcurveFault, Surface};

use super::fitted::fitted_on;
use super::*;

/// The angle in `[0, π/2]` between the lines carried by two vectors.
pub(super) fn line_angle(a: &Vec3, b: &Vec3) -> f64 {
    real::atan2(a.cross(b).norm(), a.dot(b).abs())
}

pub(super) fn on_cylinder(
    curve: &Curve,
    range: Interval,
    cyl: &Frame,
    radius: f64,
    surface: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Curve2, GeomError> {
    match curve {
        &Curve::Line { origin, direction } => {
            let d = cyl.vec_to_local(direction.into_inner());
            if real::atan2(real::hypot(d.x, d.y), d.z.abs()) <= tol.angular {
                // A ruling: constant `u`, `v` running with `t` along ±Z.
                let q = cyl.to_local(origin);
                return Ok(Curve2::Line {
                    origin: Point2::new(wrap_turn(real::atan2(q.y, q.x)), q.z),
                    direction: UnitVec2::new_unchecked(Vec2::new(0.0, d.z.signum())),
                });
            }
            fitted_on(curve, range, surface, tol, meter)
        }
        &Curve::Circle {
            ref frame,
            radius: r,
        } => {
            let centre = cyl.to_local(frame.origin());
            let parallel = line_angle(&frame.z(), &cyl.z()) <= tol.angular
                && real::hypot(centre.x, centre.y) <= tol.linear
                && (r - radius).abs() <= tol.linear;
            if parallel {
                // Constant `v`; `u` starts where the circle's `X` sits
                // against the cylinder's and runs with `t` in the sense of
                // the circle's `Z` against the cylinder's.
                let x = cyl.vec_to_local(frame.x().into_inner());
                let u0 = wrap_turn(real::atan2(x.y, x.x));
                let sense = frame.z().dot(&cyl.z()).signum();
                return Ok(Curve2::Line {
                    origin: Point2::new(u0, centre.z),
                    direction: UnitVec2::new_unchecked(Vec2::new(sense, 0.0)),
                });
            }
            fitted_on(curve, range, surface, tol, meter)
        }
        Curve::Ellipse { .. } | Curve::Nurbs(_) => fitted_on(curve, range, surface, tol, meter),
    }
}

/// The exact pcurves on an elliptic cylinder: a ruling at constant `u`,
/// the section ellipse at constant `v` (the table in [`pcurve_on`]); the
/// rest fitted.
pub(super) fn on_elliptic_cylinder(
    curve: &Curve,
    range: Interval,
    cyl: &Frame,
    [a, b]: [f64; 2],
    surface: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Curve2, GeomError> {
    let kind = GeomKind::Curve(curve.kind());
    match curve {
        &Curve::Line { origin, direction } => {
            let d = cyl.vec_to_local(direction.into_inner());
            if real::atan2(real::hypot(d.x, d.y), d.z.abs()) > tol.angular {
                return fitted_on(curve, range, surface, tol, meter);
            }
            // A ruling: constant `u` at its section point, which is on
            // the ellipse (checked above) and so has a unique parameter
            // unless the ellipse is degenerate to the tolerance.
            let q = cyl.to_local(origin);
            let noise = origin.coords.norm() + cyl.origin().coords.norm();
            let (u, _) = ellipse_nearest(a, b, q.x, q.y, noise)
                .map_err(|locus| degenerate(kind, PcurveFault::RulingAmbiguous(locus)))?;
            Ok(Curve2::Line {
                origin: Point2::new(u, q.z),
                direction: UnitVec2::new_unchecked(Vec2::new(0.0, d.z.signum())),
            })
        }
        &Curve::Ellipse {
            ref frame,
            major_radius,
            minor_radius,
        } => {
            let centre = cyl.to_local(frame.origin());
            let section = real::hypot(centre.x, centre.y) <= tol.linear
                && parallel_axes(&frame.z(), &cyl.z(), tol)
                && parallel_axes(&frame.x(), &cyl.x(), tol)
                && (major_radius - a).abs() <= tol.linear
                && (minor_radius - b).abs() <= tol.linear;
            if !section {
                return fitted_on(curve, range, surface, tol, meter);
            }
            Ok(parallel_pcurve(cyl, frame, centre.z, false))
        }
        Curve::Circle { .. } | Curve::Nurbs(_) => fitted_on(curve, range, surface, tol, meter),
    }
}

/// The distance from `p` to a cone, whose meridian in `(ρ, z)` is the line
/// through `(R, 0)` at the half-angle from the axis — and its mirror, the
/// second nappe, which `ρ ≥ 0` folds onto the same half-plane.
pub(super) fn cone_distance(
    cone: &Frame,
    radius: f64,
    half_angle: f64,
    p: arris_math::Point3,
) -> f64 {
    let q = cone.to_local(p);
    let (rho, z) = (real::hypot(q.x, q.y), q.z);
    let (sin, cos) = (real::sin(half_angle), real::cos(half_angle));
    let near = ((rho - radius) * cos - z * sin).abs();
    let far = ((rho + radius) * cos + z * sin).abs();
    near.min(far)
}

/// `true` when two axes are parallel — either way round — within `tol`.
pub(super) fn parallel_axes(a: &Vec3, b: &Vec3, tol: Tolerance) -> bool {
    line_angle(a, b) <= tol.angular
}

/// `true` when two axes are perpendicular within `tol`.
pub(super) fn perpendicular_axes(a: &Vec3, b: &Vec3, tol: Tolerance) -> bool {
    (FRAC_PI_2 - line_angle(a, b)).abs() <= tol.angular
}

/// The pcurve of a circle about a surface of revolution's axis: a `Line`
/// at constant `v`, `u` starting at the offset of the circle's `X` from
/// the surface's and running in the sense of the circle's `Z` against the
/// surface's, exactly as on a cylinder. `flipped` is for a cone's circle
/// beyond the apex, whose radial factor `R + v sin α` is negative: the
/// surface reaches it at `u + π`.
pub(super) fn parallel_pcurve(surface: &Frame, circle: &Frame, v: f64, flipped: bool) -> Curve2 {
    let x = surface.vec_to_local(circle.x().into_inner());
    let half_turn = if flipped { TAU / 2.0 } else { 0.0 };
    let u0 = wrap_turn(real::atan2(x.y, x.x) + half_turn);
    let sense = if circle.z().dot(&surface.z()) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    Curve2::Line {
        origin: Point2::new(u0, v),
        direction: UnitVec2::new_unchecked(Vec2::new(sense, 0.0)),
    }
}

/// The pcurve of a circle lying in a plane through a surface of
/// revolution's axis — a sphere's meridian, a torus's tube circle — at
/// constant `u`: a `Line` whose `v` runs with `t` or against it. In the
/// (radial, axis) plane the surface's own `v` measures the angle from
/// `radial`, and the circle's `(X, Y)` is that basis turned by `φ` when
/// the two agree in orientation and reflected about `φ / 2` when they do
/// not, so `v` is `φ + t` one way and `φ − t` the other.
///
/// `radial` is the unit direction, in the surface's local frame and in the
/// equatorial plane, that `u` points along. `within` is `None` on a
/// surface whose `v` is periodic, where `φ` is wrapped into `[0, 2π)` and
/// the caller places the line by whole periods; on a sphere, whose `v` is
/// an angle in `[−π/2, π/2]` and no period of anything, it is the range
/// the pcurve is for, and `φ` is the turn of it that puts the range's
/// middle there — the half circle from pole to pole by way of `t = π` is
/// `v = t − π`, not the `t + π` the angle alone gives.
pub(super) fn meridian_pcurve(
    surface: &Frame,
    circle: &Frame,
    radial: Vec3,
    within: Option<Interval>,
) -> Curve2 {
    let u = wrap_turn(real::atan2(radial.y, radial.x));
    let x = surface.vec_to_local(circle.x().into_inner());
    let y = surface.vec_to_local(circle.y().into_inner());
    let (a, b) = (x.dot(&radial), x.z);
    let (c, d) = (y.dot(&radial), y.z);
    let phi = real::atan2(b, a);
    let sense = if a * d - c * b >= 0.0 { 1.0 } else { -1.0 };
    let phi = match within {
        None => wrap_turn(phi),
        Some(range) => phi - TAU * ((phi + sense * range.midpoint()) / TAU).round(),
    };
    Curve2::Line {
        origin: Point2::new(u, phi),
        direction: UnitVec2::new_unchecked(Vec2::new(0.0, sense)),
    }
}

/// The unit equatorial direction of the half-plane a meridian arc lies
/// in: `axis` is the candidate, `±` the one the curve's own points pick.
/// The midpoint is asked first and the start second, since a curve may
/// begin on the axis (a sphere's pole) where the half-plane is not
/// decided; neither deciding leaves the candidate as written, which names
/// the same meridian circle at `u + π`.
pub(super) fn meridian_radial(
    surface: &Frame,
    curve: &Curve,
    range: Interval,
    candidate: Vec3,
    scale: f64,
) -> Vec3 {
    for t in [range.midpoint(), range.lo()] {
        let q = surface.to_local(curve.point(t));
        let radial = Vec2::new(q.x, q.y);
        if !is_negligible(radial.norm(), scale) {
            let along = radial.x * candidate.x + radial.y * candidate.y;
            return if along >= 0.0 { candidate } else { -candidate };
        }
    }
    candidate
}

/// The exact pcurves on a cone: a ruling at constant `u`, a circle about
/// the axis at constant `v`; the rest fitted.
#[allow(clippy::too_many_arguments)]
pub(super) fn on_cone(
    curve: &Curve,
    range: Interval,
    cone: &Frame,
    radius: f64,
    half_angle: f64,
    surface: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Curve2, GeomError> {
    let kind = GeomKind::Curve(curve.kind());
    let (sin, cos) = (real::sin(half_angle), real::cos(half_angle));
    if is_negligible(sin, 1.0) || cos <= 0.0 {
        return Err(degenerate(
            GeomKind::Surface(surface.kind()),
            PcurveFault::ConeHalfAngle { half_angle },
        ));
    }
    match curve {
        &Curve::Line { origin, direction } => {
            // A line on a cone is a ruling through the apex: `v` runs
            // along it, and the sense is the sign of its axial part, since
            // the ruling climbs by `cos α > 0` per unit of `v`.
            let d = cone.vec_to_local(direction.into_inner());
            let sense = if d.z >= 0.0 { 1.0 } else { -1.0 };
            let equatorial = Vec2::new(sense * d.x, sense * d.y);
            if (real::atan2(equatorial.norm(), d.z.abs()) - half_angle).abs() > tol.angular {
                return fitted_on(curve, range, surface, tol, meter);
            }
            if is_negligible(equatorial.norm(), 1.0) {
                return Err(degenerate(kind, PcurveFault::RulingNoRadial));
            }
            let u = wrap_turn(real::atan2(equatorial.y, equatorial.x));
            let v0 = cone.to_local(origin).z / cos;
            Ok(Curve2::Line {
                origin: Point2::new(u, v0),
                direction: UnitVec2::new_unchecked(Vec2::new(0.0, sense)),
            })
        }
        Curve::Circle { frame, .. } => {
            let centre = cone.to_local(frame.origin());
            let about_axis = real::hypot(centre.x, centre.y) <= tol.linear
                && parallel_axes(&frame.z(), &cone.z(), tol);
            if !about_axis {
                return fitted_on(curve, range, surface, tol, meter);
            }
            let v = centre.z / cos;
            Ok(parallel_pcurve(cone, frame, v, radius + v * sin < 0.0))
        }
        Curve::Ellipse { .. } | Curve::Nurbs(_) => fitted_on(curve, range, surface, tol, meter),
    }
}

/// The exact pcurves on a sphere: a parallel at constant `v`, a meridian
/// at constant `u`; the rest fitted.
pub(super) fn on_sphere(
    curve: &Curve,
    range: Interval,
    sphere: &Frame,
    radius: f64,
    surface: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Curve2, GeomError> {
    match curve {
        &Curve::Circle {
            ref frame,
            radius: rho,
        } => {
            let centre = sphere.to_local(frame.origin());
            if real::hypot(centre.x, centre.y) <= tol.linear
                && parallel_axes(&frame.z(), &sphere.z(), tol)
            {
                // A parallel: `v` is the latitude of its plane.
                return Ok(parallel_pcurve(
                    sphere,
                    frame,
                    real::atan2(centre.z, rho),
                    false,
                ));
            }
            // A meridian: the great circle whose plane holds the axis.
            let through_axis = centre.coords.norm() <= tol.linear
                && (rho - radius).abs() <= tol.linear
                && perpendicular_axes(&frame.z(), &sphere.z(), tol);
            if !through_axis {
                return fitted_on(curve, range, surface, tol, meter);
            }
            let z = sphere.vec_to_local(frame.z().into_inner());
            let candidate = Vec3::new(-z.y, z.x, 0.0);
            let Some(candidate) = candidate.try_normalize(0.0) else {
                return fitted_on(curve, range, surface, tol, meter);
            };
            let radial = meridian_radial(sphere, curve, range, candidate, radius);
            Ok(meridian_pcurve(sphere, frame, radial, Some(range)))
        }
        Curve::Line { .. } | Curve::Ellipse { .. } | Curve::Nurbs(_) => {
            fitted_on(curve, range, surface, tol, meter)
        }
    }
}

/// The exact pcurves on a torus: a circle about the axis at constant `v`,
/// a circle of the tube at constant `u`; the rest fitted.
#[allow(clippy::too_many_arguments)]
pub(super) fn on_torus(
    curve: &Curve,
    range: Interval,
    torus: &Frame,
    major_radius: f64,
    minor_radius: f64,
    surface: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Curve2, GeomError> {
    match curve {
        &Curve::Circle {
            ref frame,
            radius: rho,
        } => {
            let centre = torus.to_local(frame.origin());
            let equatorial = Vec2::new(centre.x, centre.y);
            if equatorial.norm() <= tol.linear && parallel_axes(&frame.z(), &torus.z(), tol) {
                // About the axis: `v` is where the tube's own angle puts
                // this radius and height.
                let v = wrap_turn(real::atan2(centre.z, rho - major_radius));
                return Ok(parallel_pcurve(torus, frame, v, false));
            }
            let Some(radial) = Vec3::new(centre.x, centre.y, 0.0).try_normalize(0.0) else {
                return fitted_on(curve, range, surface, tol, meter);
            };
            let z = torus.vec_to_local(frame.z().into_inner());
            let of_the_tube = (equatorial.norm() - major_radius).abs() <= tol.linear
                && centre.z.abs() <= tol.linear
                && (rho - minor_radius).abs() <= tol.linear
                && perpendicular_axes(&z, &Vec3::z(), tol)
                && perpendicular_axes(&z, &radial, tol);
            if !of_the_tube {
                return fitted_on(curve, range, surface, tol, meter);
            }
            Ok(meridian_pcurve(torus, frame, radial, None))
        }
        Curve::Line { .. } | Curve::Ellipse { .. } | Curve::Nurbs(_) => {
            fitted_on(curve, range, surface, tol, meter)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::{Frame, Point3, Precision, Vec2, Vec3};

    fn tol() -> Tolerance {
        Precision::DEFAULT.tolerance()
    }

    #[test]
    fn an_elliptic_cylinder_carries_its_section_and_its_rulings_as_lines() {
        let wall = Surface::EllipticCylinder {
            frame: Frame::world(),
            major_radius: 3.0,
            minor_radius: 2.0,
        };
        // The section at height 4, traversed against the axis: `u` runs
        // backwards from 0.
        let section = Curve::Ellipse {
            frame: Frame::new(Point3::new(0.0, 0.0, 4.0), -Vec3::z(), Vec3::x()).unwrap(),
            major_radius: 3.0,
            minor_radius: 2.0,
        };
        let pc = pcurve_on(
            &section,
            Interval::TURN,
            &wall,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        let Curve2::Line { origin, direction } = pc else {
            panic!("{pc:?}")
        };
        assert_eq!(origin, Point2::new(0.0, 4.0));
        assert_eq!(direction.into_inner(), Vec2::new(-1.0, 0.0));
        // The ruling through the minor vertex, running down.
        let ruling = Curve::Line {
            origin: Point3::new(0.0, 2.0, 9.0),
            direction: -Vec3::z_axis(),
        };
        let pc = pcurve_on(
            &ruling,
            Interval::new(0.0, 5.0).unwrap(),
            &wall,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        let Curve2::Line { origin, direction } = pc else {
            panic!("{pc:?}")
        };
        assert!((origin.x - FRAC_PI_2).abs() < 1e-12 && origin.y == 9.0);
        assert_eq!(direction.into_inner(), Vec2::new(0.0, -1.0));
        // A circle of the minor radius touches the surface at two points
        // and is off it elsewhere; a section of other radii is off it.
        let circle = Curve::Circle {
            frame: Frame::world(),
            radius: 2.0,
        };
        assert!(matches!(
            pcurve_on(
                &circle,
                Interval::TURN,
                &wall,
                tol(),
                &mut arris_math::Meter::default()
            ),
            Err(GeomError::NotOnSurface { .. })
        ));
        // No ellipse lies on the surface but the sections, so a chord
        // short enough to be within the tolerance of it is what reaches
        // the fitted arm by hand.
        let chord = Curve::Line {
            origin: Point3::new(3.0, 0.0, 0.0),
            direction: Vec3::y_axis(),
        };
        let pc = pcurve_on(
            &chord,
            Interval::new(0.0, 1e-9).unwrap(),
            &wall,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert!(matches!(pc, Curve2::Nurbs(_)), "{pc:?}");
    }

    #[test]
    fn a_ruling_and_a_parallel_are_lines_in_uv() {
        let wall = Surface::Cylinder {
            frame: Frame::world(),
            radius: 2.0,
        };
        let ruling = Curve::Line {
            origin: Point3::new(0.0, 2.0, 5.0),
            direction: -Vec3::z_axis(),
        };
        let pc = pcurve_on(
            &ruling,
            Interval::new(-1.0, 1.0).unwrap(),
            &wall,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        let Curve2::Line { origin, direction } = pc else {
            panic!("{pc:?}")
        };
        assert!((origin.x - FRAC_PI_2).abs() < 1e-15 && origin.y == 5.0);
        assert_eq!(direction.into_inner(), Vec2::new(0.0, -1.0));
        // A parallel traversed against the cylinder's Z runs u backwards.
        let ring = Curve::Circle {
            frame: Frame::new(Point3::new(0.0, 0.0, 1.0), -Vec3::z(), Vec3::y()).unwrap(),
            radius: 2.0,
        };
        let pc = pcurve_on(
            &ring,
            Interval::TURN,
            &wall,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        let Curve2::Line { origin, direction } = pc else {
            panic!("{pc:?}")
        };
        assert!((origin.x - FRAC_PI_2).abs() < 1e-15 && origin.y == 1.0);
        assert_eq!(direction.into_inner(), Vec2::new(-1.0, 0.0));
    }
}
