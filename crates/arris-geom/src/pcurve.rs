//! Pcurves: the (u, v) image of a 3D curve on a surface at the curve's own
//! parameter, exact where a `Curve2` variant exists and a NURBS fitted
//! over the surface's projection otherwise, on every surface
//! (`docs/DATA-MODEL.md` §Pcurves), and the projection of a curve onto a
//! plane for a consumer's sketch.

mod fitted;
mod plane;
mod revolution;

use arris_math::real;
use core::f64::consts::FRAC_PI_2;

use arris_math::{Interval, Meter, Point2, Tolerance};

use crate::project::ellipse_distance;
use crate::{Curve, Curve2, GeomError, GeomKind, NurbsCurve2, PcurveFault, Surface, fit_curve2};

use fitted::fitted_on;
pub(crate) use plane::principal_axes;
pub use plane::project_to_plane;
use plane::{in_plane_curve, projected_in_plane};
use revolution::{
    cone_distance, line_angle, on_cone, on_cylinder, on_elliptic_cylinder, on_sphere, on_torus,
};

/// How many parameters `pcurve_on` samples over the range to decide that
/// the curve lies on the surface and where it nears a singular point, and
/// the resolution of the table that unwraps a periodic parameter along
/// the curve: an interval over which one swings by a quarter turn is
/// halved until it does not, and a curve that still does after
/// thirty-two halvings is refused. A sampling density, not a tolerance;
/// the fitted result is verified at the fit's own, denser, check
/// parameters.
pub const PCURVE_SAMPLES: usize = 256;

/// The degree of a fitted pcurve. The oblique section of a cylinder is a
/// sinusoid in (u, v) whose amplitude is `R tan α` for the angle `α`
/// between the plane's normal and the axis, so a grazing plane has an
/// amplitude thousands of times its radius; a quintic fits that to a
/// micrometre well inside `MAX_FIT_SPANS`, where a cubic would run out of
/// spans. A structural choice, not a tolerance.
pub const PCURVE_FIT_DEGREE: usize = 5;

/// The fraction of `tol.linear` within which a curve is *on* a singular
/// point of a surface — a cone's apex, a sphere's pole — for
/// [`pcurve_on`]: a range that ends that near one ends on it, and one
/// that comes that near anywhere else runs through it. A pcurve that ends
/// on the point is off the curve there by the curve's own miss, which no
/// refinement removes, and the fit accepts half the tolerance; a quarter
/// leaves the fit the other quarter, so splitting where
/// [`GeomError::ThroughSingularity`] says always gives two ranges that
/// fit. It is [`crate::SECTION_FIT_FRACTION`] as well, which is how near
/// its exact branch, and so its surfaces, a fitted section is held. A ratio between two fits, not a
/// tolerance.
pub const PCURVE_SINGULAR_BAND: f64 = 0.25;

fn degenerate(kind: GeomKind, reason: PcurveFault) -> GeomError {
    GeomError::Degenerate {
        kind,
        reason: reason.into(),
    }
}

/// The pcurve of `curve` over `range` on `surface`: a `Curve2` whose image
/// under the surface is the curve *at the same parameter*
/// (`surface.point(pcurve(t)) == curve.point(t)`), which is invariant E4
/// of `docs/DATA-MODEL.md` before the checker exists.
///
/// The table is exhaustive over (curve, surface). On a plane every
/// variant that lies in it is exact: a line is a `Line`, a circle a
/// `Circle` and an ellipse an `Ellipse` placed by a `Frame2` whose
/// handedness is the sign of the curve's `Z` against the plane's normal,
/// a NURBS a `Nurbs` with its control points projected. A line or a
/// conic tilted from the plane by more than `tol.angular` lies within
/// `tol.linear` of it over the range alone — a block of a section beside
/// an operand edge — and its projection onto the plane is a `Nurbs`
/// fitted within `tol.linear` of it there: off the curve by the curve's
/// own distance from the plane and no more than that again. On a cylinder a ruling is a `Line` at
/// constant `u`, a circle around the axis a `Line` at constant `v` whose
/// `u` starts at the offset of the circle's `X` from the cylinder's, and
/// everything else — an oblique section, a NURBS — is a `Nurbs` fitted by
/// [`fit_curve2`] over the cylinder's projection with `u` unwrapped along
/// `t`, so a seam crossing stays continuous and `u` may leave `[0, 2π)`.
///
/// On the surfaces of revolution the exact arms are the six a revolve
/// makes, each a `Line` in (u, v) at the curve's own parameter: on a
/// **cone**, a ruling — the line through the apex — at constant `u`, and a
/// circle about the axis at constant `v`; on a **sphere**, a circle about
/// the axis at constant `v` (a parallel) and the great circle through both
/// poles at constant `u` (a meridian); on a **torus**, a circle about the
/// axis at constant `v` and a circle of the tube at constant `u`. A `u`
/// origin is the offset of the circle's `X` from the surface's, in
/// `[0, 2π)`, running in the sense of the circle's `Z` against the
/// surface's, as on the cylinder; a constant-`u` arm's `v` runs with `t`
/// or against it by the turn of the circle's own axes in the plane of the
/// axis, and a meridian's `v` leaves `[−π/2, π/2]` where the great circle
/// passes a pole onto the opposite meridian, which is where the sphere's
/// parametrisation puts it.
///
/// On an **elliptic cylinder** the exact arms are the two an extrude
/// makes (ADR-0014), each a `Line`: a ruling — a line along the axis —
/// at constant `u`, the parameter of its section point, `v` running with
/// `t` or against it by the sign of the line's direction against `Z`;
/// and the section ellipse — centred on the axis, its `Z` along the axis
/// and its major axis along the surface's `X` either way, the radii
/// agreeing within `tol.linear` — at constant `v`, `u` starting at `0` or
/// `π` by the sign of its `X` against the surface's and running in the
/// sense of its `Z` against the surface's, as a parallel does on a
/// cylinder.
///
/// **Every other curve on these five surfaces is fitted**, by the one
/// fallback the cylinder's oblique section takes: a `Nurbs` from
/// [`fit_curve2`] over the surface's own projection of the curve
/// ([`Surface::project`]), held to the curve in 3D. Each periodic
/// parameter — `u`, and on a torus `v` — is unwrapped along `t`, so a seam
/// crossing is continuous and the parameter may leave `[0, 2π)`; the
/// unwrapping halves the interval between two of its [`PCURVE_SAMPLES`]
/// wherever the parameter swings by a quarter turn, which is what `u`
/// does beside a pole or an apex, and a curve passing one fits from the
/// band below out.
///
/// On a **NURBS surface** every curve is fitted, the distance that decides
/// it is on the surface is [`Surface::project`]'s global one, and the
/// fitted arm's projections start from the unwrapping table's own
/// neighbour, falling back to the global search where that lands farther
/// than the band from the curve. A direction the surface closes in
/// ([`crate::NurbsSurface::closure`]: periodic knots, or a clamped
/// direction whose ends are one row) is unwrapped by its closure as a
/// turn is on the analytic surfaces, so a pcurve crossing the seam is
/// continuous and may leave the domain, which evaluation wraps. A start on
/// the seam reads at the knots' start, and a seam's other use is the
/// caller's to place a period along ([`Surface::period`]), as on the
/// analytic surfaces. A collapsed row — a pole, an apex — is a singular
/// point like theirs, whichever parameter runs along it.
///
/// A fitted pcurve never runs through a **singular point** of the
/// surface — a cone's apex, a sphere's pole — where `u` has no value.
/// The decision is a distance: within [`PCURVE_SINGULAR_BAND`] of
/// `tol.linear` of the point the curve is on it. A range with that
/// inside it is [`GeomError::ThroughSingularity`] naming the parameter of
/// the nearest approach, and both sides of a split there fit. A range
/// that *ends* there is fitted, and ends on the point's own `v` with the
/// `u` the curve arrives with — the limit along it, read from its
/// tangent — and a range may end on the one point twice, a closed curve
/// through a pole cut there, each end read on its own side (ADR-0021).
/// The exact arms are as they were: a ruling keeps its one `u` through
/// the apex, a meridian its one `u` over a pole, its `v` the sphere's
/// own latitudes over the range asked.
///
/// Errors: [`GeomError::NotOnSurface`] when the curve is farther than
/// `tol.linear` from the surface at any of [`PCURVE_SAMPLES`] + 1 parameters
/// over the range; [`GeomError::ThroughSingularity`] as above;
/// [`GeomError::Fit`] when the fitted arm cannot reach `tol.linear`;
/// [`GeomError::Degenerate`] for an unbounded or empty range, a curve
/// winding faster than the sampling resolves — or jumping, as one that
/// changes a cone's nappe beside the apex does — or one that arrives at a
/// singular point with no tangent; [`GeomError::Ambiguous`] where the
/// global projection onto a NURBS surface is; [`GeomError::InvalidTolerance`].
///
/// ```
/// use arris_geom::{Curve, Curve2, Surface, pcurve_on};
/// use arris_math::{Frame, Interval, Point3, Precision, Vec3};
///
/// let wall = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
/// let ring = Curve::Circle { frame: Frame::from_z(Point3::new(0.0, 0.0, 3.0), Vec3::z()).unwrap(), radius: 2.0 };
/// let pc = pcurve_on(&ring, Interval::TURN, &wall, Precision::DEFAULT.tolerance(), &mut arris_math::Meter::default()).unwrap();
/// let Curve2::Line { origin, direction } = pc else { panic!() };
/// assert_eq!(origin.y, 3.0);
/// assert_eq!(direction.x, 1.0); // u runs with t, v stays at 3
/// ```
pub fn pcurve_on(
    curve: &Curve,
    range: Interval,
    surface: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Curve2, GeomError> {
    if !tol.is_consistent() {
        return Err(GeomError::InvalidTolerance(tol));
    }
    let curve_kind = GeomKind::Curve(curve.kind());
    if !range.is_bounded() || range.length() <= 0.0 {
        return Err(degenerate(
            curve_kind,
            PcurveFault::Range {
                lo: range.lo(),
                hi: range.hi(),
            },
        ));
    }
    match surface {
        Surface::Plane { frame } => {
            check_on(curve, range, surface, tol, |p| frame.to_local(p).z.abs())?;
            // A line or a conic is exact in the plane only when it lies
            // in it: one tilted from it by more than the angular
            // tolerance lies within `tol.linear` of it over the range
            // alone, and its projection there is not the curve's own
            // shape at its own parameter — a tilted circle projects to an
            // ellipse, a tilted line at a slower speed.
            let tilt = match curve {
                Curve::Line { direction, .. } => {
                    FRAC_PI_2 - line_angle(&direction.into_inner(), &frame.z())
                }
                Curve::Circle { frame: own, .. } | Curve::Ellipse { frame: own, .. } => {
                    line_angle(&own.z(), &frame.z())
                }
                Curve::Nurbs(_) => 0.0,
            };
            if tilt > tol.angular {
                return projected_in_plane(curve, range, frame, tol, meter);
            }
            in_plane_curve(curve, frame)
        }
        &Surface::Cylinder { ref frame, radius } => {
            check_on(curve, range, surface, tol, |p| {
                let q = frame.to_local(p);
                (real::hypot(q.x, q.y) - radius).abs()
            })?;
            on_cylinder(curve, range, frame, radius, surface, tol, meter)
        }
        &Surface::EllipticCylinder {
            ref frame,
            major_radius,
            minor_radius,
        } => {
            check_on(curve, range, surface, tol, |p| {
                let q = frame.to_local(p);
                let noise = p.coords.norm() + frame.origin().coords.norm();
                ellipse_distance(major_radius, minor_radius, q.x, q.y, noise)
            })?;
            on_elliptic_cylinder(
                curve,
                range,
                frame,
                [major_radius, minor_radius],
                surface,
                tol,
                meter,
            )
        }
        &Surface::Cone {
            ref frame,
            radius,
            half_angle,
        } => {
            check_on(curve, range, surface, tol, |p| {
                cone_distance(frame, radius, half_angle, p)
            })?;
            on_cone(curve, range, frame, radius, half_angle, surface, tol, meter)
        }
        &Surface::Sphere { ref frame, radius } => {
            check_on(curve, range, surface, tol, |p| {
                ((p - frame.origin()).norm() - radius).abs()
            })?;
            on_sphere(curve, range, frame, radius, surface, tol, meter)
        }
        &Surface::Torus {
            ref frame,
            major_radius,
            minor_radius,
        } => {
            check_on(curve, range, surface, tol, |p| {
                let q = frame.to_local(p);
                (real::hypot(real::hypot(q.x, q.y) - major_radius, q.z) - minor_radius).abs()
            })?;
            on_torus(
                curve,
                range,
                frame,
                major_radius,
                minor_radius,
                surface,
                tol,
                meter,
            )
        }
        Surface::Nurbs(nurbs) => {
            // The distance is the global projection's: a NURBS surface has
            // no closed form, and a local one could call a curve on a fold
            // of the surface off it.
            let mut fault = None;
            check_on(curve, range, surface, tol, |p| match nurbs.project(p) {
                Ok(near) => near.distance,
                Err(e) => {
                    fault.get_or_insert(e);
                    f64::NAN
                }
            })
            .map_err(|e| fault.take().unwrap_or(e))?;
            fitted_on(curve, range, surface, tol, meter)
        }
    }
}

/// [`GeomError::NotOnSurface`] at the first of [`PCURVE_SAMPLES`]
/// parameters where `distance` of the curve's point exceeds `tol.linear`.
fn check_on(
    curve: &Curve,
    range: Interval,
    surface: &Surface,
    tol: Tolerance,
    mut distance: impl FnMut(arris_math::Point3) -> f64,
) -> Result<(), GeomError> {
    for i in 0..=PCURVE_SAMPLES {
        let t = range.lerp(i as f64 / PCURVE_SAMPLES as f64);
        let d = distance(curve.point(t));
        if d.is_nan() || d > tol.linear {
            return Err(GeomError::NotOnSurface {
                curve: GeomKind::Curve(curve.kind()),
                surface: GeomKind::Surface(surface.kind()),
                t,
                distance: d,
            });
        }
    }
    Ok(())
}

/// `pc` over `range`, same-parameter, with its start moved to `ends[0]`
/// and its end to `ends[1]` where given: a clamped B-spline over `range`
/// whose end control points are moved, so the curve changes only over
/// its first and last spans and by no more than the move. A pcurve that
/// is one already is used as it is, a line becomes the degree-1 spline
/// through its ends, and any other — a circle or an ellipse on a plane,
/// a spline over a wider or periodic domain — is fitted over `range`
/// first, at `tolerance` on `surface` as [`pcurve_on`] fits.
///
/// Guarantees: the result is a clamped `Curve2::Nurbs` over `range`,
/// equal to `pc` at an end not given and at `ends` where given, and
/// within the move (and the fit's `tolerance`, where one was needed) of
/// `pc` everywhere else. Errors: [`GeomError::Fit`] where the fit
/// cannot reach `tolerance`, and what [`NurbsCurve2::new`] refuses.
///
/// ```
/// use arris_geom::{Curve2, Surface, pcurve_ending_on};
/// use arris_math::{Frame, Interval, Point2, UnitVec2, Vec2};
///
/// let plane = Surface::Plane { frame: Frame::world() };
/// let line = Curve2::Line { origin: Point2::new(0.0, 0.0), direction: UnitVec2::new_normalize(Vec2::x()) };
/// let range = Interval::new(0.0, 1.0).unwrap();
/// let to = Point2::new(1.0, 1e-7);
/// let moved = pcurve_ending_on(&line, range, [None, Some(to)], &plane, 1e-7, &mut arris_math::Meter::default()).unwrap();
/// assert_eq!(moved.point(1.0), to);
/// assert_eq!(moved.point(0.0), Point2::new(0.0, 0.0));
/// ```
pub fn pcurve_ending_on(
    pc: &Curve2,
    range: Interval,
    ends: [Option<Point2>; 2],
    surface: &Surface,
    tolerance: f64,
    meter: &mut Meter,
) -> Result<Curve2, GeomError> {
    let clamped = |n: &NurbsCurve2| {
        let (k, p) = (n.knots(), n.degree());
        n.period().is_none()
            && n.domain() == range
            && k[..=p].iter().all(|&x| x == k[0])
            && k[k.len() - p - 1..].iter().all(|&x| x == k[k.len() - 1])
    };
    let spline = match pc {
        Curve2::Nurbs(n) if clamped(n) => n.clone(),
        Curve2::Line { .. } => NurbsCurve2::new(
            1,
            vec![range.lo(), range.lo(), range.hi(), range.hi()],
            vec![pc.point(range.lo()), pc.point(range.hi())],
            vec![1.0; 2],
        )?,
        Curve2::Circle { .. } | Curve2::Ellipse { .. } | Curve2::Nurbs(_) => {
            let on = |q: Point2| surface.point(q.x, q.y);
            fit_curve2(
                |t| pc.point(t),
                range,
                PCURVE_FIT_DEGREE,
                |t, q| (on(q) - on(pc.point(t))).norm(),
                tolerance,
                meter,
            )?
        }
    };
    let mut points = spline.control_points().to_vec();
    if let (Some(p), Some(first)) = (ends[0], points.first_mut()) {
        *first = p;
    }
    if let (Some(p), Some(last)) = (ends[1], points.last_mut()) {
        *last = p;
    }
    NurbsCurve2::new(
        spline.degree(),
        spline.knots().to_vec(),
        points,
        spline.weights().to_vec(),
    )
    .map(Curve2::Nurbs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::{Frame, Frame2, Point3, Precision, UnitVec2, Vec2, Vec3};

    fn tol() -> Tolerance {
        Precision::DEFAULT.tolerance()
    }

    #[test]
    fn a_lifted_curve_is_not_on_the_surface_and_names_where() {
        let plane = Surface::Plane {
            frame: Frame::world(),
        };
        let lifted = Curve::Line {
            origin: Point3::new(0.0, 0.0, 1e-3),
            direction: Vec3::x_axis(),
        };
        let range = Interval::new(0.0, 1.0).unwrap();
        let err = pcurve_on(
            &lifted,
            range,
            &plane,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap_err();
        assert!(
            matches!(err, GeomError::NotOnSurface { distance, .. } if (distance - 1e-3).abs() < 1e-15),
            "{err}"
        );
        assert!(matches!(
            pcurve_on(
                &lifted,
                Interval::REAL,
                &plane,
                tol(),
                &mut arris_math::Meter::default()
            ),
            Err(GeomError::Degenerate { .. })
        ));
        let sphere = Surface::Sphere {
            frame: Frame::world(),
            radius: 1.0,
        };
        // A line is nowhere near a sphere: that is not an unsupported
        // pair, it is a curve off the surface.
        assert!(matches!(
            pcurve_on(
                &lifted,
                range,
                &sphere,
                tol(),
                &mut arris_math::Meter::default()
            ),
            Err(GeomError::NotOnSurface { .. })
        ));
        // A small circle *on* the sphere about no axis of it has no
        // `Curve2` variant, and is fitted.
        let small = Curve::Circle {
            frame: Frame::from_z(Point3::new(0.5, 0.0, 0.0), Vec3::x()).unwrap(),
            radius: 0.75f64.sqrt(),
        };
        let pc = pcurve_on(
            &small,
            Interval::TURN,
            &sphere,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert!(matches!(pc, Curve2::Nurbs(_)), "{pc:?}");
        assert!(matches!(
            pcurve_on(
                &lifted,
                range,
                &plane,
                Tolerance::new(0.0, 1.0),
                &mut arris_math::Meter::default()
            ),
            Err(GeomError::InvalidTolerance(_))
        ));
    }

    /// A pcurve ended on a (u, v) a tolerance away moves its end there
    /// exactly and elsewhere by no more than the move, same-parameter
    /// throughout: a line becomes the spline through its new ends, and a
    /// circle, fitted over the range first, changes only over its end
    /// spans, its middle where it was to the fit's rounding.
    #[test]
    fn a_pcurve_is_ended_on_a_vertex_by_its_end_control_points() {
        let plane = Surface::Plane {
            frame: Frame::world(),
        };
        let range = Interval::new(0.0, 2.0).unwrap();
        let line = Curve2::Line {
            origin: Point2::new(1.0, 1.0),
            direction: UnitVec2::new_normalize(Vec2::new(1.0, 0.0)),
        };
        let to = Point2::new(3.0, 1.0 + 1.2e-7);
        let moved = pcurve_ending_on(
            &line,
            range,
            [None, Some(to)],
            &plane,
            1e-7,
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert_eq!(moved.point(0.0), line.point(0.0));
        assert_eq!(moved.point(2.0), to);
        assert!((moved.point(1.0) - Point2::new(2.0, 1.0 + 0.6e-7)).norm() < 1e-15);

        let circle = Curve2::Circle {
            frame: Frame2::identity(),
            radius: 1.0,
        };
        let from = circle.point(0.0) + Vec2::new(0.0, -1.2e-7);
        let moved = pcurve_ending_on(
            &circle,
            range,
            [Some(from), None],
            &plane,
            1e-7,
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert_eq!(moved.point(0.0), from);
        assert!((moved.point(2.0) - circle.point(2.0)).norm() < 1e-15);
        for i in 0..=100 {
            let t = range.lerp(f64::from(i) / 100.0);
            let off = (moved.point(t) - circle.point(t)).norm();
            assert!(off <= 1.2e-7 + 1e-7, "{off} at {t}");
        }
        assert!((moved.point(1.0) - circle.point(1.0)).norm() <= 1e-7);
    }
}
