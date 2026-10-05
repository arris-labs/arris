//! Pcurves: the (u, v) image of a 3D curve on a surface at the curve's own
//! parameter, exact where a `Curve2` variant exists and a NURBS fitted
//! over the surface's projection otherwise, on every surface
//! (`docs/DATA-MODEL.md` §Pcurves), and the projection of a curve onto a
//! plane for a consumer's sketch.

use core::f64::consts::{FRAC_PI_2, TAU};

use arris_math::{
    Frame, Frame2, Handedness, Interval, Meter, Point2, Point3, Tolerance, UnitVec2, Vec2, Vec3,
    is_negligible, wrap_angle as wrap_turn, wrap_signed,
};

use crate::project::{ellipse_distance, ellipse_nearest};
use crate::{Curve, Curve2, GeomError, GeomKind, NurbsCurve, NurbsCurve2, Surface, fit_curve2};

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

/// The (u, v) coordinates of `p` in a plane's frame.
fn in_plane(plane: &Frame, p: arris_math::Point3) -> Point2 {
    let q = plane.to_local(p);
    Point2::new(q.x, q.y)
}

/// The (u, v) components of `v` in a plane's frame.
fn in_plane_vec(plane: &Frame, v: Vec3) -> Vec2 {
    let q = plane.vec_to_local(v);
    Vec2::new(q.x, q.y)
}

fn degenerate(kind: GeomKind, reason: impl Into<String>) -> GeomError {
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
            format!(
                "a pcurve needs a bounded range of positive length, not [{}, {}]",
                range.lo(),
                range.hi()
            ),
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
                (q.x.hypot(q.y) - radius).abs()
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
                ((q.x.hypot(q.y) - major_radius).hypot(q.z) - minor_radius).abs()
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

/// The exact pcurve of a curve lying in `plane`, at the same parameter.
fn in_plane_curve(curve: &Curve, plane: &Frame) -> Result<Curve2, GeomError> {
    let kind = GeomKind::Curve(curve.kind());
    match curve {
        &Curve::Line { origin, direction } => {
            let d = in_plane_vec(plane, direction.into_inner());
            let direction = UnitVec2::try_new(d, 0.0)
                .ok_or_else(|| degenerate(kind, "the line is perpendicular to the plane"))?;
            Ok(Curve2::Line {
                origin: in_plane(plane, origin),
                direction,
            })
        }
        &Curve::Circle { ref frame, radius } => Ok(Curve2::Circle {
            frame: conic_frame(plane, frame, kind)?,
            radius,
        }),
        &Curve::Ellipse {
            ref frame,
            major_radius,
            minor_radius,
        } => Ok(Curve2::Ellipse {
            frame: conic_frame(plane, frame, kind)?,
            major_radius,
            minor_radius,
        }),
        Curve::Nurbs(c) => Ok(Curve2::Nurbs(nurbs_in_plane(c, plane)?)),
    }
}

/// The projection of `curve` over `range` onto `plane`, at its own
/// parameter, as a `Nurbs` fitted to it within `tol.linear` in the plane:
/// a curve off the plane is as near as any pcurve can be to it there, the
/// projection, and holding the fit to the curve itself would ask it to
/// close the curve's own distance from the plane, which it cannot.
fn projected_in_plane(
    curve: &Curve,
    range: Interval,
    plane: &Frame,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Curve2, GeomError> {
    let f = |t: f64| in_plane(plane, curve.point(t));
    let fit = fit_curve2(
        f,
        range,
        PCURVE_FIT_DEGREE,
        |t, q| (q - f(t)).norm(),
        tol.linear,
        meter,
    )?;
    Ok(Curve2::Nurbs(fit))
}

/// The `Frame2` of a conic whose frame lies in `plane`: origin and `X` in
/// plane coordinates, right-handed when the conic's `Z` is along the
/// plane's normal and left-handed when it opposes it.
fn conic_frame(plane: &Frame, frame: &Frame, kind: GeomKind) -> Result<Frame2, GeomError> {
    let handedness = if frame.z().dot(&plane.z()) >= 0.0 {
        Handedness::Right
    } else {
        Handedness::Left
    };
    Frame2::new(
        in_plane(plane, frame.origin()),
        in_plane_vec(plane, frame.x().into_inner()),
        handedness,
    )
    .map_err(|e| degenerate(kind, format!("conic axes in the plane: {e:?}")))
}

/// A NURBS with its control points in plane coordinates: the projection
/// is affine, so knots and weights carry over and the parameter is the
/// same.
fn nurbs_in_plane(c: &NurbsCurve, plane: &Frame) -> Result<NurbsCurve2, GeomError> {
    NurbsCurve2::new(
        c.degree(),
        c.knots().to_vec(),
        c.control_points()
            .iter()
            .map(|p| in_plane(plane, *p))
            .collect(),
        c.weights().to_vec(),
    )
}

/// The angle in `[0, π/2]` between the lines carried by two vectors.
fn line_angle(a: &Vec3, b: &Vec3) -> f64 {
    a.cross(b).norm().atan2(a.dot(b).abs())
}

fn on_cylinder(
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
            if d.x.hypot(d.y).atan2(d.z.abs()) <= tol.angular {
                // A ruling: constant `u`, `v` running with `t` along ±Z.
                let q = cyl.to_local(origin);
                return Ok(Curve2::Line {
                    origin: Point2::new(wrap_turn(q.y.atan2(q.x)), q.z),
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
                && centre.x.hypot(centre.y) <= tol.linear
                && (r - radius).abs() <= tol.linear;
            if parallel {
                // Constant `v`; `u` starts where the circle's `X` sits
                // against the cylinder's and runs with `t` in the sense of
                // the circle's `Z` against the cylinder's.
                let x = cyl.vec_to_local(frame.x().into_inner());
                let u0 = wrap_turn(x.y.atan2(x.x));
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
fn on_elliptic_cylinder(
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
            if d.x.hypot(d.y).atan2(d.z.abs()) > tol.angular {
                return fitted_on(curve, range, surface, tol, meter);
            }
            // A ruling: constant `u` at its section point, which is on
            // the ellipse (checked above) and so has a unique parameter
            // unless the ellipse is degenerate to the tolerance.
            let q = cyl.to_local(origin);
            let noise = origin.coords.norm() + cyl.origin().coords.norm();
            let (u, _) = ellipse_nearest(a, b, q.x, q.y, noise).map_err(|locus| {
                degenerate(
                    kind,
                    format!("the ruling's section point is on {locus} of the ellipse"),
                )
            })?;
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
            let section = centre.x.hypot(centre.y) <= tol.linear
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
fn cone_distance(cone: &Frame, radius: f64, half_angle: f64, p: arris_math::Point3) -> f64 {
    let q = cone.to_local(p);
    let (rho, z) = (q.x.hypot(q.y), q.z);
    let (sin, cos) = (half_angle.sin(), half_angle.cos());
    let near = ((rho - radius) * cos - z * sin).abs();
    let far = ((rho + radius) * cos + z * sin).abs();
    near.min(far)
}

/// `true` when two axes are parallel — either way round — within `tol`.
fn parallel_axes(a: &Vec3, b: &Vec3, tol: Tolerance) -> bool {
    line_angle(a, b) <= tol.angular
}

/// `true` when two axes are perpendicular within `tol`.
fn perpendicular_axes(a: &Vec3, b: &Vec3, tol: Tolerance) -> bool {
    (FRAC_PI_2 - line_angle(a, b)).abs() <= tol.angular
}

/// The pcurve of a circle about a surface of revolution's axis: a `Line`
/// at constant `v`, `u` starting at the offset of the circle's `X` from
/// the surface's and running in the sense of the circle's `Z` against the
/// surface's, exactly as on a cylinder. `flipped` is for a cone's circle
/// beyond the apex, whose radial factor `R + v sin α` is negative: the
/// surface reaches it at `u + π`.
fn parallel_pcurve(surface: &Frame, circle: &Frame, v: f64, flipped: bool) -> Curve2 {
    let x = surface.vec_to_local(circle.x().into_inner());
    let half_turn = if flipped { TAU / 2.0 } else { 0.0 };
    let u0 = wrap_turn(x.y.atan2(x.x) + half_turn);
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
fn meridian_pcurve(
    surface: &Frame,
    circle: &Frame,
    radial: Vec3,
    within: Option<Interval>,
) -> Curve2 {
    let u = wrap_turn(radial.y.atan2(radial.x));
    let x = surface.vec_to_local(circle.x().into_inner());
    let y = surface.vec_to_local(circle.y().into_inner());
    let (a, b) = (x.dot(&radial), x.z);
    let (c, d) = (y.dot(&radial), y.z);
    let phi = b.atan2(a);
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
fn meridian_radial(
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
fn on_cone(
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
    let (sin, cos) = (half_angle.sin(), half_angle.cos());
    if is_negligible(sin, 1.0) || cos <= 0.0 {
        return Err(degenerate(
            GeomKind::Surface(surface.kind()),
            format!("a cone's half-angle is in (0, π/2), not {half_angle}"),
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
            if (equatorial.norm().atan2(d.z.abs()) - half_angle).abs() > tol.angular {
                return fitted_on(curve, range, surface, tol, meter);
            }
            if is_negligible(equatorial.norm(), 1.0) {
                return Err(degenerate(kind, "the ruling has no radial direction"));
            }
            let u = wrap_turn(equatorial.y.atan2(equatorial.x));
            let v0 = cone.to_local(origin).z / cos;
            Ok(Curve2::Line {
                origin: Point2::new(u, v0),
                direction: UnitVec2::new_unchecked(Vec2::new(0.0, sense)),
            })
        }
        Curve::Circle { frame, .. } => {
            let centre = cone.to_local(frame.origin());
            let about_axis =
                centre.x.hypot(centre.y) <= tol.linear && parallel_axes(&frame.z(), &cone.z(), tol);
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
fn on_sphere(
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
            if centre.x.hypot(centre.y) <= tol.linear && parallel_axes(&frame.z(), &sphere.z(), tol)
            {
                // A parallel: `v` is the latitude of its plane.
                return Ok(parallel_pcurve(sphere, frame, centre.z.atan2(rho), false));
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
fn on_torus(
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
                let v = wrap_turn(centre.z.atan2(rho - major_radius));
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

/// An end of the range that is on a singular point of the surface.
#[derive(Debug, Clone, Copy)]
struct SingularEnd {
    point: Point3,
    /// By how much the curve's end misses the point, at most the band.
    miss: Vec3,
    /// The pcurve's end: the free parameter the curve arrives with, the
    /// point's own value of the fixed one.
    uv: Point2,
    /// The parameter beyond which the curve is left where it is.
    exit: f64,
}

/// The parameter in `[a, b]` at which `curve` is nearest `point`, and the
/// distance there, by golden section: the bracket is one the sampling
/// found a single dip in.
fn nearest_approach(curve: &Curve, point: Point3, mut a: f64, mut b: f64) -> (f64, f64) {
    let ratio = 0.5 * (5f64.sqrt() - 1.0);
    let d = |t: f64| (curve.point(t) - point).norm();
    let (mut x1, mut x2) = (b - ratio * (b - a), a + ratio * (b - a));
    let (mut d1, mut d2) = (d(x1), d(x2));
    for _ in 0..GOLDEN_STEPS {
        if d1 <= d2 {
            (b, x2, d2) = (x2, x1, d1);
            x1 = b - ratio * (b - a);
            d1 = d(x1);
        } else {
            (a, x1, d1) = (x1, x2, d2);
            x2 = a + ratio * (b - a);
            d2 = d(x2);
        }
    }
    // An end of the bracket may be nearer than anything inside it.
    [(a, d(a)), (b, d(b)), (x1, d1), (x2, d2)]
        .into_iter()
        .fold(
            (a, f64::INFINITY),
            |best, c| if c.1 < best.1 { c } else { best },
        )
}

/// Golden-section steps of [`nearest_approach`]: each shrinks the bracket
/// by 0.618, so eighty take one sampling interval below the spacing of
/// `f64` parameters. An iteration count, not a tolerance.
const GOLDEN_STEPS: usize = 80;

/// How many times the interval between two of the [`PCURVE_SAMPLES`] is
/// halved to follow a periodic parameter that swings faster than a
/// quarter turn between them, which is what `u` does beside a pole or an
/// apex: by `π` over a stretch as long as the miss. Thirty-two halvings
/// of a 256th of the range reach a miss of `1e-12` of the curve's length;
/// a step that is still a quarter turn there is a jump, and is refused. A
/// sampling depth, not a tolerance.
const UNWRAP_DEPTH: usize = 32;

/// Where, in units of the band from a singular point a range ends on,
/// the curve is carried onto the point in full and where it is left
/// alone, with a smooth fade between: the end misses the point by at most
/// one unit, so its `u` as seen from the point is wrong by up to a right
/// angle within a few units and by a sixteenth of a radian at the far
/// bound, where the 3D effect of either reading is the same miss. Ratios
/// of the band, not tolerances.
const SINGULAR_FADE: [f64; 2] = [4.0, 16.0];

/// `1` at or below `SINGULAR_FADE[0]`, `0` at or above `SINGULAR_FADE[1]`,
/// the C² quintic step between.
fn fade(x: f64) -> f64 {
    let [near, far] = SINGULAR_FADE;
    let s = ((x - near) / (far - near)).clamp(0.0, 1.0);
    1.0 - s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)
}

/// The ends of `range` that are within the band of a singular point of
/// the surface ([`PCURVE_SINGULAR_BAND`]), or [`GeomError::ThroughSingularity`] when the curve
/// comes that near one anywhere else: the decision is a distance, taken
/// at the nearest approach inside each sampling interval that dips. The
/// parameters of the nearer approaches that stay outside the band come
/// with them, ascending: beside a singular point `u` swings by `π` and
/// back over a stretch as long as the miss, which a sampling coarser than
/// that never sees.
fn singular_ends(
    curve: &Curve,
    range: Interval,
    surface: &Surface,
    tol: Tolerance,
) -> Result<([Option<SingularEnd>; 2], Vec<f64>), GeomError> {
    let kind = GeomKind::Curve(curve.kind());
    let n = PCURVE_SAMPLES;
    let ts: Vec<f64> = (0..=n).map(|i| range.lerp(i as f64 / n as f64)).collect();
    let points: Vec<Point3> = ts.iter().map(|&t| curve.point(t)).collect();
    let band = PCURVE_SINGULAR_BAND * tol.linear;
    let mut ends = [None, None];
    let mut passes = Vec::new();
    for singular in surface.singularities() {
        let d: Vec<f64> = (points.iter())
            .map(|p| (p - singular.point).norm())
            .collect();
        let at_end = [d[0] <= band, d[n] <= band];
        for i in 0..=n {
            let (lo, hi) = (i.saturating_sub(1), (i + 1).min(n));
            let dips = d[i] <= d[lo] && d[i] <= d[hi];
            let reach = band + (points[hi] - points[lo]).norm();
            if !dips || d[i] > reach {
                continue;
            }
            let (t, nearest) = nearest_approach(curve, singular.point, ts[lo], ts[hi]);
            let of_an_end = (at_end[0] && lo == 0) || (at_end[1] && hi == n);
            if nearest <= band && !of_an_end {
                return Err(GeomError::ThroughSingularity {
                    curve: kind,
                    surface: GeomKind::Surface(surface.kind()),
                    t,
                });
            }
            if !of_an_end && range.lo() < t && t < range.hi() {
                passes.push(t);
            }
        }
        for (side, &end) in [0, n].iter().enumerate() {
            if !at_end[side] {
                continue;
            }
            // The free parameter the curve arrives with is its tangent's,
            // read where the tangent leads from the point into the range:
            // as far as the curve's middle on a surface of revolution,
            // where every distance along it reads the same, and on a
            // NURBS just clear of the fade, where the limit is read before
            // the surface bends away from the tangent.
            let into = if side == 0 { 1.0 } else { -1.0 };
            let middle = (curve.point(range.midpoint()) - singular.point).norm();
            let reach = match surface {
                Surface::Nurbs(_) => middle.min(SINGULAR_FADE[1] * band),
                Surface::Plane { .. }
                | Surface::Cylinder { .. }
                | Surface::EllipticCylinder { .. }
                | Surface::Cone { .. }
                | Surface::Sphere { .. }
                | Surface::Torus { .. } => middle,
            };
            let tangent = (into * curve.eval(ts[end]).d1)
                .try_normalize(0.0)
                .filter(|_| reach > 0.0)
                .ok_or_else(|| degenerate(kind, "arrives at a singular point with no tangent"))?;
            let free = surface.project(singular.point + reach * tangent)?.uv[1 - singular.fixed];
            // Carried onto the point until the first sample clear of the
            // fade, or the middle of a range too short to have one.
            let far = SINGULAR_FADE[1] * band;
            let inward: Vec<usize> = if side == 0 {
                (1..=n / 2).collect()
            } else {
                (n / 2..n).rev().collect()
            };
            let exit = (inward.iter().find(|&&i| d[i] >= far)).map_or(range.midpoint(), |&i| ts[i]);
            ends[side] = Some(SingularEnd {
                point: singular.point,
                miss: points[end] - singular.point,
                uv: if singular.fixed == 1 {
                    Point2::new(free, singular.value)
                } else {
                    Point2::new(singular.value, free)
                },
                exit,
            });
        }
    }
    passes.sort_by(f64::total_cmp);
    passes.dedup();
    Ok((ends, passes))
}

/// A NURBS pcurve fitted over the surface's own projection of the curve
/// ([`Surface::project`]), each periodic parameter unwrapped along `t`
/// through a table of [`PCURVE_SAMPLES`] parameters, halved where the
/// parameter swings; the rules by a singular point are [`pcurve_on`]'s.
fn fitted_on(
    curve: &Curve,
    range: Interval,
    surface: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Curve2, GeomError> {
    let kind = GeomKind::Curve(curve.kind());
    let (ends, passes) = singular_ends(curve, range, surface, tol)?;
    let origin = surface.frame().map_or(0.0, |f| f.origin().coords.norm());
    let band = PCURVE_SINGULAR_BAND * tol.linear;
    let raw = |t: f64| -> Result<Point2, GeomError> {
        let mut p = curve.point(t);
        // An end is read on its own side of the range only: a closed curve
        // through a pole, cut there, ends on the one point twice, half a
        // turn of `u` apart.
        let carried = |side: usize, end: &SingularEnd| {
            if side == 0 {
                t <= end.exit
            } else {
                t >= end.exit
            }
        };
        for (side, end) in ends.iter().enumerate() {
            let Some(end) = end else { continue };
            if carried(side, end) {
                p -= fade((p - end.point).norm() / band) * end.miss;
            }
        }
        for (side, end) in ends.iter().enumerate() {
            let Some(end) = end else { continue };
            if carried(side, end) && is_negligible((p - end.point).norm(), p.coords.norm() + origin)
            {
                return Ok(end.uv);
            }
        }
        let uv = surface.project(p)?.uv;
        // A cylinder's `u` as `atan2` gives it rather than wrapped into
        // the turn: the unwrapping puts the turns back either way, and
        // wrapping first costs the last bit of a negative angle, which
        // would move every fitted pcurve a cylinder already carries.
        Ok(match surface {
            Surface::Cylinder { frame, .. } => {
                let q = frame.to_local(p);
                Point2::new(q.y.atan2(q.x), uv.y)
            }
            Surface::Plane { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_) => uv,
        })
    };
    // A turn on the analytic surfaces; a NURBS surface's closure, over
    // which its projection already reports a parameter inside the domain.
    let periods = surface.period();
    // `q` with each periodic parameter moved the short way round from
    // `near`'s, which is how far along the unwrapping it is.
    let beside = |near: [f64; 2], q: Point2| {
        [0, 1].map(|k| match periods[k] {
            Some(period) => near[k] + wrap_signed(q[k] - near[k], period),
            None => q[k],
        })
    };
    let n = PCURVE_SAMPLES;
    let first = raw(range.lo())?;
    let start = [0, 1].map(|k| match (periods[k], surface) {
        (Some(_), Surface::Nurbs(_)) | (None, _) => first[k],
        (Some(_), _) => wrap_turn(first[k]),
    });
    // The unwrapped parameters along `t`, and where in the table each of
    // the `n + 1` even samples is: the halvings lie between them.
    let mut table: Vec<(f64, [f64; 2])> = Vec::with_capacity(n + 1);
    let mut even: Vec<usize> = Vec::with_capacity(n + 1);
    table.push((range.lo(), start));
    even.push(0);
    for i in 1..=n {
        // The interval's end, after the nearest approach to a singular
        // point inside it, each of which is a sample: the table follows
        // the swing of `u` there, which the even samples step over.
        let end = range.lerp(i as f64 / n as f64);
        let before = table[table.len() - 1].0;
        let targets = passes
            .iter()
            .copied()
            .filter(|&t| before < t && t < end)
            .chain([end]);
        for target in targets {
            // Depth-first over the halvings of this step, nearest first.
            let mut pending = vec![(target, 0usize)];
            while let Some(&(t, depth)) = pending.last() {
                let (t0, last) = table[table.len() - 1];
                let next = beside(last, raw(t)?);
                // The swing as a fraction of the turn.
                let swing = (0..2)
                    .filter_map(|k| periods[k].map(|period| (next[k] - last[k]).abs() / period))
                    .fold(0.0, f64::max);
                // A NaN is not below the bound, and is not halved away either.
                if swing < 0.25 {
                    table.push((t, next));
                    pending.pop();
                } else if depth < UNWRAP_DEPTH && swing.is_finite() {
                    // Both halves are one level down: the rest of the
                    // interval is looked at again once the near half is in.
                    if let Some(rest) = pending.last_mut() {
                        rest.1 = depth + 1;
                    }
                    pending.push((0.5 * (t0 + t), depth + 1));
                } else {
                    return Err(degenerate(
                        kind,
                        format!(
                            "winds {swing} of a turn around the axis between two of {n} samples halved {UNWRAP_DEPTH} times: faster than the pcurve sampling resolves"
                        ),
                    ));
                }
            }
        }
        even.push(table.len() - 1);
    }
    let f = |t: f64| {
        // The nearest even sample, and where an interval beside it was
        // halved, the nearest of what the halving put there.
        let s = ((t - range.lo()) / range.length() * n as f64).round();
        let i = (s.max(0.0) as usize).min(n);
        let (from, to) = (even[i.saturating_sub(1)], even[(i + 1).min(n)]);
        let mut near = table[even[i]];
        if to - from > 2 {
            for &entry in &table[from..=to] {
                if (entry.0 - t).abs() < (near.0 - t).abs() {
                    near = entry;
                }
            }
        }
        // On a NURBS surface, whose projection is a search, the table's
        // own neighbour starts a local one: taken where it lands within
        // the band of the curve — what the curve's own point is at most
        // off a surface that holds it — and not beside a singular end,
        // whose carrying `raw` does.
        if let Surface::Nurbs(nurbs) = surface {
            let beside_an_end = ends.iter().enumerate().any(|(side, end)| {
                end.is_some_and(|end| {
                    if side == 0 {
                        t <= end.exit
                    } else {
                        t >= end.exit
                    }
                })
            });
            if !beside_an_end {
                let local = nurbs.project_from(curve.point(t), Point2::new(near.1[0], near.1[1]));
                if local.distance <= band {
                    let [u, v] = beside(near.1, local.uv);
                    return Point2::new(u, v);
                }
            }
        }
        let Ok(q) = raw(t) else {
            return Point2::new(f64::NAN, f64::NAN);
        };
        let [u, v] = beside(near.1, q);
        Point2::new(u, v)
    };
    let deviation = |t: f64, q: Point2| (surface.point(q.x, q.y) - curve.point(t)).norm();
    let fit = fit_curve2(f, range, PCURVE_FIT_DEGREE, deviation, tol.linear, meter)?;
    Ok(Curve2::Nurbs(fit))
}

/// The orthogonal projection of `curve` onto `plane` as a `Curve2` in the
/// plane's (u, v), for a consumer's sketch (`docs/ARCHITECTURE.md`
/// §Facade): a line stays a `Line`, a circle becomes a `Circle` when its
/// plane is parallel and an `Ellipse` otherwise, an ellipse an `Ellipse`,
/// a NURBS a `Nurbs` with its control points projected. This is a
/// point-set projection: the parameter is the variant's own — a line's
/// arc length in the plane, a conic's angle about its projected axes,
/// which differs from the 3D parameter by a phase when the conic is
/// oblique — and only a NURBS, or a curve lying in the plane, keeps the
/// 3D parameter. For the same-parameter pcurve of a curve on the plane
/// use [`pcurve_on`].
///
/// Errors: [`GeomError::Degenerate`] when the projection collapses — a
/// line perpendicular to the plane, a conic in a plane perpendicular to it.
///
/// ```
/// use arris_geom::{Curve, Curve2, project_to_plane};
/// use arris_math::{Frame, Point3, Vec3};
///
/// // A unit circle tilted by 60° about x projects to an ellipse 1 × 0.5.
/// let tilted = Frame::new(Point3::origin(), Vec3::new(0.0, -(3f64.sqrt()) / 2.0, 0.5), Vec3::x()).unwrap();
/// let circle = Curve::Circle { frame: tilted, radius: 1.0 };
/// let Curve2::Ellipse { major_radius, minor_radius, .. } = project_to_plane(&circle, &Frame::world()).unwrap() else { panic!() };
/// assert!((major_radius - 1.0).abs() < 1e-15 && (minor_radius - 0.5).abs() < 1e-15);
/// ```
pub fn project_to_plane(curve: &Curve, plane: &Frame) -> Result<Curve2, GeomError> {
    let kind = GeomKind::Curve(curve.kind());
    match curve {
        &Curve::Line { origin, direction } => {
            let d = in_plane_vec(plane, direction.into_inner());
            if is_negligible(d.norm(), 1.0) {
                return Err(degenerate(
                    kind,
                    "the line is perpendicular to the plane: its projection is a point",
                ));
            }
            Ok(Curve2::Line {
                origin: in_plane(plane, origin),
                direction: UnitVec2::new_normalize(d),
            })
        }
        &Curve::Circle { ref frame, radius } => {
            if is_negligible(frame.z().cross(&plane.z()).norm(), 1.0) {
                return in_plane_curve(curve, plane);
            }
            projected_conic(plane, frame, radius, radius, kind)
        }
        &Curve::Ellipse {
            ref frame,
            major_radius,
            minor_radius,
        } => {
            if is_negligible(frame.z().cross(&plane.z()).norm(), 1.0) {
                return in_plane_curve(curve, plane);
            }
            projected_conic(plane, frame, major_radius, minor_radius, kind)
        }
        Curve::Nurbs(c) => Ok(Curve2::Nurbs(nurbs_in_plane(c, plane)?)),
    }
}

/// The ellipse `o + a cos t·X + b sin t·Y` projected onto `plane`: the
/// image is `o' + M (cos t, sin t)ᵀ` with `M = [a X' | b Y']` the projected
/// axes, and the singular value decomposition of the 2 × 2 `M` gives the
/// image's semi-axes (the singular values) and their directions (the left
/// singular vectors); a negative second singular value is a reflection,
/// which the frame's handedness records.
fn projected_conic(
    plane: &Frame,
    frame: &Frame,
    a: f64,
    b: f64,
    kind: GeomKind,
) -> Result<Curve2, GeomError> {
    let col1 = a * in_plane_vec(plane, frame.x().into_inner());
    let col2 = b * in_plane_vec(plane, frame.y().into_inner());
    let (major, minor_signed, phi) = principal_axes(col1, col2);
    if is_negligible(minor_signed, major) {
        return Err(degenerate(
            kind,
            "the conic's plane is perpendicular to the target: its projection is a segment",
        ));
    }
    let handedness = if minor_signed > 0.0 {
        Handedness::Right
    } else {
        Handedness::Left
    };
    let frame2 = Frame2::new(
        in_plane(plane, frame.origin()),
        Vec2::new(phi.cos(), phi.sin()),
        handedness,
    )
    .map_err(|e| degenerate(kind, format!("projected axes: {e:?}")))?;
    Ok(Curve2::Ellipse {
        frame: frame2,
        major_radius: major,
        minor_radius: minor_signed.abs(),
    })
}

/// The principal axes of the ellipse `M (cos t, sin t)ᵀ`, `M = [col1 |
/// col2]` two conjugate semi-diameters in a plane's (u, v): `(major,
/// minor, φ)` with the semi-axes the singular values of `M`, `φ` the
/// angle of the major axis from `u`, and `minor` signed — negative when
/// `M` is a reflection, so the ellipse is traversed clockwise. A
/// projected conic's, and an oblique plane section's of an elliptic
/// cylinder.
pub(crate) fn principal_axes(col1: Vec2, col2: Vec2) -> (f64, f64, f64) {
    // M = [[p, q], [r, s]] by rows.
    let (p, q, r, s) = (col1.x, col2.x, col1.y, col2.y);
    let (e, f, g, h) = (0.5 * (p + s), 0.5 * (p - s), 0.5 * (r + q), 0.5 * (r - q));
    let (big, small) = (e.hypot(h), f.hypot(g));
    let phi = 0.5 * (h.atan2(e) + g.atan2(f));
    (big + small, big - small, phi)
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
    use arris_math::Precision;

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

    #[test]
    fn projection_degenerates_are_named() {
        let plane = Frame::world();
        let vertical = Curve::Line {
            origin: Point3::origin(),
            direction: Vec3::z_axis(),
        };
        assert!(matches!(
            project_to_plane(&vertical, &plane),
            Err(GeomError::Degenerate { .. })
        ));
        let edge_on = Curve::Circle {
            frame: Frame::from_z(Point3::origin(), Vec3::x()).unwrap(),
            radius: 1.0,
        };
        assert!(matches!(
            project_to_plane(&edge_on, &plane),
            Err(GeomError::Degenerate { .. })
        ));
        let flat = Curve::Circle {
            frame: Frame::from_z(Point3::new(1.0, 2.0, 3.0), -Vec3::z()).unwrap(),
            radius: 1.5,
        };
        let Curve2::Circle { frame, radius } = project_to_plane(&flat, &plane).unwrap() else {
            panic!()
        };
        assert_eq!(radius, 1.5);
        assert!(!frame.is_right_handed());
        assert_eq!(frame.origin(), Point2::new(1.0, 2.0));
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
