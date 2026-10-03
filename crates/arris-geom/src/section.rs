//! A traced section as a `Meets` result: every branch of
//! [`crate::trace_quadrics`] or [`crate::trace_torus`] fitted to a
//! `Curve::Nurbs`, every tube circle of a torus section exact, every
//! singular point a point of the result (ADR-0018, ADR-0019,
//! `docs/DATA-MODEL.md` §Curves).

use arris_math::{Aabb, Interval, Meter, Point3, Tolerance};

use crate::{
    Curve, GeomError, MeetCurve, MeetKind, MeetPoint, SectionBranch, SectionTrace, Surface,
    SurfaceIntersection, fit_curve, fit_curve_periodic, trace_quadrics, trace_torus,
};

/// The fraction of the pair's `tol.linear` a fitted section curve is
/// held to, measured from the exact branch it fits at the branch's own
/// parameter, as precisely as `f64` knows the branch there
/// ([`crate::SectionBranch::distance`]) — which bounds how much farther
/// from either surface the fit is than the branch, and keeps two fits of
/// one section within twice the fraction of each other (ADR-0022). Each
/// face's pcurve is fitted afterwards to the 3D curve (`pcurve_on`), and
/// can come no nearer the 3D curve than the 3D curve is to that face;
/// that fit is accepted at half its tolerance (the fit's own margin), so
/// the 3D curve has to sit well inside that half for the pcurve to land
/// within `tol.linear` too. A quarter leaves the pcurve the other quarter.
/// That is what keeps a section edge's tolerance at its faces' and never
/// above (`docs/DATA-MODEL.md` §Tolerances). A ratio between two fits,
/// not a tolerance.
pub const SECTION_FIT_FRACTION: f64 = 0.25;

/// The degree of a fitted section curve. Measured with the fit held to
/// the exact branch ([`SECTION_FIT_FRACTION`]) at the default tolerance,
/// in control points per loop at degrees 3, 4, 5 and 6: on metre-scale
/// cylinder pairs (radii 1 to 2, crossing, skew and tilted axes) 234 to
/// 475, 84 to 204, 60 to 137 and 44 to 134; on a unit cylinder against a
/// sphere and against a crossing cylinder of radius 2, at a smallest
/// meeting angle of 20°, 5° and 1°, the quintic takes 117, 133 and 153
/// and 79, 143 and 263, degree 6 fewer against the sphere at every
/// angle but more against the cylinder at 5° and 1° (168 and 296); on
/// metre-scale torus sections (`R/r` from 1.1 to 100, against each of
/// the six analytic kinds) 67 to 401, 36 to 178, 27 to 109 and 22 to 98.
/// Degree 6 saves up to a third on the smooth loops and costs an eighth
/// more on the longest fits, two cylinders at a small angle; the quintic
/// stays, the degree the pcurves fitted to it take
/// ([`crate::PCURVE_FIT_DEGREE`]) (ADR-0019, ADR-0022). A
/// structural choice, not a tolerance.
pub const SECTION_FIT_DEGREE: usize = 5;

/// The section of two surfaces with no closed form, as the intersector
/// returns it: two quadrics one of which is ruled, traced inside
/// `within` ([`trace_quadrics`]), or a pair with a torus in it, traced
/// in the torus's parameter plane with no region at all
/// ([`trace_torus`], ADR-0019).
///
/// Each tube circle of a torus section comes first, in the tracer's
/// order, exact and never fitted — `Touch` where the surfaces do not
/// cross along it, `Crossing` where they do. Then each traced branch, a
/// crossing curve in the tracer's order and orientation, fitted at the
/// branch's own parameter — periodic when the branch is closed — until
/// it is nowhere farther than [`SECTION_FIT_FRACTION`] of `tol.linear`
/// from the exact branch at the same parameter — from the stretch of the
/// line its root was found along that `f64` does not decide
/// ([`SectionBranch::distance`]) — and so no farther than
/// that from either surface beyond the branch's own distance (which is
/// rounding, except within a singular point's reach, where the tracer
/// lets the branch be within `tol.linear` of the other surface). Each
/// singular point is a point of the result, `Touch` where it is isolated
/// and `Crossing` where branches end at it. `Empty` when there is none of
/// the three.
pub(crate) fn traced(
    a: &Surface,
    b: &Surface,
    within: &Aabb,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<SurfaceIntersection, GeomError> {
    let trace = trace_section(a, b, within, tol, meter)?;
    let circles = trace.circles().iter().map(|c| {
        Ok(MeetCurve {
            curve: c.circle.clone(),
            kind: if c.tangent {
                MeetKind::Touch
            } else {
                MeetKind::Crossing
            },
        })
    });
    let curves = circles
        .chain(trace.branches().iter().map(|branch| {
            fit_branch(branch, branch.domain(), tol, meter).map(|curve| MeetCurve {
                curve,
                kind: MeetKind::Crossing,
            })
        }))
        .collect::<Result<Vec<_>, _>>()?;
    let points: Vec<MeetPoint> = trace
        .points()
        .iter()
        .map(|p| MeetPoint {
            point: p.point,
            kind: if p.isolated {
                MeetKind::Touch
            } else {
                MeetKind::Crossing
            },
        })
        .collect();
    Ok(if curves.is_empty() && points.is_empty() {
        SurfaceIntersection::Empty
    } else {
        SurfaceIntersection::Meets { curves, points }
    })
}

/// The section of two surfaces as the intersector traces it where it has
/// no closed form: in the torus's own parameter plane when either is a
/// torus ([`trace_torus`], which ignores `within`), along the rulings of
/// one of two quadrics inside `within` otherwise ([`trace_quadrics`]) —
/// the one dispatch the intersector's sections and a blend's traced end
/// share, so the two are fits of the same branches (ADR-0019, ADR-0037).
///
/// Guarantees: those of the tracer the pair takes, its named refusals
/// ([`GeomError::DegenerateSection`]) among them; a pair neither tracer
/// takes — a plane against a plane, a `Nurbs` — is the tracer's
/// [`GeomError::Unsupported`].
///
/// ```
/// use arris_geom::{Surface, trace_section};
/// use arris_math::{Aabb, Frame, Meter, Point3, Precision, Vec3};
///
/// // A ring torus cut by a plane parallel to its axis, off it: a spiric
/// // section, traced as one loop round the tube's outer side.
/// let torus = Surface::Torus { frame: Frame::world(), major_radius: 3.0, minor_radius: 1.0 };
/// let across = Frame::from_z(Point3::new(3.5, 0.0, 0.0), Vec3::x()).unwrap();
/// let plane = Surface::Plane { frame: across };
/// let within = Aabb { min: [-5.0; 3], max: [5.0; 3] };
/// let tol = Precision::DEFAULT.tolerance();
/// let trace = trace_section(&torus, &plane, &within, tol, &mut Meter::default()).unwrap();
/// assert_eq!(trace.branches().len(), 1);
/// assert!(trace.branches()[0].is_closed());
/// ```
pub fn trace_section(
    a: &Surface,
    b: &Surface,
    within: &Aabb,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<SectionTrace, GeomError> {
    let torus = |s: &Surface| matches!(s, Surface::Torus { .. });
    if torus(a) || torus(b) {
        trace_torus(a, b, tol, meter)
    } else {
        trace_quadrics(a, b, within, tol, meter)
    }
}

/// The stretch `range` of `branch` as a `Curve::Nurbs` of degree
/// [`SECTION_FIT_DEGREE`], fitted at the branch's own parameter until it
/// is nowhere farther than [`SECTION_FIT_FRACTION`] of `tol.linear` from
/// the exact branch at the same parameter ([`SectionBranch::distance`]):
/// the rule every traced section of the intersector is held to, for the
/// whole branch or for the stretch of it a blend's end takes (ADR-0019,
/// ADR-0037).
///
/// Guarantees: the curve's domain is `range` exactly, both ends
/// interpolated; periodic when the branch is closed and `range` is its
/// whole period, open otherwise. On a closed branch `range` may run past
/// the domain's end — the branch wraps — so a stretch across the
/// branch's start is one range. [`GeomError::Fit`] when the fit does not
/// get under the fraction within [`crate::MAX_FIT_SPANS`].
///
/// ```
/// use arris_geom::{Surface, fit_branch, trace_section};
/// use arris_math::{Aabb, Frame, Interval, Meter, Point3, Precision, Vec3};
///
/// let torus = Surface::Torus { frame: Frame::world(), major_radius: 3.0, minor_radius: 1.0 };
/// let across = Frame::from_z(Point3::new(3.5, 0.0, 0.0), Vec3::x()).unwrap();
/// let plane = Surface::Plane { frame: across };
/// let within = Aabb { min: [-5.0; 3], max: [5.0; 3] };
/// let tol = Precision::DEFAULT.tolerance();
/// let mut meter = Meter::default();
/// let trace = trace_section(&torus, &plane, &within, tol, &mut meter).unwrap();
/// let branch = &trace.branches()[0];
/// // A third of the loop, across its start.
/// let len = branch.domain().hi();
/// let range = Interval::new(0.8 * len, 1.133 * len).unwrap();
/// let curve = fit_branch(branch, range, tol, &mut meter).unwrap();
/// let t = 1.1 * len;
/// assert!((curve.point(t) - branch.point(t)).norm() <= 0.25 * tol.linear);
/// ```
pub fn fit_branch(
    branch: &SectionBranch,
    range: Interval,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Curve, GeomError> {
    // Why the fit is held to the branch and not to the two surfaces: a
    // projection's distance is 1-Lipschitz, so a point `d` from the branch
    // is no more than `d` further from either surface than the branch is —
    // and it also keeps the fit on the section where the two surfaces meet
    // at a small angle `θ`: a point `ε` off both can be `ε / sin(θ/2)`
    // across from the section, a hundred times `ε` at a degree, which the
    // surfaces alone never see. Against the two surfaces' distances, on
    // the probes of [`SECTION_FIT_DEGREE`]'s doc: at most a third more
    // control points, at the same speed; a curve distance (the nearest
    // point of the branch, found by Newton along it) kept their counts at
    // five to eight times the time. Measured to the branch's stretch, not
    // its point: where a ruling or a tube circle runs a hair from tangent to
    // the other surface, the root on it steps along the section by up to
    // `3·10⁻⁶` from one float of the walked angle to the next, on both
    // surfaces all the while, and a fit held to the point ran out of spans
    // on two rods a quarter of a tolerance off parallel.
    let deviation = |t: f64, q: Point3| branch.distance(t, q);
    let f = |t: f64| branch.point(t);
    let target = SECTION_FIT_FRACTION * tol.linear;
    let whole = branch.is_closed() && range == branch.domain();
    let fit = if whole {
        fit_curve_periodic(f, range, SECTION_FIT_DEGREE, deviation, target, meter)
    } else {
        fit_curve(f, range, SECTION_FIT_DEGREE, deviation, target, meter)
    }?;
    Ok(Curve::Nurbs(fit))
}
