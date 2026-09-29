//! A traced section as a `Meets` result: every branch of
//! [`crate::trace_quadrics`] or [`crate::trace_torus`] fitted to a
//! `Curve::Nurbs`, every tube circle of a torus section exact, every
//! singular point a point of the result (ADR-0018, ADR-0019,
//! `docs/DATA-MODEL.md` §Curves).

use arris_math::{Aabb, Meter, Point3, Tolerance};

use crate::{
    Curve, GeomError, MeetCurve, MeetKind, MeetPoint, SectionBranch, Surface, SurfaceIntersection,
    fit_curve, fit_curve_periodic, trace_quadrics, trace_torus,
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
    let torus = |s: &Surface| matches!(s, Surface::Torus { .. });
    let trace = if torus(a) || torus(b) {
        trace_torus(a, b, tol, meter)?
    } else {
        trace_quadrics(a, b, within, tol, meter)?
    };
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
            fitted(branch, tol, meter).map(|curve| MeetCurve {
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

/// One branch as a `Curve::Nurbs`, held to the branch at the branch's
/// own parameter. That distance bounds each surface's too — a
/// projection's distance is 1-Lipschitz, so a point `d` from the branch
/// is no more than `d` further from either surface than the branch is —
/// and it also keeps the fit on the section where the two surfaces meet
/// at a small angle `θ`: a point `ε` off both can be `ε / sin(θ/2)`
/// across from the section, a hundred times `ε` at a degree, which the
/// surfaces alone never see. Against the two surfaces' distances, on
/// the probes of [`SECTION_FIT_DEGREE`]'s doc: at most a third more
/// control points, at the same speed; a curve distance (the nearest
/// point of the branch, found by Newton along it) kept their counts at
/// five to eight times the time. Measured to the branch's stretch, not
/// its point: where a ruling or a tube circle runs a hair from tangent to
/// the other surface, the root on it steps along the section by up to
/// `3·10⁻⁶` from one float of the walked angle to the next, on both
/// surfaces all the while, and a fit held to the point ran out of spans
/// on two rods a quarter of a tolerance off parallel.
fn fitted(branch: &SectionBranch, tol: Tolerance, meter: &mut Meter) -> Result<Curve, GeomError> {
    let deviation = |t: f64, q: Point3| branch.distance(t, q);
    let f = |t: f64| branch.point(t);
    let target = SECTION_FIT_FRACTION * tol.linear;
    let fit = if branch.is_closed() {
        fit_curve_periodic(
            f,
            branch.domain(),
            SECTION_FIT_DEGREE,
            deviation,
            target,
            meter,
        )
    } else {
        fit_curve(
            f,
            branch.domain(),
            SECTION_FIT_DEGREE,
            deviation,
            target,
            meter,
        )
    }?;
    Ok(Curve::Nurbs(fit))
}
