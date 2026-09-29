//! A `Curve::Nurbs` against every analytic surface: each span of the
//! curve substituted into the surface's implicit polynomial, in
//! Bernstein form (ADR-0018; `docs/DATA-MODEL.md` §Curves).
//!
//! Every analytic surface is the zero set of a polynomial `F` of degree
//! `d` in its own frame — one for a plane, two for a quadric, four for a
//! torus. A rational span `C(s) = A(s) / w(s)` of degree `p` put into it
//! and cleared of its denominator, `g(s) = w(s)ᵈ F(A(s) / w(s))`, is a
//! polynomial of degree `d·p` with the sign of `F` along the span, the
//! weights being positive. `g` is built from the span's homogeneous
//! Bézier points by products in the Bernstein basis
//! (`crate::bernstein`), never through power coefficients.
//!
//! What is decided on `g` is only *where to look*: the sign changes of
//! `g′` are the extrema of `g`, between two of which `g` is monotone and
//! crosses zero at most once. What is decided there is decided in
//! length, as every other arm of the table decides it: on the signed
//! **distance** `δ(t)` from the curve's point to the surface, the exact
//! one the line arms walk. `g = φ·δ` with `φ > 0` wherever `δ` is small,
//! so near the surface an extremum of `g` is an extremum of `δ` to
//! second order in `δ`, and `g` and `δ` change sign together.
//!
//! `IntCurveSurface`'s polynomial case and `IntAna_IntConicQuad` in the
//! reference tree were read for how a conic is put into a quadric;
//! nothing of either is here — they solve in power coefficients.

use arris_math::{Meter, Tolerance};

use crate::bernstein::{Binomials, derivative, sign_change_candidates};
use crate::by_distance::hits_by_distance;
use crate::implicit::{BERNSTEIN_ROUNDING, Implicit};
use crate::nurbs::BezierSpan;
use crate::{Curve, CurveSurfaceIntersection, GeomError, GeomKind, NurbsCurve, Surface};

/// A rational B-spline curve against an analytic surface.
///
/// The parameters looked at are every distinct knot of the domain and,
/// on every span, every candidate for a sign change of `g′`
/// ([`sign_change_candidates`], against [`BERNSTEIN_ROUNDING`] of the
/// span's magnitude): between two consecutive ones `g` is monotone, and
/// so is the distance, whose sign it carries. The verdict on them is
/// [`hits_by_distance`]'s, shared with the conic arms.
pub(crate) fn spline_surface(
    curve: &Curve,
    spline: &NurbsCurve,
    surface: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let unsupported = || GeomError::Unsupported {
        a: GeomKind::Curve(curve.kind()),
        b: GeomKind::Surface(surface.kind()),
    };
    let implicit = Implicit::of(surface).ok_or_else(unsupported)?;
    let domain = spline.domain();
    let period = spline.period();
    let binomials = Binomials::new(implicit.degree() * spline.degree());

    let mut splits: Vec<f64> = Vec::new();
    for span in spline.bezier_spans() {
        meter.tick()?;
        splits.push(span.lo);
        splits.push(span.hi);
        let width = span.hi - span.lo;
        splits.extend(
            extrema_on(&implicit, &span, &binomials)
                .into_iter()
                .map(|s| span.lo + width * s),
        );
    }
    // A periodic curve's last knot is its first again.
    if let Some(period) = period {
        for t in &mut splits {
            if *t >= domain.hi() {
                *t -= period;
            }
            *t = t.max(domain.lo());
        }
    }
    hits_by_distance(curve, surface, &implicit, splits, tol)
}

/// The candidates for an extremum of `g` on one span, as parameters of
/// the span's own `[0, 1]`.
fn extrema_on(implicit: &Implicit<'_>, span: &BezierSpan<3>, binomials: &Binomials) -> Vec<f64> {
    let origin = implicit.frame.origin().coords;
    let mut coords: [Vec<f64>; 4] = Default::default();
    let (mut reach, mut weight) = (0.0f64, 0.0f64);
    for (a, w) in &span.control {
        // `w·P` in the frame: `w (P − O)` turned into it.
        let local = implicit.frame.vec_to_local(a - *w * origin);
        coords[0].push(local.x);
        coords[1].push(local.y);
        coords[2].push(local.z);
        coords[3].push(*w);
        reach = reach.max(a.norm() + w * origin.norm());
        weight = weight.max(*w);
    }
    let [x, y, z, w] = &coords;
    let g = implicit.along([x, y, z, w], binomials);
    let degree = g.len().saturating_sub(1);
    // A derivative's coefficient is `degree` times a difference of two
    // of `g`'s.
    let floor = BERNSTEIN_ROUNDING * implicit.magnitude(reach, weight) * 2.0 * degree as f64;
    sign_change_candidates(&derivative(&g), floor)
}
