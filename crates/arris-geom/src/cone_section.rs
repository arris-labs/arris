//! A plane against a cone that shares no axis with it: an exact conic
//! (ADR-0018, `docs/DATA-MODEL.md` §Curves). An ellipse is a
//! `Curve::Ellipse`; a parabola or a hyperbola, which no variant carries,
//! is a rational quadratic `Curve::Nurbs` over the part of it the
//! caller's region can hold, never fitted. A plane through the apex cuts
//! the cone in rulings or at the apex alone.
//!
//! In the plane's own coordinates — `x` along `e₁`, the cone's axis `Z`
//! projected onto the plane, `y` along `e₂ = n × e₁`, the origin the
//! apex's foot `A − h·n` — a point `X` is on the double cone when
//! `((X − A)·Z)² = cos²α |X − A|²`, that is
//! `(x cos φ − h n·Z)² = cos²α (x² + y² + h²)`, `φ` the angle between the
//! axis and the plane. With `k = cos²φ − cos²α = sin(α − φ) sin(α + φ)`,
//! the section is `k (x − x₀)² − cos²α y² = h² cos²α sin²α / k`,
//! `x₀ = h (n·Z) cos φ / k`: an ellipse for `k < 0` (the plane steeper
//! than the cone), a hyperbola for `k > 0`, a parabola for `k = 0`.

use arris_math::real;
use arris_math::{Aabb, Frame, Point3, Tolerance, UnitVec3, Vec3};

use crate::{
    Curve, GeomError, GeomKind, MeetCurve, MeetKind, MeetPoint, NurbsCurve, SurfaceIntersection,
    SurfaceKind,
};

/// The largest half-span, in the hyperbola's own parameter `u` of
/// `(a cosh u, b sinh u)`, that one rational quadratic arc of it covers.
/// The arc's middle weight is `cosh` of the half-span, and a weight far
/// from one crowds the arc's parameter at its ends; a branch longer than
/// this is split into equal arcs joined at knots of multiplicity two. A
/// structural bound, not a tolerance.
pub const HYPERBOLA_HALF_SPAN: f64 = 1.0;

/// A plane against a cone, `(sin α, cos α)` its half-angle's, when the
/// plane is neither perpendicular to the axis nor through it (the
/// meridian arm's): the conic of the module's documentation, each curve
/// a crossing. Through the apex within `tol.linear`: two rulings from the
/// apex ordered along `e₂`, negative first, where the plane is shallower
/// than the cone; one touching ruling along `e₁` where it is as steep
/// within `tol.angular`; the apex alone, a crossing, where it is
/// steeper. Off the apex: an ellipse with `Z` the plane's normal and `X`
/// along `e₁`, the direction of increasing `v`; a parabola as one
/// quadratic Bézier over `y`; a hyperbola as its two branches, the one on
/// the `+e₁` side first, each running with `y` and split into arcs of at
/// most [`HYPERBOLA_HALF_SPAN`]. A parabola's or a hyperbola's branch
/// covers every point of it inside the sphere about `within`, and a
/// branch that sphere misses is left out.
pub(crate) fn plane_cone(
    plane: &Frame,
    cone: &Frame,
    radius: f64,
    half_angle: f64,
    within: &Aabb,
    tol: Tolerance,
) -> Result<SurfaceIntersection, GeomError> {
    let degenerate = |reason: &str| GeomError::Degenerate {
        kind: GeomKind::Surface(SurfaceKind::Cone),
        reason: reason.to_owned(),
    };
    let (sa, ca) = real::sin_cos(half_angle);
    let z: Vec3 = cone.z().into_inner();
    let apex = cone.origin() - (radius * ca / sa) * z;
    let n: Vec3 = plane.z().into_inner();
    let zn = n.dot(&z);
    // `e₁` the axis projected onto the plane, `cos φ` its length; the
    // meridian arm owns the plane perpendicular to the axis, so it is not
    // zero.
    let along = z - zn * n;
    let cos_phi = along.norm();
    let Some(e1) = UnitVec3::try_new(along, 0.0) else {
        return Err(degenerate("a plane perpendicular to the axis"));
    };
    let e1: Vec3 = e1.into_inner();
    let e2 = n.cross(&e1);
    let phi = real::atan2(zn.abs(), cos_phi);
    let h = (apex - plane.origin()).dot(&n);
    let foot = apex - h * n;
    let at = |x: f64, y: f64| foot + x * e1 + y * e2;
    let crossing = |curve| MeetCurve {
        curve,
        kind: MeetKind::Crossing,
    };
    let steepness = half_angle - phi;

    if h.abs() <= tol.linear {
        // Through the apex: the lines `k x² = cos²α y²`.
        let ruling = |direction: Vec3| Curve::Line {
            origin: apex,
            direction: UnitVec3::new_normalize(direction),
        };
        if steepness.abs() <= tol.angular {
            return Ok(SurfaceIntersection::curves_of(
                MeetKind::Touch,
                vec![ruling(e1)],
            ));
        }
        if steepness < 0.0 {
            return Ok(SurfaceIntersection::Meets {
                curves: Vec::new(),
                points: vec![MeetPoint {
                    point: apex,
                    kind: MeetKind::Crossing,
                }],
            });
        }
        let root_k = (real::sin(steepness) * real::sin(half_angle + phi)).sqrt();
        return Ok(SurfaceIntersection::curves_of(
            MeetKind::Crossing,
            vec![ruling(ca * e1 - root_k * e2), ruling(ca * e1 + root_k * e2)],
        ));
    }

    // The sphere about the region: every point of a branch inside it is
    // within `reach` of `centre`.
    let centre = Point3::from(within.center());
    let reach = 0.5 * within.diagonal();

    if steepness.abs() <= tol.angular {
        // A parabola, `cos φ = cos α` and `n·Z = ±sin α`:
        // `x = h (sin²α − cos²α) / (2 (n·Z) cos α) − cos α / (2 h (n·Z)) · y²`.
        // A point `y` from the vertex is at least `|y|` from it.
        let vertex = h * (sa - ca) * (sa + ca) / (2.0 * zn * ca);
        let q = ca / (2.0 * h * zn);
        let span = reach + (at(vertex, 0.0) - centre).norm();
        let point = |y: f64| at(vertex - q * y * y, y);
        let middle = at(vertex + q * span * span, 0.0);
        let curve = NurbsCurve::new(
            2,
            vec![-span, -span, -span, span, span, span],
            vec![point(-span), middle, point(span)],
            vec![1.0; 3],
        )?;
        return Ok(SurfaceIntersection::curves_of(
            MeetKind::Crossing,
            vec![Curve::Nurbs(curve)],
        ));
    }

    let k = real::sin(steepness) * real::sin(half_angle + phi);
    let x0 = h * zn * cos_phi / k;
    // The semi-axis along `e₁`, and the one along `e₂`.
    let ax = (h * ca * sa / k).abs();
    let ay = (h * sa).abs() / k.abs().sqrt();
    if k < 0.0 {
        let frame = Frame::new(at(x0, 0.0), n, e1).map_err(|_| degenerate("non-finite frame"))?;
        return Ok(SurfaceIntersection::curves_of(
            MeetKind::Crossing,
            vec![Curve::Ellipse {
                frame,
                major_radius: ax,
                minor_radius: ay,
            }],
        ));
    }
    // A hyperbola: `x = x₀ ± a cosh u`, `y = b sinh u`; a point of a
    // branch at `u` is at least `a cosh u` from the centre.
    let from_centre = (at(x0, 0.0) - centre).norm();
    let mut curves = Vec::new();
    for side in [1.0, -1.0] {
        let bound = (reach + from_centre) / ax;
        if bound < 1.0 {
            continue;
        }
        let top = real::acosh(bound);
        let arcs = (top / HYPERBOLA_HALF_SPAN).ceil().max(1.0);
        let half = top / arcs;
        let count = arcs as usize;
        let point = |u: f64| at(x0 + side * ax * real::cosh(u), ay * real::sinh(u));
        let mut knots = vec![-top; 3];
        let mut control = vec![point(-top)];
        let mut weights = vec![1.0];
        for i in 0..count {
            let lo = -top + 2.0 * half * i as f64;
            let hi = if i + 1 == count { top } else { lo + 2.0 * half };
            let mid = 0.5 * (lo + hi);
            // The ends' tangents meet at the centre plus the midpoint's
            // offset over `cosh` of the half-span, the arc's weight.
            let w = real::cosh(half);
            control.push(at(
                x0 + side * ax * real::cosh(mid) / w,
                ay * real::sinh(mid) / w,
            ));
            control.push(point(hi));
            weights.extend([w, 1.0]);
            knots.extend(if i + 1 == count {
                [hi; 3].to_vec()
            } else {
                [hi; 2].to_vec()
            });
        }
        curves.push(crossing(Curve::Nurbs(NurbsCurve::new(
            2, knots, control, weights,
        )?)));
    }
    Ok(if curves.is_empty() {
        SurfaceIntersection::Empty
    } else {
        SurfaceIntersection::Meets {
            curves,
            points: Vec::new(),
        }
    })
}
