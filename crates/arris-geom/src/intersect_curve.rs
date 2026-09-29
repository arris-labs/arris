//! Curve–surface intersection: the closed-form table, and the arm that
//! sends a NURBS curve to `crate::intersect_spline`.

use core::f64::consts::{FRAC_PI_2, PI, TAU};

use arris_math::roots::{self, RootError};
use arris_math::{
    Frame, Interval, Meter, Point2, Point3, Tolerance, Vec2, Vec3, wrap_angle as wrap_turn,
};

use crate::arc::{quarter_angle, quarter_arc};
use crate::bernstein::{Binomials, derivative, sign_change_candidates};
use crate::by_distance::hits_by_distance;
use crate::conic2::{Conic2, ConicMeet, conic_pair, trig2_roots};
use crate::implicit::{BERNSTEIN_ROUNDING, Implicit};
use crate::intersect::line_angle;
use crate::intersect_spline::spline_surface;
use crate::project::ellipse_distance;
use crate::{Curve, CurveKind, GeomError, GeomKind, Surface};

/// One point where a curve meets a surface.
///
/// `point` is the curve's point at `t`; the surface's point at `uv` is
/// within the linear tolerance of it — to rounding for a transversal hit,
/// where the curve crosses the surface, and within `tol.linear` for a
/// `tangent` one, where the curve's nearest approach to the surface is
/// that close and counts as a touch. `t` lies in the curve's domain: a
/// conic's in `[0, 2π)`, a periodic NURBS curve's in `[knots[p],
/// knots[n])`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveSurfaceHit {
    /// The curve parameter.
    pub t: f64,
    /// The surface parameters of the nearest surface point, from
    /// [`Surface::project`] — at a cone's apex, where that is ambiguous,
    /// `u = 0` and the apex's `v`.
    pub uv: Point2,
    /// `curve.point(t)`.
    pub point: Point3,
    /// `true` when the curve touches the surface here without crossing
    /// it: the signed distance along the curve has an extremum within
    /// `tol.linear` of zero. Two crossings that close are one touch. An
    /// open NURBS curve that only ends within `tol.linear` of the
    /// surface is a hit there and no touch.
    pub tangent: bool,
}

/// What a curve and a surface have in common.
#[derive(Debug, Clone, PartialEq)]
pub enum CurveSurfaceIntersection {
    /// The hits, ascending by `t`; empty when the curve misses the
    /// surface.
    Points(Vec<CurveSurfaceHit>),
    /// The curve lies on the surface within the tolerance; there is no
    /// point to return.
    Coincident,
}

/// The intersection of a curve and a surface, by the case table: every
/// pair with a closed form is computed exactly, a conic against a torus
/// and a NURBS curve against every analytic surface go through the
/// surface's implicit polynomial (ADR-0018), and a NURBS surface is the
/// one explicit [`GeomError::Unsupported`] arm — no wildcard, no
/// marcher.
///
/// Guarantees: hits are sorted by `t`, each `t` is in the curve's domain,
/// each hit's `uv` is the surface's own projection of the point — but at
/// a cone's apex, whose projection is ambiguous, `u = 0` and the apex's
/// `v`, as a sphere's pole takes `u = 0` from its projection — the
/// result is deterministic bit for bit, and a transversal hit lies on
/// both operands to rounding. The decisions: a line is parallel to a
/// plane or a cylinder's axis within `tol.angular`, and then coincident
/// or not within `tol.linear`; a circle is coincident when it lies within
/// `tol.linear` of the surface everywhere, which its extrema of distance
/// decide; a hit is `tangent` where the distance along the curve has an
/// extremum within `tol.linear` of zero, and the crossings that extremum
/// would split into are reported as that one touch; a conic's hit within
/// its own rounding of a whole turn is reported at `0`, not at `2π`'s
/// neighbourhood, no tolerance.
///
/// The table: line–plane is one hit, none (parallel), or `Coincident`;
/// line–cylinder is two hits, one tangent hit, none, or `Coincident` for
/// a ruling; conic–plane is two hits, one tangent hit, none, or
/// `Coincident`; conic–cylinder is up to four hits, found as the sign
/// changes of the radial distance between its extrema — the quartic of
/// [`arris_math::roots`] in the half-angle `tan(t/2)` locates the
/// extrema, bracketed Newton the crossings — with `Coincident` for a
/// parallel of the cylinder. *Conic* is a circle or an ellipse: the two
/// differ only in the reach along the frame's two axes, and neither
/// closed form assumes they are equal, so an oblique section ellipse is
/// tested against a third face by the same arms. A line against a cone, a
/// sphere or a torus is what a containment ray casts at their faces:
/// line–sphere is two hits, one tangent hit or none, by the nearest
/// approach to the centre; line–cone is up to two hits over both nappes,
/// one tangent hit, or `Coincident` for a ruling — a line parallel to a
/// ruling meets it once or not at all, and a line through the apex
/// touches it there; line–torus is up to four hits. The cone and the
/// torus are walked by the exact distance along the line, split at its
/// extrema and kinks — for the torus the quartic of [`arris_math::roots`]
/// — with a touch at an extremum within `tol.linear` and each crossing
/// between them polished by bracketed Newton on the distance. Read
/// `IntAna_IntConicQuad` and `IntAna_IntLinTorus` in the reference tree
/// for the case analysis, reimplemented on our frames. A line against an
/// **elliptic cylinder** (ADR-0014) is the quadratic in the frame that
/// scales the section to a circle: two hits, or `Coincident` for a
/// ruling, with the touch decided in length — the section reaches
/// `M = √((a n·X)² + (b n·Y)²)` along the line's normal `n` in the
/// section, and a line whose offset from the centre is within
/// `tol.linear` of `M` touches at that reach. A conic whose plane is
/// across the axis within `tol.angular` meets the elliptic cylinder where
/// it meets the section, two conics in one plane by the quartic
/// (`crate::conic2`): `Coincident`, or the touches and crossings as
/// hits.
///
/// A **conic against a cone, a sphere, or an elliptic cylinder in any
/// other plane** is the quadric's polynomial along the conic, which is a
/// trigonometric polynomial of degree two: between two extrema of it —
/// the roots of its derivative, the same quartic in `tan(t/2)` — it is
/// monotone, and so is the signed distance, whose sign it carries. The
/// distance's kinks go in beside them, a cone's apex plane and its axis
/// as the line arm has them, and the extrema of the distance from the
/// axis besides, which are where a conic concentric with and similar to
/// the surface's section — the one shape whose polynomial is constant
/// along it — is nearest the surface. What is decided there is decided
/// on the distance, as in the NURBS arm below: every stop within
/// `tol.linear` is `Coincident` — a parallel of a cone or a sphere is,
/// and so is an oblique section lying on an elliptic cylinder — a stop
/// within it is one touch absorbing the crossings beside it, and each
/// other stretch whose ends differ in sign holds one crossing. A conic
/// through a cone's apex touches it there, the apex being an extremum of
/// the distance whose value is zero, and takes the apex's `uv`.
///
/// A **conic against a torus** has no trigonometric polynomial of degree
/// two to solve, the torus's implicit form being quartic: the conic goes
/// in as four rational quadratic quarter arcs, and each one put into the
/// polynomial is a polynomial of degree eight in Bernstein form with the
/// torus's sign along it — the substitution and the isolation the NURBS
/// arm below makes of a span, over a conic's exact quarters, with the
/// quarters' own ends beside the sign changes of the derivative. The
/// verdict is the same one: `Coincident` for a conic lying on the torus
/// — a parallel, a tube circle, a Villarceau circle — one `tangent` hit
/// at a stop within `tol.linear`, and up to eight crossings, each hit's
/// `t` the conic's own angle.
///
/// A **NURBS curve** against a plane, a cylinder, an elliptic cylinder, a
/// cone, a sphere or a torus: each span of the curve put into the
/// surface's implicit polynomial is a polynomial in Bernstein form, of
/// the span's degree times one, two or — for the torus — four. The sign
/// changes of its derivative, isolated by subdivision on the variation of
/// the coefficients' signs and polished in the bracket, are where the
/// distance is looked at, with the curve's knots and, for a curve that
/// is not periodic, its two ends; between two of them the polynomial
/// crosses zero at most once. What is decided is decided on the exact
/// signed distance, as above: an extremum of it within `tol.linear` is
/// one `tangent` hit that absorbs the crossings beside it, every one
/// within it is `Coincident` — a fitted section curve is, with both of
/// its surfaces — and each other stretch whose ends differ in sign holds
/// one crossing, by bracketed Newton on the distance. An open curve that
/// only *ends* within `tol.linear` of the surface is a hit at that end
/// and not `tangent`: a section edge ending on a face, not a graze. A
/// closed curve that is not periodic reads its two ends as one parameter,
/// the start, so a hit at the join is one hit there.
/// Any curve against a **NURBS surface** is `Unsupported`, the one arm
/// of the table without a form.
///
/// ```
/// use arris_geom::{Curve, CurveSurfaceIntersection, Surface, intersect_curve_surface};
/// use arris_math::{Frame, Meter, Point3, Precision, Vec3};
///
/// let wall = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
/// let ray = Curve::Line { origin: Point3::new(0.0, 0.0, 1.0), direction: Vec3::x_axis() };
/// let hit = intersect_curve_surface(&ray, &wall, Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
/// let CurveSurfaceIntersection::Points(hits) = hit else { panic!() };
/// assert_eq!(hits.len(), 2);
/// assert!((hits[0].t + 2.0).abs() < 1e-15 && (hits[1].t - 2.0).abs() < 1e-15);
/// assert!(!hits[0].tangent);
/// assert_eq!(hits[1].uv.y, 1.0);
/// ```
pub fn intersect_curve_surface(
    curve: &Curve,
    surface: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<CurveSurfaceIntersection, GeomError> {
    if !tol.is_consistent() {
        return Err(GeomError::InvalidTolerance(tol));
    }
    match (curve, surface) {
        (Curve::Line { origin, direction }, Surface::Plane { frame }) => {
            line_plane(curve, surface, *origin, direction.into_inner(), frame, tol)
        }
        (Curve::Line { origin, direction }, Surface::Cylinder { frame, radius }) => line_cylinder(
            curve,
            surface,
            *origin,
            direction.into_inner(),
            frame,
            *radius,
            tol,
        ),
        (Curve::Circle { frame: cf, radius }, Surface::Plane { frame }) => {
            conic_plane(curve, surface, cf, [*radius, *radius], frame, tol)
        }
        (
            Curve::Ellipse {
                frame: cf,
                major_radius,
                minor_radius,
            },
            Surface::Plane { frame },
        ) => conic_plane(
            curve,
            surface,
            cf,
            [*major_radius, *minor_radius],
            frame,
            tol,
        ),
        (Curve::Circle { frame: cf, radius }, Surface::Cylinder { frame, radius: big }) => {
            conic_cylinder(curve, surface, cf, [*radius, *radius], frame, *big, tol)
        }
        (
            Curve::Ellipse {
                frame: cf,
                major_radius,
                minor_radius,
            },
            Surface::Cylinder { frame, radius: big },
        ) => conic_cylinder(
            curve,
            surface,
            cf,
            [*major_radius, *minor_radius],
            frame,
            *big,
            tol,
        ),
        (Curve::Line { origin, direction }, Surface::Sphere { frame, radius }) => line_sphere(
            curve,
            surface,
            *origin,
            direction.into_inner(),
            frame,
            *radius,
            tol,
        ),
        (
            Curve::Line { origin, direction },
            Surface::Cone {
                frame,
                radius,
                half_angle,
            },
        ) => line_cone(
            curve,
            surface,
            *origin,
            direction.into_inner(),
            frame,
            [*radius, *half_angle],
            tol,
        ),
        (
            Curve::Line { origin, direction },
            Surface::Torus {
                frame,
                major_radius,
                minor_radius,
            },
        ) => line_torus(
            curve,
            surface,
            *origin,
            direction.into_inner(),
            frame,
            [*major_radius, *minor_radius],
            tol,
        ),
        (
            Curve::Line { origin, direction },
            Surface::EllipticCylinder {
                frame,
                major_radius,
                minor_radius,
            },
        ) => line_elliptic_cylinder(
            curve,
            surface,
            *origin,
            direction.into_inner(),
            frame,
            [*major_radius, *minor_radius],
            tol,
        ),
        (
            Curve::Circle { frame: cf, radius },
            Surface::EllipticCylinder {
                frame,
                major_radius,
                minor_radius,
            },
        ) => conic_elliptic_cylinder(
            curve,
            surface,
            cf,
            [*radius, *radius],
            frame,
            [*major_radius, *minor_radius],
            tol,
        ),
        (
            Curve::Ellipse {
                frame: cf,
                major_radius: ca,
                minor_radius: cb,
            },
            Surface::EllipticCylinder {
                frame,
                major_radius,
                minor_radius,
            },
        ) => conic_elliptic_cylinder(
            curve,
            surface,
            cf,
            [*ca, *cb],
            frame,
            [*major_radius, *minor_radius],
            tol,
        ),
        (Curve::Circle { frame: cf, radius }, Surface::Cone { .. } | Surface::Sphere { .. }) => {
            conic_quadric(curve, surface, cf, [*radius, *radius], tol)
        }
        (
            Curve::Ellipse {
                frame: cf,
                major_radius,
                minor_radius,
            },
            Surface::Cone { .. } | Surface::Sphere { .. },
        ) => conic_quadric(curve, surface, cf, [*major_radius, *minor_radius], tol),
        (Curve::Circle { frame: cf, radius }, Surface::Torus { .. }) => {
            conic_torus(curve, surface, cf, [*radius, *radius], tol)
        }
        (
            Curve::Ellipse {
                frame: cf,
                major_radius,
                minor_radius,
            },
            Surface::Torus { .. },
        ) => conic_torus(curve, surface, cf, [*major_radius, *minor_radius], tol),
        (
            Curve::Nurbs(spline),
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. },
        ) => spline_surface(curve, spline, surface, tol, meter),
        (Curve::Line { .. } | Curve::Circle { .. } | Curve::Ellipse { .. }, Surface::Nurbs(_))
        | (Curve::Nurbs(_), Surface::Nurbs(_)) => Err(GeomError::Unsupported {
            a: GeomKind::Curve(curve.kind()),
            b: GeomKind::Surface(surface.kind()),
        }),
    }
}

/// A hit at `t`: the curve's point there, projected onto the surface for
/// its `uv` — except at a cone's apex, where the projection's `u` is
/// ambiguous and the hit takes `u = 0` and the apex's `v`, `−R / sin α`,
/// as a sphere's pole already takes `u = 0` from the projection.
pub(crate) fn hit(
    curve: &Curve,
    surface: &Surface,
    t: f64,
    tangent: bool,
) -> Result<CurveSurfaceHit, GeomError> {
    let point = curve.point(t);
    let uv = match surface.project(point) {
        Ok(projection) => projection.uv,
        Err(e @ GeomError::Ambiguous { .. }) => match *surface {
            Surface::Cone {
                radius, half_angle, ..
            } => Point2::new(0.0, -radius / half_angle.sin()),
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_) => return Err(e),
        },
        Err(e) => return Err(e),
    };
    Ok(CurveSurfaceHit {
        t,
        uv,
        point,
        tangent,
    })
}

/// `hits` sorted by `t`; a total order, since every `t` is finite.
pub(crate) fn points(mut hits: Vec<CurveSurfaceHit>) -> CurveSurfaceIntersection {
    hits.sort_by(|a, b| a.t.total_cmp(&b.t));
    CurveSurfaceIntersection::Points(hits)
}

fn line_plane(
    curve: &Curve,
    surface: &Surface,
    origin: Point3,
    direction: arris_math::Vec3,
    plane: &Frame,
    tol: Tolerance,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let n = plane.z();
    // The angle between the line and the plane: the complement of the
    // angle to the normal, from the sine and cosine of the latter.
    let along = n.dot(&direction);
    let across = n.cross(&direction).norm();
    let height = n.dot(&(origin - plane.origin()));
    if along.abs().atan2(across) <= tol.angular {
        return Ok(if height.abs() <= tol.linear {
            CurveSurfaceIntersection::Coincident
        } else {
            CurveSurfaceIntersection::Points(Vec::new())
        });
    }
    let t = -height / along;
    Ok(points(vec![hit(curve, surface, t, false)?]))
}

fn line_cylinder(
    curve: &Curve,
    surface: &Surface,
    origin: Point3,
    direction: arris_math::Vec3,
    cyl: &Frame,
    radius: f64,
    tol: Tolerance,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let q = cyl.to_local(origin);
    let d = cyl.vec_to_local(direction);
    let (q2, d2) = (Vec2::new(q.x, q.y), Vec2::new(d.x, d.y));
    let across = d2.norm();
    if across.atan2(d.z.abs()) <= tol.angular {
        // Along the axis: a ruling of the cylinder, or clear of it.
        return Ok(if (q2.norm() - radius).abs() <= tol.linear {
            CurveSurfaceIntersection::Coincident
        } else {
            CurveSurfaceIntersection::Points(Vec::new())
        });
    }
    // In the plane across the axis the line passes nearest the axis at
    // `t0`, at distance `dist`; the chord inside the circle of radius `R`
    // is symmetric about it.
    let t0 = -q2.dot(&d2) / (across * across);
    let dist = (q2 + t0 * d2).norm();
    if (dist - radius).abs() <= tol.linear {
        return Ok(points(vec![hit(curve, surface, t0, true)?]));
    }
    if dist >= radius {
        return Ok(CurveSurfaceIntersection::Points(Vec::new()));
    }
    let half = (radius - dist).sqrt() * (radius + dist).sqrt() / across;
    Ok(points(vec![
        hit(curve, surface, t0 - half, false)?,
        hit(curve, surface, t0 + half, false)?,
    ]))
}

/// A line against an elliptic cylinder, `[a, b]` its radii. In the
/// cylinder's frame the line's section direction `e` and normal `n` are
/// read; a line along the axis within `tol.angular` is `Coincident` when
/// its section point is within `tol.linear` of the ellipse and clear
/// otherwise. The ellipse's offset along `n` is `M cos(s − φ)`, `M = √((a
/// n·X)² + (b n·Y)²)`, against the line's own offset `h`: `|h|` within
/// `tol.linear` of `M` is one tangent hit at the reach `s = φ` (or `φ +
/// π`), at the foot of that point on the line; `|h| > M` misses; between,
/// the two crossings of the quadratic `|q' + t d'|² = 1` in the frame
/// scaled by `1 / a` and `1 / b`, where the section is the unit circle.
fn line_elliptic_cylinder(
    curve: &Curve,
    surface: &Surface,
    origin: Point3,
    direction: Vec3,
    cyl: &Frame,
    [a, b]: [f64; 2],
    tol: Tolerance,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let q = cyl.to_local(origin);
    let d = cyl.vec_to_local(direction);
    let (q2, d2) = (Vec2::new(q.x, q.y), Vec2::new(d.x, d.y));
    let across = d2.norm();
    if across.atan2(d.z.abs()) <= tol.angular {
        let noise = origin.coords.norm() + cyl.origin().coords.norm();
        return Ok(if ellipse_distance(a, b, q.x, q.y, noise) <= tol.linear {
            CurveSurfaceIntersection::Coincident
        } else {
            CurveSurfaceIntersection::Points(Vec::new())
        });
    }
    let e = d2 / across;
    let n = Vec2::new(-e.y, e.x);
    let h = n.dot(&q2);
    let reach = (a * n.x).hypot(b * n.y);
    let phase = (b * n.y).atan2(a * n.x);
    if (h.abs() - reach).abs() <= tol.linear {
        let s = if h >= 0.0 { phase } else { phase + PI };
        let (ss, cs) = s.sin_cos();
        let touch = Vec2::new(a * cs, b * ss);
        let t = (touch - q2).dot(&e) / across;
        return Ok(points(vec![hit(curve, surface, t, true)?]));
    }
    if h.abs() >= reach {
        return Ok(CurveSurfaceIntersection::Points(Vec::new()));
    }
    // In the scaled frame the section is the unit circle.
    let qs = Vec2::new(q.x / a, q.y / b);
    let ds = Vec2::new(d.x / a, d.y / b);
    let (aa, bb, cc) = (ds.norm_squared(), qs.dot(&ds), qs.norm_squared() - 1.0);
    let root = (bb * bb - aa * cc).max(0.0).sqrt();
    // The stable pair: the root the subtraction cannot cancel, and the
    // other from the product `t₁ t₂ = C / A`.
    let r = -(bb + root.copysign(bb));
    let (t1, t2) = if r == 0.0 {
        (0.0, 0.0)
    } else {
        (r / aa, cc / r)
    };
    Ok(points(vec![
        hit(curve, surface, t1, false)?,
        hit(curve, surface, t2, false)?,
    ]))
}

/// A circle or an ellipse against an elliptic cylinder, `radii` its
/// `[a, b]` as in [`conic_plane`]. A conic whose plane is across the
/// axis within `tol.angular` meets the surface where it meets the
/// section in that plane (`crate::conic2::conic_pair`), the conic's own
/// parameter kept through its axes projected into the section —
/// `Coincident`, or each touch and crossing a hit; a conic in any other
/// plane goes to [`conic_quadric`], the elliptic cylinder being a
/// quadric like the rest.
fn conic_elliptic_cylinder(
    curve: &Curve,
    surface: &Surface,
    conic: &Frame,
    radii: [f64; 2],
    cyl: &Frame,
    [a, b]: [f64; 2],
    tol: Tolerance,
) -> Result<CurveSurfaceIntersection, GeomError> {
    if line_angle(&conic.z(), &cyl.z()) > tol.angular {
        return conic_quadric(curve, surface, conic, radii, tol);
    }
    let degenerate = |reason: String| GeomError::Degenerate {
        kind: GeomKind::Curve(curve.kind()),
        reason,
    };
    let into_section = |v: Vec3| {
        let w = cyl.vec_to_local(v);
        Vec2::new(w.x, w.y)
            .try_normalize(0.0)
            .ok_or_else(|| degenerate("a conic axis has no part in the section".to_owned()))
    };
    let centre = cyl.to_local(conic.origin());
    let first = Conic2 {
        centre: Point2::new(centre.x, centre.y),
        x: into_section(conic.x().into_inner())?,
        y: into_section(conic.y().into_inner())?,
        a: radii[0],
        b: radii[1],
    };
    let second = Conic2 {
        centre: Point2::origin(),
        x: Vec2::x(),
        y: Vec2::y(),
        a,
        b,
    };
    match conic_pair(&first, &second, tol)
        .map_err(|e| degenerate(format!("the conic against the section: {e}")))?
    {
        ConicMeet::Coincident => Ok(CurveSurfaceIntersection::Coincident),
        ConicMeet::Empty => Ok(CurveSurfaceIntersection::Points(Vec::new())),
        ConicMeet::Meets(meets) => {
            let mut hits = Vec::with_capacity(meets.len());
            for (t, touch) in meets {
                hits.push(hit(curve, surface, t, touch)?);
            }
            Ok(points(hits))
        }
    }
}

/// A line against a sphere: in the sphere's frame the line passes nearest
/// the centre at `t0`, at distance `dist`, and the chord inside the
/// sphere is symmetric about it — the quadratic `|q + t·d|² = R²` in the
/// form whose touch is decided in length, `dist` within `tol.linear` of
/// `R`.
fn line_sphere(
    curve: &Curve,
    surface: &Surface,
    origin: Point3,
    direction: Vec3,
    frame: &Frame,
    radius: f64,
    tol: Tolerance,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let q = frame.to_local(origin).coords;
    let d = frame.vec_to_local(direction);
    let t0 = -q.dot(&d);
    let dist = (q + t0 * d).norm();
    if (dist - radius).abs() <= tol.linear {
        return Ok(points(vec![hit(curve, surface, t0, true)?]));
    }
    if dist >= radius {
        return Ok(CurveSurfaceIntersection::Points(Vec::new()));
    }
    let half = (radius - dist).sqrt() * (radius + dist).sqrt();
    Ok(points(vec![
        hit(curve, surface, t0 - half, false)?,
        hit(curve, surface, t0 + half, false)?,
    ]))
}

/// A line against a cone, both nappes. In the cone's frame, with `p` the
/// line's origin from the apex, `ρ(t)` its distance from the axis and
/// `h(t)` its height above the apex, the signed distance to the cone is
/// `g(t) = ρ cos α − |h| sin α` — the distance to the nearer ruling in the
/// half-plane through the point, exact, negative inside a nappe. A line
/// through the apex at the half-angle to the axis, within `tol.linear`
/// and `tol.angular`, is a ruling: `Coincident`. Otherwise `g` is
/// monotone between its extrema and kinks ([`line_by_distance`]): the
/// point nearest the axis `tv`, where `ρ` has its kink when the line
/// crosses the axis; the crossing of the plane through the apex, where
/// `|h|` has one; and the two stationary points `tv ± |d_z| ρ_min sin α /
/// √(a·k)` of the smooth pieces, with `a = d_x² + d_y²` and `k = a cos²α −
/// d_z² sin²α`, which exist when `k > 0` — the line leaving the double
/// cone. A line parallel to a ruling within `tol.angular` (`k = 0`)
/// crosses the cone once, at the root of the implicit form's linear term,
/// or not at all when it lies within `tol.linear` of the tangent plane
/// along that ruling; a line through the apex touches it there, `g`
/// having its extremum `0` at the apex.
fn line_cone(
    curve: &Curve,
    surface: &Surface,
    origin: Point3,
    direction: Vec3,
    frame: &Frame,
    [radius, half_angle]: [f64; 2],
    tol: Tolerance,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let (sa, ca) = half_angle.sin_cos();
    let q = frame.to_local(origin);
    let p = Vec3::new(q.x, q.y, q.z + radius * ca / sa);
    let d = frame.vec_to_local(direction);
    let a = d.x * d.x + d.y * d.y;
    let to_axis = a.sqrt().atan2(d.z.abs());
    let off_apex = (p - p.dot(&d) * d).norm();
    let parallel = (to_axis - half_angle).abs() <= tol.angular;
    if parallel && off_apex <= tol.linear {
        return Ok(CurveSurfaceIntersection::Coincident);
    }
    let rho = |t: f64| (p.x + t * d.x).hypot(p.y + t * d.y);
    let g = |t: f64| ca * rho(t) - sa * (p.z + t * d.z).abs();
    let dg = |t: f64| {
        let r = rho(t);
        let radial = if r > 0.0 {
            ((p.x + t * d.x) * d.x + (p.y + t * d.y) * d.y) / r
        } else {
            0.0
        };
        ca * radial - sa * d.z * (p.z + t * d.z).signum()
    };
    if parallel {
        // The implicit form `cos²α ρ² − sin²α h²` loses its square term,
        // its other root gone past any length within the angular
        // tolerance: out there `g` only tends to its asymptote, and
        // rounding would flip its sign. The linear term is `2 sin α cos α`
        // times the line's offset from the tangent plane along the parallel
        // ruling, so a line within `tol.linear` of that plane approaches
        // the cone without meeting it, and any other crosses it once, at
        // the one root, polished by Newton on `g`.
        let b = 2.0 * (ca * ca * (p.x * d.x + p.y * d.y) - sa * sa * p.z * d.z);
        let c = ca * ca * (p.x * p.x + p.y * p.y) - sa * sa * p.z * p.z;
        if (b / (2.0 * sa * ca)).abs() <= tol.linear {
            return Ok(CurveSurfaceIntersection::Points(Vec::new()));
        }
        let mut t = -c / b;
        for _ in 0..NEWTON_POLISH_STEPS {
            let (gt, slope) = (g(t), dg(t));
            if gt == 0.0 || slope == 0.0 {
                break;
            }
            let next = t - gt / slope;
            if g(next).abs() < gt.abs() {
                t = next;
            } else {
                break;
            }
        }
        return Ok(points(vec![hit(curve, surface, t, false)?]));
    }
    let mut splits = Vec::with_capacity(4);
    if a > 0.0 {
        let tv = -(p.x * d.x + p.y * d.y) / a;
        splits.push(tv);
        let k = a * ca * ca - d.z * d.z * sa * sa;
        if k > 0.0 {
            let delta = d.z.abs() * rho(tv) * sa / (a * k).sqrt();
            splits.extend([tv - delta, tv + delta]);
        }
    }
    if d.z != 0.0 {
        splits.push(-p.z / d.z);
    }
    line_by_distance(curve, surface, splits, &g, &dg, tol)
}

/// A line against a torus. In the torus's frame, with the line's origin
/// moved to its point nearest the centre (`p · d = 0`), `ρ(s)` its
/// distance from the axis and `z(s)` its height, the signed distance to
/// the torus is `g(s) = √((ρ − R)² + z²) − r`, exact, negative inside the
/// tube. Its extrema are where `(ρ − R)ρ′ + z z′ = 0`, which squared
/// against `ρ = √Q`, `Q = a s² + b s + c` is the quartic
/// `4a s⁴ + 4b s³ + 4(c − R²a²) s² − 4R²ab s − R²b²` of
/// [`arris_math::roots`] — the extrema of the distance to the tube's
/// mirror circle among its roots, which only split a monotone stretch in
/// two, and the kink where the line crosses the axis a double root. The
/// crossings between them are [`line_by_distance`]'s, and a line along the
/// axis, whose quartic vanishes, is clear of a torus with `R > r`.
fn line_torus(
    curve: &Curve,
    surface: &Surface,
    origin: Point3,
    direction: Vec3,
    frame: &Frame,
    [major, minor]: [f64; 2],
    tol: Tolerance,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let q = frame.to_local(origin).coords;
    let d = frame.vec_to_local(direction);
    let shift = -q.dot(&d);
    let p = q + shift * d;
    let a = d.x * d.x + d.y * d.y;
    let b = 2.0 * (p.x * d.x + p.y * d.y);
    let c = p.x * p.x + p.y * p.y;
    let r2 = major * major;
    let found = match roots::quartic(
        4.0 * a,
        4.0 * b,
        4.0 * (c - r2 * a * a),
        -4.0 * r2 * a * b,
        -r2 * b * b,
    ) {
        Ok(found) => found.iter().map(|r| r.value + shift).collect(),
        Err(RootError::Zero) => Vec::new(),
        Err(e) => {
            return Err(GeomError::Degenerate {
                kind: GeomKind::Curve(CurveKind::Line),
                reason: format!("extrema of the distance to a torus: {e}"),
            });
        }
    };
    let rho = |t: f64| (p.x + (t - shift) * d.x).hypot(p.y + (t - shift) * d.y);
    let z = |t: f64| p.z + (t - shift) * d.z;
    let g = |t: f64| (rho(t) - major).hypot(z(t)) - minor;
    let dg = |t: f64| {
        let r = rho(t);
        let tube = (r - major).hypot(z(t));
        if tube == 0.0 {
            return 0.0;
        }
        let radial = if r > 0.0 {
            ((p.x + (t - shift) * d.x) * d.x + (p.y + (t - shift) * d.y) * d.y) / r
        } else {
            0.0
        };
        ((r - major) * radial + z(t) * d.z) / tube
    };
    let mut splits: Vec<f64> = found;
    splits.push(shift);
    if a > 0.0 {
        splits.push(shift - b / (2.0 * a));
    }
    line_by_distance(curve, surface, splits, &g, &dg, tol)
}

/// How far past the first or the last split point a line's distance is
/// first read, and the first step of the doubling search for a crossing
/// in a tail: any positive length serves, since the distance is monotone
/// there, and one length unit reads a unit line's parameter directly.
const TAIL_STEP: f64 = 1.0;

/// The hits of a line on a surface by the signed distance `g` along it
/// (`dg` its derivative), given `splits`: parameters that include every
/// extremum and every kink of `g`, so that it is monotone between two
/// consecutive ones and in each tail beyond them; any others only split a
/// monotone stretch. A split that is no extremum of `g` against its
/// neighbours is dropped. An extremum within `tol.linear` of zero is a
/// `tangent` hit that absorbs the crossings on the stretches beside it,
/// and a run of such extrema with no other between them is one touch;
/// every other stretch whose ends differ in sign holds one crossing, found
/// by bracketed Newton on `g`, and so does a tail whose far side does —
/// searched by doubling steps from its end until the sign turns or `g`
/// stops being finite.
fn line_by_distance(
    curve: &Curve,
    surface: &Surface,
    mut splits: Vec<f64>,
    g: &dyn Fn(f64) -> f64,
    dg: &dyn Fn(f64) -> f64,
    tol: Tolerance,
) -> Result<CurveSurfaceIntersection, GeomError> {
    splits.retain(|t| t.is_finite());
    splits.sort_by(f64::total_cmp);
    splits.dedup();
    let (Some(&first), Some(&last)) = (splits.first(), splits.last()) else {
        return Ok(CurveSurfaceIntersection::Points(Vec::new()));
    };
    let values: Vec<f64> = splits.iter().map(|&t| g(t)).collect();
    let n = splits.len();
    let before = |i: usize| {
        if i == 0 {
            g(first - TAIL_STEP)
        } else {
            values[i - 1]
        }
    };
    let after = |i: usize| {
        if i + 1 == n {
            g(last + TAIL_STEP)
        } else {
            values[i + 1]
        }
    };
    let extrema: Vec<(f64, f64)> = (0..n)
        .filter(|&i| {
            let (v, b, a) = (values[i], before(i), after(i));
            (b >= v && a >= v) || (b <= v && a <= v)
        })
        .map(|i| (splits[i], values[i]))
        .collect();
    // With no extremum `g` is monotone along the whole line: one tail each
    // way from any point holds its one crossing, if it has one.
    let stops: Vec<(f64, f64, bool)> = if extrema.is_empty() {
        vec![(first, values[0], false)]
    } else {
        // A run of adjacent touches is one touch: between two of them `g`
        // is monotone from one value within `tol.linear` to another, so
        // the whole stretch is within it — rounding makes a cluster of
        // extrema of a flat graze. The touch is the one nearest zero.
        let mut stops: Vec<(f64, f64, bool)> = Vec::with_capacity(extrema.len());
        for (t, v) in extrema {
            let stop = (t, v, v.abs() <= tol.linear);
            match stops.last_mut() {
                Some(last) if last.2 && stop.2 => {
                    if stop.1.abs() < last.1.abs() {
                        *last = stop;
                    }
                }
                _ => stops.push(stop),
            }
        }
        stops
    };
    let crossing = |lo: f64, hi: f64| -> Result<f64, GeomError> {
        let bracket = Interval::new(lo, hi).map_err(|_| GeomError::Degenerate {
            kind: GeomKind::Curve(curve.kind()),
            reason: format!("crossing bracket [{lo}, {hi}]"),
        })?;
        roots::newton_in_interval(g, dg, bracket, 0.0).map_err(|e| GeomError::Degenerate {
            kind: GeomKind::Curve(curve.kind()),
            reason: format!("crossing in [{lo}, {hi}]: {e}"),
        })
    };
    let tail = |from: f64, value: f64, sign: f64| -> Option<f64> {
        let mut step = TAIL_STEP;
        while step.is_finite() {
            let t = from + sign * step;
            let v = g(t);
            if !v.is_finite() {
                return None;
            }
            if (v < 0.0) != (value < 0.0) {
                return Some(t);
            }
            step *= 2.0;
        }
        None
    };
    let mut hits = Vec::new();
    let (t0, v0, touch0) = stops[0];
    if !touch0 {
        if let Some(far) = tail(t0, v0, -1.0) {
            hits.push(hit(curve, surface, crossing(far, t0)?, false)?);
        }
    }
    for (i, &(t, v, touch)) in stops.iter().enumerate() {
        if touch {
            hits.push(hit(curve, surface, t, true)?);
        }
        let Some(&(next, w, next_touch)) = stops.get(i + 1) else {
            continue;
        };
        if !touch && !next_touch && (v < 0.0) != (w < 0.0) {
            hits.push(hit(curve, surface, crossing(t, next)?, false)?);
        }
    }
    let (tn, vn, touchn) = stops[stops.len() - 1];
    if !touchn {
        if let Some(far) = tail(tn, vn, 1.0) {
            hits.push(hit(curve, surface, crossing(tn, far)?, false)?);
        }
    }
    Ok(points(hits))
}

/// A circle or an ellipse against a plane. `radii` is `[a, b]`, the reach
/// of the conic along its frame's `X` and `Y` — the radius twice for a
/// circle — which is the only way the two kinds differ here: the signed
/// distance to the plane is `h + a(n·X) cos t + b(n·Y) sin t` for both,
/// one sinusoid whose extrema are its two candidate touches.
fn conic_plane(
    curve: &Curve,
    surface: &Surface,
    conic: &Frame,
    radii: [f64; 2],
    plane: &Frame,
    tol: Tolerance,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let circle = conic;
    // The signed distance along the conic is `h + M cos(t − φ)`: `h` at
    // the centre, `M` the reach of the conic out of the plane.
    let n = plane.z();
    let h = n.dot(&(circle.origin() - plane.origin()));
    let a = radii[0] * n.dot(&circle.x());
    let b = radii[1] * n.dot(&circle.y());
    let reach = a.hypot(b);
    let phase = b.atan2(a);
    // The extrema: `h + M` at `φ`, `h − M` at `φ + π`. Both within the
    // tolerance is coincident; one is a touch.
    let far_touch = (h + reach).abs() <= tol.linear;
    let near_touch = (h - reach).abs() <= tol.linear;
    if far_touch && near_touch {
        return Ok(CurveSurfaceIntersection::Coincident);
    }
    if far_touch {
        return Ok(points(vec![hit(curve, surface, wrap_turn(phase), true)?]));
    }
    if near_touch {
        return Ok(points(vec![hit(
            curve,
            surface,
            wrap_turn(phase + PI),
            true,
        )?]));
    }
    if h.abs() >= reach {
        return Ok(CurveSurfaceIntersection::Points(Vec::new()));
    }
    // `cos(t − φ) = −h / M`, with the sine from the difference of squares
    // so a root near the touch keeps its digits.
    let sine = (reach - h).sqrt() * (reach + h).sqrt();
    let half = sine.atan2(-h);
    Ok(points(vec![
        hit(curve, surface, wrap_turn(phase - half), false)?,
        hit(curve, surface, wrap_turn(phase + half), false)?,
    ]))
}

/// The radial reach of a circle or an ellipse in a cylinder's local
/// frame: the squared distance from the axis along the conic is
/// `ρ²(t) = |p + cos t·X + sin t·Y|²`, with `p` the centre across the
/// axis and `X`, `Y` the axis-scaled frame vectors across it — `a·X` and
/// `b·Y`, which is the radius twice for a circle. Nothing below assumes
/// `|X| = |Y|`, so the ellipse is the same case.
struct Radial {
    p: Vec2,
    x: Vec2,
    y: Vec2,
    radius: f64,
    /// Which conic, for the error an unsolvable extremum names.
    kind: CurveKind,
}

impl Radial {
    /// `ρ(t) − R`, the signed distance from the cylinder along the circle
    /// (negative inside), whose sign changes are the crossings.
    fn distance(&self, t: f64) -> f64 {
        let (st, ct) = t.sin_cos();
        (self.p + ct * self.x + st * self.y).norm() - self.radius
    }

    /// `d(ρ − R)/dt = (P · P′) / ρ`.
    fn slope(&self, t: f64) -> f64 {
        let (st, ct) = t.sin_cos();
        let point = self.p + ct * self.x + st * self.y;
        let rho = point.norm();
        if rho == 0.0 {
            return 0.0;
        }
        point.dot(&(-st * self.x + ct * self.y)) / rho
    }

    /// The critical parameters of `ρ²` in `[0, 2π)`, ascending
    /// ([`radial_critical`]).
    fn critical_parameters(&self) -> Result<Option<Vec<f64>>, GeomError> {
        radial_critical(self.p, self.x, self.y).map_err(|e| GeomError::Degenerate {
            kind: GeomKind::Curve(self.kind),
            reason: format!("radial extrema: {e}"),
        })
    }
}

/// The critical parameters of `ρ²(t) = |p + cos t·x + sin t·y|²` in
/// `[0, 2π)`, ascending: the roots of `P · P′ = a₁ cos t + b₁ sin t +
/// a₂ cos 2t + b₂ sin 2t`, a trigonometric polynomial of degree two,
/// through [`trig2_roots`]. `None` when the polynomial vanishes
/// identically: `ρ` is constant and the conic is a parallel of the axis.
/// Where `ρ` vanishes it has its kink, and that is a minimum of `ρ²`, so
/// these are also the kinks of a distance measured from the axis.
fn radial_critical(p: Vec2, x: Vec2, y: Vec2) -> Result<Option<Vec<f64>>, RootError> {
    let a1 = p.dot(&y);
    let b1 = -p.dot(&x);
    let a2 = x.dot(&y);
    let b2 = 0.5 * (y.norm_squared() - x.norm_squared());
    trig2_roots(a1, b1, a2, b2, 0.0)
}

/// Newton steps that polish a candidate parameter from the half-angle
/// quartic: two suffice from a root accurate to rounding, and each is
/// taken only while it reduces the residual.
const NEWTON_POLISH_STEPS: usize = 3;

/// A circle or an ellipse against a cylinder. `radii` is `[a, b]` as in
/// [`conic_plane`]; the extrema of the radial distance and the crossings
/// between them are [`Radial`]'s, which never assumed a circle.
fn conic_cylinder(
    curve: &Curve,
    surface: &Surface,
    conic: &Frame,
    radii: [f64; 2],
    cyl: &Frame,
    radius: f64,
    tol: Tolerance,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let circle = conic;
    let centre = cyl.to_local(circle.origin());
    let x = cyl.vec_to_local(radii[0] * circle.x().into_inner());
    let y = cyl.vec_to_local(radii[1] * circle.y().into_inner());
    let radial = Radial {
        p: Vec2::new(centre.x, centre.y),
        x: Vec2::new(x.x, x.y),
        y: Vec2::new(y.x, y.y),
        radius,
        kind: curve.kind(),
    };
    let Some(critical) = radial.critical_parameters()? else {
        // Constant distance from the axis: a parallel, on the cylinder
        // or not.
        return Ok(if radial.distance(0.0).abs() <= tol.linear {
            CurveSurfaceIntersection::Coincident
        } else {
            CurveSurfaceIntersection::Points(Vec::new())
        });
    };
    // Between consecutive extrema the distance is monotone, so each arc
    // holds at most one crossing; an extremum within the tolerance is a
    // touch that absorbs the crossings on the arcs beside it, and every
    // extremum within it is the whole circle within it.
    let touches: Vec<bool> = critical
        .iter()
        .map(|&t| radial.distance(t).abs() <= tol.linear)
        .collect();
    if touches.iter().all(|&touch| touch) {
        return Ok(CurveSurfaceIntersection::Coincident);
    }
    let mut hits = Vec::new();
    let n = critical.len();
    for i in 0..n {
        let (lo, lo_touch) = (critical[i], touches[i]);
        let (mut hi, hi_touch) = (critical[(i + 1) % n], touches[(i + 1) % n]);
        if lo_touch {
            hits.push(hit(curve, surface, lo, true)?);
        }
        if lo_touch || hi_touch {
            continue;
        }
        if hi <= lo {
            hi += TAU;
        }
        let (d_lo, d_hi) = (radial.distance(lo), radial.distance(hi));
        if (d_lo < 0.0) == (d_hi < 0.0) {
            continue;
        }
        let Ok(bracket) = Interval::new(lo, hi) else {
            continue;
        };
        let t =
            roots::newton_in_interval(|t| radial.distance(t), |t| radial.slope(t), bracket, 0.0)
                .map_err(|e| GeomError::Degenerate {
                    kind: GeomKind::Curve(radial.kind),
                    reason: format!("crossing in [{lo}, {hi}]: {e}"),
                })?;
        // The last arc wraps past 2π; the root comes back with it.
        hits.push(hit(curve, surface, wrap_turn(t.rem_euclid(TAU)), false)?);
    }
    Ok(points(hits))
}

/// A circle or an ellipse against a torus; `radii` is `[a, b]` as in
/// [`conic_plane`]. A torus's polynomial is quartic, so there is no
/// trigonometric polynomial of degree two for [`trig2_roots`] to solve:
/// the conic goes in as the four rational quadratic quarter arcs of
/// [`crate::arc`] instead, and each one put into the polynomial is a
/// Bernstein polynomial of degree eight with the torus's own sign along
/// it — the substitution and the isolation the NURBS arm makes of a
/// span ([`crate::intersect_spline`]), over a conic's exact quarters.
/// Between two sign changes of its derivative `g` is monotone, and so is
/// the signed distance, whose sign it carries; the quarters' own ends go
/// in beside them, since an extremum exactly at a join is a change of
/// sign neither side sees. The verdict on them is [`hits_by_distance`]'s,
/// as everywhere else: a conic on the torus — a parallel, a tube circle,
/// a Villarceau circle — is `Coincident`, and a conic that leaves and
/// re-enters the tube is up to eight crossings.
fn conic_torus(
    curve: &Curve,
    surface: &Surface,
    conic: &Frame,
    radii: [f64; 2],
    tol: Tolerance,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let implicit = Implicit::of(surface).ok_or_else(|| GeomError::Unsupported {
        a: GeomKind::Curve(curve.kind()),
        b: GeomKind::Surface(surface.kind()),
    })?;
    // A quarter arc is quadratic, so `g` has twice the torus's degree.
    let binomials = Binomials::new(2 * implicit.degree());
    let origin = implicit.frame.origin().coords;
    let centre = conic.origin().coords;
    let (u, v) = (
        radii[0] * conic.x().into_inner(),
        radii[1] * conic.y().into_inner(),
    );
    let mut splits: Vec<f64> = Vec::new();
    for quarter in 0..4 {
        let [cos, sin, w] = quarter_arc(quarter);
        // The arc's homogeneous control points, `w·P` in world, moved
        // into the surface's frame as `w·(P − O)` turned into it.
        let mut coords: [Vec<f64>; 4] = Default::default();
        let (mut reach, mut weight) = (0.0f64, 0.0f64);
        for i in 0..3 {
            let a = w[i] * centre + cos[i] * u + sin[i] * v;
            let local = implicit.frame.vec_to_local(a - w[i] * origin);
            coords[0].push(local.x);
            coords[1].push(local.y);
            coords[2].push(local.z);
            coords[3].push(w[i]);
            reach = reach.max(a.norm() + w[i] * origin.norm());
            weight = weight.max(w[i]);
        }
        let [x, y, z, ww] = &coords;
        let g = implicit.along([x, y, z, ww], &binomials);
        let degree = g.len().saturating_sub(1);
        let floor = BERNSTEIN_ROUNDING * implicit.magnitude(reach, weight) * 2.0 * degree as f64;
        splits.extend(
            sign_change_candidates(&derivative(&g), floor)
                .into_iter()
                .map(|s| wrap_turn(quarter_angle(quarter, s))),
        );
        // The join, exactly, so that no split of a periodic curve falls
        // a rounding outside its domain.
        splits.push(quarter as f64 * FRAC_PI_2);
    }
    hits_by_distance(curve, surface, &implicit, splits, tol)
}

/// A circle or an ellipse against a cone, a sphere, or an elliptic
/// cylinder in a plane that is not across its axis; `radii` is `[a, b]`
/// as in [`conic_plane`]. A quadric's polynomial along a conic is a
/// trigonometric polynomial of degree two ([`Implicit::along_conic`]),
/// and between two extrema of it — the roots of its derivative, through
/// [`trig2_roots`] — it is monotone, so the signed distance, whose sign
/// it carries, crosses zero at most once there. The distance's own kinks
/// go in beside them: a cone's apex plane, where `|h|` turns, and its
/// axis, where `ρ` does — the latter among the extrema of `ρ²`, which
/// are also where a conic whose polynomial is *constant* along it
/// (concentric with and similar to the surface's own section, the one
/// shape whose extrema the derivative cannot give) is nearest the
/// surface and farthest from it. The verdict on them is
/// [`hits_by_distance`]'s, shared with the NURBS arm.
fn conic_quadric(
    curve: &Curve,
    surface: &Surface,
    conic: &Frame,
    radii: [f64; 2],
    tol: Tolerance,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let implicit = Implicit::of(surface).ok_or_else(|| GeomError::Unsupported {
        a: GeomKind::Curve(curve.kind()),
        b: GeomKind::Surface(surface.kind()),
    })?;
    let degenerate = |what: &str, e: RootError| GeomError::Degenerate {
        kind: GeomKind::Curve(curve.kind()),
        reason: format!("{what}: {e}"),
    };
    let centre = conic.origin();
    let (u, v) = (
        radii[0] * conic.x().into_inner(),
        radii[1] * conic.y().into_inner(),
    );
    let Some([a1, b1, a2, b2, _]) = implicit.along_conic(centre, u, v) else {
        // A torus's polynomial is quartic, and no arm of the table sends
        // a conic against one here.
        return Err(GeomError::Unsupported {
            a: GeomKind::Curve(curve.kind()),
            b: GeomKind::Surface(surface.kind()),
        });
    };
    // The derivative of `a₁ cos t + b₁ sin t + a₂ cos 2t + b₂ sin 2t`;
    // `None` when the polynomial is constant along the conic.
    let extrema = trig2_roots(b1, -a1, 2.0 * b2, -2.0 * a2, 0.0)
        .map_err(|e| degenerate("the extrema along the conic", e))?;
    let constant = extrema.is_none();
    let mut splits: Vec<f64> = extrema.into_iter().flatten().collect();
    // The conic in the surface's frame, where `ρ` is the distance from
    // the axis and `h` the height above a cone's apex.
    let q = implicit.frame.to_local(centre);
    let (lu, lv) = (
        implicit.frame.vec_to_local(u),
        implicit.frame.vec_to_local(v),
    );
    let flat = |w: Vec3| Vec2::new(w.x, w.y);
    // A cone's distance has its kink where the conic crosses the axis,
    // which is a minimum of `ρ²`; and where the polynomial is constant
    // there is nothing else to look at, the distance turning where `ρ`
    // does. Nowhere else are these extrema of the distance, and a split
    // that is none can read as one against its neighbour a rounding
    // away — a conic through a sphere's pole passes the axis there.
    if constant || matches!(surface, Surface::Cone { .. }) {
        if let Some(radial) = radial_critical(flat(q.coords), flat(lu), flat(lv))
            .map_err(|e| degenerate("the radial extrema along the conic", e))?
        {
            splits.extend(radial);
        }
    }
    if let Surface::Cone {
        radius, half_angle, ..
    } = *surface
    {
        let above = q.z + radius * half_angle.cos() / half_angle.sin();
        if let Some(crossings) = trig2_roots(lu.z, lv.z, 0.0, 0.0, above)
            .map_err(|e| degenerate("the crossings of the apex plane", e))?
        {
            splits.extend(crossings);
        }
    }
    if splits.is_empty() {
        // Neither the polynomial nor the distance from the axis turns
        // anywhere: the conic is a parallel of the surface's axis, on it
        // or clear of it, which one parameter decides.
        splits.push(0.0);
    }
    hits_by_distance(curve, surface, &implicit, splits, tol)
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::{Precision, Vec3};

    fn tol() -> Tolerance {
        Precision::DEFAULT.tolerance()
    }

    #[test]
    fn a_line_in_the_plane_is_coincident_and_a_lifted_one_misses() {
        let plane = Surface::Plane {
            frame: Frame::world(),
        };
        let flat = Curve::Line {
            origin: Point3::new(1.0, 2.0, 0.0),
            direction: Vec3::x_axis(),
        };
        let lifted = Curve::Line {
            origin: Point3::new(1.0, 2.0, 3.0),
            direction: Vec3::x_axis(),
        };
        assert_eq!(
            intersect_curve_surface(&flat, &plane, tol(), &mut Meter::default()).unwrap(),
            CurveSurfaceIntersection::Coincident
        );
        assert_eq!(
            intersect_curve_surface(&lifted, &plane, tol(), &mut Meter::default()).unwrap(),
            CurveSurfaceIntersection::Points(Vec::new())
        );
    }

    #[test]
    fn a_parallel_of_the_cylinder_is_coincident() {
        let wall = Surface::Cylinder {
            frame: Frame::world(),
            radius: 2.0,
        };
        let ring = Curve::Circle {
            frame: Frame::from_z(Point3::new(0.0, 0.0, 3.0), Vec3::z()).unwrap(),
            radius: 2.0,
        };
        assert_eq!(
            intersect_curve_surface(&ring, &wall, tol(), &mut Meter::default()).unwrap(),
            CurveSurfaceIntersection::Coincident
        );
        let inner = Curve::Circle {
            frame: Frame::world(),
            radius: 1.0,
        };
        assert_eq!(
            intersect_curve_surface(&inner, &wall, tol(), &mut Meter::default()).unwrap(),
            CurveSurfaceIntersection::Points(Vec::new())
        );
    }

    #[test]
    fn a_meridional_circle_crosses_four_times() {
        let wall = Surface::Cylinder {
            frame: Frame::world(),
            radius: 1.0,
        };
        let circle = Curve::Circle {
            frame: Frame::from_z(Point3::origin(), Vec3::y()).unwrap(),
            radius: 2.0,
        };
        let CurveSurfaceIntersection::Points(hits) =
            intersect_curve_surface(&circle, &wall, tol(), &mut Meter::default()).unwrap()
        else {
            panic!()
        };
        assert_eq!(hits.len(), 4);
        for h in &hits {
            assert!((h.point.x.hypot(h.point.y) - 1.0).abs() < 1e-14);
            assert!(!h.tangent);
        }
        assert!(hits.windows(2).all(|w| w[0].t < w[1].t));
    }

    #[test]
    fn a_tangent_a_rounding_short_of_a_whole_turn_still_reports_zero() {
        // A cylinder posed and turned at the model's default scale,
        // tangent to a circle offset by `big − small` along its own `X`:
        // the touch is at the circle's own `t = 0`, where the pose's
        // rounding would otherwise wrap the found root to a few `f64`
        // units short of `2π` instead. Found by search off
        // `intersect_curve_surface` directly.
        use arris_math::nalgebra::UnitQuaternion;

        let rot = UnitQuaternion::from_euler_angles(
            0.1473530425457661,
            -2.5207683519930253,
            2.7501568180617575,
        );
        let base = Frame::from_z(
            Point3::new(38.50274331577241, 23.32493683456147, -11.174177282923182),
            Vec3::z(),
        )
        .unwrap();
        let frame = Frame::new(
            base.origin(),
            rot * base.z().into_inner(),
            rot * base.x().into_inner(),
        )
        .unwrap();
        let (big, small) = (0.11106208302349421, 0.03334964848347397);
        let wall = Surface::Cylinder { frame, radius: big };
        let delta = big - small;
        let phase: f64 = 3.7394069057115535;
        let spin_a = phase.cos() * frame.x().into_inner() + phase.sin() * frame.y().into_inner();
        let circle_frame = Frame::new(
            frame.origin() + delta * spin_a,
            frame.z().into_inner(),
            spin_a,
        )
        .unwrap();
        let circle = Curve::Circle {
            frame: circle_frame,
            radius: small,
        };
        let CurveSurfaceIntersection::Points(hits) =
            intersect_curve_surface(&circle, &wall, tol(), &mut Meter::default()).unwrap()
        else {
            panic!()
        };
        assert_eq!(hits.len(), 1, "{hits:?}");
        // Plainly, not by a turn-periodic metric: the guarantee is exact,
        // not within a tolerance of `0` or of `2π`.
        assert_eq!(hits[0].t, 0.0, "{hits:?}");
        assert!(hits[0].tangent);
    }

    #[test]
    fn a_line_against_an_elliptic_cylinder_crosses_touches_or_misses() {
        let wall = Surface::EllipticCylinder {
            frame: Frame::world(),
            major_radius: 3.0,
            minor_radius: 2.0,
        };
        // Along x through the axis at height 1: ±3.
        let ray = Curve::Line {
            origin: Point3::new(0.0, 0.0, 1.0),
            direction: Vec3::x_axis(),
        };
        let CurveSurfaceIntersection::Points(hits) =
            intersect_curve_surface(&ray, &wall, tol(), &mut Meter::default()).unwrap()
        else {
            panic!()
        };
        assert_eq!(hits.len(), 2);
        assert!((hits[0].t + 3.0).abs() < 1e-14 && (hits[1].t - 3.0).abs() < 1e-14);
        assert!(!hits[0].tangent && hits[1].uv.y == 1.0);
        assert!((hits[1].uv.x).abs() < 1e-12 && (hits[0].uv.x - PI).abs() < 1e-12);
        // Along x at y = 2: the minor vertex, a touch at x = 0.
        let graze = Curve::Line {
            origin: Point3::new(-5.0, 2.0, 0.0),
            direction: Vec3::x_axis(),
        };
        let CurveSurfaceIntersection::Points(hits) =
            intersect_curve_surface(&graze, &wall, tol(), &mut Meter::default()).unwrap()
        else {
            panic!()
        };
        assert_eq!(hits.len(), 1);
        assert!(
            hits[0].tangent && (hits[0].t - 5.0).abs() < 1e-12,
            "{hits:?}"
        );
        // Beyond, nothing; a ruling, coincident.
        let miss = Curve::Line {
            origin: Point3::new(-5.0, 2.5, 0.0),
            direction: Vec3::x_axis(),
        };
        assert_eq!(
            intersect_curve_surface(&miss, &wall, tol(), &mut Meter::default()).unwrap(),
            CurveSurfaceIntersection::Points(Vec::new())
        );
        let ruling = Curve::Line {
            origin: Point3::new(3.0, 0.0, 4.0),
            direction: Vec3::z_axis(),
        };
        assert_eq!(
            intersect_curve_surface(&ruling, &wall, tol(), &mut Meter::default()).unwrap(),
            CurveSurfaceIntersection::Coincident
        );
        // A diagonal chord, both hits on the surface.
        let chord = Curve::Line {
            origin: Point3::new(1.0, 0.5, 0.0),
            direction: arris_math::UnitVec3::new_normalize(Vec3::new(1.0, 1.0, 1.0)),
        };
        let CurveSurfaceIntersection::Points(hits) =
            intersect_curve_surface(&chord, &wall, tol(), &mut Meter::default()).unwrap()
        else {
            panic!()
        };
        assert_eq!(hits.len(), 2);
        for h in &hits {
            let p = h.point;
            assert!(
                ((p.x / 3.0).powi(2) + (p.y / 2.0).powi(2) - 1.0).abs() < 1e-12,
                "{h:?}"
            );
        }
    }

    #[test]
    fn a_conic_across_the_axis_meets_the_section() {
        let wall = Surface::EllipticCylinder {
            frame: Frame::world(),
            major_radius: 3.0,
            minor_radius: 2.0,
        };
        // The section itself, at any height: coincident.
        let section = Curve::Ellipse {
            frame: Frame::from_z(Point3::new(0.0, 0.0, 2.0), Vec3::z()).unwrap(),
            major_radius: 3.0,
            minor_radius: 2.0,
        };
        assert_eq!(
            intersect_curve_surface(&section, &wall, tol(), &mut Meter::default()).unwrap(),
            CurveSurfaceIntersection::Coincident
        );
        // A circle of radius 2.5 about the axis: four crossings.
        let ring = Curve::Circle {
            frame: Frame::world(),
            radius: 2.5,
        };
        let CurveSurfaceIntersection::Points(hits) =
            intersect_curve_surface(&ring, &wall, tol(), &mut Meter::default()).unwrap()
        else {
            panic!()
        };
        assert_eq!(hits.len(), 4);
        assert!(hits.windows(2).all(|w| w[0].t < w[1].t));
        for h in &hits {
            assert!(!h.tangent);
            assert!((h.point.coords.norm() - 2.5).abs() < 1e-12);
        }
        // A circle in a plane through the axis: its distance from the
        // axis reaches 2.5 across the section's minor half-axis of 2, so
        // it crosses the wall four times.
        let tilted = Curve::Circle {
            frame: Frame::from_z(Point3::origin(), Vec3::x()).unwrap(),
            radius: 2.5,
        };
        let CurveSurfaceIntersection::Points(hits) =
            intersect_curve_surface(&tilted, &wall, tol(), &mut Meter::default()).unwrap()
        else {
            panic!()
        };
        assert_eq!(hits.len(), 4, "{hits:?}");
        assert!(hits.windows(2).all(|w| w[0].t < w[1].t));
        for h in &hits {
            assert!(!h.tangent);
            let p = h.point;
            assert!(
                ((p.x / 3.0).powi(2) + (p.y / 2.0).powi(2) - 1.0).abs() < 1e-12,
                "{h:?}"
            );
        }
    }

    #[test]
    fn an_inconsistent_tolerance_is_an_error() {
        let plane = Surface::Plane {
            frame: Frame::world(),
        };
        let line = Curve::Line {
            origin: Point3::origin(),
            direction: Vec3::z_axis(),
        };
        assert!(matches!(
            intersect_curve_surface(
                &line,
                &plane,
                Tolerance::new(1e-7, 0.0),
                &mut Meter::default()
            ),
            Err(GeomError::InvalidTolerance(_))
        ));
    }
}
