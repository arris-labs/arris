//! Curve–surface intersections follow the case table in any pose — a
//! line against every analytic surface, and a conic, being a circle or
//! an ellipse, against every one but the torus: the hit count is the
//! constructed case's, every hit lies on both operands, a hit is tangent
//! when the case was built tangent, coincident when the curve was built
//! on the surface, hits are sorted with `t` in the domain, two runs agree
//! bit for bit, and a conic against a torus is the one `Unsupported`
//! analytic pair left. The ellipse arms are what an oblique boolean
//! section edge needs (ADR-0004), and the quadric arms what an operand
//! face's circle needs of the face it is cut by (ADR-0020).

use core::f64::consts::{FRAC_PI_2, PI, TAU};

use arris_debug::prop::geom::{
    circle, curve, cylinder, line, nurbs_curve, nurbs_surface, plane, surface,
};
use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, point_in_box};
use arris_geom::{
    Curve, CurveSurfaceHit, CurveSurfaceIntersection, GeomError, GeomKind, Surface, SurfaceKind,
    intersect_curve_surface,
};
use arris_math::{Frame, Point3, Precision, Tolerance, Vec3};
use proptest::prelude::*;

/// Hit points against both operands, expected parameters in length
/// units, and expected points.
const EXACT: f64 = 1e-12 * DEFAULT_SCALE;
/// Parameters sampled around a circle to count the sign changes of its
/// distance to a cylinder.
const SAMPLES: usize = 3600;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

/// A region every traced section of these tests lies in; the closed
/// forms ignore it.
fn within() -> arris_math::Aabb {
    arris_math::Aabb {
        min: [-100.0; 3],
        max: [100.0; 3],
    }
}

/// The signed distance from `p` to the surface by its implicit form:
/// negative behind a plane, inside a cylinder, a nappe of a cone, a
/// sphere or a torus's tube.
fn signed_distance(s: &Surface, p: Point3) -> f64 {
    let q = s.frame().unwrap().to_local(p);
    let rho = q.x.hypot(q.y);
    match *s {
        Surface::Plane { .. } => q.z,
        Surface::Cylinder { radius, .. } => rho - radius,
        Surface::EllipticCylinder {
            major_radius: a,
            minor_radius: b,
            ..
        } => {
            // Exact through the projection, signed by the implicit form;
            // a point the projection finds ambiguous is deep inside.
            let sign = ((q.x / a).powi(2) + (q.y / b).powi(2) - 1.0).signum();
            sign * s.project(p).map_or(f64::INFINITY, |pr| pr.distance)
        }
        Surface::Cone {
            radius, half_angle, ..
        } => {
            let (sa, ca) = half_angle.sin_cos();
            rho * ca - (q.z + radius * ca / sa).abs() * sa
        }
        Surface::Sphere { radius, .. } => q.coords.norm() - radius,
        Surface::Torus {
            major_radius,
            minor_radius,
            ..
        } => (rho - major_radius).hypot(q.z) - minor_radius,
        Surface::Nurbs(_) => unreachable!("no NURBS pair has a closed form"),
    }
}

/// The distance from `p` to the surface by its implicit form.
fn implicit_distance(s: &Surface, p: Point3) -> f64 {
    signed_distance(s, p).abs()
}

/// `|a − b|` on a circle of period `period`, or plainly when there is
/// none.
fn param_diff(period: Option<f64>, a: f64, b: f64) -> f64 {
    match period {
        Some(p) => {
            let d = (a - b).rem_euclid(p);
            d.min(p - d)
        }
        None => (a - b).abs(),
    }
}

fn fail<T>(msg: String) -> Result<T, TestCaseError> {
    Err(TestCaseError::fail(msg))
}

fn hits_of(r: &CurveSurfaceIntersection) -> &[CurveSurfaceHit] {
    match r {
        CurveSurfaceIntersection::Points(h) => h,
        CurveSurfaceIntersection::Coincident => &[],
    }
}

/// The checks every result passes whatever its case: hits on both
/// operands, sorted with `t` in the domain, `uv` consistent with the
/// point, and determinism.
fn common_properties(c: &Curve, s: &Surface) -> Result<CurveSurfaceIntersection, TestCaseError> {
    let r = intersect_curve_surface(c, s, tol(), &mut arris_math::Meter::default())
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
    let hits = hits_of(&r);
    for h in hits {
        prop_assert!(
            c.domain().contains(h.t),
            "{c:?}: t = {} outside the domain",
            h.t
        );
        if c.period().is_some() {
            prop_assert!(h.t < TAU, "{c:?}: t = {} not in [0, 2π)", h.t);
        }
        prop_assert!((c.point(h.t) - h.point).norm() <= EXACT);
        // Rounding is relative: a crossing a line makes far out, nearly
        // parallel to a cone's ruling, is on the cone to its own digits.
        let exact = EXACT * (h.point.coords.norm() / DEFAULT_SCALE).max(1.0);
        let off = implicit_distance(s, h.point);
        let allowed = if h.tangent {
            tol().linear + exact
        } else {
            exact
        };
        prop_assert!(
            off <= allowed,
            "{c:?} vs {s:?}: hit at t = {} is off the surface by {off} (tangent {})",
            h.t,
            h.tangent
        );
        prop_assert!((s.point(h.uv.x, h.uv.y) - h.point).norm() <= allowed);
    }
    prop_assert!(
        hits.windows(2).all(|w| w[0].t < w[1].t),
        "unsorted: {hits:?}"
    );
    let again = intersect_curve_surface(c, s, tol(), &mut arris_math::Meter::default())
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
    prop_assert_eq!(&again, &r, "two runs differ");
    Ok(r)
}

/// The hits' parameters against the expected ones, in length units of
/// the curve's speed, in any order.
fn expect_hits(
    c: &Curve,
    r: &CurveSurfaceIntersection,
    expected: &[f64],
    tangent: bool,
) -> Result<(), TestCaseError> {
    let hits = hits_of(r);
    prop_assert_eq!(
        hits.len(),
        expected.len(),
        "expected {} hits at {:?}, got {:?}",
        expected.len(),
        expected,
        r
    );
    for e in expected {
        let speed = c.eval(*e).d1.norm();
        let near = hits
            .iter()
            .find(|h| param_diff(c.period(), h.t, *e) * speed <= EXACT);
        let Some(h) = near else {
            return fail(format!("no hit at t = {e} in {hits:?}"));
        };
        prop_assert_eq!(h.tangent, tangent, "{:?}", h);
    }
    Ok(())
}

fn expect_empty(r: &CurveSurfaceIntersection) -> Result<(), TestCaseError> {
    prop_assert_eq!(r, &CurveSurfaceIntersection::Points(Vec::new()));
    Ok(())
}

fn expect_coincident(r: &CurveSurfaceIntersection) -> Result<(), TestCaseError> {
    prop_assert_eq!(r, &CurveSurfaceIntersection::Coincident);
    Ok(())
}

/// The world-space direction at `phase` around a frame's `z`, and the one
/// a quarter turn on.
fn around(f: &Frame, phase: f64) -> (Vec3, Vec3) {
    let around = phase.cos() * f.x().into_inner() + phase.sin() * f.y().into_inner();
    (around, f.z().cross(&around))
}

// --- line–plane -----------------------------------------------------------

#[test]
fn a_line_crossing_a_plane_hits_once_at_the_built_parameter() {
    check(
        (
            plane(),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(0.0..=TAU),
            finite_f64(0.05..=core::f64::consts::FRAC_PI_2),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        ),
        |(p, u, v, phase, tilt, s)| {
            let point = p.point(u, v);
            let (a, _) = around(p.frame().unwrap(), phase);
            let d = tilt.cos() * a + tilt.sin() * p.frame().unwrap().z().into_inner();
            let c = Curve::Line {
                origin: point - s * d,
                direction: arris_math::UnitVec3::new_normalize(d),
            };
            let r = common_properties(&c, &p)?;
            expect_hits(&c, &r, &[s], false)?;
            let [h] = hits_of(&r) else { unreachable!() };
            prop_assert!((h.point - point).norm() <= EXACT);
            Ok(())
        },
    );
}

#[test]
fn a_line_parallel_to_a_plane_is_empty_or_coincident_by_the_gap() {
    check(
        (
            plane(),
            point_in_box(DEFAULT_SCALE),
            finite_f64(0.0..=TAU),
            finite_f64(0.01..=DEFAULT_SCALE),
            any::<bool>(),
        ),
        |(p, anchor, phase, gap, lift)| {
            let n = p.frame().unwrap().z().into_inner();
            let in_plane = anchor - n.dot(&(anchor - p.frame().unwrap().origin())) * n;
            let (a, _) = around(p.frame().unwrap(), phase);
            let c = Curve::Line {
                origin: if lift { in_plane + gap * n } else { in_plane },
                direction: arris_math::UnitVec3::new_normalize(a),
            };
            let r = common_properties(&c, &p)?;
            if lift {
                expect_empty(&r)
            } else {
                expect_coincident(&r)
            }
        },
    );
}

// --- line–cylinder --------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
enum LineCase {
    Two,
    Tangent,
    Empty,
    Ruling,
    ParallelOff,
}

fn line_case() -> impl Strategy<Value = LineCase> {
    prop_oneof![
        Just(LineCase::Two),
        Just(LineCase::Tangent),
        Just(LineCase::Empty),
        Just(LineCase::Ruling),
        Just(LineCase::ParallelOff),
    ]
}

#[test]
fn line_cylinder_follows_the_case_table() {
    check(
        (
            line_case(),
            cylinder(),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(0.0..=TAU),
            finite_f64(0.0..=0.9),
            finite_f64(-1.4..=1.4),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        ),
        |(case, cyl, height, phase, fraction, tilt, s)| {
            let Surface::Cylinder { frame, radius } = cyl else {
                unreachable!()
            };
            let z = frame.z().into_inner();
            let (a, across) = around(&frame, phase);
            let dist = match case {
                LineCase::Two => fraction * radius,
                LineCase::Tangent | LineCase::Ruling => radius,
                LineCase::Empty => radius * (1.1 + fraction),
                LineCase::ParallelOff => radius * (0.5 + fraction),
            };
            let foot = frame.origin() + height * z + dist * a;
            let d = match case {
                LineCase::Ruling | LineCase::ParallelOff => z,
                _ => tilt.cos() * across + tilt.sin() * z,
            };
            let c = Curve::Line {
                origin: foot - s * d,
                direction: arris_math::UnitVec3::new_normalize(d),
            };
            let r = common_properties(&c, &cyl)?;
            match case {
                LineCase::Two => {
                    let half = (radius * radius - dist * dist).sqrt() / tilt.cos();
                    expect_hits(&c, &r, &[s - half, s + half], false)
                }
                LineCase::Tangent => expect_hits(&c, &r, &[s], true),
                LineCase::Empty | LineCase::ParallelOff => expect_empty(&r),
                LineCase::Ruling => expect_coincident(&r),
            }
        },
    );
}

// --- circle–plane ---------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
enum CircleCase {
    Two,
    Tangent,
    Empty,
    Coincident,
    ParallelOff,
}

fn circle_case() -> impl Strategy<Value = CircleCase> {
    prop_oneof![
        Just(CircleCase::Two),
        Just(CircleCase::Tangent),
        Just(CircleCase::Empty),
        Just(CircleCase::Coincident),
        Just(CircleCase::ParallelOff),
    ]
}

#[test]
fn circle_plane_follows_the_case_table() {
    check(
        (
            circle_case(),
            circle(),
            finite_f64(0.0..=TAU),
            finite_f64(0.05..=core::f64::consts::FRAC_PI_2),
            finite_f64(-0.9..=0.9),
            finite_f64(0.01..=DEFAULT_SCALE),
            point_in_box(DEFAULT_SCALE),
            any::<bool>(),
        ),
        |(case, c, phase, tilt, fraction, gap, shift, flip)| {
            let Curve::Circle { frame, radius } = c else {
                unreachable!()
            };
            let z = frame.z().into_inner();
            let (a, _) = around(&frame, phase);
            // The normal tilted off the circle's axis towards `phase`; the
            // circle reaches `M = R sin(tilt)` out of any plane with it.
            let n = match case {
                CircleCase::Coincident | CircleCase::ParallelOff => z,
                _ => tilt.cos() * z + tilt.sin() * a,
            };
            let reach = radius * tilt.sin();
            // The signed height of the centre above the plane.
            let h = match case {
                CircleCase::Two => fraction * reach,
                CircleCase::Tangent => {
                    if flip {
                        -reach
                    } else {
                        reach
                    }
                }
                CircleCase::Empty => reach * (1.1 + fraction.abs()) * if flip { -1.0 } else { 1.0 },
                CircleCase::Coincident => 0.0,
                CircleCase::ParallelOff => gap,
            };
            let anchor = frame.origin() - h * n;
            let in_plane = shift - n.dot(&(shift - anchor)) * n;
            let sign = if flip { -1.0 } else { 1.0 };
            let p = Surface::Plane {
                frame: Frame::from_z(in_plane, sign * n).unwrap(),
            };
            let r = common_properties(&c, &p)?;
            match case {
                CircleCase::Two => {
                    // d(t) = h + M cos(t − phase) vanishes at phase ± half.
                    let half = (-h / reach).acos();
                    expect_hits(&c, &r, &[phase - half, phase + half], false)
                }
                CircleCase::Tangent => {
                    let t = if h < 0.0 { phase } else { phase + PI };
                    expect_hits(&c, &r, &[t], true)
                }
                CircleCase::Empty | CircleCase::ParallelOff => expect_empty(&r),
                CircleCase::Coincident => expect_coincident(&r),
            }
        },
    );
}

// --- circle–cylinder ------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
enum RingCase {
    Coincident,
    Four,
    Two,
    TangentOutside,
    TangentInside,
    EmptyOutside,
    EmptyInside,
}

fn ring_case() -> impl Strategy<Value = RingCase> {
    prop_oneof![
        Just(RingCase::Coincident),
        Just(RingCase::Four),
        Just(RingCase::Two),
        Just(RingCase::TangentOutside),
        Just(RingCase::TangentInside),
        Just(RingCase::EmptyOutside),
        Just(RingCase::EmptyInside),
    ]
}

#[test]
fn circle_cylinder_follows_the_case_table() {
    check(
        (
            ring_case(),
            cylinder(),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(0.0..=TAU),
            finite_f64(0.05..=0.95),
            finite_f64(0.1..=10.0),
            finite_f64(0.0..=TAU),
        ),
        |(case, cyl, height, phase, fraction, small, spin)| {
            let Surface::Cylinder { frame, radius: big } = cyl else {
                unreachable!()
            };
            let z = frame.z().into_inner();
            let (a, across) = around(&frame, phase);
            let on_axis = frame.origin() + height * z;
            // Cases in the plane across the axis need the two radii apart.
            let inside_ok = (small - big).abs() >= 0.05;
            let (c, expected): (Curve, Result<Vec<f64>, bool>) = match case {
                RingCase::Coincident => {
                    let (x, _) = around(&frame, spin);
                    let f = Frame::new(on_axis, z, x).unwrap();
                    (
                        Curve::Circle {
                            frame: f,
                            radius: big,
                        },
                        Err(true),
                    )
                }
                RingCase::Four => {
                    // In the plane through the axis, centred on it, wider
                    // than the cylinder: X = around, Y = −z.
                    let r = big * (1.1 + 2.0 * fraction);
                    let f = Frame::new(on_axis, across, a).unwrap();
                    let half = (big / r).acos();
                    (
                        Curve::Circle {
                            frame: f,
                            radius: r,
                        },
                        Ok(vec![half, PI - half, PI + half, TAU - half]),
                    )
                }
                RingCase::Two
                | RingCase::TangentOutside
                | RingCase::TangentInside
                | RingCase::EmptyOutside
                | RingCase::EmptyInside => {
                    // In the plane across the axis, centre `delta` off it:
                    // two circles of radii `big` and `small`.
                    let (near, far) = ((small - big).abs(), small + big);
                    let delta = match case {
                        RingCase::Two => near + fraction * (far - near),
                        RingCase::TangentOutside => far,
                        RingCase::TangentInside => near,
                        RingCase::EmptyOutside => far * (1.1 + fraction),
                        RingCase::EmptyInside => near * fraction * 0.9,
                        _ => unreachable!(),
                    };
                    let f = Frame::new(on_axis + delta * a, z, a).unwrap();
                    let c = Curve::Circle {
                        frame: f,
                        radius: small,
                    };
                    let expected = match case {
                        RingCase::Two => {
                            let x = (delta * delta + big * big - small * small) / (2.0 * delta);
                            let y = (big * big - x * x).sqrt();
                            let t = y.atan2(x - delta);
                            Ok(vec![t, -t])
                        }
                        RingCase::TangentOutside => Ok(vec![PI]),
                        RingCase::TangentInside => Ok(vec![if small < big { 0.0 } else { PI }]),
                        _ => Err(false),
                    };
                    (c, expected)
                }
            };
            if !inside_ok
                && matches!(
                    case,
                    RingCase::Two | RingCase::TangentInside | RingCase::EmptyInside
                )
            {
                return Ok(());
            }
            let r = common_properties(&c, &cyl)?;
            match expected {
                Err(true) => expect_coincident(&r),
                Err(false) => expect_empty(&r),
                Ok(ts) => {
                    let tangent =
                        matches!(case, RingCase::TangentOutside | RingCase::TangentInside);
                    expect_hits(&c, &r, &ts, tangent)
                }
            }
        },
    );
}

#[test]
fn a_random_circle_and_cylinder_cross_an_even_number_of_times() {
    check((circle(), cylinder()), |(c, cyl)| {
        let r = common_properties(&c, &cyl)?;
        let hits = hits_of(&r);
        let transversal = hits.iter().filter(|h| !h.tangent).count();
        prop_assert_eq!(transversal % 2, 0, "{:?}", r);
        // The sign changes of the radial distance around the circle,
        // sampled finely, are the crossings.
        let d = |t: f64| implicit_signed(&cyl, c.point(t));
        let mut changes = 0;
        let mut prev = d(0.0);
        for i in 1..=SAMPLES {
            let now = d(TAU * i as f64 / SAMPLES as f64);
            if (now < 0.0) != (prev < 0.0) {
                changes += 1;
            }
            prev = now;
        }
        if hits.iter().all(|h| !h.tangent) {
            prop_assert_eq!(changes, transversal, "{:?}", r);
        }
        Ok(())
    });
}

/// `ρ − R`: negative inside the cylinder.
fn implicit_signed(s: &Surface, p: Point3) -> f64 {
    let Surface::Cylinder { frame, radius } = s else {
        unreachable!()
    };
    let q = frame.to_local(p);
    q.x.hypot(q.y) - radius
}

// --- the rest of the table --------------------------------------------------

/// A NURBS surface is the only arm left without a form: every analytic
/// pair answers, a conic against a torus among them, and every curve
/// against a `Surface::Nurbs` is `Unsupported` naming both operands.
#[test]
fn only_a_nurbs_surface_is_unsupported() {
    check(
        (curve(), surface(), nurbs_curve(), nurbs_surface()),
        |(c, s, nc, ns)| {
            let nc = Curve::Nurbs(nc);
            let ns = Surface::Nurbs(ns);
            for curve in [&c, &nc] {
                match intersect_curve_surface(curve, &s, tol(), &mut arris_math::Meter::default()) {
                    Ok(_) => {}
                    other => return fail(format!("{curve:?} vs {s:?}: {other:?}")),
                }
                match intersect_curve_surface(curve, &ns, tol(), &mut arris_math::Meter::default())
                {
                    Err(GeomError::Unsupported { a, b }) => {
                        prop_assert_eq!(a, GeomKind::Curve(curve.kind()));
                        prop_assert_eq!(b, GeomKind::Surface(SurfaceKind::Nurbs));
                    }
                    other => return fail(format!("{curve:?} vs a NURBS surface: {other:?}")),
                }
            }
            Ok(())
        },
    );
}

#[test]
fn a_random_line_and_plane_or_cylinder_pass_the_common_properties() {
    check((line(), plane(), cylinder()), |(l, p, c)| {
        let r = common_properties(&l, &p)?;
        prop_assert_eq!(hits_of(&r).len(), 1, "{:?}", r);
        common_properties(&l, &c)?;
        Ok(())
    });
}

// --- line–cone, line–sphere and line–torus ---------------------------------

/// How far along a random line its signed distance to a quadric is
/// sampled: every coordinate the strategies make is within a few scales.
const REACH: f64 = 4.0 * DEFAULT_SCALE;
/// Samples over `[−REACH, REACH]`.
const LINE_SAMPLES: usize = 20_000;
/// The step either side of a transversal hit at which the signed
/// distance must differ in sign.
const ACROSS: f64 = 1e-6;

#[test]
fn a_random_line_crosses_a_quadric_where_its_distance_changes_sign() {
    check(
        (
            line(),
            prop_oneof![
                arris_debug::prop::geom::cone(),
                arris_debug::prop::geom::sphere(),
                arris_debug::prop::geom::torus()
            ],
        ),
        |(l, s)| {
            let r = common_properties(&l, &s)?;
            let hits = hits_of(&r);
            let f = |t: f64| signed_distance(&s, l.point(t));
            for h in hits.iter().filter(|h| !h.tangent) {
                prop_assert!(
                    f(h.t - ACROSS) * f(h.t + ACROSS) < 0.0,
                    "{l:?} vs {s:?}: no sign change at the crossing t = {}",
                    h.t
                );
            }
            let step = 2.0 * REACH / LINE_SAMPLES as f64;
            let mut previous = f(-REACH);
            for i in 1..=LINE_SAMPLES {
                let t = -REACH + step * i as f64;
                let now = f(t);
                if (now < 0.0) != (previous < 0.0) {
                    prop_assert!(
                        hits.iter().any(|h| (h.t - t).abs() <= 2.0 * step),
                        "{l:?} vs {s:?}: the distance changes sign before t = {t} but {r:?} has nothing there"
                    );
                }
                previous = now;
            }
            Ok(())
        },
    );
}

/// A line built against a cone, a sphere or a torus.
#[derive(Debug, Clone, Copy, PartialEq)]
enum QuadricLine {
    /// Through a sphere at a fraction of its radius from the centre.
    SphereChord,
    /// At the sphere's radius from its centre.
    SphereTangent,
    /// Clear of the sphere.
    SphereMiss,
    /// Along a sphere's own axis, through both poles.
    SpherePoles,
    /// A ruling of the cone.
    ConeRuling,
    /// Through the cone's apex, inside the nappes or outside them.
    ConeApex,
    /// In the cone's tangent plane along a ruling, across it.
    ConeTangent,
    /// In a plane through the cone's axis, parallel to a ruling.
    ConeParallel,
    /// In the torus's equatorial plane, through its axis.
    TorusDiameter,
    /// In the equatorial plane, grazing the outer equator.
    TorusOuter,
    /// In the equatorial plane, grazing the inner equator.
    TorusInner,
    /// Along the torus's axis.
    TorusAxis,
}

fn quadric_line() -> impl Strategy<Value = QuadricLine> {
    prop_oneof![
        Just(QuadricLine::SphereChord),
        Just(QuadricLine::SphereTangent),
        Just(QuadricLine::SphereMiss),
        Just(QuadricLine::SpherePoles),
        Just(QuadricLine::ConeRuling),
        Just(QuadricLine::ConeApex),
        Just(QuadricLine::ConeTangent),
        Just(QuadricLine::ConeParallel),
        Just(QuadricLine::TorusDiameter),
        Just(QuadricLine::TorusOuter),
        Just(QuadricLine::TorusInner),
        Just(QuadricLine::TorusAxis),
    ]
}

#[test]
fn a_line_against_a_quadric_follows_the_case_table() {
    check(
        (
            quadric_line(),
            arris_debug::prop::frame(),
            finite_f64(0.5..=10.0),
            finite_f64(0.1..=0.9),
            finite_f64(0.1..=1.45),
            finite_f64(0.0..=TAU),
            finite_f64(0.3..=1.3),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            any::<bool>(),
        ),
        |(case, frame, big, fraction, angle, phase, tilt, s, flip)| {
            let z = frame.z().into_inner();
            let (a, across) = around(&frame, phase);
            let unit = |v: Vec3| arris_math::UnitVec3::new_normalize(v);
            let small = fraction * big;
            // The line through `at` along `d`, `at` being its point `s`.
            let through = |at: Point3, d: Vec3| {
                let d = unit(if flip { -d } else { d });
                (
                    Curve::Line {
                        origin: at - s * d.into_inner(),
                        direction: d,
                    },
                    s,
                )
            };
            let sphere = Surface::Sphere { frame, radius: big };
            let cone = Surface::Cone {
                frame,
                radius: big,
                half_angle: angle,
            };
            let torus = Surface::Torus {
                frame,
                major_radius: big,
                minor_radius: small,
            };
            let (sa, ca) = angle.sin_cos();
            let apex = frame.origin() - (big * ca / sa) * z;
            let ruling = sa * a + ca * z;
            let o = frame.origin();
            match case {
                QuadricLine::SphereChord | QuadricLine::SphereTangent | QuadricLine::SphereMiss => {
                    let dist = match case {
                        QuadricLine::SphereChord => small,
                        QuadricLine::SphereTangent => big,
                        _ => big * (1.1 + fraction),
                    };
                    let (c, at) = through(o + dist * a, tilt.cos() * across + tilt.sin() * z);
                    let r = common_properties(&c, &sphere)?;
                    match case {
                        QuadricLine::SphereChord => {
                            let half = (big * big - dist * dist).sqrt();
                            expect_hits(&c, &r, &[at - half, at + half], false)
                        }
                        QuadricLine::SphereTangent => expect_hits(&c, &r, &[at], true),
                        _ => expect_empty(&r),
                    }
                }
                QuadricLine::SpherePoles => {
                    let (c, at) = through(o, z);
                    let r = common_properties(&c, &sphere)?;
                    expect_hits(&c, &r, &[at - big, at + big], false)?;
                    for h in hits_of(&r) {
                        prop_assert_eq!(h.uv.x, 0.0, "a pole takes u = 0: {:?}", h);
                        prop_assert!((h.uv.y.abs() - FRAC_PI_2).abs() <= 1e-15);
                    }
                    Ok(())
                }
                QuadricLine::ConeRuling => {
                    let (c, _) = through(apex + s * ruling, ruling);
                    expect_coincident(&common_properties(&c, &cone)?)
                }
                QuadricLine::ConeApex => {
                    // Inside the nappes at half the half-angle, outside
                    // them half way to the plane across the axis.
                    let off = if flip {
                        0.5 * angle
                    } else {
                        0.5 * (angle + FRAC_PI_2)
                    };
                    let (c, at) = through(apex, off.cos() * z + off.sin() * a);
                    let r = common_properties(&c, &cone)?;
                    expect_hits(&c, &r, &[at], true)?;
                    let [h] = hits_of(&r) else { unreachable!() };
                    prop_assert_eq!(h.uv, arris_math::Point2::new(0.0, -big / sa));
                    Ok(())
                }
                QuadricLine::ConeTangent => {
                    // A point of the ruling beyond the apex, the line in
                    // the plane of the ruling and the parallel there.
                    let at = apex + (0.5 + fraction) * big * ruling;
                    let (c, t) = through(at, tilt.cos() * ruling + tilt.sin() * across);
                    expect_hits(&c, &common_properties(&c, &cone)?, &[t], true)
                }
                QuadricLine::ConeParallel => {
                    // Off the ruling toward the axis in their plane: it
                    // meets the far ruling of the section once.
                    let (c, _) = through(apex + small * z, ruling);
                    let r = common_properties(&c, &cone)?;
                    let hits = hits_of(&r);
                    prop_assert_eq!(hits.len(), 1, "{:?}", r);
                    prop_assert!(!hits[0].tangent);
                    Ok(())
                }
                QuadricLine::TorusDiameter => {
                    let (c, at) = through(o, a);
                    let (inner, outer) = (big - small, big + small);
                    expect_hits(
                        &c,
                        &common_properties(&c, &torus)?,
                        &[at - outer, at - inner, at + inner, at + outer],
                        false,
                    )
                }
                QuadricLine::TorusOuter => {
                    let (c, at) = through(o + (big + small) * across, a);
                    expect_hits(&c, &common_properties(&c, &torus)?, &[at], true)
                }
                QuadricLine::TorusInner => {
                    let (c, at) = through(o + (big - small) * across, a);
                    let r = common_properties(&c, &torus)?;
                    let hits = hits_of(&r);
                    prop_assert_eq!(hits.len(), 3, "{:?}", r);
                    let half = ((big + small).powi(2) - (big - small).powi(2)).sqrt();
                    prop_assert!((hits[1].t - at).abs() <= EXACT && hits[1].tangent);
                    prop_assert!(!hits[0].tangent && !hits[2].tangent);
                    prop_assert!(((hits[2].t - hits[0].t) - 2.0 * half).abs() <= EXACT);
                    Ok(())
                }
                QuadricLine::TorusAxis => {
                    let (c, _) = through(o, z);
                    expect_empty(&common_properties(&c, &torus)?)
                }
            }
        },
    );
}

// --- ellipse–plane and ellipse–cylinder -----------------------------------

/// A cylinder, and a plane oblique enough to section it in an ellipse:
/// the normal is `cos θ` along the axis and `sin θ` across it, with `θ`
/// away from both `0` (a circle) and `π/2` (rulings).
fn oblique_section() -> impl Strategy<Value = (Surface, Curve)> {
    (
        cylinder(),
        finite_f64(0.2..=1.2),
        finite_f64(0.0..=TAU),
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
    )
        .prop_filter_map(
            "the oblique plane sections the cylinder in an ellipse",
            |(cyl, tilt, phase, along)| {
                let frame = cyl.frame().unwrap();
                let (across, _) = around(frame, phase);
                let normal = tilt.cos() * frame.z().into_inner() + tilt.sin() * across;
                let origin = frame.origin() + along * frame.z().into_inner();
                let plane = Surface::Plane {
                    frame: Frame::from_z(origin, normal).ok()?,
                };
                match arris_geom::intersect_surfaces(
                    &plane,
                    &cyl,
                    &within(),
                    tol(),
                    &mut arris_math::Meter::default(),
                ) {
                    Ok(r) => match r.curves().first() {
                        Some(arris_geom::MeetCurve {
                            curve: e @ Curve::Ellipse { .. },
                            kind: arris_geom::MeetKind::Crossing,
                        }) => Some((cyl, e.clone())),
                        _ => None,
                    },
                    Err(_) => None,
                }
            },
        )
}

#[test]
fn an_oblique_section_ellipse_lies_on_its_cylinder() {
    check(oblique_section(), |(cyl, section)| {
        expect_coincident(&common_properties(&section, &cyl)?)
    });
}

#[test]
fn a_plane_through_a_sections_centre_cuts_it_at_the_minor_axis() {
    check(oblique_section(), |(_, section)| {
        let Curve::Ellipse { frame, .. } = section else {
            return fail("not an ellipse".to_string());
        };
        // Normal to the major axis through the centre: the ellipse meets
        // it where `a cos t = 0`, the two ends of the minor axis.
        let cut = Surface::Plane {
            frame: Frame::from_z(frame.origin(), frame.x().into_inner())
                .map_err(|e| TestCaseError::fail(e.to_string()))?,
        };
        let r = common_properties(&section, &cut)?;
        expect_hits(&section, &r, &[PI / 2.0, 3.0 * PI / 2.0], false)
    });
}

/// An ellipse in a plane through a cylinder's axis, reaching `reach`
/// across it and `1.0` along it: the radial distance is `|reach·cos t|`,
/// so it crosses four times when `reach > R`, touches twice when
/// `reach = R`, and misses when `reach < R`.
fn meridional_ellipse() -> impl Strategy<Value = (Surface, Curve, f64)> {
    (
        cylinder(),
        finite_f64(0.0..=TAU),
        finite_f64(0.3..=2.5),
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
    )
        .prop_filter_map(
            "an ellipse in a plane through the axis",
            |(cyl, phase, factor, along)| {
                let radius = match cyl {
                    Surface::Cylinder { radius, .. } => radius,
                    _ => return None,
                };
                let frame = cyl.frame().unwrap();
                let (across, tangential) = around(frame, phase);
                let centre = frame.origin() + along * frame.z().into_inner();
                let reach = factor * radius;
                let ellipse = Curve::Ellipse {
                    frame: Frame::new(centre, tangential, across).ok()?,
                    major_radius: reach.max(radius / 2.0),
                    minor_radius: reach.min(radius / 2.0),
                };
                // The frame's `X` is `across` only when it is the major
                // axis; otherwise the reach across the axis is the minor
                // one and the case is a different one.
                (reach >= radius / 2.0).then_some((cyl, ellipse, reach))
            },
        )
}

#[test]
fn a_meridional_ellipse_crosses_four_times_when_it_reaches_past_the_wall() {
    check(meridional_ellipse(), |(cyl, ellipse, reach)| {
        let radius = match cyl {
            Surface::Cylinder { radius, .. } => radius,
            _ => return fail("not a cylinder".to_string()),
        };
        let r = common_properties(&ellipse, &cyl)?;
        let hits = hits_of(&r);
        // `reach·|cos t| = R` has four roots when `reach > R` and none
        // when `reach < R`; the tolerance band around equality is the
        // touch, which the built cases avoid.
        let expected = if reach > radius + tol().linear {
            4
        } else if reach < radius - tol().linear {
            0
        } else {
            return Ok(());
        };
        prop_assert_eq!(
            hits.len(),
            expected,
            "reach {} vs R {}: {:?}",
            reach,
            radius,
            r
        );
        for h in hits {
            prop_assert!(!h.tangent);
            let local = cyl.frame().unwrap().to_local(h.point);
            prop_assert!((local.x.hypot(local.y) - radius).abs() <= EXACT);
        }
        Ok(())
    });
}

#[test]
fn a_meridional_ellipse_that_just_reaches_the_wall_touches_twice() {
    check(
        (
            cylinder(),
            finite_f64(0.0..=TAU),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        ),
        |(cyl, phase, along)| {
            let radius = match cyl {
                Surface::Cylinder { radius, .. } => radius,
                _ => return fail("not a cylinder".to_string()),
            };
            let frame = cyl.frame().unwrap();
            let (across, tangential) = around(frame, phase);
            let centre = frame.origin() + along * frame.z().into_inner();
            let ellipse = Curve::Ellipse {
                frame: Frame::new(centre, tangential, across)
                    .map_err(|e| TestCaseError::fail(e.to_string()))?,
                major_radius: radius,
                minor_radius: 0.5 * radius,
            };
            let r = common_properties(&ellipse, &cyl)?;
            // The extrema of the radial distance are at `t = 0` and
            // `t = π`, both on the wall: one touch each, never split into
            // two crossings.
            expect_hits(&ellipse, &r, &[0.0, PI], true)
        },
    );
}

#[test]
fn a_random_ellipse_against_a_plane_or_a_cylinder_passes_the_common_properties() {
    check(
        (arris_debug::prop::geom::ellipse(), plane(), cylinder()),
        |(e, p, c)| {
            common_properties(&e, &p)?;
            common_properties(&e, &c)?;
            Ok(())
        },
    );
}

// --- conic–cone, conic–sphere and conic–elliptic cylinder -------------------

/// Any conic in a random pose.
fn conic() -> impl Strategy<Value = Curve> {
    prop_oneof![circle(), arris_debug::prop::geom::ellipse()]
}

/// The surfaces a conic has no exact section of: a cone, a sphere, an
/// elliptic cylinder, which the conic meets in a plane of any tilt, and
/// a torus, whose quartic form the conic's quarter arcs go into.
fn conic_quadric() -> impl Strategy<Value = Surface> {
    prop_oneof![
        arris_debug::prop::geom::cone(),
        arris_debug::prop::geom::sphere(),
        arris_debug::prop::geom::elliptic_cylinder(),
        arris_debug::prop::geom::torus(),
    ]
}

/// A quadric and a conic in a random pose whose centre has been moved
/// onto it, its plane and radii kept: a surface and a curve drawn
/// independently in a box a scale wide are almost always apart, and a
/// pair that misses tests nothing.
fn conic_on_a_quadric() -> impl Strategy<Value = (Curve, Surface)> {
    (
        conic(),
        conic_quadric(),
        finite_f64(0.0..=TAU),
        finite_f64(-1.4..=1.4),
    )
        .prop_filter_map(
            "the conic keeps its plane about its new centre",
            |(c, s, u, v)| {
                let centre = s.point(u, v);
                let plane =
                    |f: Frame| Frame::new(centre, f.z().into_inner(), f.x().into_inner()).ok();
                let moved = match c {
                    Curve::Circle { frame, radius } => Curve::Circle {
                        frame: plane(frame)?,
                        radius,
                    },
                    Curve::Ellipse {
                        frame,
                        major_radius,
                        minor_radius,
                    } => Curve::Ellipse {
                        frame: plane(frame)?,
                        major_radius,
                        minor_radius,
                    },
                    Curve::Line { .. } | Curve::Nurbs(_) => return None,
                };
                Some((moved, s))
            },
        )
}

#[test]
fn a_random_conic_crosses_a_quadric_where_its_distance_changes_sign() {
    check(conic_on_a_quadric(), |(c, s)| {
        let r = common_properties(&c, &s)?;
        let hits = hits_of(&r);
        let f = |t: f64| signed_distance(&s, c.point(t));
        for h in hits.iter().filter(|h| !h.tangent) {
            // `ACROSS` of length either side of a crossing, in the
            // conic's own parameter.
            let step = ACROSS / c.eval(h.t).d1.norm();
            prop_assert!(
                f(h.t - step) * f(h.t + step) < 0.0,
                "{c:?} vs {s:?}: no sign change at the crossing t = {}",
                h.t
            );
        }
        let step = TAU / SAMPLES as f64;
        let mut previous = f(0.0);
        for i in 1..=SAMPLES {
            let t = step * i as f64;
            let now = f(t);
            if (now < 0.0) != (previous < 0.0) {
                prop_assert!(
                    hits.iter()
                        .any(|h| param_diff(Some(TAU), h.t, t) <= 2.0 * step),
                    "{c:?} vs {s:?}: the distance changes sign before t = {t} but {r:?} has nothing there"
                );
            }
            previous = now;
        }
        Ok(())
    });
}

/// A conic built against a cone, a sphere or an elliptic cylinder.
#[derive(Debug, Clone, Copy, PartialEq)]
enum ConicCase {
    /// A circle of the sphere — a great one through both poles when the
    /// case is flipped.
    SphereParallel,
    /// A circle crossing the sphere at its pole.
    SpherePole,
    /// An ellipse through the centre, as long as the sphere's diameter:
    /// inside it, touching at both major vertices.
    SphereInside,
    /// A circle clear of the sphere.
    SphereOutside,
    /// A parallel of the cone.
    ConeParallel,
    /// A circle through the cone's apex, in a plane through its axis.
    ConeApex,
    /// A circle in a plane across the axis, off it: the cone's parallel
    /// there is one circle and this is another, in one plane.
    ConeSection,
    /// The same, touching that parallel from outside.
    ConeSectionTangent,
    /// An oblique section of the elliptic cylinder.
    EllipticOblique,
    /// The same section scaled about the axis: clear of the surface
    /// everywhere, with the polynomial constant along it.
    EllipticScaled,
    /// A circle in a plane through the axis, reaching past the wall.
    EllipticMeridian,
    /// The same, just reaching it.
    EllipticMeridianTangent,
    /// A parallel of the torus: the circle at one `v`, about the axis.
    TorusParallel,
    /// A tube circle of the torus, in a plane through the axis.
    TorusTube,
    /// A Villarceau circle: the torus's section with a bitangent plane
    /// through its centre, which is a circle of the major radius.
    TorusVillarceau,
    /// An ellipse in the equatorial plane about the centre, reaching
    /// past the outer equator and inside the inner one: it leaves and
    /// re-enters the tube eight times.
    TorusEquatorial,
    /// A circle of the inner radius in a plane through the axis: it
    /// touches the torus at the two inner equator points.
    TorusInnerTouch,
    /// A circle inside the hole, clear of the tube.
    TorusMiss,
}

fn conic_case() -> impl Strategy<Value = ConicCase> {
    prop_oneof![
        Just(ConicCase::SphereParallel),
        Just(ConicCase::SpherePole),
        Just(ConicCase::SphereInside),
        Just(ConicCase::SphereOutside),
        Just(ConicCase::ConeParallel),
        Just(ConicCase::ConeApex),
        Just(ConicCase::ConeSection),
        Just(ConicCase::ConeSectionTangent),
        Just(ConicCase::EllipticOblique),
        Just(ConicCase::EllipticScaled),
        Just(ConicCase::EllipticMeridian),
        Just(ConicCase::EllipticMeridianTangent),
        Just(ConicCase::TorusParallel),
        Just(ConicCase::TorusTube),
        Just(ConicCase::TorusVillarceau),
        Just(ConicCase::TorusEquatorial),
        Just(ConicCase::TorusInnerTouch),
        Just(ConicCase::TorusMiss),
    ]
}

/// How far a scaled section of an elliptic cylinder is taken past it.
const SCALED: f64 = 1.2;

#[test]
fn a_conic_against_a_quadric_follows_the_case_table() {
    check(
        (
            conic_case(),
            arris_debug::prop::frame(),
            finite_f64(0.5..=10.0),
            finite_f64(0.1..=0.9),
            finite_f64(0.1..=1.45),
            finite_f64(0.0..=TAU),
            any::<bool>(),
        ),
        |(case, frame, big, fraction, angle, phase, flip)| {
            let z = frame.z().into_inner();
            let (a, across) = around(&frame, phase);
            let o = frame.origin();
            let small = fraction * big;
            let (sa, ca) = angle.sin_cos();
            let sphere = Surface::Sphere { frame, radius: big };
            let cone = Surface::Cone {
                frame,
                radius: big,
                half_angle: angle,
            };
            let wall = Surface::EllipticCylinder {
                frame,
                major_radius: big,
                minor_radius: small,
            };
            let ring = Surface::Torus {
                frame,
                major_radius: big,
                minor_radius: small,
            };
            let apex = o - (big * ca / sa) * z;
            let plane_of = |normal: Vec3, x: Vec3, centre: Point3| {
                Frame::new(centre, normal, x).map_err(|e| TestCaseError::fail(e.to_string()))
            };
            match case {
                ConicCase::SphereParallel => {
                    // In a plane square to `a`: a great circle through
                    // both poles when it holds the centre.
                    let lift = if flip { 0.0 } else { small };
                    let c = Curve::Circle {
                        frame: plane_of(a, z, o + lift * a)?,
                        radius: (big - lift).sqrt() * (big + lift).sqrt(),
                    };
                    expect_coincident(&common_properties(&c, &sphere)?)
                }
                ConicCase::SpherePole => {
                    // Through the north pole, in the plane of the axis
                    // and `a`: `|P|² − R² = 2c²(1 − cos t) + 2Rc sin t`
                    // vanishes at `t = 0` and where `tan(t/2) = −R / c`.
                    let c = Curve::Circle {
                        frame: plane_of(across, -a, o + big * z + small * a)?,
                        radius: small,
                    };
                    let r = common_properties(&c, &sphere)?;
                    expect_hits(&c, &r, &[0.0, TAU - 2.0 * (big / small).atan()], false)?;
                    let pole = o + big * z;
                    let at_pole = hits_of(&r)
                        .iter()
                        .find(|h| (h.point - pole).norm() <= EXACT);
                    prop_assert!(at_pole.is_some(), "{r:?} has no hit at the pole {pole}");
                    Ok(())
                }
                ConicCase::SphereInside => {
                    let c = Curve::Ellipse {
                        frame: plane_of(across, a, o)?,
                        major_radius: big,
                        minor_radius: small,
                    };
                    expect_hits(&c, &common_properties(&c, &sphere)?, &[0.0, PI], true)
                }
                ConicCase::SphereOutside => {
                    let c = Curve::Circle {
                        frame: plane_of(across, a, o + (2.1 * big + small) * a)?,
                        radius: small,
                    };
                    expect_empty(&common_properties(&c, &sphere)?)
                }
                ConicCase::ConeParallel => {
                    // The cone's radius `R + tan α·s` at the height `s`.
                    let c = Curve::Circle {
                        frame: plane_of(z, a, o + small * z)?,
                        radius: big + small * sa / ca,
                    };
                    expect_coincident(&common_properties(&c, &cone)?)
                }
                ConicCase::ConeApex => {
                    // Through the apex at `t = π`, up the axis to `2r`:
                    // on the cone where `|tan(t/2)| = tan α`.
                    let c = Curve::Circle {
                        frame: plane_of(across, z, apex + big * z)?,
                        radius: big,
                    };
                    let r = common_properties(&c, &cone)?;
                    let hits = hits_of(&r);
                    prop_assert_eq!(hits.len(), 3, "{:?}", r);
                    let touches: Vec<&CurveSurfaceHit> =
                        hits.iter().filter(|h| h.tangent).collect();
                    prop_assert_eq!(touches.len(), 1, "{:?}", r);
                    prop_assert!((touches[0].t - PI).abs() * big <= EXACT, "{:?}", r);
                    prop_assert_eq!(
                        touches[0].uv,
                        arris_math::Point2::new(0.0, -big / sa),
                        "the apex takes u = 0"
                    );
                    for (h, t) in hits
                        .iter()
                        .filter(|h| !h.tangent)
                        .zip([2.0 * angle, TAU - 2.0 * angle])
                    {
                        prop_assert!((h.t - t).abs() * big <= EXACT, "{h:?} vs {t}");
                    }
                    Ok(())
                }
                ConicCase::ConeSection | ConicCase::ConeSectionTangent => {
                    // In the plane across the axis at the height `s`, the
                    // cone is the circle of radius `R_h` about the axis,
                    // and this is the two-circle case.
                    let parallel = big + small * sa / ca;
                    let radius = 0.5 * big;
                    let (near, far) = ((radius - parallel).abs(), radius + parallel);
                    let delta = match case {
                        ConicCase::ConeSection => near + fraction * (far - near),
                        _ => far,
                    };
                    let c = Curve::Circle {
                        frame: plane_of(z, a, o + small * z + delta * a)?,
                        radius,
                    };
                    let r = common_properties(&c, &cone)?;
                    if case == ConicCase::ConeSectionTangent {
                        return expect_hits(&c, &r, &[PI], true);
                    }
                    let x = (delta * delta + parallel * parallel - radius * radius) / (2.0 * delta);
                    let y = (parallel - x).sqrt() * (parallel + x).sqrt();
                    let t = y.atan2(x - delta);
                    expect_hits(&c, &r, &[t, -t], false)
                }
                ConicCase::EllipticOblique | ConicCase::EllipticScaled => {
                    // The plane `z = m·y` in the surface's frame, with
                    // the tilt short of where the section's minor axis
                    // would outgrow its major one.
                    let m = 0.5 * ((big / small).powi(2) - 1.0).sqrt();
                    let minor = small * (1.0 + m * m).sqrt();
                    let scale = if case == ConicCase::EllipticOblique {
                        1.0
                    } else {
                        SCALED
                    };
                    let along = (frame.y().into_inner() + m * z).normalize();
                    let c = Curve::Ellipse {
                        frame: plane_of(
                            frame.x().into_inner().cross(&along),
                            frame.x().into_inner(),
                            o,
                        )?,
                        major_radius: scale * big,
                        minor_radius: scale * minor,
                    };
                    let r = common_properties(&c, &wall)?;
                    if case == ConicCase::EllipticOblique {
                        expect_coincident(&r)
                    } else {
                        expect_empty(&r)
                    }
                }
                ConicCase::EllipticMeridian | ConicCase::EllipticMeridianTangent => {
                    // In the plane of the axis and `a`, the wall is the
                    // two rulings the section reaches `M` along `a` at.
                    let reach = 1.0 / ((phase.cos() / big).hypot(phase.sin() / small));
                    let radius = match case {
                        ConicCase::EllipticMeridian => reach * (1.2 + fraction),
                        _ => reach,
                    };
                    let c = Curve::Circle {
                        frame: plane_of(a.cross(&z), a, o)?,
                        radius,
                    };
                    let r = common_properties(&c, &wall)?;
                    if case == ConicCase::EllipticMeridianTangent {
                        return expect_hits(&c, &r, &[0.0, PI], true);
                    }
                    let half = (reach / radius).acos();
                    expect_hits(&c, &r, &[half, PI - half, PI + half, TAU - half], false)
                }
                ConicCase::TorusParallel => {
                    // The circle of the torus at `v = angle`: radius
                    // `R + r cos v` about the axis at the height `r sin v`.
                    let (sv, cv) = angle.sin_cos();
                    let c = Curve::Circle {
                        frame: plane_of(z, a, o + (small * sv) * z)?,
                        radius: big + small * cv,
                    };
                    expect_coincident(&common_properties(&c, &ring)?)
                }
                ConicCase::TorusTube => {
                    // The tube circle in the plane of the axis and `a`,
                    // about the point of the core circle there.
                    let c = Curve::Circle {
                        frame: plane_of(across, a, o + big * a)?,
                        radius: small,
                    };
                    expect_coincident(&common_properties(&c, &ring)?)
                }
                ConicCase::TorusVillarceau => {
                    // The bitangent plane through the centre leans by
                    // `sin α = r / R` off the axis; the two circles it
                    // cuts have the major radius and their centres `r`
                    // from the centre across it.
                    let (sv, cv) = (small / big, (1.0 - fraction * fraction).sqrt());
                    let lean = -sv * a + cv * z;
                    let c = Curve::Circle {
                        frame: plane_of(lean, cv * a + sv * z, o + small * across)?,
                        radius: big,
                    };
                    expect_coincident(&common_properties(&c, &ring)?)
                }
                ConicCase::TorusEquatorial => {
                    // In the equatorial plane about the centre, past the
                    // outer equator along `X` and inside the inner one
                    // along `Y`: four crossings of each.
                    let (major, minor) = (1.1 * (big + small), 0.9 * (big - small));
                    let c = Curve::Ellipse {
                        frame: plane_of(z, a, o)?,
                        major_radius: major,
                        minor_radius: minor,
                    };
                    let r = common_properties(&c, &ring)?;
                    // `a² cos² t + b² sin² t = ρ²` at both equators.
                    let across_at = |rho: f64| {
                        let cos = ((rho * rho - minor * minor) / (major * major - minor * minor))
                            .sqrt()
                            .acos();
                        [cos, PI - cos, PI + cos, TAU - cos]
                    };
                    let expected: Vec<f64> = across_at(big + small)
                        .into_iter()
                        .chain(across_at(big - small))
                        .collect();
                    expect_hits(&c, &r, &expected, false)
                }
                ConicCase::TorusInnerTouch => {
                    // Of the inner radius in the plane of the axis and
                    // `a`: tangent to the tube at both inner equator
                    // points, from inside the hole.
                    let c = Curve::Circle {
                        frame: plane_of(across, a, o)?,
                        radius: big - small,
                    };
                    expect_hits(&c, &common_properties(&c, &ring)?, &[0.0, PI], true)
                }
                ConicCase::TorusMiss => {
                    // About the axis inside the hole: its distance from
                    // the torus is constant and never vanishes.
                    let c = Curve::Circle {
                        frame: plane_of(z, a, o)?,
                        radius: 0.5 * (big - small),
                    };
                    expect_empty(&common_properties(&c, &ring)?)
                }
            }
        },
    );
}
