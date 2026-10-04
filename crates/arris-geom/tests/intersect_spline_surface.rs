//! A `Curve::Nurbs` against every analytic surface (ADR-0018): an
//! ellipse as a NURBS — exact and rational, or fitted and periodic —
//! meets each surface where the ellipse itself does, hit for hit, and
//! where the `Curve::Ellipse` arm has a closed form, where that arm says;
//! a straight NURBS where the line arm says; a random NURBS where its
//! distance changes sign and nowhere else; a touch is one tangent hit
//! within the tolerance, two crossings past it and nothing short of it; a
//! curve on the surface is `Coincident`, a fitted section curve with both
//! of its surfaces; an open curve's end on the surface is a hit and no
//! touch; and a periodic curve's hits come back inside its domain.

use core::f64::consts::{FRAC_1_SQRT_2, TAU};

use arris_debug::prop::geom::{ellipse, line, nurbs_curve, surface};
use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, pose};
use arris_geom::{
    Curve, CurveSurfaceHit, CurveSurfaceIntersection, GeomError, MAX_DEGREE, NurbsCurve, Surface,
    SurfaceIntersection, fit_curve_periodic, intersect_curve_surface, intersect_surfaces,
};
use arris_math::{Aabb, Frame, Interval, Isometry, Point3, Precision, Tolerance, UnitVec3, Vec3};
use proptest::prelude::*;

/// Hit points against both operands, in length units.
const EXACT: f64 = 1e-12 * DEFAULT_SCALE;
/// What a fitted ellipse is held to against the ellipse.
const FIT: f64 = 1e-10;
/// How far a crossing of the fitted ellipse may be from the ellipse's
/// own: [`FIT`] over the sine of the crossing angle, which [`MARGIN`]
/// keeps above 1e-3.
const FITTED: f64 = 1e-6;
/// Samples of the signed distance round an ellipse or along a NURBS.
const SAMPLES: usize = 7200;
/// A case is decided by sampling only where every extremum of the
/// sampled distance is at least this far from the surface: nearer, the
/// fitted curve and the ellipse may honestly differ in what they meet.
const MARGIN: f64 = 1e-2;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

fn fail<T>(msg: String) -> Result<T, TestCaseError> {
    Err(TestCaseError::fail(msg))
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
        Surface::Nurbs(_) => unreachable!("the strategies draw analytic surfaces"),
    }
}

/// The ellipse of `frame` reaching `a` along `X` and `b` along `Y` as
/// four rational quadratic arcs: exact, clamped, closed, its parameter
/// not the ellipse's angle.
fn rational_ellipse(frame: &Frame, a: f64, b: f64) -> NurbsCurve {
    let at = |x: f64, y: f64| frame.to_world(Point3::new(a * x, b * y, 0.0));
    let w = FRAC_1_SQRT_2;
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0],
        vec![
            at(1.0, 0.0),
            at(1.0, 1.0),
            at(0.0, 1.0),
            at(-1.0, 1.0),
            at(-1.0, 0.0),
            at(-1.0, -1.0),
            at(0.0, -1.0),
            at(1.0, -1.0),
            at(1.0, 0.0),
        ],
        vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0],
    )
    .unwrap()
}

/// A conic as a periodic quintic fitted at the conic's own angle.
fn fitted(conic: &Curve) -> NurbsCurve {
    fit_curve_periodic(
        |t| conic.point(t),
        Interval::new(0.0, TAU).unwrap(),
        5,
        |t, q| (q - conic.point(t)).norm(),
        FIT,
        &mut arris_math::Meter::default(),
    )
    .unwrap()
}

fn hits_of(r: &CurveSurfaceIntersection) -> &[CurveSurfaceHit] {
    match r {
        CurveSurfaceIntersection::Points(h) => h,
        CurveSurfaceIntersection::Coincident => &[],
    }
}

/// What every result owes whatever its case: hits ascending with `t` in
/// the domain, below its end for a periodic curve; each point the
/// curve's, on the surface to rounding where it crosses and within the
/// tolerance where it touches; `uv` the surface's parameters of it; two
/// runs the same bit for bit.
fn common_properties(c: &Curve, s: &Surface) -> Result<CurveSurfaceIntersection, TestCaseError> {
    let r = intersect_curve_surface(c, s, tol(), &mut arris_math::Meter::default())
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
    let hits = hits_of(&r);
    let domain = c.domain();
    for h in hits {
        prop_assert!(domain.contains(h.t), "t = {} outside {domain:?}", h.t);
        if c.period().is_some() {
            prop_assert!(h.t < domain.hi(), "t = {} not below the period's end", h.t);
        }
        prop_assert!((c.point(h.t) - h.point).norm() <= EXACT);
        let exact = EXACT * (h.point.coords.norm() / DEFAULT_SCALE).max(1.0);
        let allowed = if h.tangent {
            tol().linear + exact
        } else {
            exact
        };
        let off = signed_distance(s, h.point).abs();
        prop_assert!(
            off <= allowed,
            "{s:?}: hit at t = {} is off the surface by {off} (tangent {})",
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

/// Samples put into a coarse step whose ends are near the surface.
const REFINED: usize = 64;

/// The crossings of `f` over `[lo, hi]` by sampling and a bisection in
/// each bracket — the last sample joined to the first when `closed` —
/// or `None` where sampling decides nothing. [`SAMPLES`] even steps, and
/// [`REFINED`] more inside every step with an end nearer the surface
/// than `reach`, the farthest the distance moves in one step (the
/// curve's speed bounds it): two crossings close enough to hide between
/// two of those are a dip so shallow that a sample beside it is an
/// extremum within [`MARGIN`] of zero, which is what makes a case
/// undecided.
fn sampled_crossings(
    f: &dyn Fn(f64) -> f64,
    [lo, hi]: [f64; 2],
    reach: f64,
    closed: bool,
) -> Option<Vec<f64>> {
    let at = |i: usize| lo + (hi - lo) * i as f64 / SAMPLES as f64;
    let mut samples: Vec<(f64, f64)> = Vec::new();
    let mut here = (at(0), f(at(0)));
    for i in 0..SAMPLES {
        let next = (at(i + 1), f(at(i + 1)));
        samples.push(here);
        if here.1.abs().min(next.1.abs()) < reach {
            for k in 1..REFINED {
                let t = here.0 + (next.0 - here.0) * k as f64 / REFINED as f64;
                samples.push((t, f(t)));
            }
        }
        here = next;
    }
    if !closed {
        samples.push(here);
    }
    let n = samples.len();
    for i in 0..n {
        let (before, after) = if closed {
            (samples[(i + n - 1) % n].1, samples[(i + 1) % n].1)
        } else if i == 0 || i == n - 1 {
            // An open curve's end decides nothing near the surface either.
            (samples[i].1, samples[i].1)
        } else {
            (samples[i - 1].1, samples[i + 1].1)
        };
        let v = samples[i].1;
        let extremum = (before >= v && after >= v) || (before <= v && after <= v);
        if extremum && v.abs() < MARGIN {
            return None;
        }
    }
    let mut found = Vec::new();
    let last = if closed { n } else { n - 1 };
    for i in 0..last {
        let (from, to) = (samples[i], samples[(i + 1) % n]);
        if (from.1 < 0.0) == (to.1 < 0.0) {
            continue;
        }
        let (mut a, mut b) = (from.0, if to.0 > from.0 { to.0 } else { hi });
        for _ in 0..200 {
            let m = 0.5 * (a + b);
            if (f(m) < 0.0) == (from.1 < 0.0) {
                a = m;
            } else {
                b = m;
            }
        }
        found.push(0.5 * (a + b));
    }
    Some(found)
}

/// Twice the greatest speed among [`SAMPLES`] samples of `c`, times one
/// sampling step: how far the distance to anything moves in a step.
fn reach_of(c: &Curve) -> f64 {
    let domain = c.domain();
    let speed = (0..=SAMPLES)
        .map(|i| c.eval(domain.lerp(i as f64 / SAMPLES as f64)).d1.norm())
        .fold(0.0, f64::max);
    2.0 * speed * domain.length() / SAMPLES as f64
}

/// `hits` are crossings at `expected`, one for one, within `bound`.
fn expect_points(
    what: &str,
    hits: &[CurveSurfaceHit],
    expected: &[Point3],
    bound: f64,
) -> Result<(), TestCaseError> {
    let each: Vec<(Point3, f64)> = expected.iter().map(|&p| (p, bound)).collect();
    expect_points_within(what, hits, &each)
}

/// [`expect_points`] with a bound of its own for each expected point.
fn expect_points_within(
    what: &str,
    hits: &[CurveSurfaceHit],
    expected: &[(Point3, f64)],
) -> Result<(), TestCaseError> {
    prop_assert_eq!(
        hits.len(),
        expected.len(),
        "{}: {:?} against {:?}",
        what,
        hits,
        expected
    );
    for (p, bound) in expected {
        let Some(h) = hits.iter().find(|h| (h.point - p).norm() <= *bound) else {
            return fail(format!("{what}: no hit at {p} among {hits:?}"));
        };
        prop_assert!(!h.tangent, "{what}: a crossing reported as a touch: {h:?}");
    }
    Ok(())
}

/// An ellipse and a surface moved so that the surface's point at `uv`
/// is the ellipse's at `t`: they meet, in general across each other.
fn meeting_pair() -> impl Strategy<Value = (Curve, Surface)> {
    (
        ellipse(),
        surface(),
        finite_f64(0.0..=TAU),
        finite_f64(0.0..=TAU),
        finite_f64(-1.0..=1.0),
    )
        .prop_map(|(c, s, t, u, v)| {
            let shift = c.point(t) - s.point(u, v);
            (c, s.transformed(&Isometry::from_translation(shift)))
        })
}

#[test]
fn an_ellipse_as_a_nurbs_meets_each_surface_where_the_ellipse_does() {
    check(meeting_pair(), |(conic, s)| {
        let Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } = &conic
        else {
            unreachable!()
        };
        let f = |t: f64| signed_distance(&s, conic.point(t));
        let Some(crossings) = sampled_crossings(&f, [0.0, TAU], reach_of(&conic), true) else {
            return Ok(());
        };
        let expected: Vec<Point3> = crossings.iter().map(|&t| conic.point(t)).collect();
        // The closed form, where the table has one.
        match intersect_curve_surface(&conic, &s, tol(), &mut arris_math::Meter::default()) {
            Ok(r) => expect_points("the ellipse arm", hits_of(&r), &expected, EXACT)?,
            Err(GeomError::Unsupported { .. }) => {}
            Err(e) => return fail(e.to_string()),
        }
        let exact = Curve::Nurbs(rational_ellipse(frame, *major_radius, *minor_radius));
        let r = common_properties(&exact, &s)?;
        expect_points("the rational ellipse", hits_of(&r), &expected, EXACT)?;
        let fit = Curve::Nurbs(fitted(&conic));
        let r = common_properties(&fit, &s)?;
        expect_points("the fitted ellipse", hits_of(&r), &expected, FITTED)?;
        // The fit keeps the ellipse's own parameter.
        for (h, t) in hits_of(&r).iter().zip(&crossings) {
            let speed = conic.eval(*t).d1.norm();
            let d = (h.t - t).rem_euclid(TAU);
            prop_assert!(d.min(TAU - d) * speed <= FITTED, "{h:?} against t = {t}");
        }
        Ok(())
    });
}

/// A segment of a line as a NURBS whose parameter is the line's own,
/// from `lo` to `hi`: of degree one, or a rational quadratic with its
/// middle point weighted `w`, which keeps the image and bends the
/// parameter.
fn segment(l: &Curve, lo: f64, hi: f64, weight: Option<f64>) -> NurbsCurve {
    match weight {
        None => NurbsCurve::new(
            1,
            vec![lo, lo, hi, hi],
            vec![l.point(lo), l.point(hi)],
            vec![1.0; 2],
        ),
        Some(w) => NurbsCurve::new(
            2,
            vec![lo, lo, lo, hi, hi, hi],
            vec![l.point(lo), l.point(0.5 * (lo + hi)), l.point(hi)],
            vec![1.0, w, 1.0],
        ),
    }
    .unwrap()
}

/// How far a NURBS's hit may be from the line's own: [`EXACT`] over the
/// sine of the crossing angle, since a rounding of the surface's distance
/// moves a grazing crossing along the curve by that much more. A crossing
/// within [`GRAZE`] of tangent is held to that sine; the hit being on the
/// curve and on the surface is held to [`EXACT`] by `common_properties`
/// whatever the angle.
fn along(l: &Curve, s: &Surface, h: &CurveSurfaceHit) -> f64 {
    let Curve::Line { direction, .. } = l else {
        return EXACT;
    };
    let sine = s
        .normal(h.uv.x, h.uv.y)
        .map_or(1.0, |n| n.dot(direction).abs())
        .max(GRAZE);
    EXACT / sine
}

/// The sine below which a crossing is no better conditioned than this.
const GRAZE: f64 = 1e-6;

/// The straight-NURBS property's body: `s` is moved so that it meets the
/// line `l` at its point at `(u, v)`, and every NURBS that is the line
/// meets it where the line does.
fn straight_nurbs_meets_where_the_line_does(
    l: &Curve,
    s: &Surface,
    (u, v): (f64, f64),
    w: f64,
) -> Result<(), TestCaseError> {
    // The line through a point of the surface, the segment well
    // past every hit a bounded surface can have.
    let shift = l.point(0.0) - s.point(u, v);
    let s = s.transformed(&Isometry::from_translation(shift));
    let reach = 4.0 * DEFAULT_SCALE;
    let CurveSurfaceIntersection::Points(of_line) =
        intersect_curve_surface(l, &s, tol(), &mut arris_math::Meter::default())
            .map_err(|e| TestCaseError::fail(e.to_string()))?
    else {
        return Ok(());
    };
    let inside: Vec<&CurveSurfaceHit> = of_line.iter().filter(|h| h.t.abs() < reach).collect();
    if inside.iter().any(|h| h.tangent) {
        return Ok(());
    }
    let expected: Vec<(Point3, f64)> = inside.iter().map(|h| (h.point, along(l, &s, h))).collect();
    let straight = Curve::Nurbs(segment(l, -reach, reach, None));
    let r = common_properties(&straight, &s)?;
    expect_points_within("degree one", hits_of(&r), &expected)?;
    for (h, of_line) in hits_of(&r).iter().zip(&inside) {
        prop_assert!(
            (h.t - of_line.t).abs() <= along(l, &s, of_line),
            "{h:?} against {of_line:?}"
        );
    }
    let bent = Curve::Nurbs(segment(l, -reach, reach, Some(w)));
    let r = common_properties(&bent, &s)?;
    expect_points_within("rational", hits_of(&r), &expected)?;
    // One span of the highest degree there is, its control points
    // even along the line: against a torus, a polynomial of
    // degree a hundred.
    let p = MAX_DEGREE;
    let knots = [vec![-reach; p + 1], vec![reach; p + 1]].concat();
    let points = (0..=p)
        .map(|i| l.point(-reach + 2.0 * reach * i as f64 / p as f64))
        .collect();
    let tall = Curve::Nurbs(NurbsCurve::new(p, knots, points, vec![1.0; p + 1]).unwrap());
    let r = common_properties(&tall, &s)?;
    expect_points_within("the highest degree", hits_of(&r), &expected)
}

#[test]
fn a_straight_nurbs_meets_each_surface_where_the_line_does() {
    check(
        (
            line(),
            surface(),
            finite_f64(0.0..=TAU),
            finite_f64(-1.0..=1.0),
            finite_f64(0.5..=2.0),
        ),
        |(l, s, u, v, w)| straight_nurbs_meets_where_the_line_does(&l, &s, (u, v), w),
    );
}

/// The nightly of 2026-10-03: a line crossing a plane at a sine of 1e-4,
/// where the hit of the rational NURBS is 1.2e-10 along the line from the
/// line's own, the surface's distance there is rounding (1e-14).
#[test]
fn a_straight_nurbs_across_a_grazing_plane_is_held_to_the_crossings_conditioning() {
    let l = Curve::Line {
        origin: Point3::new(0.0, 0.0, 13.503956146623702),
        direction: UnitVec3::new_unchecked(Vec3::new(
            -0.14724798110694118,
            0.8237195066646757,
            -0.5475437940475933,
        )),
    };
    let frame = Frame::from_orthonormal(
        Point3::origin(),
        Vec3::new(-0.6207176274123941, 0.552843686608977, -0.5559437788087243),
        Vec3::new(-0.7260810910704453, -0.672882195737566, 0.14154787122858453),
        Vec3::new(
            -0.29583082362979063,
            0.49152152428554674,
            0.8190791872307173,
        ),
    )
    .unwrap();
    straight_nurbs_meets_where_the_line_does(
        &l,
        &Surface::Plane { frame },
        (0.0, 0.0),
        0.6602040923002235,
    )
    .unwrap();
}

#[test]
fn a_random_nurbs_crosses_a_surface_where_its_distance_changes_sign() {
    check(
        (
            nurbs_curve(),
            surface(),
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=TAU),
            finite_f64(-1.0..=1.0),
        ),
        |(n, s, at, u, v)| {
            let domain = n.domain();
            let shift = n.eval(domain.lerp(at)).point - s.point(u, v);
            let s = s.transformed(&Isometry::from_translation(shift));
            let c = Curve::Nurbs(n);
            let r = common_properties(&c, &s)?;
            let hits = hits_of(&r);
            let f = |t: f64| signed_distance(&s, c.point(t));
            let step = domain.length() / SAMPLES as f64;
            let mut previous = f(domain.lo());
            for i in 1..=SAMPLES {
                let t = domain.lo() + step * i as f64;
                let now = f(t);
                if (now < 0.0) != (previous < 0.0) {
                    prop_assert!(
                        hits.iter().any(|h| (h.t - t).abs() <= 2.0 * step),
                        "the distance changes sign before t = {t} but {r:?} has nothing there"
                    );
                }
                previous = now;
            }
            let Some(crossings) =
                sampled_crossings(&f, [domain.lo(), domain.hi()], reach_of(&c), false)
            else {
                return Ok(());
            };
            let expected: Vec<Point3> = crossings.iter().map(|&t| c.point(t)).collect();
            expect_points("a random curve", hits, &expected, EXACT)
        },
    );
}

/// One surface of each analytic kind about the world frame, with a
/// point of it whose neighbourhood is convex from outside: `(u, v)`.
fn convex_spots() -> Vec<(Surface, [f64; 2])> {
    let frame = Frame::world();
    vec![
        (Surface::Plane { frame }, [0.3, -0.2]),
        (Surface::Cylinder { frame, radius: 2.0 }, [0.7, 0.4]),
        (
            Surface::EllipticCylinder {
                frame,
                major_radius: 3.0,
                minor_radius: 2.0,
            },
            [0.9, -0.3],
        ),
        (
            Surface::Cone {
                frame,
                radius: 2.0,
                half_angle: 0.5,
            },
            [2.1, 0.6],
        ),
        (Surface::Sphere { frame, radius: 2.5 }, [1.2, 0.4]),
        (
            Surface::Torus {
                frame,
                major_radius: 3.0,
                minor_radius: 1.0,
            },
            [4.0, 0.0],
        ),
    ]
}

/// A circle of radius `r` outside `s`, in the plane of the normal and
/// `∂P/∂u` at `uv`, its nearest point `gap` above the surface there
/// (below, for a negative one), as an exact rational NURBS.
fn circle_over(s: &Surface, uv: [f64; 2], r: f64, gap: f64) -> NurbsCurve {
    let e = s.eval(uv[0], uv[1]);
    let n = s.normal(uv[0], uv[1]).unwrap().into_inner();
    let along = e.du.normalize();
    let centre = e.point + (r + gap) * n;
    let frame = Frame::from_orthonormal(centre, n, along, n.cross(&along)).unwrap();
    rational_ellipse(&frame, r, r)
}

#[test]
fn a_touch_is_one_tangent_hit_within_the_tolerance() {
    check(
        (pose(), finite_f64(0.2..=0.6), finite_f64(-0.5..=0.5)),
        |(motion, r, fraction)| {
            for (s, uv) in convex_spots() {
                let moved = s.transformed(&motion);
                let foot = motion.apply(s.point(uv[0], uv[1]));
                // Within the tolerance, above or below: one touch, there.
                let graze = circle_over(&s, uv, r, fraction * tol().linear).transformed(&motion);
                let result = common_properties(&Curve::Nurbs(graze), &moved)?;
                let hits = hits_of(&result);
                prop_assert_eq!(hits.len(), 1, "{:?}: {:?}", s.kind(), hits);
                prop_assert!(hits[0].tangent, "{:?}: {:?}", s.kind(), hits);
                // A touch `h` deep is a chord `2√(2rh)` long.
                let chord = 2.0 * (2.0 * r * tol().linear).sqrt();
                prop_assert!((hits[0].point - foot).norm() <= chord, "{:?}", s.kind());
                // Three tolerances clear: nothing.
                let clear = circle_over(&s, uv, r, 3.0 * tol().linear).transformed(&motion);
                let result = common_properties(&Curve::Nurbs(clear), &moved)?;
                prop_assert!(hits_of(&result).is_empty(), "{:?}: {result:?}", s.kind());
                prop_assert_ne!(&result, &CurveSurfaceIntersection::Coincident);
                // Three tolerances deep: two crossings either side.
                let deep = circle_over(&s, uv, r, -3.0 * tol().linear).transformed(&motion);
                let result = common_properties(&Curve::Nurbs(deep), &moved)?;
                let hits = hits_of(&result);
                prop_assert_eq!(hits.len(), 2, "{:?}: {:?}", s.kind(), hits);
                prop_assert!(hits.iter().all(|h| !h.tangent), "{:?}: {hits:?}", s.kind());
            }
            Ok(())
        },
    );
}

#[test]
fn a_curve_on_the_surface_is_coincident() {
    // The parallel through a point of each surface, exact; the same a
    // third of a tolerance larger; and three tolerances larger, clear.
    check((pose(), finite_f64(-0.3..=0.3)), |(motion, fraction)| {
        for (s, uv) in convex_spots() {
            let (frame, radii) = match s {
                Surface::Plane { frame } => (frame, [1.5, 1.0]),
                Surface::EllipticCylinder {
                    frame,
                    major_radius,
                    minor_radius,
                } => (frame, [major_radius, minor_radius]),
                _ => {
                    let p = s.point(uv[0], uv[1]);
                    let q = Frame::world().to_local(p);
                    let rho = q.x.hypot(q.y);
                    (
                        Frame::from_z(Point3::new(0.0, 0.0, q.z), Vec3::z()).unwrap(),
                        [rho, rho],
                    )
                }
            };
            let moved = s.transformed(&motion);
            let grown = |by: f64| {
                // A plane's curve leaves it along the normal instead.
                let lifted = match s {
                    Surface::Plane { .. } => {
                        Frame::from_z(Point3::new(0.0, 0.0, by), Vec3::z()).unwrap()
                    }
                    _ => frame,
                };
                let scale = match s {
                    Surface::Plane { .. } => 0.0,
                    _ => by,
                };
                Curve::Nurbs(
                    rational_ellipse(&lifted, radii[0] + scale, radii[1] + scale)
                        .transformed(&motion),
                )
            };
            for by in [0.0, fraction * tol().linear] {
                prop_assert_eq!(
                    intersect_curve_surface(
                        &grown(by),
                        &moved,
                        tol(),
                        &mut arris_math::Meter::default()
                    )
                    .unwrap(),
                    CurveSurfaceIntersection::Coincident,
                    "{:?} grown by {}",
                    s.kind(),
                    by
                );
            }
            // An elliptic section grown evenly in its radii is not an
            // offset of it, but it is clear of it all the same.
            let clear = common_properties(&grown(3.0 * tol().linear), &moved)?;
            prop_assert_eq!(
                clear,
                CurveSurfaceIntersection::Points(Vec::new()),
                "{:?}",
                s.kind()
            );
        }
        Ok(())
    });
}

#[test]
fn a_fitted_section_curve_is_coincident_with_both_of_its_surfaces() {
    let big = Surface::Cylinder {
        frame: Frame::world(),
        radius: 2.0,
    };
    let small = Surface::Cylinder {
        frame: Frame::from_z(Point3::new(0.3, 0.0, 0.1), Vec3::new(1.0, 0.2, 0.1)).unwrap(),
        radius: 1.0,
    };
    let within = Aabb {
        min: [-100.0; 3],
        max: [100.0; 3],
    };
    let SurfaceIntersection::Meets { curves, .. } = intersect_surfaces(
        &big,
        &small,
        &within,
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap() else {
        panic!("two crossing cylinders meet")
    };
    assert_eq!(curves.len(), 2);
    for c in &curves {
        assert!(matches!(c.curve, Curve::Nurbs(_)));
        assert!(c.curve.period().is_some());
        for s in [&big, &small] {
            assert_eq!(
                intersect_curve_surface(&c.curve, s, tol(), &mut arris_math::Meter::default())
                    .unwrap(),
                CurveSurfaceIntersection::Coincident
            );
        }
        // What the next boolean brings against the junction — a slot's
        // wall, a drill, and the rest of the table — each with a point
        // of the loop a quarter of a unit inside it, its axis across the
        // loop there. It cuts the loop where the loop's distance to it
        // changes sign, and every hit is on both cylinders within the
        // fit's own distance from them.
        let domain = c.curve.domain();
        let at = c.curve.eval(domain.lerp(0.3));
        let along = at.d1.normalize();
        let z = along.cross(&Vec3::new(0.3, 0.5, 0.8)).normalize();
        let side = z.cross(&along);
        let frame = Frame::from_z(at.point + 0.25 * side, z).unwrap();
        let thirds = [
            Surface::Plane {
                frame: Frame::from_z(at.point, along + 0.3 * z).unwrap(),
            },
            Surface::Cylinder { frame, radius: 0.4 },
            Surface::EllipticCylinder {
                frame,
                major_radius: 0.6,
                minor_radius: 0.35,
            },
            Surface::Cone {
                frame,
                radius: 0.5,
                half_angle: 0.2,
            },
            Surface::Sphere { frame, radius: 0.6 },
            Surface::Torus {
                frame: Frame::from_z(at.point + 1.8 * side + 0.2 * z, z).unwrap(),
                major_radius: 1.8,
                minor_radius: 0.5,
            },
        ];
        for third in &thirds {
            let f = |t: f64| signed_distance(third, c.curve.point(t));
            let crossings =
                sampled_crossings(&f, [domain.lo(), domain.hi()], reach_of(&c.curve), true)
                    .unwrap_or_else(|| panic!("{:?} is near a touch", third.kind()));
            assert!(crossings.len() >= 2, "{:?} misses the loop", third.kind());
            let r = common_properties(&c.curve, third).unwrap();
            let expected: Vec<Point3> = crossings.iter().map(|&t| c.curve.point(t)).collect();
            expect_points("a third face", hits_of(&r), &expected, EXACT).unwrap();
            for h in hits_of(&r) {
                assert!(signed_distance(&big, h.point).abs() <= tol().linear);
                assert!(signed_distance(&small, h.point).abs() <= tol().linear);
            }
        }
    }
}

#[test]
fn an_open_curves_end_on_the_surface_is_a_hit_and_no_touch() {
    let plane = Surface::Plane {
        frame: Frame::world(),
    };
    // A cubic that comes down onto the plane and stops there.
    let lands = |height: f64| {
        Curve::Nurbs(
            NurbsCurve::new(
                3,
                vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                vec![
                    Point3::new(0.0, 0.0, 2.0),
                    Point3::new(1.0, 0.5, 2.5),
                    Point3::new(2.0, -0.5, 1.0),
                    Point3::new(3.0, 0.0, height),
                ],
                vec![1.0; 4],
            )
            .unwrap(),
        )
    };
    for height in [0.0, 0.4 * tol().linear, -0.4 * tol().linear] {
        let CurveSurfaceIntersection::Points(hits) = intersect_curve_surface(
            &lands(height),
            &plane,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap() else {
            panic!()
        };
        assert_eq!(hits.len(), 1, "{hits:?}");
        assert_eq!(hits[0].t, 1.0);
        assert!(!hits[0].tangent);
    }
    // Short of it by more than the tolerance: nothing. Through it: the
    // crossing, before the end.
    assert_eq!(
        intersect_curve_surface(
            &lands(3.0 * tol().linear),
            &plane,
            tol(),
            &mut arris_math::Meter::default()
        )
        .unwrap(),
        CurveSurfaceIntersection::Points(Vec::new())
    );
    let CurveSurfaceIntersection::Points(hits) = intersect_curve_surface(
        &lands(-0.5),
        &plane,
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap() else {
        panic!()
    };
    assert_eq!(hits.len(), 1);
    assert!(hits[0].t < 1.0 && hits[0].point.z.abs() < 1e-15 && !hits[0].tangent);
    // An end that grazes — the distance's extremum inside the curve and
    // within the tolerance of its end — is one touch.
    let grazes = Curve::Nurbs(
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 1.0),
                Point3::new(1.0, 0.0, -1e-9),
                Point3::new(2.0, 0.0, 0.0),
            ],
            vec![1.0; 3],
        )
        .unwrap(),
    );
    let CurveSurfaceIntersection::Points(hits) =
        intersect_curve_surface(&grazes, &plane, tol(), &mut arris_math::Meter::default()).unwrap()
    else {
        panic!()
    };
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert!(hits[0].tangent);
}

#[test]
fn a_periodic_curves_hits_come_back_inside_its_domain() {
    // A fitted circle whose seam is on the plane: the crossing there is
    // found in the stretch that goes round, and comes back as a
    // parameter of the domain.
    let circle = Curve::Circle {
        frame: Frame::from_z(Point3::origin(), Vec3::y()).unwrap(),
        radius: 1.5,
    };
    let fit = Curve::Nurbs(fitted(&circle));
    let through_the_seam = Surface::Plane {
        frame: Frame::from_z(circle.point(0.0), circle.eval(0.0).d1).unwrap(),
    };
    let r = intersect_curve_surface(
        &fit,
        &through_the_seam,
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    let hits = hits_of(&r);
    assert_eq!(hits.len(), 2, "{hits:?}");
    let domain = fit.domain();
    for h in hits {
        assert!(h.t >= domain.lo() && h.t < domain.hi() && !h.tangent);
    }
    let from_seam = |t: f64| t.min(TAU - t);
    assert!(from_seam(hits[0].t) < 1e-9 || from_seam(hits[1].t) < 1e-9);
    assert!(hits.iter().any(|h| (h.t - TAU / 2.0).abs() < 1e-9));
}

#[test]
fn a_nurbs_curve_against_a_nurbs_surface_stays_unsupported() {
    check(
        (nurbs_curve(), arris_debug::prop::geom::nurbs_surface()),
        |(c, s)| {
            let r = intersect_curve_surface(
                &Curve::Nurbs(c),
                &Surface::Nurbs(s),
                tol(),
                &mut arris_math::Meter::default(),
            );
            let unsupported = matches!(r, Err(GeomError::Unsupported { .. }));
            prop_assert!(unsupported);
            Ok(())
        },
    );
}

/// Found by the `intersect_curve_surface` fuzz target (nightly 2026-09-28,
/// `fuzz/`, ADR-0024 §5): a closed clamped cubic whose first span is
/// 3.6e-304 wide — the curve jumps from its first control point to its
/// second inside it — crosses a plane inside that span. The stretch over
/// the join of a closed curve was bracketed across it, and the
/// bracket's bisection converged on the jump from the end to the first
/// parameter after the span the end's rounding can reach, 0.3 off the
/// plane. Each side of the join is solved alone now, and the hit is
/// the crossing, at a parameter of 1e-304.
#[test]
fn a_closed_curve_whose_first_span_is_a_hair_wide_meets_a_plane_inside_it() {
    let pts = [
        [
            -1.6428571428571428,
            0.035714285714285365,
            1.0714285714285716,
        ],
        [0.0857142857142863, 0.37857142857142856, 1.2571428571428573],
        [0.41428571428571503, 3.121428571428572, 0.24285714285714266],
        [-2.1714285714285713, 4.492857142857144, -0.5142857142857145],
        [-5.028571428571429, 3.2071428571428573, -0.08571428571428574],
        [-5.357142857142858, 0.46428571428571397, 0.9285714285714287],
        [-3.371428571428572, -0.30714285714285783, 0.8857142857142857],
        [
            -1.6428571428571428,
            0.035714285714285365,
            1.0714285714285716,
        ],
    ];
    let knots = vec![
        0.0,
        0.0,
        0.0,
        0.0,
        3.645561009778199e-304,
        2.000120162963867,
        3.0000038146972656,
        4.0,
        5.0,
        5.0,
        5.0,
        5.0,
    ];
    let curve = Curve::Nurbs(
        NurbsCurve::new(
            3,
            knots,
            pts.iter().map(|p| Point3::new(p[0], p[1], p[2])).collect(),
            vec![1.0; 8],
        )
        .unwrap(),
    );
    let frame = Frame::new(
        Point3::new(-2.357142857142857, 1.9642857142857144, 0.9285714285714286),
        Vec3::new(0.28571428571428575, 0.4285714285714286, 0.8571428571428572),
        Vec3::new(0.42857142857142855, -0.8571428571428571, 0.2857142857142857),
    )
    .unwrap();
    let surface = Surface::Plane { frame };
    let r = intersect_curve_surface(&curve, &surface, tol(), &mut arris_math::Meter::default())
        .unwrap();
    let hits = hits_of(&r);
    for h in hits {
        let off = (h.point - frame.origin()).dot(&frame.z()).abs();
        assert!(off <= tol().linear, "t = {:e}: {off:e} off the plane", h.t);
    }
    assert!(hits.iter().any(|h| h.t < 1e-300), "{hits:?}");
    assert_eq!(hits.len(), 2, "{hits:?}");
}
