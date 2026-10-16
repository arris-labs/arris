//! A line against a NURBS surface: an exact NURBS twin of every analytic
//! surface meets a line where the surface does, hit for hit; a random
//! NURBS meets it wherever a dense grid of the surface says the line
//! pierces it; a line through a pole is one hit there; a graze within
//! the tolerance is one tangent hit on either side of the surface and
//! nothing past it; a line along a ruling is `Coincident`; a crossing on
//! a knot line or a seam is one hit; and every hit lies on both.

use arris_math::real;
use core::f64::consts::PI;

use arris_debug::prop::geom::{
    cone, cylinder, elliptic_cylinder, nurbs_surface, plane, sphere, torus,
};
use arris_debug::prop::{DEFAULT_SCALE, finite_f64, unit_vec3};
use arris_geom::{
    Curve, CurveSurfaceHit, CurveSurfaceIntersection, NurbsSurface, Surface,
    intersect_curve_surface,
};
use arris_math::{Frame, Interval, Meter, Point2, Point3, Precision, Tolerance, UnitVec3, Vec3};
use proptest::prelude::*;

/// A transversal hit against both operands, in length units.
const EXACT: f64 = 1e-12 * DEFAULT_SCALE;
/// How far from the edge of a twin's bounds, in its parameters, a hit is
/// compared: nearer, the twin's domain decides alone.
const EDGE: f64 = 1e-6;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

fn line(o: [f64; 3], d: [f64; 3]) -> Curve {
    Curve::Line {
        origin: Point3::new(o[0], o[1], o[2]),
        direction: UnitVec3::new_normalize(Vec3::new(d[0], d[1], d[2])),
    }
}

fn meet(c: &Curve, s: &Surface) -> CurveSurfaceIntersection {
    intersect_curve_surface(c, s, tol(), &mut Meter::default())
        .unwrap_or_else(|e| panic!("{c:?} vs {s:?}: {e}"))
}

fn hits(c: &Curve, s: &Surface) -> Vec<CurveSurfaceHit> {
    match meet(c, s) {
        CurveSurfaceIntersection::Points(h) => h,
        CurveSurfaceIntersection::Coincident => panic!("{c:?} vs {s:?}: coincident"),
    }
}

fn nurbs(s: &NurbsSurface) -> Surface {
    Surface::Nurbs(s.clone())
}

/// Every hit lies on the line at its `t` and on the surface at its `uv`
/// — a transversal one to [`EXACT`], a touch within the tolerance — and
/// its `uv` is in the domain.
fn on_both(c: &Curve, s: &NurbsSurface, h: &CurveSurfaceHit) -> Result<(), String> {
    let on_line = (c.point(h.t) - h.point).norm();
    let on_surface = (s.eval(h.uv.x, h.uv.y).point - h.point).norm();
    let bound = if h.tangent { tol().linear } else { EXACT };
    if on_line > EXACT || on_surface > bound {
        return Err(format!(
            "{h:?}: {on_line} off the line, {on_surface} off the surface"
        ));
    }
    let [du, dv] = s.domain();
    if !(du.contains(h.uv.x) && dv.contains(h.uv.y)) {
        return Err(format!("{h:?}: outside the domain {du:?} × {dv:?}"));
    }
    Ok(())
}

// --- the twins ----------------------------------------------------------------

/// The bounds a twin of `s` is made over, in its own parameters: a full
/// turn of an angle, and a stretch of a length either side of the frame.
fn bounds(s: &Surface) -> [Interval; 2] {
    let span = |a: f64| Interval::new(-a, a).unwrap();
    match s {
        Surface::Plane { .. } => [span(10.0), span(10.0)],
        Surface::Cylinder { .. } | Surface::EllipticCylinder { .. } => [Interval::TURN, span(10.0)],
        // Short of the apex, which is at `−R / sin α` along the ruling.
        Surface::Cone {
            radius, half_angle, ..
        } => [
            Interval::TURN,
            Interval::new(-0.9 * radius / real::sin(*half_angle), 10.0).unwrap(),
        ],
        Surface::Sphere { .. } | Surface::Torus { .. } => s.domain(),
        Surface::Nurbs(_) => unreachable!("the strategy draws analytic surfaces"),
    }
}

/// Whether an analytic hit at `uv` is inside the twin's bounds by more
/// than [`EDGE`] in each bounded parameter.
fn well_inside(s: &Surface, b: [Interval; 2], uv: Point2) -> bool {
    let inside = |i: Interval, x: f64| i.lo() + EDGE < x && x < i.hi() - EDGE;
    match s {
        Surface::Plane { .. } => inside(b[0], uv.x) && inside(b[1], uv.y),
        Surface::Cylinder { .. } | Surface::EllipticCylinder { .. } | Surface::Cone { .. } => {
            inside(b[1], uv.y)
        }
        Surface::Sphere { .. } | Surface::Torus { .. } => true,
        Surface::Nurbs(_) => unreachable!("the strategy draws analytic surfaces"),
    }
}

/// How far two crossings of one surface by one line may be apart: a
/// point off the surface by [`EXACT`] is off it along the line by that
/// over the sine of the crossing angle, which the surface's normal at an
/// analytic hit gives.
fn along_line(s: &Surface, c: &Curve, h: &CurveSurfaceHit) -> f64 {
    let Curve::Line { direction, .. } = c else {
        unreachable!("the cases are lines")
    };
    let e = s.eval(h.uv.x, h.uv.y);
    let sine =
        e.du.cross(&e.dv)
            .try_normalize(0.0)
            .map_or(1.0, |n| n.dot(direction).abs());
    EXACT / sine.max(EXACT)
}

/// Whether the line stays within the tolerance of the analytic surface
/// from its point at `a` to its point at `b`, at 32 points between: two
/// hits it runs between so are one run of the line along the surface.
/// Where the line runs nearly along a direction the surface hardly
/// curves along, the distance has several extrema within the tolerance
/// along one long run, and the arms may report its touch at different
/// ones of them; a crossing in the run is one a touch may absorb.
fn joined(s: &Surface, c: &Curve, a: f64, b: f64) -> bool {
    (1..32).all(|i| {
        let t = a + (b - a) * i as f64 / 32.0;
        s.project(c.point(t))
            .is_ok_and(|pr| pr.distance <= tol().linear + EXACT)
    })
}

/// The twin's hits against the analytic surface's: every analytic touch
/// well inside the bounds has a twin's touch the line runs to it from
/// within the tolerance, every analytic crossing a twin's crossing — to
/// [`EXACT`] over the crossing's sine — or such a touch; and every twin's
/// hit away from the bounds' edge is one of those.
fn same_hits(
    s: &Surface,
    b: [Interval; 2],
    c: &Curve,
    ours: &[CurveSurfaceHit],
    theirs: &[CurveSurfaceHit],
) -> Result<(), String> {
    let touched = |t: f64| {
        ours.iter()
            .any(|o| o.tangent && ((o.t - t).abs() <= tol().linear.sqrt() || joined(s, c, o.t, t)))
    };
    let crossing = |a: &CurveSurfaceHit, b: &CurveSurfaceHit| {
        !a.tangent && !b.tangent && (a.point - b.point).norm() <= along_line(s, c, b)
    };
    for h in theirs.iter().filter(|h| well_inside(s, b, h.uv)) {
        let found = if h.tangent {
            touched(h.t)
        } else {
            ours.iter().any(|o| crossing(o, h)) || touched(h.t)
        };
        if !found {
            return Err(format!(
                "the surface's {h:?} is not among the twin's {ours:?} ({s:?} {c:?})"
            ));
        }
    }
    for o in ours {
        let by_edge = s.project(o.point).is_ok_and(|pr| !well_inside(s, b, pr.uv));
        let found = if o.tangent {
            theirs.iter().any(|h| {
                h.tangent && ((o.t - h.t).abs() <= tol().linear.sqrt() || joined(s, c, o.t, h.t))
            })
        } else {
            theirs.iter().any(|h| crossing(o, h))
        };
        if !(by_edge || found) {
            return Err(format!(
                "the twin's {o:?} is not among the surface's {theirs:?} ({s:?} {c:?})"
            ));
        }
    }
    Ok(())
}

fn analytic() -> impl Strategy<Value = Surface> {
    prop_oneof![
        plane(),
        cylinder(),
        elliptic_cylinder(),
        cone(),
        sphere(),
        torus()
    ]
}

/// A surface, a point of its twin's bounds as fractions, a direction and
/// how far back along it the line's origin is.
fn twin_case() -> impl Strategy<Value = (Surface, [f64; 2], UnitVec3, f64)> {
    (
        analytic(),
        [finite_f64(0.0..=1.0), finite_f64(0.0..=1.0)],
        unit_vec3(),
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
    )
}

arris_debug::prop_shards! {
    /// A twin meets a line where its analytic surface does: every
    /// analytic hit well inside the bounds is a twin's hit — a crossing
    /// to [`EXACT`], a touch within the square root of the tolerance, as
    /// a touch is located — and every twin's hit is an analytic one.
    a_twin_meets_a_line_where_its_surface_does [shard_0 shard_1 shard_2 shard_3]
        ((s, at, d, back)) = twin_case() => {
            let b = bounds(&s);
            let twin = s.to_nurbs(b).unwrap();
            let p = s.eval(b[0].lerp(at[0]), b[1].lerp(at[1])).point;
            let c = Curve::Line { origin: p - d.into_inner() * back, direction: d };
            let (ours, theirs) = match (meet(&c, &nurbs(&twin)), meet(&c, &s)) {
                (CurveSurfaceIntersection::Points(a), CurveSurfaceIntersection::Points(b)) => {
                    (a, b)
                }
                (CurveSurfaceIntersection::Coincident, CurveSurfaceIntersection::Coincident) => {
                    return Ok(());
                }
                (a, b) => {
                    return Err(TestCaseError::fail(format!("twin {a:?}, surface {b:?}")));
                }
            };
            for h in &ours {
                on_both(&c, &twin, h).map_err(TestCaseError::fail)?;
            }
            same_hits(&s, b, &c, &ours, &theirs).map_err(TestCaseError::fail)?;
            Ok(())
        }
}

/// A surface, a point of its twin's bounds as fractions, an angle in the
/// tangent plane there and an offset along the normal, in tolerances.
fn graze_case() -> impl Strategy<Value = (Surface, [f64; 2], f64, f64)> {
    (
        analytic(),
        [finite_f64(0.05..=0.95), finite_f64(0.05..=0.95)],
        finite_f64(0.0..=core::f64::consts::TAU),
        prop_oneof![Just(0.0), Just(0.5), Just(-0.5), Just(2.0), Just(-2.0)],
    )
}

arris_debug::prop_shards! {
    /// A line in the tangent plane of a twin, at a point in a random
    /// direction, moved off it along the normal by a fraction of the
    /// tolerance or by two: the twin says what the analytic surface says
    /// — `Coincident`, or the same hits with the same `tangent` flags, a
    /// touch anywhere in the run of the line within the tolerance — but
    /// where the line runs nearly along a direction the surface hardly
    /// curves along, only that each arm's hits are in runs the other's
    /// are.
    a_twin_is_grazed_where_its_surface_is [shard_0 shard_1 shard_2 shard_3]
        ((s, at, angle, off)) = graze_case() => {
            let b = bounds(&s);
            let twin = s.to_nurbs(b).unwrap();
            let (u, v) = (b[0].lerp(at[0]), b[1].lerp(at[1]));
            let e = s.eval(u, v);
            let Some(n) = e.du.cross(&e.dv).try_normalize(0.0) else {
                return Ok(());
            };
            let x = e.du.normalize();
            let y = n.cross(&x);
            let d = x * real::cos(angle) + y * real::sin(angle);
            let p = e.point + n * (off * tol().linear);
            let c = Curve::Line { origin: p - d * 5.0, direction: UnitVec3::new_normalize(d) };
            let (ours, theirs) = match (meet(&c, &nurbs(&twin)), meet(&c, &s)) {
                (CurveSurfaceIntersection::Points(a), CurveSurfaceIntersection::Points(b)) => {
                    (a, b)
                }
                (CurveSurfaceIntersection::Coincident, CurveSurfaceIntersection::Coincident) => {
                    return Ok(());
                }
                (a, b) => {
                    return Err(TestCaseError::fail(format!("twin {a:?}, surface {b:?}")));
                }
            };
            for h in &ours {
                on_both(&c, &twin, h).map_err(TestCaseError::fail)?;
            }
            // The surface's normal curvature along the line against its
            // largest there: where the line runs nearly along a direction
            // the surface hardly curves along, it stays within the
            // tolerance over a long run, which the arms may split into
            // touches and crossings differently.
            let form = |a: f64, b: f64| {
                let w = e.du * a + e.dv * b;
                n.dot(&(e.duu * (a * a) + e.duv * (2.0 * a * b) + e.dvv * (b * b))) / w.norm_squared()
            };
            let g = [e.du.dot(&e.du), e.du.dot(&e.dv), e.dv.dot(&e.dv)];
            let det = g[0] * g[2] - g[1] * g[1];
            let coords = |w: Vec3| {
                let r = [e.du.dot(&w), e.dv.dot(&w)];
                ((g[2] * r[0] - g[1] * r[1]) / det, (g[0] * r[1] - g[1] * r[0]) / det)
            };
            let along = coords(d);
            let largest = (0..32)
                .map(|i| {
                    let phi = core::f64::consts::PI * i as f64 / 32.0;
                    let w = coords(x * real::cos(phi) + y * real::sin(phi));
                    form(w.0, w.1).abs()
                })
                .fold(0.0, f64::max);
            if form(along.0, along.1).abs() < FLAT_ALONG * largest {
                return same_runs(&s, b, &c, &ours, &theirs).map_err(TestCaseError::fail);
            }
            same_hits(&s, b, &c, &ours, &theirs).map_err(TestCaseError::fail)?;
            Ok(())
        }
}

/// The fraction of the surface's largest normal curvature below which a
/// line runs along a direction it hardly curves along: the run within
/// the tolerance is then `√(1 / FLAT_ALONG)` times longer than across,
/// more than four times.
const FLAT_ALONG: f64 = 0.05;

/// The twin's hits against the analytic surface's where the line runs
/// nearly along a direction the surface hardly curves along: every hit
/// of either well inside the bounds is in a run of the line within the
/// tolerance that holds a hit of the other.
fn same_runs(
    s: &Surface,
    b: [Interval; 2],
    c: &Curve,
    ours: &[CurveSurfaceHit],
    theirs: &[CurveSurfaceHit],
) -> Result<(), String> {
    let near = |a: &CurveSurfaceHit, others: &[CurveSurfaceHit]| {
        others
            .iter()
            .any(|o| (o.t - a.t).abs() <= tol().linear.sqrt() || joined(s, c, o.t, a.t))
    };
    for h in theirs.iter().filter(|h| well_inside(s, b, h.uv)) {
        if !near(h, ours) {
            return Err(format!(
                "the surface's {h:?} is in no run of the twin's {ours:?} ({s:?} {c:?})"
            ));
        }
    }
    for o in ours {
        let by_edge = s.project(o.point).is_ok_and(|pr| !well_inside(s, b, pr.uv));
        if !(by_edge || near(o, theirs)) {
            return Err(format!(
                "the twin's {o:?} is in no run of the surface's {theirs:?} ({s:?} {c:?})"
            ));
        }
    }
    Ok(())
}

// --- random NURBS against a grid ----------------------------------------------

/// Cells of the grid per unit span of a random NURBS.
const CELLS: usize = 40;
/// How far inside its flat triangle, in barycentric weight, the line has
/// to pierce the grid for the piercing to be a start Newton's iteration
/// converges from to the crossing there rather than to a neighbour's.
const INSIDE: f64 = 0.15;
/// How steeply the line has to cross a triangle, as the cosine to its
/// normal, for the piercing to be a crossing and not a near graze.
const STEEP: f64 = 0.2;

fn grid_case() -> impl Strategy<Value = (NurbsSurface, [f64; 2], UnitVec3)> {
    (
        nurbs_surface(),
        [finite_f64(0.0..=1.0), finite_f64(0.0..=1.0)],
        unit_vec3(),
    )
}

arris_debug::prop_shards! {
    /// A random line through a random NURBS: every hit is on both, and
    /// wherever the line pierces a dense grid of the surface's flat
    /// triangles steeply and well inside one, the crossing Newton's
    /// iteration on the surface converges to from there is a hit, or
    /// inside a touch — the brute force that no crossing is missed.
    a_random_nurbs_is_met_where_its_grid_is_pierced [shard_0 shard_1 shard_2 shard_3]
        ((s, at, d)) = grid_case() => {
            let [du, dv] = s.domain();
            let p = s.eval(du.lerp(at[0]), dv.lerp(at[1])).point;
            let c = Curve::Line { origin: p - d.into_inner() * DEFAULT_SCALE, direction: d };
            let found = match meet(&c, &nurbs(&s)) {
                CurveSurfaceIntersection::Points(h) => h,
                CurveSurfaceIntersection::Coincident => {
                    return Err(TestCaseError::fail("a random line coincident".to_string()));
                }
            };
            for h in &found {
                on_both(&c, &s, h).map_err(TestCaseError::fail)?;
            }
            prop_assert!(
                found.iter().any(|h| (h.point - p).norm() <= EXACT),
                "the point the line was drawn through is not among {:?}", found
            );
            let o = c.point(0.0);
            let frame = Frame::from_z(o, d.into_inner()).unwrap();
            let (n1, n2) = (frame.x().into_inner(), frame.y().into_inner());
            let flat = |q: Point3| [n1.dot(&(q - o)), n2.dot(&(q - o))];
            let (nu, nv) = (CELLS * du.length() as usize, CELLS * dv.length() as usize);
            let node = |i: usize, j: usize| {
                let uv = [du.lerp(i as f64 / nu as f64), dv.lerp(j as f64 / nv as f64)];
                (s.eval(uv[0], uv[1]).point, uv)
            };
            for i in 0..nu {
                for j in 0..nv {
                    let corners = [node(i, j), node(i + 1, j), node(i + 1, j + 1), node(i, j + 1)];
                    for tri in [[0, 1, 2], [0, 2, 3]] {
                        let [a, b, e] = tri.map(|k| corners[k].0);
                        let normal = (b - a).cross(&(e - a));
                        if normal.norm() == 0.0 || normal.normalize().dot(&d).abs() < STEEP {
                            continue;
                        }
                        let [fa, fb, fe] = [flat(a), flat(b), flat(e)];
                        let area = (fb[0] - fa[0]) * (fe[1] - fa[1])
                            - (fe[0] - fa[0]) * (fb[1] - fa[1]);
                        let weight = |p: [f64; 2], q: [f64; 2]| (p[0] * q[1] - p[1] * q[0]) / area;
                        let w = [weight(fb, fe), weight(fe, fa), weight(fa, fb)];
                        if w.iter().any(|x| *x < INSIDE) {
                            continue;
                        }
                        // Newton's iteration on the surface itself from
                        // the piercing: where it converges onto the line,
                        // that crossing is a hit — or inside a touch.
                        let [ua, ub, ue] = tri.map(|k| corners[k].1);
                        let start = [0, 1].map(|k| w[0] * ua[k] + w[1] * ub[k] + w[2] * ue[k]);
                        let Some(zero) = newton(&s, &c, start) else {
                            continue;
                        };
                        let near = found.iter().any(|h| {
                            let bound = if h.tangent { tol().linear.sqrt() } else { 1e3 * EXACT };
                            (h.point - zero).norm() <= bound
                        });
                        prop_assert!(
                            near,
                            "the grid is pierced at {:?} and the crossing {:?} there is not among {:?}",
                            start, zero, found
                        );
                    }
                }
            }
            Ok(())
        }
}

/// Newton's iteration on the two offsets of the surface's point from the
/// line, from `start`, held in the domain: the point it converges to on
/// the line, to [`EXACT`], if it does in a few dozen steps.
fn newton(s: &NurbsSurface, c: &Curve, start: [f64; 2]) -> Option<Point3> {
    let Curve::Line { origin, direction } = c else {
        unreachable!("the cases are lines")
    };
    let frame = Frame::from_z(*origin, direction.into_inner()).ok()?;
    let n = [frame.x().into_inner(), frame.y().into_inner()];
    let [du, dv] = s.domain();
    let mut x = start;
    for _ in 0..48 {
        let e = s.eval(x[0], x[1]);
        let f = n.map(|n| n.dot(&(e.point - origin)));
        if real::hypot(f[0], f[1]) <= EXACT {
            return Some(e.point);
        }
        let j = n.map(|n| [n.dot(&e.du), n.dot(&e.dv)]);
        let det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
        if det == 0.0 {
            return None;
        }
        let step = [
            (j[1][1] * f[0] - j[0][1] * f[1]) / det,
            (j[0][0] * f[1] - j[1][0] * f[0]) / det,
        ];
        x = [
            (x[0] - step[0]).clamp(du.lo(), du.hi()),
            (x[1] - step[1]).clamp(dv.lo(), dv.hi()),
        ];
    }
    None
}

// --- the cases ----------------------------------------------------------------

fn sphere_twin(radius: f64) -> NurbsSurface {
    let ball = Surface::Sphere {
        frame: Frame::world(),
        radius,
    };
    ball.to_nurbs(ball.domain()).unwrap()
}

/// A line through both poles of a sphere's twin is two hits, one at each
/// pole, at the poles' own parameters: `u` at the start of its knots. A
/// line along the tangent plane at a pole touches it there.
#[test]
fn a_line_through_the_poles_is_a_hit_at_each() {
    let twin = sphere_twin(2.0);
    let c = line([0.0, 0.0, -5.0], [0.0, 0.0, 1.0]);
    let h = hits(&c, &nurbs(&twin));
    assert_eq!(h.len(), 2, "{h:?}");
    let [du, dv] = twin.domain();
    for (hit, v, t) in [(&h[0], dv.lo(), 3.0), (&h[1], dv.hi(), 7.0)] {
        assert!(!hit.tangent, "{hit:?}");
        assert_eq!(hit.uv, Point2::new(du.lo(), v));
        assert!((hit.t - t).abs() <= EXACT, "{hit:?}");
    }
    let c = line([-5.0, -1.5, 2.0], [1.0, 0.3, 0.0]);
    let h = hits(&c, &nurbs(&twin));
    assert_eq!(h.len(), 1, "{h:?}");
    assert!(h[0].tangent, "{h:?}");
    assert!(
        (h[0].point - Point3::new(0.0, 0.0, 2.0)).norm() <= tol().linear.sqrt(),
        "{h:?}"
    );
}

/// A line passing a pole closer than the tolerance, or farther, crosses
/// the sphere beside it, transversally.
#[test]
fn a_line_beside_a_pole_crosses_beside_it() {
    let twin = sphere_twin(2.0);
    for off in [1e-12, 1e-9, 1e-7, 1e-3] {
        let c = line([off, 0.0, -5.0], [0.0, 0.0, 1.0]);
        let h = hits(&c, &nurbs(&twin));
        assert_eq!(h.len(), 2, "{off}: {h:?}");
        for hit in &h {
            assert!(!hit.tangent, "{off}: {hit:?}");
            on_both(&c, &twin, hit).unwrap();
        }
    }
}

/// A line grazing a sphere's twin within the tolerance is one tangent
/// hit at the nearest approach, whether it passes inside — where it
/// crosses twice, `2√(2Rδ)` apart — or outside; two tolerances out,
/// nothing. First where the twin's equator and a meridian seam meet, a
/// corner of four patches, then inside a patch in a general direction.
#[test]
fn a_graze_within_the_tolerance_is_one_touch_and_past_it_none() {
    let tau = tol().linear;
    let twin = sphere_twin(2.0);
    let normal = Vec3::new(0.3, -0.5, 0.7).normalize();
    let across = normal.cross(&Vec3::new(1.0, 2.0, 0.5)).normalize();
    let poses = [(Vec3::y(), Vec3::x()), (normal, across)];
    for (n, along) in poses {
        for (off, count) in [(-0.5, 1), (0.0, 1), (0.5, 1), (2.0, 0)] {
            let p = Point3::from(n * (2.0 + off * tau));
            let c = Curve::Line {
                origin: p - along * 5.0,
                direction: UnitVec3::new_normalize(along),
            };
            let h = hits(&c, &nurbs(&twin));
            assert_eq!(h.len(), count, "{n:?} {off}: {h:?}");
            if let Some(hit) = h.first() {
                assert!(hit.tangent, "{n:?} {off}: {hit:?}");
                assert!((hit.t - 5.0).abs() <= tau.sqrt(), "{n:?} {off}: {hit:?}");
                on_both(&c, &twin, hit).unwrap();
            }
        }
    }
}

/// A line along a ruling of a cylinder's twin, or within the tolerance
/// of one, is `Coincident`; two tolerances off it, nothing. So is a line
/// in a plane's twin.
#[test]
fn a_line_along_a_ruling_is_coincident() {
    let tau = tol().linear;
    let wall = Surface::Cylinder {
        frame: Frame::world(),
        radius: 2.0,
    };
    let twin = wall
        .to_nurbs([Interval::TURN, Interval::new(-3.0, 3.0).unwrap()])
        .unwrap();
    // On a seam of the twin's patches, and inside one.
    for angle in [0.0, 0.4] {
        let (sin, cos) = real::sin_cos(angle);
        for (off, coincident) in [(0.0, true), (0.5, true), (-0.5, true), (2.0, false)] {
            let r = 2.0 + off * tau;
            let c = line([r * cos, r * sin, -5.0], [0.0, 0.0, 1.0]);
            match meet(&c, &nurbs(&twin)) {
                CurveSurfaceIntersection::Coincident => assert!(coincident, "{angle} {off}"),
                CurveSurfaceIntersection::Points(h) => {
                    assert!(!coincident && h.is_empty(), "{angle} {off}: {h:?}")
                }
            }
        }
    }
    let floor = Surface::Plane {
        frame: Frame::world(),
    };
    let unit = Interval::new(-1.0, 1.0).unwrap();
    let flat = floor.to_nurbs([unit, unit]).unwrap();
    let c = line([-5.0, -0.5, 0.0], [1.0, 0.1, 0.0]);
    assert_eq!(
        meet(&c, &nurbs(&flat)),
        CurveSurfaceIntersection::Coincident
    );
}

/// A bicubic of two spans each way, crossed exactly on a knot line, on
/// the other and where they meet: one hit each, at the knot, not one per
/// patch.
#[test]
fn a_crossing_on_a_knot_line_is_one_hit() {
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0];
    let mut points = Vec::new();
    for i in 0..5 {
        for j in 0..5 {
            let (x, y) = (i as f64, j as f64);
            points.push(Point3::new(x, y, 0.3 * (x - 2.0) * (y - 2.0) + 0.1 * x));
        }
    }
    let s = NurbsSurface::new([3, 3], [knots.clone(), knots], points, vec![1.0; 25]).unwrap();
    for at in [[1.0, 0.4], [0.7, 1.0], [1.0, 1.0]] {
        let e = s.eval(at[0], at[1]);
        let n = e.du.cross(&e.dv).normalize();
        let d = (n + 0.3 * e.du.normalize()).normalize();
        let c = Curve::Line {
            origin: e.point - d * 3.0,
            direction: UnitVec3::new_normalize(d),
        };
        let h = hits(&c, &nurbs(&s));
        assert_eq!(h.len(), 1, "{at:?}: {h:?}");
        assert!(!h[0].tangent, "{at:?}: {h:?}");
        assert!(
            (h[0].uv.x - at[0]).abs() <= 1e-12 && (h[0].uv.y - at[1]).abs() <= 1e-12,
            "{at:?}: {h:?}"
        );
        on_both(&c, &s, &h[0]).unwrap();
    }
}

/// A line through a closed twin's seam is one hit there, at the start of
/// the closed parameter.
#[test]
fn a_crossing_on_a_seam_is_one_hit_at_the_start() {
    let wall = Surface::Cylinder {
        frame: Frame::world(),
        radius: 2.0,
    };
    let twin = wall
        .to_nurbs([Interval::TURN, Interval::new(-3.0, 3.0).unwrap()])
        .unwrap();
    let c = line([5.0, 0.0, 0.5], [-1.0, 0.0, 0.2]);
    let h = hits(&c, &nurbs(&twin));
    assert_eq!(h.len(), 2, "{h:?}");
    let seam = h.iter().find(|h| h.point.x > 0.0).unwrap();
    assert_eq!(seam.uv.x, twin.domain()[0].lo(), "{seam:?}");
    assert!(h.iter().any(|h| (h.uv.x - PI).abs() <= 1e-12), "{h:?}");
}

/// A torus's twin, crossed through both sides of its tube, is four
/// crossings; along its axis, none.
#[test]
fn a_line_through_a_torus_twin_crosses_its_tube_four_times() {
    let ring = Surface::Torus {
        frame: Frame::world(),
        major_radius: 3.0,
        minor_radius: 1.0,
    };
    let twin = ring.to_nurbs(ring.domain()).unwrap();
    let h = hits(&line([-5.0, 0.0, 0.2], [1.0, 0.0, 0.0]), &nurbs(&twin));
    assert_eq!(h.len(), 4, "{h:?}");
    assert!(h.iter().all(|h| !h.tangent), "{h:?}");
    assert!(hits(&line([0.0, 0.0, -5.0], [0.0, 0.0, 1.0]), &nurbs(&twin)).is_empty());
}
