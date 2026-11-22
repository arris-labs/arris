//! The trimmed NURBS face's tight box against dense samples refined to
//! the extremes: outer, and within `eps` of the face on every side.

use super::{TrimDomain, nurbs_face_box};

use arris_check::Level;
use arris_debug::prop::{check, finite_f64, frame};
use arris_debug::sample;
use arris_geom::{Curve, Curve2, NurbsCurve, NurbsSurface, Surface, fit_curve_periodic};
use arris_math::{
    Aabb, Control, Frame2, Handedness, Interval, Meter, Point2, Point3, Vec2, Vec3, real,
};
use arris_topo::entity::{
    Body as BodyEntity, BodyKind, Coedge, Edge, EdgeGeometry, Face, Loop, Shell, Vertex,
};
use arris_topo::{Body, Face as FaceHandle, FaceId, Model, Orientation, Shell as ShellHandle};
use core::f64::consts::TAU;
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

#[test]
fn tight_bounds_split_the_other_direction_when_a_midpoint_stalls() {
    let u = Interval::new(1.0, 1.0_f64.next_up()).unwrap();
    // Give u two degrees so the one-span linear ruling proof does not
    // bypass subdivision. There is still no representable midpoint in u.
    let points = [0.0, 0.5, 1.0]
        .into_iter()
        .flat_map(|x| {
            [0.0, 2.0, 0.0]
                .into_iter()
                .enumerate()
                .map(move |(j, z)| Point3::new(x, j as f64 / 2.0, z))
        })
        .collect();
    let surface = NurbsSurface::new(
        [2, 2],
        [
            vec![u.lo(), u.lo(), u.lo(), u.hi(), u.hi(), u.hi()],
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        ],
        points,
        vec![1.0; 9],
    )
    .unwrap();
    let mut m = Model::default();
    let body = sample::patch(&mut m, Surface::Nurbs(surface), u, Interval::UNIT).unwrap();
    let report = arris_check::check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report:?}");
    let got = crate::query::tight_bounds(&m, body.id.into(), &Control::NONE)
        .unwrap()
        .unwrap();
    let eps = m.precision().default_tolerance;
    // z = 4v(1-v) has a maximum of 1 at v = 1/2 for every u.
    assert!(
        got.max[2] >= 1.0 - SLACK && got.max[2] <= 1.0 + eps,
        "{got:?}"
    );
}

#[test]
fn tight_bounds_names_the_face_when_the_subdivision_limit_is_reached() {
    // x = u, y = v. On a narrow interior knot rectangle, the height
    // in normalized local parameters is 16s(1-s)t(1-t), with maximum 1.
    // The rest of the surface has z = 0. The hull reaches 4 at the cap.
    let centre = 1.0 / 3.0;
    let half_width = real::powi(2.0, -40);
    let (lo, hi) = (centre - half_width, centre + half_width);
    let knots = vec![0.0, 0.0, 0.0, lo, lo, hi, hi, 1.0, 1.0, 1.0];
    // Greville abscissae give x = u and y = v.
    let values: Vec<_> = knots
        .windows(4)
        .map(|span| f64::midpoint(span[1], span[2]))
        .collect();
    let points = values
        .iter()
        .flat_map(|&u| {
            values.iter().map(move |&v| {
                let z = if u == centre && v == centre { 4.0 } else { 0.0 };
                Point3::new(u, v, z)
            })
        })
        .collect();
    let surface = NurbsSurface::new([2, 2], [knots.clone(), knots], points, vec![1.0; 49]).unwrap();
    assert!((surface.eval(centre, centre).point.z - 1.0).abs() <= SLACK);
    let mut m = Model::default();
    let body = sample::patch(
        &mut m,
        Surface::Nurbs(surface),
        Interval::UNIT,
        Interval::UNIT,
    )
    .unwrap();
    let report = arris_check::check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report:?}");
    let face = m.faces(body).unwrap()[0];
    assert_eq!(
        crate::query::tight_bounds(&m, body.id.into(), &Control::NONE),
        Err(crate::OpError::Degenerate {
            entities: vec![face.shape()],
            reason: crate::Reason::Query(crate::QueryReason::BoundsUncertified),
        })
    );
}

#[test]
fn tight_bounds_find_the_peak_across_a_clamped_revolution_seam() {
    // Radius r(v) = 2 + 4v(1-v), with the seam at angle 1/4. The point
    // at angle zero and v = 1/2 has x = 3 and lies strictly in the trim,
    // on the side of the seam where the pcurve wraps.
    let meridian = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.5),
            Point3::new(2.0, 0.0, 1.0),
        ],
        vec![1.0; 3],
    )
    .unwrap();
    let turn = Interval::new(0.25, 0.25 + TAU).unwrap();
    let surface =
        NurbsSurface::revolution(&meridian, Point3::origin(), Vec3::z_axis(), turn).unwrap();
    assert_eq!(surface.period(), [None, None]);
    assert_eq!(surface.closure(), [Some(TAU), None]);
    for seam in [turn.lo(), turn.hi()] {
        let (mut m, face) = build(&Case {
            surface: surface.clone(),
            trim: Some((Point2::new(seam, 0.5), 0.4)),
        });
        let shell = m
            .raw()
            .add_shell(Shell::new(vec![FaceHandle::forward(face)]));
        let body = Body::forward(m.raw().add_body(BodyEntity::new(
            BodyKind::Sheet,
            vec![ShellHandle::forward(shell)],
            vec![],
            vec![],
        )));
        let report = arris_check::check(&m, body, Level::Full);
        assert!(report.is_ok(), "{report:?}");
        let got = crate::query::tight_bounds(&m, body.id.into(), &Control::NONE)
            .unwrap()
            .unwrap();
        let eps = m.precision().default_tolerance;
        assert!(
            got.max[0] >= 3.0 - SLACK && got.max[0] <= 3.0 + eps,
            "{got:?}"
        );
    }
}

/// How far past the refined extreme the boundary box the test hands in
/// is set, so that it holds the boundary's true extreme too.
// tol-literal: the margin of a reference computed to rounding, not a model tolerance
const BOUNDARY_PAD: f64 = 1e-9;

/// What the refined samples may miss the true extremes by, and the box may
/// miss a sample by in rounding.
// tol-literal: rounding of a refined sample at the test's scale
const SLACK: f64 = 1e-9;

/// A clamped cubic knot vector over `spans` unit spans.
fn cubic_knots(spans: usize) -> Vec<f64> {
    let mut k = vec![0.0; 4];
    k.extend((1..spans).map(|i| i as f64));
    k.extend([spans as f64; 4]);
    k
}

/// A bicubic patch and, maybe, a circle in its (u, v) that trims it.
#[derive(Debug, Clone)]
struct Case {
    surface: NurbsSurface,
    trim: Option<(Point2, f64)>,
}

fn case() -> impl Strategy<Value = Case> {
    (1..=2usize, 1..=2usize, any::<bool>())
        .prop_flat_map(|(su, sv, rational)| {
            let count = (su + 3) * (sv + 3);
            let weight = if rational {
                finite_f64(0.5..=2.0).boxed()
            } else {
                Just(1.0).boxed()
            };
            (
                Just((su, sv)),
                proptest::collection::vec(finite_f64(-3.0..=3.0), count),
                proptest::collection::vec((finite_f64(-0.8..=0.8), finite_f64(-0.8..=0.8)), count),
                proptest::collection::vec(weight, count),
                frame(),
                proptest::option::of((
                    finite_f64(0.35..=0.65),
                    finite_f64(0.35..=0.65),
                    finite_f64(0.1..=0.3),
                )),
            )
        })
        .prop_map(|((su, sv), heights, jitter, weights, place, trim)| {
            let m = sv + 3;
            let points = (0..(su + 3) * m)
                .map(|k| {
                    let (i, j) = (k / m, k % m);
                    let local = Point3::new(
                        3.0 * i as f64 + jitter[k].0,
                        3.0 * j as f64 + jitter[k].1,
                        heights[k],
                    );
                    place.to_world(local)
                })
                .collect();
            let surface =
                NurbsSurface::new([3, 3], [cubic_knots(su), cubic_knots(sv)], points, weights)
                    .expect("a clamped bicubic net");
            let trim = trim.map(|(cu, cv, rf)| {
                (
                    Point2::new(cu * su as f64, cv * sv as f64),
                    rf * su.min(sv) as f64,
                )
            });
            Case { surface, trim }
        })
}

/// The face of `case` in a model of its own.
fn build(case: &Case) -> (Model, FaceId) {
    let mut m = Model::default();
    let s = case.surface.clone();
    let [du, dv] = s.domain();
    let Some((c, r)) = case.trim else {
        let body = sample::patch(&mut m, Surface::Nurbs(s), du, dv).expect("a whole patch");
        let face = m.faces(body).expect("its face")[0].id;
        return (m, face);
    };
    let tol = m.precision().default_tolerance;
    let lift = |t: f64| {
        let (sin, cos) = real::sin_cos(t);
        let p = c + r * Vec2::new(cos, sin);
        s.eval(p.x, p.y).point
    };
    let curve = fit_curve_periodic(
        lift,
        Interval::TURN,
        3,
        |t, q| (q - lift(t)).norm(),
        tol,
        &mut Meter::default(),
    )
    .expect("the circle's image fits");
    let surface = m.add_surface(Surface::Nurbs(s.clone()));
    let curve = m.add_curve(Curve::Nurbs(curve));
    let pcurve = m.add_curve2(Curve2::Circle {
        frame: Frame2::new(c, Vec2::x(), Handedness::Right).expect("a frame"),
        radius: r,
    });
    let vertex = m.raw().add_vertex(Vertex::new(lift(0.0), tol));
    let edge = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve,
            range: Interval::TURN,
        },
        vertex,
        vertex,
        tol,
    ));
    let coedge = Coedge::new(edge, Orientation::Forward, pcurve);
    let face = m
        .raw()
        .add_face(Face::new(surface, vec![Loop::new(vec![coedge])], tol));
    (m, face)
}

/// The face's region: the whole domain, or the trimming disc.
fn inside(case: &Case, p: Point2) -> bool {
    let [du, dv] = case.surface.domain();
    match case.trim {
        None => du.contains(p.x) && dv.contains(p.y),
        Some((c, r)) => (p - c).norm() <= r,
    }
}

/// The boundary at `t ∈ [0, 1]`: the circle, or the four sides in turn.
fn boundary_at(case: &Case, t: f64) -> Point2 {
    let [du, dv] = case.surface.domain();
    match case.trim {
        Some((c, r)) => {
            let (sin, cos) = real::sin_cos(TAU * t);
            c + r * Vec2::new(cos, sin)
        }
        None => {
            let s = 4.0 * t.clamp(0.0, 1.0);
            let side = (s as usize).min(3);
            let f = s - side as f64;
            match side {
                0 => Point2::new(du.lerp(f), dv.lo()),
                1 => Point2::new(du.hi(), dv.lerp(f)),
                2 => Point2::new(du.lerp(1.0 - f), dv.hi()),
                _ => Point2::new(du.lo(), dv.lerp(1.0 - f)),
            }
        }
    }
}

/// `sign · S_k` at `uv`.
fn height(case: &Case, k: usize, sign: f64, uv: Point2) -> f64 {
    sign * case.surface.eval(uv.x, uv.y).point[k]
}

/// The largest `sign · S_k` along the boundary: the best of a dense
/// sample, refined by golden section around each of the three best.
fn boundary_extreme(case: &Case, k: usize, sign: f64) -> f64 {
    const N: usize = 800;
    let f = |t: f64| height(case, k, sign, boundary_at(case, t));
    let mut samples: Vec<(f64, f64)> = (0..=N)
        .map(|i| {
            let t = i as f64 / N as f64;
            (f(t), t)
        })
        .collect();
    samples.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut best = samples[0].0;
    let gr = 0.5 * (5.0_f64.sqrt() - 1.0);
    for &(_, t) in samples.iter().take(3) {
        let h = 1.0 / N as f64;
        let (mut a, mut b) = (t - h, t + h);
        if case.trim.is_none() {
            (a, b) = (a.max(0.0), b.min(1.0));
        }
        for _ in 0..80 {
            let (c, d) = (b - gr * (b - a), a + gr * (b - a));
            if f(c) > f(d) {
                b = d;
            } else {
                a = c;
            }
        }
        best = best.max(f(0.5 * (a + b)));
    }
    best
}

/// The largest `sign · S_k` over the face: the best of a dense grid inside
/// it and its boundary's extreme, the grid's three best each climbed by
/// Newton's iteration on the gradient (steepest ascent where the Hessian
/// does not point up), every step held inside the face.
fn face_extreme(case: &Case, k: usize, sign: f64) -> f64 {
    const N: usize = 64;
    let [du, dv] = case.surface.domain();
    let mut grid: Vec<(f64, Point2)> = Vec::new();
    for i in 0..=N {
        for j in 0..=N {
            let p = Point2::new(du.lerp(i as f64 / N as f64), dv.lerp(j as f64 / N as f64));
            if inside(case, p) {
                grid.push((height(case, k, sign, p), p));
            }
        }
    }
    grid.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut best = boundary_extreme(case, k, sign);
    for &(value, start) in grid.iter().take(3) {
        let (mut p, mut f) = (start, value);
        for _ in 0..200 {
            let e = case.surface.eval(p.x, p.y);
            let g = Vec2::new(sign * e.du[k], sign * e.dv[k]);
            let (a, b, c) = (sign * e.duu[k], sign * e.duv[k], sign * e.dvv[k]);
            let det = a * c - b * b;
            // Newton towards a maximum where the Hessian is negative
            // definite, else up the gradient.
            let mut step = if a < 0.0 && det > 0.0 {
                -Vec2::new(c * g.x - b * g.y, a * g.y - b * g.x) / det
            } else {
                g * (1.0 / (1.0 + g.norm()))
            };
            let mut moved = false;
            for _ in 0..60 {
                let q = p + step;
                if inside(case, q) {
                    let fq = height(case, k, sign, q);
                    if fq > f {
                        (p, f, moved) = (q, fq, true);
                        break;
                    }
                }
                step *= 0.5;
            }
            if !moved {
                break;
            }
        }
        best = best.max(f);
    }
    best
}

/// The box `nurbs_face_box` gives for `case`, with a boundary box from
/// its refined boundary extremes, padded out.
fn run(case: &Case) -> (Aabb, f64) {
    let (m, face) = build(case);
    let loops = m
        .face(face)
        .unwrap()
        .loops()
        .iter()
        .map(|l| m.loop_pieces(l).unwrap())
        .collect::<Vec<_>>();
    let domain = TrimDomain::of(
        &loops,
        case.surface.closure(),
        m.precision().parametric_tolerance,
        &mut Meter::default(),
    )
    .unwrap()
    .expect("a certified trim");
    let mut boundary = Aabb {
        min: [0.0; 3],
        max: [0.0; 3],
    };
    for k in 0..3 {
        boundary.min[k] = -boundary_extreme(case, k, -1.0) - BOUNDARY_PAD;
        boundary.max[k] = boundary_extreme(case, k, 1.0) + BOUNDARY_PAD;
    }
    let eps = m.precision().default_tolerance;
    let got = nurbs_face_box(
        face,
        &case.surface,
        &domain,
        boundary,
        eps,
        &mut Meter::default(),
    )
    .expect("the face has a tight box");
    (got, eps)
}

#[test]
fn a_nurbs_face_can_reach_its_minimum_at_a_c0_knot() {
    // z = |u - 1/2| + (v - 3/8)^2. Both patches rise away from the
    // crease, whose low point is between the first subdivision samples.
    let a = 3.0 / 8.0;
    let points = (0..3)
        .flat_map(|i| {
            let u = i as f64 / 2.0;
            [a * a, a * a - a, (1.0 - a) * (1.0 - a)]
                .into_iter()
                .enumerate()
                .map(move |(j, z)| Point3::new(u, j as f64 / 2.0, (u - 0.5).abs() + z))
        })
        .collect();
    let surface = NurbsSurface::new(
        [1, 2],
        [
            vec![0.0, 0.0, 0.5, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        ],
        points,
        vec![1.0; 9],
    )
    .unwrap();
    let (got, eps) = run(&Case {
        surface,
        trim: None,
    });
    assert!((-eps..=SLACK).contains(&got.min[2]), "{got:?}");
}

fn holds(case: &Case) -> Result<(), TestCaseError> {
    let (got, eps) = run(case);
    let [du, dv] = case.surface.domain();
    // Outer: every sample of the face is inside.
    for i in 0..=40 {
        for j in 0..=40 {
            let p = Point2::new(du.lerp(i as f64 / 40.0), dv.lerp(j as f64 / 40.0));
            if !inside(case, p) {
                continue;
            }
            let q = case.surface.eval(p.x, p.y).point;
            for k in 0..3 {
                prop_assert!(
                    got.min[k] - SLACK <= q[k] && q[k] <= got.max[k] + SLACK,
                    "sample {q:?} at {p:?} outside {got:?}"
                );
            }
        }
    }
    // Outer and tight: each side within `eps` of the refined extreme.
    for k in 0..3 {
        let hi = face_extreme(case, k, 1.0);
        let lo = -face_extreme(case, k, -1.0);
        prop_assert!(
            got.max[k] >= hi - SLACK && got.max[k] <= hi + eps + BOUNDARY_PAD + SLACK,
            "axis {k}: high side {} against the face's {hi}",
            got.max[k]
        );
        prop_assert!(
            got.min[k] <= lo + SLACK && got.min[k] >= lo - eps - BOUNDARY_PAD - SLACK,
            "axis {k}: low side {} against the face's {lo}",
            got.min[k]
        );
    }
    Ok(())
}

#[test]
fn a_trimmed_bicubic_box_is_outer_and_within_eps_of_the_face() {
    check(case(), |case| holds(&case));
}

/// A tent whose ridge is a C0 knot line at the root's own midpoint, with
/// its highest point inside the face and off every point the first
/// patches evaluate: both halves rise towards the ridge, and only the
/// crease guard keeps them.
#[test]
fn a_ridge_on_a_c0_knot_line_is_found() {
    let a = [0.0, 1.0, 2.0, 3.0, 2.0, 1.0, 0.0];
    let b = [0.2, 1.0, 0.4, 0.1];
    let mut points = Vec::new();
    for (i, ai) in a.iter().enumerate() {
        for (j, bj) in b.iter().enumerate() {
            points.push(Point3::new(i as f64, j as f64, ai * bj));
        }
    }
    let surface = NurbsSurface::new(
        [3, 3],
        [
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0],
            cubic_knots(1),
        ],
        points,
        vec![1.0; 28],
    )
    .expect("a net with a C0 knot line");
    let case = Case {
        surface,
        trim: None,
    };
    let (got, eps) = run(&case);
    // `S_z(1, v) = 3 B(v)`, `B` the cubic over `b`: its top, refined.
    let ridge = |v: f64| case.surface.eval(1.0, v).point.z;
    let (mut lo, mut hi) = (0.0, 1.0);
    let gr = 0.5 * (5.0_f64.sqrt() - 1.0);
    for _ in 0..100 {
        let (c, d) = (hi - gr * (hi - lo), lo + gr * (hi - lo));
        if ridge(c) > ridge(d) {
            hi = d;
        } else {
            lo = c;
        }
    }
    let peak = ridge(0.5 * (lo + hi));
    assert!(peak > ridge(0.5) + 1e3 * eps, "the peak is off the centre");
    assert!(
        got.max[2] >= peak - SLACK && got.max[2] <= peak + eps,
        "{got:?} against {peak}"
    );
}

/// A net on a tilted plane: every coordinate is monotone along both
/// directions, so every patch is set aside at once and the box is the
/// boundary's, the corners'.
#[test]
fn a_monotone_patch_is_its_boundary_box() {
    let mut points = Vec::new();
    for i in 0..4 {
        for j in 0..4 {
            let (x, y) = (i as f64, j as f64);
            points.push(Point3::new(x, y, 0.5 * x - 0.25 * y));
        }
    }
    let surface = NurbsSurface::new(
        [3, 3],
        [cubic_knots(1), cubic_knots(1)],
        points,
        vec![1.0; 16],
    )
    .expect("a planar net");
    let (got, _) = run(&Case {
        surface,
        trim: None,
    });
    let (lo, hi) = ([0.0, 0.0, -0.75], [3.0, 3.0, 1.5]);
    for k in 0..3 {
        assert!(
            (got.min[k] - lo[k]).abs() <= BOUNDARY_PAD + SLACK,
            "{got:?}"
        );
        assert!(
            (got.max[k] - hi[k]).abs() <= BOUNDARY_PAD + SLACK,
            "{got:?}"
        );
    }
}
